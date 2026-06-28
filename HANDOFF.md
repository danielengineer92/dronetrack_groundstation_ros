# Audit Handoff — Mission Planner Changes
**Date:** 2026-06-28  
**Scope:** Commits `a6ea9ab`, `b7b955d`, `feb9442` (Interactive Mission Planner feature)  
**Audited files:** `web_dashboard_node.py`, `mission_plan_model.py`, `mission_executor_node.py`, `mission_preview.py`, `mission_preview_ext.py`  
**Tests run:** `test_mission_plan.py` (18/18 pass), `test_mission_preview.py` (2/2 pass)

---

## 1. Security Review

### 1.1 `_safe_filename` — Path Traversal Fix (PRIMARY CONCERN)

**Location:** `web_dashboard_node.py:699–705`

```python
@staticmethod
def _safe_filename(filename: str) -> bool:
    if not filename or ".." in filename:
        return False
    if filename.startswith("/") or "\\" in filename:
        return False
    return True
```

**Verdict: Correct for its stated purpose.** Blocks all primary traversal vectors:

| Input | Result | Reason |
|---|---|---|
| `../etc/passwd` | BLOCKED | contains `..` |
| `/etc/passwd` | BLOCKED | starts with `/` |
| `foo\bar` | BLOCKED | contains `\` |
| `%2e%2e/etc/passwd` | allowed by first check… | no literal `..` or `/` yet |

**Defense-in-depth for URL-encoded traversal (`%2e%2e/`):**  
`_safe_filename` is called **twice** per request — once on the raw URL before `unquote()`, then again inside `_load_mission_plan` and `_delete_mission_plan` after decoding. A URL-encoded `%2e%2e` passes the first check but fails the second after `unquote()` produces literal `..`. This layered approach is correct.

Both `_load_mission_plan` and `_delete_mission_plan` additionally call `fp.resolve()` and check the resolved absolute path stays within the intended base directory, providing a third layer against symlink attacks.

### 1.2 Minor Security Findings

**`startswith` prefix collision in resolve() boundary checks (Low)**  
`_load_mission_plan:760` and `_delete_mission_plan:813` use:
```python
str(resolved).startswith(str(base))
```
If `plans_dir` resolves to `/home/user/drone_mission_plans`, a symlink inside it pointing to `/home/user/drone_mission_plans_evil/file` would incorrectly pass the boundary check (prefix match). The fix is to use `resolved.is_relative_to(base)` (Python 3.9+) or append `os.sep` to the base string:
```python
str(resolved).startswith(str(base) + os.sep) or str(resolved) == str(base)
```
Risk is low because `_safe_filename` already blocks `..` and `/`, so this is only reachable via symlinks planted inside the plans directory.

**`do_DELETE` does not strip query params (Low)**  
`do_GET` strips query strings with `path = self.path.split("?", 1)[0]` before extracting the filename. `do_DELETE` does not. A request like `DELETE /api/mission-plans/foo.yaml?x=y` extracts `foo.yaml?x=y` as the filename. If `x=y` contains `..`, `_safe_filename` catches it. If not, the file lookup returns 404 (file with `?` in name doesn't exist). Not exploitable, but inconsistent. Fix: apply the same `split("?", 1)[0]` strip in `do_DELETE`.

**Unbounded POST body (Low)**  
`do_POST` reads `int(headers.get("Content-Length", 0))` bytes with no upper cap. A client sending a very large plan payload could exhaust memory. For a LAN-only tool this is acceptable, but consider capping at 64 KB:
```python
MAX_BODY = 65_536
length = min(int(self.headers.get("Content-Length", 0) or 0), MAX_BODY)
```

**Exception messages in 500 responses (Informational)**  
`str(exc)` is returned in HTTP 500 responses, which may expose internal file paths. Acceptable for a local operator tool.

**CORS `*` (Informational)**  
All endpoints send `Access-Control-Allow-Origin: *`. Intentional for LAN use.

---

## 2. Architecture & Interface Consistency

### 2.1 Schema Duplication — Dashboard vs. Pi

The mission step schema exists in two places:

| Layer | File | Purpose |
|---|---|---|
| Dashboard | `mission_plan_model.py` (`STEP_SCHEMA`) | UI validation, serialization, type/range checks |
| Pi | `mission_plan.py` (`VALID_STEP_TYPES`, `_parse_step`) | Runtime validation before execution |

**Currently in sync** on all 9 verbs and their parameters. Minor intentional differences:

- Dashboard `scan.yaw_rate_deg_s`: min=1.0, `scan.yaw_deg`: min=5.0  
  Pi accepts any positive float. **Dashboard-stricter is correct** — no plans that pass the dashboard can fail Pi validation on these fields.

- **"complete" verb gap**: `VALID_STEP_TYPES` in `mission_plan.py` and `mission_preview.py` include `"complete"`, but `STEP_SCHEMA` in `mission_plan_model.py` does **not**. Users cannot create a `complete` step from the dashboard UI, though it is valid in hand-authored YAML. This is intentional (the step is a no-op sentinel) but should be documented or explicitly excluded by comment.

**Risk:** The two schema definitions will drift silently as new verbs are added. There is no automated consistency check. Consider adding a test that compares `STEP_SCHEMA.keys()` against `VALID_STEP_TYPES` from `mission_plan.py` (minus `complete`).

### 2.2 Topic Naming

`WebDashboardNode` (laptop) publishes plans to `mission_plan_topic` (default `/drone/mission/plan`).  
`MissionExecutorNode` (Pi) subscribes to the same parameter default. **Consistent.**

### 2.3 Plan Reception on the Pi — Safety Architecture

`_on_plan_received` in `mission_executor_node.py:410` correctly:
1. **Rejects during active mission** — plan updates cannot interrupt a flight in progress.
2. Re-validates via `yaml.safe_load` + `parse_mission_plan` (not trusting dashboard serialization).
3. Logs rejected plans with reason.
4. Resets `step_index = 0` so the next mission start uses the new plan.

This satisfies the safety contract: the dashboard is advisory; the Pi always re-validates.

### 2.4 API Endpoints vs. Dashboard HTML

The 6 CRUD endpoints are fully implemented in the Python backend:

| Endpoint | Status |
|---|---|
| `GET /api/mission-plans` | Complete |
| `GET /api/mission-plans/<file>` | Complete |
| `POST /api/mission-plans/save` | Complete |
| `POST /api/mission-plans/send` | Complete |
| `DELETE /api/mission-plans/<file>` | Complete |
| `GET /api/mission-step-schema` | Complete |

**The embedded dashboard HTML (`DASHBOARD_HTML`) does not wire these endpoints to any UI controls.** The JavaScript only calls `/api/missions` (read-only catalog preview) and the four operator-action POSTs. The frontend editing/creating/sending UI is pending work — the API layer is ready to receive it.

### 2.5 `mission_preview_ext.py` Redundancy

This module is a thin two-function pass-through to `mission_plan_model`. It exists to give `web_dashboard_node.py` a clean import path (`from ...mission_preview_ext import plan_from_steps`). Currently only `plan_from_steps` is used externally (in `web_dashboard_node.py` import). The module is harmless; consider collapsing it into `mission_plan_model.py` if it doesn't grow.

### 2.6 `_list_mission_plans` vs. `mission_catalog_snapshot` Data Shapes

| Endpoint | Source | Key | Shape |
|---|---|---|---|
| `/api/missions` | `mission_catalog_snapshot()` → `load_mission_catalog()` | `"missions"` | `[{name, filename, valid, steps, warnings, path, pi_param_hint}]` |
| `/api/mission-plans` | `_list_mission_plans()` | `"plans"` | `[{name, filename, step_count, valid, modified, is_template}]` |

Different shapes for different use cases — correct by design. The legacy `/api/missions` catalog is rich (includes steps/warnings for preview); the new `/api/mission-plans` list is a lightweight index (for a Load dropdown). No confusion risk as long as the JS frontend keeps them separate.

---

## 3. Test Coverage

### What Passes
- `test_mission_plan.py` — 18/18: default plans, YAML parsing, scan validation, lint warnings, error paths
- `test_mission_preview.py` — 2/2: file preview, catalog loading

### Gaps (Recommended Follow-On Tests)

**`mission_plan_model.py` has no tests** despite being the dashboard's primary validation layer:

| Missing test | Target | Priority |
|---|---|---|
| `validate_step` — valid/invalid/missing params | `mission_plan_model.validate_step` | High |
| `sanitize_filename` — special chars, empty, long names | `mission_plan_model.sanitize_filename` | Medium |
| `steps_to_yaml` — round-trips through YAML parse | `mission_plan_model.steps_to_yaml` | Medium |
| `lint_steps` — motion-before-prime, missing timeouts | `mission_plan_model.lint_steps` | Medium |
| `_safe_filename` — traversal, encoded, empty | `web_dashboard_node._safe_filename` | High |
| HTTP routing — CRUD endpoint dispatch, 404 paths | Handler in `_start_server` | Low (needs mock node) |

`tests/test_mission_states.py` is a hardware integration test (requires live SITL + armed PX4). Not runnable offline. No gap — just confirm it's not expected to pass in CI.

---

## 4. Summary of Actionable Items

Priority order:

1. **[Low/security]** Fix `do_DELETE` to strip query params before filename extraction — one-liner matching `do_GET`.
2. **[Low/security]** Replace `str(resolved).startswith(str(base))` with `resolved.is_relative_to(base)` (or `startswith(str(base) + os.sep)`) in `_load_mission_plan` and `_delete_mission_plan`.
3. **[Medium/robustness]** Add a POST body size cap (e.g., 64 KB) in `do_POST`.
4. **[Medium/correctness]** Add `mission_plan_model.py` unit tests (`validate_step`, `sanitize_filename`, `steps_to_yaml`, `lint_steps`).
5. **[Low/consistency]** Add a cross-schema consistency test: assert `set(STEP_SCHEMA) == set(VALID_STEP_TYPES) - {"complete"}`.
6. **[Future work]** Wire the 6 CRUD API endpoints to a UI in `DASHBOARD_HTML` (the backend is ready).
7. **[Informational]** Document or add a comment explaining why `"complete"` is absent from `STEP_SCHEMA` but present in the Pi-side model.

---

## 5. What Is Complete and Correct

- `_safe_filename` correctly blocks all primary and URL-encoded path traversal attacks via defense-in-depth (pre-decode check, post-decode check, resolve() boundary check).
- `_save_mission_plan` is safe: uses `sanitize_filename` which reduces output to `[a-z0-9_]+.yaml` before path construction.
- `_on_plan_received` (Pi side) re-validates all dashboard-supplied YAML and rejects it during an active mission.
- Both lint implementations (`mission_plan_model.lint_steps`, `mission_plan.lint_plan`) are functionally equivalent and agree on warnings.
- Thread safety in `WebDashboardNode` is correct: all shared state accesses are protected by `self._lock`.
- Topic names are consistent across the dashboard↔Pi boundary.
- All existing tests pass.
