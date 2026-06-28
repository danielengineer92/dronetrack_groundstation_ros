# Mission API Route Audit

**File audited:** `web_dashboard_node.py` (864 lines)
**Date:** 2026-06-28
**Method:** Read-only; no edits made.

All four mission planner routes are **present** in the file. Below are their exact
URL paths, expected request/response shapes, and error responses.

---

## 1. GET /api/mission-plans

**HTTP method:** GET
**Line:** 604–605 (`do_GET`)
**Handler:** `WebDashboardNode._list_mission_plans()` (lines 711–743)

### Request
No body. No query parameters used.

### Success response (200)
```json
{
  "plans": [
    {
      "name": "<plan name from YAML or stem>",
      "filename": "<basename>.yaml",
      "modified": <float epoch timestamp>,
      "step_count": <int>,
      "valid": <bool>,
      "is_template": <bool>
    }
  ]
}
```

Sources:
- Saved plans: read from `~/drone_mission_plans/` (configurable via
  `mission_plans_dir` ROS param). Each `*.yaml` is parsed to extract
  `mission.name` and `len(mission.steps)`. If parsing fails, `valid` is `false`.
- Templates: read from the package-relative
  `drone_control/missions/` directory. Each `*.yaml` is listed with
  `is_template: true`, `valid: true`, `step_count: 0`.

`plans` is the concatenation of saved plans + templates.

### Error responses
None defined for this route. An empty `{"plans": []}` is returned if no plans
or templates exist.

---

## 2. POST /api/mission-plans/save

**HTTP method:** POST
**Lines:** 665–673 (`do_POST`)
**Handler:** `WebDashboardNode._save_mission_plan(payload)` (lines 791–810)

### Request body
```json
{
  "name": "<string, default 'untitled'>",
  "steps": [
    {
      "type": "<verb from step schema>",
      "params": {
        "<param_key>": <value>
      }
    }
  ],
  "overwrite": <bool, default false>
}
```

Each step is validated by `validate_step()` from `mission_plan_model.py`
(lines 186–228). Validation checks:
- `"type"` must be one of the 9 known verbs.
- `"params"` must be a dict containing all required keys for that verb.
- Float params: must be `int` or `float`, finite, within `min`/`max` bounds.
- Enum/str params: must be a string; enums must match allowed `options`.

The filename is derived from `sanitize_filename(name) + ".yaml"` (lowercase,
underscores, strip special chars; default `"untitled_mission"`).

### Success response (200)
```json
{
  "ok": true,
  "filename": "<sanitized_name>.yaml",
  "warnings": ["<lint warning string>"]
}
```

Warnings come from `lint_steps()` (lines 257–305): empty plan, motion step
before `prime_offboard`, missing timeouts, scan without `until` or `timeout_s`.

### Conflict response (409)
File already exists on disk and `overwrite` was `false` or missing:
```json
{
  "ok": false,
  "conflict": true,
  "filename": "<sanitized_name>.yaml"
}
```

### Validation error response (400)
One or more steps fail `validate_step()`:
```json
{
  "ok": false,
  "errors": ["<error string>"]
}
```

### Internal error (500)
Any unhandled exception in the handler (line 683–685).

---

## 3. POST /api/mission-plans/send

**HTTP method:** POST
**Lines:** 674–679 (`do_POST`)
**Handler:** `WebDashboardNode._send_mission_plan(payload)` (lines 828–841)

### Request body
```json
{
  "name": "<string, default 'untitled'>",
  "steps": [ <same shape as save> ]
}
```

Same step validation as `/api/mission-plans/save`. **No `overwrite` field**
is used — this route does not write to disk.

On success, the plan is serialized to YAML via `steps_to_yaml()` (lines 231–254)
and published as a `std_msgs/String` on the topic configured by the
`mission_plan_topic` ROS parameter (default: `"/drone/mission/plan"`).

### Success response (200)
```json
{
  "ok": true,
  "yaml": "<serialized YAML string>",
  "warnings": ["<lint warning string>"]
}
```

### Validation error response (400)
```json
{
  "ok": false,
  "errors": ["<error string>"]
}
```

### Internal error (500)
Any unhandled exception.

---

## 4. GET /api/mission-step-schema

**HTTP method:** GET
**Lines:** 602–603 (`do_GET`)
**Handler:** `get_step_schema()` from `mission_plan_model.py` (line 170–172)

### Request
No body. No query parameters.

### Success response (200)
```json
{
  "verbs": {
    "<verb_name>": {
      "label": "<human-readable label>",
      "description": "<description string>",
      "params": {
        "<param_key>": {
          "type": "float" | "enum" | "str",
          "default": <default value>,
          "label": "<human-readable label>",
          "min": <number, floats only>,
          "max": <number, floats only>,
          "step": <number, floats only>,
          "options": ["<string>", ...]  // enums only
        }
      },
      "category": "preflight" | "action" | "motion"
    }
  }
}
```

### Defined verbs (9 total)
| Verb | Category |
|------|----------|
| `takeoff` | action |
| `prime_offboard` | preflight |
| `scan` | motion |
| `track_center` | motion |
| `approach` | motion |
| `orbit` | motion |
| `rtl` | action |
| `land` | action |
| `hold` | preflight |

### Error responses
None defined. Always returns 200.

---

## Additional related routes (not part of the audit scope)

For completeness, the file also contains these mission-adjacent routes:

| Method | Path | Line | Purpose |
|--------|------|------|---------|
| GET | `/api/mission-plans/<filename>` | 606–616 | Load a single plan by filename |
| DELETE | `/api/mission-plans/<filename>` | 620–633 | Delete a saved plan |

---

## Summary

All **four routes are present** and implemented. The docstring at the top of the
file (lines 14–28) accurately documents all four endpoints with correct paths.
No discrepancies were found between the docstring and the implementation.

**Confirmed URL paths:**
- `GET /api/mission-plans`
- `POST /api/mission-plans/save`
- `POST /api/mission-plans/send`
- `GET /api/mission-step-schema`
