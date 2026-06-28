# Bug-hunt handoff — DroneTrack Mission Planner

## What was built

Interactive mission planner for the ROS 2 / PX4 drone groundstation. 4 commits across 9 files:

| Commit | What |
|---|---|
| `a6ea9ab` | Foundation — `mission_plan_model.py`, `mission_preview_ext.py`, executor plan receiver, configs |
| `b7b955d` | 6 REST endpoints, plan publisher on `/drone/mission/plan` |
| `feb9442` | CORS, `_safe_filename` path traversal guard, step validation on save |
| `c31546a` | Audit + hardening — 180-line HANDOFF, schema consistency fix, boundary checks |
| `851de56` | Interactive builder HTML/JS in `DASHBOARD_HTML` — 2652 lines |

## Key files to audit

| File | Lines | What to check |
|---|---|---|
| `ros_ws/src/dronetrack_web_bridge/dronetrack_web_bridge/web_dashboard_node.py` | ~920 | `_list_mission_plans`, `_load_mission_plan`, `_save_mission_plan`, `_send_mission_plan`, `_safe_filename`, `do_DELETE` query-param strip, max body size, the DASHBOARD_HTML JS: `renderSteps()`, `stepSummary()`, `initPlanner()`, `autosave()`, `savePlan()` conflict overwrite, `clearPlan()`, `sendPlan()` |
| `ros_ws/src/dronetrack_web_bridge/dronetrack_web_bridge/mission_plan_model.py` | ~320 | `validate_step()`, `steps_to_yaml()`, `lint_steps()` — edge cases with missing params, step ordering, serialisation round-trip |
| `ros_ws/src/drone_control/drone_control/mission_executor_node.py` | ~1010 | `_on_plan_received()` — state guard, YAML parsing, plan replacement, error logging |
| `ros_ws/src/dronetrack_web_bridge/dronetrack_web_bridge/mission_preview.py` | ~370 | Backward compat — `load_mission_catalog()` still works |
| `configs/groundstation.yaml` | ~53 | New `mission_plans_dir`, `mission_plan_topic` params |
| `configs/pi.yaml` | ~239 | New `mission_plan_topic` param |
| `configs/topics.yaml` | ~87 | New `/drone/mission/plan` topic entry |

## Known issues from prior audit (`c31546a`)

1. **`do_DELETE` doesn't strip query params** — inconsistent with `do_GET`. Add `path = self.path.split("?", 1)[0]` before extracting filename.
2. **`startswith` boundary collision** — `_load_mission_plan` and `_delete_mission_plan` use `str(resolved).startswith(str(base))` which could match `/home/user/drone_mission_plans_evil/`. Change to `resolved.is_relative_to(base)` or append `os.sep` to base.
3. **No max POST body** — `do_POST` reads unbounded body. Cap at 64 KB.
4. **Schema duplication** — `STEP_SCHEMA` in `mission_plan_model.py` and `VALID_STEP_TYPES` in `mission_plan.py` will drift. Add a test comparing them.
5. **"complete" verb missing from `STEP_SCHEMA`** — intentionally excluded (no-op sentinel), but undocumented. Add comment.

## What needs QA testing

### 1. Backend — run and hit endpoints

```bash
cd ros_ws
source install/setup.bash
# Only the web bridge node needed (no drone hardware):
ros2 run dronetrack_web_bridge web_dashboard_node
```

Then test:
```bash
# Schema
curl http://127.0.0.1:8080/api/mission-step-schema

# Save a plan
curl -X POST http://127.0.0.1:8080/api/mission-plans/save \
  -H 'Content-Type: application/json' \
  -d '{"name":"test mission","steps":[{"type":"takeoff","params":{"altitude_m":5}},{"type":"land"}]}'

# List
curl http://127.0.0.1:8080/api/mission-plans

# Load
curl http://127.0.0.1:8080/api/mission-plans/test_mission.yaml

# Send (won't publish without ROS running, but should validate)
curl -X POST http://127.0.0.1:8080/api/mission-plans/send \
  -H 'Content-Type: application/json' \
  -d '{"name":"test","steps":[{"type":"takeoff"},{"type":"land"}]}'

# Send invalid
curl -X POST http://127.0.0.1:8080/api/mission-plans/send \
  -H 'Content-Type: application/json' \
  -d '{"name":"bad","steps":[{"type":"scan","params":{"yaw_deg":-999}}]}'

# Path traversal
curl http://127.0.0.1:8080/api/mission-plans/../etc/passwd
curl http://127.0.0.1:8080/api/mission-plans/..%2f..%2fetc%2fpasswd

# DELETE
curl -X DELETE http://127.0.0.1:8080/api/mission-plans/test_mission.yaml

# CORS preflight
curl -X OPTIONS http://127.0.0.1:8080/api/mission-plans
```

### 2. Frontend — open browser

Open `http://127.0.0.1:8080/`:

- [ ] Mission Planner section shows with plan name input + toolbar
- [ ] Add Step toolbar has all 9 verb buttons
- [ ] Clicking verb adds a step with defaults
- [ ] Step cards show colored badges (green=action, blue=preflight, orange=motion)
- [ ] Clicking a step card opens parameter editor below it
- [ ] Changing params updates the step summary text
- [ ] Move up/down/delete buttons work
- [ ] Save works — file appears in `~/drone_mission_plans/`
- [ ] Reload page — plan auto-restored from localStorage
- [ ] Load dropdown shows saved plans + templates
- [ ] Clear wipes everything
- [ ] Status bar shows step count + validity
- [ ] Camera stream and operator buttons still work (unchanged)

### 3. CLI tests

```bash
cd ros_ws
python -m pytest src/dronetrack_web_bridge/test/test_mission_planner_logic.js 2>/dev/null || true
# Run Python unit tests if present:
python -c "
from dronetrack_web_bridge.mission_plan_model import *
v = get_step_schema()
assert len(v) == 9, f'Expected 9 verbs, got {len(v)}'
for verb in v:
    s = create_default_step(verb)
    errs = validate_step(s)
    assert not errs, f'{verb} default failed: {errs}'
yaml_str = steps_to_yaml('test', [create_default_step('takeoff'), create_default_step('land')])
assert 'mission:' in yaml_str
w = lint_steps([create_default_step('scan')])
assert any('prime' in x for x in w), 'scan without prime should warn'
print('All model checks passed')
"
```

## Bug-hunting checklist

- [ ] Empty plan send → should reject gracefully
- [ ] Step with missing `type` field → should not crash
- [ ] Step with null params → should not crash  
- [ ] `steps_to_yaml` on empty step list → should produce valid YAML
- [ ] `sanitize_filename` with all special chars → should produce safe string
- [ ] `sanitize_filename` with empty string → should return default
- [ ] Save with empty steps → should still save or reject cleanly
- [ ] Save with existing filename, no overwrite → should return 409
- [ ] Load non-existing file → should return 404
- [ ] Delete non-existing file → should return 404
- [ ] Send while mission active (executor) → should reject with warning
- [ ] Send YAML containing unicode → should serialize/publish correctly
- [ ] Two rapid sends → idempotent, last wins
- [ ] Parameters outside schema bounds → should fail validation
- [ ] `_safe_filename` against null/undefined → should return false
- [ ] `validate_step` on non-dict input → should return error list
- [ ] localStorage corrupted → should fall back to empty plan
- [ ] Load dropdown with no plans → should show empty
- [ ] Plan with 20+ steps → UI should scroll, not break layout
- [ ] Browser back/forward → should maintain plan state

## Build verification

```bash
python -c "import py_compile; py_compile.compile('ros_ws/src/dronetrack_web_bridge/dronetrack_web_bridge/mission_plan_model.py', doraise=True)"
python -c "import py_compile; py_compile.compile('ros_ws/src/dronetrack_web_bridge/dronetrack_web_bridge/web_dashboard_node.py', doraise=True)"
python -c "import py_compile; py_compile.compile('ros_ws/src/dronetrack_web_bridge/dronetrack_web_bridge/mission_preview_ext.py', doraise=True)"
python -c "import py_compile; py_compile.compile('ros_ws/src/drone_control/drone_control/mission_executor_node.py', doraise=True)"
```

## File paths

```
C:\Users\danie\Documents\Python\dronetrack_groundstation_ros\
├── configs/groundstation.yaml
├── configs/pi.yaml
├── configs/topics.yaml
├── ros_ws/src/dronetrack_web_bridge/dronetrack_web_bridge/
│   ├── web_dashboard_node.py         ← main HTML/JS + API endpoints
│   ├── mission_plan_model.py         ← step schema + validation
│   ├── mission_preview.py            ← mission catalog (existing)
│   └── mission_preview_ext.py        ← steps_to_yaml bridge
├── ros_ws/src/drone_control/drone_control/
│   └── mission_executor_node.py      ← plan receiver subscriber
└── HANDOFF.md                        ← prior audit findings
```
