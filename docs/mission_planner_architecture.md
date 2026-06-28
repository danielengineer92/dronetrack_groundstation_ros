# Mission Planner Architecture

The mission planner lets the operator build, save, load, and send mission plans
from the web dashboard to the mission executor — all within the trust boundary.

## Component Diagram

```
┌─ Dashboard Builder (JS) ─────┐     ┌─ web_dashboard_node.py ───┐     ┌─ mission_executor_node.py ──┐
│  plan = {name, steps}         │────▶│ POST /api/mission-plans/  │     │                              │
│  validatePlan()               │     │    /save → write to disk  │     │                              │
│  renderPlan()                 │     │    /send → String publish │────▶│ /drone/mission/plan         │
│  drag-and-drop reorder        │     │ steps_to_yaml()           │     │ _on_plan_received()         │
│  inline param editor          │     │ lint_steps()              │     │ parse_mission_plan()        │
│  localStorage autosave        │     │ validate_step()           │     │ lint_plan()                 │
└───────────────────────────────┘     └───────────────────────────┘     │ if valid: replace self.plan │
                                                                        │ if invalid: log + ignore    │
                                                                        │ if mission active: reject   │
                                                                        └─────────────────────────────┘
```

## Trust Boundary

The dashboard sends an **advisory** `std_msgs/String` payload on
`/drone/mission/plan`. The mission executor validates it with
`parse_mission_plan()` before accepting:

- If the payload is valid YAML and parses to a legal `MissionPlan`, the executor
  replaces `self.plan` and resets the step cursor to step 0.
- If parsing fails, the plan is **rejected and logged** — the current plan is
  unchanged.
- If a mission is **currently active** (state not IDLE/DISABLED/COMPLETE/ABORTED),
  the plan is rejected. Plans can only be changed while idle.
- Lint warnings are logged but do not block acceptance.

The laptop **never** publishes control, enable, or arming topics. This topic is
purely advisory, joining the existing set of operator request topics.

## Data Flows

### Save Plan (browser → disk)
```
Browser: POST /api/mission-plans/save {name, steps, overwrite}
         → validate_step() on each step
         → steps_to_yaml()
         → write to ~/drone_mission_plans/<name>.yaml
         → return {ok, filename, warnings}
```

### Load Plan (disk → browser)
```
Browser: GET /api/mission-plans/<filename>
          → scan ~/drone_mission_plans/ (user plans) +
            ros_ws/src/drone_control/missions/ (templates)
          → parse YAML, fill defaults from STEP_SCHEMA
          → return {name, steps, warnings}
```

### Send Plan (browser → executor)
```
Browser: POST /api/mission-plans/send {name, steps}
         → validate_step() on each step
         → steps_to_yaml()
         → publish std_msgs/String to /drone/mission/plan
         → return {ok, yaml, warnings}

Executor: String arrives on /drone/mission/plan
          → _on_plan_received()
          → if mission_active: reject
          → yaml.safe_load() → parse_mission_plan()
          → if error: log + reject
          → lint_plan() → log warnings
          → replace self.plan, reset step_index=0
          → log + publish state: "plan received: <name>"
```

## ROS Topic: `/drone/mission/plan`

| Field | Value |
|---|---|
| Type | `std_msgs/String` |
| Publisher | `web_dashboard_node` (laptop) |
| Subscriber | `mission_executor_node` (Pi) |
| QoS | RELIABLE, depth 5 |
| Direction | laptop → Pi |
| Trust | Advisory (Pi validates before use) |

## File Map

| File | Role |
|---|---|
| `ros_ws/src/dronetrack_web_bridge/dronetrack_web_bridge/mission_plan_model.py` | Pure-Python plan model: schema, validation, serialization, lint |
| `ros_ws/src/dronetrack_web_bridge/dronetrack_web_bridge/mission_preview_ext.py` | Thin wrappers for dashboard use |
| `ros_ws/src/dronetrack_web_bridge/dronetrack_web_bridge/web_dashboard_node.py` | Interactive builder HTML/JS + CRUD API + plan publisher |
| `ros_ws/src/drone_control/drone_control/mission_executor_node.py` | Plan subscriber (`_on_plan_received`) + executor state machine |
| `ros_ws/src/drone_control/drone_control/mission_plan.py` | Pi-side plan model (parse, validate, lint) |
| `configs/groundstation.yaml` | `mission_plans_dir`, `mission_plan_topic` |
| `configs/pi.yaml` | `mission_plan_topic` |
| `configs/topics.yaml` | `/drone/mission/plan` topic spec |

## Edge Cases

| Case | Behavior |
|---|---|
| Empty plan send | `validate_step` catches empty; returns errors, not published |
| Send while mission active | `_on_plan_received` rejects with logged warning |
| Concurrent sends | Idempotent — last one wins, each replaces `self.plan` |
| File save collision | Returns 409 Conflict; browser prompts to overwrite |
| localStorage corruption | `JSON.parse` fails → builder starts with empty plan |
| Invalid YAML from dashboard | `parse_mission_plan` raises → logged + rejected |
| Mission plan with lint warnings | Plan accepted; warnings logged on both sides |
| Dashboard disconnected after send | Plan already published — executor keeps it |
| goto_relative/absolute in plan | Executor has handlers; control_node handles GOTO mode |
