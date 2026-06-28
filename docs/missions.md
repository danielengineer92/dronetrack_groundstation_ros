# Mission Plans

Missions are declarative YAML walked by `mission_executor_node` (on the Pi). A
plan is an ordered list of step *verbs*; the executor dispatches each to a step
handler and advances when the step reports done. Plans are pure data — reorder or
swap a flight script by editing/selecting a YAML, not editing code.

The model and loader live in
[`drone_control/mission_plan.py`](../ros_ws/src/drone_control/drone_control/mission_plan.py)
and are pure Python (no rclpy), unit-tested in
[`test/test_mission_plan.py`](../ros_ws/src/drone_control/test/test_mission_plan.py).

> **Safety:** a mission plan never bypasses a Pi safety gate. Every step still
> runs the executor's preflight/airborne/local-position checks, and every MAVSDK
> action and translation/Offboard command is still gated on the Pi exactly as
> described in [safety_contract.md](safety_contract.md). The laptop/dashboard only
> *requests* a mission; the Pi owns execution.

## Format

```yaml
mission:
  name: my_mission
  steps:
    - {type: takeoff, altitude_m: 3.0}
    - {type: prime_offboard, hold_s: 1.5}
    - {type: scan, direction: ccw, yaw_deg: 180, yaw_rate_deg_s: 20, until: locked, timeout_s: 12}
    - {type: track_center, until: centered, timeout_s: 15}
    - {type: approach, distance_m: 2.0, until: approach_done, timeout_s: 20}
    - {type: orbit, radius_m: 2.0, speed_m_s: 0.4, revolutions: 1, timeout_s: 45}
    - {type: rtl, timeout_s: 15}
    - {type: land}
```

A step may also be a bare string (`- takeoff`). Any key omitted falls back to the
matching `mission_executor_node` parameter (see [configs/pi.yaml](../configs/pi.yaml)).

### Verbs

| Verb | Meaning | Common keys |
|---|---|---|
| `takeoff` | TAKEOFF if not already airborne | `altitude_m` |
| `prime_offboard` | Hold zero/HOLD setpoints so control_node captures its anchor and Offboard engages | `hold_s` |
| `scan` | **Hold position and yaw-sweep to search for the target** | `direction`, `yaw_deg`, `yaw_rate_deg_s`, `until`, `timeout_s` |
| `track_center` | Yaw-center the locked target (no translation) | `until`, `timeout_s` |
| `approach` | Close to a desired distance (yaw-centers; translation opt-in, see below) | `distance_m`, `until`, `timeout_s` |
| `orbit` | PX4 `DO_ORBIT` around the estimated target center | `radius_m`, `speed_m_s`, `revolutions`, `timeout_s` |
| `rtl` | Return to launch | `timeout_s` |
| `land` | Land | `timeout_s` |
| `hold` | Hold position | `status`, `timeout_s` |

`until` predicates: `airborne`, `centered`, `locked`, `approach_done`, `none`
(never auto-advance — exit only on `timeout_s`, if any).

### The `scan` step (Phase 1)

`scan` holds the captured local-NED anchor and rotates **in place** — yaw only, no
translation, so it stays inside the existing position-hold+yaw safety envelope.

- `direction`: `ccw` (counter-clockwise viewed from above; negative NED yaw rate)
  or `cw`. Default `ccw`.
- `yaw_deg`: total sweep angle before the step completes. Default param
  `scan_yaw_deg` (180).
- `yaw_rate_deg_s`: sweep rate. Default param `scan_yaw_rate_deg_s` (20).
- `until: locked`: **exit early the moment the target locks** (recommended).
- `timeout_s`: hard cap so a never-seen target can't sweep forever. Default param
  `scan_timeout_s` (12).

Exit conditions (any one advances the plan to the next step): target locks
(`until`), full `yaw_deg` swept, or `timeout_s` elapses. On timeout the plan
simply advances — make the next step a `hold` or `land` if you want a specific
fallback. When the executor publishes `SCAN`, `control_node` integrates the
commanded yaw rate into its held-position yaw target.

### The `approach` step (Phase 3)

`approach` reads its goal distance from `distance_m` (falling back to
`desired_approach_distance_m`) and exits on `until: approach_done` when the
tracker's `distance_m` is within `approach_distance_tolerance_m` of the goal.

By default `approach` **holds position and yaw-centers** (no forward motion) — the
conservative behavior. Real forward translation is opt-in and **double-gated**;
both must be true to move:

1. `control_node.enable_approach_translation: true` — enables the forward-velocity
   command path. The math is the pure, unit-tested `approach_forward_velocity` in
   [`control_math.py`](../ros_ws/src/drone_control/drone_control/control_math.py)
   (`test/test_control_math.py`). It returns 0 (hold) on any stale/invalid/lost
   distance, so the stale-target fail-safe is preserved.
2. `telemetry_node.allow_translation_commands: true` — the Pi-side gate that
   actually lets a VELOCITY setpoint reach PX4.

Both default **false**. With defaults, `approach` is harmless.

## Linting

`lint_plan()` returns non-fatal warnings (logged on load). It flags:

- a motion step (`scan`/`track_center`/`approach`/`orbit`) before any `prime_offboard`;
- a plan with motion steps but no `prime_offboard` at all;
- `scan`/`approach`/`orbit` with no `timeout_s`;
- a `scan` with neither `until` nor `timeout_s` (would sweep forever).

## Selecting a plan

Set `mission_executor_node.mission_plan_file` to the YAML path (empty = built-in
default, which reproduces the original `takeoff → prime_offboard → track_center`
hold). Installed examples live under the package share, e.g.
`install/drone_control/share/drone_control/missions/scan_and_orbit.yaml`. A bad or
missing file logs an error and falls back to the built-in default — the node never
crashes on a bad plan.

The laptop dashboard has a **preview-only** mission-plan selector. It loads local
YAML files, displays parser/lint warnings, and shows the step list, but it does
not change the Pi's active plan over the network. The active plan remains the
Pi-side `mission_executor_node.mission_plan_file` parameter, set before mission
start. This keeps mission selection inside the existing trust boundary until a
dedicated Pi-validated advisory selection protocol exists.

Example plans:
[`example_min.yaml`](../ros_ws/src/drone_control/missions/example_min.yaml),
[`orbit_red_ball.yaml`](../ros_ws/src/drone_control/missions/orbit_red_ball.yaml),
[`scan_and_orbit.yaml`](../ros_ws/src/drone_control/missions/scan_and_orbit.yaml).

## Running `scan_and_orbit` — SITL first, then bench

**Always validate in PX4 SITL before any powered hardware.**

### 1. SITL (no airframe, no Pi)

1. Start PX4 SITL (MAVLink on udp 14540), e.g.
   `sudo docker run --rm -it --network host px4io/px4-sitl:latest`.
2. Point the mission at this plan and enable the actions SITL needs. Either set in
   the SITL config / launch overrides:
   - `mission_executor_node.mission_plan_file:=<share>/missions/scan_and_orbit.yaml`
   - `telemetry_node.allow_mavsdk_actions:=true` (TAKEOFF/DO_ORBIT/RTL/LAND)
   - (optional, to exercise forward motion) `control_node.enable_approach_translation:=true`
     **and** `telemetry_node.allow_translation_commands:=true`
3. Launch the SITL mission stack: `bash scripts/sim_sitl.sh`.
4. In the PX4 `pxh>` shell: `param set COM_DISARM_PRFLT -1`, then `commander arm`.
5. On the dashboard (`http://127.0.0.1:8091/`): **System Ready → Start Mission**.
   Watch `/drone/mission/state` walk `TAKEOFF → PRIME_OFFBOARD → SCAN →
   TRACK_CENTER → APPROACH_TARGET → DO_ORBIT → RETURN_TO_LAUNCH → LAND → COMPLETE`.

Verify the scan sweeps CCW and exits the instant the faked target locks, that
approach holds (or closes to ~2 m if translation was enabled), and that abort/land
from the dashboard publish only advisory requests.

### 2. Hardware bench, **props off**

Only after SITL looks correct. Keep props removed and the airframe restrained.

1. Keep the hardware-safe defaults: `allow_mavsdk_actions: false`,
   `allow_translation_commands: false`, `enable_approach_translation: false`.
   With these, no real motion/actions are emitted — you're checking that the
   mission *sequences* and *gates* behave on real telemetry.
2. Arm on the bench (props off) and run **Start Mission**. Confirm the state
   machine advances on real telemetry, that `scan` requests yaw-only commands, and
   that every blocked gate reports a clear reason in `/drone/mission/state`.
3. Re-enable actions one at a time (`allow_mavsdk_actions:=true` first, still props
   off) and confirm TAKEOFF/RTL/LAND requests reach the action gate as expected.
4. Only enable translation (`enable_approach_translation` + `allow_translation_commands`)
   on a real airframe outdoors with props on, GPS, and space — never on the bench.
