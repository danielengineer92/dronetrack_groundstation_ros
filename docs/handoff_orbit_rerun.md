# Handoff: make the orbit mission re-runnable from a landed state

> **STATUS: FIXED** (see the commit touching this file). What was done:
> 1. `docker/px4_headless.sh` injects `NAV_DLL_ACT 0`, `NAV_RCL_ACT 0`,
>    `COM_RCL_EXCEPT 4`, `COM_DISARM_LAND 2.0` in the background once PX4
>    prints "Ready for takeoff" — a dropped GCS client can no longer latch
>    an arming denial.
> 2. `_takeoff_ground_recovery()` in `mission_executor_node.py`: when the
>    takeoff step finds the vehicle on the ground it breaks out of AUTO.LAND
>    with a HOLD, then requests ARM through the gated MAVSDK action path
>    (telemetry still enforces allow_mavsdk_actions AND allow_arm_via_mavsdk).
>    A mission re-run now needs only `POST /api/mission_request` — no
>    dashboard arm click, no pxh commands.
>
> Acceptance test: cold start + 3 consecutive runs (runs 2–3 with NO manual
> arm), each verified orbiting via gz world pose (~7.4–7.7 m XY displacement
> per 12 s at r=4/v=1.0), each landing + disarming cleanly; zero
> "Arming denied" in `/tmp/px4_raw.log`. The original write-up is kept below
> for context.

## Your task
After the `orbit_red_ball` mission lands, a **second** run in the same PX4
session cannot take off on its own — the operator has to poke PX4 by hand.
Fix it so repeated `arm → mission_request` cycles "just work," with no manual
`pxh` commands between runs. Do this without weakening the real preflight
safety gates (this is a SITL convenience, keep it SITL-scoped).

## System / where things live
- Repo: `~/Documents/Python/dronetrack_groundstation_ros`, branch `px4-gazebo-sim`.
- Everything runs in the `dronetrack-sim` Docker container (ROS 2 Jazzy +
  PX4 SITL + Gazebo Harmonic on an RTX 5080). `docker exec dronetrack-sim ...`.
- Launch model (all headless/detached):
  - `docker/px4_headless.sh` → PX4 + Gazebo. Inject pxh commands with
    `docker exec dronetrack-sim bash -c 'echo "commander takeoff" > /tmp/px4in'`.
    Readable log: `/tmp/px4.log`; full log: `/tmp/px4_raw.log`.
  - `docker/stack_headless.sh orbit_red_ball` → the ROS 2 stack. Log `/tmp/stack.log`.
  - Dashboard API on `http://127.0.0.1:8091`: `POST /api/arm {"confirm":true}`,
    `POST /api/mission_request {"enabled":true|false}`, `POST /api/land`,
    `GET /api/status`.
- The sim overlay is **symlink-installed** from source, so editing files under
  `ros_ws/src/**` is live for any freshly-launched node — no colcon rebuild
  needed, just restart the stack (`stack_headless.sh` re-runs with a
  single-instance guard).

## What already works (don't re-litigate — commit 5486dce)
A cold start → `arm` → `mission_request` runs the full plan end to end:
takeoff(4 m) → prime_offboard → track_center → **orbit** → land, and the orbit
is a real ~4 m circle around the ball. The orbit uses MAVSDK
`action.do_orbit()` (COMMAND_INT) with offboard released during the orbit step.
NOTE: during orbit the dashboard shows `flight_mode=POSCTL` — that is just
MAVSDK's FlightMode enum having no "Orbit" value; PX4 is genuinely orbiting.
Confirm motion via NED position, not the mode string.

## The bug (reproduce)
1. Cold start, run the mission, let it land (ends `mission_state=COMPLETE`,
   `flight_mode=LAND`, disarmed, on ground).
2. `POST /api/arm` then `POST /api/mission_request {"enabled":true}`.
3. It wedges on step `1_takeoff_if_needed` with
   `mission_state=TAKEOFF: blocked: PX4_NOT_ARMED`, never climbs.

## Root cause (two compounding issues, both confirmed in logs)
1. **Stuck in AUTO.LAND.** After landing PX4 stays in `AUTO.LAND`. Arming while
   on the ground in that mode triggers an immediate `commander: Disarmed by
   landing` (COM_DISARM_LAND grace), so the arm never sticks.
2. **Latched "no GCS" preflight fail.** `/tmp/px4_raw.log` shows
   `Preflight Fail: No connection to the GCS` → `Arming denied: Resolve system
   health failures first`. PX4's datalink-loss check latches once a GCS-type
   link connects and drops. (It gets tripped by anything speaking on the GCS
   port 14550 — e.g. a QGC or a throwaway MAVSDK script — then leaving.)

The mission's `_step_takeoff` (in
`ros_ws/src/drone_control/drone_control/mission_executor_node.py`) gates on
`_check_preflight_or_hold` which requires `armed`, and arming never succeeds —
hence the deadlock. Arming/takeoff is expected to be initiated externally
(dashboard `arm` + the TAKEOFF action), and neither can break AUTO.LAND.

## The manual workaround I used (what to automate)
One shot through the pxh shell bootstraps out of LAND:
`docker exec dronetrack-sim bash -c 'echo "commander takeoff" > /tmp/px4in'`
— PX4 arms, climbs to 4 m, settles into HOLD; then `mission_request` proceeds
straight into the orbit (takeoff step sees it airborne and skips).

## Recommended fix (two parts)
1. **Kill the GCS-loss arming block for SITL.** Set, once after PX4 boots:
   `NAV_DLL_ACT 0`, `NAV_RCL_ACT 0`, `COM_RCL_EXCEPT 4` (keep
   `COM_DISARM_LAND 2.0`). Persisting these means a dropped GCS never blocks
   arming. `px4_headless.sh` currently `exec`s `make px4_sitl`, so it can't set
   params after boot inline — add a small background injector that waits for
   `/tmp/px4.log` to show "Ready for takeoff", then writes the `param set`
   lines to `/tmp/px4in`. (The watchdog `recover()` already does exactly this
   pattern for `COM_DISARM_LAND`.)
2. **Break AUTO.LAND before takeoff.** In `_step_takeoff`, when
   `not is_airborne()` and telemetry mode is `LAND` (landed on ground), request
   a HOLD first (so PX4 leaves AUTO.LAND and stops auto-disarming), then let the
   existing arm/TAKEOFF path run. Alternatively drive it through the TAKEOFF
   action so the executor can bring it up without an external arm click.
   Keep the `is_airborne()` threshold vs takeoff-altitude margin intact (that
   flap was fixed by taking off to 4 m; don't reintroduce it).

## Verify (the acceptance test)
From a cold start, run the mission to completion, then **without any manual pxh
command**, `arm` + `mission_request` again and confirm it takes off and
re-enters the orbit. Repeat 3× back-to-back. Confirm the orbit by sampling NED
position (a ~4 m circle), not by the `flight_mode` string. Watch
`/tmp/stack.log` and `/tmp/px4_raw.log` for `Arming denied` / `Disarmed by
landing` — they should be gone.

## Gotchas
- Don't leave stray MAVSDK/QGC clients on udp:14550 — connecting then dropping
  re-latches the GCS-loss check you're trying to defeat.
- After editing `ros_ws/src/**`, restart the stack (don't rebuild). The
  `stack_headless.sh` guard kills the old launch AND its node children first;
  verify exactly one live `mission_executor_node` / `telemetry_node` after.
- The ball is pinned still (`ball_radius:=0.0`) and scenery posts come from
  `scripts/spawn_scenery.sh` (re-run after any container restart — they don't
  persist). Neither is related to this bug.
- Files most likely in scope: `docker/px4_headless.sh`,
  `ros_ws/src/drone_control/drone_control/mission_executor_node.py`. Tests:
  `pytest ros_ws/src/drone_control/test/` (38 should stay green).
