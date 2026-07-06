# Field debrief — first outdoor runs, 2026-07-05

Three missions flew outdoors (dashboard-built plan: takeoff → prime → scan →
track_center → orbit). None completed the orbit. Full forensics from the Pi
launch logs (persistent copies in `~/.ros/log/`), the GS log, and a live Pi
sweep. This document records the fault tree and what changed in response.

## What actually happened

- **Mission #1** ran scan for its full 30 s with the ball already locked (the
  builder plan had no `until: locked`), then was stopped by the operator.
- **Mission #2** is the informative one: the (just-added) orbit entry envelope
  correctly commanded APPROACH_TARGET and the drone drove at 0.52→0.97 m/s
  toward the ring — then the tracker lost the ball for ~13 s, autonomy flapped,
  and the pilot took over (OFFBOARD→POSCTL) and landed.
- The **orbit strafe never executed in the air** (`right=0.000` in every
  control command; `mission_mode` never reached ORBIT_TARGET).

## Fault tree (ranked, with evidence)

1. **Chronic GS↔Pi link flakiness.** `HEARTBEAT_STALE` watchdog losses every
   ~15–30 s through *all* missions (timeout was 1.0 s). Two losses match the
   autonomy `REQUEST_OFF` flaps within ~12 ms. Camera→GS frames kept flowing;
   it was the GS→Pi heartbeat direction that dropped. Every drop de-asserted
   autonomy → control froze mid-approach.
2. **YOLO model dropouts outdoors.** During the approach, detections went to
   zero for ~13 s while the detection *stream* stayed fresh (empty frames —
   model failure, not transport; the gate dropped nothing). Reacquire
   confidences 0.09–0.44 vs 0.85+ stable. No configured threshold was
   responsible — everything already sat at 0.05–0.08.
3. **Yaw centering was hunting.** `error_x` oscillated with sign flips to both
   frame edges (−0.97 → +0.89); the +0.89 swing immediately preceded the loss.
   Root cause: the tracker was configured with a guessed 62° FOV, but the lens
   is a **120° wide-angle** — fx wrong by ~2.9×, feeding garbage bearings into
   the KF LOS-rate feed-forward and the yaw braking cap.
4. **Battery subsystem, 3 defects** (found during forensics; the pack was
   genuinely at 7–16% during some flying):
   - telemetry multiplied MAVSDK v3's 0–100 `remaining_percent` by 100
     (7% read "700%"), defeating both `>0 && <min` low-battery gates;
   - control_node's gate had no unknown-value guard, so the bench sentinel
     (−100%) blocked every command on USB power (7,343 BLOCKED_LOW_BATTERY);
   - unknown-battery semantics were inconsistent across the three gates.

## Changes made in response (same day)

| Problem | Change |
|---|---|
| Link in the perception path | **YOLO moved onto the Pi** (`local_yolo` in `pi_launch.py`, ncnn 320×320 on CPU, publishing straight to the tracker; detection gate bypassed for the local source). GS defaults to dashboard-only (`up.sh` GS_YOLO). |
| ncnn starved by the stack (500–730 ms/inf) | CPU split: stack pinned to cores 0–1 (`pi_split_launcher.sh`), YOLO to cores 2–3 (`taskset` prefix) → **237 ms / 4.2 fps**. Also: frame throttle moved before the JPEG decode in `yolo_node`. |
| QoS mismatch (tracker RELIABLE vs yolo BEST_EFFORT → zero delivery) | `reliable_detections` param on `yolo_node`; local launch sets it true. |
| Link flaps killing autonomy | Watchdog `max_heartbeat_age_s` 1→10 s in the hardware config (backstop stays). |
| Wrong FOV / yaw hunting | `horizontal_fov_deg: 120`, `vertical_fov_deg: 105` + first-flight yaw detune (gain_yaw 1.0, yaw_kd 0.1, yaw_ff_gain 0.4, max_yaw_rate 0.6). |
| Battery | Scale fix (drop ×100, clamp, −1 = unknown), control-gate unknown-guard, all three gates consistent. |
| Disarmed-on-ground "airborne" wedge | `is_airborne()` returns False when disarmed. |
| Config clobbering (sim staged over hardware values) | New tracked `configs/pi_hardware.yaml`; `deploy_pi.sh` always stages it for the Pi. |
| Mission authoring | Builder seeds `scan until:locked`; lint warns on orbit without approach. |

## Bench results (same day, on the Pi 4, stack running, GS fully off)

| model | imgsz | inference | fps | conf @ ~4 m | conf @ ~1.4 m |
|---|---|---|---|---|---|
| red_ball_ncnn_model | 320 | **238 ms** | **4.2** | 0.14–0.45 (locks) | 0.86 |
| red_ball_480_ncnn_model | 480 | 1134 ms | 0.9 | 0.18–0.20 | — |

480 is a bad trade: 4.8× slower for no meaningful confidence gain — the range
ceiling is the model's training data, not input resolution. **320 is the
operational default**; the 480 export ships as a switchable option
(`PI_YOLO_MODEL`/`PI_YOLO_IMGSZ` env → launch args). End-to-end verified with
the GS off: camera → local ncnn → tracker LOCKED, distance calibration holds
(reads 3.57–4.4 m at ~4 m truth), battery sentinel clean (−1 = unknown, no
false blocks).

## Still open

- The **model itself** is weak outdoors and past ~4 m (zero detections for
  13 s in flight; conf ≤0.45 at 4 m on the bench regardless of imgsz). Real
  fix: collect outdoor flight imagery and retrain (deferred by operator
  decision). Until then, missions should scan/acquire from ≲4 m standoffs.
- The orbit strafe remains **flight-unproven**.
- Real camera FOV set from the lens spec (120°); a measured verification pass
  is still worthwhile.
