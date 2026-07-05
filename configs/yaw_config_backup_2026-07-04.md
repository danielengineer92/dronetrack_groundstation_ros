# Yaw config backup — SITL candidate (before experimental override)

**Saved 2026-07-04**, before switching `configs/pi.yaml` to a bare-PD+FF
experimental baseline (kp 1.6 / kd 0.15 / ff 1.0, tuning aids off).

This file preserves the **SITL-candidate** yaw config from commit `63d41c1`
(docs/controller_math_review.md §11). It is a SITL candidate — **not
flight-validated**. Use it to restore the tuned config.

## Values that were in effect (control_node)

| param | SITL-candidate value | notes |
|---|---|---|
| `gain_yaw` (P) | **2.0** | pure-P sweep knee |
| `yaw_kd` (D) | **0.3** | phase-lead margin, raw-error derivative |
| `yaw_ki` (I) | 0.0 | off |
| `yaw_ff_gain` | **0.8** | full FF over-drives the near pass; 0.8 = balance |
| `yaw_ff_source` | "state" | |
| `yaw_ff_limit` | 1.5 | |
| `yaw_ff_lpf_alpha` | 0.45 | (inactive; source=state) |
| `yaw_ff_align_error_limit` | **0.6** | FF acquisition gate ON |
| `yaw_ff_align_inner` | 0.0 | ramp, no knee |
| `yaw_ff_state_lpf_alpha` | 1.0 | FF-state LPF off (filtering made it worse) |
| `yaw_d_lpf_alpha` | 0.35 | D low-pass |
| `yaw_i_limit` | 0.3 | (inactive; ki=0) |
| `yaw_approach_limit_enabled` | **true** | braking cap ON |
| `yaw_approach_delay_s` | 0.35 | |
| `yaw_approach_decel` | 0.8 | |
| `yaw_approach_min_rate` | 0.35 | |
| `max_yaw_rate` | 1.0 | actuation ceiling |
| `max_yaw_accel` | 1.2 | slew limit (load-bearing damping) |
| `log_yaw_terms` | false | bring-up instrument |

**estimator (target_state_estimator_node)**

| param | value |
|---|---|
| `pair_at_capture_stamp` | true |
| `accel_noise_density` | 0.3 |
| `prediction_horizon_s` | 0.5 |

## Restore

**Persistent** — set these back in `configs/pi.yaml`, then
`scripts/ros_wsl.sh build-sim` to re-stage. (Git also has the exact file at
`63d41c1:configs/pi.yaml`: `git show 63d41c1:configs/pi.yaml > configs/pi.yaml`.)

**Live** — with the stack running (sim env sourced):

```bash
ros2 param set /control_node gain_yaw 2.0
ros2 param set /control_node yaw_kd 0.3
ros2 param set /control_node yaw_ff_gain 0.8
ros2 param set /control_node yaw_ff_align_error_limit 0.6
ros2 param set /control_node yaw_approach_limit_enabled true
# (these were already at the candidate value; included for completeness)
ros2 param set /control_node yaw_ff_state_lpf_alpha 1.0
ros2 param set /control_node yaw_ff_align_inner 0.0
ros2 param set /control_node yaw_ki 0.0
ros2 param set /target_state_estimator_node pair_at_capture_stamp true
```
