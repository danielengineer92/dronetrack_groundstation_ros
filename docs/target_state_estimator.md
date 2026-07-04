# Target State Estimator — velocity & direction for arbitrary motion

`target_state_estimator_node` (drone_tracker) answers "where is the target,
how fast is it moving, in what direction, and where will it be in half a
second" — **model-free about the path**: circles, straight lines, stop-and-go
all work. It exists because the yaw feed-forward only knows the *angular*
rate; leading a target positionally (orbit-ahead, intercept, latency-cancelled
pursuit) needs the full world-frame velocity vector.

## How it works

1. Every detection is projected into PX4 local NED using only measurements
   (never our own commands, so our maneuvering cannot contaminate it):
   `horizontal = distance·cos(bearing_y)`, then rotate by `yaw + bearing_x`
   from the drone's telemetry position.
2. A constant-velocity Kalman filter (state `[N, E, vN, vE]`) fuses the
   samples. The measurement covariance is **anisotropic in the LOS frame**:
   cross-range (bearing, ~0.23°) is centimeter-class, radial (monocular
   `k/bbox_px`) is ~8 % of range — at 9 m one pixel of bbox jitter is ~0.6 m.
   Naive isotropic noise would bleed range error into the good axis.
3. Chi-square gating rejects bbox glitches; the filter coasts through short
   target losses and resets after long gaps. Outputs on
   `/drone/tracking/target_state` (10 Hz): position, velocity, speed,
   heading, ±0.5 s prediction, 1-sigma confidences.

Pure math in `drone_tracker/target_state_math.py` (9 unit tests); params in
`configs/pi.yaml` under `target_state_estimator_node`.

## Validated against gz ground truth (2026-07-04, SITL)

Drone airborne yaw-tracking; gz `dynamic_pose` as truth; frame map fitted by
Procrustes **allowing reflections** (gz ENU → PX4 NED is an axis swap,
det = −1 — force a proper rotation and every heading comparison is garbage;
the fit residual, 0.43 m, doubles as the end-to-end position accuracy check).

| case | true speed | speed err mean/rms | heading err mean/rms |
|---|---|---|---|
| orbiting ball (dir rotates 18°/s) | 0.94 m/s | +0.06 / 0.18 m/s | +6.9° / 13.9° |
| straight line leg 1 (no circle) | 0.80 m/s | +0.03 / 0.09 m/s | −13.5° / 21.5° |
| straight line leg 2 (no circle) | 0.80 m/s | +0.00 / 0.07 m/s | −4.1° / 9.3° |

The orbit heading rms is dominated by CV-filter lag on a continuously rotating
velocity (expected); the straight-line legs are the clean read: **speed to a
few percent, direction to ~10°.** Leg 1's mean includes the re-convergence
transient after the ball teleports to the leg start.

## Knobs

- `accel_noise_density` (0.3): how hard the target may maneuver. Raise for
  agile targets → faster response, noisier velocity.
- `sigma_range_fraction` (0.08): monocular range trust. If the distance
  calibration (`scripts/calibrate_distance.py`) is re-fit, revisit.
- `prediction_horizon_s` (0.5): lead time for the predicted position.

## Consumers (future)

Nothing consumes `/drone/tracking/target_state` yet. Natural next steps:
orbit-ahead-of-target in the mission executor, and feeding the yaw
feed-forward from the prediction to cancel the ~0.5 s pipeline latency
(the residual 0.054 error in pid_tuning_report.md Update 3).
