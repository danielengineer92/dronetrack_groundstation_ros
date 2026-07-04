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

## Tuning (2026-07-04)

Knobs, the variables tracked while tuning, and the measured sweep.

**Tracked variables** — vs gz truth (sim): prediction-error-at-horizon
|p̂(t+h) − p_true(t+h)| (the deliverable metric), speed/heading error;
filter-internal (works on hardware, no truth needed): **avg NIS** (normalized
innovation squared, logged by the node every 10 s — healthy ≈ 2.0 for 2-DOF;
higher = overconfident, lower = underconfident) and **gated %** (target <3%).
`accel_noise_density` and the sigma params are runtime-settable
(`ros2 param set /target_state_estimator_node ...`) for live sweeps.

q sweep on the orbiting ball (40 s each, prediction horizon 0.5 s):

| q | pred@h err (m) | no-lead err (m) | avg NIS | gated % | still→circle recovery |
|---|---|---|---|---|---|
| 0.1 | 0.546 | 0.654 | 0.52 | 2.6 | 1.0 s |
| **0.3** | **0.483** | 0.626 | **1.71** | 2.6 | 0.5 s |
| 1.0 | 0.466 | 0.607 | 0.47 | 2.3 | 0.2 s |
| 3.0 | 0.485 | 0.625 | 1.40 | 2.3 | 0.1 s |

**q = 0.3 is the tuned choice**: prediction error within 4 % of best while the
only NIS in the healthy band (1.71 ≈ 2) — q=1.0 predicts marginally better but
is badly underconfident (NIS 0.47), making the published std-devs untrustworthy.
`sigma_range_fraction: 0.08` is corroborated by the k-fit spread (±10 % over
19 ground-truth samples) and the near-2 NIS.

## Consumers (validated 2026-07-04)

- **State-fed yaw feed-forward** (`yaw_ff_source: state` in control_node):
  LOS rate computed geometrically from the predicted relative state
  (`los_rate_from_state` in control_math.py) instead of differentiating the
  delayed camera angle. Measured **parity** with `los_diff` (mean|e| 0.053 vs
  0.056) — on curved motion the KF velocity carries similar lag to the
  los-diff low-pass, so latency cancellation nets out. Default stays
  `los_diff`; the state source remains available.
- **Orbit-ahead** (`orbit_lead_s` in mission_executor, default 0 = off):
  leads the DO_ORBIT centre to where the ball will be. Straight `v x lead`
  overshoots and mis-aims on a turning target — the tangent isn't the chord.
  `orbit_lead_curved: true` (default) extrapolates on a **constant-turn-rate
  arc** (`curved_lead_offset` + `estimate_turn_rate` in control_math.py; turn
  rate from the velocity heading slope). Swept straight vs curved across
  q=0.3..30 (2 s lead): curved cut direction error ~23 deg -> ~9 deg,
  q-robust. Live orbit mission (17 samples, curved): **direction error 4.8 deg
  (was ~23), magnitude 106 % of true displacement (was ~114)** — ~4x smaller
  lead-point placement error. PX4 flew the led orbit to completion.
  Residual ~6 % magnitude overshoot = the estimator's velocity reads ~12 %
  high on the orbit (candidate for a distance-calibration recheck; direction,
  which dominates placement, is the fixed part).

- **QoS trap (bit us once):** the estimator publishes BEST_EFFORT; any
  subscriber left at default RELIABLE QoS silently receives NOTHING. Both
  consumers subscribe BEST_EFFORT — do the same in new consumers.
