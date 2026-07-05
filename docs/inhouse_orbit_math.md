# In-house visual-servo orbit — math & design

_Applies to the `px4-gazebo-sim` branch. Implemented in `control_node.py`
(`ORBIT_TARGET` branch), selected by `use_mavsdk_do_orbit: false`._

## 1. Why we don't use PX4's `MAV_CMD_DO_ORBIT`

The mission executor originally handed the orbit to PX4: estimate the target's
global centre (lat/lon), release offboard, and send `MAV_CMD_DO_ORBIT` so PX4's
ORBIT flight mode flies the circle. Two problems killed that path on this build:

1. **`DO_ORBIT` is accepted but never engages.** A direct MAVSDK probe
   (`action.do_orbit(...)` from a hovering vehicle, 2026-07-05) returned
   `ACCEPTED`, yet the flight mode stayed `POSCTL` — PX4 never entered ORBIT.
   The whole hand-off (offboard release → `DO_ORBIT` → recapture) can therefore
   never complete; the mission wedged in "orbit hold / releasing offboard".
2. **The centre needs a global position estimate.** Even when it engaged,
   `DO_ORBIT` orbits a fixed lat/lon computed once from a monocular bearing +
   range. Any bias in that single projection (the stale-yaw pairing bug, or
   distance-calibration error) offsets the whole circle from the ball.

The in-house orbit removes both failure modes: it **never leaves OFFBOARD** and
it **never needs a world-frame target position** — the circle is closed-loop on
the target's *image* position and *range*, so it is centred on the ball by
construction.

## 2. Frames and the command

Setpoints go to PX4 as `VelocityBodyYawspeed(v_f, v_r, v_d, ω)` (MAVSDK
offboard, body-FRD):

| symbol | axis | meaning |
|--------|------|---------|
| `v_f`  | body +X (front) | forward velocity |
| `v_r`  | body +Y (right) | rightward velocity |
| `v_d`  | body +Z (down)  | descent (held 0 — planar orbit) |
| `ω`    | yaw rate        | rotation about body +Z |

Every control tick the `ORBIT_TARGET` branch emits three quantities:

```
ω    = yaw_pid(error_x) + FF          # keep the nose on the ball
v_f  = approach_forward_velocity(d, r) # regulate slant range to the radius r
v_r  = v_t  (constant)                 # strafe tangentially at v_t
```

then `limit_motion(...)` clamps each to its velocity/yaw limit and rate-limits
it by `max_accel · control_period`.

## 3. The three loops

### 3.1 Yaw loop — pin the nose on the ball
`error_x` is the target's normalised horizontal offset in the image; the true
camera bearing is `β = atan((c_x − W/2) / f_x)`. A PID on `error_x`
(plus a Kalman line-of-sight feed-forward, and a delay-aware braking cap so it
does not swing past centre) drives `error_x → 0`, i.e. **`β → 0`**. With the
nose on the ball, the line of sight (LOS) lies along body **+X**. This is the
loop that makes the geometry below hold.

### 3.2 Radial loop — hold the ring radius
Because the LOS is along +X, **forward motion is purely radial** (toward/away
from the ball). `approach_forward_velocity` is a proportional law on the range
error with two refinements:

```
v_f = clamp( k · (d − r),  −v_fmax,  +v_fmax )
```

- `d` = monocular slant range from the tracker (`d = k_cal / bbox_diameter_px`),
- `r` = desired radius (the orbit step's `radius_m`, passed as
  `desired_distance_m`),
- zeroed inside a deadband; capped by a **delay-aware trapezoidal braking
  profile** (`braking_speed_limit`) so a far target is closed on at full speed
  but decelerates *into* the ring instead of lunging past it (the vision range
  is ~0.35 s stale); the **closing** command is scaled down while the target is
  off-centre (`alignment_scale`) — charging forward is the wrong move until the
  nose is on the ball.

Sign: `d > r` ⇒ `v_f > 0` (move in), `d < r` ⇒ `v_f < 0` (back off). Steady
state ⇒ `d ≈ r`.

### 3.3 Tangential loop — drive around the ring
`v_r = v_t` is a **constant** body-right velocity (`orbit_tangential_speed_m_s`,
default 0.4 m/s). Since +Y is perpendicular to the LOS, this is pure tangential
motion — it carries the drone around the ball. Its sign sets the direction
(CW/CCW seen from above).

## 4. Why the path is a circle centred on the ball

Two feedback constraints hold in steady state:

```
|LOS| = d = r            (radial loop)
bearing β = 0            (yaw loop)
```

The first fixes the drone's distance to the ball at `r`; the second fixes its
heading on the ball. The locus of points at fixed distance `r` from the ball is
exactly a **circle of radius `r` centred on the ball**. The tangential velocity
`v_t` moves the drone along that locus. No world-frame target coordinate ever
enters — the centre *is* the ball because the range and bearing are measured to
the ball every tick. A calibration/bias error can change the *radius*, but it
**cannot decentre the circle**, which was the failure mode of the DO_ORBIT
path.

### Kinematics
For a target treated as stationary, moving tangentially at speed `v_t` on a
circle of radius `r` gives an orbital angular rate

```
Ω = v_t / r                    [rad/s]          (period T = 2πr / v_t)
```

As the drone advances along the ring, the LOS to the ball rotates at that same
rate, so to keep `β = 0` the body must yaw at

```
ω_steady = Ω = v_t / r
```

The yaw loop supplies this **automatically**: the tangential strafe makes the
ball drift in the image, the yaw PID rotates to re-centre it, and the loop
settles at `ω = v_t/r`. The operator never has to compute a feed-forward yaw
rate for the orbit.

**Defaults** (`r = 2.0 m`, `v_t = 0.4 m/s`):

```
Ω = 0.4 / 2.0 = 0.20 rad/s
T = 2π·2.0 / 0.4 ≈ 31.4 s per revolution
ω_steady = 0.20 rad/s ≈ 11.5 °/s   (well under max_yaw_rate = 1.0 rad/s)
```

## 5. Radius fidelity depends on the distance calibration

The radial loop regulates the *measured* range to `r`. If the tracker's range
is biased by a scale factor `s` (`d_meas = s · d_true`), the loop drives
`d_meas → r`, so the *true* radius settles at `r / s`. A 22 % range over-read
(the pre-fix state, `k = 147` contaminated by a self-detection false positive)
therefore held the drone ~22 % too close and made the ring the wrong size. The
recalibration to `k = 120.3` (clean 3 m/7 m gz-truth fit, 8.5-in ball) brings
the held radius to within a few percent of commanded. **Radius accuracy is
exactly range-calibration accuracy** for this controller.

## 6. Coupling & stability

`v_f` (radial) and `v_r` (tangential) are orthogonal body axes; the yaw loop
couples them. If yaw lags the LOS by a small angle `δ`, the strafe acquires a
radial component `v_t·sin δ` and forward acquires a tangential component — the
radial loop absorbs the former (range error), the yaw loop drives `δ → 0`.
Stability holds while the yaw loop is much faster than the orbit rate, which it
is: yaw bandwidth ≫ `Ω = 0.2 rad/s`. Both the yaw and forward loops carry the
delay-aware braking caps, so the ~0.35 s vision latency does not turn into
overshoot/limit-cycling.

## 7. Parameters

| parameter | node | default | role |
|-----------|------|---------|------|
| `use_mavsdk_do_orbit` | mission_executor | **false** | select the in-house orbit over PX4 DO_ORBIT |
| `enable_approach_translation` | control | true (gazebo launch) | master gate for ORBIT/APPROACH translation; pi.yaml raw default `false` for hardware caution |
| `orbit_tangential_speed_m_s` | control | 0.4 | tangential speed `v_t` (sign = direction) |
| `radius_m` (orbit step) → `desired_distance_m` | plan → control | 2.0 | ring radius `r` |
| `max_velocity_right` | control | 2.0 | clamp on `v_t` |
| `max_velocity_forward` | control | 2.0 | clamp on `v_f` |
| `max_yaw_rate` | control | 1.0 rad/s | clamp on `ω` |
| `distance_calibration_k` | tracker | 120.3 | sets range→radius fidelity |

## 8. Safety gates (unchanged envelope)

The orbit reuses the existing control-node safety stance — it does **not** widen
it:

- runs only while the target is `LOCKED` **and** `target_visible`; otherwise
  falls to position-hold + yaw or IDLE;
- `local_position_ready()` required, else IDLE hold (fail-safe on EKF/GPS drop);
- descent blocked by the altitude floor (`v_d` is held 0 here anyway);
- the VELOCITY setpoint is only forwarded by the telemetry bridge when the
  control node stamps `execution_status` with the `SENT` approval prefix;
- gated behind `/drone/autonomy/enabled` like every other command.

## 9. Status & verification

- **Implemented** in `control_node.py`; unit suite green (95 passing).
- **Enabled** under `scripts/ros_wsl.sh gazebo` (`enable_approach_translation`
  defaults `true` there; `use_mavsdk_do_orbit: false` in `configs/pi.yaml`).
- **Recommended in-flight check:** with the ball still at a known point, sample
  the vehicle XY through ≥1 revolution, least-squares-fit a circle, and confirm
  (a) fitted centre ≈ ball position (decentring ≈ 0 by design), (b) fitted
  radius ≈ `r` (validates the range calibration), (c) period ≈ `2πr/v_t`.

## 10. Limitations

- **Planar, constant-altitude** orbit (`v_d = 0`); altitude is whatever the
  approach/takeoff left.
- Radius fidelity is only as good as the **monocular range**, which is
  bbox-diameter based and model-specific — recalibrate `k` after retraining or
  swapping the detector.
- Assumes a **stationary / slow** target. A fast-moving ball would bias the
  ring; the executor's orbit-ahead lead exists for the DO_ORBIT centre but this
  visual-servo path simply tracks the live measurement each tick.
