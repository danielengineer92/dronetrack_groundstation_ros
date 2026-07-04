# Yaw Tracking Tuning Report — DroneTrack SITL

_How the visual-yaw controller was tuned against the orbiting red ball, and how we
diagnosed what was actually wrong. Measured in the PX4 + Gazebo SITL (branch
`px4-gazebo-sim`)._

## The controller

Target following in `TRACK_CENTER` is **position-hold + yaw only**. The drone
holds its captured local-NED anchor and rotates to keep the ball centered
horizontally. The yaw law (in `control_node.py`) is **pure proportional**:

```
error_x   = normalized horizontal image error, in [-1, +1]  (0 = centered)
desired   = apply_deadband(error_x, deadband_x) * gain_yaw
yaw_rate  = slew_limit( clamp(desired, ±max_yaw_rate), max_yaw_accel * dt )
```

No integral, no derivative. There are three knobs that matter:

- `gain_yaw` — proportional gain (image error → commanded yaw rate)
- `max_yaw_rate` — hard saturation on commanded yaw rate (rad/s)
- `max_yaw_accel` — slew-rate limit on how fast the yaw command can change
- `deadband_x` — dead zone around center to stop micro-hunting on noise

D-term was deliberately avoided: `error_x` comes from vision and is noisy, so a
raw derivative would amplify jitter. The saturation + slew limits are what keep
motion smooth instead.

## How we measured

The tracker publishes `/drone/tracking/target_error`. Its `error_x` field is the
whole signal we care about, and it has a useful property: **when the target is
not locked, the tracker zeros `error_x`**. So a single 60-second capture gives us
everything:

```
ros2 topic echo /drone/tracking/target_error --field error_x --csv   # 60 s window
```

From that stream, four numbers:

| Metric | Meaning |
|---|---|
| **lock %** | fraction of samples with `error_x != 0` — how often we held the target |
| **mean \|error_x\|** | steady-state tracking accuracy over locked samples (lower = tighter) |
| **peak \|error_x\|** | worst transient |
| **sign-flips / min** | how often `error_x` crosses zero — our **oscillation proxy** |

The sign-flip count is the key diagnostic. Low sign-flips with high mean error =
the controller is **lagging** (always chasing, never crossing center). High
sign-flips = the controller is **oscillating** (overshooting through center and
back). That distinction is what told us which way to move.

## The tuning sweep

Each row is a fresh 60 s window while the ball orbited (center 6 m, radius 3 m,
20 s period). Gains were changed live via `ros2 param set /control_node …`
(the node supports runtime reconfig, so no rebuild/restart per trial).

| Trial | `gain_yaw` | `max_yaw_rate` | mean \|error_x\| | peak | sign-flips/min | Read |
|---|---|---|---|---|---|---|
| Baseline | 0.25 | 0.4 | 0.337 | 0.61 | 6 | **lagging badly** |
| Round 1 | 0.5 | 0.8 | 0.263 | 0.56 | 6 | still lagging |
| Round 2 | 1.0 | 0.8 | 0.150 | 0.41 | 9 | much tighter, still smooth |
| **Final** | **1.5** | **1.0** | **0.11–0.17** | 0.31–0.49 | 6–13 | **tight + smooth** |
| Probe | 2.0 | 1.0 | 0.135 | 0.37 | 22 | **oscillation onset** |

## The key insight

The original tune (`gain_yaw 0.25`, `max_yaw_rate 0.4`) had **low sign-flips (6)
but high mean error (0.337)**. Low sign-flips means it was _not_ overshooting —
so the problem was never aggressiveness, it was **lag**. The controller
physically could not slew fast enough to keep up with the moving ball.

A back-of-envelope check confirmed it. At the near side of the orbit (~3 m) the
ball's tangential speed is `2πr/T = 2π·3/20 ≈ 0.94 m/s`, giving a line-of-sight
angular rate of `v/d ≈ 0.94/3 ≈ 0.3 rad/s` from the orbit alone — and the yaw
also has to close the standing image error on top of that. Peak demand lands near
**~0.6 rad/s**, but the rate cap was **0.4**. The controller was saturated and
falling behind on every near-side pass.

So the fix was to **raise the saturation limit, not just the gain**. Lifting
`max_yaw_rate` 0.4 → 1.0 (and `gain_yaw` to 1.5) cut mean error ~2.5× while
sign-flips stayed low — smooth, not twitchy.

We then found the ceiling: at `gain_yaw 2.0` the mean error stopped improving
(0.135, no better than 1.5) but **sign-flips jumped to 22** — the classic
overshoot/hunting signature. That put the oscillation onset just above 1.5, so
**1.5 / 1.0 is the sweet spot: as tight as it gets before it starts ringing.**

## Final values (committed in `configs/pi.yaml`, `control_node`)

```yaml
gain_yaw:       1.5    # was 0.25
max_yaw_rate:   1.0    # was 0.4  — this was the binding constraint
max_yaw_accel:  1.2    # was 0.8
deadband_x:     0.05   # was 0.10
```

Result: **mean \|error_x\| ≈ 0.11, peak ≈ 0.31, no oscillation**, and once the
separate distance/size fix landed (see below), **100 % lock across the full
orbit**.

## Two traps found while tuning

1. **The 90 Hz camera stale-frame trap.** Raising the Gazebo camera to 90 Hz to
   "get more fps" quietly broke tracking: the renderer fell behind the lockstep
   sim, so **sensor time advanced at 0.30× wall-clock and frames went ~12.5 s
   stale over an 18 s window** — while `gz stats` still reported RTF 1.0. The
   republisher restamps frames with wall-clock time, which _hides_ the lag from
   everything downstream. We were tuning a controller against a 12-second-old
   ball; error tripled. **Keep the camera at 30 Hz.**

2. **Lock loss at distance was size, not confidence.** The far arc of the orbit
   kept dropping lock. Lowering the confidence floors did nothing (lock stayed
   ~41 %). The real cause was the tracker's `target_area_min`: it rejects any
   detection whose normalized bbox area is too small. At `0.0004` that cuts off a
   0.2 m ball at **~8.2 m**, but the orbit reaches **9 m**. Lowering it to
   `0.0001` (trackable to ~16 m) took visibility **40 % → 100 %**, tracking the
   ball continuously from 3.2 m out to 11.6 m. Lesson: when a target is lost as it
   recedes, check the size gate before the confidence gate.

## Update (2026-07-04): the controller is now full PID

The yaw law gained optional I and D terms (`yaw_pid_step` in
`control_math.py`, unit-tested). **Defaults are `yaw_ki: 0`, `yaw_kd: 0` — the
committed behavior is still exactly the pure-P law tuned above.** All knobs are
runtime-tunable for live sweeps with the same measurement method:

```bash
ros2 param set /control_node yaw_ki 0.1    # trims standing lag on a moving target
ros2 param set /control_node yaw_kd 0.05   # damps overshoot near the gain ceiling
```

Guard rails built in: the integral contribution is capped at `yaw_i_limit`
(0.3 rad/s) and frozen while the command is rate-saturated (anti-windup), so I
cannot reintroduce the saturation lag described above; D runs through a low-pass
(`yaw_d_lpf_alpha`, 0.35) because the raw vision-error derivative is exactly the
jitter amplifier the original design avoided. PID state resets automatically on
target loss/reacquisition and on any yaw-gain change.

## Update 2 (2026-07-04): I and D measured — they don't help this workload

Two live sweeps (10 combos each, drone airborne in `track_center` on the
orbiting ball, 40 s captures) — first with the production guards, then a
**pure-PID** sweep (D low-pass off, I clamp off, rate/slew raised to 3.0/20 so
nothing reshapes the command). Every row landed at mean |error_x| ≈ 0.10–0.11
with exactly 6 sign-flips/min, from ki up to 0.5 and raw kd up to 0.6.

That flat table is *not* a plumbing failure. Regressing the live commanded
`yaw_rate` against the deadbanded error and its derivative recovered
**effective kp = 1.50, kd = 0.600** — precisely the values set. The PID is in
the command; the closed loop just doesn't care, for measurable reasons:

- **The residual error is a 0.05 Hz sinusoid** (the ball's 20 s orbit). The
  6 flips/min in every row is just its 2 zero-crossings per period — a
  geometry constant, not a controller signature.
- **There is no DC error for I to remove**: mean *signed* error ≈ −0.015
  (vs 0.105 mean magnitude). The error is velocity lag, alternating sign with
  the orbit. An integrator only kills DC; on zero-mean AC it contributes
  90°-lagged action (measured: ki rows equal-or-slightly-worse).
- **D has no authority at 0.05 Hz**: D/P magnitude = kd·ω/kp = 13 % at
  kd 0.6 (7° lead). Meaningful lead (45°) needs kd ≈ kp/ω ≈ **4.8** — but
  vision-noise amplification already rings the loop at kd ≈ 1.0 (measured:
  filtered kd 1.0 → mean 0.089, **20 flips/min**). The D benefit curve dead-ends
  an order of magnitude short of usefulness.

So for a continuously *moving* target, the error obeys ≈ LOS_rate / gain_yaw,
and the earlier conclusion stands stronger: **P and the rate cap are the only
effective feedback knobs, and 1.5/1.0 sits just under the ring ceiling.** The
correction to Update 1: `yaw_ki` will *not* trim the ~0.11 standing component —
that lag is AC, not bias. I/D remain worth keeping for the field, where slow
DC disturbances P can't null (wind-induced trim, camera misalignment on the
real airframe) are exactly what I is for.

What would actually cut the remaining error, in order of leverage:
1. **Feed-forward the target's LOS rate** (from the tracker's error slope or
   ball-state estimate) added directly to the yaw-rate command — lead without
   the feedback phase penalty.
2. **Cut loop transport delay** (YOLO is capped at 15 fps; detection→command
   latency is the phase lag that sets the P ring ceiling).

If a future gain hunt pushes past 1.5 and rings (the 22 flips/min regime),
`yaw_kd` buys a little headroom — but per the numbers above, expect ~10 %, not
a new regime.

## How to re-run this

```bash
# with the stack flying a long track_center mission:
cd ~/dronetrack_groundstation_ros
bash scripts/ros_wsl.sh env ros2 topic echo /drone/tracking/target_error \
    --field error_x --csv > /tmp/err.csv        # ~60 s, then Ctrl-C
# lock% = nonzero fraction; mean|err|, peak, sign-flips from the same file.
```

Note: run ROS 2 CLI through `scripts/ros_wsl.sh env …` so it inherits the stack's
DDS environment — a plain `ros2 topic echo` sees the topic as unpublished.
