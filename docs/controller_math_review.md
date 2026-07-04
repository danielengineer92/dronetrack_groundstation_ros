# Controller & Flight-Math Review

**Scope:** Checklist item #2 — *Controller & Flight Math Review*
**Date:** 2026-07-03
**Reviewer:** code review pass (static, no SITL run)
**Verdict:** Math is dimensionally consistent and mostly correct. **One real geometry
bug** in the orbit-center projection (directly the "target directly below vehicle"
edge case), one latent MAVLink trap in dead code, and a **test-coverage gap** on all
the perception→world geometry. None block bench work; F1 should be fixed before the
first orbit flown around a low/ground target.

> **Resolution (2026-07-03):** F1, F2, F3 fixed in code — see §6. Both ⚠️ SITL-verify
> rows in §4 are now **CLOSED** — verified in a live PX4 SITL + Gazebo run (see §7).
> This doc is kept as the record of what was found, fixed, and verified. 

> ⚠️ **UPDATE (2026-07-04) — §7's verification was not trustworthy.** §7 used
> *injected* detections in a 960×720 frame. The **real camera renders at 1280×960**,
> and a live, vision-driven, ground-truth-anchored re-run found **two bugs the static
> review and the injected-detection run both missed**: the tracker's `bearing_x`/
> `bearing_y` were computed with the wrong image resolution (**wrong sign**, ~20° off),
> and the distance estimate read a systematic **−32%**. Both feed the F1 orbit
> projection, so the "verified" orbit geometry was actually being fed corrupted inputs
> in the real pipeline. **Both are now fixed and re-verified against gz ground truth —
> see §8 (F4/F5).**

---

## 1. Where the math lives

| Concern | File | Lines |
|---|---|---|
| Forward/approach P-controller (distance → velocity) | `drone_control/control_math.py` | `approach_forward_velocity` 18–53 |
| Yaw P-control (image error → yaw rate) | `drone_control/control_node.py` | 900–901 |
| Deadband rescale | `drone_control/control_node.py` | 442–448 (guard 274–277) |
| Velocity/accel limiting, altitude floor | `drone_control/control_node.py` | 632–659 |
| Yaw integration + wrap | `drone_control/control_node.py` | 466–467, 535–545 |
| Monocular distance estimate (pinhole) | `drone_tracker/tracker_node.py` | 450–501 |
| Pixel → bearing | `drone_tracker/tracker_node.py` | 480–481 |
| Image-error normalization | `drone_tracker/tracker_node.py` | 572–579 |
| **Target → global lat/lon projection (orbit center)** | `drone_control/mission_executor_node.py` | 733–755 |
| GoTo offset projection | `drone_control/mission_executor_node.py` | 1096–1134 |
| DO_ORBIT to PX4 (live path, COMMAND_INT) | `drone_telemetry/telemetry_node.py` | 837–869 |
| DO_ORBIT MavlinkDirect (unused, COMMAND_LONG) | `drone_telemetry/telemetry_node.py` | 880–941 |

The controller itself does **not** compute the orbit; it holds position + yaw and
delegates the circle to PX4's `MAV_CMD_DO_ORBIT`. So the flight-critical geometry is
the **orbit-center projection** in `mission_executor_node.py`, not the control loop.

---

## 2. Findings (ranked)

### F1 — Orbit-center projection ignores camera elevation (`bearing_y`)  ·  **Medium-High · fix before orbiting a low target**

`estimate_target_global_center()` (`mission_executor_node.py:744-749`):

```python
distance_m   = float(self.last_target.distance_m)      # SLANT range (pinhole)
bearing_x    = float(self.last_target.bearing_x_rad)   # horizontal only
global_bearing = yaw + bearing_x
north_m = distance_m * math.cos(global_bearing)
east_m  = distance_m * math.sin(global_bearing)
```

`distance_m` is the **slant range** from the monocular estimator, but it is used as
the **horizontal ground distance**. The vertical bearing (`bearing_y_rad`, which the
tracker already computes at `tracker_node.py:481`) is never applied. The correct
horizontal component is:

```
horizontal_m = distance_m * cos(bearing_y_rad)
```

**Failure scenario:** drone at 3.2 m looking down at the ball at ~1 m altitude. The
depression angle is real (tens of degrees at close range), so the projected center is
pushed **outward** by `1/cos(elevation)` — the orbit ring is placed past the ball, in
the heading direction. In the limit "target directly below the vehicle" (the exact
edge case in the checklist), the true horizontal offset is ~0 but the formula emits
the full slant distance. The median-over-samples logic (`orbit_center_min_samples`)
does **not** save this: every sample carries the same systematic bias, so the median
is biased too.

**Fix:** multiply the horizontal distance by `cos(bearing_y_rad)` (guard tiny values),
and — since the ball altitude is then known — optionally set the orbit-center altitude
from the target rather than the drone. Add a unit test with a known depression angle.

---

### F2 — Dead `_send_do_orbit_mavlink_direct` builds a COMMAND_LONG  ·  **Low · latent trap**

`telemetry_node.py:922-929` constructs `MavlinkMessage('COMMAND_LONG', …)` with
lat/lon in `param5/param6` as float32. The sibling live method's own docstring
(`:837-848`) documents the lesson learned the hard way:

> "PX4's orbit handler expects the global center in a COMMAND_INT, so it silently
> never entered [orbit] … If a hard revolution count is ever needed again, send a
> COMMAND_INT (not _LONG)."

`_send_do_orbit_mavlink_direct` is currently **not wired into the dispatch**
(`_execute_action` only calls `_send_do_orbit_action`), so this is latent. But it is a
booby-trap: the day someone enables the revolutions path for a bounded orbit, it will
(a) silently fail to enter ORBIT for the documented COMMAND_INT reason, and (b) even if
accepted, float32 lat/lon loses ~1–5 m of precision on the orbit center.

**Fix:** delete the method, or convert it to COMMAND_INT with int32-scaled
(`round(deg * 1e7)`) lat/lon before it's ever dispatched.

---

### F3 — Zero unit tests on the perception→world geometry  ·  **Medium · process**

`test_control_math.py` covers only `approach_forward_velocity` + `clamp`. **Untested:**
distance estimate (`update_target_geometry`), bearing, image-error normalization, and
the `estimate_target_global_center` projection — i.e. every function in F1's path.
These are pure functions (no ROS I/O once refactored) and are exactly the "unit
consistency audit across the control stack" the checklist calls for. Recommend
extracting them into a `perception_math.py` alongside `control_math.py` and adding
table-driven tests (known object at known distance/angle → expected range/bearing/NE).

---

## 3. Unit-consistency audit

| Quantity | Convention | Consistent? | Notes |
|---|---|---|---|
| Angles (internal) | radians | ✅ | telemetry converts PX4 deg→rad at ingest (`telemetry_node.py:523-525`); deg only re-appears at the MAVSDK boundary (`yaw_rad_to_deg_0_360`, `control_node.py:474`) |
| Yaw rate | rad/s | ✅ | `gain_yaw`·error → rad/s, integrated with `dt` → rad (`update_yaw_target`), clamped by `max_yaw_rate` (rad/s) and `max_yaw_accel` (rad/s²·period) |
| Heading frame | NED, 0=N, +CW→E | ✅ | `north=d·cos θ, east=d·sin θ` matches; `bearing_x` (+right) adds to yaw correctly **for a forward-facing camera** |
| Linear velocity | m/s | ✅ | body-frame fwd/right/down and orbit speed all m/s; accel limit = `max_accel·control_period` (correct Δ/tick) |
| Distance | m | ✅ | pinhole `Z = f·D/d_px` correct; `ball_diameter_m=0.20` matches SDF sphere radius 0.1 → diameter 0.2 (no factor-2 error) |
| Image error | normalized [−1,1] | ✅ | `(center−0.5)·2`; deadband rescale continuous and guarded to `[0,1)` |
| Focal length | px, from FOV | ✅ | `f = W / (2·tan(FOV/2))`, FOV clamped to (1°,179°) |
| Earth model | equirectangular, R=6.378e6 | ✅ (for scale) | flat-earth approx fine at orbit-radius scale; `cos(lat)` floored to avoid pole blow-up |

**No unit inconsistencies found.** The one geometry defect (F1) is a *missing term*,
not a units mismatch.

---

## 4. Edge-case status (checklist item #2)

| Edge case | Status | Evidence |
|---|---|---|
| Target directly below vehicle | ✅ **Fixed (F1) + SITL-verified** | node-level: 89° → 0.087 m horizontal (was full slant); in-sim orbit center matches F1 to 0.02–0.07 m (§7) |
| Moving target | ⚠️ Partially | node math handles any bearing; in-sim run used a **still** ball. Orbit center = median of pre-send samples then frozen; PX4 orbits the fixed center (a target moving *during* the orbit is not re-centered — likely acceptable). Circling-ball run not yet driven. |
| Target-loss event | ✅ Looks correct | control node → position-hold on stale target (`control_node.py:882-897`); tracker coast/expire ladder (`tracker_node.py:512-545`); approach velocity zeroes on invalid/stale/non-finite distance (`control_math.py:40-45`) |
| Yaw sign closes the loop | ✅ **SITL-verified** | live: ball held right (error_x=+0.250) → `yaw_rate=+0.296 rad/s`, turns toward target (§7) |

---

## 5. Recommended next steps (to close checklist #2)

1. **Fix F1** — apply `cos(bearing_y_rad)` to the horizontal distance in
   `estimate_target_global_center` (and mirror it in any goto/orbit center path).
2. **Fix/remove F2** — delete the dead COMMAND_LONG orbit method or convert to
   COMMAND_INT so it can't be re-enabled as a silent failure.
3. **Close F3** — extract perception geometry into a testable pure-Python module and
   add table-driven unit tests (this *is* the "unit consistency audit" deliverable).
4. **SITL-verify** the two ⚠️ rows: yaw-sign closes the loop, and moving-target /
   directly-below behavior after F1 lands.

Items 1–3 are code changes reviewable without hardware; item 4 needs the Gazebo SITL
already validated in checklist #1.

---

## 6. Fixes applied (2026-07-03)

| # | Change | Files |
|---|---|---|
| F1 | Added pure `project_target_global()` that foreshortens the slant range by `cos(bearing_y_rad)`; `estimate_target_global_center` now calls it (passing `bearing_y_rad`) | `control_math.py`, `mission_executor_node.py` |
| F2 | Removed dead `_send_do_orbit_mavlink_direct` (COMMAND_LONG) + its exclusive helpers/constants and the now-unused `json` import; left a breadcrumb comment on how to do a COMMAND_INT bounded orbit if ever needed. Live `action.do_orbit` (COMMAND_INT) path untouched | `telemetry_node.py` |
| F3 | New `test_project_target_global.py` — 6 tests covering heading composition, lat/lon offset, elevation foreshortening, and the directly-below edge case | `drone_control/test/` |

**Test status:** `test_project_target_global.py` 6/6 pass, `test_control_math.py`
7/7 still pass. All three edited modules byte-compile.

**Note on scope:** F1's fix corrects the *horizontal* orbit-center placement. The
orbit-center **altitude** is still taken as the vehicle's own absolute altitude
(`estimate_target_global_center` returns drone `alt`), which is the intended behavior
(orbit at flight altitude above the ball) — not changed. If you later want the ring
centered on the ball's true altitude, that's a separate, deliberate change.

---

## 7. Live SITL verification (2026-07-03)

Verified in a real **PX4 SITL (`gz_x500_mono_cam`) + Gazebo Harmonic + full ROS 2
node stack** run on the RTX 5080 box, brought up with the repo helpers
(`scripts/ros_wsl.sh gazebo`, `docker/px4_headless.sh` technique for PX4).

**Why detections were injected.** Headless Gazebo does not render the camera sensor
(a GL/offscreen-render limitation noted in `docs/gazebo_sitl.md`), so the camera topic
carries no frames. The math under review is *not* YOLO — it is the tracker geometry,
the F1 projection, and the control yaw sign. Those were exercised by publishing a
synthetic `DetectionArray` at the tracker input (`/drone/vision/detections`) in the
tracker's real 960×720 / HFOV-99.7° / `distance_calibration_k=100` frame. Everything
downstream — tracker → mission projection → control → PX4 `DO_ORBIT` — is the real,
unmodified code path with the drone's real SITL telemetry. Harness:
`tmp/inject_and_capture.py` (not committed; a test fixture).

The mission ran a **full autonomous sequence to completion**: takeoff → scan → lock →
track_center → approach → **orbit (PX4 `MAV_CMD_DO_ORBIT`)** → RTL → land.

| Check | Result | Detail |
|---|---|---|
| **Tracker geometry** | ✅ matches independent recompute | injected ball ⌀45 px → `distance_m=2.222` (=`k/diam`=100/45); off-center pixel → `bearing_x`, `bearing_y` match `atan((px−center)/f)` to 0.01° |
| **F1 orbit center** | ✅ **0.017 m & 0.074 m** (two runs) | emitted `DO_ORBIT` center vs `project_target_global()` with `cos(bearing_y)`, at `bearing_y≈+19°`. Pre-fix (no-cos) formula would differ by **0.12–0.14 m** — the elevation correction is demonstrably active end-to-end |
| **Yaw sign** | ✅ **PASS** | ball held right, `error_x=+0.250` → control `yaw_rate=+0.296 rad/s` (30 samples): +error → +yaw, turns toward the target |
| **End-to-end orbit** | ✅ | PX4 accepted `DO_ORBIT` (r=2.0 m, v=0.40 m/s) and flew it; full mission reached `land` |

**Still open (not blocking):** (a) camera never rendered headless — a *moving-ball,
fully vision-driven* run (YOLO in the loop) still needs either a working GL/offscreen
render or a display-attached Gazebo; (b) the in-sim run used a **still** ball, so the
"target moving *during* the orbit" case is not yet exercised (`ball_motion:=circle`
would drive it). Neither affects the math verified above.

---

## 8. Live vision-driven re-verification & two new bugs (2026-07-04)

The §7 run injected a synthetic `DetectionArray` in the tracker's **960×720** frame
because the headless camera wasn't rendering. That made the *config* (960×720) match
the *injected* pixels by construction — so any bug tied to the real camera resolution
was invisible. With the camera pipeline now fully live (real Gazebo render → YOLO →
tracker), the geometry was re-checked against **gz ground truth** (drone vs `red_ball`
`/world/default/dynamic_pose/info` poses). Two real bugs surfaced.

### F4 — Tracker bearings computed at the wrong resolution · **High · corrupts orbit heading**

The live camera renders **1280×960**, but `configs/pi.yaml` set `image_width_px: 960`,
`image_height_px: 720` (a stale comment even claimed "must match … 960x720"). YOLO
fills `pixel_center_x/y` in **actual** pixels (0–1280) *and* a correctly-normalized
`center_x/y`. The tracker (`update_target_geometry`) forms bearings from the **actual
pixels** against the **configured** principal point and focal length:

```
bearing_x = atan((pixel_center_x − image_width_px/2) / fx),  fx = image_width_px/(2·tan(HFOV/2))
```

With `image_width_px=960`, the principal point sits at 480 while the true image centre
is 640 — so the sign flips and the magnitude inflates. **Measured live** (ball truly
6.2° *left* of centre): tracker emitted **+14.4°** (another sample, +32.6°) — wrong
sign, ~2× magnitude. The **yaw loop was unaffected**: it uses `detection.center_x`
(correctly normalized to 1280), which is why lock and the §7/PID tuning all looked
fine — but `bearing_x/bearing_y` feed the **F1 orbit-centre projection**, so F1 was
projecting the orbit centre in a *wrong heading* in the real pipeline.

*Root cause:* config resolution ≠ camera resolution. *Fix:* `image_width_px: 1280`,
`image_height_px: 960` in `pi.yaml` (principal point → 640/480, fx=fy=539). No code
change — the bearing math itself was correct; it was fed the wrong intrinsics.

### F5 — Distance calibration read 32% low on the real camera · **Medium-High · corrupts orbit range & approach**

`distance_calibration_k = 100` (`distance = k / diameter_px`) was hand-tuned for the
old 960-wide assumption (fx=405). On the real 1280-wide camera (fx=539) it reads a
consistent **~32% short**. Fit against **19 ground-truth samples** across the orbit
(true slant 3.3–9 m): `k = diameter_px · true_slant` clusters at **mean 146.6** (range
134–164), vs the configured 100. (Pinhole predicts fx·D = 539·0.2 = 108; YOLO boxes
run ~1.36× fat, giving ~147.) *Fix:* `distance_calibration_k: 147.0`. Distance feeds
the F1 slant range and the approach P-controller, so a 32% bias mis-sizes the orbit
ring and makes the approach stop short.

### What was actually re-verified (live, vs gz ground truth)

| Check | Before fix | After fix (F4+F5) | Ground truth |
|---|---|---|---|
| `bearing_x` sign & magnitude | +14.4° (ball at −6.2°) — **wrong sign** | matches to **≤0.1°** across −30°…+30° | normalized-centre + FOV |
| `bearing_y` | wrong sign, ~+10° | matches to **≤0.1°** | — |
| `distance_m` bias | **−32%** (systematic) | **±few %**, no bias (±10% noise at range extremes) | gz drone-vs-ball slant |
| Yaw tracking loop | ✅ already correct | ✅ unchanged | uses normalized `center_x`, never touched by F4/F5 |

### Corrected status of earlier claims

- **§3 unit-audit row "pinhole `Z=f·D/d_px` correct … no factor-2 error"** — the
  *formula* is correct, but the *inputs* (fx via `image_width_px`, and k) were wrong for
  the real camera. Units were never the problem; **resolution calibration was.**
- **§7 "F1 orbit centre 0.017 m / 0.074 m"** — that agreement was between two
  computations *sharing the same injected 960×720 bearings*, so it validated the
  arithmetic of `project_target_global`, **not** the real bearing inputs. F1's `cos`
  math remains correct; it was being fed corrupted `bearing_x/y` (F4) and slant (F5) in
  the live pipeline. With F4/F5 fixed, F1 now operates on ground-truth-correct inputs.

### Fixes applied (2026-07-04)

| # | Change | File |
|---|---|---|
| F4 | `image_width_px 960→1280`, `image_height_px 720→960` (+ corrected comment) | `configs/pi.yaml` |
| F5 | `distance_calibration_k 100→147` (empirical fit to 19 gz-ground-truth samples) | `configs/pi.yaml` |

**Still open / recommended (not applied):**

1. **Make the tracker self-consistent instead of config-pinned.** It already receives
   the true resolution on every `DetectionArray` (`image_width`/`image_height`) and a
   correctly-normalized `center_x/y`. Deriving bearings from the normalized centre + FOV
   (or from the per-message resolution) would make F4 structurally impossible. Right now
   a camera-resolution change silently reintroduces it.
2. **Add a runtime parameter callback** for `image_width_px/height_px` — `ros2 param
   set` currently reports success but the tracker keeps its init-time values (confirmed
   live), a foot-gun for calibration.
3. **Vehicle pitch in the projection.** `project_target_global` assumes a horizontal
   optical axis (camera mount is `0 0 0`, true at hover). In forward flight the airframe
   (and the rigidly-mounted camera) pitches down, adding an un-modelled elevation term
   to `bearing_y`. Fine for position-hold `TRACK_CENTER`; revisit if orbit/approach ever
   runs while pitched.
4. **F5's k is model-specific** (the 1.36× fat factor is a `yolo11s` property). Re-fit k
   if the YOLO model or ball size changes; better, prefer the physical
   `ball_diameter_m·fx/diam` path with a single fat-factor multiplier.

---

## 9. Delay-aware motion profiling & acquisition math (2026-07-04)

Live circling-ball runs (the §7/§8 "moving target during orbit / approach" gaps, now
driven with `ball_motion:=circle`) exposed three *dynamics* problems the earlier
*geometry* review could not see, because they only appear against a target that is
moving and/or being closed on:

1. **Yaw over-shoots and swings** when acquiring a target far off-bearing.
2. **The drone lunges at / launches past the target** on approach and on orbit entry.
3. **First-lock tracking is poor** — trails a moving ball, and the feed-forward *spikes*
   the yaw right as the loop crosses centre.

All three are the same root cause: a proportional law reacting to a **delayed**
measurement (vision pipeline ≈ 0.35 s) closes at full commanded rate and cannot brake
in time. The fix is one piece of math applied on every closing axis.

### 9.1 Trapezoidal braking speed limit (`braking_speed_limit`)

Given a remaining error `Δ` (rad for yaw, m for range), a measurement dead time `τ`
during which we keep moving blind, and a deceleration `a` we are willing to apply, the
distance covered before stopping from speed `v` is the dead-time run plus the ramp-down:

```
Δ = v·τ  +  v² / (2a)
```

Solving the quadratic for the largest `v` that still stops exactly on the target:

```
v(Δ) = −a·τ + √( (a·τ)² + 2a·|Δ| )
```

with the two asymptotics that make it correct across the whole range:

| Regime | Limit | Behaviour |
|---|---|---|
| Small `Δ` (near goal) | `v → Δ/τ` | **dead-time limited** — never command more than the delay can arrest |
| Large `Δ` (far) | `v → √(2a·Δ)` | **decel limited** — classic stopping distance |

The result is a non-negative *magnitude* clamped to `[min_speed, max_speed]`; callers
apply it as a symmetric clamp on their command, so it only ever **reduces** an
over-eager closing rate. It is **dimension-agnostic** — the same function caps yaw rate
(rad/s over a bearing `Δ`) and forward speed (m/s over a distance `Δ`). A `min_speed`
floor preserves authority just outside the deadband (critical for yaw — see §9.3).

Pure fn: `control_math.braking_speed_limit`. Tests: `test_yaw_pid.py`
(`test_approach_cap_*`, 8 cases: monotonicity, both asymptotes, ceiling, floor).

**Measured (closed-loop model, real `control_math` fns + 0.35 s transport delay + yaw
plant), far-off acquisition at 30° bearing:**

| | raw P slew | + braking cap |
|---|---|---|
| Peak overshoot past target | **20.5°** | **5.8°** (−72%) |
| Settle to <3° | 5.0 s | **2.25 s** |
| Peak yaw-rate cmd | 1.00 rad/s | 0.66 rad/s |

Applied to yaw it is the anti-swing limiter (`yaw_approach_*` params); applied to the
approach P-law (`approach_forward_velocity`, `approach_*` params) it brakes into the
standoff distance instead of lunging past it.

### 9.2 Alignment gate (`alignment_scale`)

Translating toward a target whose *bearing* has not yet converged drives the vehicle in
the wrong direction — the literal "launch at the target": full forward speed while the
yaw loop is still swinging. Gate any forward/closing command by how centred the target
is:

```
scale(error_x) = clamp( 1 − |error_x| / L , 0 , 1 )      (L = align error limit; L≤0 disables)
```

`scale = 1` centred → `0` at `|error_x| ≥ L`. Two consumers:

- **Approach closing speed** (`approach_forward_velocity`): closing (positive) speed is
  multiplied by `scale`; **backing off is never gated** (increasing separation is safe
  regardless of centring). So the drone centres first, then closes.
- **Yaw feed-forward** (§9.3).

Pure fn: `control_math.alignment_scale`. Tests: `test_control_math.py`
(`test_alignment_*`, incl. the never-blocks-backoff and disabled-by-zero cases).

### 9.3 Feed-forward is a *steady-state* term — gate it during acquisition

The yaw feed-forward commands the target's inertial LOS rate so the loop stops trailing
a moving target (`yaw_ff_source: "state"` reads the KF's predicted relative velocity;
latency-cancelled). But on a **fresh lock**, the state KF sees the re-acquisition
position jump as a huge velocity and emits a spurious LOS rate. **Measured live:** as the
feedback crossed centre (`error_x` −0.08), the FF drove `yaw_rate` to the **−1.0 rad/s
saturation** and swung the ball back to **+0.92** — a self-inflicted swing *after* the
reel-in had succeeded.

Fix: FF carries an **already-centred** target, so scale it by the §9.2 alignment factor
(`yaw_ff_align_error_limit`). FF is off during acquisition (large `error_x`, unreliable
estimate) and ramps to full as the target centres. The feedback + §9.1 cap own the
reel-in; FF owns steady tracking. This also required raising the yaw cap **floor**
(`yaw_approach_min_rate` 0.15→0.35): with FF gated off at acquisition, the feedback must
not be starved, or it cannot keep up with a moving ball while centring.

**Measured live (circling ball ≈0.9 m/s, forced re-acquisition, `track_center`):**

| | before | after (this section) |
|---|---|---|
| Steady-state mean \|error_x\| | 0.13 – 0.75 | **≈0.05** |
| Lock stability | 0.7–1.3 s micro-locks (constant loss) | **sustained 12–58 s locks** |
| Centre-crossing yaw | **saturates ±1.0, swings to +0.9** | smooth, no saturation |
| Reel-in from frame edge | — | one controlled overshoot, centred ≈4 s |

Supporting tuning (all in `pi.yaml`, documented inline): tracker `smoothing_alpha`
0.35→0.5 (halves EMA lag at ~12 fps YOLO), and — outside the control loop — the mission
`prime_offboard` keeps yaw on a locked target instead of freezing (a moving ball no
longer walks out of frame during the prime window) and the scan sweep 20→40 °/s out-runs
a moving target's LOS rate (~19 °/s worst case) instead of tail-chasing it.

### 9.4 Orbit entry envelope (anti launch-at-target)

PX4's `MAV_CMD_DO_ORBIT` captures the ring by flying **straight at the centre** from
wherever the vehicle is. Handing it an orbit from far away therefore *dashes the drone
at the target* — the geometry is correct (§F1/§8) but the entry trajectory is not. Gate
the hand-off on range:

```
send DO_ORBIT  ⇔  distance_valid  ∧  distance ≤ radius·factor + slack
```

Outside the envelope the orbit step keeps commanding the §9.1/§9.2 profiled approach;
the step timeout is the backstop. `orbit_entry_distance_factor` (1.75), `orbit_entry_slack_m`
(0.5) in `mission_executor_node`. **Measured live:** DO_ORBIT handed over at ≈1.9 m for a
2 m ring (drone already on the ring), zero dash — vs the earlier full-speed run across
the map at the ball.

### 9.5 Unit consistency (new math)

| Quantity | Convention | Consistent? | Notes |
|---|---|---|---|
| `braking_speed_limit` (yaw) | Δ rad, τ s, a rad/s² → v rad/s | ✅ | dimensionless `a·τ` and `2a·Δ` both (rad/s)² under the root |
| `braking_speed_limit` (approach) | Δ m, τ s, a m/s² → v m/s | ✅ | same form, m instead of rad |
| `alignment_scale` | normalized `error_x` [−1,1] ÷ limit | ✅ | dimensionless [0,1]; FF and closing speed both dimensionless-scaled |
| FF gate | `scale · gain · LOS_rate` | ✅ | LOS rate rad/s, scale [0,1], gain [-] → rad/s |
| orbit entry | `radius·factor + slack` | ✅ | all metres; `factor` dimensionless |

No unit inconsistencies. All new closing-axis math shares the one profiling law (§9.1),
so the yaw and translation behaviours are guaranteed consistent by construction.

### Fixes applied (2026-07-04, §9)

| # | Change | File |
|---|---|---|
| C1 | `braking_speed_limit` (generalized from the yaw-only limiter) + `alignment_scale` | `control_math.py` |
| C2 | `approach_forward_velocity`: braking profile + alignment gate on closing speed | `control_math.py` |
| C3 | Yaw FF alignment-gated; approach-cap floor 0.15→0.35 | `control_node.py` |
| C4 | Orbit entry envelope before `DO_ORBIT`; `prime_offboard` tracks locked target; OFFBOARD arm recovery; scan 20→40 °/s | `mission_executor_node.py` |
| C5 | VELOCITY approval matches `SENT` status **prefix** | `telemetry_node.py` |
| C6 | FF source `state`, `smoothing_alpha` 0.5, `yaw_ff_align_error_limit`, approach + orbit-entry params | `configs/pi.yaml` |

**Test status:** `test_control_math.py` 15/15, `test_yaw_pid.py` (incl. 8 braking-cap
cases) OK, full `drone_control` suite green (5+15+26+6 + yaw/lead). 23 new pure-math
cases across the braking profile, alignment, and approach.

## 10. Steady-tracking wobble: pose-pairing bug + PD damping (2026-07-04)

After §9, tracking of the circling ball was good on average (mean |error_x| ≈ 0.05)
but the drone **hunted side-to-side (~1–2 s period)** whenever the ball passed its
**nearest and farthest** orbit points — exactly where the ball's LOS angular rate
peaks (motion purely tangential there). The yaw law is now explicitly:

```
yaw_rate = PD(image error) + FF(KF LOS rate)
```

with `gain_yaw` (P), `yaw_kd` (D), `yaw_ki` (I, kept 0) all runtime-tunable.

### 10.1 Root cause: stale bearing paired with fresh yaw (the dominant one)

The state estimator projected each detection to world NED using the **latest**
telemetry pose — but the bearing in that detection is ~0.35 s old. While yawing at
rate ω, the projection error is ω·0.35 in LOS angle (~0.1 rad at 0.3 rad/s —
0.6–0.9 m of phantom cross-range at 3–9 m). That error oscillates **with our own
yaw motion**, corrupts the KF velocity, and returns through the yaw feed-forward:
a self-excited loop that peaks exactly at the LOS-rate extremes.

**Closed-loop model** (real `control_math` + real `ConstantVelocityKF`, 12 fps ZOH
vision, 0.35 s transport delay, EMA 0.5, 20 Hz control, PX4 lag; orbiting ball
r=3 m T=20 s from 6 m):

| | mean\|error_x\| | sign-flips/min | p2p error @ LOS-rate peaks |
|---|---|---|---|
| shipped pairing (latest yaw) | 0.109 | 32.0 | **0.407** |
| kd=0.3 alone (pairing unfixed) | 0.110 | 27.2 | 0.397 |
| **pairing fixed** | **0.056** | **5.2** | **0.062** (−85%) |

PID tuning barely moves the wobble while the contamination is present — the fix is
structural, not a gain. Fix: the tracker now publishes the **camera capture stamp**
in `TargetError.stamp` (propagated from the Pi camera / gz republisher through
YOLO's `Detection.stamp`; no consumer read the old publish-time stamp), and the
estimator keeps a ~2 s pose history, **interpolating the drone pose (N, E, yaw —
wrap-aware) at the detection's capture stamp** before projecting. Falls back to
the latest telemetry when the stamp can't be resolved (counted as
`pair_fallback` in the node log; `paired@capture` counts the good path).
Runtime A/B: `ros2 param set /target_state_estimator_node pair_at_capture_stamp false`.
Verified live: 454/454 measurements paired, 0 fallbacks.

### 10.2 D term: the damping nothing else provides

Does the §9 tuning already do what a PID would? **No.** The braking cap (§9.1) is
an acquisition-phase *magnitude* limiter — during centred tracking the P output
(~0.1–0.2 rad/s) is far below the cap's own `min_rate` floor (0.35), so the clamp
is inert and contributes **zero damping**. The FF does the *I-term's* job (carries
the standing rate so feedback holds near-zero error without integrator lag). But
nothing injected output ∝ error *slope* — phase lead near crossover. The loop was
pure P (K ≈ kp/half_fov ≈ 2.46 s⁻¹) through ~0.5–0.6 s of effective delay:
K·L ≈ 1.2–1.5 vs the π/2 pure-delay instability threshold — thin margin, lightly
damped ringing at ~4·delay ≈ 1.4–2.4 s, matching the observed wobble period.

Enabled `yaw_kd: 0.3` (Td = 0.2 s → ≈ +13° net phase margin after the D-LPF's own
lag). Offline: neutral at T=20 s once pairing is fixed, **−20% peak wobble at
T=12 s**, mean|error_x| best of all arms. D noise at 12 fps ZOH quantified ≈
0.04 rad/s command jitter at kd=0.3 with `yaw_d_lpf_alpha` 0.35 — below the slew
step (0.06 rad/s per tick). ki stays 0: I adds phase lag (the wrong direction for
wobble) and the FF already carries the standing rate.

**D runs on the RAW pre-deadband error** (`yaw_pid_step(derivative_error=...)`,
new kwarg; P and I keep the deadbanded error). The deadband rescale zeroes the
slope inside the band and kinks it at the boundary — precisely where a centred
loop lives — so D on the deadbanded error is blind inside the band and sees
phantom discontinuities at every crossing. Inert at kd=0 (hardware-safe).

### 10.3 Knobs added but left OFF (offline A/B said unnecessary once 10.1 landed)

- `yaw_ff_state_lpf_alpha` (1.0 = off): low-pass on the state-FF branch,
  symmetric with the `los_diff` branch's filter. The state LOS rate divides by
  range² (noisiest at the nearest point) — but with capture-stamp pairing the
  estimate is clean and the filter only added ~90 ms lag for no p2p gain.
- `yaw_ff_align_inner` (0.0 = original ramp): flat zone of the FF alignment gate
  (`alignment_scale(inner=...)`) — full FF for |error_x| ≤ inner. Removes the
  gate's in-band modulation (slope 1/0.6 ≈ 1.67 couples the error wobble into the
  FF as sign-switching gain, up to +50% effective loop gain at FF ≈ 0.3–0.5 rad/s).
  Not needed in sim after 10.1; live knob for hardware where delays are larger.

### 10.4 Verification & tuning assets

- Offline harness: `orbit_ab.py` (job tmp) — real `control_math` + real KF through
  the full sensing chain; metrics: mean|error_x|, sign-flips/min, p2p error in
  ±2 s windows at the LOS-rate extremes, FF-vs-truth phase lag. Reproduces the
  wobble with the old pairing and its disappearance with the fix.
- Unit tests: `test_yaw_pid.py` +4 (derivative_error semantics incl.
  alive-inside-deadband), `test_control_math.py` +4 (alignment knee, incl.
  inner=0 regression-identity). All suites green.
- Live smoke: 16 nodes up, new params live, 454/454 paired@capture, LOCKED.

| # | Change | File |
|---|---|---|
| D1 | `TargetError.stamp` = camera capture stamp of the current target | `tracker_node.py` |
| D2 | Pose history + capture-stamp pairing (`pair_at_capture_stamp`) | `target_state_estimator_node.py` |
| D3 | `yaw_pid_step(derivative_error=, prev_derivative_error=)`; `alignment_scale(inner=)` | `control_math.py` |
| D4 | D fed raw error; `yaw_ff_state_lpf_alpha`; `yaw_ff_align_inner` | `control_node.py` |
| D5 | `yaw_kd: 0.3`, `pair_at_capture_stamp: true`, new knobs documented | `configs/pi.yaml` |

Gain tuning (kp/kd/ki sweep against live SITL data) is the designated follow-up.

## 11. Live SITL gain tuning (2026-07-04)

Staged tuning against live SITL data (P → D → FF → clamps → I), each term isolated
before the next was added. Full method, 54 recorded runs, sweep tables and
oscilloscope traces are in the tuning artifact; the outcome:

| term | shipped | **SITL candidate** | why |
|---|---|---|---|
| `gain_yaw` (P) | 1.5 | **2.0** | pure-P sweep knee; lag 0.13→0.094; kp=3.0 unstable |
| `yaw_kd` (D) | 0 | **0.3** | phase-lead margin; kd≥0.7 destabilises |
| `yaw_ff_gain` | 1.0 | **0.8** | full FF over-drives the near pass (1/r² LOS spike → saturation); 0.8 keeps nominal optimal |
| `yaw_ki` (I) | 0 | **0** | near≈far error (no standing bias); FF carries the rate |
| `max_yaw_accel` | 1.2 | **1.2** | raising it made hunting WORSE — the slew limit is load-bearing damping |

Key findings: (1) at the nominal 20 s orbit the loop was already fine in every config —
the wobble is a **high-LOS-rate** phenomenon that only surfaces on a fast (10 s) target;
(2) the near-pass residual is bounded by **phase margin, not actuation** (more yaw accel
= worse); (3) low-pass filtering the FF made it worse (adds phase lag that mistimes the
1/r² spike) — the clean lever is the FF gain, not a filter.

**Status: SITL candidate, NOT flight-validated.**

## 12. Hardware bring-up plan (before first real flight)

The §11 gains live in the shared `configs/pi.yaml`, so they also load on the aircraft.
They are SITL-only. Real vision latency and yaw dynamics differ — bring the loop up
conservatively, not at the SITL candidate values:

1. **Instrument.** `ros2 param set /control_node log_yaw_terms true` — logs, at ~2 Hz
   while tracking: `P / D / FF / I` terms, braking `cap`, `raw` (pre-limit) command,
   `clamped` command actually sent, and `achieved` yaw rate (differentiated telemetry).
2. **Start low, P/D minimal.** `gain_yaw ≈ 1.0`, `yaw_kd ≈ 0.1`, `yaw_ff_gain ≈ 0.4`,
   `yaw_ki = 0`.
3. **Confirm FF sign first.** On a moving, off-centre target the **FF term must share the
   P term's sign** (both drive yaw the same way). If FF opposes P, the LOS-rate sign or a
   frame convention is wrong — stop and fix before raising any gain. (SITL preview,
   verified 2026-07-04: `err=+0.048 → P=+0.097, FF=+0.145`; `err=−0.033 → P=−0.067,
   FF=−0.065` — same sign, FF assists.)
4. **Raise P**, then **FF**, toward the SITL candidates while watching `raw` vs `clamped`
   (how hard the limiter clips) and `achieved` (does the airframe follow the command).
5. **Add D last, and judge it.** Watch whether the `D` term is coherent on transients
   (real damping) or just sign-flipping noise at centre (in SITL it is already noticeably
   noisy near centre) — back it off if it is feeding jitter into the command.
6. **Keep `yaw_ki = 0`** unless a *measured* standing bias appears (near vs far error
   asymmetric); I only adds phase lag against a moving target.

Instrumentation: `log_yaw_terms` (`control_node.py`, off by default); the `_log_yaw_terms`
line is the single source for all six signals above.
