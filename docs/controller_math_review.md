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
