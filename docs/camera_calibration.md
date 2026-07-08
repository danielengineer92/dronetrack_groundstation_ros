# Camera Intrinsic Calibration (IMX296 + 120° wide lens)

Why: the tracker used to derive fx/fy from a hand-guessed FOV and assume a
distortion-free pinhole centered on the sensor. A 120° lens has strong barrel
distortion, so bearings away from frame center were systematically wrong —
biasing yaw tracking, the target-state estimator's NED projection, and
orbit_fixed center fixes. Calibration measures the real camera matrix
(fx, fy, cx, cy) + distortion coefficients; the tracker then undistorts each
detection center before computing bearings (`drone_tracker/camera_geometry.py`).

Distance is unaffected: the `distance_calibration_k` path is empirical and
independent of fx. The sim is unaffected: the gz camera is a true pinhole and
keeps the legacy FOV path (do NOT paste intrinsics into `configs/pi.yaml`).

## 1. The target

**Daniel's board (2026-07): plain checkerboard, 16×10 squares, 15.55 mm,
BLUE/white.** That means:
- `--pattern 15x9` — OpenCV counts **inner corners** (squares − 1 per side).
  Orientation (15x9 vs 9x15) just needs to match how the detector sees the
  grid; the scripts accept either.
- `--square-mm 15.55`
- Blue/white is handled: detection uses `findChessboardCornersSB` and a
  `--gray-channel` option (auto tries luminance, falls back to the RED
  channel where blue squares read near-black). Verified working on
  synthetic blue/white renders down to ~10 px squares.
- **Hold it CLOSE**: on the 120° lens at 640×480, a 25 cm board at 30 cm is
  only ~150 px wide (~10 px squares — borderline). Work at **15–30 cm** from
  the lens so squares stay ≥10 px. Mount it dead-flat; flatness matters
  more than anything else.

(ChArUco boards are also supported — `--board charuco --pattern <squares>
--marker-mm ... --aruco-dict ...` — and are the better choice if buying
again: partial edge views still contribute corners.)

## 2. Capture (~40 frames)

Pi camera stack running and streaming; on the laptop (ROS env sourced):

```bash
# Daniel's blue/white 16x10 board:
python3 scripts/capture_calib_frames.py --out-dir calib_frames --count 40 \
    --pattern 15x9

# ChArUco alternative (use the board's own specs):
python3 scripts/capture_calib_frames.py --out-dir calib_frames --count 40 \
    --board charuco --pattern 10x7 --square-mm 25 --marker-mm 19 \
    --aruco-dict DICT_4X4_50
```

A preview window shows detected corners (green = counting) and a coverage
HUD (`cov L# R# T# B# corners:n`). Frames auto-save every 1.5 s when the
board is detected AND has moved since the last save. Choreography:

- Fill the frame center at 2–3 distances (0.4–1.5 m).
- Push the board into **all four edges and all four corners** — with
  ChArUco, let it run off-frame; partial views count.
- Tilt ±30° both axes, and a few rolled views.
- Keep the board still at each pose (motion blur ruins corners).

Stop when the HUD shows every region covered. `--manual` for SPACE-to-save.

## 3. Calibrate (offline)

```bash
# Daniel's board:
python3 scripts/calibrate_camera_intrinsics.py --images calib_frames \
    --pattern 15x9 --square-mm 15.55
# or with --board charuco --pattern 10x7 --marker-mm 19 --aruco-dict ...
```

Reading the report:
- **RMS reprojection error** ≤ 0.5 px is a good fit. 1–2 px = usable but
  mediocre; recapture with more variety.
- **Max edge-band residual** is the number that matters for this lens —
  it is the bearing error where the old model was worst.
- The script fits plumb_bob(5) and rational(8) and prints
  `RECOMMENDED MODEL`. If both fit poorly it suggests `--try-fisheye`.
- Heed the edge-coverage warning: an unconstrained edge fit will happily
  extrapolate garbage exactly where you need accuracy.

## 4. Install the intrinsics

Paste the emitted YAML block into the `tracker_node:` → `ros__parameters:`
section of **`configs/pi_hardware.yaml`** (deploy_pi.sh stages that file onto
the Pi as its `pi.yaml`). Then deploy + restart the stack and check the
tracker startup line:

```
Camera geometry: CALIBRATED rational (fx=..., fy=..., cx=..., cy=..., 8 coeffs)
```

If it says `legacy FOV pinhole`, the paste didn't land (wrong file or the
deploy didn't restage the config). A wrong coefficient count for the model
fails loudly at startup by design.

## 5. Validate

1. **Center bearing**: ball at a tape-measured lateral offset X at distance D
   from the lens, near frame center. `ros2 topic echo --once
   /drone/tracking/target_error` → `bearing_x_rad` ≈ `atan(X/D)` (±0.5°).
2. **Edge bearing**: same but with the ball near the frame edge. Before
   calibration this was off by multiple degrees; now it should match.
3. Optional distance cross-check with the measured fx:
   `python3 scripts/calibrate_distance.py --true-distance-m 3.0 --fx <camera_fx>`
   (k itself is unchanged by calibration; this just makes the fat-factor
   report honest.)

## Notes

- Intrinsics are resolution-specific: these are for **640×480**. If the
  capture pipeline ever changes resolution, recalibrate.
- All-zero `dist_coeffs` are treated as "no distortion" (pinhole with the
  measured fx/fy/cx/cy still applies).
- Bearing math lives in one place: `drone_tracker/camera_geometry.py`
  (unit tests: `drone_tracker/test/test_camera_geometry.py`).
