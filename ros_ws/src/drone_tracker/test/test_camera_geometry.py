"""Unit tests for camera_geometry (pure python; cv2 cases skip if absent). Run:

    python3 src/drone_tracker/test/test_camera_geometry.py
"""

import math
import os
import sys

try:
    from drone_tracker.camera_geometry import (
        CameraGeometry,
        bearing_from_pixel,
        build_geometry,
        describe,
    )
except ImportError:  # pragma: no cover - direct-run convenience
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from drone_tracker.camera_geometry import (  # noqa: E402
        CameraGeometry,
        bearing_from_pixel,
        build_geometry,
        describe,
    )

try:
    import cv2
    import numpy as np

    HAVE_CV2 = True
except ImportError:  # pragma: no cover
    HAVE_CV2 = False

W, H = 640, 480


def _legacy_fx_fy(hfov_deg=120.0, vfov_deg=105.0):
    hfov = math.radians(hfov_deg)
    vfov = math.radians(vfov_deg)
    return W / (2.0 * math.tan(hfov / 2.0)), H / (2.0 * math.tan(vfov / 2.0))


def _legacy_geom():
    fx, fy = _legacy_fx_fy()
    return build_geometry(
        image_width=W, image_height=H, fx=0.0, fy=0.0, cx=0.0, cy=0.0,
        dist_coeffs=[], distortion_model="rational", legacy_fx=fx, legacy_fy=fy)


def test_legacy_parity_with_old_formula():
    # Must match the pre-calibration tracker formula to machine precision.
    fx, fy = _legacy_fx_fy()
    geom = _legacy_geom()
    assert not geom.calibrated
    for u in (0.0, 17.5, 320.0, 555.2, 639.0):
        for v in (0.0, 100.1, 240.0, 400.9, 479.0):
            bx, by = bearing_from_pixel(geom, u, v)
            assert bx == math.atan((u - W / 2.0) / max(fx, 1e-6))
            assert by == math.atan((v - H / 2.0) / max(fy, 1e-6))


def test_pinhole_with_offcenter_principal_point():
    geom = build_geometry(
        image_width=W, image_height=H, fx=310.0, fy=312.0, cx=331.7, cy=236.1,
        dist_coeffs=[], distortion_model="plumb_bob", legacy_fx=1.0, legacy_fy=1.0)
    assert geom.calibrated
    bx, by = bearing_from_pixel(geom, 331.7, 236.1)
    assert abs(bx) < 1e-12 and abs(by) < 1e-12  # principal point = zero bearing
    bx, _ = bearing_from_pixel(geom, 331.7 + 310.0, 236.1)
    assert abs(bx - math.pi / 4) < 1e-12  # one focal length off-axis = 45 deg


def test_legacy_selected_when_fx_unset():
    geom = build_geometry(
        image_width=W, image_height=H, fx=0.0, fy=312.0, cx=1.0, cy=1.0,
        dist_coeffs=[1.0], distortion_model="rational", legacy_fx=184.75, legacy_fy=182.97)
    assert not geom.calibrated and geom.dist_coeffs == ()
    assert "legacy" in describe(geom)


def test_model_coeff_mismatch_raises():
    for model, badlen in (("plumb_bob", 8), ("rational", 5), ("fisheye", 5)):
        try:
            build_geometry(
                image_width=W, image_height=H, fx=310.0, fy=312.0, cx=0.0, cy=0.0,
                dist_coeffs=[0.1] * badlen, distortion_model=model,
                legacy_fx=1.0, legacy_fy=1.0)
        except ValueError:
            pass
        else:
            raise AssertionError(f"{model} with {badlen} coeffs should raise")


def test_unknown_model_raises():
    try:
        build_geometry(
            image_width=W, image_height=H, fx=310.0, fy=312.0, cx=0.0, cy=0.0,
            dist_coeffs=[], distortion_model="equidistant",
            legacy_fx=1.0, legacy_fy=1.0)
    except ValueError:
        pass
    else:
        raise AssertionError("unknown model should raise")


def _roundtrip_case(model, dist):
    """Project known-bearing rays through K+dist, then assert
    bearing_from_pixel recovers the bearings to < 0.2 deg."""
    fx, fy, cx, cy = 305.0, 308.0, 328.5, 243.2
    k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    d = np.array(dist, dtype=np.float64)
    geom = build_geometry(
        image_width=W, image_height=H, fx=fx, fy=fy, cx=cx, cy=cy,
        dist_coeffs=list(dist), distortion_model=model, legacy_fx=1.0, legacy_fy=1.0)

    max_err_deg = 0.0
    for bx_deg in (-40, -25, -10, 0, 10, 25, 40):
        for by_deg in (-30, -15, 0, 15, 30):
            bx, by = math.radians(bx_deg), math.radians(by_deg)
            obj = np.array([[[math.tan(bx), math.tan(by), 1.0]]], dtype=np.float64)
            if model == "fisheye":
                img, _ = cv2.fisheye.projectPoints(
                    obj, np.zeros(3), np.zeros(3), k, d)
            else:
                img, _ = cv2.projectPoints(
                    obj, np.zeros(3), np.zeros(3), k, d)
            u, v = float(img[0, 0, 0]), float(img[0, 0, 1])
            if not (0 <= u < W and 0 <= v < H):
                continue  # distorted outside the sensor; skip
            rbx, rby = bearing_from_pixel(geom, u, v)
            max_err_deg = max(
                max_err_deg,
                abs(math.degrees(rbx - bx)),
                abs(math.degrees(rby - by)),
            )
    assert max_err_deg < 0.2, f"{model}: max bearing error {max_err_deg:.3f} deg"


def test_roundtrip_plumb_bob():
    if not HAVE_CV2:
        print("  (skipped: no cv2)")
        return
    _roundtrip_case("plumb_bob", [-0.30, 0.09, 0.0002, -0.0003, -0.01])


def test_roundtrip_rational():
    if not HAVE_CV2:
        print("  (skipped: no cv2)")
        return
    _roundtrip_case(
        "rational", [-0.30, 0.09, 0.0002, -0.0003, -0.01, 0.05, -0.02, 0.004])


def test_roundtrip_fisheye():
    if not HAVE_CV2:
        print("  (skipped: no cv2)")
        return
    _roundtrip_case("fisheye", [-0.05, 0.01, -0.004, 0.001])


def test_empty_dist_equals_pinhole_atan():
    geom = build_geometry(
        image_width=W, image_height=H, fx=305.0, fy=308.0, cx=328.5, cy=243.2,
        dist_coeffs=[], distortion_model="rational", legacy_fx=1.0, legacy_fy=1.0)
    bx, by = bearing_from_pixel(geom, 500.0, 100.0)
    assert bx == math.atan((500.0 - 328.5) / 305.0)
    assert by == math.atan((100.0 - 243.2) / 308.0)


def _run_all():
    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as exc:
                failures += 1
                print(f"FAIL {name}: {exc}")
    return failures


if __name__ == "__main__":
    sys.exit(1 if _run_all() else 0)
