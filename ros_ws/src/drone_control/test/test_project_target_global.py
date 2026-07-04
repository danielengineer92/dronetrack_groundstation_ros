"""Unit tests for project_target_global (pure, no rclpy). Run directly:

    python3 src/drone_control/test/test_project_target_global.py

These cover the geometry that places the DO_ORBIT centre: heading composition,
the equirectangular lat/lon offset, and — the reason this function exists — the
elevation (bearing_y) foreshortening of the slant range onto the ground plane.
"""

import math
import os
import sys

try:
    from drone_control.control_math import project_target_global
except ImportError:  # pragma: no cover - direct-run convenience
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from drone_control.control_math import project_target_global  # noqa: E402


ORIGIN = dict(lat_deg=47.3977508, lon_deg=8.5455938)  # a non-trivial mid-lat point
_M_PER_DEG_LAT = math.pi * 6378137.0 / 180.0  # ~111319 m


def _p(**kw):
    base = dict(
        **ORIGIN,
        yaw_rad=0.0,
        bearing_x_rad=0.0,
        bearing_y_rad=0.0,
        slant_distance_m=10.0,
    )
    base.update(kw)
    return project_target_global(**base)


def test_straight_ahead_north_when_yaw_zero():
    # Heading 0 (north), no bearing offsets, level camera: 10 m due north.
    lat, lon = _p(slant_distance_m=10.0)
    north_m = (lat - ORIGIN["lat_deg"]) * _M_PER_DEG_LAT
    assert math.isclose(north_m, 10.0, abs_tol=1e-3)
    assert math.isclose(lon, ORIGIN["lon_deg"], abs_tol=1e-9)  # no easting


def test_heading_east_puts_target_east():
    # Yaw = +90 deg (east). 10 m should land due east, no northing.
    lat, lon = _p(yaw_rad=math.pi / 2.0, slant_distance_m=10.0)
    m_per_deg_lon = _M_PER_DEG_LAT * math.cos(math.radians(ORIGIN["lat_deg"]))
    east_m = (lon - ORIGIN["lon_deg"]) * m_per_deg_lon
    assert math.isclose(east_m, 10.0, abs_tol=1e-2)
    assert math.isclose(lat, ORIGIN["lat_deg"], abs_tol=1e-9)


def test_bearing_x_adds_to_yaw():
    # Yaw 0 + bearing_x +90deg == pure east, same as heading east.
    a = _p(bearing_x_rad=math.pi / 2.0)
    b = _p(yaw_rad=math.pi / 2.0)
    assert math.isclose(a[0], b[0], abs_tol=1e-9)
    assert math.isclose(a[1], b[1], abs_tol=1e-9)


def test_elevation_foreshortens_horizontal_distance():
    # This is the F1 fix: a 10 m slant range seen 60deg below the optical axis
    # is only 10*cos(60deg) = 5 m of horizontal ground distance.
    lat, _ = _p(bearing_y_rad=math.radians(60.0), slant_distance_m=10.0)
    north_m = (lat - ORIGIN["lat_deg"]) * _M_PER_DEG_LAT
    assert math.isclose(north_m, 5.0, abs_tol=1e-3)


def test_target_directly_below_has_near_zero_horizontal_offset():
    # bearing_y -> 90deg (straight down): horizontal offset collapses to ~0,
    # instead of emitting the full slant distance (the pre-fix bug).
    lat, lon = _p(bearing_y_rad=math.radians(89.9), slant_distance_m=10.0)
    north_m = (lat - ORIGIN["lat_deg"]) * _M_PER_DEG_LAT
    assert abs(north_m) < 0.02  # 10*cos(89.9deg) ~= 0.017 m
    assert math.isclose(lon, ORIGIN["lon_deg"], abs_tol=1e-9)


def test_zero_distance_returns_origin():
    lat, lon = _p(slant_distance_m=0.0)
    assert math.isclose(lat, ORIGIN["lat_deg"], abs_tol=1e-12)
    assert math.isclose(lon, ORIGIN["lon_deg"], abs_tol=1e-12)


def _run_all():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failures = 0
    for test in tests:
        try:
            test()
            print(f"PASS {test.__name__}")
        except Exception as exc:  # noqa: BLE001
            failures += 1
            print(f"FAIL {test.__name__}: {exc}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(_run_all())
