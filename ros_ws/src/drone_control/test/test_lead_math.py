"""Unit tests for curved_lead_offset + estimate_turn_rate (pure). Run:

    python3 src/drone_control/test/test_lead_math.py
"""
import math
import os
import sys

try:
    from drone_control.control_math import curved_lead_offset, estimate_turn_rate
except ImportError:  # pragma: no cover
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from drone_control.control_math import curved_lead_offset, estimate_turn_rate  # noqa: E402


def test_zero_turn_is_straight_line():
    dn, de = curved_lead_offset(velocity_north_m_s=0.8, velocity_east_m_s=-0.4,
                                turn_rate_rad_s=0.0, lead_s=2.0)
    assert math.isclose(dn, 1.6) and math.isclose(de, -0.8)


def test_curved_matches_circle_chord():
    # Ball on a radius-3 circle, omega=0.314, speed=0.942, at heading due east
    # (tangent). Over 2 s it sweeps 0.628 rad; chord length = 2*r*sin(w*L/2).
    r, w, L = 3.0, 0.314, 2.0
    s = w * r
    dn, de = curved_lead_offset(velocity_north_m_s=0.0, velocity_east_m_s=s,
                                turn_rate_rad_s=w, lead_s=L)
    chord = 2 * r * math.sin(w * L / 2)
    assert math.isclose(math.hypot(dn, de), chord, rel_tol=1e-6), math.hypot(dn, de)
    # magnitude is SHORTER than the straight arc s*L
    assert math.hypot(dn, de) < s * L


def test_curved_rotates_toward_turn():
    # Heading north, turning +CW (toward east): displacement bends east.
    dn, de = curved_lead_offset(velocity_north_m_s=1.0, velocity_east_m_s=0.0,
                                turn_rate_rad_s=0.3, lead_s=2.0)
    assert de > 0.0 and dn > 0.0


def test_turn_rate_clamped():
    dn, de = curved_lead_offset(velocity_north_m_s=1.0, velocity_east_m_s=0.0,
                                turn_rate_rad_s=100.0, lead_s=2.0,
                                max_turn_rate_rad_s=2.0)
    # clamped: finite, bounded by |s|*L in magnitude
    assert math.hypot(dn, de) <= 1.0 * 2.0 + 1e-9


def test_estimate_turn_rate_recovers_omega():
    w, dt = 0.314, 0.1
    s = 0.942
    ts = [i * dt for i in range(12)]
    vn = [s * math.cos(w * t) for t in ts]
    ve = [s * math.sin(w * t) for t in ts]
    assert math.isclose(estimate_turn_rate(ts, vn, ve), w, rel_tol=1e-3)


def test_estimate_turn_rate_straight_is_zero():
    ts = [i * 0.1 for i in range(10)]
    vn = [0.8] * 10
    ve = [-0.4] * 10
    assert abs(estimate_turn_rate(ts, vn, ve)) < 1e-6


def test_estimate_turn_rate_too_few_samples():
    assert estimate_turn_rate([0.0, 0.1], [1.0, 1.0], [0.0, 0.0]) == 0.0


def main() -> int:
    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as exc:
                failures += 1
                print(f"FAIL {name}: {exc}")
    print("OK" if failures == 0 else f"{failures} FAILURES")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
