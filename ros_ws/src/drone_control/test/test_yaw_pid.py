"""Unit tests for yaw_pid_step (pure, no rclpy). Run directly:

    python3 src/drone_control/test/test_yaw_pid.py
"""

import math
import os
import sys

try:
    from drone_control.control_math import yaw_pid_step
except ImportError:  # pragma: no cover - direct-run convenience
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from drone_control.control_math import yaw_pid_step  # noqa: E402

DT = 1.0 / 20.0  # control node default rate


def _step(error, integral=0.0, derivative=0.0, prev_error=0.0, first=False, **kw):
    base = dict(
        error=error,
        dt=DT,
        kp=1.5,
        ki=0.0,
        kd=0.0,
        integral=integral,
        filtered_derivative=derivative,
        prev_error=prev_error,
        first_sample=first,
        output_limit=1.0,
        integral_limit=0.3,
        derivative_alpha=0.35,
    )
    base.update(kw)
    return yaw_pid_step(**base)


def test_pure_p_matches_legacy_law():
    # ki=kd=0 must reproduce the original `error * gain_yaw` exactly.
    for err in (-1.0, -0.25, 0.0, 0.1, 0.73):
        out, i, d = _step(err, first=True)
        assert out == err * 1.5, (err, out)
        assert i == 0.0 and d == 0.0


def test_ki_zero_zeroes_stale_integral():
    # Enabling/disabling I live must not replay old windup.
    out, i, _ = _step(0.2, integral=5.0, ki=0.0)
    assert i == 0.0
    assert out == 0.2 * 1.5


def test_integral_accumulates_standing_error():
    i = 0.0
    for _ in range(40):  # 2 s of a constant small error
        out, i, _ = _step(0.1, integral=i, ki=0.5)
    # integral ≈ 0.1 * 2s = 0.2 -> contribution ≈ 0.1 rad/s on top of P
    assert 0.15 < i < 0.21, i
    assert out > 0.1 * 1.5, "I must push beyond pure P on standing error"


def test_integral_contribution_clamped():
    i = 0.0
    for _ in range(2000):  # 100 s — way past the clamp
        _, i, _ = _step(0.1, integral=i, ki=0.5, integral_limit=0.3)
    assert abs(0.5 * i) <= 0.3 + 1e-9, "ki*integral must respect integral_limit"


def test_anti_windup_freezes_integral_when_saturated():
    # error=1.0, kp=1.5 -> P alone (1.5) already exceeds output_limit (1.0):
    # the integral must not grow while we push further into saturation.
    _, i1, _ = _step(1.0, integral=0.0, ki=0.5)
    assert i1 == 0.0, f"integral grew while saturated: {i1}"
    # ...but it must still unwind when the error flips sign.
    _, i2, _ = _step(-0.2, integral=0.4, ki=0.5, prev_error=1.0)
    assert i2 < 0.4, "integral must unwind on sign reversal"


def test_derivative_first_sample_is_zero():
    out, _, d = _step(0.5, first=True, kd=0.2)
    assert d == 0.0
    assert out == 0.5 * 1.5


def test_derivative_filtered():
    # Step from 0 -> 0.5 error in one tick: raw slope = 0.5/DT = 10 rad/s.
    # alpha=0.35 -> filtered = 3.5; kd=0.2 adds 0.7.
    out, _, d = _step(0.5, prev_error=0.0, kd=0.2, derivative_alpha=0.35)
    assert math.isclose(d, 0.35 * (0.5 / DT)), d
    assert math.isclose(out, 0.5 * 1.5 + 0.2 * d), out
    # Same slope again: filtered keeps converging toward raw (10), no jump.
    _, _, d2 = _step(1.0, prev_error=0.5, derivative=d, kd=0.2, derivative_alpha=0.35)
    assert d < d2 < 0.5 / DT, d2


def test_derivative_damps_overshoot_direction():
    # Error shrinking (target centring): D must oppose P (negative contribution).
    out_pd, _, _ = _step(0.3, prev_error=0.5, kd=0.2)
    out_p, _, _ = _step(0.3, prev_error=0.5, kd=0.0)
    assert out_pd < out_p, "D on a shrinking error must reduce the command"


try:
    from drone_control.control_math import yaw_feedforward_step
except ImportError:  # pragma: no cover
    from drone_control.control_math import yaw_feedforward_step  # noqa: F401


def _ff(**kw):
    base = dict(
        los_angle_rad=0.0, prev_los_angle_rad=0.0, dt=DT,
        prev_ff_rad_s=0.0, first_sample=False, lpf_alpha=1.0, limit_rad_s=1.5,
    )
    base.update(kw)
    return yaw_feedforward_step(**base)


def test_ff_first_sample_is_zero():
    assert _ff(los_angle_rad=0.4, first_sample=True) == 0.0


def test_ff_still_target_is_structurally_zero():
    # Still ball: inertial LOS angle constant no matter how we rotate.
    assert _ff(los_angle_rad=1.234, prev_los_angle_rad=1.234) == 0.0


def test_ff_recovers_target_rate():
    # Ball moving at +0.3 rad/s inertially: LOS advances 0.3*dt per tick.
    ff = _ff(los_angle_rad=0.3 * DT, prev_los_angle_rad=0.0)
    assert math.isclose(ff, 0.3), ff


def test_ff_wraps_across_pi():
    # LOS crossing the +/-pi seam must not produce a 2*pi/dt spike.
    ff = _ff(los_angle_rad=-math.pi + 0.01, prev_los_angle_rad=math.pi - 0.01,
             limit_rad_s=100.0)
    assert math.isclose(ff, 0.02 / DT, rel_tol=1e-6), ff


def test_ff_lowpass_blends():
    ff = _ff(los_angle_rad=0.3 * DT, prev_los_angle_rad=0.0, lpf_alpha=0.25)
    assert math.isclose(ff, 0.25 * 0.3), ff


def test_ff_clamped():
    ff = _ff(los_angle_rad=1.0, prev_los_angle_rad=0.0, limit_rad_s=1.5)
    assert ff == 1.5


try:
    from drone_control.control_math import los_rate_from_state
except ImportError:  # pragma: no cover
    from drone_control.control_math import los_rate_from_state  # noqa: F401


def _los(**kw):
    base = dict(rel_north_m=5.0, rel_east_m=0.0,
                rel_velocity_north_m_s=0.0, rel_velocity_east_m_s=0.0)
    base.update(kw)
    return los_rate_from_state(**base)


def test_los_rate_still_target_zero():
    assert _los() == 0.0


def test_los_rate_crossing_target_positive_cw():
    # Target 5 m north, moving east at 1 m/s: LOS rotates toward east,
    # NED yaw is +CW toward east -> positive rate, magnitude v/r.
    assert math.isclose(_los(rel_velocity_east_m_s=1.0), 1.0 / 5.0)


def test_los_rate_crossing_west_negative():
    assert math.isclose(_los(rel_velocity_east_m_s=-1.0), -0.2)


def test_los_rate_radial_motion_zero():
    # Straight at us: bearing unchanged.
    assert _los(rel_velocity_north_m_s=-2.0) == 0.0


def test_los_rate_point_blank_guard():
    assert _los(rel_north_m=0.2, rel_velocity_east_m_s=5.0) == 0.0


def test_los_rate_general_geometry():
    # Target due east 4 m, moving north 0.8: lambda=atan2(rE,rN)=90deg,
    # moving north swings LOS back toward north = CCW = negative.
    r = los_rate_from_state(rel_north_m=0.0, rel_east_m=4.0,
                            rel_velocity_north_m_s=0.8,
                            rel_velocity_east_m_s=0.0)
    assert math.isclose(r, -0.8 / 4.0), r


try:
    from drone_control.control_math import braking_speed_limit
except ImportError:  # pragma: no cover
    from drone_control.control_math import braking_speed_limit  # noqa: F401


def _cap(**kw):
    base = dict(
        remaining=0.6, delay_s=0.35, decel=0.8,
        max_speed=1.0, min_speed=0.0,
    )
    base.update(kw)
    return braking_speed_limit(**base)


def test_approach_cap_zero_angle_is_floor():
    # Centred target: cap collapses to the min-rate floor (0 by default).
    assert _cap(remaining=0.0) == 0.0
    assert _cap(remaining=0.0, min_speed=0.15) == 0.15


def test_approach_cap_symmetric_in_sign():
    assert _cap(remaining=0.3) == _cap(remaining=-0.3)


def test_approach_cap_monotonic_increasing():
    # Farther target -> higher allowed rate (up to the ceiling).
    prev = -1.0
    for ang in (0.02, 0.05, 0.1, 0.2, 0.4, 0.8):
        c = _cap(remaining=ang)
        assert c >= prev, (ang, c, prev)
        prev = c


def test_approach_cap_deadtime_limited_small_angle():
    # Small angle (<< a*delay^2): rate -> angle/delay (the sqrt term linearises).
    c = _cap(remaining=0.005, delay_s=0.35, decel=0.8)
    assert math.isclose(c, 0.005 / 0.35, rel_tol=0.05), c


def test_approach_cap_decel_limited_large_angle():
    # Large angle with no dead time: rate -> sqrt(2*a*angle).
    c = _cap(remaining=1.0, delay_s=0.0, decel=0.8, max_speed=99.0)
    assert math.isclose(c, math.sqrt(2 * 0.8 * 1.0)), c


def test_approach_cap_respects_ceiling():
    assert _cap(remaining=5.0, max_speed=1.0) == 1.0


def test_approach_cap_prevents_full_rate_mid_approach():
    # The whole point: at a moderate remaining bearing the cap is BELOW the
    # saturated max rate, so the drone is already decelerating (no overshoot).
    c = _cap(remaining=0.15, delay_s=0.35, decel=0.8, max_speed=1.0)
    assert c < 1.0, c
    assert c < 0.5, c  # meaningfully backed off, not a token reduction


def test_approach_cap_never_below_floor_when_angle_positive():
    c = _cap(remaining=0.001, min_speed=0.15)
    assert c >= 0.15


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
