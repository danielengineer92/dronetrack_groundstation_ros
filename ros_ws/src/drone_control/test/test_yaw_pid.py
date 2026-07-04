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
