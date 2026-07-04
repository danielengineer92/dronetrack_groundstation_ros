"""Unit tests for control_math (pure, no rclpy). Run directly:

    python3 src/drone_control/test/test_control_math.py
"""

import math
import os
import sys

try:
    from drone_control.control_math import (
        alignment_scale,
        approach_forward_velocity,
        braking_speed_limit,
        clamp,
    )
except ImportError:  # pragma: no cover - direct-run convenience
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from drone_control.control_math import (  # noqa: E402
        alignment_scale,
        approach_forward_velocity,
        braking_speed_limit,
        clamp,
    )


def _v(**kw):
    base = dict(
        distance_valid=True,
        distance_m=5.0,
        desired_distance_m=2.0,
        gain=0.6,
        max_speed=2.0,
        deadband_m=0.1,
        target_locked=True,
    )
    base.update(kw)
    return approach_forward_velocity(**base)


def test_moves_forward_when_too_far():
    # 5m away, want 2m -> error +3 -> 0.6*3 = 1.8 m/s forward (under the 2.0 clamp).
    assert math.isclose(_v(distance_m=5.0), 1.8, rel_tol=1e-6)


def test_backs_off_when_too_close():
    v = _v(distance_m=1.0)  # error -1 -> -0.6
    assert math.isclose(v, -0.6, rel_tol=1e-6)


def test_clamped_to_max_speed():
    v = _v(distance_m=20.0, max_speed=2.0)  # 0.6*18=10.8 -> clamp 2.0
    assert v == 2.0


def test_deadband_holds_near_goal():
    assert _v(distance_m=2.05, deadband_m=0.1) == 0.0  # within deadband -> hold


def test_invalid_distance_holds():
    assert _v(distance_valid=False) == 0.0
    assert _v(distance_m=0.0) == 0.0
    assert _v(distance_m=float("nan")) == 0.0
    assert _v(distance_m=-3.0) == 0.0


def test_unlocked_target_holds():
    assert _v(target_locked=False) == 0.0


def test_clamp_helper():
    assert clamp(5, 0, 2) == 2
    assert clamp(-5, -1, 1) == -1
    assert clamp(0.5, 0, 1) == 0.5


# ---- professional approach profile (delay-aware braking + alignment gate) ----

def test_braking_cap_reduces_far_approach_speed():
    # 5 m out, desired 2 m -> error 3 m. Raw P = 0.6*3 = 1.8 m/s, but the
    # profile (delay 0.35 s, decel 0.5) allows only sqrt-ish of that.
    capped = _v(delay_s=0.35, decel_m_s2=0.5)
    raw = _v()
    assert 0.0 < capped < raw, (capped, raw)
    expected = braking_speed_limit(remaining=3.0, delay_s=0.35, decel=0.5, max_speed=2.0)
    assert math.isclose(capped, expected), (capped, expected)


def test_braking_cap_decelerates_into_standoff():
    # Approaching the goal, the allowed speed must shrink monotonically.
    prev = float("inf")
    for d in (8.0, 6.0, 4.0, 3.0, 2.5, 2.2):
        v = _v(distance_m=d, delay_s=0.35, decel_m_s2=0.5)
        assert v <= prev + 1e-12, (d, v, prev)
        prev = v


def test_braking_cap_applies_to_backoff_too():
    # Way too close: backing off is also profile-limited (symmetric clamp).
    v = _v(distance_m=0.3, desired_distance_m=2.0, gain=5.0, delay_s=0.35, decel_m_s2=0.5)
    cap = braking_speed_limit(remaining=1.7, delay_s=0.35, decel=0.5, max_speed=2.0)
    assert math.isclose(v, -cap), (v, cap)


def test_alignment_gate_zeroes_off_center_charge():
    # Target far off-centre: closing speed must be zero, not a lunge.
    assert _v(error_x=0.6, align_error_limit=0.5) == 0.0
    assert _v(error_x=-0.6, align_error_limit=0.5) == 0.0


def test_alignment_gate_ramps_linearly():
    full = _v(error_x=0.0, align_error_limit=0.5)
    half = _v(error_x=0.25, align_error_limit=0.5)
    assert math.isclose(half, 0.5 * full), (half, full)


def test_alignment_gate_never_blocks_backoff():
    # Too close AND off-centre: increasing separation stays allowed.
    v = _v(distance_m=1.0, desired_distance_m=2.0, error_x=0.9, align_error_limit=0.5)
    assert v < 0.0, v


def test_alignment_gate_disabled_by_zero_limit():
    assert _v(error_x=0.9, align_error_limit=0.0) == _v()


def test_alignment_scale_helper():
    assert alignment_scale(error_x=0.0, error_limit=0.5) == 1.0
    assert alignment_scale(error_x=0.5, error_limit=0.5) == 0.0
    assert alignment_scale(error_x=-0.25, error_limit=0.5) == 0.5
    assert alignment_scale(error_x=0.9, error_limit=0.0) == 1.0  # disabled


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
