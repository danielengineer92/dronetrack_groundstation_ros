"""Unit tests for control_math (pure, no rclpy). Run directly:

    python3 src/drone_control/test/test_control_math.py
"""

import math
import os
import sys

try:
    from drone_control.control_math import approach_forward_velocity, clamp
except ImportError:  # pragma: no cover - direct-run convenience
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from drone_control.control_math import approach_forward_velocity, clamp  # noqa: E402


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
