"""Unit tests for autonomy_logic (pure, no rclpy). Run directly:

    python3 src/drone_control/test/test_autonomy_logic.py
"""

import os
import sys

try:
    from drone_control.autonomy_logic import scan_yaw_authorized
except ImportError:  # pragma: no cover - direct-run convenience
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from drone_control.autonomy_logic import scan_yaw_authorized  # noqa: E402


def _auth(**kw):
    base = dict(
        allow_scan_without_lock=True,
        mission_is_scan=True,
        request_fresh=True,
        safety_ok=True,
        target_locked=False,
    )
    base.update(kw)
    return scan_yaw_authorized(**base)


def test_authorized_when_scanning_and_safe():
    assert _auth() is True


def test_disabled_by_default_param():
    assert _auth(allow_scan_without_lock=False) is False


def test_not_authorized_when_not_scan_step():
    assert _auth(mission_is_scan=False) is False


def test_not_authorized_when_locked():
    # A locked target is normal TRACKING, not a search sweep.
    assert _auth(target_locked=True) is False


def test_requires_fresh_request_and_safety():
    assert _auth(request_fresh=False) is False
    assert _auth(safety_ok=False) is False


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
