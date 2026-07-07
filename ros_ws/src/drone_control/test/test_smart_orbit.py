"""Unit tests for the smart_orbit plan verb + settle-then-sample gate. Run directly:

    python3 src/drone_control/test/test_smart_orbit.py
"""

import os
import sys

try:
    from drone_control.control_math import settle_gate
    from drone_control.mission_plan import (
        MissionPlanError,
        lint_plan,
        parse_mission_plan_text,
    )
except ImportError:  # pragma: no cover - direct-run convenience
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from drone_control.control_math import settle_gate  # noqa: E402
    from drone_control.mission_plan import (  # noqa: E402
        MissionPlanError,
        lint_plan,
        parse_mission_plan_text,
    )


def test_settle_gate_not_centered_resets_streak():
    streak, ready = settle_gate(3.0, 5.0, centered=False, settle_s=2.0)
    assert streak is None and not ready


def test_settle_gate_starts_streak_on_first_centered_tick():
    streak, ready = settle_gate(None, 4.2, centered=True, settle_s=2.0)
    assert streak == 4.2 and not ready


def test_settle_gate_not_ready_before_settle_elapsed():
    streak, ready = settle_gate(4.0, 5.9, centered=True, settle_s=2.0)
    assert streak == 4.0 and not ready


def test_settle_gate_ready_after_continuous_centered_hold():
    streak, ready = settle_gate(4.0, 6.0, centered=True, settle_s=2.0)
    assert streak == 4.0 and ready


def test_settle_gate_streak_restart_delays_ready():
    # Centered 0..1.5s, blip at 1.5s, centered again: the clock restarts.
    streak, ready = settle_gate(0.0, 1.5, centered=False, settle_s=2.0)
    assert streak is None and not ready
    streak, ready = settle_gate(streak, 1.6, centered=True, settle_s=2.0)
    assert streak == 1.6 and not ready
    streak, ready = settle_gate(streak, 3.5, centered=True, settle_s=2.0)
    assert streak == 1.6 and not ready
    streak, ready = settle_gate(streak, 3.6, centered=True, settle_s=2.0)
    assert ready


def test_settle_gate_zero_settle_is_ready_when_centered():
    _, ready = settle_gate(None, 0.0, centered=True, settle_s=0.0)
    assert ready
    _, ready = settle_gate(None, 0.0, centered=False, settle_s=0.0)
    assert not ready


def test_settle_gate_never_ready_on_uncentered_tick():
    # ready implies centered-now, so fixes gated on ready are centered fixes.
    streak, ready = settle_gate(0.0, 100.0, centered=False, settle_s=0.0)
    assert streak is None and not ready


SMART_ORBIT_PLAN = """
mission:
  name: t
  steps:
    - {type: takeoff, altitude_m: 3.2}
    - {type: prime_offboard, hold_s: 1.0}
    - {type: scan, until: locked, timeout_s: 60}
    - {type: track_center, until: centered, timeout_s: 30}
    - {type: smart_orbit, radius_m: 1.5, speed_m_s: 0.6, revolutions: 2,
       center_samples: 30, settle_s: 2.0, sample_timeout_s: 30}
    - {type: land}
"""


def test_smart_orbit_verb_parses_and_lints_clean():
    plan = parse_mission_plan_text(SMART_ORBIT_PLAN)
    step = plan.steps[4]
    assert step.type == "smart_orbit"
    assert step.state_name == "SMART_ORBIT"
    assert step.get_float("center_samples", 0.0) == 30.0
    assert step.get_float("settle_s", 0.0) == 2.0
    # No timeout_s lint nag: smart_orbit derives its timeout from revolutions.
    assert lint_plan(plan) == []


def test_smart_orbit_rejects_out_of_range_settle():
    bad = SMART_ORBIT_PLAN.replace("settle_s: 2.0", "settle_s: 500")
    try:
        parse_mission_plan_text(bad)
    except MissionPlanError:
        pass
    else:
        raise AssertionError("settle_s=500 should fail range validation")


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
