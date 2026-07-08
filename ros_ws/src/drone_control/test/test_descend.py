"""Unit tests for the descend step math + plan verb. Run directly:

    python3 src/drone_control/test/test_descend.py
"""

import os
import sys

try:
    from drone_control.control_math import descend_target_down
    from drone_control.mission_plan import (
        MissionPlanError,
        lint_plan,
        parse_mission_plan_text,
    )
except ImportError:  # pragma: no cover - direct-run convenience
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from drone_control.control_math import descend_target_down  # noqa: E402
    from drone_control.mission_plan import (  # noqa: E402
        MissionPlanError,
        lint_plan,
        parse_mission_plan_text,
    )


def test_relative_descend():
    # At 4 m altitude (down=-4), descend 1.5 -> 2.5 m altitude (down=-2.5).
    assert abs(descend_target_down(-4.0, descend_m=1.5) - (-2.5)) < 1e-9


def test_absolute_altitude():
    assert abs(descend_target_down(-4.0, altitude_m=2.0) - (-2.0)) < 1e-9


def test_altitude_wins_over_descend():
    assert abs(descend_target_down(-4.0, descend_m=3.0, altitude_m=3.5) - (-3.5)) < 1e-9


def test_floor_clamps_deep_descend():
    # 3 m altitude, descend 5 -> clamped at the 1.2 m floor.
    assert abs(descend_target_down(-3.0, descend_m=5.0) - (-1.2)) < 1e-9


def test_floor_never_commands_a_climb():
    # Already below the floor at 0.8 m: descend must hold, not climb to 1.2.
    assert abs(descend_target_down(-0.8, descend_m=2.0) - (-0.8)) < 1e-9


def test_absolute_ascent_is_allowed():
    # altitude_m above the current altitude passes through unclamped.
    assert abs(descend_target_down(-2.0, altitude_m=5.0) - (-5.0)) < 1e-9


DESCEND_PLAN = """
mission:
  name: t
  steps:
    - {type: takeoff, altitude_m: 4.0}
    - {type: prime_offboard, hold_s: 1.0}
    - {type: descend, descend_m: 1.5, tolerance_m: 0.3, timeout_s: 20}
    - {type: scan, until: locked, timeout_s: 60}
    - {type: land}
"""


def test_descend_verb_parses_and_lints_clean():
    plan = parse_mission_plan_text(DESCEND_PLAN)
    step = plan.steps[2]
    assert step.type == "descend"
    assert step.state_name == "DESCEND"
    assert step.get_float("descend_m", 0.0) == 1.5
    assert lint_plan(plan) == []


def test_descend_requires_a_target():
    bad = DESCEND_PLAN.replace("descend_m: 1.5, ", "")
    try:
        parse_mission_plan_text(bad)
    except MissionPlanError:
        pass
    else:
        raise AssertionError("descend without descend_m/altitude_m should fail")


def test_descend_rejects_out_of_range():
    bad = DESCEND_PLAN.replace("descend_m: 1.5", "descend_m: 100")
    try:
        parse_mission_plan_text(bad)
    except MissionPlanError:
        pass
    else:
        raise AssertionError("descend_m=100 should fail range validation")


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
