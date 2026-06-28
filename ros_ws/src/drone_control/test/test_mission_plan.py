"""Unit tests for the mission plan model/loader.

Pure Python, no rclpy/ROS imports, so it runs with plain `python3` as well as
under `colcon test` / pytest. Run directly:

    python3 src/drone_control/test/test_mission_plan.py
"""

import os
import sys

# Allow running directly (without the package installed) by adding the package root.
try:
    from drone_control.mission_plan import (
        MissionPlanError,
        build_default_plan,
        lint_plan,
        parse_mission_plan,
    )
except ImportError:  # pragma: no cover - direct-run convenience
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from drone_control.mission_plan import (  # noqa: E402
        MissionPlanError,
        build_default_plan,
        lint_plan,
        parse_mission_plan,
    )


def test_default_plan_matches_original_sequence():
    plan = build_default_plan(run_full_orbit=False)
    assert [s.type for s in plan.steps] == ["takeoff", "prime_offboard", "track_center"]
    # track_center holds forever in the conservative default (until: none).
    assert plan.steps[-1].until == "none"


def test_default_plan_with_orbit_appends_full_sequence():
    plan = build_default_plan(run_full_orbit=True)
    assert [s.type for s in plan.steps] == [
        "takeoff", "prime_offboard", "track_center", "approach", "orbit", "rtl", "land",
    ]
    assert plan.steps[2].until == "centered"


def test_parse_orbit_plan():
    data = {
        "mission": {
            "name": "orbit_red_ball",
            "steps": [
                {"type": "takeoff", "altitude_m": 3.0},
                {"type": "prime_offboard", "hold_s": 1.5},
                {"type": "track_center", "until": "centered", "timeout_s": 20},
                {"type": "orbit", "radius_m": 2.0, "speed_m_s": 0.4, "revolutions": 1},
                {"type": "land"},
            ],
        }
    }
    plan = parse_mission_plan(data)
    assert plan.name == "orbit_red_ball"
    assert [s.type for s in plan.steps] == [
        "takeoff", "prime_offboard", "track_center", "orbit", "land",
    ]
    assert plan.steps[0].get_float("altitude_m", 0.0) == 3.0
    assert plan.steps[2].until == "centered"
    assert plan.steps[2].timeout_s == 20.0
    assert plan.steps[3].get_float("revolutions", 0.0) == 1.0


def test_bare_string_step_shorthand():
    plan = parse_mission_plan({"mission": {"steps": ["takeoff", "land"]}})
    assert [s.type for s in plan.steps] == ["takeoff", "land"]


def test_unknown_step_type_raises():
    try:
        parse_mission_plan({"mission": {"steps": [{"type": "barrel_roll"}]}})
    except MissionPlanError as exc:
        assert "barrel_roll" in str(exc)
    else:
        raise AssertionError("expected MissionPlanError for unknown step type")


def test_unknown_until_raises():
    try:
        parse_mission_plan({"mission": {"steps": [{"type": "track_center", "until": "forever"}]}})
    except MissionPlanError as exc:
        assert "forever" in str(exc)
    else:
        raise AssertionError("expected MissionPlanError for unknown until")


def test_empty_steps_raises():
    for bad in ({"mission": {"steps": []}}, {"mission": {}}, {}, []):
        try:
            parse_mission_plan(bad)
        except MissionPlanError:
            pass
        else:
            raise AssertionError(f"expected MissionPlanError for {bad!r}")


def test_lint_warns_motion_before_prime():
    plan = parse_mission_plan({"mission": {"steps": [{"type": "track_center"}, {"type": "land"}]}})
    warnings = lint_plan(plan)
    assert any("track_center" in w for w in warnings)
    # A properly ordered plan produces no warnings.
    ok = parse_mission_plan({"mission": {"steps": [{"type": "prime_offboard"}, {"type": "track_center"}]}})
    assert lint_plan(ok) == []


def test_parse_scan_step():
    plan = parse_mission_plan(
        {
            "mission": {
                "steps": [
                    {"type": "prime_offboard"},
                    {
                        "type": "scan",
                        "direction": "ccw",
                        "yaw_deg": 180,
                        "yaw_rate_deg_s": 20,
                        "until": "locked",
                        "timeout_s": 12,
                    },
                ]
            }
        }
    )
    scan = plan.steps[1]
    assert scan.type == "scan"
    assert scan.scan_direction == "ccw"
    assert scan.get_float("yaw_deg", 0.0) == 180.0
    assert scan.get_float("yaw_rate_deg_s", 0.0) == 20.0
    assert scan.until == "locked"
    assert scan.timeout_s == 12.0


def test_scan_direction_defaults_ccw():
    plan = parse_mission_plan({"mission": {"steps": [{"type": "scan", "timeout_s": 5}]}})
    assert plan.steps[0].scan_direction == "ccw"


def test_scan_bad_direction_raises():
    try:
        parse_mission_plan({"mission": {"steps": [{"type": "scan", "direction": "sideways"}]}})
    except MissionPlanError as exc:
        assert "sideways" in str(exc)
    else:
        raise AssertionError("expected MissionPlanError for bad scan direction")


def test_scan_nonpositive_yaw_rate_raises():
    for bad in (0, -5):
        try:
            parse_mission_plan({"mission": {"steps": [{"type": "scan", "yaw_rate_deg_s": bad}]}})
        except MissionPlanError as exc:
            assert "yaw_rate_deg_s" in str(exc)
        else:
            raise AssertionError(f"expected MissionPlanError for yaw_rate_deg_s={bad}")


def test_scan_bad_yaw_deg_raises():
    try:
        parse_mission_plan({"mission": {"steps": [{"type": "scan", "yaw_deg": "lots"}]}})
    except MissionPlanError as exc:
        assert "yaw_deg" in str(exc)
    else:
        raise AssertionError("expected MissionPlanError for non-numeric yaw_deg")


def test_negative_timeout_raises():
    try:
        parse_mission_plan({"mission": {"steps": [{"type": "orbit", "timeout_s": -1}]}})
    except MissionPlanError as exc:
        assert "timeout_s" in str(exc)
    else:
        raise AssertionError("expected MissionPlanError for negative timeout_s")


def test_lint_warns_missing_timeout_on_scan():
    plan = parse_mission_plan(
        {"mission": {"steps": [{"type": "prime_offboard"}, {"type": "scan", "until": "locked"}]}}
    )
    warnings = lint_plan(plan)
    assert any("scan" in w and "timeout_s" in w for w in warnings)


def test_lint_warns_scan_without_until_or_timeout():
    plan = parse_mission_plan({"mission": {"steps": [{"type": "prime_offboard"}, {"type": "scan"}]}})
    assert any("scan" in w and "until" in w for w in lint_plan(plan))


def test_lint_warns_motion_without_any_prime():
    plan = parse_mission_plan({"mission": {"steps": [{"type": "scan", "timeout_s": 5}, {"type": "land"}]}})
    assert any("no 'prime_offboard'" in w for w in lint_plan(plan))


def test_lint_clean_scan_plan_has_no_timeout_warning():
    plan = parse_mission_plan(
        {
            "mission": {
                "steps": [
                    {"type": "prime_offboard"},
                    {"type": "scan", "direction": "ccw", "yaw_deg": 180, "until": "locked", "timeout_s": 12},
                ]
            }
        }
    )
    warnings = lint_plan(plan)
    assert not any("timeout_s" in w for w in warnings)


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
