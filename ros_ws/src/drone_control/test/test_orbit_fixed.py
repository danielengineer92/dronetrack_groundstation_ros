"""Unit tests for the fixed-center orbit math + orbit_fixed plan verb. Run directly:

    python3 src/drone_control/test/test_orbit_fixed.py
"""

import math
import os
import sys

try:
    from drone_control.control_math import (
        orbit_fixed_setpoint,
        project_target_local_offset,
    )
    from drone_control.mission_plan import (
        MissionPlanError,
        lint_plan,
        parse_mission_plan_text,
    )
except ImportError:  # pragma: no cover - direct-run convenience
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from drone_control.control_math import (  # noqa: E402
        orbit_fixed_setpoint,
        project_target_local_offset,
    )
    from drone_control.mission_plan import (  # noqa: E402
        MissionPlanError,
        lint_plan,
        parse_mission_plan_text,
    )


def test_local_offset_straight_ahead_north():
    # Facing north, target dead center, 4 m slant on the horizon -> 4 m north.
    n, e = project_target_local_offset(
        yaw_rad=0.0, bearing_x_rad=0.0, bearing_y_rad=0.0, slant_distance_m=4.0)
    assert abs(n - 4.0) < 1e-9 and abs(e) < 1e-9


def test_local_offset_foreshortens_slant_by_elevation():
    # Target 60 deg below the optical axis: horizontal = 4 * cos(60) = 2 m.
    n, e = project_target_local_offset(
        yaw_rad=0.0, bearing_x_rad=0.0, bearing_y_rad=math.radians(60), slant_distance_m=4.0)
    assert abs(n - 2.0) < 1e-9 and abs(e) < 1e-9


def test_local_offset_yaw_plus_bearing_east():
    # Facing east (yaw 90), target dead ahead -> offset due east.
    n, e = project_target_local_offset(
        yaw_rad=math.pi / 2, bearing_x_rad=0.0, bearing_y_rad=0.0, slant_distance_m=3.0)
    assert abs(n) < 1e-9 and abs(e - 3.0) < 1e-9


def test_setpoint_sits_on_circle():
    n, e, _ = orbit_fixed_setpoint(
        drone_north=10.0, drone_east=0.0, center_north=0.0, center_east=0.0,
        radius_m=4.0, speed_m_s=1.0, lead_s=1.5)
    assert abs(math.hypot(n, e) - 4.0) < 1e-9


def test_setpoint_leads_clockwise_by_speed_over_radius():
    # Drone due north of center: theta=0. Lead = (1.0/4.0)*2.0 = 0.5 rad cw.
    n, e, _ = orbit_fixed_setpoint(
        drone_north=4.0, drone_east=0.0, center_north=0.0, center_east=0.0,
        radius_m=4.0, speed_m_s=1.0, lead_s=2.0)
    assert abs(n - 4.0 * math.cos(0.5)) < 1e-9
    assert abs(e - 4.0 * math.sin(0.5)) < 1e-9


def test_setpoint_counterclockwise_flag():
    n, e, _ = orbit_fixed_setpoint(
        drone_north=4.0, drone_east=0.0, center_north=0.0, center_east=0.0,
        radius_m=4.0, speed_m_s=1.0, lead_s=2.0, clockwise=False)
    assert abs(e - 4.0 * math.sin(-0.5)) < 1e-9


def test_setpoint_yaw_faces_center():
    # Setpoint east of center -> yaw looking west (-pi/2 ... exactly toward center).
    n, e, yaw = orbit_fixed_setpoint(
        drone_north=0.0, drone_east=4.0, center_north=0.0, center_east=0.0,
        radius_m=4.0, speed_m_s=0.0, lead_s=1.5)  # zero speed: no lead
    assert abs(n) < 1e-9 and abs(e - 4.0) < 1e-9
    assert abs(yaw - math.atan2(-4.0, 0.0)) < 1e-9


def test_setpoint_off_circle_projects_to_ring():
    # Drone 10 m out with zero lead: carrot is the nearest point on the ring.
    n, e, _ = orbit_fixed_setpoint(
        drone_north=10.0, drone_east=0.0, center_north=0.0, center_east=0.0,
        radius_m=4.0, speed_m_s=0.0, lead_s=1.5)
    assert abs(n - 4.0) < 1e-9 and abs(e) < 1e-9


def test_setpoint_degenerate_at_center_is_finite():
    n, e, yaw = orbit_fixed_setpoint(
        drone_north=0.0, drone_east=0.0, center_north=0.0, center_east=0.0,
        radius_m=4.0, speed_m_s=1.0, lead_s=1.5)
    assert all(math.isfinite(v) for v in (n, e, yaw))
    assert abs(math.hypot(n, e) - 4.0) < 1e-9


ORBIT_FIXED_PLAN = """
mission:
  name: t
  steps:
    - {type: takeoff, altitude_m: 3.2}
    - {type: prime_offboard, hold_s: 1.0}
    - {type: scan, until: locked, timeout_s: 60}
    - {type: orbit_fixed, radius_m: 4.0, speed_m_s: 1.0, revolutions: 3,
       center_samples: 5, sample_timeout_s: 20, descend_m: 1.2}
    - {type: land}
"""


def test_orbit_fixed_verb_parses_and_lints_clean():
    plan = parse_mission_plan_text(ORBIT_FIXED_PLAN)
    step = plan.steps[3]
    assert step.type == "orbit_fixed"
    assert step.state_name == "ORBIT_FIXED"
    assert step.get_float("center_samples", 0.0) == 5.0
    # No timeout_s lint nag: orbit_fixed derives its timeout from revolutions.
    assert lint_plan(plan) == []


def test_orbit_fixed_rejects_out_of_range_samples():
    bad = ORBIT_FIXED_PLAN.replace("center_samples: 5", "center_samples: 5000")
    try:
        parse_mission_plan_text(bad)
    except MissionPlanError:
        pass
    else:
        raise AssertionError("center_samples=5000 should fail range validation")


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
