"""Unit tests for target_state_math (pure, no rclpy). Run directly:

    python3 src/drone_tracker/test/test_target_state_math.py
"""

import math
import os
import sys

import numpy as np

try:
    from drone_tracker.target_state_math import (
        ConstantVelocityKF,
        measurement_covariance_ned,
        project_detection_ned,
    )
except ImportError:  # pragma: no cover - direct-run convenience
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from drone_tracker.target_state_math import (  # noqa: E402
        ConstantVelocityKF,
        measurement_covariance_ned,
        project_detection_ned,
    )


def _run_line(kf, *, v_n, v_e, n0=0.0, e0=0.0, dt=1 / 15, steps=90,
              noise_n=0.0, noise_e=0.0, seed=7, tail_avg=0):
    """Drive the KF along a line. tail_avg>0 -> also return the mean
    (vN, vE) over the last tail_avg steps (averages out the steady-state
    noise wander that any KF velocity estimate carries)."""
    rng = np.random.default_rng(seed)
    R = np.diag([0.05**2, 0.05**2])
    tail = []
    for k in range(steps):
        t = k * dt
        kf.predict(dt)
        kf.update(n0 + v_n * t + rng.normal(0, noise_n),
                  e0 + v_e * t + rng.normal(0, noise_e), R)
        if tail_avg and k >= steps - tail_avg:
            tail.append((kf.x[2], kf.x[3]))
    if tail_avg:
        arr = np.array(tail)
        return kf, float(arr[:, 0].mean()), float(arr[:, 1].mean())
    return kf


def test_projection_straight_ahead():
    n, e, los = project_detection_ned(
        drone_north_m=1.0, drone_east_m=2.0, yaw_rad=0.0,
        bearing_x_rad=0.0, bearing_y_rad=0.0, slant_distance_m=5.0)
    assert math.isclose(n, 6.0) and math.isclose(e, 2.0) and los == 0.0


def test_projection_foreshortens_and_rotates():
    # 30 deg down-elevation, LOS pointing due east
    n, e, _ = project_detection_ned(
        drone_north_m=0.0, drone_east_m=0.0, yaw_rad=math.pi / 2,
        bearing_x_rad=0.0, bearing_y_rad=math.radians(30), slant_distance_m=4.0)
    assert math.isclose(e, 4.0 * math.cos(math.radians(30)), rel_tol=1e-9)
    assert abs(n) < 1e-9


def test_r_anisotropy_alignment():
    # LOS due north: radial = N axis -> var_N >> var_E
    R = measurement_covariance_ned(los_angle_rad=0.0, range_m=8.0,
                                   sigma_bearing_rad=0.002,
                                   sigma_range_fraction=0.07,
                                   sigma_range_floor_m=0.1)
    assert R[0, 0] > 100 * R[1, 1]
    # LOS due east: swaps
    R = measurement_covariance_ned(los_angle_rad=math.pi / 2, range_m=8.0,
                                   sigma_bearing_rad=0.002,
                                   sigma_range_fraction=0.07,
                                   sigma_range_floor_m=0.1)
    assert R[1, 1] > 100 * R[0, 0]


def test_converges_exactly_without_noise():
    kf = ConstantVelocityKF(accel_noise_density=0.5)
    _run_line(kf, v_n=0.8, v_e=-0.4)
    assert abs(kf.x[2] - 0.8) < 0.02, kf.x
    assert abs(kf.x[3] + 0.4) < 0.02, kf.x
    assert abs(kf.heading_rad() - math.atan2(-0.4, 0.8)) < 0.03


def test_converges_on_noisy_line_time_averaged():
    # Instantaneous KF velocity wanders with measurement noise by design;
    # the 2 s time-average is the meaningful accuracy claim.
    kf = ConstantVelocityKF(accel_noise_density=0.5)
    _, vn, ve = _run_line(kf, v_n=0.8, v_e=-0.4, noise_n=0.05, noise_e=0.05,
                          steps=150, tail_avg=30)
    assert abs(vn - 0.8) < 0.15, (vn, ve)
    assert abs(ve + 0.4) < 0.15, (vn, ve)


def test_still_target_velocity_near_zero():
    kf = ConstantVelocityKF()
    _, vn, ve = _run_line(kf, v_n=0.0, v_e=0.0, noise_n=0.05, noise_e=0.05,
                          steps=150, tail_avg=30)
    assert math.hypot(vn, ve) < 0.12, (vn, ve)


def test_gating_rejects_outlier_and_keeps_state():
    kf = ConstantVelocityKF()
    _run_line(kf, v_n=0.5, v_e=0.0)
    v_before = kf.x[2]
    R = np.diag([0.05**2, 0.05**2])
    accepted = kf.update(kf.x[0] + 30.0, kf.x[1] - 30.0, R)  # absurd jump
    assert not accepted
    assert math.isclose(kf.x[2], v_before)


def test_prediction_leads_along_velocity():
    kf = ConstantVelocityKF()
    _run_line(kf, v_n=1.0, v_e=0.0)
    n_now = kf.x[0]
    n_pred, e_pred = kf.predict_position(0.5)
    assert 0.35 < (n_pred - n_now) < 0.65
    assert abs(e_pred - kf.x[1]) < 0.05


def test_covariance_shrinks_with_updates():
    kf = ConstantVelocityKF()
    kf.reset(0.0, 0.0)
    v0 = kf.velocity_std_m_s()
    _run_line(kf, v_n=0.3, v_e=0.3, noise_n=0.05, noise_e=0.05)
    assert kf.velocity_std_m_s() < v0 / 5


def test_nis_averages_two_for_consistent_filter():
    # Feed white noise matched to R: NIS must average ~2 (2-DOF chi-square).
    kf = ConstantVelocityKF(accel_noise_density=0.3)
    rng = np.random.default_rng(3)
    sigma = 0.05
    R = np.diag([sigma**2, sigma**2])
    nis = []
    for k in range(400):
        kf.predict(1 / 15)
        kf.update(rng.normal(0, sigma), rng.normal(0, sigma), R)
        if k > 30:
            nis.append(kf.last_nis)
    avg = sum(nis) / len(nis)
    assert 1.4 < avg < 2.6, avg


def test_nis_inflates_when_overconfident():
    # Claim 10x less noise than reality: NIS average must blow past 2.
    kf = ConstantVelocityKF(accel_noise_density=0.3)
    rng = np.random.default_rng(4)
    R = np.diag([0.005**2, 0.005**2])   # claimed
    nis = []
    for k in range(400):
        kf.predict(1 / 15)
        kf.update(rng.normal(0, 0.05), rng.normal(0, 0.05), R)  # actual 10x
        if k > 30:
            nis.append(kf.last_nis)
    assert sum(nis) / len(nis) > 4.0


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
