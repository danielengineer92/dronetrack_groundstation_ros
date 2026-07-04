"""Pure math for the world-frame target state estimator.

Constant-velocity Kalman filter over horizontal PX4-local-NED position, fed by
monocular detections projected into the world:

    horizontal = slant_distance * cos(bearing_y)          (F1 foreshortening)
    target_N   = drone_N + horizontal * cos(yaw + bearing_x)
    target_E   = drone_E + horizontal * sin(yaw + bearing_x)

State x = [N, E, vN, vE]. The filter exists because the two measurement axes
are wildly unequal: cross-range (bearing) is ~0.1 deg accurate, while radial
range comes from `k / bbox_px` — at 9 m a ONE PIXEL bbox jump is ~0.6 m. The
measurement covariance R is therefore built anisotropic in the line-of-sight
frame (sigma_range >> sigma_cross) and rotated into NED per sample; naive
isotropic R would let range noise bleed into the good axis.

Kept free of rclpy so it unit-tests with plain python + numpy
(see test/test_target_state_math.py).
"""

from __future__ import annotations

import math

import numpy as np


def project_detection_ned(
    *,
    drone_north_m: float,
    drone_east_m: float,
    yaw_rad: float,
    bearing_x_rad: float,
    bearing_y_rad: float,
    slant_distance_m: float,
) -> tuple[float, float, float]:
    """Monocular detection -> horizontal NED target position.

    Returns (north_m, east_m, los_angle_rad); los is needed to orient R.
    """
    horizontal = float(slant_distance_m) * math.cos(float(bearing_y_rad))
    los = float(yaw_rad) + float(bearing_x_rad)
    return (
        float(drone_north_m) + horizontal * math.cos(los),
        float(drone_east_m) + horizontal * math.sin(los),
        los,
    )


def measurement_covariance_ned(
    *,
    los_angle_rad: float,
    range_m: float,
    sigma_bearing_rad: float,
    sigma_range_fraction: float,
    sigma_range_floor_m: float,
) -> np.ndarray:
    """Anisotropic 2x2 measurement covariance, rotated LOS -> NED.

    Radial sigma grows with range (monocular k/diam: a fixed pixel jitter is a
    fixed FRACTION of range, so sigma_r ~ fraction * range, floored).
    Cross-range sigma = range * bearing noise (small-angle arc length).
    """
    r = max(0.1, float(range_m))
    sigma_radial = max(float(sigma_range_floor_m), float(sigma_range_fraction) * r)
    sigma_cross = max(1e-3, r * float(sigma_bearing_rad))
    c, s = math.cos(float(los_angle_rad)), math.sin(float(los_angle_rad))
    rot = np.array([[c, -s], [s, c]])
    diag = np.diag([sigma_radial**2, sigma_cross**2])
    return rot @ diag @ rot.T


class ConstantVelocityKF:
    """4-state (N, E, vN, vE) constant-velocity Kalman filter.

    accel_noise_density (m^2/s^3) is the white-acceleration spectral density —
    how much the target is allowed to maneuver. ~0.5 suits a walking/running
    ground target; raise it for agile targets (slower convergence, less lag).
    """

    #: Mahalanobis^2 gate for a 2-DOF measurement (chi-square 99%).
    GATE = 9.21

    def __init__(self, accel_noise_density: float = 0.5):
        self.q = float(accel_noise_density)
        self.x = np.zeros(4)
        self.P = np.eye(4)
        self.initialized = False
        self.updates = 0

    def reset(self, north_m: float, east_m: float) -> None:
        self.x = np.array([float(north_m), float(east_m), 0.0, 0.0])
        # Confident about where it is, ignorant about how it moves.
        self.P = np.diag([1.0, 1.0, 25.0, 25.0])
        self.initialized = True
        self.updates = 1

    def predict(self, dt: float) -> None:
        if not self.initialized or dt <= 0.0:
            return
        dt = float(dt)
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        # Discrete white-noise-acceleration Q (per horizontal axis).
        q11 = self.q * dt**3 / 3.0
        q12 = self.q * dt**2 / 2.0
        q22 = self.q * dt
        Q = np.array([
            [q11, 0.0, q12, 0.0],
            [0.0, q11, 0.0, q12],
            [q12, 0.0, q22, 0.0],
            [0.0, q12, 0.0, q22],
        ])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, north_m: float, east_m: float, R: np.ndarray) -> bool:
        """Fuse one position measurement. Returns False if gated out."""
        if not self.initialized:
            self.reset(north_m, east_m)
            return True
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        z = np.array([float(north_m), float(east_m)])
        innov = z - H @ self.x
        S = H @ self.P @ H.T + R
        S_inv = np.linalg.inv(S)
        if float(innov @ S_inv @ innov) > self.GATE:
            return False  # outlier (bbox glitch, misdetection) — coast instead
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ innov
        I_KH = np.eye(4) - K @ H
        # Joseph form: keeps P symmetric positive-definite under rounding.
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.updates += 1
        return True

    # -- outputs ---------------------------------------------------------

    def speed(self) -> float:
        return float(math.hypot(self.x[2], self.x[3]))

    def heading_rad(self) -> float:
        """Direction of motion, NED convention (0 = north, +CW to east)."""
        return float(math.atan2(self.x[3], self.x[2]))

    def predict_position(self, horizon_s: float) -> tuple[float, float]:
        h = float(horizon_s)
        return float(self.x[0] + self.x[2] * h), float(self.x[1] + self.x[3] * h)

    def position_std_m(self) -> float:
        return float(math.sqrt(max(0.0, (self.P[0, 0] + self.P[1, 1]) / 2.0)))

    def velocity_std_m_s(self) -> float:
        return float(math.sqrt(max(0.0, (self.P[2, 2] + self.P[3, 3]) / 2.0)))
