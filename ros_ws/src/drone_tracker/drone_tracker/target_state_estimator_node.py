"""World-frame target state estimator (position, velocity, heading).

Fuses the tracker's monocular measurements (bearing + estimated range) with
drone telemetry (local-NED pose + yaw) into a constant-velocity Kalman filter,
publishing where the target IS, how fast it is moving, in what direction, and
where it will be prediction_horizon_s from now.

The estimate is model-free about the target's path: a circle, a straight
line, or stop-and-go all work — CV-KF only assumes velocity changes smoothly
at the accel_noise_density scale. All inputs are measurements (never our own
commands), so our own maneuvering does not contaminate the estimate; see
target_state_math.py for the anisotropic range/bearing noise treatment.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_interfaces.msg import DroneTelemetry, TargetError, TargetState
from drone_tracker.target_state_math import (
    ConstantVelocityKF,
    measurement_covariance_ned,
    project_detection_ned,
)


class TargetStateEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("target_state_estimator_node")

        self.declare_parameter("target_error_topic", "/drone/tracking/target_error")
        self.declare_parameter("telemetry_topic", "/drone/telemetry")
        self.declare_parameter("target_state_topic", "/drone/tracking/target_state")
        self.declare_parameter("publish_rate_hz", 10.0)
        # CV process noise: how hard the target may maneuver (m^2/s^3).
        self.declare_parameter("accel_noise_density", 0.3)
        # Measurement noise model (see target_state_math).
        self.declare_parameter("sigma_bearing_rad", 0.004)
        self.declare_parameter("sigma_range_fraction", 0.08)
        self.declare_parameter("sigma_range_floor_m", 0.15)
        self.declare_parameter("prediction_horizon_s", 0.5)
        # Freshness: coast (predict-only) up to coast_timeout_s without an
        # accepted measurement, then invalidate; reset after reset_timeout_s.
        self.declare_parameter("coast_timeout_s", 2.0)
        self.declare_parameter("reset_timeout_s", 4.0)
        self.declare_parameter("min_updates_for_valid", 8)
        # Telemetry older than this cannot anchor a world-frame measurement.
        self.declare_parameter("max_telemetry_age_s", 0.5)

        g = self.get_parameter
        self.publish_rate = max(1.0, float(g("publish_rate_hz").value))
        self.pred_horizon = float(g("prediction_horizon_s").value)
        self.coast_timeout = float(g("coast_timeout_s").value)
        self.reset_timeout = float(g("reset_timeout_s").value)
        self.min_updates = int(g("min_updates_for_valid").value)
        self.max_tel_age = float(g("max_telemetry_age_s").value)
        self.sigma_bearing = float(g("sigma_bearing_rad").value)
        self.sigma_range_frac = float(g("sigma_range_fraction").value)
        self.sigma_range_floor = float(g("sigma_range_floor_m").value)

        self.kf = ConstantVelocityKF(float(g("accel_noise_density").value))
        self.last_meas_time = 0.0
        self.last_kf_time = 0.0
        self.last_telemetry: Optional[DroneTelemetry] = None
        self.last_telemetry_time = 0.0
        self.meas_count = 0
        self.gated_count = 0
        self.skipped_no_pose = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(TargetError, str(g("target_error_topic").value),
                                 self._on_target_error, qos)
        self.create_subscription(DroneTelemetry, str(g("telemetry_topic").value),
                                 self._on_telemetry, qos)
        self.state_pub = self.create_publisher(
            TargetState, str(g("target_state_topic").value), qos)

        self.create_timer(1.0 / self.publish_rate, self._publish)
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            f"Target state estimator up | q={self.kf.q}, "
            f"pred_horizon={self.pred_horizon}s, "
            f"out={str(g('target_state_topic').value)}"
        )

    # -- inputs ----------------------------------------------------------

    def _on_telemetry(self, msg: DroneTelemetry) -> None:
        self.last_telemetry = msg
        self.last_telemetry_time = time.monotonic()

    def _on_target_error(self, msg: TargetError) -> None:
        if not (msg.target_visible and msg.distance_valid):
            return
        tel = self.last_telemetry
        now = time.monotonic()
        if (tel is None or not tel.local_position_valid
                or now - self.last_telemetry_time > self.max_tel_age):
            self.skipped_no_pose += 1
            return

        north, east, los = project_detection_ned(
            drone_north_m=float(tel.local_position_north),
            drone_east_m=float(tel.local_position_east),
            yaw_rad=float(tel.yaw),
            bearing_x_rad=float(msg.bearing_x_rad),
            bearing_y_rad=float(msg.bearing_y_rad),
            slant_distance_m=float(msg.distance_m),
        )
        R = measurement_covariance_ned(
            los_angle_rad=los,
            range_m=float(msg.distance_m),
            sigma_bearing_rad=self.sigma_bearing,
            sigma_range_fraction=self.sigma_range_frac,
            sigma_range_floor_m=self.sigma_range_floor,
        )

        if (self.last_meas_time > 0.0
                and now - self.last_meas_time > self.reset_timeout):
            self.kf.initialized = False  # long gap: stale kinematics, restart

        self._advance(now)
        if self.kf.update(north, east, R):
            self.last_meas_time = now
            self.meas_count += 1
        else:
            self.gated_count += 1

    # -- filter time base --------------------------------------------------

    def _advance(self, now: float) -> None:
        if self.kf.initialized and self.last_kf_time > 0.0:
            self.kf.predict(min(now - self.last_kf_time, 1.0))
        self.last_kf_time = now

    # -- output ------------------------------------------------------------

    def _publish(self) -> None:
        now = time.monotonic()
        msg = TargetState()
        msg.stamp = self.get_clock().now().to_msg()
        age = now - self.last_meas_time if self.last_meas_time > 0.0 else -1.0
        fresh = self.kf.initialized and 0.0 <= age <= self.coast_timeout
        msg.valid = bool(fresh and self.kf.updates >= self.min_updates)
        msg.coasting = bool(fresh and age > 1.5 / self.publish_rate)
        msg.measurement_age_s = float(age)
        msg.updates = int(self.kf.updates)
        if self.kf.initialized:
            self._advance(now)
            msg.position_north = float(self.kf.x[0])
            msg.position_east = float(self.kf.x[1])
            msg.velocity_north = float(self.kf.x[2])
            msg.velocity_east = float(self.kf.x[3])
            msg.speed = self.kf.speed()
            msg.heading_rad = self.kf.heading_rad()
            pn, pe = self.kf.predict_position(self.pred_horizon)
            msg.predicted_north, msg.predicted_east = float(pn), float(pe)
            msg.prediction_horizon_s = float(self.pred_horizon)
            msg.position_std_m = self.kf.position_std_m()
            msg.velocity_std_m_s = self.kf.velocity_std_m_s()
        self.state_pub.publish(msg)

    def _report(self) -> None:
        self.get_logger().info(
            f"Target state | valid_updates={self.meas_count}, gated={self.gated_count}, "
            f"skipped_no_pose={self.skipped_no_pose}, kf_updates={self.kf.updates}, "
            f"speed={self.kf.speed():.2f} m/s, "
            f"heading={math.degrees(self.kf.heading_rad()):.0f} deg, "
            f"pos_std={self.kf.position_std_m():.2f} m, "
            f"vel_std={self.kf.velocity_std_m_s():.2f} m/s"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TargetStateEstimatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
