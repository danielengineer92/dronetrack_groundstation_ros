"""World-frame target state estimator (position, velocity, heading).

Fuses the tracker's monocular measurements (bearing + estimated range) with
drone telemetry (local-NED pose + yaw) into a constant-velocity Kalman filter,
publishing where the target IS, how fast it is moving, in what direction, and
where it will be prediction_horizon_s from now.

The estimate is model-free about the target's path: a circle, a straight
line, or stop-and-go all work — CV-KF only assumes velocity changes smoothly
at the accel_noise_density scale. All inputs are measurements (never our own
commands), and each detection is paired with the drone pose AT ITS CAPTURE
STAMP (pair_at_capture_stamp), so neither our commands nor our own rotation
during the vision delay contaminate the estimate; see target_state_math.py
for the anisotropic range/bearing noise treatment.
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
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
        # Pair each detection with the drone pose AT ITS CAPTURE STAMP
        # (interpolated from a short telemetry history) instead of the latest
        # telemetry. The bearing is ~0.35 s old when it arrives; projecting it
        # with the CURRENT yaw leaks our own rotation into the target position
        # (up to ~0.1 rad of oscillating LOS error while yawing at 0.3 rad/s),
        # which corrupts the KF velocity and feeds back through the yaw FF —
        # measured in the closed-loop model as THE dominant cause of the
        # side-to-side wobble at high target LOS rate (p2p error 0.41 -> 0.06
        # with pairing fixed). Runtime-tunable for live A/B.
        self.declare_parameter("pair_at_capture_stamp", True)

        g = self.get_parameter
        self.publish_rate = max(1.0, float(g("publish_rate_hz").value))
        self.pred_horizon = float(g("prediction_horizon_s").value)
        self.coast_timeout = float(g("coast_timeout_s").value)
        self.reset_timeout = float(g("reset_timeout_s").value)
        self.min_updates = int(g("min_updates_for_valid").value)
        self.max_tel_age = float(g("max_telemetry_age_s").value)
        self.pair_at_capture = bool(g("pair_at_capture_stamp").value)
        self.sigma_bearing = float(g("sigma_bearing_rad").value)
        self.sigma_range_frac = float(g("sigma_range_fraction").value)
        self.sigma_range_floor = float(g("sigma_range_floor_m").value)

        self.kf = ConstantVelocityKF(float(g("accel_noise_density").value))
        self.last_meas_time = 0.0
        self.last_kf_time = 0.0
        self.last_telemetry: Optional[DroneTelemetry] = None
        self.last_telemetry_time = 0.0
        # Short pose history for capture-stamp pairing: (stamp_s, north, east,
        # yaw). ~2 s at 10 Hz telemetry; each entry is only appended when the
        # local position is valid, so lookups never return an invalid pose.
        self._pose_history: deque[tuple[float, float, float, float]] = deque(maxlen=40)
        self.meas_count = 0
        self.gated_count = 0
        self.skipped_no_pose = 0
        self.paired_at_capture = 0
        self.paired_fallback = 0
        # Rolling NIS window (~20 s at 15 Hz) for the consistency metric.
        self._nis_window: list[float] = []

        self.add_on_set_parameters_callback(self._on_set_parameters)

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
        if msg.local_position_valid:
            stamp_s = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
            if stamp_s > 0.0:
                self._pose_history.append((
                    stamp_s,
                    float(msg.local_position_north),
                    float(msg.local_position_east),
                    float(msg.yaw),
                ))

    def _pose_at(self, stamp_s: float) -> Optional[tuple[float, float, float]]:
        """Drone (north, east, yaw) at ``stamp_s``, interpolated from the pose
        history. None when the stamp falls outside the buffered window (caller
        falls back to the latest telemetry — the pre-pairing behaviour)."""
        hist = self._pose_history
        if len(hist) < 2 or stamp_s <= 0.0:
            return None
        if stamp_s < hist[0][0] or stamp_s > hist[-1][0] + 0.5:
            return None  # older than the buffer / absurdly in the future
        prev = hist[0]
        for cur in hist:
            if cur[0] >= stamp_s:
                t0, n0, e0, y0 = prev
                t1, n1, e1, y1 = cur
                span = t1 - t0
                f = 0.0 if span <= 1e-6 else (stamp_s - t0) / span
                f = min(max(f, 0.0), 1.0)
                dyaw = math.atan2(math.sin(y1 - y0), math.cos(y1 - y0))
                return (n0 + f * (n1 - n0), e0 + f * (e1 - e0), y0 + f * dyaw)
            prev = cur
        # Stamp is newer than the last sample (≤0.5 s): extrapolate by holding.
        t1, n1, e1, y1 = hist[-1]
        return (n1, e1, y1)

    def _on_target_error(self, msg: TargetError) -> None:
        if not (msg.target_visible and msg.distance_valid):
            return
        tel = self.last_telemetry
        now = time.monotonic()
        if (tel is None or not tel.local_position_valid
                or now - self.last_telemetry_time > self.max_tel_age):
            self.skipped_no_pose += 1
            return

        # Pair the (delayed) bearing with the drone pose at its CAPTURE stamp,
        # not the pose now — see pair_at_capture_stamp. Falls back to the
        # latest telemetry when the stamp cannot be resolved from the history.
        drone_n = float(tel.local_position_north)
        drone_e = float(tel.local_position_east)
        drone_yaw = float(tel.yaw)
        if self.pair_at_capture:
            cap_s = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
            pose = self._pose_at(cap_s)
            if pose is not None:
                drone_n, drone_e, drone_yaw = pose
                self.paired_at_capture += 1
            else:
                self.paired_fallback += 1

        north, east, los = project_detection_ned(
            drone_north_m=drone_n,
            drone_east_m=drone_e,
            yaw_rad=drone_yaw,
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
        accepted = self.kf.update(north, east, R)
        self._nis_window.append(self.kf.last_nis)
        if len(self._nis_window) > 300:
            self._nis_window.pop(0)
        if accepted:
            self.last_meas_time = now
            self.meas_count += 1
        else:
            self.gated_count += 1

    def _on_set_parameters(self, params) -> SetParametersResult:
        """Runtime tuning: accel_noise_density (q) and the noise model, so
        sweeps don't need node restarts (params are otherwise cached at init —
        the same foot-gun the tracker intrinsics had)."""
        for p in params:
            if p.name == "pair_at_capture_stamp":
                self.pair_at_capture = bool(p.value)
                self.get_logger().info(f"pair_at_capture_stamp -> {self.pair_at_capture}")
            elif p.name == "accel_noise_density":
                if float(p.value) <= 0.0:
                    return SetParametersResult(successful=False,
                                               reason="accel_noise_density must be > 0")
                self.kf.q = float(p.value)
                self._nis_window.clear()
                self.get_logger().info(f"accel_noise_density -> {self.kf.q}")
            elif p.name in ("sigma_bearing_rad", "sigma_range_fraction",
                            "sigma_range_floor_m", "prediction_horizon_s"):
                if float(p.value) <= 0.0:
                    return SetParametersResult(successful=False,
                                               reason=f"{p.name} must be > 0")
                attr = {"sigma_bearing_rad": "sigma_bearing",
                        "sigma_range_fraction": "sigma_range_frac",
                        "sigma_range_floor_m": "sigma_range_floor",
                        "prediction_horizon_s": "pred_horizon"}[p.name]
                setattr(self, attr, float(p.value))
                self._nis_window.clear()
        return SetParametersResult(successful=True)

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
            f"skipped_no_pose={self.skipped_no_pose}, "
            f"paired@capture={self.paired_at_capture}, "
            f"pair_fallback={self.paired_fallback}, kf_updates={self.kf.updates}, "
            f"speed={self.kf.speed():.2f} m/s, "
            f"heading={math.degrees(self.kf.heading_rad()):.0f} deg, "
            f"pos_std={self.kf.position_std_m():.2f} m, "
            f"vel_std={self.kf.velocity_std_m_s():.2f} m/s, "
            f"avg_nis={sum(self._nis_window)/max(len(self._nis_window),1):.2f} "
            f"(healthy~2.0), gated_pct="
            f"{100.0*self.gated_count/max(self.meas_count+self.gated_count,1):.1f}%"
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
