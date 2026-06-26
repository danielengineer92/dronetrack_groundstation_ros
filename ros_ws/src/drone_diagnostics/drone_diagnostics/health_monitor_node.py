"""ROS 2 health monitor for the drone vision pipeline."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from drone_interfaces.msg import ControlCommand, DetectionArray, DroneTelemetry, TargetError


@dataclass
class TopicHealth:
    name: str
    label: str
    stale_seconds: float
    count: int = 0
    last_time: Optional[float] = None
    last_summary: str = ""
    _last_rate_time: float = 0.0
    _last_rate_count: int = 0

    def mark(self, summary: str = "") -> None:
        self.count += 1
        self.last_time = time.monotonic()
        if summary:
            self.last_summary = summary

    def age(self, now: float) -> Optional[float]:
        if self.last_time is None:
            return None
        return now - self.last_time

    def rate(self, now: float) -> float:
        if self._last_rate_time <= 0.0:
            self._last_rate_time = now
            self._last_rate_count = self.count
            return 0.0
        elapsed = now - self._last_rate_time
        if elapsed <= 0.0:
            return 0.0
        rate = (self.count - self._last_rate_count) / elapsed
        self._last_rate_time = now
        self._last_rate_count = self.count
        return rate


class HealthMonitorNode(Node):
    """Monitor topic freshness, rates, and graph connections from one place."""

    def __init__(self) -> None:
        super().__init__('health_monitor_node')

        self.declare_parameter('image_is_compressed', False)
        self.declare_parameter('image_topic', '/drone/camera/image_raw')
        self.declare_parameter('detections_topic', '/drone/vision/detections')
        self.declare_parameter('target_error_topic', '/drone/tracking/target_error')
        self.declare_parameter('telemetry_topic', '/drone/telemetry')
        self.declare_parameter('control_command_topic', '/drone/control/command')
        self.declare_parameter('mavsdk_command_status_topic', '/drone/mavsdk/command_status')
        self.declare_parameter('monitor_mavsdk_command_status', False)
        self.declare_parameter('heartbeat_period', 2.0)
        self.declare_parameter('stale_seconds', 2.0)

        self._image_is_compressed = bool(self.get_parameter('image_is_compressed').value)
        self._image_topic = str(self.get_parameter('image_topic').value)
        self._detections_topic = str(self.get_parameter('detections_topic').value)
        self._target_error_topic = str(self.get_parameter('target_error_topic').value)
        self._telemetry_topic = str(self.get_parameter('telemetry_topic').value)
        self._control_command_topic = str(self.get_parameter('control_command_topic').value)
        self._mavsdk_command_status_topic = str(self.get_parameter('mavsdk_command_status_topic').value)
        self._monitor_mavsdk_command_status = bool(self.get_parameter('monitor_mavsdk_command_status').value)
        self._heartbeat_period = float(self.get_parameter('heartbeat_period').value)
        self._stale_seconds = float(self.get_parameter('stale_seconds').value)
        self._start_time = time.monotonic()

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._topics: dict[str, TopicHealth] = {
            self._image_topic: TopicHealth(self._image_topic, 'camera', self._stale_seconds),
            self._detections_topic: TopicHealth(self._detections_topic, 'detections', self._stale_seconds),
            self._target_error_topic: TopicHealth(self._target_error_topic, 'target_error', self._stale_seconds),
            self._telemetry_topic: TopicHealth(self._telemetry_topic, 'telemetry', self._stale_seconds),
            self._control_command_topic: TopicHealth(self._control_command_topic, 'control_command', self._stale_seconds),
        }
        if self._monitor_mavsdk_command_status:
            self._topics[self._mavsdk_command_status_topic] = TopicHealth(
                self._mavsdk_command_status_topic, 'mavsdk_command_status', self._stale_seconds
            )

        if self._image_is_compressed:
            self._image_sub = self.create_subscription(
                CompressedImage,
                self._image_topic,
                self._compressed_image_callback,
                qos_profile_sensor_data,
            )
        else:
            self._image_sub = self.create_subscription(
                Image,
                self._image_topic,
                self._image_callback,
                qos_profile_sensor_data,
            )
        self._detections_sub = self.create_subscription(
            DetectionArray,
            self._detections_topic,
            self._detections_callback,
            reliable_qos,
        )
        self._target_error_sub = self.create_subscription(
            TargetError,
            self._target_error_topic,
            self._target_error_callback,
            reliable_qos,
        )
        self._telemetry_sub = self.create_subscription(
            DroneTelemetry,
            self._telemetry_topic,
            self._telemetry_callback,
            reliable_qos,
        )
        self._command_sub = self.create_subscription(
            ControlCommand,
            self._control_command_topic,
            self._command_callback,
            reliable_qos,
        )
        self._mavsdk_command_status_sub = None
        if self._monitor_mavsdk_command_status:
            self._mavsdk_command_status_sub = self.create_subscription(
                String,
                self._mavsdk_command_status_topic,
                self._mavsdk_command_status_callback,
                reliable_qos,
            )

        self._timer = self.create_timer(self._heartbeat_period, self._report_health)

        self.get_logger().info(
            'Health monitor started | '
            f'image={self._image_topic}, detections={self._detections_topic}, '
            f'image_is_compressed={self._image_is_compressed}, '
            f'target_error={self._target_error_topic}, telemetry={self._telemetry_topic}, '
            f'control_command={self._control_command_topic}, '
            f'mavsdk_command_status={self._mavsdk_command_status_topic if self._monitor_mavsdk_command_status else "disabled"}, '
            f'heartbeat={self._heartbeat_period:.1f}s, stale>{self._stale_seconds:.1f}s'
        )

    def _image_callback(self, msg: Image) -> None:
        self._topics[self._image_topic].mark(
            f'{msg.width}x{msg.height}, frame_id={msg.header.frame_id or "none"}'
        )

    def _compressed_image_callback(self, msg: CompressedImage) -> None:
        self._topics[self._image_topic].mark(
            f'compressed {len(msg.data)} bytes, format={msg.format or "none"}, frame_id={msg.header.frame_id or "none"}'
        )

    def _detections_callback(self, msg: DetectionArray) -> None:
        self._topics[self._detections_topic].mark(f'count={msg.count}')

    def _target_error_callback(self, msg: TargetError) -> None:
        self._topics[self._target_error_topic].mark(
            f'state={msg.tracking_state}, visible={msg.target_visible}, age_seen={msg.time_since_last_seen:.2f}s'
        )

    def _telemetry_callback(self, msg: DroneTelemetry) -> None:
        self._topics[self._telemetry_topic].mark(
            f'connected={msg.connected}, status={msg.connection_status}, batt={msg.battery_remaining_percent:.1f}%'
        )

    def _command_callback(self, msg: ControlCommand) -> None:
        self._topics[self._control_command_topic].mark(
            f'type={msg.command_type}, executed={msg.executed}, status={msg.execution_status}'
        )

    def _mavsdk_command_status_callback(self, msg: String) -> None:
        if self._mavsdk_command_status_topic in self._topics:
            self._topics[self._mavsdk_command_status_topic].mark(msg.data)

    def _report_health(self) -> None:
        now = time.monotonic()
        uptime = now - self._start_time
        warnings: list[str] = []
        parts: list[str] = [f'uptime={uptime:.1f}s']

        for topic, stats in self._topics.items():
            age = stats.age(now)
            rate = stats.rate(now)
            publishers = self.count_publishers(topic)
            subscribers = self.count_subscribers(topic)
            age_text = 'never' if age is None else f'{age:.2f}s'
            summary = f', {stats.last_summary}' if stats.last_summary else ''

            parts.append(
                f'{stats.label}: count={stats.count}, rate={rate:.1f}Hz, age={age_text}, '
                f'publishers={publishers}, subscribers={subscribers}{summary}'
            )

            if publishers == 0:
                warnings.append(f'NO_PUBLISHER {topic}')
            if age is None:
                warnings.append(f'NO_DATA {topic}')
            elif age > stats.stale_seconds:
                warnings.append(f'STALE {topic} age={age:.2f}s')

        if warnings:
            self.get_logger().warning('SYSTEM HEALTH WARN | ' + ' | '.join(parts) + ' | ' + '; '.join(warnings))
        else:
            self.get_logger().info('SYSTEM HEALTH OK | ' + ' | '.join(parts))

    def destroy_node(self) -> None:
        self.get_logger().info('Health monitor node shut down.')
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = HealthMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger('health_monitor_node').fatal(f'Fatal: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
