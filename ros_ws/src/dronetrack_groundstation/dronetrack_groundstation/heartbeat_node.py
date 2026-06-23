"""Ground-station heartbeat publisher (runs ON THE LAPTOP).

Publishes a steady GroundStationHeartbeat so the Pi watchdog can tell the laptop
link is alive. This is deliberately decoupled from YOLO: the heartbeat keeps
beating even when there are zero detections, so the Pi can distinguish
"link up, no target" from "link down".

It measures perception health by watching the outbound detections topic (FPS and
freshness) and reports CPU load if psutil is available. These are advisory; the
Pi never trusts them for safety, but they are handy on the dashboard.
"""

import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_interfaces.msg import DetectionArray
from dronetrack_msgs.msg import GroundStationHeartbeat

try:
    import psutil  # optional
except ImportError:  # pragma: no cover
    psutil = None


class HeartbeatNode(Node):
    def __init__(self) -> None:
        super().__init__("groundstation_heartbeat_node")

        self.declare_parameter("heartbeat_topic", "/groundstation/heartbeat")
        self.declare_parameter("detections_topic", "/groundstation/vision/detections")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("station_id", socket.gethostname())
        self.declare_parameter("software_version", "0.1.0")
        self.declare_parameter("perception_stale_s", 1.0)

        self.heartbeat_topic = str(self.get_parameter("heartbeat_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.station_id = str(self.get_parameter("station_id").value).strip() or socket.gethostname()
        self.software_version = str(self.get_parameter("software_version").value)
        self.perception_stale_s = float(self.get_parameter("perception_stale_s").value)

        self._sequence = 0
        self._last_detection_mono = None
        self._detection_count = 0
        self._fps_window_start = time.monotonic()
        self._fps_window_count = 0
        self._measured_fps = 0.0

        lossy_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=1)

        self.pub = self.create_publisher(GroundStationHeartbeat, self.heartbeat_topic, lossy_qos)
        self.create_subscription(DetectionArray, self.detections_topic, self._on_detections, lossy_qos)
        self.create_timer(1.0 / max(0.5, self.rate_hz), self._beat)

        self.get_logger().info(
            f"Heartbeat up | topic={self.heartbeat_topic}, rate={self.rate_hz}Hz, "
            f"station_id={self.station_id}")

    def _on_detections(self, msg: DetectionArray) -> None:
        self._last_detection_mono = time.monotonic()
        self._fps_window_count += 1
        self._detection_count += 1

    def _beat(self) -> None:
        now = time.monotonic()
        elapsed = now - self._fps_window_start
        if elapsed >= 1.0:
            self._measured_fps = self._fps_window_count / elapsed
            self._fps_window_start = now
            self._fps_window_count = 0

        perception_fresh = (
            self._last_detection_mono is not None
            and (now - self._last_detection_mono) <= self.perception_stale_s
        )

        self._sequence += 1
        msg = GroundStationHeartbeat()
        msg.stamp = self.get_clock().now().to_msg()
        msg.sequence = self._sequence
        msg.station_id = self.station_id
        msg.software_version = self.software_version
        msg.perception_fps = float(self._measured_fps)
        msg.cpu_percent = float(psutil.cpu_percent()) if psutil is not None else -1.0
        msg.perception_ok = bool(perception_fresh)
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
