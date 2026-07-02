"""Gazebo camera → compressed image republisher for SITL simulation.

Subscribes to the raw sensor_msgs/Image that ros_gz_bridge publishes from
Gazebo's camera sensor and compresses + republishes it on the same topic the
Pi's camera_compressor_node normally owns
(/drone/camera/image_raw/compressed). This lets the ground-station YOLO node
and the full split-architecture pipeline run against Gazebo imagery with zero
code changes anywhere else.

Also optionally republishes the raw image on /drone/camera/image_raw so nodes
that subscribe to raw (e.g. YOLO in raw transport mode) work too.
"""

from __future__ import annotations

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image


class GzCamRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("gz_cam_republisher")

        self.declare_parameter("gz_image_topic", "/sim/camera/image_raw")
        self.declare_parameter("compressed_out_topic", "/drone/camera/image_raw/compressed")
        self.declare_parameter("raw_out_topic", "/drone/camera/image_raw")
        self.declare_parameter("republish_raw", True)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("report_period_s", 5.0)

        self.gz_topic = str(self.get_parameter("gz_image_topic").value)
        self.compressed_out = str(self.get_parameter("compressed_out_topic").value)
        self.raw_out = str(self.get_parameter("raw_out_topic").value)
        self.republish_raw = bool(self.get_parameter("republish_raw").value)
        self.jpeg_quality = max(1, min(100, int(self.get_parameter("jpeg_quality").value)))
        report_s = max(0.5, float(self.get_parameter("report_period_s").value))

        self.declare_parameter("stall_warn_s", 5.0)
        self.stall_warn_s = max(1.0, float(self.get_parameter("stall_warn_s").value))

        self.bridge = CvBridge()
        self.frames_in = 0
        self.frames_out = 0
        self._last_frame_mono = 0.0
        self._last_report_frames_in = 0
        self._last_report_mono = time.monotonic()
        self._stalled = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.compressed_pub = self.create_publisher(CompressedImage, self.compressed_out, qos)
        self.raw_pub = None
        if self.republish_raw:
            self.raw_pub = self.create_publisher(Image, self.raw_out, qos)

        self.sub = self.create_subscription(Image, self.gz_topic, self._on_image, qos)
        self.create_timer(report_s, self._report)

        self.get_logger().info(
            f"Gazebo cam republisher up | in={self.gz_topic}, "
            f"compressed_out={self.compressed_out}, raw_out={self.raw_out if self.republish_raw else 'disabled'}, "
            f"jpeg_quality={self.jpeg_quality}"
        )

    def _on_image(self, msg: Image) -> None:
        self.frames_in += 1
        self._last_frame_mono = time.monotonic()
        if self._stalled:
            self._stalled = False
            self.get_logger().warning("Gazebo camera stream recovered; frames flowing again.")
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(
                f"Failed to convert Gazebo image: {exc}", throttle_duration_sec=2.0
            )
            return

        ok, encoded = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            return

        out = CompressedImage()
        out.header = msg.header
        # Gazebo stamps frames with simulation time (starts near 0); the rest of
        # the stack (detection gate freshness checks, latency) runs on wall clock.
        # This node emulates the Pi camera, whose stamp IS wall clock, so restamp
        # here — otherwise every detection looks billions of seconds "stale".
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id or "camera_optical_frame"
        out.format = "jpeg"
        out.data = encoded.tobytes()
        self.compressed_pub.publish(out)

        if self.raw_pub is not None:
            raw_out = Image()
            raw_out.header = out.header
            raw_out.height = frame.shape[0]
            raw_out.width = frame.shape[1]
            raw_out.encoding = "bgr8"
            raw_out.is_bigendian = 0
            raw_out.step = frame.shape[1] * 3
            raw_out.data = frame.tobytes()
            self.raw_pub.publish(raw_out)

        self.frames_out += 1

    def _report(self) -> None:
        now = time.monotonic()
        window = max(1e-6, now - self._last_report_mono)
        fps = (self.frames_in - self._last_report_frames_in) / window
        self._last_report_frames_in = self.frames_in
        self._last_report_mono = now

        # Loud stall detection. The gz->ROS image bridge is known to stop
        # forwarding frames occasionally while Gazebo keeps rendering; when that
        # happens perception silently dies. Make it unmissable in the logs and
        # name the recovery so an operator (or a log-watching script) can act.
        frame_age = now - self._last_frame_mono if self._last_frame_mono > 0.0 else -1.0
        if self.frames_in > 0 and frame_age > self.stall_warn_s:
            self._stalled = True
            self.get_logger().error(
                f"Gazebo camera stream STALLED: no frames for {frame_age:.1f}s "
                f"(in={self.frames_in} total). ros_gz image bridge likely hung; "
                "restart the bridge/container to recover."
            )
            return

        self.get_logger().info(
            f"Gazebo cam republisher | in={self.frames_in}, out={self.frames_out}, fps={fps:.1f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GzCamRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
