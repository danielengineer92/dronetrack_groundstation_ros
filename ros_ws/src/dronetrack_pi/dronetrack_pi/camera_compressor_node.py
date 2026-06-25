"""Pi-side raw Image -> CompressedImage bridge for the split architecture."""

from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class CameraCompressorNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_compressor_node")

        self.declare_parameter("image_topic", "/drone/camera/image_raw")
        self.declare_parameter(
            "compressed_image_topic",
            "/drone/camera/image_raw/compressed",
        )
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("report_period_s", 5.0)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.compressed_image_topic = str(
            self.get_parameter("compressed_image_topic").value
        )
        self.jpeg_quality = max(1, min(100, int(self.get_parameter("jpeg_quality").value)))
        self.report_period_s = max(0.5, float(self.get_parameter("report_period_s").value))

        self.bridge = CvBridge()
        self.frames_in = 0
        self.frames_out = 0
        self.encode_failures = 0
        self.last_jpeg_bytes = 0

        self.pub = self.create_publisher(
            CompressedImage,
            self.compressed_image_topic,
            qos_profile_sensor_data,
        )
        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_timer(self.report_period_s, self._report)

        self.get_logger().info(
            "Camera compressor up | "
            f"in={self.image_topic}, out={self.compressed_image_topic}, "
            f"jpeg_quality={self.jpeg_quality}, qos=sensor_data"
        )

    def _on_image(self, msg: Image) -> None:
        self.frames_in += 1
        # No laptop subscribed -> don't waste Pi CPU JPEG-encoding frames nobody reads.
        if self.pub.get_subscription_count() == 0:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
        except Exception as exc:  # noqa: BLE001
            self.encode_failures += 1
            self.get_logger().warning(
                f"Failed to encode camera frame: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        if not ok:
            self.encode_failures += 1
            self.get_logger().warning(
                "Failed to encode camera frame",
                throttle_duration_sec=2.0,
            )
            return

        out = CompressedImage()
        out.header = msg.header
        out.format = "jpeg"
        out.data = encoded.tobytes()
        self.pub.publish(out)

        self.frames_out += 1
        self.last_jpeg_bytes = len(out.data)

    def _report(self) -> None:
        self.get_logger().info(
            "Camera compressor | "
            f"in={self.frames_in}, out={self.frames_out}, "
            f"last_jpeg={self.last_jpeg_bytes} bytes, failures={self.encode_failures}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraCompressorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
