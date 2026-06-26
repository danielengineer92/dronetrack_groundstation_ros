"""ROS 2 camera node. Publishes sensor_msgs/Image on /drone/camera/image_raw by default."""

import cv2
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from drone_diagnostics.node_diagnostics import NodeDiagnostics


class CameraNode(Node):
    MAX_CONSECUTIVE_FAILURES = 30

    def __init__(self) -> None:
        super().__init__("camera_node")

        # Use tolerant parameter reading so launch files cannot break the node
        # by passing integer-like values as strings, like "0" or "640".
        self.camera_index = self._get_int_param("camera_index", 0, aliases=("device_id",))
        self.frame_width = self._get_int_param("frame_width", 640, aliases=("width",))
        self.frame_height = self._get_int_param("frame_height", 480, aliases=("height",))
        self.fps = self._get_int_param("fps", 60, aliases=("frame_rate",))
        self.fourcc = self._get_str_param("fourcc", "MJPG").strip().upper()
        self.frame_id = self._get_str_param("frame_id", "camera_optical_frame")
        self.image_topic = self._get_str_param("image_topic", "/drone/camera/image_raw")
        self.camera_backend = self._get_str_param("camera_backend", "v4l2").strip().lower()
        self.buffer_size = self._get_int_param("buffer_size", 1)

        if self.fps <= 0:
            self.get_logger().warning(f"Invalid fps={self.fps}; using fps=30")
            self.fps = 30

        if self.frame_width <= 0:
            self.get_logger().warning(f"Invalid frame_width={self.frame_width}; using 640")
            self.frame_width = 640

        if self.frame_height <= 0:
            self.get_logger().warning(f"Invalid frame_height={self.frame_height}; using 480")
            self.frame_height = 480

        if self.buffer_size < 0:
            self.get_logger().warning(f"Invalid buffer_size={self.buffer_size}; using 1")
            self.buffer_size = 1

        backend_map = {
            "v4l2": cv2.CAP_V4L2,
            "any": cv2.CAP_ANY,
        }

        backend = backend_map.get(self.camera_backend, cv2.CAP_V4L2)
        if self.camera_backend not in backend_map:
            self.get_logger().warning(
                f"Unknown camera_backend={self.camera_backend!r}; falling back to v4l2"
            )
            self.camera_backend = "v4l2"

        self.bridge = CvBridge()

        self.get_logger().info(
            f"Opening camera | index={self.camera_index}, backend={self.camera_backend}"
        )

        self.cap = cv2.VideoCapture(int(self.camera_index), backend)

        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera index {self.camera_index}")

        if len(self.fourcc) == 4:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.frame_width))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.frame_height))
        self.cap.set(cv2.CAP_PROP_FPS, int(self.fps))

        if self.buffer_size > 0:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, int(self.buffer_size))

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(self.cap.get(cv2.CAP_PROP_FPS))
        fourcc_raw = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        actual_fourcc = "".join([chr((fourcc_raw >> 8 * i) & 0xFF) for i in range(4)])

        self.publisher = self.create_publisher(
            Image,
            self.image_topic,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(1.0 / float(self.fps), self.publish_frame)

        self._consecutive_failures = 0
        self.frame_count = 0

        self.diagnostics = NodeDiagnostics(
            self,
            heartbeat_period=5.0,
            stale_seconds=2.0,
        )
        self.diagnostics.add_output(self.image_topic, "camera_frames")

        self.get_logger().info(
            f"Camera node started | topic={self.image_topic}, "
            f"index={self.camera_index}, "
            f"requested={self.frame_width}x{self.frame_height}@{self.fps} {self.fourcc}, "
            f"actual={actual_w}x{actual_h}@{actual_fps:.1f} {actual_fourcc}, "
            f"backend={self.camera_backend}, "
            f"buffer_size={self.buffer_size}"
        )

    def _declare_dynamic_parameter(self, name: str, default_value):
        descriptor = ParameterDescriptor(dynamic_typing=True)

        try:
            self.declare_parameter(name, default_value, descriptor)
        except ParameterAlreadyDeclaredException:
            pass

        return self.get_parameter(name).value

    def _get_raw_param(self, name: str, default_value):
        value = self._declare_dynamic_parameter(name, default_value)

        if value is None:
            return default_value

        return value

    def _get_raw_param_with_aliases(self, name: str, default_value, aliases=()):
        value = self._get_raw_param(name, None)

        if value is not None:
            return value

        for alias in aliases:
            alias_value = self._get_raw_param(alias, None)

            if alias_value is not None:
                self.get_logger().warning(
                    f"Using alias parameter {alias!r} for {name!r}. "
                    f"Prefer {name!r} in launch files."
                )
                return alias_value

        return default_value

    def _get_int_param(self, name: str, default_value: int, aliases=()) -> int:
        value = self._get_raw_param_with_aliases(name, default_value, aliases)

        try:
            return int(value)
        except (TypeError, ValueError):
            try:
                # Handles values like "640.0" just in case.
                return int(float(value))
            except (TypeError, ValueError):
                self.get_logger().warning(
                    f"Invalid integer parameter {name}={value!r}; "
                    f"using default {default_value}"
                )
                return int(default_value)

    def _get_float_param(self, name: str, default_value: float, aliases=()) -> float:
        value = self._get_raw_param_with_aliases(name, default_value, aliases)

        try:
            return float(value)
        except (TypeError, ValueError):
            self.get_logger().warning(
                f"Invalid float parameter {name}={value!r}; "
                f"using default {default_value}"
            )
            return float(default_value)

    def _get_str_param(self, name: str, default_value: str, aliases=()) -> str:
        value = self._get_raw_param_with_aliases(name, default_value, aliases)

        if value is None:
            return str(default_value)

        return str(value)

    def publish_frame(self) -> None:
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        stamp = self.get_clock().now().to_msg()

        if not ret or frame is None:
            self._consecutive_failures += 1
            self.get_logger().warning(
                f"Failed to read frame ({self._consecutive_failures})"
            )

            if self._consecutive_failures >= self.MAX_CONSECUTIVE_FAILURES:
                raise RuntimeError("Camera read failures exceeded threshold")

            return

        self._consecutive_failures = 0

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        self.publisher.publish(msg)
        self.frame_count += 1

        self.diagnostics.mark_published(
            self.image_topic,
            summary=f"frames={self.frame_count}, size={msg.width}x{msg.height}",
        )

    def destroy_node(self) -> None:
        if hasattr(self, "cap") and self.cap is not None and self.cap.isOpened():
            self.cap.release()

        self.get_logger().info("Camera node shut down.")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = CameraNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        rclpy.logging.get_logger("camera_node").fatal(f"Fatal: {e}")

    finally:
        if node is not None:
            node.destroy_node()

        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
