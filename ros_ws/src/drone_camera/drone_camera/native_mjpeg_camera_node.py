"""Native V4L2 MJPEG camera node.

Publishes the camera's compressed MJPEG frames directly as
sensor_msgs/CompressedImage. This avoids the slower path:

    V4L2 MJPG -> OpenCV decoded BGR Image -> JPEG re-encode

The node uses v4l2-ctl for the streaming path because OpenCV returns decoded
frames from cap.read(), even when CAP_PROP_FOURCC is set to MJPG.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import threading
import time
from collections import deque

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from drone_diagnostics.node_diagnostics import NodeDiagnostics


class NativeMjpegCameraNode(Node):
    """Stream native camera MJPEG bytes onto a ROS CompressedImage topic."""

    def __init__(self) -> None:
        super().__init__("native_mjpeg_camera_node")

        self.camera_index = self._get_int_param("camera_index", 0, aliases=("device_id",))
        self.camera_device = self._get_str_param("camera_device", "")
        self.frame_width = self._get_int_param("frame_width", 640, aliases=("width",))
        self.frame_height = self._get_int_param("frame_height", 480, aliases=("height",))
        self.fps = self._get_float_param("fps", 60.0, aliases=("frame_rate",))
        self.fourcc = self._get_str_param("fourcc", "MJPG").strip().upper()
        self.frame_id = self._get_str_param("frame_id", "camera_optical_frame")
        self.compressed_image_topic = self._get_str_param(
            "compressed_image_topic",
            "/drone/camera/image_raw/compressed",
        )
        self.v4l2_ctl_path = self._get_str_param("v4l2_ctl_path", "v4l2-ctl")
        self.buffer_count = self._get_int_param("buffer_count", 4)
        self.chunk_size = self._get_int_param("chunk_size", 65536)
        self.stream_count = self._get_int_param("stream_count", 2147483647)
        self.report_period_s = self._get_float_param("report_period_s", 5.0)
        self.print_v4l2_formats = self._get_bool_param("print_v4l2_formats", False)

        if not self.camera_device:
            self.camera_device = f"/dev/video{self.camera_index}"

        if self.frame_width <= 0:
            self.get_logger().warning(f"Invalid frame_width={self.frame_width}; using 640")
            self.frame_width = 640
        if self.frame_height <= 0:
            self.get_logger().warning(f"Invalid frame_height={self.frame_height}; using 480")
            self.frame_height = 480
        if self.fps <= 0.0:
            self.get_logger().warning(f"Invalid fps={self.fps}; using 60")
            self.fps = 60.0
        if self.fourcc != "MJPG":
            self.get_logger().warning(
                f"Native MJPEG node only supports MJPG; requested {self.fourcc!r}, using MJPG"
            )
            self.fourcc = "MJPG"
        if self.buffer_count <= 0:
            self.get_logger().warning(f"Invalid buffer_count={self.buffer_count}; using 4")
            self.buffer_count = 4
        if self.chunk_size < 1024:
            self.get_logger().warning(f"Invalid chunk_size={self.chunk_size}; using 65536")
            self.chunk_size = 65536
        if self.stream_count <= 0:
            self.stream_count = 2147483647
        if self.report_period_s < 0.5:
            self.report_period_s = 0.5

        resolved_v4l2 = shutil.which(self.v4l2_ctl_path)
        if resolved_v4l2 is None:
            raise RuntimeError(
                f"{self.v4l2_ctl_path!r} not found; install v4l-utils on the Pi"
            )
        self.v4l2_ctl_path = resolved_v4l2

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(
            CompressedImage,
            self.compressed_image_topic,
            qos,
        )

        self.diagnostics = NodeDiagnostics(
            self,
            heartbeat_period=5.0,
            stale_seconds=2.0,
        )
        self.diagnostics.add_output(self.compressed_image_topic, "native_mjpeg_frames")

        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._pending = bytearray()
        self._stderr_lines: deque[str] = deque(maxlen=20)
        self._process: subprocess.Popen[bytes] | None = None
        self._reader_thread: threading.Thread | None = None
        self._stderr_thread: threading.Thread | None = None
        self._frames = 0
        self._bytes_total = 0
        self._dropped_prefix_bytes = 0
        self._last_jpeg_bytes = 0
        self._encode_failures = 0
        self._last_report_time = time.monotonic()
        self._last_report_frames = 0
        self._last_report_bytes = 0
        self._last_frame_time = 0.0

        if self.print_v4l2_formats:
            self._log_v4l2_formats()

        self._start_stream()
        self.create_timer(self.report_period_s, self._report)

        self.get_logger().info(
            "Native MJPEG camera node started | "
            f"device={self.camera_device}, requested="
            f"{self.frame_width}x{self.frame_height}@{self.fps:g} {self.fourcc}, "
            f"topic={self.compressed_image_topic}, frame_id={self.frame_id}, "
            f"buffer_count={self.buffer_count}, chunk_size={self.chunk_size}"
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
        return default_value if value is None else value

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
                return int(float(value))
            except (TypeError, ValueError):
                self.get_logger().warning(
                    f"Invalid integer parameter {name}={value!r}; using default {default_value}"
                )
                return int(default_value)

    def _get_float_param(self, name: str, default_value: float, aliases=()) -> float:
        value = self._get_raw_param_with_aliases(name, default_value, aliases)
        try:
            return float(value)
        except (TypeError, ValueError):
            self.get_logger().warning(
                f"Invalid float parameter {name}={value!r}; using default {default_value}"
            )
            return float(default_value)

    def _get_bool_param(self, name: str, default_value: bool, aliases=()) -> bool:
        value = self._get_raw_param_with_aliases(name, default_value, aliases)
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        text = str(value).strip().lower()
        if text in ("1", "true", "yes", "on"):
            return True
        if text in ("0", "false", "no", "off"):
            return False
        self.get_logger().warning(
            f"Invalid bool parameter {name}={value!r}; using default {default_value}"
        )
        return bool(default_value)

    def _get_str_param(self, name: str, default_value: str, aliases=()) -> str:
        value = self._get_raw_param_with_aliases(name, default_value, aliases)
        return str(default_value) if value is None else str(value)

    def _log_v4l2_formats(self) -> None:
        try:
            completed = subprocess.run(
                [self.v4l2_ctl_path, "-d", self.camera_device, "--list-formats-ext"],
                check=False,
                text=True,
                capture_output=True,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Could not list V4L2 formats: {exc}")
            return
        if completed.stdout:
            self.get_logger().info("V4L2 formats:\n" + completed.stdout.strip())
        if completed.stderr:
            self.get_logger().warning("V4L2 format listing stderr:\n" + completed.stderr.strip())

    def _start_stream(self) -> None:
        command = [
            self.v4l2_ctl_path,
            "-d",
            self.camera_device,
            f"--set-fmt-video=width={self.frame_width},height={self.frame_height},pixelformat={self.fourcc}",
            f"--set-parm={self.fps:g}",
            f"--stream-mmap={self.buffer_count}",
            f"--stream-count={self.stream_count}",
            "--stream-to=-",
        ]
        self.get_logger().info("Starting native MJPEG stream | " + " ".join(command))
        self._process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._stderr_thread = threading.Thread(target=self._stderr_loop, daemon=True)
        self._reader_thread.start()
        self._stderr_thread.start()

    def _stderr_loop(self) -> None:
        process = self._process
        if process is None or process.stderr is None:
            return
        while not self._stop_event.is_set():
            line = process.stderr.readline()
            if not line:
                break
            text = line.decode("utf-8", errors="replace").rstrip()
            if text:
                with self._lock:
                    self._stderr_lines.append(text)

    def _reader_loop(self) -> None:
        process = self._process
        if process is None or process.stdout is None:
            return
        try:
            while not self._stop_event.is_set():
                chunk = os.read(process.stdout.fileno(), self.chunk_size)
                if not chunk:
                    if process.poll() is not None:
                        break
                    continue
                self._pending.extend(chunk)
                for jpeg in self._extract_jpegs():
                    self._publish_jpeg(jpeg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Native MJPEG reader failed: {exc}")

    def _extract_jpegs(self):
        while True:
            soi = self._pending.find(b"\xff\xd8")
            if soi < 0:
                if len(self._pending) > 2:
                    with self._lock:
                        self._dropped_prefix_bytes += len(self._pending) - 2
                    del self._pending[:-2]
                return
            if soi > 0:
                with self._lock:
                    self._dropped_prefix_bytes += soi
                del self._pending[:soi]

            eoi = self._pending.find(b"\xff\xd9", 2)
            if eoi < 0:
                return

            frame = bytes(self._pending[: eoi + 2])
            del self._pending[: eoi + 2]
            yield frame

    def _publish_jpeg(self, jpeg: bytes) -> None:
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = jpeg
        self.publisher.publish(msg)

        with self._lock:
            self._frames += 1
            self._bytes_total += len(jpeg)
            self._last_jpeg_bytes = len(jpeg)
            self._last_frame_time = time.monotonic()
            frames = self._frames

        self.diagnostics.mark_published(
            self.compressed_image_topic,
            summary=f"frames={frames}, jpeg={len(jpeg)} bytes",
        )

    def _report(self) -> None:
        now = time.monotonic()
        with self._lock:
            frames = self._frames
            bytes_total = self._bytes_total
            last_jpeg_bytes = self._last_jpeg_bytes
            dropped_prefix_bytes = self._dropped_prefix_bytes
            stderr_lines = list(self._stderr_lines)
            last_frame_time = self._last_frame_time

        elapsed = max(1e-6, now - self._last_report_time)
        frame_delta = frames - self._last_report_frames
        byte_delta = bytes_total - self._last_report_bytes
        rate = frame_delta / elapsed
        avg_jpeg_kb = (byte_delta / max(frame_delta, 1)) / 1024.0
        frame_age = "never" if last_frame_time <= 0.0 else f"{now - last_frame_time:.2f}s"
        subscribers = self.count_subscribers(self.compressed_image_topic)

        self.get_logger().info(
            "Native MJPEG camera | "
            f"rate={rate:.1f}Hz, frames={frames}, subscribers={subscribers}, "
            f"last_jpeg={last_jpeg_bytes} bytes, avg_jpeg={avg_jpeg_kb:.1f}KB, "
            f"frame_age={frame_age}, dropped_prefix_bytes={dropped_prefix_bytes}"
        )
        if stderr_lines:
            self.get_logger().debug("Recent v4l2-ctl stderr | " + " | ".join(stderr_lines[-3:]))

        process = self._process
        if process is not None and process.poll() is not None:
            self.get_logger().error(f"v4l2-ctl exited with code {process.returncode}")

        self._last_report_time = now
        self._last_report_frames = frames
        self._last_report_bytes = bytes_total

    def destroy_node(self) -> None:
        self._stop_event.set()
        process = self._process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
        for thread in (self._reader_thread, self._stderr_thread):
            if thread is not None:
                thread.join(timeout=1.0)
        self.get_logger().info("Native MJPEG camera node shut down.")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = NativeMjpegCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger("native_mjpeg_camera_node").fatal(f"Fatal: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
