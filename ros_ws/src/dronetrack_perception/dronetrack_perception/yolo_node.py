"""YOLO object detection node (runs ON THE LAPTOP ground station).

Adapted from dronetrack_pi_ros/src/drone_yolo/drone_yolo/yolo_node.py. The
detection logic is unchanged; the differences for the split architecture are:

  - Subscribes to the COMPRESSED camera stream from the Pi by default
    (`/drone/camera/image_raw/compressed`, sensor_msgs/CompressedImage), so we do
    not push raw full-res frames over Wi-Fi. Set image_transport:=raw to use
    sensor_msgs/Image instead (e.g. for local laptop-camera dev).

  - Publishes detections on an OUTBOUND ground-station topic
    (`/groundstation/vision/detections`) which the Pi's detection_gate_node
    validates before any Pi node trusts it. We do NOT publish onto the Pi's
    internal /drone/vision/detections directly.

  - Detection header stamp is copied straight from the incoming image header, so
    the Pi can measure end-to-end latency (requires NTP/chrony clock sync).

Message types (Detection/DetectionArray) are reused unchanged from
drone_interfaces to stay wire-compatible with the Pi.
"""

import time
from typing import List, Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from drone_interfaces.msg import Detection, DetectionArray


class YoloNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_node")

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("device", "cpu")          # "cpu", "cuda:0", "mps", ...
        self.declare_parameter("input_size", 640)
        self.declare_parameter("max_detections", 20)
        self.declare_parameter("half_precision", False)
        self.declare_parameter("verbose", False)
        self.declare_parameter("target_class", "")
        self.declare_parameter("image_transport", "compressed")  # "compressed" | "raw"
        self.declare_parameter("image_topic", "/drone/camera/image_raw")
        self.declare_parameter("compressed_image_topic", "/drone/camera/image_raw/compressed")
        self.declare_parameter("detections_topic", "/groundstation/vision/detections")
        self.declare_parameter("process_every_n_frames", 1)
        self.declare_parameter("max_fps", 30.0)

        self.model_path = str(self.get_parameter("model_path").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.device = str(self.get_parameter("device").value)
        self.input_size = int(self.get_parameter("input_size").value)
        self.max_detections = int(self.get_parameter("max_detections").value)
        self.half_precision = bool(self.get_parameter("half_precision").value)
        self.verbose = bool(self.get_parameter("verbose").value)
        self.target_class = str(self.get_parameter("target_class").value).strip().lower()
        self.transport = str(self.get_parameter("image_transport").value).strip().lower()
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.compressed_image_topic = str(self.get_parameter("compressed_image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.process_every_n_frames = max(1, int(self.get_parameter("process_every_n_frames").value))
        self.max_fps = max(0.0, float(self.get_parameter("max_fps").value))
        self.min_process_interval_s = 1.0 / self.max_fps if self.max_fps > 0.0 else 0.0

        # Ultralytics requires imgsz to be a multiple of the model stride (32).
        # Round up rather than erroring per-frame with a confusing message.
        rounded_input = max(32, ((self.input_size + 31) // 32) * 32)
        if rounded_input != self.input_size:
            self.get_logger().warning(
                f"input_size={self.input_size} is not a multiple of 32; using {rounded_input}.")
            self.input_size = rounded_input
        self.confidence_threshold = min(1.0, max(0.0, self.confidence_threshold))
        self.iou_threshold = min(1.0, max(0.0, self.iou_threshold))

        self._received_frames = 0
        self._published_arrays = 0
        self._next_process_time = 0.0
        self._report_window_start = time.monotonic()
        self._target_class_ids: Optional[List[int]] = None
        self.bridge = CvBridge()
        self.model = None
        self.model_loaded = False
        self.inference_count = 0
        self.total_inference_time = 0.0

        self._load_model()
        self._target_class_ids = self._resolve_target_class_ids()

        image_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=1)
        # BEST_EFFORT outbound: laptop->Pi over Wi-Fi, freshest-wins, never blocks.
        detection_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                   history=HistoryPolicy.KEEP_LAST, depth=1)

        if self.transport == "raw":
            self.sub = self.create_subscription(Image, self.image_topic, self._on_raw, image_qos)
            sub_topic = self.image_topic
        else:
            self.sub = self.create_subscription(
                CompressedImage, self.compressed_image_topic, self._on_compressed, image_qos)
            sub_topic = self.compressed_image_topic

        self.detection_pub = self.create_publisher(DetectionArray, self.detections_topic, detection_qos)
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f"YOLO (ground station) up | transport={self.transport}, in={sub_topic}, "
            f"out={self.detections_topic}, model={self.model_path}, device={self.device}, "
            f"target_class='{self.target_class or 'all'}', max_fps={self.max_fps:g}")

    # ---- model -----------------------------------------------------------
    def _load_model(self) -> None:
        try:
            from ultralytics import YOLO
            import os
            # Expand ~ / env vars: launch args aren't shell-expanded, so a path like
            # ~/models/foo.pt would otherwise be taken literally and fail to load.
            self.model_path = os.path.expanduser(os.path.expandvars(self.model_path))
            self.get_logger().info(f"Loading YOLO model: {self.model_path}")
            self.model = YOLO(self.model_path)
            dummy = np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8)
            self.model.predict(dummy, conf=self.confidence_threshold, iou=self.iou_threshold,
                               device=self.device, half=self.half_precision, verbose=False,
                               imgsz=self.input_size)
            self.model_loaded = True
            self.get_logger().info("YOLO model loaded.")
        except ImportError:
            self.get_logger().error("ultralytics not installed. pip install ultralytics")
            self.model_loaded = False
        except Exception as exc:  # noqa: BLE001
            if self.device not in ("", "cpu"):
                # A missing/broken GPU stack should degrade to slower perception,
                # not take the whole perception pipeline down.
                self.get_logger().error(
                    f"Failed to load YOLO model on device '{self.device}' ({exc}); retrying on cpu.")
                self.device = "cpu"
                self.half_precision = False  # half is CUDA-only
                self._load_model()
                return
            self.get_logger().error(f"Failed to load YOLO model: {exc}")
            self.model_loaded = False

    def _resolve_target_class_ids(self) -> Optional[List[int]]:
        if not self.target_class or not self.model_loaded or self.model is None:
            return None
        names = getattr(self.model, "names", {})
        matches = [int(cid) for cid, cname in names.items()
                   if str(cname).strip().lower() == self.target_class]
        if not matches:
            self.get_logger().warning(
                f"target_class={self.target_class!r} not in model names; running all classes.")
            return None
        return matches

    # ---- image callbacks -------------------------------------------------
    def _on_compressed(self, msg: CompressedImage) -> None:
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to decode CompressedImage: {exc}", throttle_duration_sec=2.0)
            return
        self._process(frame, msg.header.stamp)

    def _on_raw(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to convert Image: {exc}", throttle_duration_sec=2.0)
            return
        self._process(frame, msg.header.stamp)

    # ---- inference -------------------------------------------------------
    def _process(self, frame, stamp) -> None:
        self._received_frames += 1
        if not self.model_loaded:
            return
        if self._received_frames % self.process_every_n_frames != 0:
            return
        # Anchored-schedule throttle. A naive "now - last >= interval" check drops
        # every other frame when max_fps equals the camera rate: frame periods
        # jitter around the interval, so half of them land a hair early. Anchoring
        # the next slot to the previous slot (not to the last processed frame)
        # absorbs that jitter and delivers the configured rate exactly.
        now = time.monotonic()
        if self.min_process_interval_s > 0.0:
            if now < self._next_process_time:
                return
            slot = self._next_process_time + self.min_process_interval_s
            # If we fell far behind (slow inference, stalled stream), re-anchor to
            # now instead of racing to catch up on a backlog of virtual slots.
            self._next_process_time = slot if slot > now - self.min_process_interval_s else now + self.min_process_interval_s

        try:
            img_h, img_w = frame.shape[:2]
            t0 = time.time()
            kwargs = {"conf": self.confidence_threshold, "iou": self.iou_threshold,
                      "device": self.device, "half": self.half_precision, "verbose": self.verbose,
                      "imgsz": self.input_size, "max_det": self.max_detections}
            if self._target_class_ids is not None:
                kwargs["classes"] = self._target_class_ids
            results = self.model.predict(frame, **kwargs)
            self.total_inference_time += time.time() - t0
            self.inference_count += 1

            arr = DetectionArray()
            arr.stamp = stamp            # preserve Pi camera stamp for latency measurement
            arr.image_width = img_w
            arr.image_height = img_h
            dets: List[Detection] = []

            if results and len(results) > 0 and results[0].boxes is not None:
                boxes = results[0].boxes
                for i in range(len(boxes)):
                    cls_id = int(boxes.cls[i].item())
                    class_name = self.model.names.get(cls_id, f"class_{cls_id}")
                    # Class filtering is handled at inference via `classes=` when the
                    # target_class resolves to a model id. We deliberately do NOT
                    # re-filter by name here: if target_class wasn't found in the
                    # model we fall back to "all classes" (see _resolve_target_class_ids),
                    # and a name re-filter would wrongly drop everything in that case.
                    x1, y1, x2, y2 = boxes.xyxy[i].tolist()
                    pcx, pcy = int((x1 + x2) / 2.0), int((y1 + y2) / 2.0)
                    pw, ph = int(x2 - x1), int(y2 - y1)
                    d = Detection()
                    d.stamp = stamp
                    d.class_id = cls_id
                    d.class_name = class_name
                    d.confidence = float(boxes.conf[i].item())
                    d.pixel_center_x, d.pixel_center_y = pcx, pcy
                    d.pixel_width, d.pixel_height = pw, ph
                    d.center_x = float(pcx) / float(img_w)
                    d.center_y = float(pcy) / float(img_h)
                    d.width = float(pw) / float(img_w)
                    d.height = float(ph) / float(img_h)
                    dets.append(d)

            arr.detections = dets
            arr.count = len(dets)
            self.detection_pub.publish(arr)
            self._published_arrays += 1
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"YOLO inference error: {exc}", throttle_duration_sec=2.0)

    def _report(self) -> None:
        if self.inference_count == 0:
            self.get_logger().info(
                f"YOLO | no inferences yet, frames_received={self._received_frames}")
            return
        avg = self.total_inference_time / self.inference_count
        capacity_fps = 1.0 / avg if avg > 0.0 else 0.0
        elapsed = max(1e-6, time.monotonic() - self._report_window_start)
        processed_fps = self.inference_count / elapsed
        self.get_logger().info(
            f"YOLO | processed_fps={processed_fps:.1f}, capacity_fps={capacity_fps:.1f}, "
            f"avg_inference={avg*1000:.1f}ms, cap={self.max_fps:g}, "
            f"frames={self._received_frames}, arrays={self._published_arrays}")
        self.inference_count = 0
        self.total_inference_time = 0.0
        self._report_window_start = time.monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
