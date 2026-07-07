"""Classic-CV red-ball detector: HSV colour threshold + circularity gate.

A saturated red ball is trivially separable by colour, and a colour blob stays
detectable at ranges where the small ncnn YOLO model's confidence collapses
(bench-measured: YOLO ~0.1 confidence on a 28 px ball at 2.5 m, while the same
ball segments cleanly as a 0.83-circular red blob). The catch with pure colour
is false positives — anything red in the scene. The circularity gate is what
makes it usable: red AND round = ball; red-but-irregular = rejected (bench:
the ball scored 0.83 while every red distractor scored 0.27-0.50).

Publishes the SAME DetectionArray/Detection contract as yolo_node.py (pixel +
normalized boxes, class_name, a [0,1] confidence), so it is a drop-in
alternative or cross-check on `/…/vision/detections`. Confidence is the blob's
circularity, which is exactly the quantity a downstream min_confidence gate
should threshold on for a colour detector.

cv2 comes from apt python3-opencv (already used by the camera/dashboard nodes).
Red wraps hue 0/180 in OpenCV HSV, so two hue bands are unioned.
"""

from __future__ import annotations

import time
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from drone_interfaces.msg import Detection, DetectionArray
from dronetrack_perception.color_detection import RedDetectParams, find_red_circles, red_mask


class ColorDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("color_detection_node")

        self.declare_parameter("class_name", "red_ball")
        self.declare_parameter("image_transport", "compressed")  # "compressed" | "raw"
        self.declare_parameter("image_topic", "/drone/camera/image_raw")
        self.declare_parameter("compressed_image_topic", "/drone/camera/image_raw/compressed")
        self.declare_parameter("detections_topic", "/groundstation/vision/detections")
        self.declare_parameter("reliable_detections", False)
        self.declare_parameter("max_detections", 10)
        self.declare_parameter("max_fps", 30.0)

        # HSV red bands (two, because red wraps 0/180). Defaults validated on a
        # live bench frame; tune per lighting with the debug mask (see below).
        self.declare_parameter("hue_lo_1", 0)
        self.declare_parameter("hue_hi_1", 10)
        self.declare_parameter("hue_lo_2", 170)
        self.declare_parameter("hue_hi_2", 180)
        self.declare_parameter("sat_min", 90)
        self.declare_parameter("val_min", 60)

        # Blob acceptance.
        self.declare_parameter("min_area_px", 12.0)      # ignore specks
        self.declare_parameter("min_circularity", 0.65)  # ball ~0.83, distractors <=0.50
        self.declare_parameter("min_diameter_px", 4.0)
        self.declare_parameter("morph_kernel", 3)        # opening kernel (0 disables)

        self.declare_parameter("publish_debug_mask", False)
        self.declare_parameter("debug_mask_topic", "/groundstation/vision/color_mask")
        self.declare_parameter("report_period_s", 5.0)

        self.class_name = str(self.get_parameter("class_name").value)
        self.transport = str(self.get_parameter("image_transport").value).strip().lower()
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.compressed_image_topic = str(self.get_parameter("compressed_image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.max_detections = max(1, int(self.get_parameter("max_detections").value))
        self.max_fps = float(self.get_parameter("max_fps").value)

        self.params = RedDetectParams(
            hue_lo_1=int(self.get_parameter("hue_lo_1").value),
            hue_hi_1=int(self.get_parameter("hue_hi_1").value),
            hue_lo_2=int(self.get_parameter("hue_lo_2").value),
            hue_hi_2=int(self.get_parameter("hue_hi_2").value),
            sat_min=int(self.get_parameter("sat_min").value),
            val_min=int(self.get_parameter("val_min").value),
            min_area_px=float(self.get_parameter("min_area_px").value),
            min_circularity=float(self.get_parameter("min_circularity").value),
            min_diameter_px=float(self.get_parameter("min_diameter_px").value),
            morph_kernel=int(self.get_parameter("morph_kernel").value),
        )
        self.min_circularity = self.params.min_circularity

        self.publish_debug_mask = bool(self.get_parameter("publish_debug_mask").value)
        self.report_period_s = float(self.get_parameter("report_period_s").value)

        self.min_process_interval_s = 1.0 / self.max_fps if self.max_fps > 0 else 0.0
        self._next_process_time = 0.0

        self.bridge = CvBridge()

        image_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable = bool(self.get_parameter("reliable_detections").value)
        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        if self.transport == "raw":
            self.sub = self.create_subscription(Image, self.image_topic, self._on_raw, image_qos)
            sub_topic = self.image_topic
        else:
            self.sub = self.create_subscription(
                CompressedImage, self.compressed_image_topic, self._on_compressed, image_qos)
            sub_topic = self.compressed_image_topic

        self.detection_pub = self.create_publisher(DetectionArray, self.detections_topic, detection_qos)
        self.debug_pub = None
        if self.publish_debug_mask:
            self.debug_pub = self.create_publisher(
                CompressedImage, str(self.get_parameter("debug_mask_topic").value), image_qos)

        self._frames = 0
        self._published = 0
        self._last_det_count = 0
        self._last_best_circ = 0.0
        self.create_timer(self.report_period_s, self._report)

        self.get_logger().info(
            f"Color detector up | transport={self.transport}, in={sub_topic}, "
            f"out={self.detections_topic}, class='{self.class_name}', "
            f"min_circularity={self.min_circularity}, max_fps={self.max_fps}")

    # ---- throttle --------------------------------------------------------
    def _should_process(self) -> bool:
        if self.min_process_interval_s <= 0.0:
            return True
        now = time.monotonic()
        if now < self._next_process_time:
            return False
        slot = self._next_process_time + self.min_process_interval_s
        self._next_process_time = slot if slot > now - self.min_process_interval_s else now + self.min_process_interval_s
        return True

    # ---- image callbacks -------------------------------------------------
    def _on_compressed(self, msg: CompressedImage) -> None:
        if not self._should_process():
            return
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"decode CompressedImage failed: {exc}", throttle_duration_sec=2.0)
            return
        self._process(frame, msg.header.stamp)

    def _on_raw(self, msg: Image) -> None:
        if not self._should_process():
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"convert Image failed: {exc}", throttle_duration_sec=2.0)
            return
        self._process(frame, msg.header.stamp)

    # ---- detection -------------------------------------------------------
    def _process(self, frame, stamp) -> None:
        self._frames += 1
        try:
            img_h, img_w = frame.shape[:2]
            blobs = find_red_circles(frame, self.params)[:self.max_detections]

            arr = DetectionArray()
            arr.stamp = stamp
            arr.image_width = img_w
            arr.image_height = img_h
            dets: List[Detection] = []
            for b in blobs:
                d = Detection()
                d.stamp = stamp
                d.class_id = 0
                d.class_name = self.class_name
                d.confidence = float(min(1.0, b.circularity))
                d.pixel_center_x = int(round(b.cx))
                d.pixel_center_y = int(round(b.cy))
                # Square box from the enclosing circle — a round ball has ~1:1
                # aspect, which the tracker's aspect gate (0.5-2.0) accepts.
                d.pixel_width = int(round(b.diameter))
                d.pixel_height = int(round(b.diameter))
                d.center_x = float(b.cx) / float(img_w)
                d.center_y = float(b.cy) / float(img_h)
                d.width = float(b.diameter) / float(img_w)
                d.height = float(b.diameter) / float(img_h)
                dets.append(d)

            arr.detections = dets
            arr.count = len(dets)
            self.detection_pub.publish(arr)
            self._published += 1
            self._last_det_count = len(dets)
            self._last_best_circ = blobs[0].circularity if blobs else 0.0

            if self.debug_pub is not None:
                dbg = self.bridge.cv2_to_compressed_imgmsg(red_mask(frame, self.params))
                dbg.header.stamp = stamp
                self.debug_pub.publish(dbg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"color detection error: {exc}", throttle_duration_sec=2.0)

    def _report(self) -> None:
        self.get_logger().info(
            f"Color | frames={self._frames}, published={self._published}, "
            f"last_detections={self._last_det_count}, best_circularity={self._last_best_circ:.2f}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
