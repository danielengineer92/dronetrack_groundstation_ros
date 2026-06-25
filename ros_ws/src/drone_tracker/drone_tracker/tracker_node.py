"""
Target tracking node.

Subscribes:
    /drone/vision/detections

Publishes:
    /drone/tracking/target_error

Safety-critical behavior:
    - detection_visible_now is the truth from the latest /drone/vision/detections message.
    - target_visible is what downstream control should treat as a fresh usable target.
    - A short bounded grace period prevents LOCKED/LOST flicker on 1-frame YOLO drops.
    - Tracker memory may help choose between detections, but memory can only keep
      target_visible=True for the configured grace window, never forever.
    - Once the grace window expires, target_visible=False and error_x/error_y are zero.
"""

import math
import time
from enum import Enum
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_interfaces.msg import Detection, DetectionArray, TargetError
from drone_diagnostics.node_diagnostics import NodeDiagnostics


class TrackingState(Enum):
    SEARCHING = "SEARCHING"
    LOCKED = "LOCKED"
    LOST = "LOST"


class ExponentialMovingAverage:
    def __init__(self, alpha: float = 0.4) -> None:
        self.alpha = alpha
        self.value: Optional[float] = None

    def update(self, new_value: float) -> float:
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1.0 - self.alpha) * self.value
        return self.value

    def reset(self) -> None:
        self.value = None


class TrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("tracker_node")

        # Core tracking parameters
        self.declare_parameter("target_class", "person")
        self.declare_parameter("min_confidence", 0.4)
        self.declare_parameter("reacquisition_timeout", 5.0)
        self.declare_parameter("smoothing_alpha", 0.4)
        self.declare_parameter("target_area_min", 0.001)
        self.declare_parameter("target_area_max", 0.8)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("proximity_threshold", 0.15)

        # Stale stream timeout means /drone/vision/detections stopped arriving at all.
        self.declare_parameter("stale_detection_timeout", 2.5)

        # Grace/debounce settings. These prevent LOCKED/LOST flicker when YOLO
        # drops one or two frames even though the target is still actually there.
        self.declare_parameter("missed_detection_threshold", 2)
        self.declare_parameter("coast_timeout", 2.0)
        self.declare_parameter("lock_confirm_frames", 1)

        # Monocular distance estimation. Use distance_calibration_k first because
        # it is easy to calibrate with a tape measure: K = distance_m * bbox_diameter_px.
        self.declare_parameter("estimate_distance", True)
        self.declare_parameter("distance_calibration_k", 0.0)
        self.declare_parameter("ball_diameter_m", 0.20)
        self.declare_parameter("focal_length_px", 0.0)
        self.declare_parameter("horizontal_fov_deg", 70.0)
        self.declare_parameter("vertical_fov_deg", 55.0)
        self.declare_parameter("image_width_px", 640)
        self.declare_parameter("image_height_px", 480)
        self.declare_parameter("min_distance_m", 0.25)
        self.declare_parameter("max_distance_m", 12.0)
        self.declare_parameter("distance_filter_alpha", 0.25)

        # Topics
        self.declare_parameter("detections_topic", "/drone/vision/detections")
        self.declare_parameter("target_error_topic", "/drone/tracking/target_error")

        self.target_class = str(self.get_parameter("target_class").value).strip().lower()
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.reacquisition_timeout = float(self.get_parameter("reacquisition_timeout").value)
        self.smoothing_alpha = float(self.get_parameter("smoothing_alpha").value)
        self.target_area_min = float(self.get_parameter("target_area_min").value)
        self.target_area_max = float(self.get_parameter("target_area_max").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.proximity_threshold = float(self.get_parameter("proximity_threshold").value)
        self.stale_detection_timeout = float(self.get_parameter("stale_detection_timeout").value)
        self.missed_detection_threshold = int(self.get_parameter("missed_detection_threshold").value)
        self.coast_timeout = float(self.get_parameter("coast_timeout").value)
        self.lock_confirm_frames = int(self.get_parameter("lock_confirm_frames").value)
        self.estimate_distance = bool(self.get_parameter("estimate_distance").value)
        self.distance_calibration_k = float(self.get_parameter("distance_calibration_k").value)
        self.ball_diameter_m = float(self.get_parameter("ball_diameter_m").value)
        self.focal_length_px = float(self.get_parameter("focal_length_px").value)
        self.horizontal_fov_deg = float(self.get_parameter("horizontal_fov_deg").value)
        self.vertical_fov_deg = float(self.get_parameter("vertical_fov_deg").value)
        self.image_width_px = int(self.get_parameter("image_width_px").value)
        self.image_height_px = int(self.get_parameter("image_height_px").value)
        self.min_distance_m = float(self.get_parameter("min_distance_m").value)
        self.max_distance_m = float(self.get_parameter("max_distance_m").value)
        self.distance_filter_alpha = float(self.get_parameter("distance_filter_alpha").value)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.target_error_topic = str(self.get_parameter("target_error_topic").value)

        # Keep parameters sane even if launch passes a bad value.
        self.publish_rate = max(self.publish_rate, 1.0)
        self.stale_detection_timeout = max(self.stale_detection_timeout, 0.05)
        self.missed_detection_threshold = max(self.missed_detection_threshold, 1)
        self.coast_timeout = max(self.coast_timeout, 0.05)
        self.lock_confirm_frames = max(self.lock_confirm_frames, 1)
        self.image_width_px = max(self.image_width_px, 1)
        self.image_height_px = max(self.image_height_px, 1)
        self.ball_diameter_m = max(self.ball_diameter_m, 0.001)
        self.min_distance_m = max(self.min_distance_m, 0.01)
        self.max_distance_m = max(self.max_distance_m, self.min_distance_m)
        self.distance_filter_alpha = min(1.0, max(0.01, self.distance_filter_alpha))

        self.state = TrackingState.SEARCHING

        # Latest /drone/vision/detections message truth.
        self.last_detection_message_time = 0.0
        self.latest_detection_count = 0
        self.latest_usable_target_count = 0
        self.detection_visible_now = False

        # Downstream-safe visibility. This may remain true for a short bounded
        # grace period after a missed frame, but it expires quickly.
        self.target_visible_for_control = False
        self.missed_detection_count = 0
        self.consecutive_valid_detection_count = 0

        # Current publishable target values. These come from the most recent
        # valid detection and are only published while target_visible_for_control
        # is true. Once the grace period expires, outputs are zeroed.
        self.current_target_center_x = 0.5
        self.current_target_center_y = 0.5
        self.current_target_confidence = 0.0
        self.current_target_area = 0.0
        self.current_target_diameter_px = 0.0
        self.current_distance_valid = False
        self.current_distance_m = 0.0
        self.current_raw_distance_m = 0.0
        self.current_bearing_x_rad = 0.0
        self.current_bearing_y_rad = 0.0

        # Memory for reacquisition/scoring only.
        self.last_valid_target_time = 0.0
        self.last_known_target_center_x = 0.5
        self.last_known_target_center_y = 0.5
        self.last_known_target_confidence = 0.0
        self.last_known_target_area = 0.0
        self.last_known_distance_valid = False
        self.last_known_distance_m = 0.0

        self.detection_message_count = 0
        self.target_error_publish_count = 0
        self.last_published_tracking_state = self.state.value
        self.last_published_target_visible = False
        self.last_logged_state = self.state
        self.last_logged_target_visible = False
        self.last_logged_detection_visible_now = False

        self.error_x_filter = ExponentialMovingAverage(self.smoothing_alpha)
        self.error_y_filter = ExponentialMovingAverage(self.smoothing_alpha)
        self.distance_filter = ExponentialMovingAverage(self.distance_filter_alpha)

        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        error_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.detection_sub = self.create_subscription(
            DetectionArray,
            self.detections_topic,
            self.detection_callback,
            detection_qos,
        )

        self.error_pub = self.create_publisher(
            TargetError,
            self.target_error_topic,
            error_qos,
        )

        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_error,
        )

        self.status_timer = self.create_timer(
            5.0,
            self.report_status,
        )

        self.diagnostics = NodeDiagnostics(self, heartbeat_period=5.0, stale_seconds=2.0)
        self.diagnostics.add_input(self.detections_topic, "detections")
        self.diagnostics.add_output(self.target_error_topic, "target_error")

        self.get_logger().info(
            f"Tracker node started with bounded-loss debounce | "
            f"detections_topic={self.detections_topic}, "
            f"target_error_topic={self.target_error_topic}, "
            f"target_class={self.target_class}, "
            f"min_confidence={self.min_confidence}, "
            f"target_area_min={self.target_area_min}, "
            f"target_area_max={self.target_area_max}, "
            f"smoothing_alpha={self.smoothing_alpha}, "
            f"missed_detection_threshold={self.missed_detection_threshold}, "
            f"coast_timeout={self.coast_timeout}s, "
            f"stale_detection_timeout={self.stale_detection_timeout}s, "
            f"lock_confirm_frames={self.lock_confirm_frames}, "
            f"estimate_distance={self.estimate_distance}, "
            f"distance_k={self.distance_calibration_k:.2f}, "
            f"ball_diameter_m={self.ball_diameter_m:.3f}"
        )

    def detection_callback(self, msg: DetectionArray) -> None:
        current_time = time.monotonic()
        self.detection_message_count += 1
        self.last_detection_message_time = current_time

        actual_detection_count = len(msg.detections)
        self.latest_detection_count = actual_detection_count

        if msg.count != actual_detection_count:
            self.get_logger().warning(
                f"DetectionArray count mismatch: msg.count={msg.count}, "
                f"len(detections)={actual_detection_count}"
            )

        previous_state = self.state
        previous_target_visible = self.target_visible_for_control
        previous_detection_visible_now = self.detection_visible_now

        best_detection, usable_target_count = self.select_best_detection(msg)
        self.latest_usable_target_count = usable_target_count

        if best_detection is not None:
            self.handle_valid_detection(best_detection, current_time)
        else:
            self.handle_missed_detection(current_time)

        if (
            previous_state != self.state
            or previous_target_visible != self.target_visible_for_control
            or previous_detection_visible_now != self.detection_visible_now
        ):
            self.log_tracker_change(current_time, previous_state, previous_target_visible)

        self.diagnostics.mark_received(
            self.detections_topic,
            summary=(
                f"messages={self.detection_message_count}, "
                f"detections={self.latest_detection_count}, "
                f"usable_targets={self.latest_usable_target_count}, "
                f"detection_visible_now={self.detection_visible_now}, "
                f"target_visible={self.target_visible_for_control}, "
                f"missed={self.missed_detection_count}, "
                f"distance_valid={self.current_distance_valid}, "
                f"distance_m={self.current_distance_m:.2f}"
            ),
        )

    def select_best_detection(self, msg: DetectionArray) -> Tuple[Optional[Detection], int]:
        best_detection: Optional[Detection] = None
        best_score = -999.0
        usable_target_count = 0
        current_time = time.monotonic()
        target_memory_is_recent = (
            self.last_valid_target_time > 0.0
            and current_time - self.last_valid_target_time <= self.reacquisition_timeout
        )

        for detection in msg.detections:
            class_name = detection.class_name.strip().lower()

            if class_name != self.target_class:
                continue

            if detection.confidence < self.min_confidence:
                continue

            area = float(detection.width * detection.height)

            if area < self.target_area_min or area > self.target_area_max:
                continue

            usable_target_count += 1
            score = float(detection.confidence)

            # Memory is used only to rank detections that exist in the latest frame.
            # It must never create a target when YOLO did not publish one.
            if target_memory_is_recent:
                dx = float(detection.center_x) - self.last_known_target_center_x
                dy = float(detection.center_y) - self.last_known_target_center_y
                distance = (dx * dx + dy * dy) ** 0.5

                if distance < self.proximity_threshold:
                    score += 0.3
                else:
                    score -= distance * 0.5

            if score > best_score:
                best_score = score
                best_detection = detection

        return best_detection, usable_target_count

    def handle_valid_detection(self, detection: Detection, current_time: float) -> None:
        was_visible = self.target_visible_for_control
        previous_missed_count = self.missed_detection_count

        self.detection_visible_now = True
        self.missed_detection_count = 0
        self.consecutive_valid_detection_count += 1

        self.current_target_center_x = float(detection.center_x)
        self.current_target_center_y = float(detection.center_y)
        self.current_target_confidence = float(detection.confidence)
        self.current_target_area = float(detection.width * detection.height)
        self.update_target_geometry(detection)

        self.last_valid_target_time = current_time
        self.last_known_target_center_x = self.current_target_center_x
        self.last_known_target_center_y = self.current_target_center_y
        self.last_known_target_confidence = self.current_target_confidence
        self.last_known_target_area = self.current_target_area
        self.last_known_distance_valid = self.current_distance_valid
        self.last_known_distance_m = self.current_distance_m

        if self.consecutive_valid_detection_count >= self.lock_confirm_frames:
            self.target_visible_for_control = True
            self.state = TrackingState.LOCKED
        else:
            self.target_visible_for_control = False
            self.state = TrackingState.SEARCHING

        if not was_visible and self.target_visible_for_control:
            self.get_logger().info(
                f"Target locked | class={self.target_class}, "
                f"confidence={self.current_target_confidence:.2f}, "
                f"area={self.current_target_area:.4f}, "
                f"valid_frames={self.consecutive_valid_detection_count}, "
                f"previous_missed={previous_missed_count}"
            )

    def handle_missed_detection(self, current_time: float) -> None:
        self.detection_visible_now = False
        self.consecutive_valid_detection_count = 0
        self.missed_detection_count += 1

        if self.last_valid_target_time <= 0.0:
            self.target_visible_for_control = False
            self.state = TrackingState.SEARCHING
            self.clear_publishable_target(reset_memory=False)
            return

        target_age = current_time - self.last_valid_target_time
        still_inside_time_grace = target_age <= self.coast_timeout
        still_inside_frame_grace = self.missed_detection_count < self.missed_detection_threshold

        if still_inside_time_grace and still_inside_frame_grace:
            # Hold the last target briefly to avoid one-frame YOLO dropout flicker.
            # This is bounded by both missed_detection_threshold and coast_timeout.
            self.target_visible_for_control = True
            self.state = TrackingState.LOCKED

            if self.missed_detection_count == 1:
                self.get_logger().info(
                    f"Missed target frame; holding LOCKED during grace | "
                    f"missed={self.missed_detection_count}/{self.missed_detection_threshold}, "
                    f"target_age={target_age:.2f}/{self.coast_timeout:.2f}s"
                )
            return

        self.mark_target_lost(
            current_time,
            reason=(
                f"Target lost after grace | "
                f"missed={self.missed_detection_count}/{self.missed_detection_threshold}, "
                f"target_age={target_age:.2f}/{self.coast_timeout:.2f}s"
            ),
        )

    def mark_target_lost(self, current_time: float, reason: str) -> None:
        was_visible = self.target_visible_for_control

        self.target_visible_for_control = False
        self.detection_visible_now = False
        self.clear_publishable_target(reset_memory=False)

        if self.last_valid_target_time <= 0.0:
            self.state = TrackingState.SEARCHING
        else:
            target_age = current_time - self.last_valid_target_time
            if target_age > self.reacquisition_timeout:
                self.state = TrackingState.SEARCHING
            else:
                self.state = TrackingState.LOST

        if was_visible or self.state == TrackingState.LOST:
            self.get_logger().warning(reason)

    def clear_publishable_target(self, reset_memory: bool = False) -> None:
        self.current_target_center_x = 0.5
        self.current_target_center_y = 0.5
        self.current_target_confidence = 0.0
        self.current_target_area = 0.0
        self.error_x_filter.reset()
        self.error_y_filter.reset()
        self.current_target_diameter_px = 0.0
        self.current_distance_valid = False
        self.current_distance_m = 0.0
        self.current_raw_distance_m = 0.0
        self.current_bearing_x_rad = 0.0
        self.current_bearing_y_rad = 0.0
        self.distance_filter.reset()

        if reset_memory:
            self.last_valid_target_time = 0.0
            self.last_known_target_center_x = 0.5
            self.last_known_target_center_y = 0.5
            self.last_known_target_confidence = 0.0
            self.last_known_target_area = 0.0

    def get_focal_lengths_px(self) -> tuple[float, float]:
        if self.focal_length_px > 0.0:
            return self.focal_length_px, self.focal_length_px

        hfov = math.radians(max(1.0, min(179.0, self.horizontal_fov_deg)))
        vfov = math.radians(max(1.0, min(179.0, self.vertical_fov_deg)))
        fx = self.image_width_px / (2.0 * math.tan(hfov / 2.0))
        fy = self.image_height_px / (2.0 * math.tan(vfov / 2.0))
        return fx, fy

    def update_target_geometry(self, detection: Detection) -> None:
        pixel_width = float(detection.pixel_width)
        pixel_height = float(detection.pixel_height)

        if pixel_width <= 0.0:
            pixel_width = float(detection.width) * float(self.image_width_px)
        if pixel_height <= 0.0:
            pixel_height = float(detection.height) * float(self.image_height_px)

        diameter_px = 0.5 * (pixel_width + pixel_height)
        self.current_target_diameter_px = max(0.0, diameter_px)

        center_x_px = float(detection.pixel_center_x)
        center_y_px = float(detection.pixel_center_y)
        if center_x_px <= 0.0:
            center_x_px = float(detection.center_x) * float(self.image_width_px)
        if center_y_px <= 0.0:
            center_y_px = float(detection.center_y) * float(self.image_height_px)

        fx, fy = self.get_focal_lengths_px()
        self.current_bearing_x_rad = math.atan((center_x_px - self.image_width_px / 2.0) / max(fx, 1e-6))
        self.current_bearing_y_rad = math.atan((center_y_px - self.image_height_px / 2.0) / max(fy, 1e-6))

        self.current_distance_valid = False
        self.current_raw_distance_m = 0.0
        self.current_distance_m = 0.0

        if not self.estimate_distance or diameter_px <= 1.0:
            self.distance_filter.reset()
            return

        if self.distance_calibration_k > 0.0:
            raw_distance_m = self.distance_calibration_k / diameter_px
        else:
            raw_distance_m = self.ball_diameter_m * fx / diameter_px

        raw_distance_m = self.clamp(raw_distance_m, self.min_distance_m, self.max_distance_m)
        filtered_distance_m = self.distance_filter.update(raw_distance_m)

        self.current_raw_distance_m = float(raw_distance_m)
        self.current_distance_m = float(filtered_distance_m)
        self.current_distance_valid = True

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    def detection_stream_is_stale(self, current_time: float) -> bool:
        if self.last_detection_message_time <= 0.0:
            return False
        return current_time - self.last_detection_message_time > self.stale_detection_timeout

    def expire_target_if_needed(self, current_time: float) -> None:
        if not self.target_visible_for_control:
            # After a longer loss, move from LOST back to SEARCHING.
            if (
                self.state == TrackingState.LOST
                and self.last_valid_target_time > 0.0
                and current_time - self.last_valid_target_time > self.reacquisition_timeout
            ):
                self.state = TrackingState.SEARCHING
                self.missed_detection_count = 0
                self.get_logger().info("Reacquisition timeout expired. Switching LOST -> SEARCHING.")
            return

        if self.detection_stream_is_stale(current_time):
            self.mark_target_lost(
                current_time,
                reason=(
                    f"Target lost: /drone/vision/detections stream stale | "
                    f"stream_age={current_time - self.last_detection_message_time:.2f}s"
                ),
            )
            return

        if self.last_valid_target_time > 0.0:
            target_age = current_time - self.last_valid_target_time
            if target_age > self.coast_timeout:
                self.mark_target_lost(
                    current_time,
                    reason=(
                        f"Target lost: no valid target within grace | "
                        f"target_age={target_age:.2f}/{self.coast_timeout:.2f}s, "
                        f"missed={self.missed_detection_count}/{self.missed_detection_threshold}"
                    ),
                )

    def get_target_age(self, current_time: Optional[float] = None) -> float:
        if self.last_valid_target_time <= 0.0:
            return -1.0
        if current_time is None:
            current_time = time.monotonic()
        return float(current_time - self.last_valid_target_time)

    def publish_error(self) -> None:
        current_time = time.monotonic()
        self.expire_target_if_needed(current_time)

        msg = TargetError()
        msg.stamp = self.get_clock().now().to_msg()
        msg.target_class = self.target_class
        msg.tracking_state = self.state.value

        target_visible = (
            self.target_visible_for_control
            and self.state == TrackingState.LOCKED
            and not self.detection_stream_is_stale(current_time)
            and self.last_valid_target_time > 0.0
            and current_time - self.last_valid_target_time <= self.coast_timeout
        )

        if target_visible:
            raw_error_x = self.current_target_center_x - 0.5
            raw_error_y = self.current_target_center_y - 0.5

            normalized_error_x = raw_error_x * 2.0
            normalized_error_y = raw_error_y * 2.0

            msg.error_x = float(self.error_x_filter.update(normalized_error_x))
            msg.error_y = float(self.error_y_filter.update(normalized_error_y))
            msg.target_visible = True
            msg.target_confidence = float(self.current_target_confidence)
            msg.target_area = float(self.current_target_area)
            msg.distance_valid = bool(self.current_distance_valid)
            msg.distance_m = float(self.current_distance_m)
            msg.raw_distance_m = float(self.current_raw_distance_m)
            msg.bearing_x_rad = float(self.current_bearing_x_rad)
            msg.bearing_y_rad = float(self.current_bearing_y_rad)
            msg.target_diameter_px = float(self.current_target_diameter_px)
            msg.time_since_last_seen = self.get_target_age(current_time)
        else:
            msg.error_x = 0.0
            msg.error_y = 0.0
            msg.target_visible = False
            msg.target_confidence = 0.0
            msg.target_area = 0.0
            msg.distance_valid = False
            msg.distance_m = 0.0
            msg.raw_distance_m = 0.0
            msg.bearing_x_rad = 0.0
            msg.bearing_y_rad = 0.0
            msg.target_diameter_px = 0.0
            msg.time_since_last_seen = self.get_target_age(current_time)

        self.error_pub.publish(msg)
        self.target_error_publish_count += 1
        self.last_published_tracking_state = msg.tracking_state
        self.last_published_target_visible = bool(msg.target_visible)

        self.diagnostics.mark_published(
            self.target_error_topic,
            summary=(
                f"messages={self.target_error_publish_count}, "
                f"published_tracking_state={msg.tracking_state}, "
                f"published_target_visible={msg.target_visible}, "
                f"detection_visible_now={self.detection_visible_now}, "
                f"target_age={msg.time_since_last_seen:.2f}s, "
                f"missed={self.missed_detection_count}, "
                f"distance_valid={self.current_distance_valid}, "
                f"distance_m={self.current_distance_m:.2f}"
            ),
        )

    def log_tracker_change(
        self,
        current_time: float,
        previous_state: TrackingState,
        previous_target_visible: bool,
    ) -> None:
        target_age = self.get_target_age(current_time)
        self.get_logger().info(
            f"Tracker state update | "
            f"state={previous_state.value}->{self.state.value}, "
            f"target_visible={previous_target_visible}->{self.target_visible_for_control}, "
            f"detection_visible_now={self.detection_visible_now}, "
            f"latest_detection_count={self.latest_detection_count}, "
            f"usable_target_count={self.latest_usable_target_count}, "
            f"missed={self.missed_detection_count}/{self.missed_detection_threshold}, "
            f"target_age={target_age:.2f}s, "
            f"error_x={(self.current_target_center_x - 0.5) * 2.0:.3f}, "
            f"error_y={(self.current_target_center_y - 0.5) * 2.0:.3f}, "
            f"distance_valid={self.current_distance_valid}, "
            f"distance_m={self.current_distance_m:.2f}"
        )

    def report_status(self) -> None:
        current_time = time.monotonic()
        target_age = self.get_target_age(current_time)

        if self.last_detection_message_time > 0.0:
            detection_stream_age = current_time - self.last_detection_message_time
        else:
            detection_stream_age = -1.0

        self.get_logger().info(
            f"Tracker status | "
            f"detection_messages={self.detection_message_count}, "
            f"latest_detection_count={self.latest_detection_count}, "
            f"usable_target_count={self.latest_usable_target_count}, "
            f"detection_visible_now={self.detection_visible_now}, "
            f"target_visible={self.target_visible_for_control}, "
            f"tracking_state={self.state.value}, "
            f"missed={self.missed_detection_count}/{self.missed_detection_threshold}, "
            f"target_age={target_age:.2f}s, "
            f"detection_stream_age={detection_stream_age:.2f}s, "
            f"published_tracking_state={self.last_published_tracking_state}, "
            f"published_target_visible={self.last_published_target_visible}, "
            f"target_error_messages={self.target_error_publish_count}, "
            f"last_detection_age={self.diagnostics.format_age(self.detections_topic)}, "
            f"confidence={self.current_target_confidence:.2f}, "
            f"area={self.current_target_area:.4f}, "
            f"error_x={(self.current_target_center_x - 0.5) * 2.0:.3f}, "
            f"error_y={(self.current_target_center_y - 0.5) * 2.0:.3f}, "
            f"distance_valid={self.current_distance_valid}, "
            f"distance_m={self.current_distance_m:.2f}"
        )

    def destroy_node(self) -> None:
        self.get_logger().info("Tracker node shut down.")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
