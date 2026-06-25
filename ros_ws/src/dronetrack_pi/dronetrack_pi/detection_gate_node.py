"""Detection gate / command validator (runs ON THE PI).

This is the trust boundary for perception coming from the laptop ground station.
The laptop's YOLO node publishes detections on an *untrusted* inbound topic. This
node validates every message and only then republishes sanitized detections onto
the *internal* topic that the existing Pi-side tracker_node already consumes
(`/drone/vision/detections`). Because it republishes onto the original topic name,
no existing Pi node has to change.

Validation rules (all configurable):
  - heartbeat must be fresh           -> otherwise the link is considered down and
                                         ALL detections are dropped
  - message stamp must be fresh        -> drop stale/buffered frames
  - stamp must be monotonic            -> drop out-of-order / replayed frames
  - per-message rate limited           -> flood / DoS protection
  - count clamped to max               -> reject absurd payloads
  - confidence >= min                  -> drop low-confidence noise
  - normalized coords in [0, 1]        -> reject malformed geometry
  - pixel box within image bounds      -> reject malformed geometry

If a message fails a hard check it is dropped whole. If individual detections in
an otherwise-valid array are malformed, only those detections are dropped.

Key safety property: this node can only ever *reduce or pass through* perception.
It never synthesizes a target and it never touches flight control, arming, or
MAVSDK. If the laptop disconnects, fresh detections simply stop arriving and the
existing Pi control stack holds position on its own.
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_interfaces.msg import Detection, DetectionArray
from dronetrack_msgs.msg import GroundStationHeartbeat


class DetectionGateNode(Node):
    def __init__(self) -> None:
        super().__init__("detection_gate_node")

        # Topics
        self.declare_parameter("inbound_detections_topic", "/groundstation/vision/detections")
        self.declare_parameter("outbound_detections_topic", "/drone/vision/detections")
        self.declare_parameter("heartbeat_topic", "/groundstation/heartbeat")

        # Validation knobs
        self.declare_parameter("max_detection_age_s", 1.5)      # drop frames older than this (Pi->laptop->Pi round-trip)
        self.declare_parameter("max_heartbeat_age_s", 1.0)      # link considered down past this
        self.declare_parameter("require_heartbeat", True)       # drop detections if no heartbeat yet
        self.declare_parameter("max_message_rate_hz", 60.0)     # inbound flood cap
        self.declare_parameter("max_detections", 20)            # absurd-payload cap
        self.declare_parameter("min_confidence", 0.10)          # noise floor
        self.declare_parameter("clock_skew_tolerance_s", 0.25)  # allow small future stamps (clock jitter)
        self.declare_parameter("report_period_s", 5.0)

        self.inbound_topic = str(self.get_parameter("inbound_detections_topic").value)
        self.outbound_topic = str(self.get_parameter("outbound_detections_topic").value)
        self.heartbeat_topic = str(self.get_parameter("heartbeat_topic").value)

        self.max_detection_age_s = float(self.get_parameter("max_detection_age_s").value)
        self.max_heartbeat_age_s = float(self.get_parameter("max_heartbeat_age_s").value)
        self.require_heartbeat = bool(self.get_parameter("require_heartbeat").value)
        self.max_message_rate_hz = float(self.get_parameter("max_message_rate_hz").value)
        self.max_detections = int(self.get_parameter("max_detections").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.clock_skew_tolerance_s = float(self.get_parameter("clock_skew_tolerance_s").value)
        self.report_period_s = float(self.get_parameter("report_period_s").value)

        # State
        self._last_heartbeat_time = None        # monotonic receipt time (sec) when last heartbeat accepted
        self._last_heartbeat_seq = None
        self._last_passed_stamp = None          # last accepted detection stamp (sec, from header)
        self._last_inbound_mono = 0.0           # for rate limiting (time.monotonic)

        # Counters for the heartbeat report
        self._received = 0
        self._passed = 0
        self._dropped_rate = 0
        self._dropped_stale = 0
        self._dropped_order = 0
        self._dropped_no_link = 0
        self._dropped_detections = 0

        # The laptop link is lossy Wi-Fi: BEST_EFFORT inbound so we never block on
        # a missing subscriber and always act on the freshest frame.
        inbound_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Republish RELIABLE to the local tracker, matching the original pipeline.
        outbound_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(
            DetectionArray, self.inbound_topic, self._on_detections, inbound_qos
        )
        self.hb_sub = self.create_subscription(
            GroundStationHeartbeat, self.heartbeat_topic, self._on_heartbeat, inbound_qos
        )
        self.pub = self.create_publisher(DetectionArray, self.outbound_topic, outbound_qos)

        self.create_timer(self.report_period_s, self._report)

        self.get_logger().info(
            f"Detection gate up | in={self.inbound_topic} -> out={self.outbound_topic} | "
            f"max_age={self.max_detection_age_s}s, max_hb_age={self.max_heartbeat_age_s}s, "
            f"rate_cap={self.max_message_rate_hz}Hz, min_conf={self.min_confidence}"
        )

    # ---- helpers ---------------------------------------------------------
    def _ros_now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _monotonic_s() -> float:
        return time.monotonic()

    @staticmethod
    def _stamp_to_s(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _heartbeat_fresh(self) -> bool:
        if self._last_heartbeat_time is None:
            return not self.require_heartbeat
        return (self._monotonic_s() - self._last_heartbeat_time) <= self.max_heartbeat_age_s

    # ---- callbacks -------------------------------------------------------
    def _on_heartbeat(self, msg: GroundStationHeartbeat) -> None:
        # Freshness is governed by receipt time (below). The sequence is only used
        # to ignore an exact redelivered duplicate of the last beat. A sequence
        # that goes BACKWARDS means the ground station restarted (its counter
        # resets to 1) -- we must accept that and rebaseline, not reject it
        # forever, otherwise a single laptop restart wedges the link DOWN.
        seq = msg.sequence
        last = self._last_heartbeat_seq
        if last is not None:
            if seq == last:
                return  # exact duplicate of the last accepted beat; ignore
            if seq < last:
                self.get_logger().warning(
                    f"Ground-station heartbeat sequence reset ({last} -> {seq}); "
                    "treating as ground-station restart."
                )
        self._last_heartbeat_seq = seq
        self._last_heartbeat_time = self._monotonic_s()

    def _on_detections(self, msg: DetectionArray) -> None:
        self._received += 1
        ros_now = self._ros_now_s()

        # 1) Rate limit (flood protection) using a TRUE monotonic clock, so an NTP
        #    step on the Pi can't wedge the limiter or let a burst through.
        mono = time.monotonic()
        if self.max_message_rate_hz > 0.0:
            min_dt = 1.0 / self.max_message_rate_hz
            if (mono - self._last_inbound_mono) < min_dt:
                self._dropped_rate += 1
                return
        self._last_inbound_mono = mono

        # 2) Link must be alive.
        if not self._heartbeat_fresh():
            self._dropped_no_link += 1
            self.get_logger().warning(
                "Dropping detections: ground-station heartbeat stale/absent.",
                throttle_duration_sec=2.0,
            )
            return

        # 3) Stamp freshness + monotonicity.
        stamp_s = self._stamp_to_s(msg.stamp)
        age = ros_now - stamp_s
        if age > self.max_detection_age_s:
            self._dropped_stale += 1
            self.get_logger().warning(
                f"Dropping stale detections: age={age:.3f}s > {self.max_detection_age_s}s",
                throttle_duration_sec=2.0,
            )
            return
        if age < -self.clock_skew_tolerance_s:
            # Stamp is too far in the future -> bad clock or spoofed frame.
            self._dropped_stale += 1
            self.get_logger().warning(
                f"Dropping future-stamped detections: age={age:.3f}s (check clock sync)",
                throttle_duration_sec=2.0,
            )
            return
        if self._last_passed_stamp is not None and stamp_s <= self._last_passed_stamp:
            self._dropped_order += 1
            return

        # 4) Sanitize individual detections.
        clean = DetectionArray()
        clean.stamp = msg.stamp
        clean.image_width = msg.image_width
        clean.image_height = msg.image_height

        kept = []
        for det in msg.detections:
            if not self._detection_valid(det, msg.image_width, msg.image_height):
                self._dropped_detections += 1
                continue
            kept.append(det)
            if len(kept) >= self.max_detections:
                break

        clean.detections = kept
        clean.count = len(kept)

        self._last_passed_stamp = stamp_s
        self.pub.publish(clean)
        self._passed += 1

    def _detection_valid(self, det: Detection, img_w: int, img_h: int) -> bool:
        if not math.isfinite(det.confidence) or det.confidence < self.min_confidence:
            return False
        # Normalized geometry must be finite and in [0, 1].
        for v in (det.center_x, det.center_y, det.width, det.height):
            if not math.isfinite(v) or v < 0.0 or v > 1.0:
                return False
        # Pixel box must be non-negative and fit inside the image. The tracker
        # uses pixel_width/height for its distance estimate, so reject malformed
        # boxes here at the trust boundary rather than letting them skew range.
        if det.pixel_width < 0 or det.pixel_height < 0:
            return False
        if img_w > 0:
            if not (0 <= det.pixel_center_x <= img_w) or det.pixel_width > img_w:
                return False
            if det.pixel_center_x - det.pixel_width / 2 < -1 or \
               det.pixel_center_x + det.pixel_width / 2 > img_w + 1:
                return False
        if img_h > 0:
            if not (0 <= det.pixel_center_y <= img_h) or det.pixel_height > img_h:
                return False
            if det.pixel_center_y - det.pixel_height / 2 < -1 or \
               det.pixel_center_y + det.pixel_height / 2 > img_h + 1:
                return False
        return True

    def _report(self) -> None:
        link = "UP" if self._heartbeat_fresh() else "DOWN"
        self.get_logger().info(
            f"Gate | link={link} recv={self._received} pass={self._passed} "
            f"drop[rate={self._dropped_rate} stale={self._dropped_stale} "
            f"order={self._dropped_order} nolink={self._dropped_no_link} "
            f"bad_det={self._dropped_detections}]"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
