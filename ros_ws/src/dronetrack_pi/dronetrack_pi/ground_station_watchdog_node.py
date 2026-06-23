"""Ground-station watchdog / heartbeat monitor (runs ON THE PI).

Owns the single authoritative verdict on the laptop link and reacts safely when
it drops. It publishes:

  - /drone/groundstation/link_status  (dronetrack_msgs/LinkStatus) for observability
  - /drone/groundstation/ok           (std_msgs/Bool) convenience flag

SAFETY MODEL
------------
This node does NOT fly the drone and does NOT command PX4. The Pi is already safe
on its own: the existing control stack holds local NED position and only *yaws*
toward a fresh target, so if detections stop the drone simply stops yawing and
holds. This watchdog adds an explicit, observable reaction on top of that:

  on link loss -> de-assert operator permissions by publishing:
      /drone/autonomy/request        = false   (autonomy_manager will block control)
      /drone/mavsdk/offboard_request = false   (no new Offboard sending)

  optionally (disabled by default) request a HOLD through the EXISTING
  Pi-owned, safety-gated MAVSDK action path:
      /drone/mavsdk/action_command   = HOLD

It never publishes /drone/autonomy/enabled or /drone/mavsdk/offboard_enable
(those are owned by autonomy_manager_node) and never publishes ControlCommand.
De-asserting a *request* can only ever make the system more conservative.

The reaction fires once on the UP->DOWN edge, not continuously, so an operator
can still re-arm/recover deliberately once the link returns.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from drone_interfaces.msg import DetectionArray, MavsdkActionCommand
from dronetrack_msgs.msg import GroundStationHeartbeat, LinkStatus


class GroundStationWatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__("ground_station_watchdog_node")

        # Inputs
        self.declare_parameter("heartbeat_topic", "/groundstation/heartbeat")
        self.declare_parameter("inbound_detections_topic", "/groundstation/vision/detections")

        # Outputs (observability)
        self.declare_parameter("link_status_topic", "/drone/groundstation/link_status")
        self.declare_parameter("link_ok_topic", "/drone/groundstation/ok")

        # Safe-reaction request topics (reuse existing Pi request topics)
        self.declare_parameter("autonomy_request_topic", "/drone/autonomy/request")
        self.declare_parameter("offboard_request_topic", "/drone/mavsdk/offboard_request")
        self.declare_parameter("mavsdk_action_topic", "/drone/mavsdk/action_command")

        # Timing / behavior
        self.declare_parameter("max_heartbeat_age_s", 1.0)
        self.declare_parameter("max_detection_age_s", 1.0)
        self.declare_parameter("check_rate_hz", 10.0)
        self.declare_parameter("deassert_on_link_loss", True)   # publish requests=false on loss
        self.declare_parameter("request_hold_on_link_loss", False)  # also request MAVSDK HOLD

        self.heartbeat_topic = str(self.get_parameter("heartbeat_topic").value)
        self.inbound_detections_topic = str(self.get_parameter("inbound_detections_topic").value)
        self.link_status_topic = str(self.get_parameter("link_status_topic").value)
        self.link_ok_topic = str(self.get_parameter("link_ok_topic").value)
        self.autonomy_request_topic = str(self.get_parameter("autonomy_request_topic").value)
        self.offboard_request_topic = str(self.get_parameter("offboard_request_topic").value)
        self.mavsdk_action_topic = str(self.get_parameter("mavsdk_action_topic").value)

        self.max_heartbeat_age_s = float(self.get_parameter("max_heartbeat_age_s").value)
        self.max_detection_age_s = float(self.get_parameter("max_detection_age_s").value)
        self.check_rate_hz = float(self.get_parameter("check_rate_hz").value)
        self.deassert_on_link_loss = bool(self.get_parameter("deassert_on_link_loss").value)
        self.request_hold_on_link_loss = bool(self.get_parameter("request_hold_on_link_loss").value)

        # State
        self._last_heartbeat_time = None
        self._last_heartbeat_seq = 0
        self._last_detection_time = None
        self._last_heartbeat_stamp_s = None     # header stamp of last heartbeat (for latency)
        self._link_ok = False
        self._link_initialized = False
        self._action_id = 0

        lossy_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(GroundStationHeartbeat, self.heartbeat_topic, self._on_heartbeat, lossy_qos)
        self.create_subscription(DetectionArray, self.inbound_detections_topic, self._on_detections, lossy_qos)

        self.link_status_pub = self.create_publisher(LinkStatus, self.link_status_topic, reliable_qos)
        self.link_ok_pub = self.create_publisher(Bool, self.link_ok_topic, reliable_qos)
        self.autonomy_request_pub = self.create_publisher(Bool, self.autonomy_request_topic, reliable_qos)
        self.offboard_request_pub = self.create_publisher(Bool, self.offboard_request_topic, reliable_qos)
        self.action_pub = self.create_publisher(MavsdkActionCommand, self.mavsdk_action_topic, reliable_qos)

        self.create_timer(1.0 / max(1.0, self.check_rate_hz), self._tick)

        self.get_logger().info(
            f"Ground-station watchdog up | hb_timeout={self.max_heartbeat_age_s}s, "
            f"deassert_on_loss={self.deassert_on_link_loss}, "
            f"request_hold_on_loss={self.request_hold_on_link_loss}"
        )

    # ---- helpers ---------------------------------------------------------
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_to_s(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    # ---- callbacks -------------------------------------------------------
    def _on_heartbeat(self, msg: GroundStationHeartbeat) -> None:
        # See detection_gate_node for rationale: ignore exact duplicates, but a
        # backwards sequence means the ground station restarted -> accept and
        # rebaseline rather than rejecting it as a replay forever.
        seq = msg.sequence
        last = self._last_heartbeat_seq
        if last:
            if seq == last:
                return  # exact duplicate of the last accepted beat; ignore
            if seq < last:
                self.get_logger().warning(
                    f"Ground-station heartbeat sequence reset ({last} -> {seq}); "
                    "treating as ground-station restart."
                )
        self._last_heartbeat_seq = seq
        self._last_heartbeat_time = self._now_s()
        self._last_heartbeat_stamp_s = self._stamp_to_s(msg.stamp)

    def _on_detections(self, msg: DetectionArray) -> None:
        self._last_detection_time = self._now_s()

    # ---- main loop -------------------------------------------------------
    def _tick(self) -> None:
        now = self._now_s()

        hb_age = -1.0 if self._last_heartbeat_time is None else now - self._last_heartbeat_time
        det_age = -1.0 if self._last_detection_time is None else now - self._last_detection_time

        heartbeat_ok = self._last_heartbeat_time is not None and hb_age <= self.max_heartbeat_age_s

        if not heartbeat_ok:
            link_ok = False
            reason = "NO_HEARTBEAT" if self._last_heartbeat_time is None else "HEARTBEAT_STALE"
        elif det_age >= 0.0 and det_age > self.max_detection_age_s:
            # Heartbeat is alive but perception went quiet. Link is "up" for control
            # purposes (drone holds), but we flag detections as stale for the operator.
            link_ok = True
            reason = "DETECTIONS_STALE"
        else:
            link_ok = True
            reason = "OK"

        # Edge detection: react once on UP -> DOWN.
        if self._link_initialized and self._link_ok and not link_ok:
            self._on_link_lost(reason)
        self._link_ok = link_ok
        self._link_initialized = True

        # Latency estimate (needs clock sync to be meaningful).
        latency = float("nan")
        if self._last_heartbeat_stamp_s is not None and heartbeat_ok:
            latency = max(0.0, now - self._last_heartbeat_stamp_s)

        status = LinkStatus()
        status.stamp = self.get_clock().now().to_msg()
        status.link_ok = link_ok
        status.heartbeat_age_s = float(hb_age)
        status.detection_age_s = float(det_age)
        status.estimated_latency_s = float(latency)
        status.last_heartbeat_sequence = int(self._last_heartbeat_seq)
        status.reason = reason
        self.link_status_pub.publish(status)
        self.link_ok_pub.publish(Bool(data=link_ok))

    def _on_link_lost(self, reason: str) -> None:
        self.get_logger().error(f"GROUND STATION LINK LOST ({reason}). Entering safe reaction.")
        if self.deassert_on_link_loss:
            self.autonomy_request_pub.publish(Bool(data=False))
            self.offboard_request_pub.publish(Bool(data=False))
            self.get_logger().warning("De-asserted /drone/autonomy/request and /drone/mavsdk/offboard_request.")
        if self.request_hold_on_link_loss:
            self._action_id += 1
            cmd = MavsdkActionCommand()
            cmd.stamp = self.get_clock().now().to_msg()
            cmd.command_id = self._action_id
            cmd.action = "HOLD"
            cmd.execute = True
            cmd.note = f"watchdog link-loss HOLD ({reason})"
            self.action_pub.publish(cmd)
            self.get_logger().warning("Requested MAVSDK HOLD via Pi-owned action gate.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundStationWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
