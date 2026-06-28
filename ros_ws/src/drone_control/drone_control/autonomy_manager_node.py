"""
Mission/autonomy state machine for the drone vision system.

This node is the central decision-maker for when tracking autonomy is allowed.
It does not talk directly to PX4 and it does not generate movement commands.
It only publishes the two safety gates already used by the system:

Subscribes:
    /drone/autonomy/request          std_msgs/Bool
    /drone/mavsdk/offboard_request   std_msgs/Bool
    /drone/tracking/target_error              drone_interfaces/TargetError
    /drone/telemetry           drone_interfaces/DroneTelemetry

Publishes:
    /drone/autonomy/state            std_msgs/String
    /drone/autonomy/enabled          std_msgs/Bool
    /drone/mavsdk/offboard_enable    std_msgs/Bool

Mission states:
    - STANDBY: operator/RC has not requested autonomy
    - PREFLIGHT_BLOCKED: autonomy is requested, but safety preconditions are not met
    - READY: request is on and vehicle is safe, but target is not locked yet
    - TRACKING: request is on, vehicle is safe, target is locked/fresh
    - SCANNING: request is on, vehicle is safe, no lock yet, but the mission is in
      a SCAN step and allow_scan_without_lock is enabled (pre-lock, yaw-only search)
    - TARGET_LOST: request is on, vehicle is safe, but target was lost/stale
    - FAILSAFE: autonomy was already ready/active, then a safety check failed

TRACKING and (opt-in) SCANNING publish /drone/autonomy/enabled true. SCANNING is
yaw-only in practice: while the mission mode is SCAN, control_node holds position
and only rotates, so this never authorizes translation.
This node intentionally does not publish /drone/mission/state; mission_executor_node owns that topic.
READY/TRACKING/TARGET_LOST can publish /drone/mavsdk/offboard_enable true, but only
after the operator explicitly requests MAVSDK Offboard on /drone/mavsdk/offboard_request.
control_node still decides whether a real movement command is valid.
"""

from __future__ import annotations

import time
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from drone_diagnostics.node_diagnostics import NodeDiagnostics
from drone_interfaces.msg import DroneTelemetry, MissionCommand, TargetError
from drone_control.autonomy_logic import scan_yaw_authorized


class MissionState(str, Enum):
    STANDBY = "STANDBY"
    PREFLIGHT_BLOCKED = "PREFLIGHT_BLOCKED"
    READY = "READY"
    TRACKING = "TRACKING"
    SCANNING = "SCANNING"
    TARGET_LOST = "TARGET_LOST"
    FAILSAFE = "FAILSAFE"


# Backward-compatible alias for older code/comments that still say autonomy state.
AutonomyState = MissionState


class AutonomyManagerNode(Node):
    """Conservative state machine that owns autonomy/offboard gate decisions."""

    def __init__(self) -> None:
        super().__init__("autonomy_manager_node")

        # Topics
        self.declare_parameter("autonomy_request_topic", "/drone/autonomy/request")
        self.declare_parameter("offboard_request_topic", "/drone/mavsdk/offboard_request")
        self.declare_parameter("target_error_topic", "/drone/tracking/target_error")
        self.declare_parameter("telemetry_topic", "/drone/telemetry")
        self.declare_parameter("mission_command_topic", "/drone/mission/command")
        self.declare_parameter("autonomy_enable_topic", "/drone/autonomy/enabled")
        self.declare_parameter("offboard_enable_topic", "/drone/mavsdk/offboard_enable")
        self.declare_parameter("autonomy_state_topic", "/drone/autonomy/state")

        # Behavior
        self.declare_parameter("manager_enabled", True)
        self.declare_parameter("initial_autonomy_request", False)
        self.declare_parameter("initial_offboard_request", False)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("target_timeout", 1.0)
        self.declare_parameter("telemetry_timeout", 2.0)
        self.declare_parameter("request_timeout", 0.0)  # 0 disables request staleness checking
        self.declare_parameter("enable_offboard_when_ready", True)

        # Opt-in (default OFF): allow pre-lock, yaw-only autonomy while the mission
        # is in a SCAN step, so the drone can yaw-sweep to search for a target.
        # Yaw-only is enforced by control_node's SCAN behavior (position-hold + yaw).
        self.declare_parameter("allow_scan_without_lock", False)
        self.declare_parameter("mission_command_timeout", 1.0)  # SCAN command freshness

        # Safety requirements
        self.declare_parameter("require_connected", True)
        self.declare_parameter("require_armed", True)
        self.declare_parameter("require_health_all_ok", False)
        self.declare_parameter("require_gps", False)
        self.declare_parameter("required_flight_mode", "")  # leave blank to avoid Offboard chicken/egg loop
        self.declare_parameter("min_battery_percent", 20.0)

        self.autonomy_request_topic = str(self.get_parameter("autonomy_request_topic").value)
        self.offboard_request_topic = str(self.get_parameter("offboard_request_topic").value)
        self.target_error_topic = str(self.get_parameter("target_error_topic").value)
        self.telemetry_topic = str(self.get_parameter("telemetry_topic").value)
        self.mission_command_topic = str(self.get_parameter("mission_command_topic").value)
        self.autonomy_enable_topic = str(self.get_parameter("autonomy_enable_topic").value)
        self.offboard_enable_topic = str(self.get_parameter("offboard_enable_topic").value)
        self.autonomy_state_topic = str(self.get_parameter("autonomy_state_topic").value)

        self.manager_enabled = bool(self.get_parameter("manager_enabled").value)
        self.autonomy_requested = bool(self.get_parameter("initial_autonomy_request").value)
        self.offboard_requested = bool(self.get_parameter("initial_offboard_request").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.target_timeout = float(self.get_parameter("target_timeout").value)
        self.telemetry_timeout = float(self.get_parameter("telemetry_timeout").value)
        self.request_timeout = float(self.get_parameter("request_timeout").value)
        self.enable_offboard_when_ready = bool(self.get_parameter("enable_offboard_when_ready").value)
        self.allow_scan_without_lock = bool(self.get_parameter("allow_scan_without_lock").value)
        self.mission_command_timeout = float(self.get_parameter("mission_command_timeout").value)

        self.require_connected = bool(self.get_parameter("require_connected").value)
        self.require_armed = bool(self.get_parameter("require_armed").value)
        self.require_health_all_ok = bool(self.get_parameter("require_health_all_ok").value)
        self.require_gps = bool(self.get_parameter("require_gps").value)
        self.required_flight_mode = str(self.get_parameter("required_flight_mode").value).strip().upper()
        self.min_battery_percent = float(self.get_parameter("min_battery_percent").value)

        self._validate_parameters()

        self.state = MissionState.STANDBY
        self.last_state = MissionState.STANDBY
        self.state_reason = "startup"
        self.had_target_lock = False
        self.had_safe_autonomy = False

        self.last_target_error: Optional[TargetError] = None
        self.last_target_time = 0.0
        self.last_telemetry: Optional[DroneTelemetry] = None
        self.last_telemetry_time = 0.0
        self.last_request_time = time.monotonic() if self.autonomy_requested else 0.0
        self.last_mission_mode = ""
        self.last_mission_active = False
        self.last_mission_command_time = 0.0

        self.request_count = 0
        self.offboard_request_count = 0
        self.target_count = 0
        self.telemetry_count = 0
        self.state_publish_count = 0
        self.autonomy_gate_publish_count = 0
        self.offboard_gate_publish_count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.request_sub = self.create_subscription(
            Bool,
            self.autonomy_request_topic,
            self.autonomy_request_callback,
            qos,
        )
        self.offboard_request_sub = self.create_subscription(
            Bool,
            self.offboard_request_topic,
            self.offboard_request_callback,
            qos,
        )
        self.target_sub = self.create_subscription(
            TargetError,
            self.target_error_topic,
            self.target_error_callback,
            qos,
        )
        self.telemetry_sub = self.create_subscription(
            DroneTelemetry,
            self.telemetry_topic,
            self.telemetry_callback,
            qos,
        )
        # Read the mission command only to know whether the active step is a SCAN
        # (for the opt-in pre-lock yaw allowance). This node never publishes it.
        self.mission_command_sub = self.create_subscription(
            MissionCommand,
            self.mission_command_topic,
            self.mission_command_callback,
            qos,
        )

        self.autonomy_enable_pub = self.create_publisher(Bool, self.autonomy_enable_topic, qos)
        self.offboard_enable_pub = self.create_publisher(Bool, self.offboard_enable_topic, qos)
        self.state_pub = self.create_publisher(String, self.autonomy_state_topic, qos)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.update_state_machine)
        self.status_timer = self.create_timer(5.0, self.report_status)

        self.diagnostics = NodeDiagnostics(self, heartbeat_period=5.0, stale_seconds=2.0)
        self.diagnostics.add_input(self.autonomy_request_topic, "autonomy_request", stale_seconds=60.0)
        self.diagnostics.add_input(self.offboard_request_topic, "mavsdk_offboard_request", stale_seconds=60.0)
        self.diagnostics.add_input(self.target_error_topic, "target_error", stale_seconds=self.target_timeout)
        self.diagnostics.add_input(self.telemetry_topic, "telemetry", stale_seconds=self.telemetry_timeout)
        self.diagnostics.add_output(self.autonomy_enable_topic, "autonomy_enable")
        self.diagnostics.add_output(self.offboard_enable_topic, "mavsdk_offboard_enable")
        self.diagnostics.add_output(self.autonomy_state_topic, "autonomy_state")

        self.get_logger().warning(
            "Autonomy manager started | "
            f"request_topic={self.autonomy_request_topic}, state_topic={self.autonomy_state_topic}, "
            f"autonomy_enable_topic={self.autonomy_enable_topic}, "
            f"offboard_request_topic={self.offboard_request_topic}, offboard_enable_topic={self.offboard_enable_topic}, "
            f"manager_enabled={self.manager_enabled}, initial_request={self.autonomy_requested}, "
            f"initial_offboard_request={self.offboard_requested}, "
            f"require_armed={self.require_armed}, require_gps={self.require_gps}, "
            f"required_flight_mode={self.required_flight_mode or 'ANY'}"
        )

    def _validate_parameters(self) -> None:
        if self.publish_rate <= 0.0:
            raise ValueError(f"publish_rate must be > 0, got {self.publish_rate}")
        if self.target_timeout <= 0.0:
            raise ValueError(f"target_timeout must be > 0, got {self.target_timeout}")
        if self.telemetry_timeout <= 0.0:
            raise ValueError(f"telemetry_timeout must be > 0, got {self.telemetry_timeout}")
        if self.request_timeout < 0.0:
            raise ValueError(f"request_timeout must be >= 0, got {self.request_timeout}")
        if self.min_battery_percent < 0.0:
            raise ValueError(f"min_battery_percent must be >= 0, got {self.min_battery_percent}")

    def autonomy_request_callback(self, msg: Bool) -> None:
        self.autonomy_requested = bool(msg.data)
        self.last_request_time = time.monotonic()
        self.request_count += 1
        self.diagnostics.mark_received(
            self.autonomy_request_topic,
            summary=f"messages={self.request_count}, requested={self.autonomy_requested}",
        )

        if not self.autonomy_requested:
            self.had_target_lock = False
            self.had_safe_autonomy = False
            self.offboard_requested = False
            self.get_logger().warning("Autonomy request OFF; mission state will force STANDBY, MAVSDK request OFF, and publish zero gates.")
        else:
            self.get_logger().warning("Autonomy request ON; entering PREFLIGHT/READY once telemetry safety passes.")

    def offboard_request_callback(self, msg: Bool) -> None:
        self.offboard_requested = bool(msg.data)
        self.offboard_request_count += 1
        self.diagnostics.mark_received(
            self.offboard_request_topic,
            summary=f"messages={self.offboard_request_count}, requested={self.offboard_requested}",
        )

        if self.offboard_requested:
            self.get_logger().warning("MAVSDK Offboard request ON; safety state still decides /drone/mavsdk/offboard_enable.")
        else:
            self.get_logger().warning("MAVSDK Offboard request OFF; publishing offboard enable false.")

    def target_error_callback(self, msg: TargetError) -> None:
        self.last_target_error = msg
        self.last_target_time = time.monotonic()
        self.target_count += 1

        locked = self._target_locked(now=self.last_target_time)
        if locked:
            self.had_target_lock = True

        self.diagnostics.mark_received(
            self.target_error_topic,
            summary=(
                f"messages={self.target_count}, state={msg.tracking_state}, "
                f"visible={msg.target_visible}, locked={locked}"
            ),
        )

    def telemetry_callback(self, msg: DroneTelemetry) -> None:
        self.last_telemetry = msg
        self.last_telemetry_time = time.monotonic()
        self.telemetry_count += 1
        self.diagnostics.mark_received(
            self.telemetry_topic,
            summary=(
                f"messages={self.telemetry_count}, connected={msg.connected}, "
                f"armed={msg.armed}, mode={msg.flight_mode}, battery={msg.battery_remaining_percent:.1f}%"
            ),
        )

    def mission_command_callback(self, msg: MissionCommand) -> None:
        self.last_mission_mode = str(msg.mode).strip().upper()
        self.last_mission_active = bool(msg.active)
        self.last_mission_command_time = time.monotonic()

    def _mission_is_scan(self, now: float) -> bool:
        if self.last_mission_command_time <= 0.0:
            return False
        if now - self.last_mission_command_time > self.mission_command_timeout:
            return False
        return self.last_mission_active and self.last_mission_mode == "SCAN"

    def _request_is_fresh(self, now: float) -> bool:
        if not self.autonomy_requested:
            return False
        if self.request_timeout <= 0.0:
            return True
        if self.last_request_time <= 0.0:
            return False
        return now - self.last_request_time <= self.request_timeout

    def _target_locked(self, now: float) -> bool:
        if self.last_target_error is None:
            return False
        if now - self.last_target_time > self.target_timeout:
            return False
        return bool(
            self.last_target_error.target_visible
            and self.last_target_error.tracking_state == "LOCKED"
        )

    def _telemetry_safety_ok(self, now: float) -> tuple[bool, str]:
        if self.last_telemetry is None:
            return False, "NO_TELEMETRY"

        telemetry_age = now - self.last_telemetry_time
        if telemetry_age > self.telemetry_timeout:
            return False, f"TELEMETRY_STALE ({telemetry_age:.2f}s)"

        telemetry = self.last_telemetry

        if self.require_connected and not telemetry.connected:
            return False, "PX4_NOT_CONNECTED"

        if self.require_armed and not telemetry.armed:
            return False, "PX4_NOT_ARMED"

        battery = float(telemetry.battery_remaining_percent)
        if battery > 0.0 and battery < self.min_battery_percent:
            return False, f"LOW_BATTERY ({battery:.1f}%)"

        if self.require_health_all_ok and not telemetry.health_all_ok:
            return False, "HEALTH_NOT_OK"

        if self.require_gps and not telemetry.health_gps_ok:
            return False, "GPS_NOT_OK"

        if self.required_flight_mode:
            flight_mode = str(telemetry.flight_mode).upper()
            if self.required_flight_mode not in flight_mode:
                return False, f"WRONG_FLIGHT_MODE ({telemetry.flight_mode})"

        return True, "SAFETY_OK"

    def _decide_state(self, now: float) -> tuple[MissionState, str]:
        if not self.manager_enabled:
            return MissionState.STANDBY, "MANAGER_DISABLED"

        if not self._request_is_fresh(now):
            if self.autonomy_requested and self.request_timeout > 0.0:
                age = now - self.last_request_time
                return MissionState.STANDBY, f"REQUEST_STALE ({age:.2f}s)"
            return MissionState.STANDBY, "REQUEST_OFF"

        safety_ok, safety_reason = self._telemetry_safety_ok(now)
        if not safety_ok:
            if self.had_safe_autonomy or self.state in (
                MissionState.READY,
                MissionState.TRACKING,
                MissionState.SCANNING,
                MissionState.TARGET_LOST,
            ):
                return MissionState.FAILSAFE, safety_reason
            return MissionState.PREFLIGHT_BLOCKED, safety_reason

        if self._target_locked(now):
            return MissionState.TRACKING, "TARGET_LOCKED"

        # Opt-in pre-lock yaw-only autonomy while the mission is in a SCAN step.
        # We are past the request-fresh, safety-ok, and not-locked checks here.
        if scan_yaw_authorized(
            allow_scan_without_lock=self.allow_scan_without_lock,
            mission_is_scan=self._mission_is_scan(now),
            request_fresh=True,
            safety_ok=True,
            target_locked=False,
        ):
            return MissionState.SCANNING, "SCAN_YAW_NO_LOCK"

        if self.had_target_lock:
            return MissionState.TARGET_LOST, "TARGET_LOST_OR_STALE"

        return MissionState.READY, "WAITING_FOR_TARGET_LOCK"

    def _publish_gates(self, state: MissionState, reason: str) -> None:
        # TRACKING allows full control; SCANNING (opt-in) allows yaw-only control
        # because control_node holds position and only rotates in SCAN mode.
        autonomy_enabled = state in (MissionState.TRACKING, MissionState.SCANNING)

        # Let the MAVSDK executor be ready while requested and safe, but keep actual
        # movement blocked by /drone/autonomy/enabled until TRACKING/SCANNING.
        # PREFLIGHT_BLOCKED is not safe yet. SCANNING needs offboard so its yaw
        # setpoints reach PX4.
        if self.enable_offboard_when_ready:
            offboard_ready = state in (
                MissionState.READY,
                MissionState.TRACKING,
                MissionState.SCANNING,
                MissionState.TARGET_LOST,
            )
        else:
            offboard_ready = state in (MissionState.TRACKING, MissionState.SCANNING)
        offboard_enabled = bool(self.offboard_requested and offboard_ready)

        autonomy_msg = Bool()
        autonomy_msg.data = bool(autonomy_enabled)
        self.autonomy_enable_pub.publish(autonomy_msg)
        self.autonomy_gate_publish_count += 1
        self.diagnostics.mark_published(
            self.autonomy_enable_topic,
            summary=f"enabled={autonomy_enabled}, state={state.value}, reason={reason}",
        )

        offboard_msg = Bool()
        offboard_msg.data = bool(offboard_enabled)
        self.offboard_enable_pub.publish(offboard_msg)
        self.offboard_gate_publish_count += 1
        self.diagnostics.mark_published(
            self.offboard_enable_topic,
            summary=f"enabled={offboard_enabled}, requested={self.offboard_requested}, state={state.value}, reason={reason}",
        )

    def _publish_state(self, state: MissionState, reason: str) -> None:
        msg = String()
        msg.data = f"{state.value}: {reason}"
        self.state_pub.publish(msg)
        self.state_publish_count += 1
        summary = f"messages={self.state_publish_count}, state={state.value}, reason={reason}"
        self.diagnostics.mark_published(self.autonomy_state_topic, summary=summary)

    def update_state_machine(self) -> None:
        now = time.monotonic()
        new_state, reason = self._decide_state(now)

        self.state = new_state
        self.state_reason = reason

        if self.state in (
            MissionState.READY,
            MissionState.TRACKING,
            MissionState.SCANNING,
            MissionState.TARGET_LOST,
        ):
            self.had_safe_autonomy = True
        elif self.state == MissionState.STANDBY:
            self.had_safe_autonomy = False

        if self.state != self.last_state:
            self.get_logger().warning(
                f"AUTONOMY STATE: {self.last_state.value} -> {self.state.value} | reason={reason}"
            )
            self.last_state = self.state

        self._publish_gates(self.state, reason)
        self._publish_state(self.state, reason)

    def report_status(self) -> None:
        now = time.monotonic()
        target_age = "never" if self.last_target_error is None else f"{now - self.last_target_time:.2f}s"
        telemetry_age = "never" if self.last_telemetry is None else f"{now - self.last_telemetry_time:.2f}s"
        request_age = "never" if self.last_request_time <= 0.0 else f"{now - self.last_request_time:.2f}s"

        telemetry_text = "none"
        if self.last_telemetry is not None:
            telemetry_text = (
                f"connected={self.last_telemetry.connected}, armed={self.last_telemetry.armed}, "
                f"mode={self.last_telemetry.flight_mode}, battery={self.last_telemetry.battery_remaining_percent:.1f}%"
            )

        target_text = "none"
        if self.last_target_error is not None:
            target_text = (
                f"state={self.last_target_error.tracking_state}, visible={self.last_target_error.target_visible}, "
                f"confidence={self.last_target_error.target_confidence:.2f}, error_x={self.last_target_error.error_x:+.3f}"
            )

        self.get_logger().info(
            "Autonomy manager status | "
            f"state={self.state.value}, reason={self.state_reason}, requested={self.autonomy_requested}, "
            f"offboard_requested={self.offboard_requested}, "
            f"request_age={request_age}, target_age={target_age}, telemetry_age={telemetry_age}, "
            f"target=({target_text}), telemetry=({telemetry_text}), "
            f"state_msgs={self.state_publish_count}, autonomy_gate_msgs={self.autonomy_gate_publish_count}, "
            f"offboard_gate_msgs={self.offboard_gate_publish_count}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AutonomyManagerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger("autonomy_manager_node").fatal(f"Fatal: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
