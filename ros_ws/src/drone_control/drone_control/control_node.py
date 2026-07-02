"""
Control command generation node.

Subscribes:
    /drone/tracking/target_error
    /drone/telemetry
    /drone/autonomy/enabled
    /drone/mission/command

Publishes:
    /drone/control/command

This node is intentionally conservative for early flight testing:
- target tracking is gated by /drone/autonomy/enabled
- horizontal image error drives yaw in TRACK_CENTER
- the takeoff/local NED position is captured and held during yaw
- mission commands can request FLY_FORWARD, APPROACH_TARGET, or ORBIT_TARGET later
- current safe stage publishes POSITION setpoints only for hold+yaw
- commands are zeroed unless all safety gates pass
"""

import math
import time
from typing import Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from drone_interfaces.msg import ControlCommand, DroneTelemetry, MissionCommand, TargetError
from drone_diagnostics.node_diagnostics import NodeDiagnostics
from drone_control.control_math import approach_forward_velocity


CMD_IDLE = "IDLE"
CMD_VELOCITY = "VELOCITY"
CMD_POSITION = "POSITION"

STATUS_SENT = "SENT"
STATUS_NO_TARGET = "NO_TARGET_DATA"
STATUS_TARGET_STALE = "TARGET_DATA_STALE"
STATUS_TARGET_NOT_VISIBLE = "TARGET_NOT_VISIBLE"
STATUS_BLOCKED_DISABLED = "BLOCKED_AUTONOMY_DISABLED"
STATUS_BLOCKED_NO_TELEM = "BLOCKED_NO_TELEMETRY"
STATUS_BLOCKED_TELEM_STALE = "BLOCKED_TELEMETRY_STALE"
STATUS_BLOCKED_LOW_BATTERY = "BLOCKED_LOW_BATTERY"
STATUS_BLOCKED_GPS = "BLOCKED_GPS_NOT_HEALTHY"
STATUS_BLOCKED_DISARMED = "BLOCKED_NOT_ARMED"
STATUS_BLOCKED_DISCONNECTED = "BLOCKED_NOT_CONNECTED"
STATUS_ALTITUDE_CLAMPED = "ALTITUDE_FLOOR_CLAMPED"
STATUS_MISSION_STALE = "MISSION_COMMAND_STALE"
STATUS_BLOCKED_NO_LOCAL_POSITION = "BLOCKED_NO_LOCAL_POSITION"

DYNAMIC_PARAMS = {
    "autonomy_enabled", "autonomous_enabled",
    "gain_forward", "gain_right", "gain_down", "gain_yaw",
    "deadband_x", "deadband_y",
    "max_velocity_forward", "max_velocity_right", "max_velocity_down", "max_yaw_rate",
    "max_accel_forward", "max_accel_right", "max_accel_down", "max_yaw_accel",
    "min_battery_percent", "require_gps", "require_armed",
    "min_altitude_m", "target_timeout", "telemetry_timeout",
    "mission_command_timeout", "desired_distance_m", "distance_gain_forward", "target_area_goal",
    "orbit_speed_m_s",
}


class ControlNode(Node):
    def __init__(self) -> None:
        super().__init__("control_node")

        # New preferred parameter name is autonomy_enabled.
        # Keep autonomous_enabled as a backward-compatible alias for older launch/config files.
        self.declare_parameter("autonomy_enabled", False)
        self.declare_parameter("autonomous_enabled", False)

        self.declare_parameter("gain_forward", 1.0)  # reserved for future distance/area control
        self.declare_parameter("gain_right", 1.0)    # reserved; strafe disabled for now
        self.declare_parameter("gain_down", 0.5)
        self.declare_parameter("gain_yaw", 0.8)

        self.declare_parameter("deadband_x", 0.05)
        self.declare_parameter("deadband_y", 0.05)

        self.declare_parameter("max_velocity_forward", 2.0)
        self.declare_parameter("max_velocity_right", 2.0)
        self.declare_parameter("max_velocity_down", 0.5)
        self.declare_parameter("max_yaw_rate", 1.0)

        self.declare_parameter("max_accel_forward", 1.5)
        self.declare_parameter("max_accel_right", 1.5)
        self.declare_parameter("max_accel_down", 0.5)
        self.declare_parameter("max_yaw_accel", 2.0)

        self.declare_parameter("min_battery_percent", 25.0)
        self.declare_parameter("require_gps", False)
        self.declare_parameter("require_armed", True)
        self.declare_parameter("min_altitude_m", 2.0)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("target_timeout", 1.0)
        self.declare_parameter("telemetry_timeout", 2.0)
        self.declare_parameter("target_error_topic", "/drone/tracking/target_error")
        self.declare_parameter("telemetry_topic", "/drone/telemetry")
        self.declare_parameter("control_command_topic", "/drone/control/command")
        self.declare_parameter("autonomy_enable_topic", "/drone/autonomy/enabled")
        self.declare_parameter("mission_command_topic", "/drone/mission/command")
        self.declare_parameter("mission_command_timeout", 1.0)
        self.declare_parameter("desired_distance_m", 2.0)
        self.declare_parameter("distance_gain_forward", 0.6)
        self.declare_parameter("target_area_goal", 0.08)
        self.declare_parameter("orbit_speed_m_s", 0.30)

        # APPROACH translation is opt-in and OFF by default. Even when true, the
        # resulting VELOCITY command is still gated downstream by
        # telemetry_node.allow_translation_commands (also false by default), so
        # enabling this alone does not move the vehicle. When false, APPROACH_TARGET
        # behaves exactly like TRACK_CENTER (position-hold + yaw only).
        self.declare_parameter("enable_approach_translation", False)
        self.declare_parameter("approach_distance_deadband_m", 0.15)

        autonomy_param = bool(self.get_parameter("autonomy_enabled").value)
        legacy_autonomous_param = bool(self.get_parameter("autonomous_enabled").value)
        self.autonomy_enabled = autonomy_param or legacy_autonomous_param
        self.autonomous_enabled = self.autonomy_enabled  # compatibility for existing status/log tooling

        self.gain_forward = float(self.get_parameter("gain_forward").value)
        self.gain_right = float(self.get_parameter("gain_right").value)
        self.gain_down = float(self.get_parameter("gain_down").value)
        self.gain_yaw = float(self.get_parameter("gain_yaw").value)

        self.deadband_x = float(self.get_parameter("deadband_x").value)
        self.deadband_y = float(self.get_parameter("deadband_y").value)

        self.max_velocity_forward = float(self.get_parameter("max_velocity_forward").value)
        self.max_velocity_right = float(self.get_parameter("max_velocity_right").value)
        self.max_velocity_down = float(self.get_parameter("max_velocity_down").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)

        self.max_accel_forward = float(self.get_parameter("max_accel_forward").value)
        self.max_accel_right = float(self.get_parameter("max_accel_right").value)
        self.max_accel_down = float(self.get_parameter("max_accel_down").value)
        self.max_yaw_accel = float(self.get_parameter("max_yaw_accel").value)

        self.min_battery_percent = float(self.get_parameter("min_battery_percent").value)
        self.require_gps = bool(self.get_parameter("require_gps").value)
        self.require_armed = bool(self.get_parameter("require_armed").value)
        self.min_altitude_m = float(self.get_parameter("min_altitude_m").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.target_timeout = float(self.get_parameter("target_timeout").value)
        self.telemetry_timeout = float(self.get_parameter("telemetry_timeout").value)
        self.target_error_topic = str(self.get_parameter("target_error_topic").value)
        self.telemetry_topic = str(self.get_parameter("telemetry_topic").value)
        self.control_command_topic = str(self.get_parameter("control_command_topic").value)
        self.autonomy_enable_topic = str(self.get_parameter("autonomy_enable_topic").value)
        self.mission_command_topic = str(self.get_parameter("mission_command_topic").value)
        self.mission_command_timeout = float(self.get_parameter("mission_command_timeout").value)
        self.desired_distance_m = float(self.get_parameter("desired_distance_m").value)
        self.distance_gain_forward = float(self.get_parameter("distance_gain_forward").value)
        self.target_area_goal = float(self.get_parameter("target_area_goal").value)
        self.orbit_speed_m_s = float(self.get_parameter("orbit_speed_m_s").value)
        self.enable_approach_translation = bool(self.get_parameter("enable_approach_translation").value)
        self.approach_distance_deadband_m = float(self.get_parameter("approach_distance_deadband_m").value)

        self.validate_parameters()
        self.control_period = 1.0 / self.control_rate

        self.last_target_error: Optional[TargetError] = None
        self.last_telemetry: Optional[DroneTelemetry] = None
        self.last_mission_command: Optional[MissionCommand] = None

        self.last_target_error_time = 0.0
        self.last_telemetry_time = 0.0
        self.last_mission_command_time = 0.0

        self.last_command_forward = 0.0
        self.last_command_right = 0.0
        self.last_command_down = 0.0
        self.last_command_yaw = 0.0

        # POSITION mode: capture the local NED position when autonomy starts,
        # then hold that coordinate while the yaw setpoint changes.
        self.position_hold_valid = False
        self.hold_position_north = 0.0
        self.hold_position_east = 0.0
        self.hold_position_down = 0.0
        self.position_yaw_target_rad = 0.0
        self.last_yaw_update_time = 0.0
        self.position_hold_anchor_source = "none"

        self.command_count = 0
        self.idle_command_count = 0
        self.executed_command_count = 0
        self.target_error_count = 0
        self.telemetry_count = 0
        self.autonomy_enable_count = 0
        self.mission_command_count = 0
        self.last_mission_mode = "TRACK_CENTER"
        self.target_locked = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.error_sub = self.create_subscription(
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

        self.autonomy_sub = self.create_subscription(
            Bool,
            self.autonomy_enable_topic,
            self.autonomy_enable_callback,
            qos,
        )

        self.mission_sub = self.create_subscription(
            MissionCommand,
            self.mission_command_topic,
            self.mission_command_callback,
            qos,
        )

        self.command_pub = self.create_publisher(
            ControlCommand,
            self.control_command_topic,
            qos,
        )

        self.control_timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        self.status_timer = self.create_timer(5.0, self.report_status)

        self.diagnostics = NodeDiagnostics(self, heartbeat_period=5.0, stale_seconds=2.0)
        self.diagnostics.add_input(self.target_error_topic, "target_error", stale_seconds=self.target_timeout)
        self.diagnostics.add_input(self.telemetry_topic, "telemetry", stale_seconds=self.telemetry_timeout)
        self.diagnostics.add_input(self.mission_command_topic, "mission_command", stale_seconds=self.mission_command_timeout)
        self.diagnostics.add_output(self.control_command_topic, "control_command")

        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.get_logger().warning(
            f"Control node started | target_error_topic={self.target_error_topic}, "
            f"telemetry_topic={self.telemetry_topic}, "
            f"control_command_topic={self.control_command_topic}, "
            f"autonomy_enable_topic={self.autonomy_enable_topic}, "
            f"mission_command_topic={self.mission_command_topic}, "
            f"autonomy_enabled={self.autonomy_enabled}, "
            "mode=MISSION_AWARE, default=TRACK_CENTER/position-hold-yaw"
        )

        if not self.autonomy_enabled:
            self.get_logger().warning(
                "AUTONOMY DISABLED - publishing IDLE/zero commands until /drone/autonomy/enabled is true."
            )

    def validate_parameters(self) -> None:
        if self.control_rate <= 0.0:
            raise ValueError(f"control_rate must be > 0, got {self.control_rate}")
        if not 0.0 <= self.deadband_x < 1.0:
            raise ValueError(f"deadband_x must be in [0, 1), got {self.deadband_x}")
        if not 0.0 <= self.deadband_y < 1.0:
            raise ValueError(f"deadband_y must be in [0, 1), got {self.deadband_y}")
        if self.target_timeout <= 0.0:
            raise ValueError(f"target_timeout must be > 0, got {self.target_timeout}")
        if self.telemetry_timeout <= 0.0:
            raise ValueError(f"telemetry_timeout must be > 0, got {self.telemetry_timeout}")
        if self.mission_command_timeout <= 0.0:
            raise ValueError(f"mission_command_timeout must be > 0, got {self.mission_command_timeout}")

        nonnegative = {
            "max_velocity_forward": self.max_velocity_forward,
            "max_velocity_right": self.max_velocity_right,
            "max_velocity_down": self.max_velocity_down,
            "max_yaw_rate": self.max_yaw_rate,
            "max_accel_forward": self.max_accel_forward,
            "max_accel_right": self.max_accel_right,
            "max_accel_down": self.max_accel_down,
            "max_yaw_accel": self.max_yaw_accel,
            "min_battery_percent": self.min_battery_percent,
            "min_altitude_m": self.min_altitude_m,
        }
        for name, value in nonnegative.items():
            if value < 0.0:
                raise ValueError(f"{name} must be >= 0, got {value}")

    def on_parameter_change(self, params) -> SetParametersResult:
        for param in params:
            if param.name not in DYNAMIC_PARAMS:
                return SetParametersResult(
                    successful=False,
                    reason=f"{param.name} is not runtime-reconfigurable",
                )

            if param.name in ("deadband_x", "deadband_y"):
                if not 0.0 <= float(param.value) < 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be in [0, 1)",
                    )

            if param.name.startswith(("max_", "min_")):
                if float(param.value) < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be >= 0",
                    )

            if param.name in ("target_timeout", "telemetry_timeout", "mission_command_timeout"):
                if float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be > 0",
                    )

        for param in params:
            # Coerce to the attribute's existing type: `ros2 param set ... 1`
            # arrives as an int and would silently replace a float gain/limit.
            old_value = getattr(self, param.name, None)
            if isinstance(old_value, bool):
                new_value = bool(param.value)
            elif isinstance(old_value, float):
                new_value = float(param.value)
            elif isinstance(old_value, int):
                new_value = int(param.value)
            else:
                new_value = param.value
            setattr(self, param.name, new_value)

            if param.name in ("autonomy_enabled", "autonomous_enabled"):
                self.set_autonomy_enabled(bool(param.value), source=f"parameter:{param.name}")

        return SetParametersResult(successful=True)

    def set_autonomy_enabled(self, enabled: bool, source: str) -> None:
        enabled = bool(enabled)
        old_enabled = self.autonomy_enabled
        self.autonomy_enabled = enabled
        self.autonomous_enabled = enabled  # compatibility alias

        if enabled and not old_enabled:
            self.get_logger().warning(f"*** AUTONOMY ENABLED by {source} ***")
        elif not enabled and old_enabled:
            self.get_logger().warning(f"Autonomy disabled by {source}; publishing IDLE/zero commands.")

        if not enabled:
            self.last_command_forward = 0.0
            self.last_command_right = 0.0
            self.last_command_down = 0.0
            self.last_command_yaw = 0.0
            if old_enabled:
                self.reset_position_hold_anchor()

            # Push a zero command immediately on disable instead of waiting for
            # the next control timer tick. During __init__, command_pub does not
            # exist yet, so guard this for startup safety.
            if hasattr(self, "command_pub"):
                source_error_x = 0.0
                source_error_y = 0.0
                if self.last_target_error is not None:
                    source_error_x = float(self.last_target_error.error_x)
                    source_error_y = float(self.last_target_error.error_y)
                self.publish_idle(
                    STATUS_BLOCKED_DISABLED,
                    source_error_x=source_error_x,
                    source_error_y=source_error_y,
                )

    def autonomy_enable_callback(self, msg: Bool) -> None:
        self.autonomy_enable_count += 1
        self.set_autonomy_enabled(bool(msg.data), source=self.autonomy_enable_topic)
        self.diagnostics.mark_received(
            self.autonomy_enable_topic,
            summary=f"messages={self.autonomy_enable_count}, enabled={self.autonomy_enabled}",
        )

    def mission_command_callback(self, msg: MissionCommand) -> None:
        self.last_mission_command = msg
        self.last_mission_command_time = time.monotonic()
        self.mission_command_count += 1
        self.last_mission_mode = str(msg.mode)

        step_name = str(getattr(msg, "step_name", "")).strip().lower()
        mode = str(msg.mode).strip().upper()
        if not msg.active or mode == "IDLE" or step_name in ("0_preflight", "1_takeoff_if_needed"):
            self.reset_position_hold_anchor()

        self.diagnostics.mark_received(
            self.mission_command_topic,
            summary=(
                f"messages={self.mission_command_count}, active={msg.active}, "
                f"mode={msg.mode}, step={msg.step_index}:{msg.step_name}, status={msg.status}"
            ),
        )

    def target_error_callback(self, msg: TargetError) -> None:
        self.last_target_error = msg
        self.last_target_error_time = time.monotonic()
        self.target_error_count += 1

        is_locked = bool(msg.target_visible and msg.tracking_state == "LOCKED")
        if is_locked and not self.target_locked:
            self.get_logger().info(
                f"Target acquired by control node | class={msg.target_class}, "
                f"confidence={msg.target_confidence:.2f}, error_x={msg.error_x:+.3f}, error_y={msg.error_y:+.3f}, "
                f"distance_valid={getattr(msg, 'distance_valid', False)}, distance_m={getattr(msg, 'distance_m', 0.0):.2f}"
            )
        elif not is_locked and self.target_locked:
            self.get_logger().warning(
                f"Target lost by control node | state={msg.tracking_state}, visible={msg.target_visible}"
            )
        self.target_locked = is_locked

        self.diagnostics.mark_received(
            self.target_error_topic,
            summary=f"messages={self.target_error_count}, state={msg.tracking_state}, visible={msg.target_visible}",
        )

    def telemetry_callback(self, msg: DroneTelemetry) -> None:
        self.last_telemetry = msg
        self.last_telemetry_time = time.monotonic()
        self.telemetry_count += 1
        self.diagnostics.mark_received(
            self.telemetry_topic,
            summary=f"messages={self.telemetry_count}, connected={msg.connected}, battery={msg.battery_remaining_percent:.1f}%",
        )

    @staticmethod
    def apply_deadband(value: float, deadband: float) -> float:
        if abs(value) < deadband:
            return 0.0

        sign = 1.0 if value > 0.0 else -1.0
        return sign * (abs(value) - deadband) / (1.0 - deadband)

    @staticmethod
    def rate_limit_value(new_value: float, old_value: float, max_change: float) -> float:
        change = new_value - old_value

        if change > max_change:
            change = max_change
        elif change < -max_change:
            change = -max_change

        return old_value + change

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    @staticmethod
    def wrap_pi(angle_rad: float) -> float:
        return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def finite(*values: float) -> bool:
        return all(math.isfinite(float(value)) for value in values)

    @staticmethod
    def yaw_rad_to_deg_0_360(yaw_rad: float) -> float:
        return math.degrees(float(yaw_rad)) % 360.0

    def reset_position_hold_anchor(self) -> None:
        self.position_hold_valid = False
        self.hold_position_north = 0.0
        self.hold_position_east = 0.0
        self.hold_position_down = 0.0
        self.position_yaw_target_rad = 0.0
        self.last_yaw_update_time = 0.0
        self.position_hold_anchor_source = "none"

    def local_position_ready(self) -> bool:
        if self.last_telemetry is None:
            return False
        return bool(getattr(self.last_telemetry, "local_position_valid", False)) and self.finite(
            getattr(self.last_telemetry, "local_position_north", float("nan")),
            getattr(self.last_telemetry, "local_position_east", float("nan")),
            getattr(self.last_telemetry, "local_position_down", float("nan")),
            getattr(self.last_telemetry, "yaw", float("nan")),
        )

    def capture_position_hold_anchor(self, current_time: float, source: str = "control") -> bool:
        if self.position_hold_valid:
            return True
        if not self.local_position_ready():
            return False

        telemetry = self.last_telemetry
        self.hold_position_north = float(telemetry.local_position_north)
        self.hold_position_east = float(telemetry.local_position_east)
        self.hold_position_down = float(telemetry.local_position_down)
        self.position_yaw_target_rad = self.wrap_pi(float(telemetry.yaw))
        self.last_yaw_update_time = current_time
        self.position_hold_valid = True
        self.position_hold_anchor_source = str(source)

        self.get_logger().warning(
            f"Captured POSITION hold anchor ({self.position_hold_anchor_source}) | "
            f"N={self.hold_position_north:.2f}m, "
            f"E={self.hold_position_east:.2f}m, "
            f"D={self.hold_position_down:.2f}m, "
            f"yaw={self.yaw_rad_to_deg_0_360(self.position_yaw_target_rad):.1f}deg"
        )
        return True

    @staticmethod
    def mission_is_prime_offboard_hold(mission: Optional[MissionCommand]) -> bool:
        if mission is None or not mission.active:
            return False
        mode = str(mission.mode).strip().upper()
        step_name = str(getattr(mission, "step_name", "")).strip().lower()
        status = str(getattr(mission, "status", "")).strip().lower()
        return (
            mode == "HOLD"
            and (
                step_name == "2_prime_offboard"
                or "priming px4 offboard" in status
            )
        )

    def update_yaw_target(self, yaw_rate_rad_s: float, current_time: float) -> float:
        if self.last_yaw_update_time <= 0.0:
            self.last_yaw_update_time = current_time
            return self.position_yaw_target_rad

        dt = max(0.0, min(current_time - self.last_yaw_update_time, 0.25))
        self.last_yaw_update_time = current_time
        self.position_yaw_target_rad = self.wrap_pi(
            self.position_yaw_target_rad + float(yaw_rate_rad_s) * dt
        )
        return self.position_yaw_target_rad

    def check_safety(self, current_time: float) -> tuple[bool, str]:
        if self.last_telemetry is None:
            return False, STATUS_BLOCKED_NO_TELEM

        telemetry_age = current_time - self.last_telemetry_time
        if telemetry_age > self.telemetry_timeout:
            return False, f"{STATUS_BLOCKED_TELEM_STALE} ({telemetry_age:.2f}s)"

        telemetry = self.last_telemetry

        if not telemetry.connected:
            return False, STATUS_BLOCKED_DISCONNECTED

        if telemetry.battery_remaining_percent < self.min_battery_percent:
            return False, STATUS_BLOCKED_LOW_BATTERY

        if self.require_gps and not telemetry.health_gps_ok:
            return False, STATUS_BLOCKED_GPS

        if self.require_armed and not telemetry.armed:
            return False, STATUS_BLOCKED_DISARMED

        return True, STATUS_SENT

    def make_command(
        self,
        command_type: str,
        status: str,
        executed: bool,
        velocity_forward: float = 0.0,
        velocity_right: float = 0.0,
        velocity_down: float = 0.0,
        yaw_rate: float = 0.0,
        position_valid: bool = False,
        position_north: float = 0.0,
        position_east: float = 0.0,
        position_down: float = 0.0,
        yaw_deg: float = 0.0,
        source_error_x: float = 0.0,
        source_error_y: float = 0.0,
    ) -> ControlCommand:
        command = ControlCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.command_type = command_type

        command.velocity_forward = float(velocity_forward)
        command.velocity_right = float(velocity_right)
        command.velocity_down = float(velocity_down)
        command.yaw_rate = float(yaw_rate)

        command.position_valid = bool(position_valid)
        command.position_north = float(position_north)
        command.position_east = float(position_east)
        command.position_down = float(position_down)
        command.yaw_deg = float(yaw_deg)

        command.executed = bool(executed)
        command.execution_status = status

        command.source_error_x = float(source_error_x)
        command.source_error_y = float(source_error_y)

        return command


    def get_active_mission_command(self, current_time: float) -> Optional[MissionCommand]:
        if self.last_mission_command is None:
            return None
        mission_age = current_time - self.last_mission_command_time
        if mission_age > self.mission_command_timeout:
            return None
        if not self.last_mission_command.active:
            return None
        return self.last_mission_command

    def get_distance_forward_correction(self, target: TargetError, desired_distance_m: float) -> float:
        if bool(getattr(target, "distance_valid", False)) and float(getattr(target, "distance_m", 0.0)) > 0.0:
            distance_error_m = float(target.distance_m) - float(desired_distance_m)
            return self.distance_gain_forward * distance_error_m

        # Fallback for old TargetError messages or before distance calibration:
        # if target area is smaller than goal, move forward; if larger, back up.
        area_error = float(self.target_area_goal) - float(target.target_area)
        return self.gain_forward * area_error

    def limit_motion(self, forward: float, right: float, down: float, yaw: float) -> tuple[float, float, float, float]:
        forward = self.clamp(forward, -self.max_velocity_forward, self.max_velocity_forward)
        right = self.clamp(right, -self.max_velocity_right, self.max_velocity_right)
        down = self.clamp(down, -self.max_velocity_down, self.max_velocity_down)
        down = self.clamp_descent_to_altitude_floor(down)
        yaw = self.clamp(yaw, -self.max_yaw_rate, self.max_yaw_rate)

        forward = self.rate_limit_value(forward, self.last_command_forward, self.max_accel_forward * self.control_period)
        right = self.rate_limit_value(right, self.last_command_right, self.max_accel_right * self.control_period)
        down = self.rate_limit_value(down, self.last_command_down, self.max_accel_down * self.control_period)
        yaw = self.rate_limit_value(yaw, self.last_command_yaw, self.max_yaw_accel * self.control_period)
        return forward, right, down, yaw

    def clamp_descent_to_altitude_floor(self, down: float) -> float:
        # NED down is positive descent. The floor only constrains descent, never climb.
        if down <= 0.0 or self.min_altitude_m <= 0.0 or self.last_telemetry is None:
            return down

        altitude = float(getattr(self.last_telemetry, "relative_altitude", float("nan")))
        if not math.isfinite(altitude):
            return down

        margin_m = altitude - self.min_altitude_m
        if margin_m <= 0.0:
            return 0.0

        max_descent_rate = margin_m / max(self.control_period, 1e-6)
        return min(down, max_descent_rate)

    def publish_position_hold(
        self,
        status: str,
        yaw_rate: float = 0.0,
        source_error_x: float = 0.0,
        source_error_y: float = 0.0,
        update_yaw: bool = False,
    ) -> None:
        # Revalidate local position every cycle. The hold anchor is captured once,
        # but if the EKF/GPS later drops out, blindly re-sending the stale absolute
        # NED setpoint would fly the drone to a wrong coordinate. Fall back to IDLE
        # so PX4 holds on its own failsafe instead of chasing a stale anchor.
        if not self.local_position_ready():
            self.reset_position_hold_anchor()
            self.publish_idle(STATUS_BLOCKED_NO_LOCAL_POSITION)
            return
        current_time = time.monotonic()
        if update_yaw:
            yaw_rad = self.update_yaw_target(yaw_rate, current_time)
        else:
            yaw_rad = self.position_yaw_target_rad
            self.last_yaw_update_time = current_time

        yaw_deg = self.yaw_rad_to_deg_0_360(yaw_rad)

        command = self.make_command(
            CMD_POSITION,
            status,
            executed=True,
            velocity_forward=0.0,
            velocity_right=0.0,
            velocity_down=0.0,
            yaw_rate=float(yaw_rate),
            position_valid=True,
            position_north=self.hold_position_north,
            position_east=self.hold_position_east,
            position_down=self.hold_position_down,
            yaw_deg=yaw_deg,
            source_error_x=source_error_x,
            source_error_y=source_error_y,
        )

        self.last_command_forward = 0.0
        self.last_command_right = 0.0
        self.last_command_down = 0.0
        self.last_command_yaw = float(yaw_rate)

        self.command_pub.publish(command)
        self.command_count += 1
        self.executed_command_count += 1

        self.diagnostics.mark_published(
            self.control_command_topic,
            summary=(
                f"commands={self.command_count}, executed={self.executed_command_count}, "
                f"type={CMD_POSITION}, N={self.hold_position_north:.2f}, "
                f"E={self.hold_position_east:.2f}, D={self.hold_position_down:.2f}, "
                f"yaw_deg={yaw_deg:.1f}, yaw_rate={yaw_rate:.3f}, status={status}"
            ),
        )

    def publish_velocity(
        self,
        status: str,
        velocity_forward: float = 0.0,
        velocity_right: float = 0.0,
        velocity_down: float = 0.0,
        yaw_rate: float = 0.0,
        source_error_x: float = 0.0,
        source_error_y: float = 0.0,
    ) -> None:
        # Body-frame VELOCITY setpoint. Only reached for opt-in APPROACH
        # translation; translation is still gated downstream by
        # telemetry_node.allow_translation_commands. Local position must be valid
        # so we fail safe (PX4 holds) if the EKF/GPS drops out mid-approach.
        if not self.local_position_ready():
            self.reset_position_hold_anchor()
            self.publish_idle(STATUS_BLOCKED_NO_LOCAL_POSITION)
            return

        down = self.clamp_descent_to_altitude_floor(float(velocity_down))
        command = self.make_command(
            CMD_VELOCITY,
            status,
            executed=True,
            velocity_forward=float(velocity_forward),
            velocity_right=float(velocity_right),
            velocity_down=down,
            yaw_rate=float(yaw_rate),
            position_valid=False,
            source_error_x=source_error_x,
            source_error_y=source_error_y,
        )

        self.last_command_forward = float(velocity_forward)
        self.last_command_right = float(velocity_right)
        self.last_command_down = down
        self.last_command_yaw = float(yaw_rate)

        self.command_pub.publish(command)
        self.command_count += 1
        self.executed_command_count += 1

        self.diagnostics.mark_published(
            self.control_command_topic,
            summary=(
                f"commands={self.command_count}, executed={self.executed_command_count}, "
                f"type={CMD_VELOCITY}, fwd={velocity_forward:.3f}, yaw_rate={yaw_rate:.3f}, status={status}"
            ),
        )

    def publish_idle(
        self,
        status: str,
        source_error_x: float = 0.0,
        source_error_y: float = 0.0,
        desired_yaw: float | None = None,
    ) -> None:
        # IDLE must always mean no real movement command is being sent.
        # desired_yaw is debug-only so we can see what yaw WOULD have been
        # commanded if autonomy/safety gates allowed movement.
        self.last_command_forward = 0.0
        self.last_command_right = 0.0
        self.last_command_down = 0.0
        self.last_command_yaw = 0.0

        debug_status = status
        if desired_yaw is not None:
            debug_status = f"{status} | desired_yaw={desired_yaw:.3f}"

        command = self.make_command(
            CMD_IDLE,
            debug_status,
            executed=False,
            source_error_x=source_error_x,
            source_error_y=source_error_y,
        )

        # Extra safety: make sure idle command cannot accidentally carry motion.
        command.velocity_forward = 0.0
        command.velocity_right = 0.0
        command.velocity_down = 0.0
        command.yaw_rate = 0.0
        command.position_valid = False
        command.position_north = 0.0
        command.position_east = 0.0
        command.position_down = 0.0
        command.yaw_deg = 0.0

        self.command_pub.publish(command)
        self.command_count += 1
        self.idle_command_count += 1

        self.diagnostics.mark_published(
            self.control_command_topic,
            summary=(
                f"commands={self.command_count}, "
                f"idle={self.idle_command_count}, "
                f"status={debug_status}"
            ),
        )

    def control_loop(self) -> None:
        current_time = time.monotonic()
        desired_yaw = 0.0
        mission = self.get_active_mission_command(current_time)
        mission_mode = "TRACK_CENTER" if mission is None else str(mission.mode).strip().upper()

        if not self.autonomy_enabled:
            if self.mission_is_prime_offboard_hold(mission) and not self.position_hold_valid:
                safe, _ = self.check_safety(current_time)
                if safe:
                    self.capture_position_hold_anchor(current_time, source="mission_prime_offboard")

            source_error_x = 0.0
            source_error_y = 0.0
            if self.last_target_error is not None:
                source_error_x = float(self.last_target_error.error_x)
                source_error_y = float(self.last_target_error.error_y)
            self.publish_idle(
                STATUS_BLOCKED_DISABLED,
                source_error_x=source_error_x,
                source_error_y=source_error_y,
            )
            return

        safe, reason = self.check_safety(current_time)
        if not safe:
            self.publish_idle(reason)
            return

        if not self.capture_position_hold_anchor(current_time, source="tracking_start"):
            self.publish_idle(STATUS_BLOCKED_NO_LOCAL_POSITION)
            return

        # Stage 1 mission: hold the captured takeoff/local NED coordinate and
        # only change yaw. This prevents body-frame velocity drift from wind/GPS
        # controller bias while we test YOLO yaw behavior.
        if mission_mode in ("IDLE", "HOLD"):
            self.publish_position_hold(f"MISSION_{mission_mode}", yaw_rate=0.0, update_yaw=False)
            return

        if mission_mode == "FLY_FORWARD":
            self.publish_position_hold("POSITION_HOLD_TRANSLATION_DISABLED_FOR_STAGE1", yaw_rate=0.0, update_yaw=False)
            return

        if mission_mode == "SCAN":
            # Open-loop yaw sweep: hold the captured local-NED anchor and rotate
            # in place at the executor-commanded yaw rate. No target is required
            # and no translation is ever commanded, so this stays inside the
            # existing position-hold+yaw safety envelope. yaw_rate is clamped and
            # rate-limited by limit_motion like any other yaw command.
            commanded_yaw = float(mission.yaw_rate) if mission is not None else 0.0
            _, _, _, yaw = self.limit_motion(0.0, 0.0, 0.0, commanded_yaw)
            self.publish_position_hold("MISSION_SCAN", yaw_rate=yaw, update_yaw=True)
            return

        if self.last_target_error is None:
            self.publish_position_hold(STATUS_NO_TARGET, yaw_rate=0.0, update_yaw=False)
            return

        target_age = current_time - self.last_target_error_time
        if target_age > self.target_timeout:
            if self.target_locked:
                self.get_logger().warning(
                    f"Target lost by control node | target_error stale for {target_age:.2f}s"
                )
                self.target_locked = False

            self.publish_position_hold(
                f"{STATUS_TARGET_STALE} ({target_age:.2f}s)",
                yaw_rate=0.0,
                source_error_x=float(self.last_target_error.error_x),
                source_error_y=float(self.last_target_error.error_y),
                update_yaw=False,
            )
            return

        target = self.last_target_error
        error_x = self.apply_deadband(float(target.error_x), self.deadband_x)
        desired_yaw = error_x * self.gain_yaw

        if not (target.target_visible and target.tracking_state == "LOCKED"):
            self.publish_position_hold(
                STATUS_TARGET_NOT_VISIBLE,
                yaw_rate=0.0,
                source_error_x=float(target.error_x),
                source_error_y=float(target.error_y),
                update_yaw=False,
            )
            return

        # Opt-in APPROACH translation (default OFF, double-gated): command forward
        # velocity from the distance estimate while keeping the yaw centering.
        # Disabled -> falls through to the position-hold+yaw behavior below.
        if mission_mode == "APPROACH_TARGET" and self.enable_approach_translation:
            desired_distance_m = (
                float(mission.desired_distance_m) if mission is not None else self.desired_distance_m
            )
            forward_cmd = approach_forward_velocity(
                distance_valid=bool(getattr(target, "distance_valid", False)),
                distance_m=float(getattr(target, "distance_m", 0.0)),
                desired_distance_m=desired_distance_m,
                gain=self.distance_gain_forward,
                max_speed=self.max_velocity_forward,
                deadband_m=self.approach_distance_deadband_m,
                target_locked=True,  # target is verified visible + LOCKED above
            )
            forward, _, _, yaw = self.limit_motion(forward_cmd, 0.0, 0.0, desired_yaw)
            self.publish_velocity(
                f"APPROACH_TRANSLATION(d={desired_distance_m:.2f}m)",
                velocity_forward=forward,
                yaw_rate=yaw,
                source_error_x=float(target.error_x),
                source_error_y=float(target.error_y),
            )
            return

        # No forward/right/down in this stage. APPROACH/ORBIT stay in the same
        # position-hold+yaw behavior until we add true NED waypoint math.
        _, _, _, yaw = self.limit_motion(0.0, 0.0, 0.0, desired_yaw)

        if mission_mode not in ("TRACK_CENTER", "APPROACH_TARGET", "ORBIT_TARGET"):
            self.publish_position_hold(
                f"UNKNOWN_MISSION_MODE:{mission_mode}",
                yaw_rate=0.0,
                source_error_x=float(target.error_x),
                source_error_y=float(target.error_y),
                update_yaw=False,
            )
            return

        self.publish_position_hold(
            STATUS_SENT,
            yaw_rate=yaw,
            source_error_x=float(target.error_x),
            source_error_y=float(target.error_y),
            update_yaw=True,
        )

    def report_status(self) -> None:
        self.get_logger().info(
            f"Control status | autonomy={self.autonomy_enabled}, target_locked={self.target_locked}, "
            f"commands={self.command_count}, executed={self.executed_command_count}, idle={self.idle_command_count}, "
            f"target_msgs={self.target_error_count}, telemetry_msgs={self.telemetry_count}, "
            f"autonomy_msgs={self.autonomy_enable_count}, mission_msgs={self.mission_command_count}, "
            f"mission_mode={self.last_mission_mode}, "
            f"anchor_valid={self.position_hold_valid}, anchor_source={self.position_hold_anchor_source}, "
            f"target_age={self.diagnostics.format_age(self.target_error_topic)}, "
            f"telemetry_age={self.diagnostics.format_age(self.telemetry_topic)}, "
            f"forward={self.last_command_forward:.3f}, "
            f"right={self.last_command_right:.3f}, "
            f"down={self.last_command_down:.3f}, "
            f"yaw={self.last_command_yaw:.3f}"
        )

    def destroy_node(self) -> None:
        self.get_logger().info("Control node shut down.")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger("control_node").fatal(f"Fatal: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
