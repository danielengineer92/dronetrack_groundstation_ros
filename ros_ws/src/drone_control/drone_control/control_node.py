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

from drone_interfaces.msg import ControlCommand, DroneTelemetry, MissionCommand, TargetError, TargetState
from drone_diagnostics.node_diagnostics import NodeDiagnostics
from drone_control.control_math import (
    alignment_scale,
    approach_forward_velocity,
    braking_speed_limit,
    los_rate_from_state,
    yaw_feedforward_step,
    yaw_pid_step,
)


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
    "yaw_ki", "yaw_kd", "yaw_i_limit", "yaw_d_lpf_alpha",
    "yaw_ff_gain", "yaw_ff_lpf_alpha", "yaw_ff_limit", "yaw_ff_source",
    "yaw_ff_align_error_limit", "yaw_ff_align_inner", "yaw_ff_state_lpf_alpha",
    "log_yaw_terms",
    "yaw_approach_limit_enabled", "yaw_approach_delay_s",
    "yaw_approach_decel", "yaw_approach_min_rate", "camera_half_fov_rad",
    "approach_delay_s", "approach_decel", "approach_align_error_limit",
    "enable_approach_translation", "approach_distance_deadband_m",
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
        # Yaw PID extras (gain_yaw is the P term). Defaults keep the
        # controller pure-P; tune live: ros2 param set /control_node yaw_ki 0.1
        self.declare_parameter("yaw_ki", 0.0)
        self.declare_parameter("yaw_kd", 0.0)
        self.declare_parameter("yaw_i_limit", 0.3)      # max |I| contribution, rad/s
        self.declare_parameter("yaw_d_lpf_alpha", 0.35)  # D low-pass, (0,1], 1=raw
        # LOS-rate feed-forward: command the rate the target is moving at so
        # the loop stops trailing a moving target (error no longer has to be
        # nonzero to sustain yaw). 0 = off; 1 = full estimated LOS rate.
        self.declare_parameter("yaw_ff_gain", 0.0)
        self.declare_parameter("yaw_ff_lpf_alpha", 0.25)  # LOS-rate low-pass
        self.declare_parameter("yaw_ff_limit", 1.5)       # rad/s cap on the FF term
        # FF is scaled from 1 (target centred) to 0 at this normalized |error_x|,
        # so a fresh-lock/off-centre FF estimate (esp. the state KF's velocity
        # transient) cannot spike the yaw during acquisition. 0 disables the gate.
        self.declare_parameter("yaw_ff_align_error_limit", 0.6)
        # Optional flat zone of the FF alignment gate: full FF (scale 1.0) for
        # |error_x| <= inner, ramp only between inner and the limit. Removes the
        # gate's in-band modulation during steady tracking (the sloped gate
        # couples the error wobble into the FF as sign-switching extra gain).
        # 0 = original pure ramp. Offline A/B on the orbiting ball showed the
        # capture-stamp pairing fix makes this unnecessary in sim; kept as a
        # live knob for hardware, where delays are larger.
        self.declare_parameter("yaw_ff_align_inner", 0.0)
        # Optional low-pass on the "state"-source FF (same blend the los_diff
        # branch already has): 1.0 = off (current behaviour). The state LOS
        # rate divides by range^2, so its noise blows up exactly at the
        # nearest point of a passing target; a small filter (e.g. 0.35) trades
        # ~90 ms of FF lag for a quieter rate command.
        self.declare_parameter("yaw_ff_state_lpf_alpha", 1.0)
        # Bring-up instrumentation (default OFF). When true, log the yaw-law
        # term breakdown at ~2 Hz while tracking: P / D / FF terms, the capped
        # centring command, the raw (pre-limit) command, the clamped command
        # actually sent, and the achieved yaw rate (differentiated telemetry).
        # For hardware first-flight: confirm the FF term has the SAME sign as
        # the P term when the target is off-centre and moving, and watch whether
        # the D term is doing useful work or just tracking sensor noise.
        self.declare_parameter("log_yaw_terms", False)
        # FF source: "los_diff" differentiates the measured LOS angle (carries
        # the ~0.5 s vision latency); "state" computes the LOS rate
        # geometrically from the target state estimator's PREDICTED state
        # (latency-cancelled), falling back to los_diff when the estimate is
        # invalid/stale.
        self.declare_parameter("yaw_ff_source", "los_diff")
        self.declare_parameter("target_state_topic", "/drone/tracking/target_state")
        self.declare_parameter("target_state_max_age_s", 1.0)
        # Delay-aware approach limiter: caps the yaw CENTRING command (P+I+D, not
        # the feed-forward) to a trapezoidal motion profile in bearing space so a
        # target we start pointed far away from is closed on quickly but WITHOUT
        # sailing past centre and swinging back. yaw_approach_delay_s is the vision
        # dead time the profile must stop within; yaw_approach_decel is how hard we
        # are willing to brake the yaw; yaw_approach_min_rate keeps a little
        # authority for fine corrections just outside the deadband. Disable to get
        # the old raw-P slew: ros2 param set /control_node yaw_approach_limit_enabled false
        self.declare_parameter("yaw_approach_limit_enabled", True)
        self.declare_parameter("yaw_approach_delay_s", 0.35)
        self.declare_parameter("yaw_approach_decel", 0.8)
        self.declare_parameter("yaw_approach_min_rate", 0.15)
        # Fallback camera half-FOV (rad) used to convert normalized error_x to an
        # approximate bearing angle when TargetError.bearing_x_rad is absent (old
        # messages). ~0.61 rad = 35 deg = half of a 70 deg horizontal FOV.
        self.declare_parameter("camera_half_fov_rad", 0.61)

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
        # Professional approach profile: brake into the standoff distance
        # instead of lunging past it (the vision distance is ~delay_s stale),
        # and never charge forward while the target is still off-centre.
        # approach_decel is how hard we are willing to brake (m/s^2);
        # approach_align_error_limit is the normalized |error_x| at which the
        # closing speed reaches zero (0 disables the alignment gate).
        self.declare_parameter("approach_delay_s", 0.35)
        self.declare_parameter("approach_decel", 0.5)
        self.declare_parameter("approach_align_error_limit", 0.5)

        autonomy_param = bool(self.get_parameter("autonomy_enabled").value)
        legacy_autonomous_param = bool(self.get_parameter("autonomous_enabled").value)
        self.autonomy_enabled = autonomy_param or legacy_autonomous_param
        self.autonomous_enabled = self.autonomy_enabled  # compatibility for existing status/log tooling

        self.gain_forward = float(self.get_parameter("gain_forward").value)
        self.gain_right = float(self.get_parameter("gain_right").value)
        self.gain_down = float(self.get_parameter("gain_down").value)
        self.gain_yaw = float(self.get_parameter("gain_yaw").value)
        self.yaw_ki = float(self.get_parameter("yaw_ki").value)
        self.yaw_kd = float(self.get_parameter("yaw_kd").value)
        self.yaw_i_limit = float(self.get_parameter("yaw_i_limit").value)
        self.yaw_d_lpf_alpha = float(self.get_parameter("yaw_d_lpf_alpha").value)
        self.yaw_ff_gain = float(self.get_parameter("yaw_ff_gain").value)
        self.yaw_ff_lpf_alpha = float(self.get_parameter("yaw_ff_lpf_alpha").value)
        self.yaw_ff_limit = float(self.get_parameter("yaw_ff_limit").value)
        self.yaw_ff_align_error_limit = float(self.get_parameter("yaw_ff_align_error_limit").value)
        self.yaw_ff_align_inner = float(self.get_parameter("yaw_ff_align_inner").value)
        self.yaw_ff_state_lpf_alpha = float(self.get_parameter("yaw_ff_state_lpf_alpha").value)
        self.log_yaw_terms = bool(self.get_parameter("log_yaw_terms").value)
        self.yaw_ff_source = str(self.get_parameter("yaw_ff_source").value)
        self.target_state_max_age_s = float(self.get_parameter("target_state_max_age_s").value)
        self.yaw_approach_limit_enabled = bool(self.get_parameter("yaw_approach_limit_enabled").value)
        self.yaw_approach_delay_s = float(self.get_parameter("yaw_approach_delay_s").value)
        self.yaw_approach_decel = float(self.get_parameter("yaw_approach_decel").value)
        self.yaw_approach_min_rate = float(self.get_parameter("yaw_approach_min_rate").value)
        self.camera_half_fov_rad = float(self.get_parameter("camera_half_fov_rad").value)
        self.last_target_state: Optional[TargetState] = None
        self.last_target_state_time = 0.0
        self._yaw_ff = 0.0
        self._yaw_ff_prev_los = 0.0
        self._yaw_ff_have_los = False
        # Yaw PID state (owned here; yaw_pid_step is pure).
        self._yaw_pid_integral = 0.0
        self._yaw_pid_derivative = 0.0
        self._yaw_pid_prev_error = 0.0
        self._yaw_pid_prev_raw_error = 0.0
        self._yaw_pid_last_time = 0.0
        self._yaw_terms = {}
        self._log_prev_yaw = None
        self._log_prev_yaw_t = 0.0

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
        self.approach_delay_s = float(self.get_parameter("approach_delay_s").value)
        self.approach_decel = float(self.get_parameter("approach_decel").value)
        self.approach_align_error_limit = float(self.get_parameter("approach_align_error_limit").value)

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
        self.target_state_sub = self.create_subscription(
            TargetState, str(self.get_parameter("target_state_topic").value),
            self.on_target_state,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=5))

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

            if param.name in ("yaw_ki", "yaw_kd", "yaw_i_limit"):
                if float(param.value) < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be >= 0",
                    )

            if param.name in ("yaw_d_lpf_alpha", "yaw_ff_lpf_alpha",
                              "yaw_ff_state_lpf_alpha"):
                if not 0.0 < float(param.value) <= 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be in (0, 1]",
                    )

            if param.name == "yaw_ff_source":
                if str(param.value) not in ("los_diff", "state"):
                    return SetParametersResult(
                        successful=False,
                        reason="yaw_ff_source must be 'los_diff' or 'state'",
                    )

            if param.name in ("yaw_ff_gain", "yaw_ff_limit", "yaw_ff_align_error_limit",
                              "yaw_ff_align_inner"):
                if float(param.value) < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be >= 0",
                    )

            if param.name in ("yaw_approach_delay_s", "yaw_approach_min_rate", "camera_half_fov_rad",
                              "approach_delay_s", "approach_decel", "approach_align_error_limit",
                              "approach_distance_deadband_m"):
                if float(param.value) < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be >= 0",
                    )

            if param.name == "yaw_approach_decel":
                if float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="yaw_approach_decel must be > 0",
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

            if param.name in ("gain_yaw", "yaw_ki", "yaw_kd", "yaw_i_limit",
                              "yaw_d_lpf_alpha", "yaw_ff_gain", "yaw_ff_lpf_alpha",
                              "yaw_ff_limit", "yaw_ff_source"):
                self._reset_yaw_pid()

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

    def on_target_state(self, msg: TargetState) -> None:
        self.last_target_state = msg
        self.last_target_state_time = time.monotonic()

    def _target_state_usable(self) -> bool:
        return (
            self.last_target_state is not None
            and self.last_target_state.valid
            and time.monotonic() - self.last_target_state_time <= self.target_state_max_age_s
            and self.last_telemetry is not None
            and bool(getattr(self.last_telemetry, "local_position_valid", False))
        )

    def _reset_yaw_pid(self) -> None:
        self._yaw_pid_integral = 0.0
        self._yaw_pid_derivative = 0.0
        self._yaw_pid_prev_error = 0.0
        self._yaw_pid_prev_raw_error = 0.0
        self._yaw_pid_last_time = 0.0
        self._yaw_ff = 0.0
        self._yaw_ff_prev_los = 0.0
        self._yaw_ff_have_los = False

    def step_yaw_pid(self, error_x: float, current_time: float,
                     bearing_x_rad: float = 0.0,
                     raw_error_x: float | None = None) -> float:
        """Yaw PID on the deadbanded image error (gain_yaw = P term).

        ``raw_error_x`` (pre-deadband) feeds the D term when provided: the
        deadband rescale zeroes the slope inside the band and kinks it at the
        boundary — exactly where a centred tracking loop lives — so D on the
        deadbanded error would see phantom slope discontinuities at every
        band crossing. P and I stay on the deadbanded error.

        State auto-resets after any interruption: if this was not called for
        a few control periods (target lost/stale, SCAN, IDLE, autonomy off),
        the integral and derivative history are meaningless — start fresh.
        """
        gap = current_time - self._yaw_pid_last_time
        first_sample = self._yaw_pid_last_time <= 0.0 or gap > 3.0 * self.control_period
        if first_sample:
            self._reset_yaw_pid()

        output, self._yaw_pid_integral, self._yaw_pid_derivative = yaw_pid_step(
            error=error_x,
            dt=self.control_period,
            kp=self.gain_yaw,
            ki=self.yaw_ki,
            kd=self.yaw_kd,
            integral=self._yaw_pid_integral,
            filtered_derivative=self._yaw_pid_derivative,
            prev_error=self._yaw_pid_prev_error,
            first_sample=first_sample,
            output_limit=self.max_yaw_rate,
            integral_limit=self.yaw_i_limit,
            derivative_alpha=self.yaw_d_lpf_alpha,
            derivative_error=raw_error_x,
            prev_derivative_error=self._yaw_pid_prev_raw_error,
        )
        # Delay-aware approach limiter. Cap the CENTRING command (P+I+D) so a
        # far-off-bearing target is closed on without overshooting/swinging past
        # centre. Applied here, before the feed-forward is added, so a fast-moving
        # but already-centred target is never strangled (its rate comes from FF,
        # not from a large centring error). Uses the true camera bearing when
        # available; falls back to error_x scaled by the nominal half-FOV.
        # Per-term breakdown for bring-up logging (log_yaw_terms). P/I/D are the
        # individual contributions BEFORE the braking cap clips their sum; FF is
        # captured below as (raw - pid_capped).
        pid_sum = output
        cap_val = None
        if self.yaw_approach_limit_enabled:
            angle_rad = abs(float(bearing_x_rad))
            if angle_rad <= 1e-6:
                angle_rad = abs(float(error_x)) * self.camera_half_fov_rad
            cap = braking_speed_limit(
                remaining=angle_rad,
                delay_s=self.yaw_approach_delay_s,
                decel=self.yaw_approach_decel,
                max_speed=self.max_yaw_rate,
                min_speed=self.yaw_approach_min_rate,
            )
            cap_val = cap
            output = self.clamp(output, -cap, cap)
        pid_capped = output  # centring command after the cap, before FF is added
        # Feed-forward runs on MEASURED/estimated quantities only; the
        # command never feeds back into its own estimate.
        #
        # Alignment gate: FF exists to carry an ALREADY-CENTRED moving target at
        # its own LOS rate. During acquisition (target far off-centre) the FF
        # estimate is both unnecessary and unreliable — the state KF reads the
        # re-lock position jump as a huge velocity and would spike the yaw right
        # as the feedback crosses centre, swinging past. So scale FF from 1 at
        # centre to 0 at |error_x| >= yaw_ff_align_error_limit; the feedback
        # (P+I+D + anti-swing cap) owns the reel-in, FF owns steady tracking.
        ff_scale = alignment_scale(error_x=error_x,
                                   error_limit=self.yaw_ff_align_error_limit,
                                   inner=self.yaw_ff_align_inner)
        if self.yaw_ff_gain > 0.0 and self.last_telemetry is not None:
            use_state = self.yaw_ff_source == "state" and self._target_state_usable()
            active = "state" if use_state else "los_diff"
            if active != getattr(self, "_ff_active_source", None):
                self._ff_active_source = active
                self.get_logger().info(f"Yaw FF source active: {active}")
            if use_state:
                # Latency-cancelled: LOS rate from the estimator's PREDICTED
                # relative state (KF already smooths; clamp only).
                ts = self.last_target_state
                tel = self.last_telemetry
                ff = los_rate_from_state(
                    rel_north_m=float(ts.predicted_north) - float(tel.local_position_north),
                    rel_east_m=float(ts.predicted_east) - float(tel.local_position_east),
                    rel_velocity_north_m_s=float(ts.velocity_north) - float(tel.velocity_north),
                    rel_velocity_east_m_s=float(ts.velocity_east) - float(tel.velocity_east),
                )
                ff = self.clamp(ff, -self.yaw_ff_limit, self.yaw_ff_limit)
                # Optional low-pass (yaw_ff_state_lpf_alpha < 1): the state
                # LOS rate divides by range^2, so it is noisiest exactly at a
                # passing target's nearest point. alpha=1.0 = original
                # unfiltered behaviour.
                a = self.clamp(self.yaw_ff_state_lpf_alpha, 1e-3, 1.0)
                self._yaw_ff = a * ff + (1.0 - a) * self._yaw_ff
                output += ff_scale * self.yaw_ff_gain * self._yaw_ff
                # keep the los_diff state warm for a seamless fallback
                self._yaw_ff_prev_los = float(tel.yaw) + float(bearing_x_rad)
                self._yaw_ff_have_los = True
            else:
                los = float(self.last_telemetry.yaw) + float(bearing_x_rad)
                if not self._yaw_ff_have_los or first_sample:
                    self._yaw_ff = 0.0
                else:
                    self._yaw_ff = yaw_feedforward_step(
                        los_angle_rad=los,
                        prev_los_angle_rad=self._yaw_ff_prev_los,
                        dt=self.control_period,
                        prev_ff_rad_s=self._yaw_ff,
                        first_sample=False,
                        lpf_alpha=self.yaw_ff_lpf_alpha,
                        limit_rad_s=self.yaw_ff_limit,
                    )
                    output += ff_scale * self.yaw_ff_gain * self._yaw_ff
                self._yaw_ff_prev_los = los
                self._yaw_ff_have_los = True
        self._yaw_pid_prev_error = float(error_x)
        if raw_error_x is not None:
            self._yaw_pid_prev_raw_error = float(raw_error_x)
        self._yaw_pid_last_time = float(current_time)
        # Term breakdown for the bring-up log (raw = pid_capped + FF).
        self._yaw_terms = {
            "p": self.gain_yaw * error_x,
            "i": self.yaw_ki * self._yaw_pid_integral,
            "d": self.yaw_kd * self._yaw_pid_derivative,
            "pid": pid_sum,
            "cap": cap_val,
            "ff": output - pid_capped,
            "raw": output,
        }
        return output

    def _log_yaw_terms(self, error_x: float, raw_yaw: float, clamped_yaw: float,
                       current_time: float) -> None:
        """Bring-up log: yaw-law term breakdown + achieved rate (~2 Hz).

        Off unless ``log_yaw_terms`` is set. ``achieved`` is the yaw rate the
        airframe actually reached, differentiated from telemetry yaw (there is
        no telemetry yaw-rate field). Use it to (1) confirm the FF term shares
        the P term's sign on a moving off-centre target, (2) judge whether the
        D term is real signal or sensor noise, and (3) see how hard the limiter
        is clipping (raw vs clamped)."""
        if not self.log_yaw_terms:
            return
        t = self._yaw_terms
        achieved = float("nan")
        tel = self.last_telemetry
        if tel is not None:
            yaw = float(tel.yaw)
            if self._log_prev_yaw is not None:
                dt = current_time - self._log_prev_yaw_t
                if dt > 1e-3:
                    dyaw = math.atan2(math.sin(yaw - self._log_prev_yaw),
                                      math.cos(yaw - self._log_prev_yaw))
                    achieved = dyaw / dt
            self._log_prev_yaw = yaw
            self._log_prev_yaw_t = current_time
        cap = t.get("cap")
        cap_s = f"{cap:.3f}" if cap is not None else "off"
        self.get_logger().info(
            "YAWTERMS "
            f"err={error_x:+.3f} | P={t.get('p', 0.0):+.3f} D={t.get('d', 0.0):+.3f} "
            f"FF={t.get('ff', 0.0):+.3f} I={t.get('i', 0.0):+.3f} | cap={cap_s} "
            f"raw={raw_yaw:+.3f} clamped={clamped_yaw:+.3f} achieved={achieved:+.3f} rad/s",
            throttle_duration_sec=0.5,
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
        desired_yaw = self.step_yaw_pid(
            error_x, current_time,
            bearing_x_rad=float(getattr(target, "bearing_x_rad", 0.0)),
            raw_error_x=float(target.error_x),
        )

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
                delay_s=self.approach_delay_s,
                decel_m_s2=self.approach_decel,
                error_x=float(target.error_x),
                align_error_limit=self.approach_align_error_limit,
            )
            forward, _, _, yaw = self.limit_motion(forward_cmd, 0.0, 0.0, desired_yaw)
            # Status MUST start with STATUS_SENT: the telemetry bridge treats a
            # SENT-prefixed execution_status as the control node's approval
            # marker for VELOCITY commands and drops anything else.
            self.publish_velocity(
                f"{STATUS_SENT}: APPROACH d={desired_distance_m:.2f}m",
                velocity_forward=forward,
                yaw_rate=yaw,
                source_error_x=float(target.error_x),
                source_error_y=float(target.error_y),
            )
            return

        # No forward/right/down in this stage. APPROACH/ORBIT stay in the same
        # position-hold+yaw behavior until we add true NED waypoint math.
        _, _, _, yaw = self.limit_motion(0.0, 0.0, 0.0, desired_yaw)
        self._log_yaw_terms(error_x, desired_yaw, yaw, current_time)

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
