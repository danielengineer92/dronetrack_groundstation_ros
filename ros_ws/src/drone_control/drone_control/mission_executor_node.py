"""
Smart mission executor node for the PX4/ROS 2 object-tracking drone.

Operator intent is now simple:
    System Ready     -> autonomy request true
    Start Mission    -> this node owns the sequence
    Abort / Hold     -> stop mission motion and request HOLD
    Land             -> request LAND through the MAVSDK action gate

Start Mission sequence:
    1. Check telemetry connected/fresh
    2. Check PX4 is armed
    3. If landed or below the airborne threshold, request TAKEOFF
    4. Wait until airborne
    5. Request MAVSDK Offboard through the autonomy manager
    6. Prime Offboard with zero/HOLD setpoints
    7. Publish TRACK_CENTER so the control node can yaw toward the YOLO target

This node does not arm the vehicle. MAVSDK actions are still gated by telemetry_node
through allow_mavsdk_actions, so real hardware can keep those actions disabled while
SITL/dev setups can enable them intentionally.
"""

from __future__ import annotations

import json
import math
import time
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Optional, TextIO

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from drone_interfaces.msg import DroneTelemetry, MavsdkActionCommand, MissionCommand, TargetError
from drone_diagnostics.node_diagnostics import NodeDiagnostics
from drone_control.mission_plan import (
    MissionPlan,
    MissionPlanError,
    build_default_plan,
    lint_plan,
    load_mission_plan,
)


class MissionState(Enum):
    DISABLED = "DISABLED"
    IDLE = "IDLE"
    HOLD = "HOLD"
    PREFLIGHT = "PREFLIGHT"
    TAKEOFF = "TAKEOFF"
    PRIME_OFFBOARD = "PRIME_OFFBOARD"
    SCAN = "SCAN"
    TRACK_CENTER = "TRACK_CENTER"
    APPROACH_TARGET = "APPROACH_TARGET"
    DO_ORBIT = "DO_ORBIT"
    GOTO_RELATIVE = "GOTO_RELATIVE"
    GOTO_ABSOLUTE = "GOTO_ABSOLUTE"
    RETURN_TO_LAUNCH = "RETURN_TO_LAUNCH"
    LAND = "LAND"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"


STEP_NAMES = {
    MissionState.PREFLIGHT: "0_preflight",
    MissionState.TAKEOFF: "1_takeoff_if_needed",
    MissionState.PRIME_OFFBOARD: "2_prime_offboard",
    MissionState.SCAN: "3_scan_for_target",
    MissionState.TRACK_CENTER: "4_track_center_yaw",
    MissionState.APPROACH_TARGET: "5_approach_target",
    MissionState.DO_ORBIT: "6_orbit_target",
    MissionState.GOTO_RELATIVE: "goto_relative",
    MissionState.GOTO_ABSOLUTE: "goto_absolute",
    MissionState.RETURN_TO_LAUNCH: "7_return_home",
    MissionState.LAND: "8_land",
}


class MissionExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_executor_node")

        self.declare_parameter("mission_enabled", False)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("mission_request_topic", "/drone/mission/request")
        self.declare_parameter("mission_command_topic", "/drone/mission/command")
        self.declare_parameter("mission_state_topic", "/drone/mission/state")
        self.declare_parameter("mavsdk_action_topic", "/drone/mavsdk/action_command")
        self.declare_parameter("autonomy_request_topic", "/drone/autonomy/request")
        self.declare_parameter("offboard_request_topic", "/drone/mavsdk/offboard_request")
        self.declare_parameter("target_error_topic", "/drone/tracking/target_error")
        self.declare_parameter("telemetry_topic", "/drone/telemetry")
        self.declare_parameter("publish_rate", 10.0)

        # Smart mission behavior.
        self.declare_parameter("require_connected_for_mission", True)
        self.declare_parameter("require_armed_for_mission", True)
        self.declare_parameter("telemetry_timeout_s", 2.0)
        self.declare_parameter("takeoff_altitude_m", 3.0)
        self.declare_parameter("airborne_altitude_m", 3.0)
        self.declare_parameter("takeoff_timeout_s", 20.0)
        self.declare_parameter("offboard_prime_time_s", 1.5)
        self.declare_parameter("track_center_timeout_s", 0.0)  # 0 = keep yaw tracking until stopped.
        self.declare_parameter("run_full_orbit_after_track_center", False)
        self.declare_parameter("mission_plan_topic", "/drone/mission/plan")
        self.declare_parameter("scan_yaw_deg", 180.0)
        self.declare_parameter("scan_yaw_rate_deg_s", 20.0)
        self.declare_parameter("scan_timeout_s", 12.0)

        # Optional external YAML mission plan. Empty = use the built-in default plan,
        # which reproduces the original hardcoded sequence (backward compatible).
        self.declare_parameter("mission_plan_file", "")

        # Later/full mission behavior parameters. Kept so the old orbit path can be re-enabled later.
        self.declare_parameter("target_timeout_s", 2.0)
        self.declare_parameter("desired_approach_distance_m", 2.0)
        self.declare_parameter("approach_distance_tolerance_m", 0.25)
        self.declare_parameter("approach_timeout_s", 20.0)
        self.declare_parameter("orbit_radius_m", 2.0)
        self.declare_parameter("orbit_speed_m_s", 0.4)
        self.declare_parameter("orbit_revolutions", 1.0)
        self.declare_parameter("orbit_timeout_s", 45.0)
        self.declare_parameter("rtl_wait_s", 10.0)
        self.declare_parameter("land_wait_s", 10.0)
        self.declare_parameter("use_mavsdk_do_orbit", True)
        self.declare_parameter("require_distance_for_orbit", True)
        self.declare_parameter("require_target_centered_for_orbit", True)
        self.declare_parameter("center_error_threshold", 0.15)
        self.declare_parameter("event_log_enabled", True)
        self.declare_parameter("event_log_directory", "~/drone_mission_logs")

        self.mission_enabled = bool(self.get_parameter("mission_enabled").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.mission_request_topic = str(self.get_parameter("mission_request_topic").value)
        self.mission_command_topic = str(self.get_parameter("mission_command_topic").value)
        self.mission_state_topic = str(self.get_parameter("mission_state_topic").value)
        self.mavsdk_action_topic = str(self.get_parameter("mavsdk_action_topic").value)
        self.autonomy_request_topic = str(self.get_parameter("autonomy_request_topic").value)
        self.offboard_request_topic = str(self.get_parameter("offboard_request_topic").value)
        self.target_error_topic = str(self.get_parameter("target_error_topic").value)
        self.telemetry_topic = str(self.get_parameter("telemetry_topic").value)
        self.publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))

        self.require_connected_for_mission = bool(self.get_parameter("require_connected_for_mission").value)
        self.require_armed_for_mission = bool(self.get_parameter("require_armed_for_mission").value)
        self.telemetry_timeout_s = float(self.get_parameter("telemetry_timeout_s").value)
        self.takeoff_altitude_m = float(self.get_parameter("takeoff_altitude_m").value)
        self.airborne_altitude_m = float(self.get_parameter("airborne_altitude_m").value)
        self.takeoff_timeout_s = float(self.get_parameter("takeoff_timeout_s").value)
        self.offboard_prime_time_s = float(self.get_parameter("offboard_prime_time_s").value)
        self.track_center_timeout_s = float(self.get_parameter("track_center_timeout_s").value)
        self.run_full_orbit_after_track_center = bool(self.get_parameter("run_full_orbit_after_track_center").value)
        self.scan_yaw_deg = float(self.get_parameter("scan_yaw_deg").value)
        self.scan_yaw_rate_deg_s = float(self.get_parameter("scan_yaw_rate_deg_s").value)
        self.scan_timeout_s = float(self.get_parameter("scan_timeout_s").value)
        self.mission_plan_file = str(self.get_parameter("mission_plan_file").value).strip()

        self.target_timeout_s = float(self.get_parameter("target_timeout_s").value)
        self.desired_approach_distance_m = float(self.get_parameter("desired_approach_distance_m").value)
        self.approach_distance_tolerance_m = float(self.get_parameter("approach_distance_tolerance_m").value)
        self.approach_timeout_s = float(self.get_parameter("approach_timeout_s").value)
        self.orbit_radius_m = float(self.get_parameter("orbit_radius_m").value)
        self.orbit_speed_m_s = float(self.get_parameter("orbit_speed_m_s").value)
        self.orbit_revolutions = float(self.get_parameter("orbit_revolutions").value)
        self.orbit_timeout_s = float(self.get_parameter("orbit_timeout_s").value)
        self.rtl_wait_s = float(self.get_parameter("rtl_wait_s").value)
        self.land_wait_s = float(self.get_parameter("land_wait_s").value)
        self.use_mavsdk_do_orbit = bool(self.get_parameter("use_mavsdk_do_orbit").value)
        self.require_distance_for_orbit = bool(self.get_parameter("require_distance_for_orbit").value)
        self.require_target_centered_for_orbit = bool(self.get_parameter("require_target_centered_for_orbit").value)
        self.center_error_threshold = float(self.get_parameter("center_error_threshold").value)
        self.event_log_enabled = bool(self.get_parameter("event_log_enabled").value)
        self.event_log_directory = str(self.get_parameter("event_log_directory").value)

        self.state = MissionState.IDLE if self.mission_enabled else MissionState.DISABLED
        self.state_enter_time = time.monotonic()
        self.mission_active = bool(self.auto_start and self.mission_enabled)
        self.action_command_id = 0
        self.actions_sent: set[str] = set()
        self._last_autonomy_request: Optional[bool] = None
        self._last_offboard_request: Optional[bool] = None
        self._last_autonomy_request_publish_time = 0.0
        self._last_offboard_request_publish_time = 0.0

        self.last_target: Optional[TargetError] = None
        self.last_target_time = 0.0
        self.last_telemetry: Optional[DroneTelemetry] = None
        self.last_telemetry_time = 0.0
        self._event_log_file: Optional[TextIO] = None
        self._event_log_path: Optional[Path] = None
        self._event_log_error_reported = False
        self._last_logged_mission_command_key: Optional[tuple[str, str, bool]] = None

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.request_sub = self.create_subscription(Bool, self.mission_request_topic, self.mission_request_callback, qos)
        self.target_sub = self.create_subscription(TargetError, self.target_error_topic, self.target_callback, qos)
        self.telemetry_sub = self.create_subscription(DroneTelemetry, self.telemetry_topic, self.telemetry_callback, qos)
        self.command_pub = self.create_publisher(MissionCommand, self.mission_command_topic, qos)
        self.state_pub = self.create_publisher(String, self.mission_state_topic, qos)
        self.action_pub = self.create_publisher(MavsdkActionCommand, self.mavsdk_action_topic, qos)
        self.autonomy_request_pub = self.create_publisher(Bool, self.autonomy_request_topic, qos)
        self.offboard_request_pub = self.create_publisher(Bool, self.offboard_request_topic, qos)

        self.mission_plan_topic = str(self.get_parameter("mission_plan_topic").value)
        self.plan_sub = self.create_subscription(String, self.mission_plan_topic, self._on_plan_received, qos)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.loop)
        self.diagnostics = NodeDiagnostics(self, heartbeat_period=5.0, stale_seconds=2.0)
        self.diagnostics.add_input(self.mission_request_topic, "mission_request", stale_seconds=60.0)
        self.diagnostics.add_input(self.mission_plan_topic, "mission_plan_receiver", stale_seconds=3600.0)
        self.diagnostics.add_input(self.target_error_topic, "target_error", stale_seconds=self.target_timeout_s)
        self.diagnostics.add_input(self.telemetry_topic, "telemetry", stale_seconds=self.telemetry_timeout_s)
        self.diagnostics.add_output(self.mission_command_topic, "mission_command")
        self.diagnostics.add_output(self.mission_state_topic, "mission_state")
        self.diagnostics.add_output(self.mavsdk_action_topic, "mavsdk_action_command")
        self.diagnostics.add_output(self.autonomy_request_topic, "autonomy_request")
        self.diagnostics.add_output(self.offboard_request_topic, "mavsdk_offboard_request")

        self._open_event_log()

        # Load the mission plan (external YAML, or the built-in default) and set up
        # the step cursor. The executor walks self.plan.steps instead of a hardcoded
        # state chain. Each verb is dispatched to a _step_* handler.
        self.plan: MissionPlan = self._load_plan()
        self.step_index = 0
        self.step_enter_time = time.monotonic()
        self._step_handlers = {
            "takeoff": self._step_takeoff,
            "prime_offboard": self._step_prime_offboard,
            "scan": self._step_scan,
            "track_center": self._step_track_center,
            "approach": self._step_approach,
            "orbit": self._step_orbit,
            "rtl": self._step_rtl,
            "land": self._step_land,
            "hold": self._step_hold,
            "goto_relative": self._step_goto_relative,
            "goto_absolute": self._step_goto_absolute,
            "complete": self._step_complete,
        }

        self.get_logger().warning(
            f"Mission executor started | enabled={self.mission_enabled}, active={self.mission_active}, "
            f"request_topic={self.mission_request_topic}, state_topic={self.mission_state_topic}, "
            f"command_topic={self.mission_command_topic}, mavsdk_action_topic={self.mavsdk_action_topic}, "
            f"autonomy_request_topic={self.autonomy_request_topic}, offboard_request_topic={self.offboard_request_topic}, "
            f"takeoff_altitude={self.takeoff_altitude_m:.1f}m, airborne_altitude={self.airborne_altitude_m:.1f}m"
        )
        self.log_event(
            "node_started",
            mission_enabled=self.mission_enabled,
            mission_active=self.mission_active,
            takeoff_altitude_m=self.takeoff_altitude_m,
            airborne_altitude_m=self.airborne_altitude_m,
            event_log_path=str(self._event_log_path) if self._event_log_path else "",
        )
        if not self.mission_enabled:
            self.get_logger().warning("Mission executor is disabled by parameter. Start Mission will stay DISABLED until enabled in config.")

    def _open_event_log(self) -> None:
        if not self.event_log_enabled:
            return
        try:
            log_dir = Path(self.event_log_directory).expanduser()
            log_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._event_log_path = log_dir / f"mission_events_{stamp}.jsonl"
            self._event_log_file = self._event_log_path.open("a", encoding="utf-8")
            self.get_logger().warning(f"Mission event log: {self._event_log_path}")
        except OSError as exc:
            self._event_log_file = None
            self._event_log_path = None
            self._event_log_error_reported = True
            self.get_logger().warning(f"Mission event logging disabled; could not open log file: {exc}")

    def _load_plan(self) -> MissionPlan:
        """Resolve the active mission plan.

        Uses the external YAML file when mission_plan_file is set; otherwise the
        built-in default plan (which reproduces the original hardcoded sequence).
        A load failure is logged and falls back to the default so the node never
        crashes on a bad plan file.
        """
        if self.mission_plan_file:
            try:
                plan = load_mission_plan(self.mission_plan_file)
                self.get_logger().warning(
                    f"Loaded mission plan '{plan.name}' from {self.mission_plan_file} "
                    f"| {len(plan.steps)} steps: {[s.type for s in plan.steps]}"
                )
                self.log_event(
                    "mission_plan_loaded",
                    source=self.mission_plan_file,
                    plan=plan.name,
                    steps=[s.type for s in plan.steps],
                )
            except MissionPlanError as exc:
                self.get_logger().error(
                    f"Failed to load mission plan '{self.mission_plan_file}': {exc}. "
                    "Falling back to built-in default plan."
                )
                self.log_event("mission_plan_load_failed", source=self.mission_plan_file, error=str(exc))
                plan = build_default_plan(self.run_full_orbit_after_track_center)
        else:
            plan = build_default_plan(self.run_full_orbit_after_track_center)
            self.get_logger().warning(
                f"Using built-in default mission plan | {len(plan.steps)} steps: "
                f"{[s.type for s in plan.steps]} (run_full_orbit={self.run_full_orbit_after_track_center})"
            )

        for warning in lint_plan(plan):
            self.get_logger().warning(f"Mission plan lint: {warning}")
        return plan

    def _telemetry_event_summary(self) -> Optional[dict[str, object]]:
        if self.last_telemetry is None:
            return None
        return {
            "age_s": round(time.monotonic() - self.last_telemetry_time, 3),
            "connected": bool(self.last_telemetry.connected),
            "armed": bool(self.last_telemetry.armed),
            "flight_mode": self.last_telemetry.flight_mode,
            "landed_state": self.last_telemetry.landed_state,
            "relative_altitude_m": round(float(self.last_telemetry.relative_altitude), 3),
            "battery_percent": round(float(self.last_telemetry.battery_remaining_percent), 1),
            "local_position_valid": bool(getattr(self.last_telemetry, "local_position_valid", False)),
            "local_ned": [
                round(float(getattr(self.last_telemetry, "local_position_north", 0.0)), 3),
                round(float(getattr(self.last_telemetry, "local_position_east", 0.0)), 3),
                round(float(getattr(self.last_telemetry, "local_position_down", 0.0)), 3),
            ],
        }

    def _target_event_summary(self) -> Optional[dict[str, object]]:
        if self.last_target is None:
            return None
        return {
            "age_s": round(time.monotonic() - self.last_target_time, 3),
            "visible": bool(self.last_target.target_visible),
            "tracking_state": self.last_target.tracking_state,
            "class": self.last_target.target_class,
            "confidence": round(float(self.last_target.target_confidence), 3),
            "error_x": round(float(self.last_target.error_x), 3),
            "error_y": round(float(self.last_target.error_y), 3),
            "distance_valid": bool(getattr(self.last_target, "distance_valid", False)),
            "distance_m": round(float(getattr(self.last_target, "distance_m", 0.0)), 3),
        }

    def log_event(self, event: str, **fields: object) -> None:
        if self._event_log_file is None:
            return
        record: dict[str, object] = {
            "ts_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "event": event,
            "state": self.state.value,
            "mission_active": bool(self.mission_active),
        }
        telemetry = self._telemetry_event_summary()
        target = self._target_event_summary()
        if telemetry is not None:
            record["telemetry"] = telemetry
        if target is not None:
            record["target"] = target
        record.update(fields)
        try:
            self._event_log_file.write(json.dumps(record, separators=(",", ":")) + "\n")
            self._event_log_file.flush()
        except OSError as exc:
            self._event_log_file = None
            if not self._event_log_error_reported:
                self._event_log_error_reported = True
                self.get_logger().warning(f"Mission event logging disabled after write failure: {exc}")

    def destroy_node(self) -> bool:
        self.log_event("node_shutdown")
        if self._event_log_file is not None:
            try:
                self._event_log_file.close()
            except OSError:
                pass
            self._event_log_file = None
        return super().destroy_node()

    def mission_request_callback(self, msg: Bool) -> None:
        requested = bool(msg.data)
        self.log_event("mission_request", requested=requested)
        if requested:
            if not self.mission_enabled:
                self.get_logger().warning("Mission start requested but mission_enabled is false; staying DISABLED.")
                self.log_event("mission_request_rejected", reason="mission_enabled_false")
                self.state = MissionState.DISABLED
                self.mission_active = False
                self.publish_autonomy_request(False)
                self.publish_offboard_request(False)
                return
            self.get_logger().warning(f"*** SMART MISSION REQUESTED | plan='{self.plan.name}' ***")
            self.mission_active = True
            self.actions_sent.clear()
            self.publish_autonomy_request(True)
            self.publish_offboard_request(False)
            self._start_plan()
        else:
            self.get_logger().warning("Mission stop requested; publishing HOLD/IDLE and dropping autonomy/offboard requests.")
            self.mission_active = False
            self.actions_sent.clear()
            self.publish_offboard_request(False)
            self.publish_autonomy_request(False)
            self.transition(MissionState.IDLE if self.mission_enabled else MissionState.DISABLED)
        self.diagnostics.mark_received(self.mission_request_topic, summary=f"request={requested}, active={self.mission_active}")

    def _on_plan_received(self, msg: String) -> None:
        """Accept a YAML mission plan from the dashboard (advisory, validated before use)."""
        if self.mission_active:
            self.get_logger().warning(
                f"Ignoring /drone/mission/plan: a mission is active (state={self.state.name})"
            )
            self.diagnostics.mark_received(
                self.mission_plan_topic, summary=f"rejected_during_mission (state={self.state.name})"
            )
            return

        import yaml
        from drone_control.mission_plan import parse_mission_plan
        try:
            data = yaml.safe_load(msg.data)
            plan = parse_mission_plan(data)
        except (MissionPlanError, yaml.YAMLError, ImportError) as exc:
            self.get_logger().error(f"Invalid mission plan from dashboard: {exc}")
            self.log_event("mission_plan_rejected", source="dashboard", error=str(exc))
            self.diagnostics.mark_received(self.mission_plan_topic, summary=f"rejected_parse_error ({exc})")
            return

        warnings = lint_plan(plan)
        for w in warnings:
            self.get_logger().warning(f"Mission plan lint: {w}")

        self.plan = plan
        self.step_index = 0
        self.step_enter_time = time.monotonic()
        self.get_logger().info(f"Accepted mission plan '{plan.name}' from dashboard | {len(plan.steps)} steps")
        self.log_event("mission_plan_received", source="dashboard", plan=plan.name, steps=[s.type for s in plan.steps])
        self.publish_state(f"plan received: {plan.name}")
        self.diagnostics.mark_received(
            self.mission_plan_topic, summary=f"accepted: {plan.name} ({len(plan.steps)} steps)"
        )

    def target_callback(self, msg: TargetError) -> None:
        self.last_target = msg
        self.last_target_time = time.monotonic()
        self.diagnostics.mark_received(
            self.target_error_topic,
            summary=f"visible={msg.target_visible}, state={msg.tracking_state}, dist={getattr(msg, 'distance_m', 0.0):.2f}",
        )

    def telemetry_callback(self, msg: DroneTelemetry) -> None:
        self.last_telemetry = msg
        self.last_telemetry_time = time.monotonic()
        self.diagnostics.mark_received(
            self.telemetry_topic,
            summary=(
                f"connected={msg.connected}, armed={msg.armed}, mode={msg.flight_mode}, "
                f"landed={msg.landed_state}, alt={msg.relative_altitude:.2f}"
            ),
        )

    def publish_autonomy_request(self, enabled: bool) -> None:
        enabled = bool(enabled)
        now = time.monotonic()
        changed = self._last_autonomy_request != enabled
        if not changed and now - self._last_autonomy_request_publish_time < 1.0:
            return
        self._last_autonomy_request = enabled
        self._last_autonomy_request_publish_time = now
        msg = Bool()
        msg.data = enabled
        self.autonomy_request_pub.publish(msg)
        self.diagnostics.mark_published(self.autonomy_request_topic, summary=f"requested={enabled}")
        if changed:
            self.get_logger().warning(f"Mission executor autonomy request: {enabled}")
            self.log_event("autonomy_request_published", requested=enabled)

    def publish_offboard_request(self, enabled: bool) -> None:
        enabled = bool(enabled)
        now = time.monotonic()
        changed = self._last_offboard_request != enabled
        if not changed and now - self._last_offboard_request_publish_time < 1.0:
            return
        self._last_offboard_request = enabled
        self._last_offboard_request_publish_time = now
        msg = Bool()
        msg.data = enabled
        self.offboard_request_pub.publish(msg)
        self.diagnostics.mark_published(self.offboard_request_topic, summary=f"requested={enabled}")
        if changed:
            self.get_logger().warning(f"Mission executor MAVSDK Offboard request: {enabled}")
            self.log_event("offboard_request_published", requested=enabled)

    def transition(self, new_state: MissionState) -> None:
        if new_state == self.state:
            return
        self.get_logger().warning(f"Mission state: {self.state.value} -> {new_state.value}")
        self.log_event("state_transition", from_state=self.state.value, to_state=new_state.value)
        self.state = new_state
        self.state_enter_time = time.monotonic()
        if hasattr(self, "_goto_anchor"):
            del self._goto_anchor

    def telemetry_is_fresh(self) -> bool:
        if self.last_telemetry is None:
            return False
        return time.monotonic() - self.last_telemetry_time <= self.telemetry_timeout_s

    def preflight_ok(self) -> tuple[bool, str]:
        if self.last_telemetry is None:
            return False, "NO_TELEMETRY"
        age = time.monotonic() - self.last_telemetry_time
        if age > self.telemetry_timeout_s:
            return False, f"TELEMETRY_STALE ({age:.2f}s)"
        if self.require_connected_for_mission and not bool(self.last_telemetry.connected):
            return False, "PX4_NOT_CONNECTED"
        if self.require_armed_for_mission and not bool(self.last_telemetry.armed):
            return False, "PX4_NOT_ARMED"
        return True, "PREFLIGHT_OK"

    def is_airborne(self) -> bool:
        if self.last_telemetry is None:
            return False
        t = self.last_telemetry
        alt_baro = float(t.relative_altitude)
        alt_ned = -float(getattr(t, "local_position_down", 0.0)) if bool(
            getattr(t, "local_position_valid", False)) else alt_baro
        altitude = max(alt_baro, alt_ned)
        landed_state = str(t.landed_state).upper()
        if "ON_GROUND" in landed_state or "LANDED" in landed_state:
            return altitude >= self.airborne_altitude_m
        return altitude >= self.airborne_altitude_m

    def target_is_fresh_locked(self) -> bool:
        if self.last_target is None:
            return False
        if time.monotonic() - self.last_target_time > self.target_timeout_s:
            return False
        return bool(self.last_target.target_visible and self.last_target.tracking_state == "LOCKED")

    def local_position_ready(self) -> bool:
        if self.last_telemetry is None:
            return False
        return bool(getattr(self.last_telemetry, "local_position_valid", False)) and all(
            math.isfinite(float(getattr(self.last_telemetry, field, float("nan"))))
            for field in ("local_position_north", "local_position_east", "local_position_down")
        )

    def _check_airborne_local_or_hold(self, label: str) -> bool:
        if self.last_telemetry is None:
            reason = "NO_TELEMETRY"
        else:
            age = time.monotonic() - self.last_telemetry_time
            if age > self.telemetry_timeout_s:
                reason = f"TELEMETRY_STALE ({age:.2f}s)"
            elif self.require_connected_for_mission and not bool(self.last_telemetry.connected):
                reason = "PX4_NOT_CONNECTED"
            elif not self.is_airborne():
                reason = "NOT_AIRBORNE"
            elif not self.local_position_ready():
                reason = "LOCAL_POSITION_NOT_VALID"
            else:
                return True

        self.publish_mission_command("HOLD", True, f"{label} blocked: {reason}")
        self.publish_state(f"{label} blocked: {reason}")
        return False

    def target_distance_ready(self) -> bool:
        if not self.target_is_fresh_locked() or self.last_target is None:
            return False
        if not bool(getattr(self.last_target, "distance_valid", False)):
            return False
        return float(getattr(self.last_target, "distance_m", 0.0)) > 0.0

    def target_centered(self) -> bool:
        if not self.target_is_fresh_locked() or self.last_target is None:
            return False
        return abs(float(self.last_target.error_x)) <= self.center_error_threshold

    def approach_done(self, desired_distance_m: Optional[float] = None) -> bool:
        if not self.target_distance_ready() or self.last_target is None:
            return False
        desired = self.desired_approach_distance_m if desired_distance_m is None else float(desired_distance_m)
        return abs(float(self.last_target.distance_m) - desired) <= self.approach_distance_tolerance_m

    def send_action_once(
        self,
        key: str,
        action: str,
        note: str = "",
        *,
        takeoff_altitude_m: Optional[float] = None,
        radius_m: Optional[float] = None,
        velocity_m_s: Optional[float] = None,
        orbit_revolutions: Optional[float] = None,
    ) -> None:
        if key in self.actions_sent:
            return
        self.actions_sent.add(key)
        self.action_command_id += 1
        msg = MavsdkActionCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command_id = int(self.action_command_id)
        msg.action = action
        msg.execute = True
        msg.takeoff_altitude_m = float(self.takeoff_altitude_m if takeoff_altitude_m is None else takeoff_altitude_m)
        msg.radius_m = float(self.orbit_radius_m if radius_m is None else radius_m)
        msg.velocity_m_s = float(self.orbit_speed_m_s if velocity_m_s is None else velocity_m_s)
        msg.orbit_revolutions = float(self.orbit_revolutions if orbit_revolutions is None else orbit_revolutions)
        msg.yaw_behavior = "FRONT_TO_CIRCLE_CENTER"
        msg.latitude_deg = math.nan
        msg.longitude_deg = math.nan
        msg.absolute_altitude_m = math.nan
        msg.note = note

        if action == "DO_ORBIT":
            center = self.estimate_target_global_center()
            if center is not None:
                msg.latitude_deg, msg.longitude_deg, msg.absolute_altitude_m = center
                msg.note = note + " | center=estimated_target_global"
            else:
                msg.note = note + " | center=current_position_nan"

        self.action_pub.publish(msg)
        self.diagnostics.mark_published(self.mavsdk_action_topic, summary=f"id={msg.command_id}, action={action}, note={msg.note}")
        self.get_logger().warning(f"Mission requested MAVSDK action: id={msg.command_id}, action={action}, note={msg.note}")
        self.log_event(
            "mavsdk_action_requested",
            command_id=int(msg.command_id),
            action=action,
            note=msg.note,
            takeoff_altitude_m=round(float(msg.takeoff_altitude_m), 3),
        )

    def estimate_target_global_center(self) -> Optional[tuple[float, float, float]]:
        if self.last_target is None or self.last_telemetry is None:
            return None
        if not self.target_distance_ready():
            return None
        lat = float(self.last_telemetry.latitude)
        lon = float(self.last_telemetry.longitude)
        alt = float(self.last_telemetry.absolute_altitude)
        if abs(lat) < 1e-9 and abs(lon) < 1e-9:
            return None

        distance_m = float(self.last_target.distance_m)
        bearing_x = float(getattr(self.last_target, "bearing_x_rad", 0.0))
        yaw = float(self.last_telemetry.yaw)
        global_bearing = yaw + bearing_x
        north_m = distance_m * math.cos(global_bearing)
        east_m = distance_m * math.sin(global_bearing)

        earth_radius_m = 6378137.0
        lat_rad = math.radians(lat)
        out_lat = lat + math.degrees(north_m / earth_radius_m)
        out_lon = lon + math.degrees(east_m / (earth_radius_m * max(math.cos(lat_rad), 1e-6)))
        return out_lat, out_lon, alt

    def publish_state(self, detail: str) -> None:
        msg = String()
        msg.data = f"{self.state.value}: {detail}"
        self.state_pub.publish(msg)
        self.diagnostics.mark_published(self.mission_state_topic, summary=msg.data)

    def publish_mission_command(
        self, mode: str, active: bool, status: str, *, yaw_rate: float = 0.0,
        desired_distance_m: Optional[float] = None,
        target_north: float = 0.0, target_east: float = 0.0, target_down: float = 0.0,
        target_position_valid: bool = False,
    ) -> None:
        msg = MissionCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = mode
        msg.active = bool(active)
        msg.velocity_forward = 0.0
        msg.velocity_right = 0.0
        msg.velocity_down = 0.0
        msg.yaw_rate = float(yaw_rate)
        msg.desired_distance_m = float(
            self.desired_approach_distance_m if desired_distance_m is None else desired_distance_m
        )
        msg.orbit_radius_m = float(self.orbit_radius_m)
        msg.orbit_speed_m_s = float(self.orbit_speed_m_s)
        msg.target_position_valid = bool(target_position_valid)
        msg.target_north = float(target_north)
        msg.target_east = float(target_east)
        msg.target_down = float(target_down)
        msg.step_index = self.step_index_for_state(self.state)
        msg.step_name = STEP_NAMES.get(self.state, self.state.value.lower())
        msg.status = status

        if mode == "ORBIT_TARGET" and not self.use_mavsdk_do_orbit:
            msg.velocity_right = float(self.orbit_speed_m_s)

        self.command_pub.publish(msg)
        self.diagnostics.mark_published(self.mission_command_topic, summary=f"mode={mode}, active={active}, status={status}")
        command_key = (self.state.value, mode, bool(active))
        if command_key != self._last_logged_mission_command_key:
            self._last_logged_mission_command_key = command_key
            self.log_event(
                "mission_command_published",
                mode=mode,
                active=bool(active),
                step_index=int(msg.step_index),
                step_name=msg.step_name,
                status=status,
            )

    @staticmethod
    def step_index_for_state(state: MissionState) -> int:
        order = [
            MissionState.PREFLIGHT,
            MissionState.TAKEOFF,
            MissionState.PRIME_OFFBOARD,
            MissionState.SCAN,
            MissionState.TRACK_CENTER,
            MissionState.APPROACH_TARGET,
            MissionState.DO_ORBIT,
            MissionState.RETURN_TO_LAUNCH,
            MissionState.LAND,
        ]
        try:
            return order.index(state)
        except ValueError:
            return 0

    def loop(self) -> None:
        if not self.mission_enabled:
            self.state = MissionState.DISABLED
            self.publish_offboard_request(False)
            self.publish_autonomy_request(False)
            self.publish_mission_command("IDLE", False, "mission disabled")
            self.publish_state("mission_enabled=false")
            return

        if not self.mission_active:
            self.publish_mission_command("IDLE", False, "waiting for /drone/mission/request true")
            self.publish_state("idle")
            return

        self.publish_autonomy_request(True)

        # Walk the mission plan: dispatch the current step's verb to its handler.
        if self.step_index >= len(self.plan.steps):
            self._enter_complete()
            return

        step = self.plan.steps[self.step_index]
        self.transition(MissionState[step.state_name])  # keep state synced for logs/dashboard

        handler = self._step_handlers.get(step.type)
        if handler is None:
            # Plans are validated at load time, so this should not normally happen.
            self.publish_mission_command("HOLD", True, f"unknown step type {step.type}")
            self.publish_state(f"unknown step type {step.type}")
            return

        if handler(step):
            self.advance()

    # ------------------------------------------------------------------
    # Plan cursor management
    # ------------------------------------------------------------------
    def _start_plan(self) -> None:
        self.step_index = 0
        self.step_enter_time = time.monotonic()
        if self.plan.steps:
            self.transition(MissionState[self.plan.steps[0].state_name])
        self.log_event("plan_started", plan=self.plan.name, steps=[s.type for s in self.plan.steps])

    def advance(self) -> None:
        prev = self.plan.steps[self.step_index].type if self.step_index < len(self.plan.steps) else "?"
        self.step_index += 1
        self.step_enter_time = time.monotonic()
        if hasattr(self, "_goto_anchor"):
            del self._goto_anchor
        if self.step_index < len(self.plan.steps):
            nxt = self.plan.steps[self.step_index]
            self.get_logger().warning(f"Mission step '{prev}' done -> step {self.step_index} '{nxt.type}'")
            self.log_event("step_advance", from_step=prev, to_index=self.step_index, to_type=nxt.type)
        else:
            self.log_event("plan_complete", last_step=prev)

    def step_age(self) -> float:
        return time.monotonic() - self.step_enter_time

    def _enter_complete(self) -> None:
        self.transition(MissionState.COMPLETE)
        self.publish_offboard_request(False)
        # De-assert autonomy too (mirrors the abort path). Otherwise the last
        # autonomy_request from the active loop stays True and autonomy_manager
        # can keep /drone/autonomy/enabled=true after the mission finishes while
        # the target is still locked. The drone should be cleanly idle on COMPLETE.
        self.publish_autonomy_request(False)
        self.publish_mission_command("IDLE", False, "mission complete")
        self.publish_state("complete")
        self.mission_active = False

    def _check_preflight_or_hold(self, label: str) -> bool:
        """Shared preflight gate for takeoff/prime/track steps (matches the original
        per-state checks). Publishes a blocked HOLD and returns False when not ready."""
        ok, reason = self.preflight_ok()
        if ok:
            return True
        self.publish_mission_command("HOLD", True, f"{label} blocked: {reason}")
        self.publish_state(f"blocked: {reason}")
        return False

    def _until_satisfied(self, step, default: str) -> bool:
        until = step.until or default
        if until == "airborne":
            return self.is_airborne()
        if until == "centered":
            return self.target_centered()
        if until == "locked":
            return self.target_is_fresh_locked()
        if until == "approach_done":
            return self.approach_done(step.get_float("distance_m", self.desired_approach_distance_m))
        return False  # "none" or unset -> never auto-advance on a condition

    def _orbit_default_timeout(self, radius_m: float, speed_m_s: float, revolutions: float) -> float:
        if speed_m_s > 0.0 and revolutions > 0.0 and radius_m > 0.0:
            return (2.0 * math.pi * radius_m * revolutions) / speed_m_s + 5.0
        return self.orbit_timeout_s

    # ------------------------------------------------------------------
    # Step handlers: each publishes its intent and returns True when the step is
    # complete (the loop then advances). These are the original per-state bodies,
    # parameterized by the plan step. Safety gates are unchanged.
    # ------------------------------------------------------------------
    def _step_takeoff(self, step) -> bool:
        if not self._check_preflight_or_hold("takeoff"):
            return False
        self.publish_offboard_request(False)
        # Takeoff if needed: skip when already airborne.
        if self.is_airborne():
            self.publish_mission_command("HOLD", True, "already airborne; skipping takeoff")
            self.publish_state("already airborne; skipping takeoff")
            return True
        altitude_m = step.get_float("altitude_m", self.takeoff_altitude_m)
        self.send_action_once(
            "takeoff", "TAKEOFF", "smart mission takeoff before Offboard", takeoff_altitude_m=altitude_m
        )
        current_altitude = 0.0 if self.last_telemetry is None else float(self.last_telemetry.relative_altitude)
        self.publish_mission_command(
            "HOLD", True, f"takeoff action requested; waiting for altitude >= {self.airborne_altitude_m:.1f}m"
        )
        age = self.step_age()
        self.publish_state(
            f"takeoff requested, altitude={current_altitude:.2f}/{self.airborne_altitude_m:.2f}m, "
            f"ready_for_offboard={self.is_airborne()}, age={age:.1f}/{self.takeoff_timeout_s:.1f}s"
        )
        if self.is_airborne():
            return True
        if age > self.takeoff_timeout_s:
            # Stay on this step instead of yawing on the ground. Catches disabled action gates.
            self.publish_state("takeoff timeout; still waiting for takeoff altitude")
        return False

    def _step_prime_offboard(self, step) -> bool:
        if not self._check_preflight_or_hold("prime"):
            return False
        self.publish_offboard_request(True)
        # Status must keep "priming PX4 Offboard" so control_node captures its prime anchor.
        self.publish_mission_command("HOLD", True, "priming PX4 Offboard with zero/hold setpoints")
        hold_s = step.get_float("hold_s", self.offboard_prime_time_s)
        age = self.step_age()
        self.publish_state(f"priming offboard, age={age:.1f}/{hold_s:.1f}s")
        return age >= hold_s

    def _step_scan(self, step) -> bool:
        """Hold position and yaw-sweep to search for the target.

        Safety: this only commands yaw (the control node holds the captured local
        NED anchor and rotates in place — no translation). It exits early as soon
        as the target locks (until: locked), after sweeping the requested
        yaw_deg, or on timeout. On timeout it returns True so the plan advances to
        its next step (e.g. a hold or land), never wedging on a missing target.
        """
        if not self._check_preflight_or_hold("scan"):
            return False
        self.publish_offboard_request(True)

        yaw_deg = step.get_float("yaw_deg", self.scan_yaw_deg)
        yaw_rate_deg_s = step.get_float("yaw_rate_deg_s", self.scan_yaw_rate_deg_s)
        # Convention: ccw (counter-clockwise viewed from above) = negative NED yaw
        # rate; cw = positive. The control node integrates this into its yaw target.
        sign = -1.0 if step.scan_direction == "ccw" else 1.0
        yaw_rate_rad_s = sign * math.radians(max(0.0, yaw_rate_deg_s))

        locked = self.target_is_fresh_locked()
        age = self.step_age()
        swept_deg = abs(yaw_rate_deg_s) * age

        # Stop yawing once we have locked (exit will advance) so we don't sweep
        # past a freshly found target; otherwise keep sweeping.
        commanded_rate = 0.0 if locked else yaw_rate_rad_s
        self.publish_mission_command(
            "SCAN", True,
            f"scan {step.scan_direction} {yaw_deg:.0f}deg @ {yaw_rate_deg_s:.0f}deg/s",
            yaw_rate=commanded_rate,
        )

        # Exit early when the target locks (until: locked) or any other satisfied
        # predicate, when the full sweep is done, or on timeout.
        if self._until_satisfied(step, default="locked"):
            self.publish_state(f"scan locked target, swept={swept_deg:.0f}/{yaw_deg:.0f}deg")
            return True
        if swept_deg >= yaw_deg:
            self.publish_state(f"scan complete, swept full {yaw_deg:.0f}deg, locked={locked}")
            return True
        timeout = step.timeout_s if step.timeout_s is not None else self.scan_timeout_s
        self.publish_state(
            f"scanning {step.scan_direction}, swept={swept_deg:.0f}/{yaw_deg:.0f}deg, "
            f"locked={locked}, age={age:.1f}/{timeout:.1f}s"
        )
        return timeout > 0.0 and age > timeout

    def _step_track_center(self, step) -> bool:
        if not self._check_preflight_or_hold("track"):
            return False
        self.publish_offboard_request(True)
        locked = self.target_is_fresh_locked()
        self.publish_mission_command("TRACK_CENTER", True, "yaw toward YOLO target; no forward motion")
        age = self.step_age()
        self.publish_state(f"track center yaw, locked={locked}, age={age:.1f}s")
        if self._until_satisfied(step, default="none"):
            return True
        timeout = step.timeout_s if step.timeout_s is not None else self.track_center_timeout_s
        return timeout > 0.0 and age > timeout

    def _step_approach(self, step) -> bool:
        if not self._check_airborne_local_or_hold("approach"):
            self.publish_offboard_request(False)
            return False
        self.publish_offboard_request(True)
        desired = step.get_float("distance_m", self.desired_approach_distance_m)
        self.publish_mission_command(
            "APPROACH_TARGET", True,
            f"approach to {desired:.2f}m using distance estimate",
            desired_distance_m=desired,
        )
        dist_text = "none"
        if self.last_target is not None and bool(getattr(self.last_target, "distance_valid", False)):
            dist_text = f"{self.last_target.distance_m:.2f}m"
        age = self.step_age()
        self.publish_state(f"approaching, distance={dist_text}, desired={desired:.2f}m, age={age:.1f}s")
        if self.approach_done(desired):
            return True
        timeout = step.timeout_s if step.timeout_s is not None else self.approach_timeout_s
        return age > timeout

    def _step_orbit(self, step) -> bool:
        if not self._check_airborne_local_or_hold("orbit"):
            self.publish_offboard_request(False)
            return False
        self.publish_offboard_request(True)
        if self.require_distance_for_orbit and not self.target_distance_ready():
            self.publish_mission_command("TRACK_CENTER", True, "waiting for valid distance before orbit")
            self.publish_state("orbit hold: distance not ready")
            return False
        if self.require_target_centered_for_orbit and not self.target_centered():
            self.publish_mission_command("TRACK_CENTER", True, "centering target before orbit")
            self.publish_state("orbit hold: target not centered")
            return False

        radius_m = step.get_float("radius_m", self.orbit_radius_m)
        speed_m_s = step.get_float("speed_m_s", self.orbit_speed_m_s)
        revolutions = step.get_float("revolutions", self.orbit_revolutions)

        if self.use_mavsdk_do_orbit:
            self.send_action_once(
                "do_orbit", "DO_ORBIT", "MAV_CMD_DO_ORBIT around estimated ball center",
                radius_m=radius_m, velocity_m_s=speed_m_s, orbit_revolutions=revolutions,
            )
            self.publish_mission_command("HOLD", True, "DO_ORBIT requested; PX4 owns orbit if accepted")
        else:
            self.publish_mission_command("ORBIT_TARGET", True, "visual-servo orbit fallback")

        if step.timeout_s is not None:
            timeout = step.timeout_s
        elif "revolutions" in step.params:
            timeout = self._orbit_default_timeout(radius_m, speed_m_s, revolutions)
        else:
            timeout = self.orbit_timeout_s
        age = self.step_age()
        self.publish_state(f"orbiting/requested, age={age:.1f}/{timeout:.1f}s")
        return age > timeout

    def _step_rtl(self, step) -> bool:
        self.publish_offboard_request(False)
        self.send_action_once("rtl", "RETURN_TO_LAUNCH", "return to launch")
        self.publish_mission_command("HOLD", True, "RTL requested")
        timeout = step.timeout_s if step.timeout_s is not None else self.rtl_wait_s
        age = self.step_age()
        self.publish_state(f"returning, age={age:.1f}/{timeout:.1f}s")
        return age > timeout

    def _step_land(self, step) -> bool:
        self.publish_offboard_request(False)
        self.send_action_once("land", "LAND", "land")
        self.publish_mission_command("HOLD", True, "land requested")
        timeout = step.timeout_s if step.timeout_s is not None else self.land_wait_s
        age = self.step_age()
        self.publish_state(f"landing, age={age:.1f}/{timeout:.1f}s")
        return age > timeout

    def _step_hold(self, step) -> bool:
        self.publish_offboard_request(True)
        status = str(step.params.get("status", "holding position"))
        self.publish_mission_command("HOLD", True, status)
        timeout = step.timeout_s if step.timeout_s is not None else 0.0
        age = self.step_age()
        self.publish_state(f"hold, age={age:.1f}/{timeout:.1f}s")
        return timeout > 0.0 and age >= timeout

    def _goto_target_ned(self, step, relative: bool) -> tuple[float, float, float]:
        """Compute target NED. For relative, offset from current position."""
        n = step.get_float("north_m", 0.0)
        e = step.get_float("east_m", 0.0)
        d = step.get_float("down_m", 0.0)
        if relative and self.last_telemetry is not None:
            if not hasattr(self, "_goto_anchor"):
                self._goto_anchor = (
                    float(getattr(self.last_telemetry, "local_position_north", 0.0)),
                    float(getattr(self.last_telemetry, "local_position_east", 0.0)),
                    float(getattr(self.last_telemetry, "local_position_down", 0.0)),
                )
            an, ae, ad = self._goto_anchor
            return an + n, ae + e, ad + d
        return n, e, d

    def _goto_distance_to_target(self, tn: float, te: float, td: float) -> float:
        if self.last_telemetry is None:
            return float("inf")
        cn = float(getattr(self.last_telemetry, "local_position_north", 0.0))
        ce = float(getattr(self.last_telemetry, "local_position_east", 0.0))
        cd = float(getattr(self.last_telemetry, "local_position_down", 0.0))
        return math.sqrt((tn - cn) ** 2 + (te - ce) ** 2 + (td - cd) ** 2)

    def _step_goto(self, step, relative: bool) -> bool:
        if not self._check_airborne_local_or_hold("goto"):
            self.publish_offboard_request(False)
            return False
        self.publish_offboard_request(True)
        tn, te, td = self._goto_target_ned(step, relative)
        acceptance = step.get_float("acceptance_m", 0.5)
        dist = self._goto_distance_to_target(tn, te, td)
        label = "goto_relative" if relative else "goto_absolute"
        self.publish_mission_command(
            "GOTO", True,
            f"{label} to N={tn:.2f} E={te:.2f} D={td:.2f}, dist={dist:.2f}m",
            target_north=tn, target_east=te, target_down=td,
            target_position_valid=True,
        )
        age = self.step_age()
        timeout = step.timeout_s if step.timeout_s is not None else 15.0
        self.publish_state(
            f"{label}, target=({tn:.1f},{te:.1f},{td:.1f}), "
            f"dist={dist:.2f}m, accept={acceptance:.1f}m, age={age:.1f}/{timeout:.1f}s"
        )
        if dist <= acceptance:
            return True
        if timeout > 0.0 and age > timeout:
            return True
        return False

    def _step_goto_relative(self, step) -> bool:
        return self._step_goto(step, relative=True)

    def _step_goto_absolute(self, step) -> bool:
        return self._step_goto(step, relative=False)

    def _step_complete(self, step) -> bool:
        self._enter_complete()
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MissionExecutorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger("mission_executor_node").fatal(f"Fatal: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
