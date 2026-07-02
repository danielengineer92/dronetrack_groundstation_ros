"""
PX4 MAVSDK bridge node for the drone vision system.

This node owns the single MAVSDK connection to the Pixhawk. It publishes
telemetry and, when explicitly enabled, consumes /drone/control/command and sends
safe position-hold/yaw Offboard setpoints to PX4.

Safety posture for this first command bridge:
- Does NOT arm the drone.
- Does NOT take off.
- Does NOT command forward/right/down motion by default.
- Requires /drone/mavsdk/offboard_enable true before starting Offboard.
- Sends local-NED PositionNedYaw hold setpoints when priming or tracking.
- Legacy VELOCITY commands are still understood, but the smart mission now uses POSITION.
"""

from __future__ import annotations

import asyncio
import json
import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from drone_interfaces.msg import ControlCommand, DroneTelemetry, MavsdkActionCommand
from drone_diagnostics.node_diagnostics import NodeDiagnostics


CMD_VELOCITY = "VELOCITY"
CMD_POSITION = "POSITION"
CMD_IDLE = "IDLE"
STATUS_SENT = "SENT"

BRIDGE_DISABLED = "MAVSDK_OFFBOARD_DISABLED"
BRIDGE_NO_COMMAND = "NO_CONTROL_COMMAND"
BRIDGE_COMMAND_STALE = "CONTROL_COMMAND_STALE"
BRIDGE_COMMAND_IDLE = "CONTROL_COMMAND_IDLE"
BRIDGE_COMMAND_NOT_APPROVED = "CONTROL_COMMAND_NOT_APPROVED"
BRIDGE_NO_LOCAL_POSITION = "NO_LOCAL_POSITION_NED"
BRIDGE_BAD_POSITION_COMMAND = "BAD_POSITION_COMMAND"
BRIDGE_NOT_CONNECTED = "PX4_NOT_CONNECTED"
BRIDGE_NOT_ARMED = "PX4_NOT_ARMED"
BRIDGE_LOW_BATTERY = "PX4_LOW_BATTERY"
BRIDGE_READY = "READY_TO_SEND"
BRIDGE_SENT = "SENT_TO_PX4"
BRIDGE_ZERO_SENT = "ZERO_SENT_TO_PX4"
BRIDGE_PRIMING = "PRIMING_OFFBOARD_POSITION_HOLD"
BRIDGE_OFFBOARD_STARTED = "OFFBOARD_STARTED"
BRIDGE_OFFBOARD_STOPPED = "OFFBOARD_STOPPED"
BRIDGE_OFFBOARD_START_FAILED = "OFFBOARD_START_FAILED"
BRIDGE_SEND_FAILED = "OFFBOARD_SEND_FAILED"
ACTION_DISABLED = "MAVSDK_ACTIONS_DISABLED"
ACTION_RECEIVED = "MAVSDK_ACTION_RECEIVED"
ACTION_SENT = "MAVSDK_ACTION_SENT"
ACTION_FAILED = "MAVSDK_ACTION_FAILED"
ACTION_IGNORED = "MAVSDK_ACTION_IGNORED"
ACTION_BLOCKED = "MAVSDK_ACTION_BLOCKED"

MAV_CMD_DO_ORBIT = 34
MAV_COMP_ID_AUTOPILOT1 = 1

# MAVLink ORBIT_YAW_BEHAVIOUR values. Keep these local so the telemetry bridge
# can send MAV_CMD_DO_ORBIT through MavlinkDirect without depending on a
# Python enum that may not exist in the installed MAVSDK-Python package.
ORBIT_YAW_BEHAVIOR_VALUES = {
    'HOLD_FRONT_TO_CIRCLE_CENTER': 0,
    'FRONT_TO_CIRCLE_CENTER': 0,
    'HOLD_INITIAL_HEADING': 1,
    'UNCONTROLLED': 2,
    'HOLD_FRONT_TANGENT_TO_CIRCLE': 3,
    'RC_CONTROLLED': 4,
}


class TelemetryNode(Node):
    """ROS 2 node that bridges PX4/MAVSDK telemetry and position-hold yaw commands."""

    def __init__(self) -> None:
        super().__init__('telemetry_node')

        # Connection and telemetry parameters
        self.declare_parameter('connection_url', 'serial:///dev/ttyACM0:57600')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('connection_timeout', 10.0)
        self.declare_parameter('telemetry_topic', '/drone/telemetry')

        # Command bridge parameters
        self.declare_parameter('control_command_topic', '/drone/control/command')
        self.declare_parameter('offboard_enable_topic', '/drone/mavsdk/offboard_enable')
        self.declare_parameter('command_status_topic', '/drone/mavsdk/command_status')
        self.declare_parameter('mavsdk_offboard_enabled', False)
        self.declare_parameter('command_rate', 20.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('require_armed_for_offboard', True)
        self.declare_parameter('min_battery_percent', 20.0)
        self.declare_parameter('max_yaw_rate_rad_s', 1.0)
        self.declare_parameter('allow_translation_commands', False)
        self.declare_parameter('stop_offboard_on_disable', True)
        self.declare_parameter('action_command_topic', '/drone/mavsdk/action_command')
        self.declare_parameter('allow_mavsdk_actions', False)
        self.declare_parameter('action_command_timeout', 3.0)
        self.declare_parameter('action_telemetry_timeout', 2.0)
        self.declare_parameter('action_airborne_altitude_m', 0.5)

        # Read parameters
        self._connection_url: str = str(self.get_parameter('connection_url').value)
        self._publish_rate: float = float(self.get_parameter('publish_rate').value)
        self._reconnect_interval: float = float(self.get_parameter('reconnect_interval').value)
        self._connection_timeout: float = float(self.get_parameter('connection_timeout').value)
        self._telemetry_topic: str = str(self.get_parameter('telemetry_topic').value)

        self._control_command_topic: str = str(self.get_parameter('control_command_topic').value)
        self._offboard_enable_topic: str = str(self.get_parameter('offboard_enable_topic').value)
        self._command_status_topic: str = str(self.get_parameter('command_status_topic').value)
        self._mavsdk_offboard_enabled: bool = bool(self.get_parameter('mavsdk_offboard_enabled').value)
        self._command_rate: float = float(self.get_parameter('command_rate').value)
        self._command_timeout: float = float(self.get_parameter('command_timeout').value)
        self._require_armed_for_offboard: bool = bool(self.get_parameter('require_armed_for_offboard').value)
        self._min_battery_percent: float = float(self.get_parameter('min_battery_percent').value)
        self._max_yaw_rate_rad_s: float = float(self.get_parameter('max_yaw_rate_rad_s').value)
        self._allow_translation_commands: bool = bool(self.get_parameter('allow_translation_commands').value)
        self._stop_offboard_on_disable: bool = bool(self.get_parameter('stop_offboard_on_disable').value)
        self._action_command_topic: str = str(self.get_parameter('action_command_topic').value)
        self._allow_mavsdk_actions: bool = bool(self.get_parameter('allow_mavsdk_actions').value)
        self._action_command_timeout: float = float(self.get_parameter('action_command_timeout').value)
        self._action_telemetry_timeout: float = float(self.get_parameter('action_telemetry_timeout').value)
        self._action_airborne_altitude_m: float = float(self.get_parameter('action_airborne_altitude_m').value)

        self._validate_parameters()

        # MAVSDK state
        self._system = None
        self._connected: bool = False
        self._connection_status: str = "DISCONNECTED"
        self._offboard_active: bool = False
        self._telemetry_publish_count: int = 0
        self._control_command_count: int = 0
        self._offboard_enable_count: int = 0
        self._px4_send_count: int = 0
        self._px4_zero_count: int = 0
        self._last_bridge_status: str = BRIDGE_DISABLED
        self._last_status_publish_time: float = 0.0
        self._last_log_times: dict[str, float] = {}
        self._action_command_count: int = 0
        self._handled_action_keys: set[tuple[int, str, int, int, str]] = set()
        self._handled_action_key_order: list[tuple[int, str, int, int, str]] = []
        self._action_in_progress: bool = False
        self._prime_hold_position: Optional[tuple[float, float, float, float]] = None
        self._last_position_time: float = 0.0
        self._last_local_position_time: float = 0.0
        self._last_armed_time: float = 0.0
        self._last_landed_state_time: float = 0.0

        # Telemetry data protected by lock
        self._data_lock = threading.Lock()
        self._telemetry_data: dict = {
            'battery_voltage': 0.0,
            'battery_remaining': 0.0,
            'latitude': 0.0,
            'longitude': 0.0,
            'absolute_altitude': 0.0,
            'relative_altitude': 0.0,
            'local_position_valid': False,
            'local_position_north': 0.0,
            'local_position_east': 0.0,
            'local_position_down': 0.0,
            'gps_num_satellites': 0,
            'gps_fix_type': 0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'velocity_north': 0.0,
            'velocity_east': 0.0,
            'velocity_down': 0.0,
            'armed': False,
            'flight_mode': 'UNKNOWN',
            'landed_state': 'UNKNOWN',
            'health_all_ok': False,
            'health_accelerometer_ok': False,
            'health_gyroscope_ok': False,
            'health_magnetometer_ok': False,
            'health_gps_ok': False,
        }

        # Latest control command protected by lock
        self._command_lock = threading.Lock()
        self._latest_command: Optional[ControlCommand] = None
        self._latest_command_time: float = 0.0

        # Latest one-shot MAVSDK action request protected by lock
        self._action_lock = threading.Lock()
        self._latest_action_command: Optional[MavsdkActionCommand] = None
        self._latest_action_command_time: float = 0.0

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._telemetry_pub = self.create_publisher(
            DroneTelemetry,
            self._telemetry_topic,
            reliable_qos,
        )
        self._command_status_pub = self.create_publisher(
            String,
            self._command_status_topic,
            reliable_qos,
        )

        self._command_sub = self.create_subscription(
            ControlCommand,
            self._control_command_topic,
            self._control_command_callback,
            reliable_qos,
        )
        self._offboard_enable_sub = self.create_subscription(
            Bool,
            self._offboard_enable_topic,
            self._offboard_enable_callback,
            reliable_qos,
        )
        self._action_command_sub = self.create_subscription(
            MavsdkActionCommand,
            self._action_command_topic,
            self._action_command_callback,
            reliable_qos,
        )

        publish_period = 1.0 / self._publish_rate
        self._publish_timer = self.create_timer(publish_period, self._publish_telemetry)

        self._diagnostics = NodeDiagnostics(self, heartbeat_period=5.0, stale_seconds=2.0)
        self._diagnostics.add_output(self._telemetry_topic, "telemetry")
        self._diagnostics.add_input(self._control_command_topic, "control_command", stale_seconds=self._command_timeout)
        self._diagnostics.add_input(self._offboard_enable_topic, "offboard_enable", stale_seconds=60.0)
        self._diagnostics.add_input(self._action_command_topic, "mavsdk_action_command", stale_seconds=60.0)
        self._diagnostics.add_output(self._command_status_topic, "mavsdk_command_status")

        self._status_timer = self.create_timer(5.0, self._report_status)

        self._async_thread: Optional[threading.Thread] = None
        self._running: bool = True
        self._start_mavsdk_connection()

        self.get_logger().warning(
            'PX4 MAVSDK bridge initialized | '
            f'telemetry_topic={self._telemetry_topic}, '
            f'control_command_topic={self._control_command_topic}, '
            f'offboard_enable_topic={self._offboard_enable_topic}, '
            f'command_status_topic={self._command_status_topic}, '
            f'action_command_topic={self._action_command_topic}, '
            f'url={self._connection_url}, telemetry_rate={self._publish_rate:.1f}Hz, '
            f'command_rate={self._command_rate:.1f}Hz, '
            f'mavsdk_offboard_enabled={self._mavsdk_offboard_enabled}, '
            f'allow_mavsdk_actions={self._allow_mavsdk_actions}, '
            f'mode=POSITION_HOLD_YAW, max_yaw={self._max_yaw_rate_rad_s:.2f}rad/s, '
            'actions=blocked_by_default_until_allow_mavsdk_actions_true'
        )
        if not self._mavsdk_offboard_enabled:
            self.get_logger().warning(
                'MAVSDK OFFBOARD DISABLED - this node will publish telemetry but will not send movement '
                'setpoints until /drone/mavsdk/offboard_enable is true.'
            )

    def _validate_parameters(self) -> None:
        if self._publish_rate <= 0.0:
            raise ValueError(f'publish_rate must be > 0, got {self._publish_rate}')
        if self._command_rate <= 0.0:
            raise ValueError(f'command_rate must be > 0, got {self._command_rate}')
        if self._command_timeout <= 0.0:
            raise ValueError(f'command_timeout must be > 0, got {self._command_timeout}')
        if self._max_yaw_rate_rad_s < 0.0:
            raise ValueError(f'max_yaw_rate_rad_s must be >= 0, got {self._max_yaw_rate_rad_s}')
        if self._min_battery_percent < 0.0:
            raise ValueError(f'min_battery_percent must be >= 0, got {self._min_battery_percent}')
        if self._action_command_timeout <= 0.0:
            raise ValueError(f'action_command_timeout must be > 0, got {self._action_command_timeout}')
        if self._action_telemetry_timeout <= 0.0:
            raise ValueError(f'action_telemetry_timeout must be > 0, got {self._action_telemetry_timeout}')
        if self._action_airborne_altitude_m < 0.0:
            raise ValueError(f'action_airborne_altitude_m must be >= 0, got {self._action_airborne_altitude_m}')

    def _control_command_callback(self, msg: ControlCommand) -> None:
        with self._command_lock:
            self._latest_command = msg
            self._latest_command_time = time.monotonic()
            self._control_command_count += 1

        self._diagnostics.mark_received(
            self._control_command_topic,
            summary=(
                f'messages={self._control_command_count}, type={msg.command_type}, '
                f'executed={msg.executed}, status={msg.execution_status}, yaw_rate={msg.yaw_rate:+.3f}, pos_valid={getattr(msg, 'position_valid', False)}'
            ),
        )

    def _offboard_enable_callback(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        old_enabled = self._mavsdk_offboard_enabled
        self._mavsdk_offboard_enabled = enabled
        self._offboard_enable_count += 1

        if enabled and not old_enabled:
            self._prime_hold_position = None
            self.get_logger().warning('*** MAVSDK OFFBOARD EXECUTOR ENABLED by /drone/mavsdk/offboard_enable ***')
        elif not enabled and old_enabled:
            self._prime_hold_position = None
            self.get_logger().warning('MAVSDK offboard executor disabled; sending/stopping zero/hold setpoints.')

        self._diagnostics.mark_received(
            self._offboard_enable_topic,
            summary=f'messages={self._offboard_enable_count}, enabled={self._mavsdk_offboard_enabled}',
        )
        self._publish_command_status(
            f'{BRIDGE_READY if enabled else BRIDGE_DISABLED}: enabled={self._mavsdk_offboard_enabled}',
            force=True,
        )

    def _action_command_callback(self, msg: MavsdkActionCommand) -> None:
        with self._action_lock:
            self._latest_action_command = msg
            self._latest_action_command_time = time.monotonic()
            self._action_command_count += 1

        self._diagnostics.mark_received(
            self._action_command_topic,
            summary=(
                f'messages={self._action_command_count}, id={msg.command_id}, '
                f'action={msg.action}, execute={msg.execute}, note={msg.note}'
            ),
        )
        self._publish_command_status(f'{ACTION_RECEIVED}: id={msg.command_id}, action={msg.action}', force=True)

    def _start_mavsdk_connection(self) -> None:
        self._async_thread = threading.Thread(
            target=self._run_async_loop,
            daemon=True,
            name='mavsdk_bridge_thread',
        )
        self._async_thread.start()

    def _run_async_loop(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        try:
            loop.run_until_complete(self._mavsdk_main())
        except Exception as exc:
            self.get_logger().error(f'MAVSDK async loop error: {exc}')
        finally:
            loop.close()

    async def _mavsdk_main(self) -> None:
        while self._running:
            try:
                await self._connect_and_stream()
            except Exception as exc:
                self.get_logger().error(f'MAVSDK connection/bridge error: {exc}')
                self._connected = False
                self._offboard_active = False
                self._connection_status = f"ERROR: {str(exc)[:50]}"

            if self._running:
                self.get_logger().info(f'Reconnecting in {self._reconnect_interval}s...')
                await asyncio.sleep(self._reconnect_interval)

    async def _connect_and_stream(self) -> None:
        try:
            from mavsdk import System
        except ImportError:
            self.get_logger().error('MAVSDK-Python not installed. Install with: pip install mavsdk')
            self._connection_status = "MAVSDK_NOT_INSTALLED"
            self._running = False
            return

        self.get_logger().info(f'Connecting to PX4 at: {self._connection_url}')
        self._connection_status = "CONNECTING"
        drone = System()
        self._system = drone
        await drone.connect(system_address=self._connection_url)

        self.get_logger().info('Waiting for drone connection...')
        start_time = time.monotonic()
        async for state in drone.core.connection_state():
            if state.is_connected:
                self._connected = True
                self._connection_status = "CONNECTED"
                self.get_logger().info('Drone connected! Telemetry active; command bridge waiting for gates.')
                break

            if time.monotonic() - start_time > self._connection_timeout:
                self._connected = False
                self._connection_status = "TIMEOUT"
                self.get_logger().warning('Connection timeout.')
                return

        await asyncio.gather(
            self._watch_connection_state(drone),
            self._stream_battery(drone),
            self._stream_position(drone),
            self._stream_position_velocity_ned(drone),
            self._stream_attitude(drone),
            self._stream_velocity(drone),
            self._stream_armed(drone),
            self._stream_flight_mode(drone),
            self._stream_landed_state(drone),
            self._stream_health(drone),
            self._stream_gps_info(drone),
            self._offboard_command_loop(drone),
            self._action_command_loop(drone),
        )

    async def _watch_connection_state(self, drone) -> None:
        """Keep _connected truthful for the LIFETIME of the session.

        The initial wait loop in _connect_and_stream breaks out on the first
        is_connected=True, so without this watcher a later PX4 drop would leave
        _connected stuck True: the dashboard would show CONNECTED and the command
        bridge would keep trying to send.

        Disconnects are DEBOUNCED: under lockstep SITL (and on lossy links)
        MAVSDK's connection_state flaps False->True within a fraction of a
        second every few seconds. Reacting instantly would flicker every
        downstream safety gate and drop one-shot actions, so a loss only counts
        after it persists for the grace window; recovery counts immediately.
        """
        grace_s = 3.0
        pending_disconnect: Optional[asyncio.Task] = None

        async def flip_down_after_grace() -> None:
            try:
                await asyncio.sleep(grace_s)
            except asyncio.CancelledError:
                return
            if self._running and self._connected:
                self._connected = False
                self._connection_status = "DISCONNECTED"
                self._offboard_active = False
                self._prime_hold_position = None
                self.get_logger().error(
                    f'PX4 connection LOST (no MAVSDK heartbeat for {grace_s:.0f}s).')

        async for state in drone.core.connection_state():
            if not self._running:
                if pending_disconnect is not None:
                    pending_disconnect.cancel()
                return
            if state.is_connected:
                if pending_disconnect is not None:
                    pending_disconnect.cancel()
                    pending_disconnect = None
                if not self._connected:
                    self._connected = True
                    self._connection_status = "CONNECTED"
                    self.get_logger().warning('PX4 connection RECOVERED (MAVSDK connection_state).')
            elif pending_disconnect is None or pending_disconnect.done():
                pending_disconnect = asyncio.ensure_future(flip_down_after_grace())

    async def _stream_battery(self, drone) -> None:
        async for battery in drone.telemetry.battery():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['battery_voltage'] = battery.voltage_v
                self._telemetry_data['battery_remaining'] = battery.remaining_percent * 100.0

    async def _stream_position(self, drone) -> None:
        async for position in drone.telemetry.position():
            if not self._running:
                return
            now = time.monotonic()
            with self._data_lock:
                self._telemetry_data['latitude'] = position.latitude_deg
                self._telemetry_data['longitude'] = position.longitude_deg
                self._telemetry_data['absolute_altitude'] = position.absolute_altitude_m
                self._telemetry_data['relative_altitude'] = position.relative_altitude_m
                self._last_position_time = now

    async def _stream_position_velocity_ned(self, drone) -> None:
        try:
            async for pv in drone.telemetry.position_velocity_ned():
                if not self._running:
                    return
                now = time.monotonic()
                with self._data_lock:
                    self._telemetry_data['local_position_valid'] = True
                    self._telemetry_data['local_position_north'] = float(pv.position.north_m)
                    self._telemetry_data['local_position_east'] = float(pv.position.east_m)
                    self._telemetry_data['local_position_down'] = float(pv.position.down_m)
                    self._telemetry_data['velocity_north'] = float(pv.velocity.north_m_s)
                    self._telemetry_data['velocity_east'] = float(pv.velocity.east_m_s)
                    self._telemetry_data['velocity_down'] = float(pv.velocity.down_m_s)
                    self._last_local_position_time = now
        except Exception as exc:
            with self._data_lock:
                self._telemetry_data['local_position_valid'] = False
            self._log_throttled(
                'position_velocity_ned_failed',
                'warning',
                f'position_velocity_ned stream failed/unavailable: {exc}',
                5.0,
            )

    async def _stream_attitude(self, drone) -> None:
        async for attitude in drone.telemetry.attitude_euler():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['roll'] = math.radians(attitude.roll_deg)
                self._telemetry_data['pitch'] = math.radians(attitude.pitch_deg)
                self._telemetry_data['yaw'] = math.radians(attitude.yaw_deg)

    async def _stream_velocity(self, drone) -> None:
        async for velocity in drone.telemetry.velocity_ned():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['velocity_north'] = velocity.north_m_s
                self._telemetry_data['velocity_east'] = velocity.east_m_s
                self._telemetry_data['velocity_down'] = velocity.down_m_s

    async def _stream_armed(self, drone) -> None:
        async for is_armed in drone.telemetry.armed():
            if not self._running:
                return
            now = time.monotonic()
            with self._data_lock:
                self._telemetry_data['armed'] = is_armed
                self._last_armed_time = now

    async def _stream_flight_mode(self, drone) -> None:
        async for flight_mode in drone.telemetry.flight_mode():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['flight_mode'] = str(flight_mode)

    async def _stream_landed_state(self, drone) -> None:
        async for landed_state in drone.telemetry.landed_state():
            if not self._running:
                return
            now = time.monotonic()
            with self._data_lock:
                self._telemetry_data['landed_state'] = str(landed_state)
                self._last_landed_state_time = now

    async def _stream_health(self, drone) -> None:
        async for health in drone.telemetry.health():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['health_all_ok'] = health.is_global_position_ok and health.is_home_position_ok
                self._telemetry_data['health_accelerometer_ok'] = health.is_accelerometer_calibration_ok
                self._telemetry_data['health_gyroscope_ok'] = health.is_gyrometer_calibration_ok
                self._telemetry_data['health_magnetometer_ok'] = health.is_magnetometer_calibration_ok
                self._telemetry_data['health_gps_ok'] = health.is_global_position_ok

    async def _stream_gps_info(self, drone) -> None:
        async for gps_info in drone.telemetry.gps_info():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['gps_num_satellites'] = gps_info.num_satellites
                self._telemetry_data['gps_fix_type'] = gps_info.fix_type.value

    async def _action_command_loop(self, drone) -> None:
        period = 0.1
        while self._running:
            await asyncio.sleep(period)

            with self._action_lock:
                command = self._latest_action_command
                command_time = self._latest_action_command_time

            if command is None:
                continue

            action_key = self._action_dedupe_key(command)
            if action_key in self._handled_action_keys:
                continue

            command_age = time.monotonic() - command_time
            if command_age > self._action_command_timeout:
                self._mark_action_handled(action_key)
                self._publish_command_status(
                    f'{ACTION_IGNORED}: id={command.command_id}, stale={command_age:.2f}s',
                    force=True,
                )
                continue

            self._mark_action_handled(action_key)

            if not bool(command.execute):
                self._publish_command_status(f'{ACTION_IGNORED}: id={command.command_id}, execute=false', force=True)
                continue

            if not self._allow_mavsdk_actions:
                self._publish_command_status(
                    f'{ACTION_DISABLED}: id={command.command_id}, action={command.action}',
                    force=True,
                )
                self._log_throttled(
                    'actions_disabled',
                    'warning',
                    'MAVSDK action request blocked because allow_mavsdk_actions is false.',
                    2.0,
                )
                continue

            if not self._connected:
                self._publish_command_status(f'{ACTION_FAILED}: id={command.command_id}, PX4_NOT_CONNECTED', force=True)
                continue

            allowed, reason = self._action_policy_allows(command)
            if not allowed:
                self._publish_command_status(
                    f'{ACTION_BLOCKED}: id={command.command_id}, action={command.action}, reason={reason}',
                    force=True,
                )
                self._log_throttled(
                    'action_policy_blocked',
                    'warning',
                    f'MAVSDK action blocked by policy: id={command.command_id}, action={command.action}, reason={reason}',
                    1.0,
                )
                continue

            self._action_in_progress = True
            try:
                if self._offboard_active:
                    await self._stop_offboard(drone)
                await self._execute_action_command(drone, command)
                self._publish_command_status(
                    f'{ACTION_SENT}: id={command.command_id}, action={command.action}',
                    force=True,
                )
            except Exception as exc:
                self._publish_command_status(
                    f'{ACTION_FAILED}: id={command.command_id}, action={command.action}, error={str(exc)[:80]}',
                    force=True,
                )
                self._log_throttled('action_failed', 'warning', f'MAVSDK action failed: {exc}', 1.0)
            finally:
                self._action_in_progress = False

    @staticmethod
    def _action_dedupe_key(command: MavsdkActionCommand) -> tuple[int, str, int, int, str]:
        stamp = getattr(command, 'stamp', None)
        stamp_sec = int(getattr(stamp, 'sec', 0))
        stamp_nanosec = int(getattr(stamp, 'nanosec', 0))
        return (
            int(command.command_id),
            str(command.action).strip().upper(),
            stamp_sec,
            stamp_nanosec,
            str(command.note),
        )

    def _mark_action_handled(self, action_key: tuple[int, str, int, int, str]) -> None:
        self._handled_action_keys.add(action_key)
        self._handled_action_key_order.append(action_key)

        # Keep the dedupe cache bounded. It only needs to suppress repeats of
        # recently received ROS messages, not remember every action forever.
        max_keys = 128
        while len(self._handled_action_key_order) > max_keys:
            old_key = self._handled_action_key_order.pop(0)
            self._handled_action_keys.discard(old_key)

    def _action_policy_allows(self, command: MavsdkActionCommand) -> tuple[bool, str]:
        action = str(command.action).strip().upper()
        if action in ('LAND', 'RETURN_TO_LAUNCH', 'RTL', 'HOLD'):
            return True, "ACTION_ESCAPE_ALLOWED"

        if action == 'TAKEOFF':
            state = self._action_state_snapshot()
            ok, reason = self._action_state_fresh(
                state,
                'position_time',
                'armed_time',
                'landed_state_time',
            )
            if not ok:
                return False, reason
            if not state['armed']:
                return False, 'PX4_NOT_ARMED'
            if self._action_state_airborne(state):
                return False, 'ALREADY_AIRBORNE'
            return True, 'TAKEOFF_POLICY_OK'

        if action == 'DO_ORBIT':
            state = self._action_state_snapshot()
            ok, reason = self._action_state_fresh(
                state,
                'position_time',
                'local_position_time',
                'landed_state_time',
            )
            if not ok:
                return False, reason
            if not self._action_state_airborne(state):
                return False, 'NOT_AIRBORNE'
            if not self._action_state_local_position_ready(state):
                return False, 'LOCAL_POSITION_NOT_VALID'
            return True, 'DO_ORBIT_POLICY_OK'

        return True, 'NO_EXTRA_POLICY'

    def _action_state_snapshot(self) -> dict:
        with self._data_lock:
            return {
                'armed': bool(self._telemetry_data['armed']),
                'relative_altitude': float(self._telemetry_data['relative_altitude']),
                'local_position_valid': bool(self._telemetry_data['local_position_valid']),
                'local_position_north': float(self._telemetry_data['local_position_north']),
                'local_position_east': float(self._telemetry_data['local_position_east']),
                'local_position_down': float(self._telemetry_data['local_position_down']),
                'landed_state': str(self._telemetry_data['landed_state']).upper(),
                'position_time': float(self._last_position_time),
                'local_position_time': float(self._last_local_position_time),
                'armed_time': float(self._last_armed_time),
                'landed_state_time': float(self._last_landed_state_time),
            }

    def _action_state_fresh(self, state: dict, *time_keys: str) -> tuple[bool, str]:
        now = time.monotonic()
        for key in time_keys:
            seen = float(state.get(key, 0.0))
            if seen <= 0.0:
                return False, f'{key.upper()}_NEVER_RECEIVED'
            age = now - seen
            if age > self._action_telemetry_timeout:
                return False, f'{key.upper()}_STALE ({age:.2f}s)'
        return True, 'ACTION_TELEMETRY_FRESH'

    def _action_state_airborne(self, state: dict) -> bool:
        landed_state = str(state.get('landed_state', '')).upper()
        altitude = float(state.get('relative_altitude', 0.0))
        if 'ON_GROUND' in landed_state or 'LANDED' in landed_state:
            return False
        if 'IN_AIR' in landed_state or 'TAKING_OFF' in landed_state:
            return True
        return altitude >= self._action_airborne_altitude_m

    @staticmethod
    def _action_state_local_position_ready(state: dict) -> bool:
        if not bool(state.get('local_position_valid', False)):
            return False
        return all(
            math.isfinite(float(state.get(key, float('nan'))))
            for key in ('local_position_north', 'local_position_east', 'local_position_down')
        )

    async def _execute_action_command(self, drone, command: MavsdkActionCommand) -> None:
        action = str(command.action).strip().upper()

        if action == 'ARM':
            raise ValueError('ARM action is intentionally not supported by telemetry_node; arm manually from RC/QGC')

        if action == 'TAKEOFF':
            altitude = max(0.5, float(command.takeoff_altitude_m))
            try:
                await drone.action.set_takeoff_altitude(altitude)
            except Exception as exc:
                self._log_throttled('set_takeoff_altitude_failed', 'warning', f'set_takeoff_altitude failed: {exc}', 2.0)
            await drone.action.takeoff()
            return

        if action == 'LAND':
            await drone.action.land()
            return

        if action in ('RETURN_TO_LAUNCH', 'RTL'):
            await drone.action.return_to_launch()
            return

        if action == 'HOLD':
            await drone.action.hold()
            return

        if action == 'DO_ORBIT':
            await self._send_do_orbit_mavlink_direct(drone, command)
            return

        raise ValueError(f'unknown MAVSDK action: {command.action}')

    async def _send_do_orbit_mavlink_direct(self, drone, command: MavsdkActionCommand) -> None:
        """Send MAV_CMD_DO_ORBIT through MAVSDK MavlinkDirect.

        MAVSDK action.do_orbit() is nice, but it does not expose MAV_CMD_DO_ORBIT
        param4, which is the orbit amount in radians. MavlinkDirect lets the
        mission layer request a fixed number of revolutions while telemetry_node
        remains the only owner of the MAVSDK connection.
        """
        radius_m = float(command.radius_m)
        velocity_m_s = float(command.velocity_m_s)
        revolutions = float(command.orbit_revolutions)
        orbit_angle_rad = 0.0 if revolutions <= 0.0 else revolutions * 2.0 * math.pi
        yaw_behavior = self._orbit_yaw_behavior_value(command.yaw_behavior)

        fields = {
            'target_system': self._get_target_system_id(drone),
            'target_component': MAV_COMP_ID_AUTOPILOT1,
            'command': MAV_CMD_DO_ORBIT,
            'confirmation': 0,
            'param1': radius_m,
            'param2': velocity_m_s,
            'param3': float(yaw_behavior),
            'param4': orbit_angle_rad,
            'param5': self._json_float_or_null(float(command.latitude_deg)),
            'param6': self._json_float_or_null(float(command.longitude_deg)),
            'param7': self._json_float_or_null(float(command.absolute_altitude_m)),
        }

        mavlink_direct = getattr(drone, 'mavlink_direct', None)
        if mavlink_direct is None:
            raise RuntimeError('MAVSDK-Python has no mavlink_direct plugin on this install')
        if not hasattr(mavlink_direct, 'send_message'):
            raise RuntimeError('MAVSDK-Python mavlink_direct plugin has no send_message() method')

        try:
            from mavsdk.mavlink_direct import MavlinkMessage
        except ImportError as exc:
            raise RuntimeError('mavsdk.mavlink_direct.MavlinkMessage import failed') from exc

        # MAVSDK-Python generated structs use positional construction. Keep this
        # intentionally simple so COMMAND_LONG matches the documented
        # MavlinkMessage fields: name, sender ids, target ids, JSON fields.
        message = MavlinkMessage(
            'COMMAND_LONG',
            0,
            0,
            int(fields['target_system']),
            int(fields['target_component']),
            json.dumps(fields, allow_nan=False),
        )

        self.get_logger().warning(
            'Sending MAV_CMD_DO_ORBIT via MavlinkDirect | '
            f'radius={radius_m:.2f}m, velocity={velocity_m_s:.2f}m/s, '
            f'revolutions={revolutions:.2f}, orbit_angle={orbit_angle_rad:.3f}rad, '
            f'yaw_behavior={yaw_behavior}, center=({fields["param5"]}, {fields["param6"]}, {fields["param7"]})'
        )

        # In MAVSDK-Python, plugin calls normally raise on failure. Some generated
        # variants return None/False-ish values even when the message was accepted,
        # so do not convert the return value into a second failure gate.
        await mavlink_direct.send_message(message)

    def _get_target_system_id(self, drone) -> int:
        """Best-effort target system id for COMMAND_LONG.

        MAVSDK normally talks to one system in this project. Use target_system=1
        as the practical default, but take an exposed value if this package gives
        us one.
        """
        mavlink_direct = getattr(drone, 'mavlink_direct', None)
        for attr in ('target_system_id', 'target_sysid', 'system_id'):
            value = getattr(mavlink_direct, attr, None) if mavlink_direct is not None else None
            if isinstance(value, int) and value > 0:
                return value
        return 1

    def _orbit_yaw_behavior_value(self, behavior_name: str) -> int:
        name = str(behavior_name).strip().upper()
        if not name:
            return ORBIT_YAW_BEHAVIOR_VALUES['HOLD_FRONT_TO_CIRCLE_CENTER']
        if name in ORBIT_YAW_BEHAVIOR_VALUES:
            return ORBIT_YAW_BEHAVIOR_VALUES[name]
        self._log_throttled(
            'unknown_orbit_yaw_behavior',
            'warning',
            f'Unknown orbit yaw_behavior={behavior_name!r}; using HOLD_FRONT_TO_CIRCLE_CENTER.',
            2.0,
        )
        return ORBIT_YAW_BEHAVIOR_VALUES['HOLD_FRONT_TO_CIRCLE_CENTER']

    @staticmethod
    def _json_float_or_null(value: float):
        # MavlinkDirect represents MAVLink NaN/infinity as JSON null.
        if math.isnan(value) or math.isinf(value):
            return None
        return float(value)

    async def _offboard_command_loop(self, drone) -> None:
        try:
            from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityBodyYawspeed
        except ImportError:
            self.get_logger().error('MAVSDK offboard plugin import failed. Check MAVSDK-Python install.')
            return

        period = 1.0 / self._command_rate
        zero_velocity_setpoint = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)

        while self._running:
            if self._action_in_progress:
                if self._offboard_active:
                    await self._stop_offboard(drone)
                self._publish_command_status('OFFBOARD_PAUSED_FOR_ACTION')
                await asyncio.sleep(period)
                continue

            decision = self._get_command_decision()

            if not decision['executor_enabled']:
                if self._offboard_active and self._stop_offboard_on_disable:
                    await self._stop_offboard(drone)
                self._publish_command_status(BRIDGE_DISABLED)
                await asyncio.sleep(period)
                continue

            if not decision['valid']:
                if decision.get('prime_ok', False):
                    prime_setpoint = self._get_or_capture_prime_position_setpoint(PositionNedYaw)
                    if prime_setpoint is None:
                        if self._offboard_active:
                            await self._stop_offboard(drone)
                        self._publish_command_status(f'{BRIDGE_NO_LOCAL_POSITION}: waiting for position_velocity_ned')
                        await asyncio.sleep(period)
                        continue

                    if not self._offboard_active:
                        try:
                            # PX4/MAVSDK requires a setpoint before starting Offboard.
                            # Prime with a captured local NED hold position so there is
                            # no body-frame velocity drift during the button/state race.
                            await drone.offboard.set_position_ned(prime_setpoint)
                            await drone.offboard.start()
                            self._offboard_active = True
                            self.get_logger().warning('*** PX4 OFFBOARD PRIMED with local POSITION hold setpoint ***')
                            self._publish_command_status(BRIDGE_PRIMING, force=True)
                        except OffboardError as exc:
                            result = getattr(getattr(exc, '_result', None), 'result', exc)
                            self._publish_command_status(f'{BRIDGE_OFFBOARD_START_FAILED}: {result}', force=True)
                            self._log_throttled(
                                'offboard_prime_failed',
                                'warning',
                                f'Offboard prime failed: {result}. Keep PX4 armed/airborne and keep setpoints flowing.',
                                2.0,
                            )
                            await asyncio.sleep(period)
                            continue
                    await self._send_position_ned(
                        drone,
                        prime_setpoint,
                        f'{BRIDGE_SENT}: position_hold_prime {decision["reason"]}',
                        is_hold=True,
                    )
                else:
                    if self._offboard_active:
                        hold_setpoint = self._get_or_capture_prime_position_setpoint(PositionNedYaw)
                        if hold_setpoint is not None:
                            await self._send_position_ned(drone, hold_setpoint, BRIDGE_ZERO_SENT, is_hold=True)
                        else:
                            await self._send_velocity_body(drone, zero_velocity_setpoint, BRIDGE_ZERO_SENT, is_zero=True)
                    else:
                        self._publish_command_status(str(decision['reason']))
                        self._log_throttled('offboard_blocked', 'info', f'Offboard blocked: {decision["reason"]}', 2.0)
                await asyncio.sleep(period)
                continue

            if decision['command_type'] == CMD_POSITION:
                position_setpoint = PositionNedYaw(
                    float(decision['position_north_m']),
                    float(decision['position_east_m']),
                    float(decision['position_down_m']),
                    float(decision['yaw_deg']),
                )

                if not self._offboard_active:
                    try:
                        await drone.offboard.set_position_ned(position_setpoint)
                        await drone.offboard.start()
                        self._offboard_active = True
                        self.get_logger().warning('*** PX4 OFFBOARD STARTED in POSITION hold mode ***')
                        self._publish_command_status(BRIDGE_OFFBOARD_STARTED, force=True)
                    except OffboardError as exc:
                        result = getattr(getattr(exc, '_result', None), 'result', exc)
                        self._publish_command_status(f'{BRIDGE_OFFBOARD_START_FAILED}: {result}', force=True)
                        self._log_throttled(
                            'offboard_start_failed',
                            'warning',
                            f'Offboard start failed: {result}. Drone must already be safely armed/ready; this node will not arm or take off.',
                            2.0,
                        )
                        await asyncio.sleep(period)
                        continue

                await self._send_position_ned(
                    drone,
                    position_setpoint,
                    (
                        f'{BRIDGE_SENT}: posNED=({decision["position_north_m"]:+.2f},'
                        f'{decision["position_east_m"]:+.2f},{decision["position_down_m"]:+.2f})m, '
                        f'yaw={decision["yaw_deg"]:.1f}deg, yaw_rate={decision["yaw_rate_rad_s"]:+.3f}rad/s'
                    ),
                    is_hold=False,
                )
                await asyncio.sleep(period)
                continue

            velocity_setpoint = VelocityBodyYawspeed(
                float(decision['forward_m_s']),
                float(decision['right_m_s']),
                float(decision['down_m_s']),
                float(decision['yawspeed_deg_s']),
            )

            if not self._offboard_active:
                try:
                    await drone.offboard.set_velocity_body(zero_velocity_setpoint)
                    await drone.offboard.start()
                    self._offboard_active = True
                    self.get_logger().warning('*** PX4 OFFBOARD STARTED by legacy VELOCITY command bridge ***')
                    self._publish_command_status(BRIDGE_OFFBOARD_STARTED, force=True)
                except OffboardError as exc:
                    result = getattr(getattr(exc, '_result', None), 'result', exc)
                    self._publish_command_status(f'{BRIDGE_OFFBOARD_START_FAILED}: {result}', force=True)
                    self._log_throttled(
                        'offboard_start_failed',
                        'warning',
                        f'Offboard start failed: {result}. Drone must already be safely armed/ready; this node will not arm or take off.',
                        2.0,
                    )
                    await asyncio.sleep(period)
                    continue

            await self._send_velocity_body(
                drone,
                velocity_setpoint,
                f'{BRIDGE_SENT}: yaw={decision["yaw_rate_rad_s"]:+.3f}rad/s ({decision["yawspeed_deg_s"]:+.1f}deg/s)',
                is_zero=False,
            )
            await asyncio.sleep(period)

    def _get_or_capture_prime_position_setpoint(self, position_cls):
        if self._prime_hold_position is None:
            with self._data_lock:
                valid = bool(self._telemetry_data['local_position_valid'])
                north = float(self._telemetry_data['local_position_north'])
                east = float(self._telemetry_data['local_position_east'])
                down = float(self._telemetry_data['local_position_down'])
                yaw_deg = math.degrees(float(self._telemetry_data['yaw'])) % 360.0
                ned_time = float(self._last_local_position_time)

            if not valid or not all(math.isfinite(v) for v in (north, east, down, yaw_deg)):
                return None
            # Never anchor a hold position on a stale NED sample.
            if ned_time <= 0.0 or time.monotonic() - ned_time > self._action_telemetry_timeout:
                return None

            self._prime_hold_position = (north, east, down, yaw_deg)
            self.get_logger().warning(
                'Captured bridge prime POSITION hold | '
                f'N={north:.2f}m, E={east:.2f}m, D={down:.2f}m, yaw={yaw_deg:.1f}deg'
            )

        north, east, down, yaw_deg = self._prime_hold_position
        return position_cls(float(north), float(east), float(down), float(yaw_deg))

    async def _send_position_ned(self, drone, position_setpoint, status: str, is_hold: bool) -> None:
        try:
            await drone.offboard.set_position_ned(position_setpoint)
            self._px4_send_count += 1
            if is_hold:
                self._px4_zero_count += 1
            self._last_bridge_status = status
            self._publish_command_status(status)
            if not is_hold:
                self._log_throttled('position_sent', 'info', f'MAVSDK position command sent | {status}', 0.5)
        except Exception as exc:
            self._publish_command_status(f'{BRIDGE_SEND_FAILED}: {str(exc)[:80]}', force=True)
            self._log_throttled('send_failed', 'warning', f'Offboard position setpoint send failed: {exc}', 1.0)
            self._offboard_active = False

    async def _send_velocity_body(self, drone, velocity_setpoint, status: str, is_zero: bool) -> None:
        try:
            await drone.offboard.set_velocity_body(velocity_setpoint)
            self._px4_send_count += 1
            if is_zero:
                self._px4_zero_count += 1
            self._last_bridge_status = status
            self._publish_command_status(status)
            if not is_zero:
                self._log_throttled('yaw_sent', 'info', f'MAVSDK yaw command sent | {status}', 0.5)
        except Exception as exc:
            self._publish_command_status(f'{BRIDGE_SEND_FAILED}: {str(exc)[:80]}', force=True)
            self._log_throttled('send_failed', 'warning', f'Offboard setpoint send failed: {exc}', 1.0)
            self._offboard_active = False

    async def _stop_offboard(self, drone) -> None:
        try:
            await drone.offboard.stop()
            self._offboard_active = False
            self._prime_hold_position = None
            self.get_logger().warning('PX4 Offboard stopped by MAVSDK command bridge.')
            self._publish_command_status(BRIDGE_OFFBOARD_STOPPED, force=True)
        except Exception as exc:
            self._offboard_active = False
            self._publish_command_status(f'OFFBOARD_STOP_FAILED: {str(exc)[:80]}', force=True)
            self._log_throttled('stop_failed', 'warning', f'Offboard stop failed: {exc}', 1.0)

    def _get_command_decision(self) -> dict:
        now = time.monotonic()

        with self._command_lock:
            command = self._latest_command
            command_time = self._latest_command_time

        with self._data_lock:
            armed = bool(self._telemetry_data['armed'])
            battery_remaining = float(self._telemetry_data['battery_remaining'])
            flight_mode = str(self._telemetry_data['flight_mode'])
            local_position_valid = bool(self._telemetry_data['local_position_valid'])
            local_position_time = float(self._last_local_position_time)

        decision = {
            'executor_enabled': self._mavsdk_offboard_enabled,
            'valid': False,
            'prime_ok': False,
            'reason': BRIDGE_DISABLED,
            'command_type': CMD_IDLE,
            'forward_m_s': 0.0,
            'right_m_s': 0.0,
            'down_m_s': 0.0,
            'yaw_rate_rad_s': 0.0,
            'yawspeed_deg_s': 0.0,
            'position_north_m': 0.0,
            'position_east_m': 0.0,
            'position_down_m': 0.0,
            'yaw_deg': 0.0,
            'flight_mode': flight_mode,
        }

        if not self._mavsdk_offboard_enabled:
            return decision

        if not self._connected:
            decision['reason'] = BRIDGE_NOT_CONNECTED
            return decision

        if self._require_armed_for_offboard and not armed:
            decision['reason'] = BRIDGE_NOT_ARMED
            return decision

        if battery_remaining > 0.0 and battery_remaining < self._min_battery_percent:
            decision['reason'] = f'{BRIDGE_LOW_BATTERY}: {battery_remaining:.1f}%'
            return decision

        # Once the executor gate is true and PX4 is safe, allow position-hold
        # Offboard priming before target lock produces yaw commands.
        decision['prime_ok'] = True

        if command is None:
            decision['reason'] = BRIDGE_NO_COMMAND
            return decision

        command_age = now - command_time
        if command_age > self._command_timeout:
            decision['reason'] = f'{BRIDGE_COMMAND_STALE} ({command_age:.2f}s)'
            return decision

        if not command.executed:
            decision['reason'] = (
                f'{BRIDGE_COMMAND_NOT_APPROVED}: executed={command.executed}, '
                f'status={command.execution_status}'
            )
            return decision

        yaw_rate_rad_s = self._clamp(
            float(command.yaw_rate),
            -self._max_yaw_rate_rad_s,
            self._max_yaw_rate_rad_s,
        )

        if command.command_type == CMD_POSITION:
            north = float(getattr(command, 'position_north', 0.0))
            east = float(getattr(command, 'position_east', 0.0))
            down = float(getattr(command, 'position_down', 0.0))
            yaw_deg = float(getattr(command, 'yaw_deg', 0.0)) % 360.0
            if not bool(getattr(command, 'position_valid', False)):
                decision['reason'] = f'{BRIDGE_BAD_POSITION_COMMAND}: position_valid=false'
                return decision
            # Defense-in-depth: don't forward an absolute NED setpoint unless the
            # bridge's OWN live telemetry says local position is currently valid
            # AND recent. The command's position_valid bit can be stale if the
            # EKF/GPS dropped out after the command was built upstream, and the
            # local flag itself is only set by the NED stream — if that stream
            # stalls the flag would otherwise stay true forever.
            if not local_position_valid:
                decision['reason'] = f'{BRIDGE_BAD_POSITION_COMMAND}: local_position_invalid (live telemetry)'
                return decision
            ned_age = now - local_position_time
            if local_position_time <= 0.0 or ned_age > self._action_telemetry_timeout:
                decision['reason'] = f'{BRIDGE_BAD_POSITION_COMMAND}: local_position_stale ({ned_age:.2f}s)'
                return decision
            if not all(math.isfinite(v) for v in (north, east, down, yaw_deg)):
                decision['reason'] = f'{BRIDGE_BAD_POSITION_COMMAND}: non_finite'
                return decision

            decision.update(
                {
                    'valid': True,
                    'reason': BRIDGE_READY,
                    'command_type': CMD_POSITION,
                    'position_north_m': north,
                    'position_east_m': east,
                    'position_down_m': down,
                    'yaw_deg': yaw_deg,
                    'yaw_rate_rad_s': yaw_rate_rad_s,
                    'yawspeed_deg_s': math.degrees(yaw_rate_rad_s),
                }
            )
            return decision

        if command.execution_status != STATUS_SENT:
            decision['reason'] = (
                f'{BRIDGE_COMMAND_NOT_APPROVED}: executed={command.executed}, '
                f'status={command.execution_status}'
            )
            return decision

        if command.command_type != CMD_VELOCITY:
            decision['reason'] = f'{BRIDGE_COMMAND_IDLE}: type={command.command_type}, status={command.execution_status}'
            return decision

        forward_m_s = 0.0
        right_m_s = 0.0
        down_m_s = 0.0
        if self._allow_translation_commands:
            # Disabled by default. Stage 1 uses POSITION hold instead of VELOCITY translation.
            forward_m_s = float(command.velocity_forward)
            right_m_s = float(command.velocity_right)
            down_m_s = float(command.velocity_down)

        decision.update(
            {
                'valid': True,
                'reason': BRIDGE_READY,
                'command_type': CMD_VELOCITY,
                'forward_m_s': forward_m_s,
                'right_m_s': right_m_s,
                'down_m_s': down_m_s,
                'yaw_rate_rad_s': yaw_rate_rad_s,
                'yawspeed_deg_s': math.degrees(yaw_rate_rad_s),
            }
        )
        return decision

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    def _publish_command_status(self, status: str, force: bool = False) -> None:
        now = time.monotonic()
        if not force and status == self._last_bridge_status and now - self._last_status_publish_time < 1.0:
            return

        self._last_bridge_status = status
        self._last_status_publish_time = now
        msg = String()
        msg.data = status
        self._command_status_pub.publish(msg)
        self._diagnostics.mark_published(
            self._command_status_topic,
            summary=(
                f'status={status}, offboard_enabled={self._mavsdk_offboard_enabled}, '
                f'offboard_active={self._offboard_active}, sent={self._px4_send_count}, zeros={self._px4_zero_count}'
            ),
        )

    def _log_throttled(self, key: str, level: str, message: str, interval: float) -> None:
        now = time.monotonic()
        last = self._last_log_times.get(key, 0.0)
        if now - last < interval:
            return
        self._last_log_times[key] = now
        logger = self.get_logger()
        if level == 'warning':
            logger.warning(message)
        elif level == 'error':
            logger.error(message)
        else:
            logger.info(message)

    def _publish_telemetry(self) -> None:
        msg = DroneTelemetry()
        msg.stamp = self.get_clock().now().to_msg()

        msg.connected = self._connected
        msg.connection_status = self._connection_status

        with self._data_lock:
            msg.battery_voltage = float(self._telemetry_data['battery_voltage'])
            msg.battery_remaining_percent = float(self._telemetry_data['battery_remaining'])
            msg.latitude = float(self._telemetry_data['latitude'])
            msg.longitude = float(self._telemetry_data['longitude'])
            msg.absolute_altitude = float(self._telemetry_data['absolute_altitude'])
            msg.relative_altitude = float(self._telemetry_data['relative_altitude'])
            msg.local_position_valid = bool(self._telemetry_data['local_position_valid'])
            msg.local_position_north = float(self._telemetry_data['local_position_north'])
            msg.local_position_east = float(self._telemetry_data['local_position_east'])
            msg.local_position_down = float(self._telemetry_data['local_position_down'])
            msg.gps_num_satellites = int(self._telemetry_data['gps_num_satellites'])
            msg.gps_fix_type = int(self._telemetry_data['gps_fix_type'])
            msg.roll = float(self._telemetry_data['roll'])
            msg.pitch = float(self._telemetry_data['pitch'])
            msg.yaw = float(self._telemetry_data['yaw'])
            msg.velocity_north = float(self._telemetry_data['velocity_north'])
            msg.velocity_east = float(self._telemetry_data['velocity_east'])
            msg.velocity_down = float(self._telemetry_data['velocity_down'])
            msg.armed = bool(self._telemetry_data['armed'])
            msg.flight_mode = str(self._telemetry_data['flight_mode'])
            msg.landed_state = str(self._telemetry_data['landed_state'])
            msg.health_all_ok = bool(self._telemetry_data['health_all_ok'])
            msg.health_accelerometer_ok = bool(self._telemetry_data['health_accelerometer_ok'])
            msg.health_gyroscope_ok = bool(self._telemetry_data['health_gyroscope_ok'])
            msg.health_magnetometer_ok = bool(self._telemetry_data['health_magnetometer_ok'])
            msg.health_gps_ok = bool(self._telemetry_data['health_gps_ok'])

        self._telemetry_pub.publish(msg)
        self._telemetry_publish_count += 1
        self._diagnostics.mark_published(
            self._telemetry_topic,
            summary=(
                f"messages={self._telemetry_publish_count}, connected={msg.connected}, "
                f"status={msg.connection_status}, battery={msg.battery_remaining_percent:.1f}%"
            ),
        )

    def _report_status(self) -> None:
        with self._data_lock:
            armed = self._telemetry_data['armed']
            flight_mode = self._telemetry_data['flight_mode']
            battery = self._telemetry_data['battery_remaining']
            local_valid = self._telemetry_data['local_position_valid']

        with self._command_lock:
            command_age = None if self._latest_command is None else time.monotonic() - self._latest_command_time
            latest_type = 'NONE' if self._latest_command is None else self._latest_command.command_type
            latest_status = 'NONE' if self._latest_command is None else self._latest_command.execution_status
            latest_yaw = 0.0 if self._latest_command is None else float(self._latest_command.yaw_rate)

        age_text = 'never' if command_age is None else f'{command_age:.2f}s'
        self.get_logger().info(
            'MAVSDK bridge status | '
            f'telemetry_published={self._telemetry_publish_count}, connected={self._connected}, '
            f'status={self._connection_status}, armed={armed}, mode={flight_mode}, battery={battery:.1f}%, local_ned={local_valid}, '
            f'offboard_enabled={self._mavsdk_offboard_enabled}, offboard_active={self._offboard_active}, '
            f'control_msgs={self._control_command_count}, command_age={age_text}, '
            f'latest_type={latest_type}, latest_status={latest_status}, latest_yaw={latest_yaw:+.3f}, '
            f'px4_sends={self._px4_send_count}, zero_sends={self._px4_zero_count}, '
            f'bridge_status={self._last_bridge_status}, actions={self._action_command_count}, allow_actions={self._allow_mavsdk_actions}'
        )

    def destroy_node(self) -> None:
        self.get_logger().info('Shutting down PX4 MAVSDK bridge node...')
        self._running = False
        if self._async_thread is not None:
            self._async_thread.join(timeout=5.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TelemetryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger('telemetry_node').fatal(f'Fatal: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
