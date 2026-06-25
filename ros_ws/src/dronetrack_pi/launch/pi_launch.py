"""Pi-side launch for the split DroneTrack architecture.

Runs everything that MUST stay on the drone:
  - camera publisher                      (reused: drone_camera)
  - compressed camera stream for Wi-Fi    (dronetrack_pi compressor)
  - tracker / target selection            (reused: drone_tracker)
  - mission / autonomy / control          (reused: drone_control)
  - MAVSDK / PX4 bridge + action gate     (reused: drone_telemetry)
  - health monitor                        (reused: drone_diagnostics)
  - detection gate (laptop perception in) (new: dronetrack_pi)
  - ground-station watchdog               (new: dronetrack_pi)

YOLO and the dashboard do NOT run here — they run on the laptop ground station.

The "reused" nodes come from dronetrack_pi_ros. Copy those packages into this
workspace's src/ (see scripts/setup_pi.sh and docs/migration_from_dronetrack_pi_ros.md).
If a reused package is not present yet, set the matching *_enabled arg to false
so you can still bring up the new boundary nodes for bench testing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pi_share = get_package_share_directory('dronetrack_pi')
    default_params = os.path.join(pi_share, 'config', 'pi.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Path to the Pi params YAML (defaults to installed configs/pi.yaml).')
    raw_image_topic_arg = DeclareLaunchArgument(
        'raw_image_topic', default_value='/drone/camera/image_raw',
        description='Raw image topic published locally on the Pi.')
    compressed_image_topic_arg = DeclareLaunchArgument(
        'compressed_image_topic', default_value='/drone/camera/image_raw/compressed',
        description='Compressed topic streamed to the laptop.')
    connection_url_arg = DeclareLaunchArgument(
        'connection_url', default_value='serial:///dev/ttyACM0:57600',
        description='MAVSDK connection URL to PX4.')
    allow_mavsdk_actions_arg = DeclareLaunchArgument(
        'allow_mavsdk_actions', default_value='false',
        description='SITL/dev only: allow TAKEOFF/LAND/RTL/HOLD through MAVSDK.')

    # Toggles so you can bring up the new boundary nodes even if a reused package
    # has not been copied into the workspace yet.
    reused_pi_nodes_arg = DeclareLaunchArgument(
        'reused_pi_nodes', default_value='true',
        description='Launch camera/tracker/control/telemetry/health from dronetrack_pi_ros.')
    compress_arg = DeclareLaunchArgument(
        'compress', default_value='true',
        description='Run the dronetrack_pi camera compressor raw->compressed for the laptop.')

    params = LaunchConfiguration('params_file')

    # ---- New boundary nodes (this package) -------------------------------
    detection_gate = Node(
        package='dronetrack_pi', executable='detection_gate_node',
        name='detection_gate_node', parameters=[params], output='screen')

    watchdog = Node(
        package='dronetrack_pi', executable='ground_station_watchdog_node',
        name='ground_station_watchdog_node', parameters=[params], output='screen')

    # ---- Compressed camera stream for Wi-Fi ------------------------------
    # Keep this as a normal package-owned node instead of shelling out to
    # image_transport republish. It gives us explicit sensor-data QoS and avoids
    # remap/plugin ambiguity across machines.
    compress = Node(
        package='dronetrack_pi', executable='camera_compressor_node',
        name='camera_compressor_node',
        parameters=[params, {
            'image_topic': LaunchConfiguration('raw_image_topic'),
            'compressed_image_topic': LaunchConfiguration('compressed_image_topic'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('compress')))

    # ---- Reused safety-critical Pi nodes (from dronetrack_pi_ros) ---------
    reused = IfCondition(LaunchConfiguration('reused_pi_nodes'))
    camera = Node(package='drone_camera', executable='camera_node', name='camera_node',
                  parameters=[params], output='screen', condition=reused)
    tracker = Node(package='drone_tracker', executable='tracker_node', name='tracker_node',
                   parameters=[params], output='screen', condition=reused)
    telemetry = Node(package='drone_telemetry', executable='telemetry_node', name='telemetry_node',
                     parameters=[params, {
                         'connection_url': LaunchConfiguration('connection_url'),
                         # Wire the launch arg through so allow_mavsdk_actions:=true
                         # actually opens the action gate (was silently dropped).
                         'allow_mavsdk_actions': ParameterValue(
                             LaunchConfiguration('allow_mavsdk_actions'), value_type=bool),
                     }],
                     output='screen', condition=reused)
    autonomy_manager = Node(package='drone_control', executable='autonomy_manager_node',
                            name='autonomy_manager_node', parameters=[params],
                            output='screen', emulate_tty=True, condition=reused)
    mission_executor = Node(package='drone_control', executable='mission_executor_node',
                            name='mission_executor_node', parameters=[params],
                            output='screen', emulate_tty=True, condition=reused)
    control = Node(package='drone_control', executable='control_node', name='control_node',
                   parameters=[params], output='screen', emulate_tty=True, condition=reused)
    health = Node(package='drone_diagnostics', executable='health_monitor_node',
                  name='health_monitor_node', parameters=[params], output='screen', condition=reused)

    return LaunchDescription([
        params_file_arg, raw_image_topic_arg, compressed_image_topic_arg,
        connection_url_arg, allow_mavsdk_actions_arg, reused_pi_nodes_arg, compress_arg,
        LogInfo(msg='=== DRONETRACK PI (on-drone) — safety-critical stack ==='),
        LogInfo(msg='YOLO + dashboard run on the LAPTOP ground station.'),
        LogInfo(msg=['Streaming compressed camera on ', LaunchConfiguration('compressed_image_topic')]),
        detection_gate,
        watchdog,
        compress,
        camera,
        tracker,
        telemetry,
        autonomy_manager,
        mission_executor,
        control,
        health,
    ])
