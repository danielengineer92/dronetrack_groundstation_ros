"""Laptop ground-station launch for the split DroneTrack architecture.

Runs the compute-heavy / user-facing nodes:
  - YOLO inference        (dronetrack_perception)  subscribes Pi compressed stream
  - heartbeat publisher   (dronetrack_groundstation) link liveness beacon to the Pi
  - web dashboard         (dronetrack_web_bridge)   connection/FPS/latency/status UI

Nothing here touches flight control, arming, failsafes, or MAVSDK. The Pi
validates every detection and remains safe if this machine disconnects.
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
    gs_share = get_package_share_directory('dronetrack_groundstation')
    default_params = os.path.join(gs_share, 'config', 'groundstation.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Path to the ground-station params YAML.')
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='yolov8n.pt',
        description='Path to the YOLO model (e.g. models/red_ball_ncnn_model or a .pt file).')
    target_class_arg = DeclareLaunchArgument(
        'target_class', default_value='',
        description='Target class to detect. Empty = all classes (right for the default '
                    'yolov8n COCO model). Set e.g. red_ball with a matching model_path.')
    device_arg = DeclareLaunchArgument(
        'device', default_value='cpu',
        description="YOLO device: cpu, cuda:0, mps, ...")
    half_precision_arg = DeclareLaunchArgument(
        'half_precision', default_value='false',
        description='FP16 inference. Use true with device:=cuda:0 for a GPU speedup.')
    dashboard_arg = DeclareLaunchArgument(
        'dashboard', default_value='true',
        description='Run the web dashboard.')
    dashboard_port_arg = DeclareLaunchArgument(
        'dashboard_port', default_value='8080',
        description='HTTP port for the dashboard.')

    params = LaunchConfiguration('params_file')

    yolo = Node(
        package='dronetrack_perception', executable='yolo_node', name='yolo_node',
        parameters=[params, {
            'model_path': LaunchConfiguration('model_path'),
            'target_class': LaunchConfiguration('target_class'),
            'device': LaunchConfiguration('device'),
            'half_precision': ParameterValue(LaunchConfiguration('half_precision'), value_type=bool),
        }],
        output='screen')

    heartbeat = Node(
        package='dronetrack_groundstation', executable='heartbeat_node',
        name='groundstation_heartbeat_node', parameters=[params], output='screen')

    dashboard = Node(
        package='dronetrack_web_bridge', executable='web_dashboard_node',
        name='web_dashboard_node',
        parameters=[params, {'port': ParameterValue(LaunchConfiguration('dashboard_port'), value_type=int)}],
        output='screen', emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('dashboard')))

    return LaunchDescription([
        params_file_arg, model_path_arg, target_class_arg, device_arg,
        half_precision_arg, dashboard_arg, dashboard_port_arg,
        LogInfo(msg='=== DRONETRACK GROUND STATION (laptop) ==='),
        LogInfo(msg=['Dashboard: http://127.0.0.1:', LaunchConfiguration('dashboard_port'), '/  (use 127.0.0.1 from Windows/WSL2)']),
        LogInfo(msg='Publishes detections + heartbeat to the Pi; the Pi validates everything.'),
        yolo,
        heartbeat,
        dashboard,
    ])
