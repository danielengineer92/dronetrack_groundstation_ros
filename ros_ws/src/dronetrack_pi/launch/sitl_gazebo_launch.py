"""PX4 SITL launch with REAL VISION from a Gazebo camera.

Replaces the fake_detection_node from sitl_mission_launch.py with:
  1. ros_gz_bridge — bridges the Gazebo camera sensor into ROS 2
  2. gz_cam_republisher — compresses Gazebo images onto the standard
     /drone/camera/image_raw/compressed topic
  3. yolo_node — runs YOLO inference on the compressed stream
  4. detection_gate_node — validates detections (same as real-hardware path)
  5. heartbeat_node — ground-station heartbeat for the watchdog

Everything else (tracker, control, mission executor, telemetry, dashboard)
runs identically to the real split-architecture stack.

Gazebo camera topic:
  PX4 SITL models publish camera images via Gazebo transport. The
  ros_gz_bridge bridges them into ROS 2. The default Gazebo topic is
  set to match the x500_mono_cam model:
    /world/default/model/x500_mono_cam/link/camera_link/sensor/camera/image

  Override with gz_camera_topic:= if your model differs.

Usage (after PX4 SITL + Gazebo are running):
  scripts/ros_wsl.sh gazebo
  scripts/ros_wsl.sh gazebo device:=cuda:0 model_path:=yolov8s.pt
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pi_share = get_package_share_directory("dronetrack_pi")
    gs_share = get_package_share_directory("dronetrack_groundstation")

    try:
        control_share = get_package_share_directory("drone_control")
        default_plan = os.path.join(control_share, "missions", "scan_and_orbit.yaml")
    except Exception:
        default_plan = ""

    default_pi_params = os.path.join(pi_share, "config", "pi.yaml")
    default_gs_params = os.path.join(gs_share, "config", "groundstation.yaml")

    # ---- Launch arguments ----
    params_file_arg = DeclareLaunchArgument(
        "params_file", default_value=default_pi_params,
        description="Pi-side params YAML.")
    gs_params_file_arg = DeclareLaunchArgument(
        "gs_params_file", default_value=default_gs_params,
        description="Ground-station params YAML.")
    mission_plan_file_arg = DeclareLaunchArgument(
        "mission_plan_file", default_value=default_plan,
        description="Mission YAML for mission_executor_node.")
    connection_url_arg = DeclareLaunchArgument(
        "connection_url", default_value="udp://:14540",
        description="MAVSDK connection URL for PX4 SITL.")
    allow_mavsdk_actions_arg = DeclareLaunchArgument(
        "allow_mavsdk_actions", default_value="true",
        description="SITL: allow TAKEOFF/ORBIT/LAND actions.")
    allow_translation_commands_arg = DeclareLaunchArgument(
        "allow_translation_commands", default_value="false",
        description="SITL: allow velocity/translation commands.")
    enable_approach_translation_arg = DeclareLaunchArgument(
        "enable_approach_translation", default_value="false",
        description="Let control_node generate forward approach velocity.")
    allow_scan_without_lock_arg = DeclareLaunchArgument(
        "allow_scan_without_lock", default_value="false",
        description="Allow yaw-only SCAN while target is not locked.")
    auto_start_arg = DeclareLaunchArgument(
        "auto_start", default_value="false",
        description="Start mission immediately (skip waiting for /drone/mission/request).")
    target_class_arg = DeclareLaunchArgument(
        "target_class", default_value="",
        description="YOLO target class filter. Empty = all COCO classes.")
    dashboard_arg = DeclareLaunchArgument(
        "dashboard", default_value="true",
        description="Run the web dashboard.")
    dashboard_port_arg = DeclareLaunchArgument(
        "dashboard_port", default_value="8091",
        description="HTTP port for the SITL dashboard.")

    # ---- Gazebo camera arguments ----
    gz_camera_topic_arg = DeclareLaunchArgument(
        "gz_camera_topic",
        default_value="/world/default/model/x500_mono_cam/link/camera_link/sensor/camera/image",
        description="Gazebo transport camera topic. Override to match your PX4 model.")
    gz_ros_image_topic_arg = DeclareLaunchArgument(
        "gz_ros_image_topic", default_value="/sim/camera/image_raw",
        description="ROS 2 topic for the bridged Gazebo camera (intermediate).")

    # ---- YOLO arguments ----
    model_path_arg = DeclareLaunchArgument(
        "model_path", default_value="yolov8n.pt",
        description="YOLO model path.")
    device_arg = DeclareLaunchArgument(
        "device", default_value="cpu",
        description="YOLO inference device: cpu, cuda:0, mps.")
    half_precision_arg = DeclareLaunchArgument(
        "half_precision", default_value="false",
        description="FP16 inference (use with cuda).")
    max_fps_arg = DeclareLaunchArgument(
        "max_fps", default_value="15.0",
        description="Max YOLO processing FPS.")

    params = LaunchConfiguration("params_file")
    gs_params = LaunchConfiguration("gs_params_file")

    # ---- ros_gz_bridge: Gazebo camera → ROS 2 ----
    gz_bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            [LaunchConfiguration("gz_camera_topic"),
             "@sensor_msgs/msg/Image[gz.msgs.Image"],
            "--ros-args",
            "-r", ["__node:=gz_camera_bridge"],
            "-r", [LaunchConfiguration("gz_camera_topic"),
                   ":=", LaunchConfiguration("gz_ros_image_topic")],
        ],
        output="screen",
    )

    # ---- gz_cam_republisher: raw Gazebo → compressed ----
    gz_cam_republisher = Node(
        package="dronetrack_perception",
        executable="gz_cam_republisher",
        name="gz_cam_republisher",
        parameters=[{
            "gz_image_topic": LaunchConfiguration("gz_ros_image_topic"),
            "compressed_out_topic": "/drone/camera/image_raw/compressed",
            "raw_out_topic": "/drone/camera/image_raw",
            "republish_raw": True,
            "jpeg_quality": 80,
        }],
        output="screen",
    )

    # ---- YOLO: ground-station perception ----
    yolo = Node(
        package="dronetrack_perception",
        executable="yolo_node",
        name="yolo_node",
        parameters=[gs_params, {
            "model_path": LaunchConfiguration("model_path"),
            "target_class": LaunchConfiguration("target_class"),
            "device": LaunchConfiguration("device"),
            "half_precision": ParameterValue(LaunchConfiguration("half_precision"), value_type=bool),
            "max_fps": ParameterValue(LaunchConfiguration("max_fps"), value_type=float),
        }],
        output="screen",
    )

    # ---- Detection gate: validates YOLO output before the tracker sees it ----
    detection_gate = Node(
        package="dronetrack_pi",
        executable="detection_gate_node",
        name="detection_gate_node",
        parameters=[params, {
            "require_heartbeat": True,
        }],
        output="screen",
    )

    # ---- Heartbeat: keeps the detection gate and watchdog happy ----
    heartbeat = Node(
        package="dronetrack_groundstation",
        executable="heartbeat_node",
        name="groundstation_heartbeat_node",
        parameters=[gs_params],
        output="screen",
    )

    # ---- Pi-side mission stack (identical to real hardware) ----
    telemetry = Node(
        package="drone_telemetry",
        executable="telemetry_node",
        name="telemetry_node",
        parameters=[params, {
            "connection_url": LaunchConfiguration("connection_url"),
            "allow_mavsdk_actions": ParameterValue(LaunchConfiguration("allow_mavsdk_actions"), value_type=bool),
            "allow_translation_commands": ParameterValue(LaunchConfiguration("allow_translation_commands"), value_type=bool),
        }],
        output="screen",
        emulate_tty=True,
    )

    tracker = Node(
        package="drone_tracker",
        executable="tracker_node",
        name="tracker_node",
        parameters=[params, {"target_class": LaunchConfiguration("target_class")}],
        output="screen",
        emulate_tty=True,
    )

    autonomy_manager = Node(
        package="drone_control",
        executable="autonomy_manager_node",
        name="autonomy_manager_node",
        parameters=[params, {
            "allow_scan_without_lock": ParameterValue(LaunchConfiguration("allow_scan_without_lock"), value_type=bool),
        }],
        output="screen",
        emulate_tty=True,
    )

    control = Node(
        package="drone_control",
        executable="control_node",
        name="control_node",
        parameters=[params, {
            "enable_approach_translation": ParameterValue(LaunchConfiguration("enable_approach_translation"), value_type=bool),
        }],
        output="screen",
        emulate_tty=True,
    )

    mission_executor = Node(
        package="drone_control",
        executable="mission_executor_node",
        name="mission_executor_node",
        parameters=[params, {
            "mission_enabled": True,
            "auto_start": ParameterValue(LaunchConfiguration("auto_start"), value_type=bool),
            "mission_plan_file": LaunchConfiguration("mission_plan_file"),
            "require_distance_for_orbit": False,
            "require_target_centered_for_orbit": False,
        }],
        output="screen",
        emulate_tty=True,
    )

    health = Node(
        package="drone_diagnostics",
        executable="health_monitor_node",
        name="health_monitor_node",
        parameters=[params],
        output="screen",
        emulate_tty=True,
    )

    ground_station_watchdog = Node(
        package="dronetrack_pi",
        executable="ground_station_watchdog_node",
        name="ground_station_watchdog_node",
        parameters=[params],
        output="screen",
    )

    dashboard = Node(
        package="dronetrack_web_bridge",
        executable="web_dashboard_node",
        name="web_dashboard_node",
        parameters=[gs_params, {"port": ParameterValue(LaunchConfiguration("dashboard_port"), value_type=int)}],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("dashboard")),
    )

    return LaunchDescription([
        # Arguments
        params_file_arg,
        gs_params_file_arg,
        mission_plan_file_arg,
        connection_url_arg,
        allow_mavsdk_actions_arg,
        allow_translation_commands_arg,
        enable_approach_translation_arg,
        allow_scan_without_lock_arg,
        auto_start_arg,
        target_class_arg,
        dashboard_arg,
        dashboard_port_arg,
        gz_camera_topic_arg,
        gz_ros_image_topic_arg,
        model_path_arg,
        device_arg,
        half_precision_arg,
        max_fps_arg,

        # Info
        LogInfo(msg="=== DRONETRACK GAZEBO SITL — REAL VISION ==="),
        LogInfo(msg="Gazebo camera → YOLO → detection gate → tracker → control → PX4"),
        LogInfo(msg=["Dashboard: http://127.0.0.1:", LaunchConfiguration("dashboard_port"), "/"]),
        LogInfo(msg=["PX4 connection: ", LaunchConfiguration("connection_url")]),
        LogInfo(msg=["YOLO model: ", LaunchConfiguration("model_path"), " on ", LaunchConfiguration("device")]),
        LogInfo(msg=["Gazebo camera topic: ", LaunchConfiguration("gz_camera_topic")]),

        # Nodes
        gz_bridge,
        gz_cam_republisher,
        yolo,
        heartbeat,
        detection_gate,
        ground_station_watchdog,
        telemetry,
        tracker,
        autonomy_manager,
        control,
        mission_executor,
        health,
        dashboard,
    ])
