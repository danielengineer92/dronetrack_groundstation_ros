"""PX4 SITL launch for the current DroneTrack mission stack.

This launch is intentionally owned by this repo. It can reuse sim-only helper
packages from an older underlay (for example ``drone_fake``), but the
safety-critical mission/autonomy/control/telemetry nodes come from the current
workspace overlay that ``scripts/ros_wsl.sh`` builds.
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
    pi_share = get_package_share_directory("dronetrack_pi")
    control_share = get_package_share_directory("drone_control")
    gs_share = get_package_share_directory("dronetrack_groundstation")

    default_pi_params = os.path.join(pi_share, "config", "pi.yaml")
    default_gs_params = os.path.join(gs_share, "config", "groundstation.yaml")
    default_plan = os.path.join(control_share, "missions", "scan_and_orbit.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_pi_params,
        description="Pi-side params YAML for SITL nodes.",
    )
    dashboard_params_file_arg = DeclareLaunchArgument(
        "dashboard_params_file",
        default_value=default_gs_params,
        description="Ground-station params YAML for the SITL dashboard.",
    )
    mission_plan_file_arg = DeclareLaunchArgument(
        "mission_plan_file",
        default_value=default_plan,
        description="Mission YAML loaded by mission_executor_node.",
    )
    connection_url_arg = DeclareLaunchArgument(
        "connection_url",
        default_value="udp://:14540",
        description="MAVSDK connection URL for PX4 SITL.",
    )
    allow_mavsdk_actions_arg = DeclareLaunchArgument(
        "allow_mavsdk_actions",
        default_value="true",
        description="SITL only: allow TAKEOFF/ORBIT/RTL/LAND through the MAVSDK action gate.",
    )
    allow_translation_commands_arg = DeclareLaunchArgument(
        "allow_translation_commands",
        default_value="false",
        description="SITL only: allow velocity/translation commands through telemetry_node.",
    )
    enable_approach_translation_arg = DeclareLaunchArgument(
        "enable_approach_translation",
        default_value="false",
        description="SITL only: let control_node generate forward approach velocity.",
    )
    allow_scan_without_lock_arg = DeclareLaunchArgument(
        "allow_scan_without_lock",
        default_value="false",
        description="SITL/dev: allow yaw-only SCAN while the target is not locked.",
    )
    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="false",
        description="Start the mission immediately after launch instead of waiting for /drone/mission/request.",
    )
    target_class_arg = DeclareLaunchArgument(
        "target_class",
        default_value="red_ball",
        description="Tracker target class.",
    )
    fake_target_class_arg = DeclareLaunchArgument(
        "fake_target_class",
        default_value="red_ball",
        description="Fake detection class. Set different from target_class to simulate no lock.",
    )
    motion_pattern_arg = DeclareLaunchArgument(
        "motion_pattern",
        default_value="stationary",
        description="Fake target motion pattern.",
    )
    dashboard_arg = DeclareLaunchArgument(
        "dashboard",
        default_value="true",
        description="Run the repo web dashboard.",
    )
    dashboard_port_arg = DeclareLaunchArgument(
        "dashboard_port",
        default_value="8091",
        description="HTTP port for the SITL dashboard.",
    )

    params = LaunchConfiguration("params_file")
    dashboard_params = LaunchConfiguration("dashboard_params_file")

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
    fake_detection = Node(
        package="drone_fake",
        executable="fake_detection_node",
        name="fake_detection_node",
        parameters=[{
            "detections_topic": "/drone/vision/detections",
            "target_class": LaunchConfiguration("fake_target_class"),
            "motion_pattern": LaunchConfiguration("motion_pattern"),
            "publish_rate": 30.0,
            "detection_dropout_rate": 0.0,
            "add_false_detections": False,
        }],
        output="screen",
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
    dashboard = Node(
        package="dronetrack_web_bridge",
        executable="web_dashboard_node",
        name="web_dashboard_node",
        parameters=[dashboard_params, {"port": ParameterValue(LaunchConfiguration("dashboard_port"), value_type=int)}],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("dashboard")),
    )

    return LaunchDescription([
        params_file_arg,
        dashboard_params_file_arg,
        mission_plan_file_arg,
        connection_url_arg,
        allow_mavsdk_actions_arg,
        allow_translation_commands_arg,
        enable_approach_translation_arg,
        allow_scan_without_lock_arg,
        auto_start_arg,
        target_class_arg,
        fake_target_class_arg,
        motion_pattern_arg,
        dashboard_arg,
        dashboard_port_arg,
        LogInfo(msg="=== DRONETRACK CURRENT-REPO PX4 SITL MISSION ==="),
        LogInfo(msg=["Dashboard: http://127.0.0.1:", LaunchConfiguration("dashboard_port"), "/"]),
        LogInfo(msg=["PX4 connection: ", LaunchConfiguration("connection_url")]),
        LogInfo(msg=["Mission plan: ", LaunchConfiguration("mission_plan_file")]),
        LogInfo(msg="SITL launch enables MAVSDK actions by default; hardware configs remain locked down."),
        telemetry,
        fake_detection,
        tracker,
        autonomy_manager,
        control,
        mission_executor,
        health,
        dashboard,
    ])
