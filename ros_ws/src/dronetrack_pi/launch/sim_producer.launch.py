"""SIM PRODUCER — run on the machine that renders Gazebo (e.g. the 5080 PC).

Producer-only half of the split setup: it owns the camera + the moving ball and
publishes the camera over the LAN as /sim/camera/image_raw. It does NOT run YOLO
or the mission stack — the laptop does that (sitl_gazebo_launch.py local_sim:=false),
so there are no duplicate nodes on the shared ROS domain.

Brings up:
  1. ros_gz_bridge  — gz camera sensor -> /sim/camera/image_raw, plus the
                      /world/<world>/set_pose service (config generated at launch)
  2. ros_gz_sim create — spawns red_ball into the running world
  3. target_mover_node — orbits the ball via SetEntityPose

Run AFTER PX4 SITL + Gazebo are up:
  cd ~/PX4-Autopilot && make px4_sitl gz_x500_mono_cam      # starts gz + PX4 + camera
  ros2 launch dronetrack_pi sim_producer.launch.py

For a single-machine setup (gz + YOLO + stack all here) use sitl_gazebo_launch.py
instead. See docs/gazebo_sitl.md.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_ball_sdf():
    """Find red_ball.sdf shipped in dronetrack_perception (installed or source)."""
    try:
        share = get_package_share_directory("dronetrack_perception")
        cand = os.path.join(share, "models", "red_ball.sdf")
        if os.path.isfile(cand):
            return cand
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    for rel in ["../../../../models/red_ball.sdf", "../../../models/red_ball.sdf"]:
        cand = os.path.normpath(os.path.join(here, rel))
        if os.path.isfile(cand):
            return cand
    return ""


def _bridge_and_spawn(context, *args, **kwargs):
    world = LaunchConfiguration("gz_world_name").perform(context)
    cam_gz = LaunchConfiguration("gz_camera_topic").perform(context)
    cam_ros = LaunchConfiguration("gz_ros_image_topic").perform(context)
    ball_sdf = LaunchConfiguration("ball_sdf").perform(context)

    bf = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml",
                                     prefix="sim_producer_bridge_", delete=False)
    bf.write(
        f"- ros_topic_name: \"{cam_ros}\"\n"
        f"  gz_topic_name: \"{cam_gz}\"\n"
        f"  ros_type_name: \"sensor_msgs/msg/Image\"\n"
        f"  gz_type_name: \"gz.msgs.Image\"\n"
        f"  direction: GZ_TO_ROS\n\n"
        f"- service_name: \"/world/{world}/set_pose\"\n"
        f"  ros_type_name: \"ros_gz_interfaces/srv/SetEntityPose\"\n"
        f"  gz_req_type_name: \"gz.msgs.Pose\"\n"
        f"  gz_rep_type_name: \"gz.msgs.Boolean\"\n"
        f"  direction: ROS_TO_GZ\n"
    )
    bf.flush()
    cfg = bf.name
    bf.close()

    actions = [Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
        parameters=[{"config_file": cfg}], output="screen",
    )]

    if ball_sdf and os.path.isfile(ball_sdf):
        actions.append(TimerAction(period=2.0, actions=[
            LogInfo(msg=["Spawning red_ball from ", ball_sdf]),
            ExecuteProcess(
                cmd=["ros2", "run", "ros_gz_sim", "create",
                     "-name", "red_ball", "-file", ball_sdf,
                     "-x", "5", "-y", "0", "-z", "1", "-world", world],
                output="screen"),
        ]))
    else:
        actions.append(LogInfo(
            msg=f"WARNING: red_ball.sdf not found ('{ball_sdf}'); set ball_sdf:= manually."))
    return actions


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("gz_world_name", default_value="default"),
        DeclareLaunchArgument(
            "gz_camera_topic",
            default_value="/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image",
            description="Gazebo transport camera topic — match your PX4 model."),
        DeclareLaunchArgument("gz_ros_image_topic", default_value="/sim/camera/image_raw"),
        DeclareLaunchArgument("ball_sdf", default_value=_resolve_ball_sdf()),
        DeclareLaunchArgument("ball_center_x", default_value="5.0"),
        DeclareLaunchArgument("ball_center_y", default_value="0.0"),
        DeclareLaunchArgument("ball_altitude", default_value="1.0"),
        DeclareLaunchArgument("ball_radius", default_value="3.0"),
        DeclareLaunchArgument("ball_period", default_value="20.0"),
    ]

    target_mover = TimerAction(period=4.0, actions=[Node(
        package="dronetrack_perception", executable="target_mover_node",
        name="target_mover_node",
        parameters=[{
            "world_name": LaunchConfiguration("gz_world_name"),
            "entity_name": "red_ball",
            "center_x": ParameterValue(LaunchConfiguration("ball_center_x"), value_type=float),
            "center_y": ParameterValue(LaunchConfiguration("ball_center_y"), value_type=float),
            "altitude": ParameterValue(LaunchConfiguration("ball_altitude"), value_type=float),
            "radius": ParameterValue(LaunchConfiguration("ball_radius"), value_type=float),
            "period_s": ParameterValue(LaunchConfiguration("ball_period"), value_type=float),
            "rate_hz": 20.0,
        }],
        output="screen")])

    return LaunchDescription([
        *args,
        LogInfo(msg="=== SIM PRODUCER — camera + moving red ball (no stack) ==="),
        LogInfo(msg=["Publishing /sim/camera/image_raw from ",
                      LaunchConfiguration("gz_camera_topic")]),
        OpaqueFunction(function=_bridge_and_spawn),
        target_mover,
    ])
