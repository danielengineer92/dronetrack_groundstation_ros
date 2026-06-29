"""Move the red_ball model in a circle inside the running Gazebo world.

Calls the Gazebo /world/<name>/set_pose service (bridged to ROS via
ros_gz_bridge) at a configurable rate to drive the ball on a circular
orbit: x = cx + r*cos(wt), y = cy + r*sin(wt), z = altitude.

The node waits for the service to become available before starting the
motion loop, so launch order does not matter.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from rclpy.node import Node

try:
    from ros_gz_interfaces.srv import SetEntityPose
    from ros_gz_interfaces.msg import Entity
    _HAS_GZ_INTERFACES = True
except ImportError:
    _HAS_GZ_INTERFACES = False


class TargetMoverNode(Node):
    def __init__(self) -> None:
        super().__init__("target_mover_node")

        self.declare_parameter("world_name", "default")
        self.declare_parameter("entity_name", "red_ball")
        self.declare_parameter("center_x", 5.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("altitude", 1.0)
        self.declare_parameter("radius", 3.0)
        self.declare_parameter("period_s", 20.0)
        self.declare_parameter("rate_hz", 20.0)

        self.world_name = str(self.get_parameter("world_name").value)
        self.entity_name = str(self.get_parameter("entity_name").value)
        self.cx = float(self.get_parameter("center_x").value)
        self.cy = float(self.get_parameter("center_y").value)
        self.alt = float(self.get_parameter("altitude").value)
        self.radius = float(self.get_parameter("radius").value)
        period = max(1.0, float(self.get_parameter("period_s").value))
        self.omega = 2.0 * math.pi / period
        rate = max(1.0, float(self.get_parameter("rate_hz").value))

        if not _HAS_GZ_INTERFACES:
            self.get_logger().error(
                "ros_gz_interfaces not found. Install: "
                "sudo apt install ros-jazzy-ros-gz-interfaces"
            )
            return

        service_name = f"/world/{self.world_name}/set_pose"
        self.client = self.create_client(SetEntityPose, service_name)
        self.start_time = time.monotonic()
        self.calls_sent = 0
        self.calls_ok = 0

        self.get_logger().info(
            f"Target mover up | entity={self.entity_name}, "
            f"service={service_name}, "
            f"orbit r={self.radius:.1f}m period={period:.0f}s "
            f"center=({self.cx:.1f}, {self.cy:.1f}, {self.alt:.1f}), "
            f"rate={rate:.0f} Hz"
        )

        self.get_logger().info(f"Waiting for service {service_name} ...")
        self.wait_timer = self.create_timer(1.0, self._wait_for_service)
        self.move_timer = None

        self.report_timer = self.create_timer(10.0, self._report)
        self._move_dt = 1.0 / rate

    def _wait_for_service(self) -> None:
        if self.client.service_is_ready():
            self.wait_timer.cancel()
            self.get_logger().info("set_pose service ready — starting motion loop")
            self.start_time = time.monotonic()
            self.move_timer = self.create_timer(self._move_dt, self._move)
        else:
            self.get_logger().info(
                "Waiting for set_pose service (is ros_gz_bridge running?) ...",
                throttle_duration_sec=5.0,
            )

    def _move(self) -> None:
        t = time.monotonic() - self.start_time
        x = self.cx + self.radius * math.cos(self.omega * t)
        y = self.cy + self.radius * math.sin(self.omega * t)

        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = self.entity_name
        req.entity.type = 2  # MODEL
        req.pose = Pose(
            position=Point(x=x, y=y, z=self.alt),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )

        future = self.client.call_async(req)
        future.add_done_callback(self._on_result)
        self.calls_sent += 1

    def _on_result(self, future) -> None:
        try:
            resp = future.result()
            if resp and resp.success:
                self.calls_ok += 1
        except Exception as exc:
            self.get_logger().warning(
                f"set_pose call failed: {exc}", throttle_duration_sec=5.0
            )

    def _report(self) -> None:
        self.get_logger().info(
            f"Target mover | sent={self.calls_sent}, ok={self.calls_ok}, "
            f"entity={self.entity_name}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetMoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
