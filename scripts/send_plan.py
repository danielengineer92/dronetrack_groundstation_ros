"""Send a mission plan YAML to /drone/mission/plan for testing."""
import sys
import rclpy
from std_msgs.msg import String

plan_file = sys.argv[1] if len(sys.argv) > 1 else "ros_ws/src/drone_control/missions/full_mission_test.yaml"

rclpy.init()
node = rclpy.create_node("plan_sender")
pub = node.create_publisher(String, "/drone/mission/plan", 10)
msg = String()
with open(plan_file) as f:
    msg.data = f.read()
pub.publish(msg)
print(f"Published {len(msg.data)} bytes to /drone/mission/plan")

executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(node)
executor.spin_once(timeout_sec=2.0)
node.destroy_node()
rclpy.shutdown()
print("Done")
