#!/usr/bin/env bash
# Hardware-free self-test of the Pi safety boundary (no PX4, no YOLO, no camera).
#
# Brings up the heartbeat publisher, detection gate, and ground-station watchdog,
# feeds fake detections, and asserts:
#   PHASE 1 (healthy link): gate republishes detections, link_ok=true.
#   PHASE 2 (link lost):    link_ok=false / HEARTBEAT_STALE, requests de-asserted.
#
# Usage (after building):
#   bash scripts/selftest_boundary.sh
# Override the built workspace install dir if you built elsewhere:
#   INSTALL_DIR=~/dronetrack_gs/install bash scripts/selftest_boundary.sh
#
# NOTE: this script must be run as a FILE (not `bash -c "<inline>"`), so that the
# orchestrator's argv stays short and `pkill -f <node>` matches only the real
# node processes, never this script.
# Note: no `set -u` — ROS setup.bash references unbound vars and would trip it.
set -o pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
INSTALL_DIR="${INSTALL_DIR:-${REPO_ROOT}/ros_ws/install}"
[ -f "${INSTALL_DIR}/setup.bash" ] || INSTALL_DIR="${HOME}/dronetrack_gs/install"
DOMAIN="${ROS_DOMAIN_ID:-42}"

# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1091
source "${INSTALL_DIR}/setup.bash"
export ROS_DOMAIN_ID="${DOMAIN}"
echo "Using install: ${INSTALL_DIR} | ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

L="$(mktemp -d)"; PUB="${L}/fake_pub.py"
cleanup() {
  pkill -9 -f detection_gate_node 2>/dev/null
  pkill -9 -f ground_station_watchdog_node 2>/dev/null
  pkill -9 -f heartbeat_node 2>/dev/null
  pkill -9 -f "${PUB}" 2>/dev/null
}
trap cleanup EXIT

cat > "${PUB}" << 'PY'
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from drone_interfaces.msg import Detection, DetectionArray

class P(Node):
    def __init__(self):
        super().__init__("selftest_fake_pub")
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(DetectionArray, "/groundstation/vision/detections", q)
        self.create_timer(0.1, self.tick)
    def tick(self):
        now = self.get_clock().now().to_msg()
        a = DetectionArray(); a.stamp = now; a.image_width = 640; a.image_height = 480
        d = Detection(); d.stamp = now; d.class_name = "red_ball"; d.class_id = 0; d.confidence = 0.9
        d.center_x = 0.5; d.center_y = 0.5; d.width = 0.1; d.height = 0.1
        d.pixel_center_x = 320; d.pixel_center_y = 240; d.pixel_width = 64; d.pixel_height = 48
        a.detections = [d]; a.count = 1
        self.pub.publish(a)

rclpy.init(); rclpy.spin(P())
PY

cleanup; sleep 1
ros2 run dronetrack_groundstation heartbeat_node > "${L}/hb.log" 2>&1 &
ros2 run dronetrack_pi detection_gate_node > "${L}/gate.log" 2>&1 &
ros2 run dronetrack_pi ground_station_watchdog_node > "${L}/wd.log" 2>&1 &
python3 "${PUB}" > "${L}/pub.log" 2>&1 &
ros2 topic echo /drone/autonomy/request std_msgs/msg/Bool > "${L}/areq.log" 2>&1 &
sleep 5

fail=0
pass() { echo "  PASS: $1"; }
fail() { echo "  FAIL: $1"; fail=1; }
read_link_status() {
  local out="" i
  for i in 1 2 3 4 5; do
    out="$(timeout 8 ros2 topic echo --once --qos-reliability reliable --qos-history keep_last --qos-depth 5 /drone/groundstation/link_status 2>&1)"
    if echo "${out}" | grep -q "^link_ok:"; then
      echo "${out}"
      return 0
    fi
    sleep 1
  done
  echo "${out}"
  return 1
}

echo "PHASE 1 (healthy link):"
hz1="$(timeout 4 ros2 topic hz /drone/vision/detections 2>&1 | grep -oE 'average rate: [0-9.]+' | head -1)"
[ -n "${hz1}" ] && pass "gate republishes trusted detections (${hz1})" || fail "no trusted output on /drone/vision/detections"
ls1="$(read_link_status)"
echo "${ls1}" | grep -q "link_ok: true" && pass "link_ok=true" || fail "link_ok not true"
echo "${ls1}" | grep -q "reason: OK" && pass "reason=OK" || fail "reason not OK"

echo "PHASE 2 (kill heartbeat + publisher):"
pkill -9 -f heartbeat_node; pkill -9 -f "${PUB}"
sleep 4
ls2="$(read_link_status)"
echo "${ls2}" | grep -q "link_ok: false" && pass "link_ok=false" || fail "link_ok not false"
echo "${ls2}" | grep -q "reason: HEARTBEAT_STALE" && pass "reason=HEARTBEAT_STALE" || fail "reason not HEARTBEAT_STALE"
grep -q "data: false" "${L}/areq.log" && pass "autonomy/request de-asserted to false" || fail "autonomy/request not de-asserted"
grep -q "LINK LOST" "${L}/wd.log" && pass "watchdog logged LINK LOST" || fail "no LINK LOST in watchdog log"

echo
if [ "${fail}" -eq 0 ]; then echo "SELFTEST: ALL PASS"; else echo "SELFTEST: FAILURES ABOVE"; fi
exit "${fail}"
