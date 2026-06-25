#!/usr/bin/env bash
# Run the PX4 SITL orbit mission on the laptop (no Pi, no real airframe).
#
# Flies the full autonomy chain against a PX4 SITL: faked vision -> tracker lock
# -> autonomy gate -> offboard control -> mission_executor (orbit_red_ball.yaml)
# -> MAVSDK action gate -> PX4. Uses a loopback-only CycloneDDS config + an
# isolated ROS domain so it never touches the live split stack.
#
# PREREQS:
#   1. PX4 SITL running and offering MAVLink on udp 14540, e.g. via Docker:
#        sudo apt install -y docker.io && sudo service docker start
#        sudo docker run --rm -it --network host px4io/px4-sitl:latest
#   2. MAVSDK python installed:  python3 -m pip install --user --break-system-packages mavsdk
#   3. The sim packages built into ~/dronetrack_sim (this script prints the build
#      command if they're missing).
#
# USAGE:
#   bash scripts/sim_sitl.sh                 # launch the mission stack
#   # then: arm PX4 in its pxh> shell:  commander arm
#   # then: open http://127.0.0.1:8091/ and click System Ready -> Start Mission
#   #   (or from a shell with the same ROS env as this script:
#   #      ros2 topic pub -1 /drone/autonomy/request std_msgs/msg/Bool "{data: true}"
#   #      ros2 topic pub -1 /drone/mission/request  std_msgs/msg/Bool "{data: true}")
#
# Extra args are forwarded to ros2 launch, e.g.:
#   bash scripts/sim_sitl.sh dashboard_port:=8092
set -eo pipefail  # not -u: ROS/colcon setup.bash reference unbound vars
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

SIM_INSTALL="${SIM_INSTALL:-$HOME/dronetrack_sim/install}"
SIM_DOMAIN="${SIM_DOMAIN:-9}"
SIM_DASH_PORT="${SIM_DASH_PORT:-8091}"
CONNECTION_URL="${CONNECTION_URL:-udp://:14540}"
CYCLONE_CFG="$HOME/sim_cyclonedds.xml"

if [ ! -f "${SIM_INSTALL}/setup.bash" ]; then
  echo "ERROR: sim workspace not built at ${SIM_INSTALL}"
  echo "Build it (one time) with:"
  echo "  source /opt/ros/jazzy/setup.bash"
  echo "  cd ${PI_ROS_SRC%/src}"
  echo "  colcon build --symlink-install \\"
  echo "    --packages-up-to drone_bringup drone_fake drone_visualizer drone_dashboard \\"
  echo "                     drone_tracker drone_control drone_diagnostics \\"
  echo "    --build-base ~/dronetrack_sim/build --install-base ~/dronetrack_sim/install"
  exit 1
fi

# Loopback-only CycloneDDS: reliable same-host discovery on WSL2, isolated from
# the LAN and from the live split stack (which runs on a different domain).
cat > "${CYCLONE_CFG}" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces><NetworkInterface name="lo" presence_required="false"/></Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
XML

# shellcheck disable=SC1090
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${SIM_INSTALL}/setup.bash"
export ROS_DOMAIN_ID="${SIM_DOMAIN}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONE_CFG}"

echo "SITL mission | domain=${SIM_DOMAIN}, dashboard=http://127.0.0.1:${SIM_DASH_PORT}/, mavlink=${CONNECTION_URL}"
echo "After it's up: 'commander arm' in the PX4 shell, then System Ready -> Start Mission on the dashboard."
exec ros2 launch drone_bringup sitl_orbit_launch.py \
  connection_url:="${CONNECTION_URL}" \
  dashboard:=true dashboard_port:="${SIM_DASH_PORT}" \
  "$@"
