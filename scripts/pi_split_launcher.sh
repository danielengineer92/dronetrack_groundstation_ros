#!/bin/bash
# Canonical launcher for the split-architecture Pi stack. This file is deployed to
# the Pi (~/run_pi_split.sh) by scripts/up.sh and started with `setsid` so it keeps
# running even if the SSH connection that launched it drops (flaky Wi-Fi).
#
# Env overrides (set before invoking):
#   PI_CONN           MAVSDK connection URL   (default serial:///dev/ttyACM0:57600)
#   PI_ALLOW_ACTIONS  allow MAVSDK actions    (default false -- keep false on the bench)
#   PI_NATIVE_MJPEG   use native MJPEG camera (default true)
set -eo pipefail  # not -u: ROS setup.bash references unbound vars

cd "$HOME/drone_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# IP-pinned unicast config, kept current by the laptop's discover_ips.sh.
export CYCLONEDDS_URI="file://$HOME/cyclonedds_pi.xml"

exec ros2 launch dronetrack_pi pi_launch.py \
  connection_url:="${PI_CONN:-serial:///dev/ttyACM0:57600}" \
  allow_mavsdk_actions:="${PI_ALLOW_ACTIONS:-false}" \
  native_mjpeg:="${PI_NATIVE_MJPEG:-true}"
