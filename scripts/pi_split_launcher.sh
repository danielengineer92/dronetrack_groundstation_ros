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

# `ros2 launch` runs node console-scripts under the SYSTEM python3 (their
# shebang), which cannot see the venv where mavsdk lives. Without this the
# telemetry bridge imports nothing and reports MAVSDK_NOT_INSTALLED. Put the
# venv's site-packages on PYTHONPATH so the nodes can import it. Same technique
# as the sim runner (scripts/ros_wsl.sh).
PI_VENV="${PI_VENV:-$HOME/venv}"
if [ -d "${PI_VENV}" ]; then
  _vsp="$(ls -d "${PI_VENV}"/lib/python*/site-packages 2>/dev/null | head -1)"
  [ -n "${_vsp}" ] && export PYTHONPATH="${_vsp}${PYTHONPATH:+:${PYTHONPATH}}"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# IP-pinned unicast config, kept current by the laptop's discover_ips.sh.
export CYCLONEDDS_URI="file://$HOME/cyclonedds_pi.xml"

# CPU split on the Pi 4: the whole stack lives on cores 0-1 so the local ncnn
# YOLO (pi_launch gives it `taskset -c 2,3`) gets two uncontended cores.
# Measured 2026-07-05: yolo inference 500-730 ms with everything sharing all 4
# cores vs ~117 ms uncontended — the stack starves ncnn without this.
exec taskset -c 0,1 ros2 launch dronetrack_pi pi_launch.py \
  connection_url:="${PI_CONN:-serial:///dev/ttyACM0:57600}" \
  allow_mavsdk_actions:="${PI_ALLOW_ACTIONS:-false}" \
  native_mjpeg:="${PI_NATIVE_MJPEG:-true}" \
  local_yolo_model:="${PI_YOLO_MODEL:-$HOME/models/red_ball_ncnn_model}" \
  local_yolo_imgsz:="${PI_YOLO_IMGSZ:-320}"
