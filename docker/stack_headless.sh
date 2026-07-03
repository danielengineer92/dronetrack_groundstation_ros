#!/usr/bin/env bash
# Launch the DroneTrack ROS 2 stack INSIDE the sim container against a mission
# plan. Logs to /tmp/stack.log.
#
# Usage (from the host):
#   docker exec -d dronetrack-sim bash -lc 'setsid /tmp/stack_launch.sh orbit_red_ball </dev/null >/tmp/stack_launch.out 2>&1 &'
set -e
PLAN="${1:-scan_and_orbit}"; shift || true
REPO="$HOME/dronetrack_groundstation_ros"
MODEL="$REPO/models/red_ball_yolo11s.pt"
PLANFILE="$REPO/ros_ws/src/drone_control/missions/${PLAN}.yaml"
rm -f /tmp/stack.log
cd "$REPO"
exec bash scripts/ros_wsl.sh gazebo \
  device:=cuda:0 \
  model_path:="$MODEL" \
  mission_plan_file:="$PLANFILE" \
  max_fps:=120.0 \
  "$@" > /tmp/stack.log 2>&1
