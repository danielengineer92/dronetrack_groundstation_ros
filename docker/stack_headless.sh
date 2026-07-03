#!/usr/bin/env bash
# Launch the DroneTrack ROS 2 stack INSIDE the sim container against a mission
# plan. Logs to /tmp/stack.log.
#
# Usage (from the host):
#   docker exec -d dronetrack-sim bash -lc 'setsid /tmp/stack_launch.sh orbit_red_ball </dev/null >/tmp/stack_launch.out 2>&1 &'
set -e
PLAN="${1:-scan_and_orbit}"; shift || true
REPO="$HOME/dronetrack_groundstation_ros"

# Single-instance guard. Killing only the `ros2 launch` process orphans its
# node children, and every relaunch then stacks another full set of nodes —
# N executors/telemetry bridges all commanding the same drone (observed: 4
# copies, phantom LAND/DO_ORBIT fights). Kill launch AND any surviving nodes.
NODE_PAT='mission_executor_node|telemetry_node|control_node|tracker_node|autonomy_manager_node|yolo_node|target_mover_node|web_dashboard_node|health_monitor_node|camera_info|image_bridge|parameter_bridge'
pkill -INT -f "[r]os2 launch dronetrack_pi sitl_" 2>/dev/null || true
sleep 5   # let launch propagate SIGINT and reap its children
pkill -TERM -f "$NODE_PAT" 2>/dev/null || true
sleep 2
pkill -KILL -f "$NODE_PAT" 2>/dev/null || true
pkill -KILL -f "[r]os2 launch dronetrack_pi sitl_" 2>/dev/null || true
MODEL="$REPO/models/red_ball_yolo11s.pt"
PLANFILE="$REPO/ros_ws/src/drone_control/missions/${PLAN}.yaml"
rm -f /tmp/stack.log
cd "$REPO"
exec bash scripts/ros_wsl.sh gazebo \
  device:=cuda:0 \
  model_path:="$MODEL" \
  mission_plan_file:="$PLANFILE" \
  max_fps:=120.0 \
  ball_radius:=0.0 \
  "$@" > /tmp/stack.log 2>&1
