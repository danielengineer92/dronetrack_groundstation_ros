#!/usr/bin/env bash
# Robust WSL/Ubuntu entrypoint for DroneTrack ROS 2 work.
#
# Use this from WSL, or from PowerShell as:
#   wsl -e bash -lc "cd /mnt/c/Users/danie/Documents/Python/dronetrack_groundstation_ros && scripts/ros_wsl.sh doctor"
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${HERE}/_common.sh"

SIM_DOMAIN="${SIM_DOMAIN:-9}"
SIM_UNDERLAY="${SIM_UNDERLAY:-$HOME/dronetrack_sim/install}"
SIM_OVERLAY="${SIM_OVERLAY:-$HOME/dronetrack_groundstation_ros_sim/install}"
SIM_BUILD_BASE="${SIM_BUILD_BASE:-$HOME/dronetrack_groundstation_ros_sim/build}"
SIM_LOG_BASE="${SIM_LOG_BASE:-$HOME/dronetrack_groundstation_ros_sim/log}"
SIM_DASH_PORT="${SIM_DASH_PORT:-8091}"
CONNECTION_URL="${CONNECTION_URL:-udp://:14540}"
CYCLONE_CFG="${CYCLONE_CFG:-$HOME/.ros/dronetrack_sim_cyclonedds.xml}"
SIM_VENV="${SIM_VENV:-$HOME/ros_venv}"

SIM_PACKAGES=(
  dronetrack_msgs
  drone_interfaces
  drone_camera
  drone_control
  drone_diagnostics
  drone_telemetry
  drone_tracker
  dronetrack_pi
  dronetrack_groundstation
  dronetrack_web_bridge
  dronetrack_perception
)

usage() {
  cat <<'EOF'
Usage: scripts/ros_wsl.sh <command> [args]

Commands:
  doctor        Check WSL, ROS, overlays, required packages, and PX4/MAVSDK basics.
  build-sim     Build this repo into a dedicated SITL overlay.
  sitl          Build if needed, then launch the current-repo SITL mission stack.
  gazebo        Build if needed, then launch SITL with real Gazebo camera + YOLO vision.
  env [cmd...]  Source the exact SITL ROS environment, then print it or run cmd.
  topic <args>  Run `ros2 topic <args>` in the exact SITL ROS environment.
  echo <topic>  Echo one message from a topic in the exact SITL ROS environment.
  down          Stop launches started by this runner.

Common SITL overrides:
  SIM_DOMAIN=9
  CONNECTION_URL=udp://:14540
  SIM_DASH_PORT=8091
  SIM_UNDERLAY=~/dronetrack_sim/install
  SIM_OVERLAY=~/dronetrack_groundstation_ros_sim/install
  SIM_VENV=~/ros_venv   (auto-created by build-sim)

Examples:
  scripts/ros_wsl.sh doctor
  scripts/ros_wsl.sh sitl allow_scan_without_lock:=true auto_start:=true fake_target_class:=ignored_ball
  scripts/ros_wsl.sh gazebo device:=cuda:0 model_path:=yolov8s.pt
  scripts/ros_wsl.sh topic list
  scripts/ros_wsl.sh echo /drone/mission/state
EOF
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

have() {
  command -v "$1" >/dev/null 2>&1
}

require_wsl() {
  if ! grep -qi microsoft /proc/version 2>/dev/null; then
    echo "WARNING: this runner is tuned for WSL Ubuntu; continuing on native Linux." >&2
  fi
}

ensure_venv() {
  if [ ! -f "${SIM_VENV}/bin/activate" ]; then
    echo "Creating Python venv at ${SIM_VENV} (--system-site-packages) ..."
    python3 -m venv --system-site-packages "${SIM_VENV}"
    # shellcheck disable=SC1090
    source "${SIM_VENV}/bin/activate"
    pip install --upgrade pip >/dev/null 2>&1
    pip install "numpy<2" ultralytics opencv-python mavsdk aioconsole grpcio
    echo "Venv ready: ${SIM_VENV}"
  else
    # shellcheck disable=SC1090
    source "${SIM_VENV}/bin/activate"
  fi
}

activate_venv() {
  if [ -f "${SIM_VENV}/bin/activate" ]; then
    # shellcheck disable=SC1090
    source "${SIM_VENV}/bin/activate"
  fi
}

write_sim_cyclonedds() {
  mkdir -p "$(dirname "${CYCLONE_CFG}")"
  cat > "${CYCLONE_CFG}" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces><NetworkInterface address="127.0.0.1" presence_required="false" multicast="false"/></Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers><Peer address="127.0.0.1"/></Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
XML
}

source_ros_base() {
  [ -f /opt/ros/jazzy/setup.bash ] || die "ROS 2 Jazzy not found at /opt/ros/jazzy/setup.bash"
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
}

source_sim_env() {
  require_wsl
  write_sim_cyclonedds
  source_ros_base

  if [ -f "${SIM_UNDERLAY}/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${SIM_UNDERLAY}/setup.bash"
  fi
  if [ -f "${SIM_OVERLAY}/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${SIM_OVERLAY}/setup.bash"
  fi

  activate_venv

  # `ros2 launch` runs node console-scripts under system python3 (their shebang),
  # which cannot see the venv. Put the venv site-packages on PYTHONPATH so the
  # nodes can import torch/ultralytics/mavsdk installed there.
  if [ -d "${SIM_VENV}" ]; then
    local vsp
    vsp="$(ls -d "${SIM_VENV}"/lib/python*/site-packages 2>/dev/null | head -1)"
    [ -n "${vsp}" ] && export PYTHONPATH="${vsp}${PYTHONPATH:+:${PYTHONPATH}}"
  fi

  export ROS_DOMAIN_ID="${SIM_DOMAIN}"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="file://${CYCLONE_CFG}"
  export ROS_LOG_DIR="${SIM_LOG_BASE}"
  mkdir -p "${ROS_LOG_DIR}"
  hash -r || true
}

ros_pkg_exists() {
  ros2 pkg prefix "$1" >/dev/null 2>&1
}

cmd_doctor() {
  require_wsl
  echo "Repo: ${REPO_ROOT}"
  echo "ROS distro: jazzy"
  echo "SIM_DOMAIN=${SIM_DOMAIN}"
  echo "SIM_UNDERLAY=${SIM_UNDERLAY}"
  echo "SIM_OVERLAY=${SIM_OVERLAY}"
  echo "SIM_LOG_BASE=${SIM_LOG_BASE}"
  echo "CYCLONE_CFG=${CYCLONE_CFG}"
  echo

  [ -f /opt/ros/jazzy/setup.bash ] && echo "OK  /opt/ros/jazzy/setup.bash" || echo "MISS /opt/ros/jazzy/setup.bash"
  have python3 && echo "OK  python3: $(command -v python3)" || echo "MISS python3"
  have colcon && echo "OK  colcon: $(command -v colcon)" || echo "MISS colcon"
  have ros2 && echo "OK  ros2 already on PATH" || echo "INFO ros2 not on PATH until environment is sourced"
  [ -f "${SIM_UNDERLAY}/setup.bash" ] && echo "OK  sim underlay setup.bash" || echo "MISS sim underlay setup.bash (${SIM_UNDERLAY})"
  [ -f "${SIM_OVERLAY}/setup.bash" ] && echo "OK  current repo sim overlay setup.bash" || echo "MISS current repo sim overlay setup.bash (run: scripts/ros_wsl.sh build-sim)"
  [ -f "${SIM_VENV}/bin/activate" ] && echo "OK  python venv ${SIM_VENV}" || echo "MISS python venv (auto-created by build-sim)"
  echo

  source_sim_env
  echo "Sourced env: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
  echo "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
  echo

  local missing=0 pkg
  for pkg in drone_fake drone_interfaces drone_control drone_telemetry drone_tracker drone_diagnostics dronetrack_pi dronetrack_web_bridge; do
    if ros_pkg_exists "${pkg}"; then
      echo "OK  package ${pkg} -> $(ros2 pkg prefix "${pkg}")"
    else
      echo "MISS package ${pkg}"
      missing=1
    fi
  done

  if python3 - <<'PY' >/dev/null 2>&1
import mavsdk
PY
  then
    echo "OK  python mavsdk import"
  else
    echo "MISS python mavsdk import (install in WSL if SITL telemetry cannot start)"
    missing=1
  fi

  if [ "${missing}" -eq 0 ]; then
    echo
    echo "Doctor passed. Try: scripts/ros_wsl.sh sitl"
  else
    echo
    echo "Doctor found missing pieces. First try: scripts/ros_wsl.sh build-sim"
    return 1
  fi
}

stage_sim_configs() {
  stage_configs dronetrack_pi pi.yaml topics.yaml
  stage_configs dronetrack_groundstation groundstation.yaml topics.yaml
}

cmd_build_sim() {
  require_wsl
  stage_sim_configs
  source_ros_base
  if [ -f "${SIM_UNDERLAY}/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${SIM_UNDERLAY}/setup.bash"
  fi
  export ROS_DOMAIN_ID="${SIM_DOMAIN}"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  mkdir -p "${SIM_BUILD_BASE}" "${SIM_OVERLAY}" "${SIM_LOG_BASE}"

  # Build uses system Python (not the venv) so rosidl can find numpy C headers.
  echo "Building current repo overlay -> ${SIM_OVERLAY}"
  colcon build \
    --base-paths "${WS}/src" \
    --build-base "${SIM_BUILD_BASE}" \
    --install-base "${SIM_OVERLAY}" \
    --symlink-install \
    --packages-select "${SIM_PACKAGES[@]}"

  # Create the runtime venv after a successful build (pip packages go here).
  ensure_venv
  echo ""
  echo "Build complete. Runtime venv: ${SIM_VENV}"
  echo "To add pip packages:  source ${SIM_VENV}/bin/activate && pip install <pkg>"
}

ensure_sim_overlay() {
  if [ ! -f "${SIM_OVERLAY}/setup.bash" ]; then
    echo "Current repo SITL overlay missing; building it now."
    cmd_build_sim
  fi
}

cmd_sitl() {
  require_wsl
  ensure_sim_overlay
  source_sim_env
  ros_pkg_exists drone_fake || die "drone_fake not found. Set SIM_UNDERLAY to an install that contains drone_fake."
  ros_pkg_exists dronetrack_pi || die "dronetrack_pi not found after build. Run scripts/ros_wsl.sh doctor."

  local plan="${MISSION_PLAN_FILE:-${REPO_ROOT}/ros_ws/src/drone_control/missions/scan_and_orbit.yaml}"
  local args=(
    params_file:="${CONFIGS}/pi.yaml"
    dashboard_params_file:="${CONFIGS}/groundstation.yaml"
    mission_plan_file:="${plan}"
    connection_url:="${CONNECTION_URL}"
    dashboard_port:="${SIM_DASH_PORT}"
  )
  args+=("$@")

  echo "SITL env | domain=${ROS_DOMAIN_ID}, rmw=${RMW_IMPLEMENTATION}, cyclone=${CYCLONEDDS_URI}"
  echo "Overlay  | underlay=${SIM_UNDERLAY}, current=${SIM_OVERLAY}"
  echo "Mission  | ${plan}"
  echo "PX4      | ${CONNECTION_URL}"
  echo "Dashboard| http://127.0.0.1:${SIM_DASH_PORT}/"
  echo "Tip      | PX4 shell: param set COM_DISARM_PRFLT -1 ; commander arm"
  exec ros2 launch dronetrack_pi sitl_mission_launch.py "${args[@]}"
}

cmd_gazebo() {
  require_wsl
  ensure_sim_overlay
  source_sim_env
  ros_pkg_exists dronetrack_perception || die "dronetrack_perception not found. Run scripts/ros_wsl.sh build-sim."
  ros_pkg_exists dronetrack_pi || die "dronetrack_pi not found. Run scripts/ros_wsl.sh build-sim."

  # Check Gazebo bridge packages
  ros_pkg_exists ros_gz_bridge || die "ros_gz_bridge not found. Install: sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-interfaces"

  local plan="${MISSION_PLAN_FILE:-}"
  if [ -z "${plan}" ]; then
    local control_share
    control_share="$(ros2 pkg prefix drone_control 2>/dev/null || true)"
    if [ -n "${control_share}" ] && [ -f "${control_share}/share/drone_control/missions/scan_and_orbit.yaml" ]; then
      plan="${control_share}/share/drone_control/missions/scan_and_orbit.yaml"
    fi
  fi

  # Resolve red_ball.sdf (repo-level models/ dir)
  local ball_sdf="${REPO_ROOT}/models/red_ball.sdf"

  local args=(
    params_file:="${CONFIGS}/pi.yaml"
    gs_params_file:="${CONFIGS}/groundstation.yaml"
    connection_url:="${CONNECTION_URL}"
    dashboard_port:="${SIM_DASH_PORT}"
    ball_sdf:="${ball_sdf}"
  )
  [ -n "${plan}" ] && args+=(mission_plan_file:="${plan}")
  args+=("$@")

  echo "GAZEBO SITL env | domain=${ROS_DOMAIN_ID}, rmw=${RMW_IMPLEMENTATION}"
  echo "Venv            | ${VIRTUAL_ENV:-system python (no venv!)}"
  echo "Overlay         | underlay=${SIM_UNDERLAY}, current=${SIM_OVERLAY}"
  [ -n "${plan}" ] && echo "Mission         | ${plan}" || echo "Mission         | (default)"
  echo "PX4             | ${CONNECTION_URL}"
  echo "Dashboard       | http://127.0.0.1:${SIM_DASH_PORT}/"
  echo "Red ball SDF    | ${ball_sdf}"
  echo ""
  echo "Prereqs:"
  echo "  1. PX4 SITL + Gazebo running with a camera model (e.g. x500_mono_cam)"
  echo "  2. sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-interfaces"
  echo "  3. Vehicle armed (QGC or PX4 shell: commander arm)"
  echo ""
  echo "Red-ball model: target_class defaults to red_ball."
  echo "Use a trained model: model_path:=path/to/red_ball_yolo26s.pt device:=cuda:0"
  echo "Override Gazebo camera: gz_camera_topic:=/world/MYWORLD/model/.../image"
  exec ros2 launch dronetrack_pi sitl_gazebo_launch.py "${args[@]}"
}

cmd_env() {
  source_sim_env
  if [ "$#" -gt 0 ]; then
    exec "$@"
  fi
  cat <<EOF
ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
CYCLONEDDS_URI=${CYCLONEDDS_URI}
ROS_LOG_DIR=${ROS_LOG_DIR}
SIM_UNDERLAY=${SIM_UNDERLAY}
SIM_OVERLAY=${SIM_OVERLAY}
EOF
}

cmd_topic() {
  [ "$#" -gt 0 ] || die "topic requires ros2 topic arguments"
  source_sim_env
  exec ros2 topic "$@"
}

cmd_echo() {
  [ "$#" -gt 0 ] || die "echo requires a topic name"
  source_sim_env
  exec timeout "${ECHO_TIMEOUT:-8}" ros2 topic echo --once "$1"
}

cmd_down() {
  echo "Stopping current-repo SITL launch processes if present..."
  pkill -INT -f "[r]os2 launch dronetrack_pi sitl_" 2>/dev/null || true
  sleep 2
  pkill -TERM -f "[r]os2 launch dronetrack_pi sitl_" 2>/dev/null || true
}

main() {
  local cmd="${1:-}"
  case "${cmd}" in
    doctor) shift; cmd_doctor "$@" ;;
    build-sim) shift; cmd_build_sim "$@" ;;
    sitl) shift; cmd_sitl "$@" ;;
    gazebo) shift; cmd_gazebo "$@" ;;
    env) shift; cmd_env "$@" ;;
    topic) shift; cmd_topic "$@" ;;
    echo) shift; cmd_echo "$@" ;;
    down) shift; cmd_down "$@" ;;
    -h|--help|help|"") usage ;;
    *) usage; die "unknown command: ${cmd}" ;;
  esac
}

main "$@"
