#!/usr/bin/env bash
# Runs INSIDE the container (via `dt setup`). Idempotent: clones + builds PX4,
# builds the repo SITL overlay, and creates the YOLO venv. Re-running is safe.
set -eo pipefail

REPO="${REPO:-$HOME/dronetrack_groundstation_ros}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_REF="${PX4_REF:-main}"   # pin a tag/branch here if you want reproducibility

source /opt/ros/jazzy/setup.bash

echo "######################################################################"
echo "# 1/3  PX4-Autopilot (${PX4_REF})"
echo "######################################################################"
if [ ! -d "${PX4_DIR}/.git" ]; then
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive "${PX4_DIR}"
fi
cd "${PX4_DIR}"
git fetch --all --tags --quiet || true
git checkout "${PX4_REF}" || true
git submodule update --init --recursive

echo "--- PX4 build dependencies (apt + pip; --no-sim-tools keeps our gz-harmonic) ---"
# PX4's own dependency installer is the source of truth for the checked-out ref.
bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools || {
  echo "ubuntu.sh hit an issue; installing core PX4 python deps directly." >&2
  python3 -m pip install --break-system-packages \
    'empy==3.3.4' toml numpy jinja2 pyros-genmsg packaging kconfiglib jsonschema \
    future pyyaml cerberus pyserial pymavlink || true
}

echo "--- Building PX4 SITL firmware (first run compiles the gz bridge later) ---"
make px4_sitl

echo "######################################################################"
echo "# 2/3  DroneTrack SITL overlay + YOLO venv"
echo "######################################################################"
cd "${REPO}"
bash scripts/ros_wsl.sh build-sim

echo "######################################################################"
echo "# 3/3  Done"
echo "######################################################################"
echo "PX4:     ${PX4_DIR}"
echo "Overlay: ${HOME}/dronetrack_groundstation_ros_sim/install"
echo "Venv:    ${HOME}/ros_venv"
echo
echo "Next (two shells):"
echo "  docker/dt px4     # terminal 1: PX4 + Gazebo (renders on the 5080)"
echo "  docker/dt run     # terminal 2: the DroneTrack stack (camera->YOLO->control)"
