#!/usr/bin/env bash
# Set up the Pi-side workspace (run ON THE DRONE Pi, or in its WSL/Linux dev env).
#   - copies reused safety-critical packages from dronetrack_pi_ros
#   - copies the shared message package drone_interfaces (wire compatibility)
#   - stages configs and builds
set -eo pipefail  # not -u: ROS/colcon setup.bash reference unbound vars
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

echo "== DroneTrack Pi setup =="
mkdir -p "${WS}/src"

echo "Copying reused packages from dronetrack_pi_ros ..."
# drone_interfaces MUST be shared so laptop and Pi agree on message types.
copy_reused_pkg drone_interfaces
copy_reused_pkg drone_diagnostics    # node_diagnostics helper used by reused nodes
copy_reused_pkg drone_camera
copy_reused_pkg drone_tracker
copy_reused_pkg drone_telemetry
copy_reused_pkg drone_control

echo "Staging configs ..."
stage_configs dronetrack_pi pi.yaml topics.yaml

echo "Resolving dependencies (rosdep) ..."
if command -v rosdep >/dev/null 2>&1; then
  rosdep install --from-paths "${WS}/src" --ignore-src -r -y || \
    echo "rosdep reported issues; continuing (install MAVSDK/ultralytics/etc. manually if needed)."
fi

echo "Building Pi packages ..."
cd "${WS}"
colcon build --symlink-install \
  --packages-up-to dronetrack_pi \
  --packages-select dronetrack_msgs drone_interfaces drone_diagnostics \
                    drone_camera drone_tracker drone_telemetry drone_control dronetrack_pi \
  || colcon build --symlink-install   # fall back to building whatever is present

echo
echo "Done. Next:"
echo "  source ${WS}/install/setup.bash"
echo "  ${REPO_ROOT}/scripts/run_pi.sh"
