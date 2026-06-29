#!/usr/bin/env bash
# Set up the laptop ground-station workspace (run ON THE LAPTOP; Linux or WSL2).
#   - copies the shared message package drone_interfaces (wire compatibility)
#   - stages configs and builds the laptop packages
#   - reminds you to install YOLO deps
set -eo pipefail  # not -u: ROS/colcon setup.bash reference unbound vars
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

echo "== DroneTrack ground-station setup =="
mkdir -p "${WS}/src"

echo "Copying shared message package from dronetrack_pi_ros ..."
copy_reused_pkg drone_interfaces

echo "Staging configs ..."
stage_configs dronetrack_groundstation groundstation.yaml topics.yaml

echo "Installing Python perception deps ..."
# Ubuntu 24.04 / ROS Jazzy use an externally-managed Python, so install into the
# user site with --break-system-packages.
PIP="python3 -m pip install --user --break-system-packages"
${PIP} ultralytics psutil || \
  echo "pip install reported issues; install ultralytics/psutil manually."
# CRITICAL: ROS Jazzy cv_bridge is built against NumPy 1.x. ultralytics/torch may
# pull NumPy 2.x and break `import cv_bridge`. Pin it back, last, so it wins.
${PIP} "numpy<2" || echo "numpy pin failed; run: ${PIP} 'numpy<2'"
python3 -c "import numpy,cv_bridge,cv2; print('numpy',numpy.__version__,'cv_bridge+cv2 OK')" || \
  echo "WARN: cv_bridge import check failed -- re-pin numpy<2 before running YOLO."

# Optional NVIDIA GPU acceleration (verified ~3x faster). After the above, install
# a CUDA torch build, then RE-PIN numpy<2 (CUDA wheels often re-upgrade it):
#   ${PIP} torch torchvision --index-url https://download.pytorch.org/whl/cu130
#   ${PIP} "numpy<2"
# Then launch YOLO with: device:=cuda:0 half_precision:=True

echo "Resolving ROS dependencies (rosdep) ..."
if command -v rosdep >/dev/null 2>&1; then
  rosdep install --from-paths "${WS}/src" --ignore-src -r -y || \
    echo "rosdep reported issues; continuing."
fi

echo "Building ground-station packages ..."
cd "${WS}"
colcon build --symlink-install \
  --packages-select dronetrack_msgs drone_interfaces \
                    dronetrack_perception dronetrack_groundstation dronetrack_web_bridge \
  || colcon build --symlink-install

echo
echo "Done. Next:"
echo "  source ${WS}/install/setup.bash"
echo "  ${REPO_ROOT}/scripts/run_groundstation.sh"
