#!/usr/bin/env bash
# Push the split-architecture Pi packages from the laptop and rebuild them on the
# Pi. Run after editing dronetrack_pi / dronetrack_msgs. The Pi must be reachable
# (run discover_ips.sh first if the subnet drifted).
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${HERE}/_common.sh"

USER_AT="$(pi_target)"

# Stage the HARDWARE config into the package before rsync. configs/pi.yaml is
# SIM-flavored (gz camera intrinsics, sim calibration) and the sim's
# stage_configs copies it over the same staged path — which once shipped sim
# values toward the real drone. Deploy always ships configs/pi_hardware.yaml.
if [ -f "${CONFIGS}/pi_hardware.yaml" ]; then
  cp -f "${CONFIGS}/pi_hardware.yaml" "${WS}/src/dronetrack_pi/config/pi.yaml"
  echo "==> Staged configs/pi_hardware.yaml as the deployed pi.yaml"
else
  echo "WARNING: configs/pi_hardware.yaml missing; deploying whatever is staged." >&2
fi

# All packages to sync: boundary nodes + vendored flight packages + perception
# (local ncnn YOLO runs on the Pi since 2026-07-05).
SYNC_PKGS=(dronetrack_msgs dronetrack_pi drone_interfaces drone_camera drone_tracker drone_control drone_telemetry drone_diagnostics dronetrack_perception)

echo "==> Syncing ${#SYNC_PKGS[@]} packages to ${USER_AT} ..."
for pkg in "${SYNC_PKGS[@]}"; do
  src="${WS}/src/${pkg}"
  if [ ! -d "${src}" ]; then
    echo "  ${pkg}: SKIP (not in workspace)"
    continue
  fi
  rsync -a --exclude __pycache__ --exclude build --exclude install \
    -e "ssh ${PI_SSH_OPTS[*]}" \
    "${src}/" "${USER_AT}:drone_ws/src/${pkg}/"
  echo "  synced ${pkg}"
done

# Pure-Python packages rebuild instantly with symlink-install. Message packages
# (dronetrack_msgs, drone_interfaces) need a clean rebuild when .msg files change
# because ament leaves a real dir where it later wants a symlink. Use MSGS=1 to
# trigger the clean.
BUILD_PKGS="dronetrack_pi drone_tracker drone_control drone_telemetry drone_diagnostics drone_camera dronetrack_perception"
PRECLEAN=""
if [ "${MSGS:-0}" = "1" ]; then
  echo "==> MSGS=1: clean-rebuilding message packages as well ..."
  PRECLEAN='rm -rf ~/drone_ws/build/dronetrack_msgs ~/drone_ws/install/dronetrack_msgs ~/drone_ws/build/drone_interfaces ~/drone_ws/install/drone_interfaces;'
  BUILD_PKGS="dronetrack_msgs drone_interfaces ${BUILD_PKGS}"
fi

echo "==> Syncing the ncnn model for the Pi-local YOLO ..."
pi_run 'mkdir -p ~/models/red_ball_ncnn_model'
rsync -a -e "ssh ${PI_SSH_OPTS[*]}" \
  "${REPO_ROOT}/models/red_ball_ncnn_model/" "${USER_AT}:models/red_ball_ncnn_model/"
echo "  synced models/red_ball_ncnn_model"

echo "==> Rebuilding on the Pi (colcon): ${BUILD_PKGS} ..."
pi_run "cd ~/drone_ws && ${PRECLEAN} source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select ${BUILD_PKGS} 2>&1 | tail -12"

echo "Done. Bring the system up with:  bash scripts/up.sh"
