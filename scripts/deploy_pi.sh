#!/usr/bin/env bash
# Push the split-architecture Pi packages from the laptop and rebuild them on the
# Pi. Run after editing dronetrack_pi / dronetrack_msgs. The Pi must be reachable
# (run discover_ips.sh first if the subnet drifted).
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${HERE}/_common.sh"

USER_AT="$(pi_target)"
echo "==> Syncing Pi packages to ${USER_AT} ..."
for pkg in dronetrack_msgs dronetrack_pi; do
  rsync -a --exclude __pycache__ --exclude build --exclude install \
    -e "ssh ${PI_SSH_OPTS[*]}" \
    "${WS}/src/${pkg}/" "${USER_AT}:drone_ws/src/${pkg}/"
  echo "  synced ${pkg}"
done

# dronetrack_pi is pure-Python (symlink-install) and rebuilds instantly. We only
# touch dronetrack_msgs when you actually changed a .msg (MSGS=1), and then we
# clean its build/install first -- ament leaves a real dir where it later wants a
# symlink, which otherwise fails the rebuild ("existing path cannot be removed").
PKGS="dronetrack_pi"
PRECLEAN=""
if [ "${MSGS:-0}" = "1" ]; then
  echo "==> MSGS=1: clean-rebuilding dronetrack_msgs as well ..."
  PRECLEAN='rm -rf ~/drone_ws/build/dronetrack_msgs ~/drone_ws/install/dronetrack_msgs;'
  PKGS="dronetrack_msgs dronetrack_pi"
fi

echo "==> Rebuilding on the Pi (colcon): ${PKGS} ..."
pi_run "cd ~/drone_ws && ${PRECLEAN} source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select ${PKGS} 2>&1 | tail -8"

echo "Done. Bring the system up with:  bash scripts/up.sh"
