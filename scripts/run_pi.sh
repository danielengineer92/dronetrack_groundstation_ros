#!/usr/bin/env bash
# Launch the Pi-side stack. Run ON THE DRONE Pi after setup_pi.sh.
# Extra args are forwarded to ros2 launch, e.g.:
#   ./run_pi.sh connection_url:=serial:///dev/ttyACM0:57600 allow_mavsdk_actions:=false
set -eo pipefail  # not -u: ROS/colcon setup.bash reference unbound vars
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

export_ros_env
maybe_setup_cyclonedds "$(yaml_get pi_ip)"   # pin the Pi's LAN interface

# shellcheck disable=SC1091
source "${INSTALL_DIR}/setup.bash"

echo "Launching dronetrack_pi/pi_launch.py ..."
exec ros2 launch dronetrack_pi pi_launch.py \
  params_file:="${CONFIGS}/pi.yaml" \
  "$@"
