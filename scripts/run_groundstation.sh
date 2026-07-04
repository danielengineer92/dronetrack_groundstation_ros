#!/usr/bin/env bash
# Launch the laptop ground-station stack. Run ON THE LAPTOP after setup_groundstation.sh.
# Extra args are forwarded to ros2 launch, e.g.:
#   ./run_groundstation.sh model_path:=$PWD/models/red_ball_ncnn_model target_class:=red_ball device:=cpu
# NVIDIA GPU (verified ~3x faster than CPU): device:=cuda:0 half_precision:=True
set -eo pipefail  # not -u: ROS/colcon setup.bash reference unbound vars
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

export_ros_env
maybe_setup_cyclonedds "$(yaml_get laptop_ip)"   # pin the laptop's LAN interface

# shellcheck disable=SC1091
source "${INSTALL_DIR}/setup.bash"

# Perception deps (ultralytics/torch/numpy<2) live in the repo venv when present.
# ros2-run node scripts execute under the system python, so expose the venv's
# site-packages via PYTHONPATH (prepended, so its numpy<2 pin wins).
for _venv_site in "${REPO_ROOT}"/.venv/lib/python3*/site-packages; do
  if [ -d "${_venv_site}" ]; then
    export PYTHONPATH="${_venv_site}${PYTHONPATH:+:${PYTHONPATH}}"
    echo "Using venv packages: ${_venv_site}"
  fi
done

echo "Launching dronetrack_groundstation/groundstation_launch.py ..."
# From a Windows browser over WSL2 mirrored networking, use 127.0.0.1 (not
# localhost -> IPv6 ::1, which mirrored WSL does not bridge).
echo "Dashboard: http://127.0.0.1:$(yaml_get dashboard_port || echo 8080)/  (localhost works on native Linux)"
exec ros2 launch dronetrack_groundstation groundstation_launch.py \
  params_file:="${CONFIGS}/groundstation.yaml" \
  "$@"
