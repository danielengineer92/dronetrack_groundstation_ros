#!/usr/bin/env bash
# Source this in a SECOND terminal to use ros2 CLI tools against the running stack.
#
# For the live split system (Pi + laptop):
#   source scripts/env.sh
#   ros2 topic list
#   ros2 topic echo /drone/telemetry
#
# For the SITL sim:
#   source scripts/env.sh sim
#   ros2 topic list
#
# Why this is needed: CycloneDDS unicast configs pin specific network interfaces
# and peer IPs. Without sourcing the matching config, ros2 CLI in a new terminal
# can't discover the running nodes.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${HERE}/_common.sh"

MODE="${1:-live}"

source /opt/ros/jazzy/setup.bash

if [ "${MODE}" = "sim" ]; then
    SIM_INSTALL="${SIM_INSTALL:-$HOME/dronetrack_sim/install}"
    if [ -f "${SIM_INSTALL}/setup.bash" ]; then
        source "${SIM_INSTALL}/setup.bash"
    else
        echo "WARNING: sim workspace not built at ${SIM_INSTALL}" >&2
    fi
    export ROS_DOMAIN_ID="${SIM_DOMAIN:-9}"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    CYCLONE_CFG="$HOME/sim_cyclonedds.xml"
    if [ -f "${CYCLONE_CFG}" ]; then
        export CYCLONEDDS_URI="file://${CYCLONE_CFG}"
    fi
    echo "ROS env ready (SITL sim) | domain=${ROS_DOMAIN_ID}"
else
    GS_INSTALL="${INSTALL_DIR:-$HOME/dronetrack_gs/install}"
    if [ -f "${GS_INSTALL}/setup.bash" ]; then
        source "${GS_INSTALL}/setup.bash"
    else
        echo "WARNING: ground-station workspace not built at ${GS_INSTALL}" >&2
    fi
    export_ros_env
    maybe_setup_cyclonedds ""
    echo "ROS env ready (live split) | domain=${ROS_DOMAIN_ID}"
fi

echo "Try: ros2 topic list"
