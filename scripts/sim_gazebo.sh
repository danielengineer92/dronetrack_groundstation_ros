#!/usr/bin/env bash
# Launch SITL with real Gazebo camera + YOLO vision pipeline.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${HERE}/ros_wsl.sh" gazebo "$@"
