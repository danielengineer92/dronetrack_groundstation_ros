#!/usr/bin/env bash
# Backward-compatible alias for the robust WSL ROS runner.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${HERE}/ros_wsl.sh" sitl "$@"
