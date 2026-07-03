#!/usr/bin/env bash
# Launch PX4 SITL + Gazebo headless INSIDE the sim container, detached-safe.
#
# Gotchas this script encodes (see docs/gazebo_sitl.md):
#   - `make px4_sitl` exits the moment its pxh> shell hits stdin EOF, so stdin
#     is held open forever via a read-write FIFO (fd 9). Inject pxh commands
#     with:  echo "commander arm" > /tmp/px4in
#   - Raw px4/gz output contains \r-progress spam that once produced a 702MB
#     single-line log; tr rewrites \r to \n first.
#   - /tmp/px4_raw.log keeps EVERYTHING (crash backtraces survive);
#     /tmp/px4.log is the readable filtered view.
#
# Usage (from the host):
#   docker exec -d dronetrack-sim bash -lc 'setsid /tmp/px4_launch.sh </dev/null >/dev/null 2>&1 &'
set -e
rm -f /tmp/px4in /tmp/px4.log /tmp/px4_raw.log
mkfifo /tmp/px4in
cd "$HOME/PX4-Autopilot"
exec 9<>/tmp/px4in            # hold fifo open rw -> stdin never EOFs
HEADLESS=1 make px4_sitl gz_x500_mono_cam <&9 2>&1 \
  | stdbuf -oL tr "\r" "\n" \
  | stdbuf -oL tee /tmp/px4_raw.log \
  | stdbuf -oL grep -aiE "info|warn|error|err]|pxh|gz_bridge|gazebo|camera|ready|takeoff|arm|fail|connect|EKF|home set|segfault|signal|backtrace|core" \
  > /tmp/px4.log
