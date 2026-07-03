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

# SITL bring-up params, injected in the background once PX4 is up:
#   NAV_DLL_ACT/NAV_RCL_ACT 0 + COM_RCL_EXCEPT 4: headless SITL has no GCS/RC;
#     a client connecting to the GCS port and dropping (QGC, a MAVSDK script)
#     must never latch "Preflight Fail: No connection to the GCS" -> Arming
#     denied, which deadlocks mission re-runs (docs/handoff_orbit_rerun.md).
#   COM_DISARM_LAND 2.0: disarm shortly after landing.
(
  for _ in $(seq 1 120); do
    grep -qa "Ready for takeoff" /tmp/px4.log 2>/dev/null && break
    sleep 1
  done
  {
    echo "param set NAV_DLL_ACT 0"
    echo "param set NAV_RCL_ACT 0"
    echo "param set COM_RCL_EXCEPT 4"
    echo "param set COM_DISARM_LAND 2.0"
  } > /tmp/px4in
) &

exec 9<>/tmp/px4in            # hold fifo open rw -> stdin never EOFs
HEADLESS=1 make px4_sitl gz_x500_mono_cam <&9 2>&1 \
  | stdbuf -oL tr "\r" "\n" \
  | stdbuf -oL tee /tmp/px4_raw.log \
  | stdbuf -oL grep -aiE "info|warn|error|err]|pxh|gz_bridge|gazebo|camera|ready|takeoff|arm|fail|connect|EKF|home set|segfault|signal|backtrace|core" \
  > /tmp/px4.log
