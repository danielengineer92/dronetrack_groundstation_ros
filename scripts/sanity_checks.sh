#!/usr/bin/env bash
# End-to-end sanity checks for the split DroneTrack architecture.
# Run from EITHER machine after both sides are up. Uses network.yaml for IPs.
#
#   ./sanity_checks.sh            # run all checks
#   ./sanity_checks.sh ping ros   # run a subset (ping ssh ros camera yolo roundtrip latency)
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
export_ros_env
maybe_setup_cyclonedds >/dev/null 2>&1 || true
[ -f "${WS}/install/setup.bash" ] && source "${WS}/install/setup.bash"

PI_IP="$(yaml_get pi_ip)"; LAPTOP_IP="$(yaml_get laptop_ip)"
SSH_USER="$(yaml_get ssh_user || echo robotpi)"; SSH_PORT="$(yaml_get ssh_port || echo 22)"
CHECKS=("$@"); [ ${#CHECKS[@]} -eq 0 ] && CHECKS=(ping ssh ros camera yolo roundtrip latency)

has() { for c in "${CHECKS[@]}"; do [ "$c" = "$1" ] && return 0; done; return 1; }
hr() { printf '\n=== %s ===\n' "$1"; }

if has ping; then
  hr "PING Pi (${PI_IP})"
  ping -c 3 -W 2 "${PI_IP}" && echo "OK: Pi reachable" || echo "FAIL: cannot ping Pi"
fi

if has ssh; then
  hr "SSH Pi (${SSH_USER}@${PI_IP}:${SSH_PORT})"
  ssh -o ConnectTimeout=5 -o BatchMode=yes -p "${SSH_PORT}" "${SSH_USER}@${PI_IP}" 'echo OK: ssh works; hostname' \
    || echo "FAIL/INFO: SSH non-interactive failed (key not set up?). Try a manual ssh."
fi

if has ros; then
  hr "ROS topic visibility (domain ${ROS_DOMAIN_ID})"
  echo "Topics seen:"; ros2 topic list 2>/dev/null | sort
  echo "--- key topics present? ---"
  for t in /drone/camera/image_raw/compressed /groundstation/vision/detections \
           /groundstation/heartbeat /drone/groundstation/link_status /drone/telemetry; do
    ros2 topic list 2>/dev/null | grep -qx "$t" && echo "  OK   $t" || echo "  MISS $t"
  done
fi

if has camera; then
  hr "Camera stream rate (/drone/camera/image_raw/compressed)"
  timeout 6 ros2 topic hz /drone/camera/image_raw/compressed || echo "INFO: no camera frames (is the Pi up + compressing?)"
fi

if has yolo; then
  hr "YOLO detection rate (/groundstation/vision/detections)"
  timeout 6 ros2 topic hz /groundstation/vision/detections || echo "INFO: no detections (is laptop YOLO up?)"
fi

if has roundtrip; then
  hr "Detection round trip (laptop -> Pi gate -> /drone/vision/detections)"
  echo "Inbound (laptop, untrusted):"
  timeout 5 ros2 topic hz /groundstation/vision/detections || echo "  none"
  echo "Outbound (after Pi gate, trusted):"
  timeout 5 ros2 topic hz /drone/vision/detections || echo "  none (gate dropping? check heartbeat + staleness)"
fi

if has latency; then
  hr "Link status / estimated latency (/drone/groundstation/link_status)"
  echo "Requires NTP/chrony clock sync between Pi and laptop to be meaningful."
  timeout 4 ros2 topic echo --once /drone/groundstation/link_status || echo "INFO: no link_status (is the Pi watchdog up?)"
fi

echo
echo "Sanity checks complete."
