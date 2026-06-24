#!/usr/bin/env bash
# Quick end-to-end health check of the split system, run on the LAPTOP (WSL).
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${HERE}/_common.sh"

LAP="$(ip -4 route get 1.1.1.1 2>/dev/null | grep -oP 'src \K[0-9.]+' || echo '?')"
PI_IP="$(yaml_get pi_ip)"
USER_AT="$(pi_target)"

echo "Laptop IP : $LAP"
echo "Pi IP     : $PI_IP   (network.yaml; run discover_ips.sh if stale)"

printf 'Ping Pi   : '
if ping -c1 -W2 "$PI_IP" >/dev/null 2>&1; then echo "OK"; else echo "FAIL  -> bash scripts/discover_ips.sh"; fi

printf 'Pi nodes  : '
n="$(PI_SSH_TRIES=1 pi_run 'pgrep -f "[d]rone_ws/install" | wc -l' 2>/dev/null | tr -d '[:space:]')" || n=""
if [ -n "$n" ] && [ "$n" -gt 0 ]; then echo "$n running"; else echo "0 / unreachable"; fi

printf 'Gate link : '
PI_SSH_TRIES=1 pi_run 'grep -a "Gate |" /tmp/pi_launch.log 2>/dev/null | tail -1 | sed "s/.*: //"' 2>/dev/null || echo "n/a"

printf 'Dashboard : '
if curl -s -m 3 http://127.0.0.1:8080/api/status >/dev/null 2>&1; then
  curl -s -m 3 http://127.0.0.1:8080/api/status \
    | python3 -c 'import sys,json; d=json.load(sys.stdin); print("link_ok=%s px4=%s fps=%s cam_age=%s" % (d["link_ok"], d["px4_connected"], d["perception_fps"], d["camera_frame_age_s"]))' 2>/dev/null \
    || echo "up (unparseable)"
else
  echo "not responding (ground station down?)"
fi
