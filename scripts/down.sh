#!/usr/bin/env bash
# Cleanly stop the WHOLE split system from the LAPTOP (WSL): the laptop ground
# station and the detached Pi stack.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${HERE}/_common.sh"

echo "==> Stopping laptop ground station ..."
if pkill -f "[g]roundstation_launch" 2>/dev/null; then echo "  stopped"; else echo "  (not running)"; fi

echo "==> Stopping Pi stack ($(pi_target)) ..."
# bracket-trick patterns so pkill can't match (and kill) its own SSH command.
if pi_run 'pkill -9 -f "[r]os2 launch" 2>/dev/null; pkill -9 -f "[d]rone_ws/install" 2>/dev/null; pkill -9 -f "[c]amera_compressor" 2>/dev/null; sleep 1; echo "  Pi stopped (remaining: $(pgrep -f "[d]rone_ws/install" | wc -l))"'; then
  :
else
  echo "  could not reach Pi (already off, or run: bash scripts/discover_ips.sh)"
fi
