#!/usr/bin/env bash
# One-command, robust bringup of the WHOLE split system, run on the LAPTOP (WSL).
#
#   1. discover + sync IPs (laptop + Pi) on whatever subnet we landed on
#   2. deploy the Pi launcher, clean-slate the Pi, start it DETACHED
#   3. verify the Pi nodes actually came up
#   4. start the laptop ground station in the foreground (Ctrl+C to stop it)
#
# Extra args are forwarded to the ground-station launch, overriding the defaults:
#   ./up.sh                                   # uses the defaults below
#   ./up.sh model_path:=$PWD/models/red_ball_ncnn_model target_class:=red_ball device:=cpu
#
# Pi-side overrides via env, e.g.:  PI_ALLOW_ACTIONS=true ./up.sh
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${HERE}/_common.sh"

# Default to the off-OneDrive WSL build location.
: "${INSTALL_DIR:=$HOME/dronetrack_gs/install}"
export INSTALL_DIR

GS_DEFAULTS=(model_path:="${REPO_ROOT}/models/red_ball_yolo11s.pt" target_class:=red_ball device:=cuda:0 half_precision:=True max_fps:=60.0)
GS_ARGS=("$@"); [ "${#GS_ARGS[@]}" -gt 0 ] || GS_ARGS=("${GS_DEFAULTS[@]}")

echo "==> [1/5] Discovering laptop + Pi IPs (subnet-agnostic) ..."
bash "${HERE}/discover_ips.sh"

USER_AT="$(pi_target)"

echo "==> [2/5] Deploying Pi launcher to ${USER_AT} ..."
scp "${PI_SSH_OPTS[@]}" "${HERE}/pi_split_launcher.sh" "${USER_AT}:run_pi_split.sh" >/dev/null
pi_run 'chmod +x ~/run_pi_split.sh'

echo "==> [3/5] Clean-slate the Pi (kill any orphaned nodes holding the camera) ..."
pi_run 'pkill -9 -f "[r]os2 launch" 2>/dev/null; pkill -9 -f "[d]rone_ws/install" 2>/dev/null; pkill -9 -f "[c]amera_compressor" 2>/dev/null; sleep 2; true'

echo "==> [4/5] Launching Pi stack (detached; survives SSH drops) ..."
pi_run "setsid env PI_CONN='${PI_CONN:-}' PI_ALLOW_ACTIONS='${PI_ALLOW_ACTIONS:-false}' bash ~/run_pi_split.sh >/tmp/pi_launch.log 2>&1 </dev/null & sleep 1; echo '  launcher started'"

echo -n "    waiting for Pi nodes "
pi_up=""
for _ in $(seq 1 15); do
  n="$(PI_SSH_TRIES=1 pi_run 'pgrep -f "[d]rone_ws/install" | wc -l' 2>/dev/null | tr -d '[:space:]')" || n=""
  if [ "${n:-0}" -ge 5 ]; then pi_up=1; echo " up (${n} processes)"; break; fi
  echo -n "."; sleep 2
done
if [ -z "$pi_up" ]; then
  echo " NOT confirmed."
  echo "    Check the Pi log:  ssh ${USER_AT} 'tail -40 /tmp/pi_launch.log'"
fi

echo "==> [5/5] Launching laptop ground station (Ctrl+C to stop) ..."
pkill -f "[g]roundstation_launch" 2>/dev/null || true   # free port 8080 if a stale GS is up
sleep 1
echo "    Dashboard -> http://127.0.0.1:8080/"
exec bash "${HERE}/run_groundstation.sh" "${GS_ARGS[@]}"
