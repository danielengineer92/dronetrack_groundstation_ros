#!/usr/bin/env bash
# Shared helpers for the DroneTrack ground-station scripts.
# Sourced by setup_*.sh and run_*.sh. Linux / WSL2 (ROS 2 Jazzy).
set -eo pipefail  # not -u: ROS/colcon setup.bash reference unbound vars and trip it

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${REPO_ROOT}/ros_ws"
CONFIGS="${REPO_ROOT}/configs"
# Where the built workspace lives. Defaults to an in-place build (ros_ws/install).
# Override if you built elsewhere to avoid OneDrive, e.g.
#   INSTALL_DIR=~/dronetrack_gs/install ./scripts/run_groundstation.sh
INSTALL_DIR="${INSTALL_DIR:-${WS}/install}"
# dronetrack_pi_ros is expected to sit next to this repo.
PI_ROS_SRC="${REPO_ROOT}/../dronetrack_pi_ros/src"
NETWORK_YAML="${CONFIGS}/network.yaml"
[ -f "${NETWORK_YAML}" ] || NETWORK_YAML="${CONFIGS}/network.example.yaml"

# Tiny "key: value" reader for our flat network yaml (strips quotes/comments).
yaml_get() {
  local key="$1" file="${2:-$NETWORK_YAML}"
  sed -n "s/^${key}:[[:space:]]*//p" "$file" | head -n1 | sed 's/#.*//; s/^"//; s/"$//; s/[[:space:]]*$//'
}

export_ros_env() {
  local domain rmw
  domain="$(yaml_get ros_domain_id || true)"
  rmw="$(yaml_get rmw_implementation || true)"
  export ROS_DOMAIN_ID="${domain:-0}"
  export RMW_IMPLEMENTATION="${rmw:-rmw_cyclonedds_cpp}"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
}

# Generate a concrete cyclonedds.xml from the template using network.yaml peers.
# Arg 1 = this host's interface IP to pin (pi_ip on the Pi, laptop_ip on laptop).
# Listing both peers (including self) + pinning the interface is what makes
# discovery work over WSL2 mirrored networking, where multicast is unavailable.
maybe_setup_cyclonedds() {
  local iface="$1" use pi laptop tmpl out
  use="$(yaml_get use_cyclonedds_unicast || echo true)"
  [ "${use}" = "true" ] || { echo "CycloneDDS unicast disabled; using multicast discovery."; return 0; }
  pi="$(yaml_get pi_ip)"; laptop="$(yaml_get laptop_ip)"
  [ -n "${iface}" ] || iface="${laptop}"   # default to laptop IP if not given
  tmpl="${CONFIGS}/cyclonedds.example.xml"; out="${CONFIGS}/cyclonedds.xml"
  sed "s/__IFACE_IP__/${iface}/g; s/__PI_IP__/${pi}/g; s/__LAPTOP_IP__/${laptop}/g" "${tmpl}" > "${out}"
  export CYCLONEDDS_URI="file://${out}"
  echo "CYCLONEDDS_URI=${CYCLONEDDS_URI} (interface=${iface}, peers: ${pi}, ${laptop})"
}

# Stage repo-level configs into a package's config/ dir so default launches work.
stage_configs() {
  local pkg="$1"; shift
  local dst="${WS}/src/${pkg}/config"
  mkdir -p "${dst}"
  for f in "$@"; do cp -f "${CONFIGS}/${f}" "${dst}/"; done
  echo "Staged [$*] into ${pkg}/config/"
}

# Copy a reused package from dronetrack_pi_ros into this workspace if missing.
copy_reused_pkg() {
  local pkg="$1"
  if [ -d "${WS}/src/${pkg}" ]; then echo "  ${pkg}: already present"; return 0; fi
  if [ -d "${PI_ROS_SRC}/${pkg}" ]; then
    cp -r "${PI_ROS_SRC}/${pkg}" "${WS}/src/${pkg}"
    echo "  ${pkg}: copied from dronetrack_pi_ros"
  else
    echo "  ${pkg}: NOT FOUND in ${PI_ROS_SRC} (skip; set the matching launch toggle to false)"
  fi
}

# ---- Pi connection helpers (robust over flaky Wi-Fi) ---------------------
# SSH defaults: auto-accept new host keys (the Pi's IP changes on subnet drift),
# bounded connect timeout, retry the TCP connect, and keepalives so a dead link
# is detected (instead of hanging) within ~30s.
PI_SSH_OPTS=(
  -o BatchMode=yes
  -o StrictHostKeyChecking=accept-new
  -o ConnectTimeout=10
  -o ConnectionAttempts=2
  -o ServerAliveInterval=10
  -o ServerAliveCountMax=3
)

# user@ip for the Pi, read live from network.yaml (kept current by discover_ips.sh).
pi_target() {
  local u ip
  u="$(yaml_get ssh_user)"; ip="$(yaml_get pi_ip)"
  echo "${u:-robotpi}@${ip}"
}

# Run a command on the Pi, retrying transient SSH drops. Tune with PI_SSH_TRIES.
pi_run() {
  local target tries i
  target="$(pi_target)"
  tries="${PI_SSH_TRIES:-4}"
  for ((i = 1; i <= tries; i++)); do
    if ssh "${PI_SSH_OPTS[@]}" "$target" "$@"; then
      return 0
    fi
    if [ "$i" -lt "$tries" ]; then
      echo "  [ssh ${i}/${tries}] ${target} unreachable; retrying in 3s ..." >&2
      sleep 3
    fi
  done
  echo "ERROR: could not reach ${target} after ${tries} attempts (try: bash scripts/discover_ips.sh)" >&2
  return 1
}
