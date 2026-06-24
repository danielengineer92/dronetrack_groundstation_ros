#!/usr/bin/env bash
# Auto-discover the laptop + Pi IPs on the CURRENT subnet and refresh every network
# config. Run on the LAPTOP whenever the LAN /24 drifts (DHCP) and the link breaks.
#
# It finds the Pi by its reserved MAC (pi_mac in network.yaml), so it works no
# matter what subnet the travel router hands out. It then:
#   - rewrites pi_ip / laptop_ip in configs/network.yaml
#   - regenerates the laptop's configs/cyclonedds.xml
#   - pushes a matching cyclonedds_pi.xml to the Pi
# After running it, just launch normally (run_groundstation.sh + the Pi launch).
set -eo pipefail  # not -u: _common.sh / ROS setup reference unbound vars
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

PIMAC="$(yaml_get pi_mac | tr 'A-Z' 'a-z')"
[ -n "$PIMAC" ] || { echo "ERROR: pi_mac not set in ${NETWORK_YAML}"; exit 1; }
SSH_USER="$(yaml_get ssh_user)"; SSH_USER="${SSH_USER:-robotpi}"
PI_OCTET="$(yaml_get pi_ip | awk -F. '{print $4}')"; PI_OCTET="${PI_OCTET:-201}"

LAPTOP_IP="$(ip -4 route get 1.1.1.1 2>/dev/null | grep -oP 'src \K[0-9.]+' || true)"
[ -n "$LAPTOP_IP" ] || { echo "ERROR: could not detect this host's LAN IP"; exit 1; }
PREFIX="${LAPTOP_IP%.*}"
echo "Laptop IP : $LAPTOP_IP   subnet: ${PREFIX}.0/24"

mac_of() { ip neigh show "$1" 2>/dev/null | grep -oiE '([0-9a-f]{2}:){5}[0-9a-f]{2}' | head -1 | tr 'A-Z' 'a-z'; }

PI_IP=""
# Fast path: try the Pi's reserved last octet, confirm by MAC.
CAND="${PREFIX}.${PI_OCTET}"
if ping -c1 -W2 "$CAND" >/dev/null 2>&1 && [ "$(mac_of "$CAND")" = "$PIMAC" ]; then
  PI_IP="$CAND"
  echo "Pi IP     : $PI_IP  (reserved .$PI_OCTET, MAC confirmed)"
else
  echo "Reserved octet missed; sweeping ${PREFIX}.0/24 for $PIMAC ..."
  for _ in 1 2; do
    for i in $(seq 1 254); do ping -c1 -W2 "${PREFIX}.${i}" >/dev/null 2>&1 & done
    wait
    PI_IP="$(ip neigh | grep -i "$PIMAC" | grep -oP '^[0-9.]+' | head -1 || true)"
    if [ -n "$PI_IP" ]; then break; fi
  done
  [ -n "$PI_IP" ] || { echo "ERROR: Pi (MAC $PIMAC) not found on ${PREFIX}.0/24. Powered + on this Wi-Fi?"; exit 1; }
  echo "Pi IP     : $PI_IP  (found by sweep)"
fi

# 1) network.yaml
sed -i -E "s|^pi_ip:.*|pi_ip: \"$PI_IP\"|; s|^laptop_ip:.*|laptop_ip: \"$LAPTOP_IP\"|" "$NETWORK_YAML"
echo "updated   : ${NETWORK_YAML}"

# 2) laptop cyclonedds.xml
TMPL="${CONFIGS}/cyclonedds.example.xml"
sed "s/__IFACE_IP__/${LAPTOP_IP}/g; s/__PI_IP__/${PI_IP}/g; s/__LAPTOP_IP__/${LAPTOP_IP}/g" "$TMPL" \
  > "${CONFIGS}/cyclonedds.xml"
echo "wrote     : ${CONFIGS}/cyclonedds.xml (iface ${LAPTOP_IP})"

# 3) push Pi cyclonedds_pi.xml
PI_GEN="$(mktemp)"
sed "s/__IFACE_IP__/${PI_IP}/g; s/__PI_IP__/${PI_IP}/g; s/__LAPTOP_IP__/${LAPTOP_IP}/g" "$TMPL" > "$PI_GEN"
ssh-keygen -R "$PI_IP" >/dev/null 2>&1 || true
scp -q -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10 \
  "$PI_GEN" "${SSH_USER}@${PI_IP}:/home/${SSH_USER}/cyclonedds_pi.xml"
rm -f "$PI_GEN"
echo "pushed    : ${SSH_USER}@${PI_IP}:~/cyclonedds_pi.xml (iface ${PI_IP})"

echo
echo "Done. PI_IP=$PI_IP  LAPTOP_IP=$LAPTOP_IP"
echo "  ssh ${SSH_USER}@${PI_IP}"
echo "  then launch the Pi stack + run_groundstation.sh"
