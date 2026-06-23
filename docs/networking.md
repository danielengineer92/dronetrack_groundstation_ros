# Networking

Both the Pi and the laptop join the **same travel router LAN**. We do **not** use
a laptop hotspot. Everything is keyed off reserved IPs (or stable hostnames) and
a shared `ROS_DOMAIN_ID`.

## 1. Reserve addresses on the travel router

In the router admin page, add DHCP reservations (by MAC) or set static IPs:

| Device | Example IP | Example hostname |
|---|---|---|
| Pi (drone) | `10.0.0.10` | `dronepi.local` |
| Laptop (ground station) | `10.0.0.20` | `groundstation.local` |

Copy `configs/network.example.yaml` to `configs/network.yaml` and fill these in.
The scripts read that file:

```bash
cp configs/network.example.yaml configs/network.yaml
# edit pi_ip / laptop_ip / ros_domain_id
```

## 2. Shared ROS 2 settings

Both machines MUST agree on:

- `ROS_DOMAIN_ID` (default `42` in the example) — isolates your DDS traffic.
- `RMW_IMPLEMENTATION` (default `rmw_cyclonedds_cpp`).

`scripts/_common.sh::export_ros_env` exports these from `network.yaml`, and the
`run_*.sh` scripts call it for you.

## 3. DDS discovery over a travel router

Many travel routers block or rate-limit multicast, which breaks default DDS
discovery. The robust fix is **CycloneDDS with explicit unicast peers**:

- `configs/cyclonedds.example.xml` is a template with `__PI_IP__` / `__LAPTOP_IP__`.
- `run_*.sh` generate `configs/cyclonedds.xml` from it using `network.yaml`, set
  `AllowMulticast=false`, and export `CYCLONEDDS_URI`.

Toggle with `use_cyclonedds_unicast` in `network.yaml`:

- `true`  → unicast peers (recommended on travel routers).
- `false` → rely on multicast (only if your router passes it).

Install CycloneDDS once per machine:

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

## 4. Clock sync (needed for latency numbers)

The dashboard's latency figure compares the laptop's message stamp against the
Pi's receive time, so the two clocks must agree. Use chrony/NTP:

```bash
sudo apt install chrony
# point both at the same NTP source, or make the Pi a local NTP server
```

Without sync the system still works (gate uses *relative* ages within each
machine where possible), but the reported latency will be meaningless.

## 5. Windows laptop / WSL2 ground station (VERIFIED procedure)

ROS 2 Jazzy targets Linux, so on a Windows 11 laptop run the ground station inside
**WSL2 (Ubuntu 24.04)**. The following was needed to get bidirectional DDS working
between WSL2 and the Pi — default WSL2 NAT does **not** work (the Pi cannot reach
back into WSL):

1. **Mirrored networking.** Put this in `%USERPROFILE%\.wslconfig`, then
   `wsl --shutdown` and reopen WSL:
   ```ini
   [wsl2]
   networkingMode=mirrored
   firewall=false
   ```
   Mirrored mode makes WSL share the host's LAN IP so the Pi can reach it.
   `firewall=false` drops the Hyper-V firewall layer that otherwise blocks inbound
   DDS. (Leave it on and add a scoped allow rule if you prefer — see step 2.)

2. **Windows Firewall (if keeping `firewall=true`).** Allow inbound from the bench
   subnet only, e.g. a rule `ROS2-Pi-Inbound` with
   `RemoteAddresses = 10.166.69.0/255.255.255.0`.

3. **Use CycloneDDS with unicast peers — not FastDDS.** FastDDS multicast discovery
   does **not** traverse mirrored WSL. On **both** machines export:
   ```bash
   export ROS_DOMAIN_ID=0
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export CYCLONEDDS_URI=file://$HOME/cyclonedds_<laptop|pi>.xml
   ```
   The Pi runs the original `dronetrack_pi_ros` stack, so it must use the **same**
   RMW — if the Pi defaults to FastDDS, set CycloneDDS there too or they won't talk.

4. **CycloneDDS config: pin the interface and list BOTH peers (including self).**
   See `configs/cyclonedds.example.xml`. On the laptop config set
   `<NetworkInterfaceAddress>` to the laptop LAN IP and list peers `<laptop-ip>` AND
   `<pi-ip>`; on the Pi config pin the Pi IP and list the same two peers. Listing the
   local host as a peer is required for same-host discovery under mirrored WSL.

5. **Clock sync** (step 4 of the chrony section) is needed for the dashboard latency
   number to be meaningful — without it `link_status.estimated_latency_s` reflects
   the Pi↔laptop clock offset, not real latency. Heartbeat freshness is unaffected
   (it uses receipt time).

Verified result: compressed camera Pi→laptop, detections laptop→Pi, and the Pi's
gated `/drone/vision/detections` + `/drone/groundstation/link_status` (`link_ok:
true`) all cross correctly.

## 6. Bandwidth

Stream **compressed** camera only. Raw `640x480 bgr8 @ 30 Hz` is ~26 MB/s and
will saturate Wi-Fi; JPEG-compressed is typically <1 MB/s. The Pi launch runs
`image_transport republish raw compressed` and the laptop YOLO subscribes to the
`/compressed` topic by default. Drop `frame_width/height/fps` in `configs/pi.yaml`
if the link is weak.

## 7. Quick verification

```bash
scripts/sanity_checks.sh ping ssh ros camera
```

See the README "Verifying communication" section for the full checklist.
