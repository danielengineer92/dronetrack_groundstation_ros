# DroneTrack — Split Ground-Station Architecture

A two-machine derivative of [`dronetrack_pi_ros`](../dronetrack_pi_ros). The
**Raspberry Pi stays on the drone** and owns everything safety-critical (arming,
failsafes, flight control, MAVSDK/PX4, mission sequencing, watchdogs). The
**laptop is the ground station** and runs the compute-heavy, user-facing nodes
(YOLO inference, web dashboard).

> **Safety rule:** the laptop only sends perception + advisory operator requests.
> The Pi validates everything and holds position safely if the laptop
> disconnects. See [`docs/safety_contract.md`](docs/safety_contract.md).

ROS 2 **Jazzy**. Read [`docs/architecture.md`](docs/architecture.md) first for the
full picture, then [`docs/networking.md`](docs/networking.md).

## Layout

```
dronetrack_groundstation_ros/
  docs/            architecture, networking, safety contract, migration
  configs/         pi.yaml, groundstation.yaml, topics.yaml, network.example.yaml, cyclonedds.example.xml
  scripts/         setup_*.sh, run_*.sh, sanity_checks.sh
  ros_ws/src/
    dronetrack_msgs/          GroundStationHeartbeat, LinkStatus            (new)
    dronetrack_pi/            detection gate + ground-station watchdog       (new, runs on Pi)
    dronetrack_perception/    laptop YOLO node                              (adapted)
    dronetrack_groundstation/ laptop heartbeat + bringup launch             (new)
    dronetrack_web_bridge/    laptop web dashboard                          (adapted)
```

`drone_interfaces` and the reused Pi packages (`drone_camera`, `drone_tracker`,
`drone_control`, `drone_telemetry`, `drone_diagnostics`) are **copied in from
`dronetrack_pi_ros` by the setup scripts** — they are not duplicated in git here.

## Prerequisites

- Both machines on the **same travel router LAN** with reserved IPs (see
  [networking](docs/networking.md)). No laptop hotspot.
- `dronetrack_pi_ros` checked out **next to** this repo.
- ROS 2 Jazzy on each machine. Windows laptop → use **WSL2 Ubuntu 24.04**.
- `cp configs/network.example.yaml configs/network.yaml` and set `pi_ip`,
  `laptop_ip`, and a shared `ros_domain_id` on **both** machines.

---

## Pi setup (on the drone)

```bash
cd dronetrack_groundstation_ros
cp configs/network.example.yaml configs/network.yaml      # edit IPs + ros_domain_id
sudo apt install ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-compressed-image-transport
./scripts/setup_pi.sh                                      # copy reused pkgs, build
```

## Laptop setup (ground station)

```bash
cd dronetrack_groundstation_ros
cp configs/network.example.yaml configs/network.yaml      # SAME IPs + ros_domain_id as Pi
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
./scripts/setup_groundstation.sh                          # copy drone_interfaces, pip YOLO deps, build
# copy your model over, e.g. from the Pi repo:
cp -r ../dronetrack_pi_ros/models/red_ball_ncnn_model ./models/red_ball_ncnn_model
```

> **NumPy pin (important):** ROS Jazzy `cv_bridge` is built against NumPy 1.x.
> `ultralytics`/`torch` tend to pull NumPy 2.x, which breaks `import cv_bridge`
> (and YOLO). `setup_groundstation.sh` re-pins `numpy<2` for you; if you later
> reinstall torch (e.g. a CUDA build), re-pin afterward:
> `python3 -m pip install --user --break-system-packages "numpy<2"`.
>
> **NVIDIA GPU (optional, ~3x faster):** install a CUDA torch build, re-pin
> `numpy<2`, then launch YOLO with `device:=cuda:0 half_precision:=True`. See
> `scripts/setup_groundstation.sh` for the exact commands.

---

## Running the Pi nodes

```bash
./scripts/run_pi.sh connection_url:=serial:///dev/ttyACM0:57600
```

This brings up the camera + compressed stream, detection gate, ground-station
watchdog, tracker, mission/autonomy/control, the MAVSDK bridge, and the health
monitor. Useful args:

```bash
./scripts/run_pi.sh \
  connection_url:=serial:///dev/ttyACM0:57600 \
  allow_mavsdk_actions:=false \   # keep false for bench work
  reused_pi_nodes:=true \         # set false to bench just the new boundary nodes
  compress:=true
```

## Running the laptop YOLO

The ground-station launch starts YOLO + heartbeat + dashboard together:

```bash
./scripts/run_groundstation.sh \
  model_path:=$PWD/models/red_ball_ncnn_model \
  target_class:=red_ball \
  device:=cpu                     # or cuda:0 / mps if the laptop has a GPU
```

To run **only** YOLO (no dashboard) for debugging:

```bash
source ros_ws/install/setup.bash
ros2 run dronetrack_perception yolo_node --ros-args \
  --params-file configs/groundstation.yaml \
  -p model_path:=$PWD/models/red_ball_ncnn_model -p target_class:=red_ball
```

## Running the website (dashboard)

The dashboard launches with `run_groundstation.sh`. Open:

```
http://127.0.0.1:8080/
```

> On a Windows laptop running the ground station in WSL2 (mirrored networking),
> use **`127.0.0.1`**, not `localhost` — Windows resolves `localhost` to IPv6
> `::1`, which mirrored WSL does not bridge, so it times out. `127.0.0.1` works.
> On a native-Linux ground station, `localhost` is fine.

It shows connection state, perception FPS, estimated latency, detection
confidence, and drone status, plus operator buttons (System Ready / Start Mission
/ Abort-Hold / Land). Those buttons publish **advisory request topics only** —
the Pi re-validates them. Run dashboard standalone:

```bash
ros2 run dronetrack_web_bridge web_dashboard_node --ros-args \
  --params-file configs/groundstation.yaml -p port:=8080
```

---

## Verifying communication

One script covers all checks (run from either machine, both sides up):

```bash
./scripts/sanity_checks.sh                 # all checks
./scripts/sanity_checks.sh ping ssh ros    # a subset
```

Or run them by hand:

```bash
# 1. Ping
ping 10.0.0.10                                              # Pi IP from network.yaml

# 2. SSH
ssh robotpi@10.0.0.10 hostname

# 3. ROS topic visibility (same ROS_DOMAIN_ID on both!)
ros2 topic list

# 4. Camera stream (Pi -> laptop)
ros2 topic hz /drone/camera/image_raw/compressed

# 5. YOLO FPS (laptop)
ros2 topic hz /groundstation/vision/detections
#   (or read the yolo_node log line "YOLO | fps=...")

# 6. Detection round trip (laptop -> Pi gate -> internal topic)
ros2 topic hz /drone/vision/detections     # should track the inbound rate when link is healthy

# 7. Link status + latency
ros2 topic echo --once /drone/groundstation/link_status
```

**Link-loss test (the key safety check):** with detections flowing, kill the
laptop launch. Within ~1 s the Pi logs `GROUND STATION LINK LOST`,
`/drone/groundstation/ok` flips to `false`, `/drone/vision/detections` stops, and
the drone holds position.

### Hardware-free self-test

`scripts/selftest_boundary.sh` exercises the whole safety boundary with no PX4,
camera, or YOLO. It runs the heartbeat, detection gate, and watchdog, feeds fake
detections, and asserts the healthy-link and link-loss behaviors:

```bash
bash scripts/selftest_boundary.sh            # builds expected at ros_ws/install
# or, if you built into a different tree:
INSTALL_DIR=~/dronetrack_gs/install bash scripts/selftest_boundary.sh
```

Expected tail: `SELFTEST: ALL PASS` (exit 0).

---

## What was copied, stubbed, and what still needs hardware testing

**Reused unchanged (copied from `dronetrack_pi_ros` by setup scripts):**
`drone_interfaces`, `drone_camera`, `drone_tracker`, `drone_control`,
`drone_telemetry`, `drone_diagnostics`.

**Adapted (logic preserved, boundary/topics changed):**
- `dronetrack_perception/yolo_node.py` — from `drone_yolo`; now compressed-stream
  in, untrusted-topic out, preserves the camera stamp for latency.
- `dronetrack_web_bridge/web_dashboard_node.py` — from `drone_dashboard`; now
  shows link/FPS/latency/confidence/drone-status and posts advisory requests.

**New code (written here, runnable):**
- `dronetrack_msgs` (`GroundStationHeartbeat`, `LinkStatus`).
- `detection_gate_node`, `ground_station_watchdog_node` (Pi).
- `groundstation_heartbeat_node` (laptop).
- All launch files, configs, and scripts.

**Not ported (use from the old repo if needed):** `drone_visualizer`,
`drone_fake`, the OpenCV `color_detection_node`.

**Verified on ROS 2 Jazzy (WSL2 Ubuntu 24.04):**
- `colcon build` of all 11 packages (msgs + new + reused) — clean, exit 0.
- The safety boundary via `scripts/selftest_boundary.sh` — gate validation +
  republish, healthy-link `link_ok=true/OK`, link-loss `link_ok=false/
  HEARTBEAT_STALE`, watchdog de-assert of autonomy/offboard requests, and the
  `DETECTIONS_STALE` vs `HEARTBEAT_STALE` distinction. All assertions pass.

**Still needs real hardware/SITL testing (not validated yet):**
- Two-machine run over the actual travel-router LAN (DDS discovery / CycloneDDS
  unicast, `ROS_DOMAIN_ID` match) — the self-test above ran both sides on one host.
- Compressed `image_transport republish` topic names on your ROS build (verify
  with check #4) and real camera frames.
- End-to-end latency under real Wi-Fi, and clock sync via chrony.
- The link-loss → hold behavior on the actual airframe with PX4 connected (test
  on the bench first, props off, `allow_mavsdk_actions:=false`).
- YOLO `device`/model throughput on your specific laptop (needs `ultralytics`
  installed; on Ubuntu 24.04 use a venv or `pip install --break-system-packages`).

> **Build location note:** building under `/mnt/c` (OneDrive) is slow and OneDrive
> will try to sync `build/`. The verification build used colcon's `--build-base`/
> `--install-base` pointed at the WSL native filesystem (`~/dronetrack_gs`). The
> `setup_*.sh` scripts build in-place under `ros_ws/`; if that's sluggish, pass
> `--build-base ~/... --install-base ~/...` or relocate the repo into `~`.
