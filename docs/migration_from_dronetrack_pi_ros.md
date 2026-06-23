# Migration from `dronetrack_pi_ros`

This repo does **not** replace `dronetrack_pi_ros`. It reuses it. The old repo
stays the home of the safety-critical flight code; this repo adds the split-mode
wiring and the laptop-side perception/dashboard.

## What moved where

| Piece in `dronetrack_pi_ros` | In this repo |
|---|---|
| `drone_interfaces` (messages) | **Reused unchanged.** Copied into `ros_ws/src/` by the setup scripts so the laptop and Pi share identical message types (wire compatibility). |
| `drone_camera` | **Reused on the Pi.** Now followed by a compressed republish for Wi-Fi. |
| `drone_tracker`, `drone_control`, `drone_telemetry`, `drone_diagnostics` | **Reused on the Pi, unchanged.** Copied in by `setup_pi.sh`. |
| `drone_yolo/yolo_node.py` | **Adapted → `dronetrack_perception` (laptop).** Same detection logic; now subscribes the compressed stream and publishes to the untrusted `/groundstation/vision/detections`. |
| `drone_dashboard/dashboard_node.py` | **Adapted → `dronetrack_web_bridge` (laptop).** Same stdlib-HTTP approach; now surfaces link/FPS/latency/confidence/drone-status and posts only advisory request topics. |
| `drone_visualizer` | Optional; can be run on the laptop later. Not ported in this pass. |
| `drone_fake` | Not ported. Use it from the old repo for SITL/fake-data testing. |
| `drone_bringup` launch/config | Split into `dronetrack_pi/launch/pi_launch.py` + `configs/pi.yaml` (Pi) and `dronetrack_groundstation/launch/groundstation_launch.py` + `configs/groundstation.yaml` (laptop). |

## New code added

- `dronetrack_msgs` — `GroundStationHeartbeat`, `LinkStatus`.
- `dronetrack_pi/detection_gate_node.py` — validates laptop detections, republishes to `/drone/vision/detections`.
- `dronetrack_pi/ground_station_watchdog_node.py` — heartbeat monitor + safe link-loss reaction.
- `dronetrack_groundstation/heartbeat_node.py` — laptop liveness beacon.

## Why reuse `drone_interfaces` instead of renaming into `dronetrack_msgs`

ROS 2 message identity is `package_name/MessageName`. If the laptop published
`dronetrack_msgs/DetectionArray` while the Pi tracker expected
`drone_interfaces/DetectionArray`, they would **not** connect. Keeping the shared
types in `drone_interfaces` guarantees the laptop and an existing Pi remain wire
compatible. `dronetrack_msgs` therefore holds only the genuinely new boundary
messages.

## Step-by-step

Assuming the two repos sit side by side:

```
Documents/Python/
  dronetrack_pi_ros/                 # existing, untouched
  dronetrack_groundstation_ros/      # this repo
```

### On the Pi (drone)

```bash
cd dronetrack_groundstation_ros
cp configs/network.example.yaml configs/network.yaml   # edit IPs/domain
./scripts/setup_pi.sh        # copies reused pkgs + drone_interfaces, builds
source ros_ws/install/setup.bash
./scripts/run_pi.sh connection_url:=serial:///dev/ttyACM0:57600
```

### On the laptop (ground station, Linux/WSL2)

```bash
cd dronetrack_groundstation_ros
cp configs/network.example.yaml configs/network.yaml   # same IPs/domain as Pi
./scripts/setup_groundstation.sh    # copies drone_interfaces, installs YOLO deps, builds
source ros_ws/install/setup.bash
./scripts/run_groundstation.sh model_path:=$PWD/models/red_ball_ncnn_model target_class:=red_ball
```

Copy your model (e.g. `dronetrack_pi_ros/models/red_ball_ncnn_model`) onto the
laptop and point `model_path` at it.

## Rollback

This migration is non-destructive. To go back to all-on-Pi operation, just run
`dronetrack_pi_ros` as before — it is unchanged. Nothing here writes into the old
repo; the setup scripts only **read/copy** from it.

## Verification after migration

```bash
./scripts/sanity_checks.sh        # ping, ssh, ros topics, camera, yolo, round trip, latency
```

Then confirm the safety property by hand: with both sides up and detections
flowing, kill the laptop's `run_groundstation.sh`. Within ~1 s the Pi should log
`GROUND STATION LINK LOST`, `/drone/groundstation/ok` should flip to `false`, and
`/drone/vision/detections` should stop updating while the drone holds position.
