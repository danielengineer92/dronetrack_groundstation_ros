# Gazebo SITL — setup & run (the 5080 PC and the laptop)

Track a moving red ball in Gazebo with the real DroneTrack vision pipeline
(camera → YOLO → detection gate → tracker → control → PX4). Two ways to run:

- **Mode A — all on the PC (simplest, recommended).** The 5080 renders gz, runs
  YOLO, and runs the whole stack. The laptop just opens the dashboard in a
  browser. No cross-machine ROS setup.
- **Mode B — split.** The 5080 renders + owns the ball and publishes the camera
  over the LAN; the laptop runs YOLO + the stack. Use this only if you want the
  laptop in the loop.

> Rendering note: Gazebo camera rendering needs a working GPU GL stack. It is
> reliable on the 5080 (native Linux or a good GPU). It is flaky on the laptop's
> WSL2 — that's the whole reason the sim lives on the PC.

---

## 1. One-time setup on the 5080 PC

```bash
# ROS 2 Jazzy must already be installed. Then:
sudo apt update && sudo apt install -y \
    ros-jazzy-ros-gz ros-jazzy-ros-gz-image ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces ros-jazzy-ros-gz-sim \
    ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cv-bridge

# PX4 with the camera model
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
cd ~/PX4-Autopilot && make px4_sitl gz_x500_mono_cam

# This repo + build the SITL overlay (also creates ~/ros_venv with YOLO + numpy)
cd <this-repo>
git checkout px4-gazebo-sim
scripts/ros_wsl.sh build-sim       # builds, then auto-creates ~/ros_venv
```

Put your trained red-ball model somewhere on the PC, e.g.
`~/models/red_ball_yolo26s.pt` (referenced as `model_path:=` below). Without it,
YOLO runs a generic COCO model and won't see the ball.

---

## 2. Mode A — everything on the 5080 (recommended)

**Terminal 1 — PX4 + Gazebo (renders the camera):**
```bash
cd ~/PX4-Autopilot && make px4_sitl gz_x500_mono_cam
# in the PX4 (pxh>) shell, let it arm in SITL:
#   param set COM_DISARM_PRFLT -1
#   commander arm
```

**Terminal 2 — the DroneTrack stack (bridge + ball + YOLO + tracker + control):**
```bash
cd <this-repo>
scripts/ros_wsl.sh gazebo device:=cuda:0 model_path:=~/models/red_ball_yolo26s.pt
# native Linux equivalent:
#   ros2 launch dronetrack_pi sitl_gazebo_launch.py device:=cuda:0 model_path:=~/models/red_ball_yolo26s.pt
```
This spawns the red ball, orbits it, runs YOLO, and runs the whole pipeline.

**From the laptop (or any machine on the LAN):** open the dashboard at
`http://<5080-ip>:8091/` and start the mission there.

> If the camera FPS tanks while YOLO runs (GPU contention), add `device:=cpu`.

---

## 3. Mode B — split (5080 renders, laptop runs the stack)

**On the 5080 — producer only (camera + ball, no stack):**
```bash
# Terminal 1
cd ~/PX4-Autopilot && make px4_sitl gz_x500_mono_cam
# Terminal 2
ros2 launch dronetrack_pi sim_producer.launch.py
```
This publishes `/sim/camera/image_raw` over the LAN and orbits the ball.

**On the laptop — consumer (YOLO + stack, no local gz):**
```bash
source /opt/ros/jazzy/setup.bash
source ~/dronetrack_groundstation_ros_sim/install/setup.bash
export ROS_DOMAIN_ID=9 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # MUST match the 5080
scripts/ros_wsl.sh gazebo local_sim:=false device:=cpu \
    model_path:=/mnt/c/Users/danie/Documents/Python/dronetrack_groundstation_ros/red_ball_yolo26s.pt \
    connection_url:=udp://<5080-ip>:14540
```
Dashboard: `http://127.0.0.1:8091/`.

**Both machines must agree (Mode B only):**
- Same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` (use `rmw_cyclonedds_cpp` on both).
- Same LAN; multicast DDS discovery reachable.
- Laptop WSL2 inbound firewall must allow DDS (one-time, already applied on this
  laptop): admin PowerShell
  `Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow`.
- `connection_url` points the laptop's telemetry at the 5080's PX4 MAVLink.

---

## 4. Verify it's working

```bash
# camera coming out of gz?
gz topic -l | grep -i image
ros2 topic hz /sim/camera/image_raw            # ~10-30 Hz
# YOLO seeing the ball?
ros2 topic echo /drone/vision/detections       # red_ball detections
# tracker locking?
ros2 topic echo /drone/tracking/target_error   # tracking_state -> LOCKED, target_visible: true
```
Then start the mission from the dashboard and watch the drone orbit the ball.

---

## 5. Troubleshooting

- **Camera 0 fps / `glxinfo -B` shows `llvmpipe`** → GPU rendering isn't active.
  On WSL2 this is expected/flaky — run the sim on the 5080 with a real GPU.
- **No detections** → confirm `target_class:=red_ball` and a red-ball
  `model_path`; check `ros2 topic hz /sim/camera/image_raw` first.
- **`gz topic -l` has no camera** → the gz camera topic name differs from the
  default; find it with `gz topic -l | grep image` and pass `gz_camera_topic:=…`.
- **Split: laptop sees nothing** → mismatched `ROS_DOMAIN_ID`/RMW, firewall, or
  not on the same LAN. Verify with `ros2 topic list` on the laptop showing
  `/sim/camera/image_raw`.
- **Drone won't move** → it must be armed and the mission started; in the PX4
  shell `commander arm`, then start the mission from the dashboard.
