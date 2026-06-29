# Running the PX4 + Gazebo SITL sim on Ubuntu 26.04 (RTX 5080) via Docker

This repo's `px4-gazebo-sim` branch targets **ROS 2 Jazzy + Gazebo Harmonic**
(the same stack your Pi and laptop run). Ubuntu 26.04's *native* ROS is the newer
"Lyrical Luth" with a different Gazebo, so we run Jazzy in a container instead —
same tested stack, GPU rendering via the NVIDIA Container Toolkit.

The whole sim runs **on this PC** (Mode A): PX4 + Gazebo render the drone camera,
YOLO + tracker + control run the pipeline. Your laptop only opens the dashboard
in a browser.

## One time

```bash
# 1. Host: install Docker + NVIDIA Container Toolkit (asks for your password)
sudo bash docker/bootstrap_host.sh
newgrp docker                      # or log out/in, so `docker` works without sudo

# 2. Build the Jazzy image (~10 min)
docker/dt build

# 3. Clone+build PX4, build the repo overlay, create the YOLO venv (~20-40 min)
docker/dt setup
```

## Each session — two terminals

```bash
# Terminal 1 — PX4 + Gazebo (renders the camera on the 5080)
docker/dt px4
#   in the pxh> shell:
#     param set COM_DISARM_PRFLT -1
#     commander arm

# Terminal 2 — the DroneTrack stack
docker/dt run device:=cuda:0 model_path:=~/models/red_ball_yolo26s.pt
#   (no trained model yet? just `docker/dt run` — YOLO falls back to COCO and
#    won't see the ball, but the camera/bridge/pipeline still come up.)
```

Open the dashboard from any machine on the LAN: `http://<this-pc-ip>:8091/`
(on this PC: `http://127.0.0.1:8091/`). Start the mission there.

## Verify

```bash
docker/dt status                              # GPU present? GL renderer = NVIDIA (not llvmpipe)?
docker/dt topic hz /sim/camera/image_raw      # ~10-30 Hz means gz is rendering the camera
docker/dt topic echo /drone/vision/detections # red_ball detections (needs a trained model)
```

## Manage

```bash
docker/dt shell     # bash inside the container (ROS sourced)
docker/dt down      # stop+remove container (keeps image + built PX4/overlay/venv)
docker/dt nuke      # also wipe the home volume -> forces a fresh `dt setup`
```

## Notes / gotchas

- **GPU rendering check:** `docker/dt status` must show an NVIDIA GL renderer. If
  it shows `llvmpipe`, GL isn't reaching the 5080 — re-run `sudo bash
  docker/bootstrap_host.sh` and confirm `nvidia-smi` works in the container.
- **GUI not appearing:** the container talks to your X server via XWayland. `dt`
  runs `xhost +local:` for you; if a window still won't open, run it manually.
- **PX4 version:** `dt setup` tracks PX4 `main`. Pin a release by editing
  `PX4_REF` at the top of `docker/setup_in_container.sh`, then `dt setup` again.
- **YOLO + NumPy:** the venv pins `numpy<2` (ROS `cv_bridge` needs it). If you
  reinstall torch, re-pin `numpy<2` inside the container.
- Builds (PX4, the ROS overlay, the venv) live in the `dronetrack-home` Docker
  volume, so they survive `dt down` and container restarts.
```
