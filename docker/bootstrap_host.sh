#!/usr/bin/env bash
# One-time host setup for the DroneTrack SITL container (Ubuntu 26.04).
# Installs Docker Engine + the NVIDIA Container Toolkit and wires the NVIDIA
# runtime into Docker so the RTX 5080 is usable inside containers.
#
# RUN WITH SUDO:  sudo bash docker/bootstrap_host.sh
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "Run me with sudo:  sudo bash docker/bootstrap_host.sh" >&2
  exit 1
fi

# The login user (not root) — for the docker group.
TARGET_USER="${SUDO_USER:-$(logname 2>/dev/null || echo "")}"
[ -n "${TARGET_USER}" ] || { echo "Could not determine your login user; set TARGET_USER=..." >&2; exit 1; }

echo "==> [1/5] Docker Engine (docker.io from Ubuntu repos — has a 26.04 build)"
apt-get update
apt-get install -y docker.io curl gnupg ca-certificates

echo "==> [2/5] NVIDIA Container Toolkit repo (distro-agnostic 'stable' channel)"
install -m 0755 -d /usr/share/keyrings
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  > /etc/apt/sources.list.d/nvidia-container-toolkit.list

echo "==> [3/5] Install nvidia-container-toolkit"
apt-get update
apt-get install -y nvidia-container-toolkit

echo "==> [4/5] Configure Docker to use the NVIDIA runtime"
nvidia-ctk runtime configure --runtime=docker
systemctl enable --now docker
systemctl restart docker

echo "==> [5/5] Add '${TARGET_USER}' to the docker group"
usermod -aG docker "${TARGET_USER}"

echo
echo "=========================================================================="
echo "Done. Verifying GPU access from a container..."
if docker run --rm --gpus all ubuntu:24.04 nvidia-smi >/tmp/dt_gpu_check.txt 2>&1; then
  echo "GPU OK inside containers:"
  grep -m1 "RTX 5080\|Driver Version" /tmp/dt_gpu_check.txt || true
else
  echo "WARN: GPU smoke test failed — see /tmp/dt_gpu_check.txt"
fi
echo "=========================================================================="
echo
echo "IMPORTANT: group membership only applies to NEW logins."
echo "Either log out and back in, or in your working terminal run:  newgrp docker"
echo "Then continue with:  docker/dt build"
