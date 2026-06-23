#!/usr/bin/env bash
# One-time ROS 2 Jazzy install for a bare Ubuntu 24.04 (noble) WSL distro.
# Run this yourself with sudo (it needs your password, which the assistant
# cannot enter for you):
#
#     sudo bash scripts/install_ros_wsl.sh
#
# Installs ros-base (headless) plus exactly the extra packages this project
# needs. After it finishes, the assistant can build the workspace.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "Please run with sudo:  sudo bash scripts/install_ros_wsl.sh" >&2
  exit 1
fi

RUN_USER="${SUDO_USER:-root}"
echo "== Installing ROS 2 Jazzy for user: ${RUN_USER} =="

export DEBIAN_FRONTEND=noninteractive

echo "[1/6] Base tools + universe repo"
apt update
apt install -y curl gnupg lsb-release software-properties-common locales
add-apt-repository -y universe

echo "[2/6] Locale (UTF-8)"
locale-gen en_US en_US.UTF-8 || true
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || true

echo "[3/6] ROS 2 apt key + repo"
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  > /etc/apt/sources.list.d/ros2.list

echo "[4/6] apt update"
apt update

echo "[5/6] Install ROS Jazzy (ros-base) + project deps"
apt install -y \
  ros-jazzy-ros-base \
  ros-dev-tools \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-compressed-image-transport \
  ros-jazzy-rmw-cyclonedds-cpp \
  python3-pip python3-colcon-common-extensions

echo "[6/6] rosdep init/update"
rosdep init 2>/dev/null || echo "  (rosdep already initialized)"
sudo -u "${RUN_USER}" rosdep update || echo "  (rosdep update warning; non-fatal)"

# Convenience: source ROS in this user's future shells.
PROFILE="/home/${RUN_USER}/.bashrc"
if [ -f "${PROFILE}" ] && ! grep -q "/opt/ros/jazzy/setup.bash" "${PROFILE}"; then
  echo "source /opt/ros/jazzy/setup.bash" >> "${PROFILE}"
  echo "Added ROS source line to ${PROFILE}"
fi

echo
echo "ROS 2 Jazzy installed. Verify:"
echo "  source /opt/ros/jazzy/setup.bash && ros2 --help | head"
echo "Then tell the assistant to finish the build."
