# DroneTrack SITL — ROS 2 Jazzy + Gazebo Harmonic in a container.
# Matches the Jazzy stack the Pi/laptop run, so the repo scripts work unchanged
# even though the host is Ubuntu 26.04 (whose native ROS is Lyrical Luth).
#
# GPU rendering (RTX 5080) is provided at *run* time by the NVIDIA Container
# Toolkit (--gpus all + NVIDIA_DRIVER_CAPABILITIES=all); nothing GPU-specific is
# baked here.
FROM osrf/ros:jazzy-desktop-full

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=ros
ENV DEBIAN_FRONTEND=noninteractive

# --- OSRF Gazebo apt repo (hosts gz-harmonic, which ros-gz depends on) -------
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release ca-certificates \
  && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
  && rm -rf /var/lib/apt/lists/*

# --- ROS / Gazebo / build tooling -------------------------------------------
# ros-gz on Jazzy pulls Gazebo Harmonic (the version PX4 + this repo target).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      ros-jazzy-ros-gz-bridge \
      ros-jazzy-ros-gz-sim \
      ros-jazzy-ros-gz-interfaces \
      ros-jazzy-ros-gz-image \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-cv-bridge \
      ros-jazzy-image-transport \
      ros-jazzy-compressed-image-transport \
      python3-colcon-common-extensions \
      python3-venv python3-pip python3-dev python3-vcstool \
      build-essential cmake ninja-build ccache git wget curl \
      libxml2-dev libxslt1-dev \
      gz-harmonic \
      mesa-utils \
      sudo nano less \
  && rm -rf /var/lib/apt/lists/*

# --- non-root user matching the host UID/GID (clean file ownership on mounts) -
RUN (getent group ${USER_GID} || groupadd -g ${USER_GID} ${USERNAME}) \
  && (getent passwd ${USER_UID} || useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash ${USERNAME}) \
  && usermod -aG sudo $(getent passwd ${USER_UID} | cut -d: -f1) \
  && echo "%sudo ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/nopasswd \
  && chmod 0440 /etc/sudoers.d/nopasswd

# Source ROS in every interactive shell.
RUN U=$(getent passwd ${USER_UID} | cut -d: -f1) H=$(getent passwd ${USER_UID} | cut -d: -f6) \
  && echo 'source /opt/ros/jazzy/setup.bash' >> ${H}/.bashrc \
  && chown ${USER_UID}:${USER_GID} ${H}/.bashrc

ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    QT_X11_NO_MITSHM=1

USER ${USER_UID}
WORKDIR /home/${USERNAME}
CMD ["bash"]
