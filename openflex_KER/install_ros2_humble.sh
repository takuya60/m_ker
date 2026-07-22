#!/usr/bin/env bash
set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Run this script with sudo: sudo bash $0" >&2
  exit 1
fi

source /etc/os-release
if [[ "${ID}" != "ubuntu" || "${VERSION_CODENAME}" != "jammy" ]]; then
  echo "This installer supports Ubuntu 22.04 (jammy) only." >&2
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y \
  ca-certificates \
  curl \
  gnupg \
  lsb-release \
  software-properties-common

add-apt-repository -y universe

install -d -m 0755 /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

architecture="$(dpkg --print-architecture)"
echo "deb [arch=${architecture} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${VERSION_CODENAME} main" \
  > /etc/apt/sources.list.d/ros2.list

apt-get update
apt-get install -y \
  ros-humble-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-serial \
  python3-usb \
  python3-vcstool \
  libusb-1.0-0-dev \
  qtbase5-dev

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi

cat <<'EOF'

ROS 2 Humble system packages are installed.

Run the following commands as your normal user:
  source /opt/ros/humble/setup.zsh
  rosdep update
  ros2 --help
  colcon --help
EOF
