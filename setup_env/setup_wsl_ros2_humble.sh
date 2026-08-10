#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
TARGET_HOME="${TARGET_HOME:-/home/daniel}"

apt-get update -y
apt-get install -y software-properties-common curl gnupg lsb-release
apt-get install -y locales

locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

add-apt-repository universe -y

ubuntu_codename="$(
  . /etc/os-release
  echo "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
)"

if [ "$ubuntu_codename" != "jammy" ]; then
  echo "This setup script targets Ubuntu 22.04 (jammy), but detected: ${ubuntu_codename:-unknown}" >&2
  exit 1
fi

rm -f /etc/apt/sources.list.d/ros2.list /usr/share/keyrings/ros-archive-keyring.gpg

curl -L -o /tmp/ros2-apt-source.deb \
  http://repo.ros2.org/ubuntu/main/pool/main/r/ros-apt-source/ros2-apt-source_1.2.0~jammy_all.deb
dpkg -i /tmp/ros2-apt-source.deb

apt-get update -y
apt-get install -y python3-rosdep

if ! command -v rosdep >/dev/null 2>&1; then
  echo "rosdep command was not found after installing python3-rosdep." >&2
  exit 1
fi

rosdep --version

apt-get install -y \
  ros-humble-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-humble-ros-gz \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher-gui \
  ros-humble-turtlesim \
  ros-humble-xacro \
  ros-humble-tf-transformations

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init
fi
rosdep update

grep -qxF "source /opt/ros/humble/setup.bash" "$TARGET_HOME/.bashrc" || \
  echo "source /opt/ros/humble/setup.bash" >> "$TARGET_HOME/.bashrc"

mkdir -p "$TARGET_HOME/ros2_ws/src"

echo "ROS 2 Humble development environment is ready."
