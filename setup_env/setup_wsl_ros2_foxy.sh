#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
TARGET_HOME="${TARGET_HOME:-/home/daniel}"

apt-get update -y
apt-get install -y locales software-properties-common curl gnupg2 lsb-release ca-certificates

locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

add-apt-repository universe -y

ubuntu_codename="$(
  . /etc/os-release
  echo "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
)"

if [ "$ubuntu_codename" != "focal" ]; then
  echo "This setup script targets Ubuntu 20.04 (focal), but detected: ${ubuntu_codename:-unknown}" >&2
  exit 1
fi

rm -f /etc/apt/sources.list.d/ros2.list /usr/share/keyrings/ros-archive-keyring.gpg

curl -fsSL -o /usr/share/keyrings/ros-archive-keyring.gpg \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${ubuntu_codename} main" \
  > /etc/apt/sources.list.d/ros2.list

apt-get update -y
apt-get install -y python3-rosdep

if ! command -v rosdep >/dev/null 2>&1; then
  echo "rosdep command was not found after installing python3-rosdep." >&2
  exit 1
fi

rosdep --version

apt-get install -y \
  ros-foxy-desktop \
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-foxy-gazebo-ros-pkgs \
  ros-foxy-robot-state-publisher \
  ros-foxy-joint-state-publisher-gui \
  ros-foxy-ros2-control \
  ros-foxy-ros2-controllers \
  ros-foxy-turtlesim \
  ros-foxy-xacro

if apt-cache show ros-foxy-moveit >/dev/null 2>&1; then
  apt-get install -y ros-foxy-moveit
fi

if apt-cache show ros-foxy-tf-transformations >/dev/null 2>&1; then
  apt-get install -y ros-foxy-tf-transformations
fi

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init
fi
rosdep update --include-eol-distros

grep -qxF "source /opt/ros/foxy/setup.bash" "$TARGET_HOME/.bashrc" || \
  echo "source /opt/ros/foxy/setup.bash" >> "$TARGET_HOME/.bashrc"

mkdir -p "$TARGET_HOME/ros2_ws/src"

echo "ROS 2 Foxy development environment is ready."
