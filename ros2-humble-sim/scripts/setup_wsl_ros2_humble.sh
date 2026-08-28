#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe -y

ubuntu_codename="$(
  . /etc/os-release
  echo "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
)"

if [ "$ubuntu_codename" != "jammy" ]; then
  echo "This setup script targets Ubuntu 22.04 (jammy), but detected: ${ubuntu_codename:-unknown}" >&2
  exit 1
fi

sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg

curl -L -o /tmp/ros2-apt-source.deb \
  http://repo.ros2.org/ubuntu/main/pool/main/r/ros-apt-source/ros2-apt-source_1.2.0~jammy_all.deb
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y python3-rosdep

hash -r

if ! command -v rosdep >/dev/null 2>&1; then
  echo "rosdep command was not found after installing python3-rosdep." >&2
  echo "Check with: dpkg -L python3-rosdep | grep bin" >&2
  exit 1
fi

rosdep --version

sudo apt install -y \
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
  sudo rosdep init
fi
rosdep update

grep -qxF "source /opt/ros/humble/setup.bash" "$HOME/.bashrc" || \
  echo "source /opt/ros/humble/setup.bash" >> "$HOME/.bashrc"

mkdir -p "$HOME/ros2_ws/src"

echo "ROS 2 Humble development environment is ready."
echo "Open a new shell or run: source /opt/ros/humble/setup.bash"
