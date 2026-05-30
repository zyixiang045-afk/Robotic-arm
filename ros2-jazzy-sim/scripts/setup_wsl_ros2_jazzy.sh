#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

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
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-jazzy-ros-gz \
  ros-jazzy-moveit \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-tf-transformations

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

grep -qxF "source /opt/ros/jazzy/setup.bash" "$HOME/.bashrc" || \
  echo "source /opt/ros/jazzy/setup.bash" >> "$HOME/.bashrc"

mkdir -p "$HOME/ros2_ws/src"

echo "ROS 2 Jazzy development environment is ready."
echo "Open a new shell or run: source /opt/ros/jazzy/setup.bash"
