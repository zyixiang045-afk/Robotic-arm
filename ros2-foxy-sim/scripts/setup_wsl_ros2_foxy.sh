#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y locales software-properties-common curl gnupg2 lsb-release ca-certificates

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo add-apt-repository universe -y

ubuntu_codename="$(
  . /etc/os-release
  echo "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
)"

if [ "$ubuntu_codename" != "focal" ]; then
  echo "This setup script targets Ubuntu 20.04 (focal), but detected: ${ubuntu_codename:-unknown}" >&2
  exit 1
fi

sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg

sudo curl -fsSL -o /usr/share/keyrings/ros-archive-keyring.gpg \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${ubuntu_codename} main" \
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
  sudo apt install -y ros-foxy-moveit
else
  echo "ros-foxy-moveit was not found in apt; skipping optional MoveIt 2 package."
fi

if apt-cache show ros-foxy-tf-transformations >/dev/null 2>&1; then
  sudo apt install -y ros-foxy-tf-transformations
else
  echo "ros-foxy-tf-transformations was not found in apt; skipping optional tf transformations package."
fi

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update --include-eol-distros

grep -qxF "source /opt/ros/foxy/setup.bash" "$HOME/.bashrc" || \
  echo "source /opt/ros/foxy/setup.bash" >> "$HOME/.bashrc"

mkdir -p "$HOME/ros2_ws/src"

echo "ROS 2 Foxy development environment is ready."
echo "Open a new shell or run: source /opt/ros/foxy/setup.bash"
