#!/usr/bin/env bash
set -euo pipefail

if [ -f /opt/ros/foxy/setup.bash ]; then
  set +u
  source /opt/ros/foxy/setup.bash
  set -u
else
  echo "ROS 2 Foxy setup file was not found at /opt/ros/foxy/setup.bash" >&2
  exit 1
fi

echo "ROS_DISTRO=${ROS_DISTRO:-}"
echo "ros2: $(command -v ros2)"
echo "rosdep: $(command -v rosdep)"
echo "colcon: $(command -v colcon)"

ros2 --help >/dev/null
rosdep --version

if command -v rviz2 >/dev/null 2>&1; then
  echo "rviz2: $(command -v rviz2)"
else
  echo "rviz2 was not found" >&2
  exit 1
fi

if command -v gazebo >/dev/null 2>&1; then
  echo "gazebo: $(command -v gazebo)"
else
  echo "gazebo was not found. Install ros-foxy-gazebo-ros-pkgs before Gazebo simulation work." >&2
  exit 1
fi

echo "Local ROS 2 Foxy environment looks ready."
