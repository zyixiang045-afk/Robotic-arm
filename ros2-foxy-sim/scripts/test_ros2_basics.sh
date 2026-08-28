#!/usr/bin/env bash
set -euo pipefail

if [ -f /opt/ros/foxy/setup.bash ]; then
  set +u
  source /opt/ros/foxy/setup.bash
  set -u
else
  echo "Missing /opt/ros/foxy/setup.bash" >&2
  exit 1
fi

echo "ROS_DISTRO=${ROS_DISTRO:-}"
test "${ROS_DISTRO:-}" = "foxy"

command -v ros2
command -v rosdep
command -v colcon

ros2 --help >/dev/null
rosdep --version

timeout 8s ros2 run demo_nodes_cpp talker >/tmp/ros2_foxy_talker_test.log 2>&1 || true

if ! grep -q "Publishing:" /tmp/ros2_foxy_talker_test.log; then
  echo "talker did not publish within the timeout." >&2
  cat /tmp/ros2_foxy_talker_test.log >&2
  exit 1
fi

echo "Basic ROS 2 Foxy tests passed."
