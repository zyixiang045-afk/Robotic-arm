#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME ROS_ETC_DIR ROS_ROOT ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_DISTRO
set +u
source /opt/ros/foxy/setup.bash
set -u
exec python3.8 "$HERE/run_edge_cost_experiment.py"
