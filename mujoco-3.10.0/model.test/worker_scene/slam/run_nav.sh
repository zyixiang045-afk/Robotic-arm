#!/usr/bin/env bash
# 起点对点导航节点。前提：slam_bridge.py + slam_toolbox 已在跑（./slam/run_slam.sh）。
# 或使用 --use-saved 加载保存地图离线测试。
# 目标点通过话题 /nav_goal 发送（map 系），例：
#   ros2 topic pub -1 /nav_goal geometry_msgs/msg/PoseStamped \
#     "{header: {frame_id: map}, pose: {position: {x: 4.5, y: 1.0}, orientation: {w: 1.0}}}"
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

if [ ! -f /opt/ros/foxy/setup.bash ]; then
  echo "找不到 /opt/ros/foxy/setup.bash" >&2; exit 1
fi

# 清除 ROS1 残留
unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME ROS_ETC_DIR ROS_ROOT ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES ROS_DISTRO
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash

# 默认只用在线 /map；--use-saved 则改用保存的 lab_map.pgm（可离线测试）
exec python3.8 nav_p2p.py "$@"
