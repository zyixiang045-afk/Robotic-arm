#!/usr/bin/env bash
# 方案 B：用保存的地图 + 仿真环境做路径规划和导航。
# 不需要实时 SLAM 建图，直接加载已保存的栅格地图。
#
# 启动内容：
#   1. slam_bridge.py  — MuJoCo 仿真 + TF + /clock（提供机器人环境）
#   2. nav_p2p.py --use-saved  — 加载 maps/lab_map.pgm 做 A* 路径规划
#   3. RViz2  — 可视化地图和路径，点击目标点导航
#
# 用法:
#   ./slam/run_nav_saved.sh              # headless
#   ./slam/run_nav_saved.sh --view       # 同时开 MuJoCo 查看器
#
# 在 RViz2 中：
#   - 工具栏点 "2D Goal Pose"，在地图上点击目标位置 → 自动规划并导航
#   - /nav_path 显示规划路径
#   - /nav_status 显示导航状态
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

# 清除 ROS1 残留
unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME ROS_ETC_DIR ROS_ROOT ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES ROS_DISTRO
source /opt/ros/foxy/setup.bash

BRIDGE_ARGS=()
for a in "$@"; do
  case "$a" in
    --view) BRIDGE_ARGS+=(--view) ;;
    *)      BRIDGE_ARGS+=("$a") ;;
  esac
done

# 清理遗留进程
pkill -u "$USER" -f "slam_bridge.py" 2>/dev/null || true
pkill -u "$USER" -f "slam_bridge_3d.py" 2>/dev/null || true
pkill -u "$USER" -f "nav_p2p.py" 2>/dev/null || true
pkill -u "$USER" -f "rviz2.*view_map" 2>/dev/null || true
sleep 2

PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[1/3] 起 MuJoCo 3D 仿真桥接 (slam_bridge_3d.py --no-lidar) ..."
python3.8 slam_bridge_3d.py --no-lidar "${BRIDGE_ARGS[@]}" &
PIDS+=("$!")
sleep 5

# 方案 B 没有 rtabmap，需要手动发布 map->odom 静态 TF（identity）
echo "      发布 map->odom 静态 TF ..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom --ros-args -p use_sim_time:=true &
PIDS+=("$!")

echo "[2/3] 起导航节点 (nav_p2p.py --use-saved, Lazy Theta*) ..."
python3.8 nav_p2p.py --use-saved &
PIDS+=("$!")
sleep 3

echo "[3/3] 起 RViz2 ..."
rviz2 -d "$HERE/slam/view_map.rviz" --ros-args -p use_sim_time:=true &
PIDS+=("$!")

cat <<'EOF'

=== 导航已启动（方案 B：保存地图 + Lazy Theta* 规划）===

操作方法：
  1. 在 RViz2 工具栏点 "2D Goal Pose"（Nav Goal 按钮）
  2. 在地图上点击目标位置并拖动设置方向
  3. 机器人会自动规划路径并导航过去

或命令行发目标点：
  ros2 topic pub -1 /nav_goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: 4.5, y: 1.0}, orientation: {w: 1.0}}}"

监控：
  ros2 topic echo /nav_status    # 查看状态: IDLE/PLANNING/FOLLOWING/ARRIVED/STUCK
  ros2 topic echo /nav_path      # 查看路径

Ctrl-C 退出全部。
EOF
wait
