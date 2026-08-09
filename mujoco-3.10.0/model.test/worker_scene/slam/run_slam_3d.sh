#!/usr/bin/env bash
# 一键起 3D 激光 SLAM：MuJoCo 3D 雷达 + rtabmap_ros + rviz2 显示。
#
# 与 run_slam.sh (2D) 并行独立——不影响原有 2D SLAM 流程。
# 使用 rtabmap_ros 做 ICP-based 3D SLAM，输出 3D 点云地图。
#
# 用法:
#   ./slam/run_slam_3d.sh              # 自动巡视 + rtabmap + rviz
#   ./slam/run_slam_3d.sh --view       # 同时开 MuJoCo 查看器
#   ./slam/run_slam_3d.sh --no-rviz    # 不开 rviz
#   ./slam/run_slam_3d.sh --fresh      # 清理遗留进程后启动
#   ./slam/run_slam_3d.sh --teleop     # 不自动巡视
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

if [ ! -f /opt/ros/foxy/setup.bash ]; then
  echo "找不到 /opt/ros/foxy/setup.bash" >&2; exit 1
fi

# 清除 ROS1 (noetic) 残留环境变量，避免干扰 ROS2 DDS 通信
unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME ROS_ETC_DIR ROS_ROOT ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES ROS_DISTRO

# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash

if ! ros2 pkg list 2>/dev/null | grep -qx rtabmap_slam; then
  cat >&2 <<'EOF'
没装 rtabmap_ros。需要（要 sudo 密码，本脚本不代劳）：
    sudo apt update && sudo apt install -y ros-foxy-rtabmap-ros
若 apt 源中无此包，需从源码编译：
    cd ~/colcon_ws/src
    git clone -b foxy-devel https://github.com/introlab/rtabmap_ros.git
    git clone -b foxy-devel https://github.com/introlab/rtabmap.git
    cd ~/colcon_ws && colcon build --symlink-install
装完再跑本脚本。
EOF
  exit 1
fi

if [ ! -f scene_with_robot_3d_py38.xml ]; then
  if [ -f scene_with_robot_py38.xml ]; then
    echo "[auto] 未找到 3D-only 模型，自动从 scene_with_robot_py38.xml 生成 ..."
    python3.8 gen_3d_xml.py || { echo "生成失败" >&2; exit 1; }
  else
    echo "缺 scene_with_robot_py38.xml，请先生成基础模型" >&2
    exit 1
  fi
fi

BRIDGE_ARGS=(--patrol)
WANT_RVIZ=1
FRESH=0
for a in "$@"; do
  case "$a" in
    --view)     BRIDGE_ARGS+=(--view) ;;
    --no-rviz)  WANT_RVIZ=0 ;;
    --fresh)    FRESH=1 ;;
    --teleop)   BRIDGE_ARGS=("${BRIDGE_ARGS[@]/--patrol/}") ;;
    *)          BRIDGE_ARGS+=("$a") ;;
  esac
done

if [ "$FRESH" = 1 ]; then
  echo "[fresh] 清理遗留的 3D 桥接/rtabmap/RViz 进程 ..."
  pkill -u "$USER" -f "slam_bridge_3d.py" 2>/dev/null || true
  pkill -u "$USER" -f "rtabmap_3d.launch.py" 2>/dev/null || true
  pkill -u "$USER" -f "rtabmap --delete_db_on_start" 2>/dev/null || true
  pkill -u "$USER" -f "rviz2 .*slam_3d.rviz" 2>/dev/null || true
  sleep 2
fi

EXISTING="$(ros2 node list 2>/dev/null | grep -E '^/mujoco_slam_bridge_3d$' || true)"
if [ -n "$EXISTING" ]; then
  cat >&2 <<EOF
检测到 3D 桥接节点还在运行：$EXISTING
请先 Ctrl-C 上次运行，或使用 --fresh 参数。
EOF
  exit 1
fi

PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[1/3] 起 MuJoCo 3D 点云桥接 (python3.8) ..."
python3.8 slam_bridge_3d.py "${BRIDGE_ARGS[@]}" &
PIDS+=("$!")
sleep 5

echo "[2/3] 起 rtabmap 3D SLAM ..."
ros2 launch "$HERE/slam/rtabmap_3d.launch.py" use_sim_time:=true &
PIDS+=("$!")
sleep 4

if [ "$WANT_RVIZ" = 1 ]; then
  echo "[3/3] 起 RViz (slam/slam_3d.rviz) ..."
  rviz2 -d "$HERE/slam/slam_3d.rviz" --ros-args -p use_sim_time:=true &
  PIDS+=("$!")
fi

cat <<'EOF'

3D SLAM 已启动。常用命令（另开终端，先 source /opt/ros/foxy/setup.bash）：
  ros2 topic hz /pointcloud                    # 确认点云 10 Hz
  ros2 topic hz /rtabmap/cloud_map             # 3D 地图更新频率

保存地图（建图完成后执行）：
  # 导出 3D 点云地图为 PCD 文件（可直接用于路径规划）
  ros2 service call /publish_map rtabmap_msgs/srv/PublishMap "{global_map: true, optimized: true, graph_only: false}"
  ros2 run pcl_ros pointcloud_to_pcd --ros-args -r input:=/rtabmap/cloud_map

  # 导出 2D 投影栅格（nav2 可用）
  ros2 run nav2_map_server map_saver_cli -f lab_map_3d --ros-args -p use_sim_time:=true

  # 保存 rtabmap 数据库（包含完整位姿图，可离线重处理）
  # 数据库自动存于 ~/.ros/rtabmap.db，Ctrl-C 退出时自动保存

Ctrl-C 结束全部进程。
EOF
wait
