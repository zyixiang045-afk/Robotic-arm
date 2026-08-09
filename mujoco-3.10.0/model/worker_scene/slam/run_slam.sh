#!/usr/bin/env bash
# 一键起 2D 激光 SLAM：MuJoCo 桥接 + slam_toolbox 异步建图。
#
# 为什么是两个进程：Foxy 的 rclpy 只有 cpython-38 扩展，桥接节点必须用
# python3.8 跑；而 build_scene.py / build_robot.py 用的是 py3.11 + mujoco 3.10。
# 两边通过 XML 文件交接（build_robot.py 会额外输出 *_py38.xml 兼容版）。
#
# 用法:
#   ./slam/run_slam.sh              # 自动巡视建图
#   ./slam/run_slam.sh --view       # 同时开 MuJoCo 查看器
#   ./slam/run_slam.sh --rviz       # 同时开 RViz
#   ./slam/run_slam.sh --fresh      # 启动前清理上次遗留的本场景 SLAM 进程
#   ./slam/run_slam.sh --teleop     # 不自动巡视，等 /cmd_vel（配 teleop_twist_keyboard）
# 不能用 set -u：/opt/ros/foxy/setup.bash 会引用未定义的
# AMENT_TRACE_SETUP_FILES，开了 -u 直接报错退出。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

if [ ! -f /opt/ros/foxy/setup.bash ]; then
  echo "找不到 /opt/ros/foxy/setup.bash" >&2; exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash

if ! ros2 pkg list 2>/dev/null | grep -qx slam_toolbox; then
  cat >&2 <<'EOF'
没装 slam_toolbox。需要（要 sudo 密码，本脚本不代劳）：
    sudo apt update && sudo apt install -y ros-foxy-slam-toolbox
装完再跑本脚本。
EOF
  exit 1
fi

if [ ! -f scene_with_robot_py38.xml ]; then
  echo "缺 scene_with_robot_py38.xml，先生成：" >&2
  echo "    python3 build_robot.py --scene scene.xml --out scene_with_robot.xml" >&2
  exit 1
fi

BRIDGE_ARGS=(--patrol)
WANT_RVIZ=0
FRESH=0
for a in "$@"; do
  case "$a" in
    --view)   BRIDGE_ARGS+=(--view) ;;
    --rviz)   WANT_RVIZ=1 ;;
    --fresh)  FRESH=1 ;;
    --teleop) BRIDGE_ARGS=("${BRIDGE_ARGS[@]/--patrol/}") ;;
    *)        BRIDGE_ARGS+=("$a") ;;
  esac
done

if [ "$FRESH" = 1 ]; then
  echo "[fresh] 清理上次可能遗留的 MuJoCo/SLAM/RViz 进程 ..."
  pkill -u "$USER" -f "slam_bridge.py" 2>/dev/null || true
  pkill -u "$USER" -f "slam.launch.py" 2>/dev/null || true
  pkill -u "$USER" -f "async_slam_toolbox_node" 2>/dev/null || true
  pkill -u "$USER" -f "rviz2 .*slam.rviz" 2>/dev/null || true
  sleep 2
fi

EXISTING_NODES="$(ros2 node list 2>/dev/null | grep -E '^/(mujoco_slam_bridge|slam_toolbox|rviz2)$' || true)"
if [ -n "$EXISTING_NODES" ]; then
  cat >&2 <<EOF
检测到同名 ROS 节点还在运行，直接启动会把新旧 /map、/scan、TF 混在一起：
$EXISTING_NODES

请先在上次运行的终端按 Ctrl-C，或显式使用：
    ./slam/run_slam.sh --fresh ${*}
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

echo "[1/2] 起 MuJoCo 桥接 (python3.8) ..."
python3.8 slam_bridge.py "${BRIDGE_ARGS[@]}" &
PIDS+=("$!")
sleep 3

echo "[2/2] 起 slam_toolbox 异步建图 ..."
ros2 launch "$HERE/slam/slam.launch.py" use_sim_time:=true &
PIDS+=("$!")

if [ "$WANT_RVIZ" = 1 ]; then
  sleep 4
  echo "[+] 起 RViz (slam/slam.rviz) ..."
  # use_sim_time 必须显式给 RViz：桥接发的是仿真时间，不开的话 TF 全部查不到，
  # 界面上只会看到 "Message Filter dropping message"，地图和激光都不显示。
  rviz2 -d "$HERE/slam/slam.rviz" --ros-args -p use_sim_time:=true &
  PIDS+=("$!")
fi

cat <<'EOF'

跑起来了。常用命令（另开终端，先 source /opt/ros/foxy/setup.bash）：
  ros2 topic hz /scan                       # 确认 10 Hz
  ros2 topic hz /map                        # 约 1 Hz
  ros2 run tf2_tools view_frames            # 生成 frames.pdf 检查 TF 链
  cd <想存的目录> && ros2 run nav2_map_server map_saver_cli -f lab_map
  ros2 service call /slam_toolbox/serialize_map \
      slam_toolbox/srv/SerializePoseGraph "{filename: lab_posegraph}"

Ctrl-C 结束全部进程。
EOF
wait
