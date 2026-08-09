#!/usr/bin/env bash
# 保存 rtabmap 3D SLAM 建图结果。
#
# 在 run_slam_3d.sh 运行期间，另开终端执行本脚本即可保存地图。
# 输出文件:
#   ./maps/cloud_map.pcd       3D 点云地图（PCL 格式，可导入 PCL/Open3D）
#   ./maps/lab_map_3d.pgm/yaml 2D 投影栅格（nav2 路径规划可直接用）
#   ~/.ros/rtabmap.db          rtabmap 数据库（包含完整位姿图）
#
# 用法:
#   ./slam/save_map_3d.sh                 # 保存到 ./maps/
#   ./slam/save_map_3d.sh /path/to/dir    # 保存到指定目录
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

if [ ! -f /opt/ros/foxy/setup.bash ]; then
  echo "找不到 /opt/ros/foxy/setup.bash" >&2; exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash

OUTDIR="${1:-$HERE/maps}"
mkdir -p "$OUTDIR"

echo "=== 保存 3D SLAM 地图到 $OUTDIR ==="

# 1. 触发 rtabmap 发布完整地图
echo "[1/3] 请求 rtabmap 发布完整地图 ..."
ros2 service call /publish_map \
  rtabmap_msgs/srv/PublishMap \
  "{global_map: true, optimized: true, graph_only: false}" \
  2>/dev/null || echo "  (publish_map 服务不可用，跳过)"

sleep 2

# 2. 保存 2D 栅格地图（nav2 格式）
echo "[2/3] 保存 2D 投影栅格 ..."
ros2 run nav2_map_server map_saver_cli \
  -f "$OUTDIR/lab_map_3d" \
  --ros-args -p use_sim_time:=true \
  2>/dev/null && echo "  已保存 $OUTDIR/lab_map_3d.pgm + .yaml" \
  || echo "  (map_saver 失败，可能 /map 尚未发布)"

# 3. 复制 rtabmap 数据库
echo "[3/3] 复制 rtabmap 数据库 ..."
if [ -f "$HOME/.ros/rtabmap.db" ]; then
  cp "$HOME/.ros/rtabmap.db" "$OUTDIR/rtabmap.db"
  echo "  已保存 $OUTDIR/rtabmap.db"
  echo "  (离线导出 3D 点云: rtabmap-export --cloud $OUTDIR/rtabmap.db)"
else
  echo "  未找到 ~/.ros/rtabmap.db（rtabmap 可能未启动或尚未创建数据库）"
fi

echo ""
echo "=== 完成 ==="
echo "后续路径规划用法："
echo "  2D 栅格(nav2): $OUTDIR/lab_map_3d.yaml"
echo "  3D 点云(自定义): $OUTDIR/rtabmap.db → rtabmap-export 导出 .pcd/.ply"
echo "  rtabmap 定位模式: 移除 --delete_db_on_start 参数，"
echo "    并在 rtabmap_params.yaml 中设置 Mem/IncrementalMemory: \"false\""
