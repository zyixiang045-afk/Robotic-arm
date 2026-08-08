# Maps 目录说明

本目录存放 SLAM 建图产生的地图文件。

## 文件说明

| 文件 | 说明 |
|------|------|
| `lab_map_3d.yaml` | 3D SLAM 产生的 2D 投影栅格地图配置 |
| `lab_map_3d.pgm` | 对应的栅格图像（320x141，分辨率 0.05m/pixel） |
| `lab_map.yaml` | 2D SLAM 产生的栅格地图配置 |
| `lab_map.pgm` | 对应的栅格图像 |
| `rtabmap.db` | rtabmap 数据库（含完整位姿图，可离线重处理） |

## 快速查看地图

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
./slam/view_map.sh                    # 默认打开 lab_map_3d.yaml
./slam/view_map.sh maps/lab_map.yaml  # 指定其他地图
```

地图会自动加载并在 RViz2 中显示，无需手动添加。

---

## 路径规划（Lazy Theta* + 纯追踪跟随）

导航脚本位置：`nav_p2p.py`（项目根目录）

### 规划算法

- **全局规划**: Lazy Theta* 算法（8连通 any-angle 搜索，Bresenham LOS 检查）
- **局部跟随**: 纯追踪（Pure Pursuit），支持全向底盘侧移
- **障碍膨胀**: scipy EDT 精确欧氏距离变换，膨胀半径 = 机器人半径(0.32m) + 余量(0.05m)
- **重规划触发**: 偏离路径 > 0.6m 或被堵超过 3s 自动重规划

---

## 方案 A：3D SLAM 实时建图 + 导航

实时建图的同时做路径规划导航。需要3个终端：

**终端 1**: 启动 3D SLAM 建图
```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
./slam/run_slam_3d.sh --view
```

**终端 2**: 启动导航节点（等建图几秒后启动）
```bash
source /opt/ros/foxy/setup.bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
python3.8 nav_p2p.py
```

**终端 3**: 发送目标点
```bash
source /opt/ros/foxy/setup.bash
ros2 topic pub -1 /nav_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 4.5, y: 1.0}, orientation: {w: 1.0}}}"
```

---

## 方案 B：用保存的地图 + 3D 仿真做导航（推荐离线测试）

不需要实时建图，直接加载已保存的地图做路径规划。一键启动：

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
./slam/run_nav_saved.sh --view
```

脚本位置：`slam/run_nav_saved.sh`

启动内容：
1. `slam_bridge_3d.py` — MuJoCo 3D 仿真 + TF + /clock
2. `nav_p2p.py --use-saved` — 加载 maps/lab_map.pgm 做 A* 路径规划
3. RViz2 — 可视化地图和路径

在 RViz2 中操作：
1. 工具栏点 **"2D Goal Pose"**（Nav Goal 按钮）
2. 在地图上点击目标位置并拖动设置方向
3. 机器人自动规划路径并导航

或命令行发目标点：
```bash
source /opt/ros/foxy/setup.bash
ros2 topic pub -1 /nav_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 4.5, y: 1.0}, orientation: {w: 1.0}}}"
```

---

## 坐标系说明

- 狗的起点在 map 原点，对应世界坐标 (-8.5, 0)
- 世界系 → map 系转换: `(x_world + 8.5, y_world)`
- 例: 世界坐标 (-4, 1) = map 坐标 (4.5, 1.0)

## 导航状态（/nav_status topic）

| 状态 | 含义 |
|------|------|
| IDLE | 空闲，等待目标 |
| PLANNING | 正在规划路径 |
| FOLLOWING | 跟随路径中 |
| ARRIVED | 已到达目标 |
| NO_MAP | 没有地图数据 |
| STUCK | 被堵，尝试重规划 |
| UNREACHABLE | 目标不可达（连续 3 次规划失败） |

## 常见排查命令

```bash
source /opt/ros/foxy/setup.bash
ros2 topic hz /pointcloud            # 确认点云发布
ros2 topic hz /map                   # 确认地图已发布
ros2 topic echo /nav_status          # 查看导航状态
ros2 topic echo /nav_path            # 查看规划路径
ros2 run tf2_tools view_frames       # 查看 TF 树
```
