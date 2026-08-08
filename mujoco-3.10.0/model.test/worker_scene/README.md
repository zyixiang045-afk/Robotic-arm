# 实验室内景场景 — MuJoCo SLAM 仿真

MuJoCo 仿真场景（约 15m × 6m 实验室），配四足机械狗（双臂双手）做 2D/3D 激光 SLAM。

## 环境要求

- Python 3.8 + mujoco 3.2.3（ROS Foxy 的 rclpy 只有 cpython-38 扩展）
- ROS 2 Foxy
- `ros-foxy-slam-toolbox`（2D SLAM）
- `ros-foxy-rtabmap-ros`（3D SLAM，若 apt 无此包需源码编译）

## 快速开始

### 2D 激光 SLAM（slam_toolbox）

```bash
source /opt/ros/foxy/setup.bash
./slam/run_slam.sh --fresh --view --rviz
```

狗自动巡视，RViz 实时显示 2D 栅格地图。建图完成后保存：

```bash
ros2 run nav2_map_server map_saver_cli -f maps/lab_map
```

### 3D 点云 SLAM（rtabmap）

```bash
source /opt/ros/foxy/setup.bash

# 首次运行或模型更新后，生成 3D 雷达 XML（自动完成，也可手动）：
python3.8 gen_3d_xml.py

# 一键启动：3D 桥接 + rtabmap + rviz
./slam/run_slam_3d.sh --view --fresh
```

巡视完成后保存 3D 地图：

```bash
./slam/save_map_3d.sh           # 保存到 ./maps/
```

输出格式：
- `maps/lab_map_3d.yaml` + `.pgm` — 2D 投影栅格（nav2 路径规划可用）
- `maps/rtabmap.db` — 完整 3D 地图数据库（可离线导出 .pcd/.ply）

## 文件结构

### 模型文件
| 文件 | 说明 |
|------|------|
| `scene.xml` | 场景模型（手工调整过，不要重新生成覆盖） |
| `scene_with_robot_py38.xml` | 基准模型：场景 + 机器人 + 2D 雷达 |
| `scene_with_robot_3d_py38.xml` | 3D 版：场景 + 机器人 + 3D 雷达（由 gen_3d_xml.py 生成）|

### 脚本
| 文件 | 说明 |
|------|------|
| `gen_3d_xml.py` | 从 2D XML 生成 3D 雷达版本（mujoco 3.2.3 可运行）|
| `slam_bridge.py` | 2D 桥接：MuJoCo → /scan (LaserScan) |
| `slam_bridge_3d.py` | 3D 桥接：MuJoCo → /pointcloud (PointCloud2) |
| `build_robot.py` | 机器人装配脚本（需 mujoco ≥3.6，当前环境不可用） |
| `build_scene.py` | 场景生成器（参考用，scene.xml 已手工修改）|

### slam/ 目录
| 文件 | 说明 |
|------|------|
| `run_slam.sh` | 2D SLAM 一键启动 |
| `run_slam_3d.sh` | 3D SLAM 一键启动 |
| `save_map_3d.sh` | 保存 3D 地图 |
| `slam.launch.py` | slam_toolbox launch |
| `rtabmap_3d.launch.py` | rtabmap launch |
| `rtabmap_params.yaml` | rtabmap ICP-SLAM 参数 |
| `slam.rviz` | 2D SLAM RViz 配置 |
| `slam_3d.rviz` | 3D SLAM RViz 配置 |

## 工作流说明

### 模型生成（mujoco 3.2.3 环境）

基准模型 `scene_with_robot_py38.xml` 是预先生成好的，包含完整场景和 2D 雷达。

3D 版本通过 `gen_3d_xml.py` 从基准模型生成：
- 删除 2D lidar sites/sensors
- 注入 16 层 × 180 束 = 2880 根 3D rangefinder 射线
- 验证 MuJoCo 能正常加载

`build_robot.py` 是为 mujoco ≥3.6 编写的装配脚本，使用 MjSpec 类方法 API。
当前 3.2.3 环境无法运行它，保留作为文档和未来升级参考。

### ROS 话题

**2D 模式：**
- `/scan` (LaserScan) — 360 束 10 Hz
- `/odom` (Odometry) — 50 Hz
- TF: `odom → base_footprint → base_link → laser`

**3D 模式：**
- `/pointcloud` (PointCloud2) — 2880 点 10 Hz
- `/odom` (Odometry) — 50 Hz
- `/rtabmap/cloud_map` (PointCloud2) — 3D 点云地图
- `/rtabmap/map` (OccupancyGrid) — 2D 投影栅格
- TF: `map → odom → base_footprint → base_link → lidar3d`

### 路径规划

建图完成后的地图可直接用于 nav2：

```bash
# 加载已保存的 2D 栅格地图
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=maps/lab_map_3d.yaml \
  -p use_sim_time:=true
```

rtabmap 也支持定位模式（不再建图，只做位姿估计）：
在 `slam/rtabmap_params.yaml` 中设置 `Mem/IncrementalMemory: "false"`，
并移除 launch 文件中的 `--delete_db_on_start` 参数。
