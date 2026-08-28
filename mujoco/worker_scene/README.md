# ARIAC 实验室场景 - MuJoCo SLAM 仿真

主场景为 ARIAC 2025 实验室，配四足机械狗（双臂双手）进行 3D 激光 SLAM。
项目仍保留原 `lab` 和 `warehouse` 场景，`run_slam_3d.sh` 默认打开 `ariac`。

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

# ARIAC 资源更新后重新生成 3.2.3 兼容网格和机器人组合场景：
python3.8 model/scenes/build_ariac_compat_meshes.py
python3.8 model/robot/gen_ariac_robot.py

# 一键启动：3D 桥接 + rtabmap + rviz（固定航点巡视）
./slam/run_slam_3d.sh --view --fresh

# Frontier 自主探索建图（无需航点，适用于陌生环境）
./slam/run_slam_3d.sh --view --fresh --explore
```

显式选择场景时使用 `--scene ariac|lab|warehouse`。不传 `--scene` 等同于
`--scene ariac`。只查看纯 ARIAC 场景可运行：

```bash
../../bin/simulate model/scenes/ariac_lab.xml
```

3D 桥接默认发布无噪声仿真里程计，以避免长路径随机游走在回环时造成点云整体
跳变。`--odom-noise` 仅用于回环抗漂移压力测试，不建议正常建图时开启。

`--explore` 模式使用 Frontier Exploration 算法自主建图：
- 自动检测地图中 已知/未知 区域边界（frontier）
- 选择最优 frontier 作为下一个探索目标
- 用 VFH（Vector Field Histogram）局部避障安全导航
- 到达后原地旋转 360° 扫描，再寻找下一个 frontier
- 全部区域覆盖后自动停止

无需手动规划航点，换场景也通用。

巡视/探索完成后保存 3D 地图：

```bash
./slam/save_map_3d.sh           # 默认保存到 ./maps/ariac/
```

输出格式：
- `maps/ariac/ariac_map_3d.yaml` + `.pgm` - 2D 投影栅格
- `maps/ariac/rtabmap.db` - 完整 3D 地图数据库

## 项目目录结构

```
worker_scene/
├── README.md                    # 本文件，项目总览
├── HOWTOSTART.md                # 新手入门指南
├── interactive_view.py          # 交互查看场景：MuJoCo 主窗口 + 双手腕相机实时画面
├── nav_p2p.py                   # 点对点导航：3D地图 + Lazy Theta* 规划 + 纯追踪跟随
├── send_goal.py                 # 发送导航目标点（支持 map/world 坐标系）
├── check_map.py                 # 检查 /map 建图进度（打印占据/自由/未知比例）
├── smolvla_infer.py             # SmolVLA 视觉-语言-动作模型推理入口
├── run_smolvla_infer.sh         # SmolVLA 推理启动脚本
├── smolvla_scene.xml            # SmolVLA 桌面操作场景模型
├── view_overview.png            # 场景全局预览图
├── view_top.png                 # 场景俯视图
│
├── model/                       # MuJoCo 模型文件
│   ├── assets/                  # 3D 网格和纹理资源
│   │   ├── ariac/               # ARIAC 网格、材质、预览图和转换报告
│   │   ├── arm/                 # RM65 机械臂 STL（base_link ~ link_6）
│   │   ├── dog/                 # 四足机械狗 STL（dog_visual_0~5）
│   │   ├── hand/                # 灵巧手 STL（左手/右手各指段）
│   │   ├── urdf/                # 原始 URDF 文件（RM65、dexhand 左右手）
│   │   └── calib_board.png      # 标定板纹理
│   ├── robot/                   # 机器人组装与场景集成
│   │   ├── build_robot.py       # 机器人装配脚本（需 mujoco ≥3.6）
│   │   ├── gen_3d_xml.py        # 从 2D XML 生成 3D 雷达版本
│   │   ├── gen_warehouse_robot.py # 提取机器人注入仓库场景
│   │   ├── gen_ariac_robot.py   # 将机器人和 3D 雷达注入 ARIAC
│   │   ├── ariac_lab_with_robot_3d.xml # SLAM 使用的 ARIAC 组合模型
│   │   ├── robot.xml            # 纯机器人模型
│   │   ├── robot_py38.xml       # Python 3.8 兼容机器人模型
│   │   ├── scene_with_robot.xml          # 实验室 + 机器人
│   │   ├── scene_with_robot_py38.xml     # 基准：实验室 + 机器人 + 2D 雷达
│   │   ├── scene_with_robot_3d_py38.xml  # 3D版：实验室 + 机器人 + 3D 雷达
│   │   └── warehouse_with_robot_3d.xml   # 仓库 + 机器人 + 3D 雷达
│   └── scenes/                  # 纯场景定义
│       ├── ariac_lab.xml        # 主 ARIAC 纯场景
│       ├── build_ariac_compat_meshes.py # 生成 3.2.3 兼容平面网格
│       ├── build_scene.py       # 场景生成器（参考用，scene.xml 已手工修改）
│       ├── scene.xml            # 实验室场景（手工调整，勿覆盖）
│       └── warehouse_with_obstacles_mujoco.xml  # 仓库场景（含障碍物）
│
├── slam/                        # SLAM 建图与导航相关
│   ├── bridge/                  # MuJoCo → ROS 桥接节点
│   │   ├── bridge_ariac.py      # 默认 ARIAC 场景 3D 桥接
│   │   ├── bridge_lab.py        # 实验室场景 3D 桥接
│   │   ├── bridge_warehouse.py  # 仓库场景 3D 桥接
│   │   └── legacy_2d/           # 旧版 2D SLAM 桥接（已弃用）
│   │       ├── bridge_lab_2d.py
│   │       └── slam.launch.py
│   ├── run_slam.sh              # 2D SLAM 一键启动
│   ├── run_slam_3d.sh           # 3D SLAM 一键启动
│   ├── frontier_explorer.py     # Frontier 自主探索 + VFH 避障节点
│   ├── run_nav.sh               # 导航启动（在线建图模式）
│   ├── run_nav_saved.sh         # 导航启动（加载已保存地图）
│   ├── restart_nav.sh           # 重启导航栈
│   ├── save_map_3d.sh           # 保存 3D 地图到 maps/
│   ├── view_map.sh              # 离线查看已保存地图
│   ├── rtabmap_3d.launch.py     # rtabmap ROS2 launch
│   ├── rtabmap_params.yaml      # rtabmap ICP-SLAM 参数
│   ├── mapper_params_online_async.yaml  # slam_toolbox 在线异步参数
│   ├── slam.rviz                # 2D SLAM RViz 配置
│   ├── slam_3d.rviz             # 3D SLAM RViz 配置
│   └── view_map.rviz            # 地图查看 RViz 配置
│
├── maps/                        # 建图输出（SLAM 生成的地图文件）
│   ├── lab_map.pgm / .yaml      # 2D 实验室栅格地图
│   ├── lab_map_3d.pgm / .yaml   # 3D 投影 2D 栅格地图（nav2 可用）
│   └── rtabmap.db               # 完整 3D 地图数据库
│
└── smolvla_trace/               # SmolVLA 推理可视化追踪
    ├── index.html               # 可视化页面
    ├── trace.json               # 推理轨迹数据
    └── images/                  # 推理过程截图
```

## 工作流说明

### 模型生成（mujoco 3.2.3 环境）

基准模型 `scene_with_robot_py38.xml` 是预先生成好的，包含完整场景和 2D 雷达。

3D 版本通过 `gen_3d_xml.py` 从基准模型生成：
- 删除 2D lidar sites/sensors
- XML 保留 16 层 × 180 束的兼容传感器定义；运行时禁用这些逐束传感器，
  改用 `mj_multiRay` 批量投射 64 层 × 360 束（23040 束）
- 验证 MuJoCo 能正常加载

`build_robot.py` 是为 mujoco ≥3.6 编写的装配脚本，使用 MjSpec 类方法 API。
当前 3.2.3 环境无法运行它，保留作为文档和未来升级参考。

### ROS 话题

**2D 模式：**
- `/scan` (LaserScan) — 360 束 10 Hz
- `/odom` (Odometry) — 50 Hz
- TF: `odom → base_footprint → base_link → laser`

**3D 模式：**
- `/pointcloud` (PointCloud2) — 最多 23040 个有效命中点，10 Hz
- `/odom` (Odometry) — 50 Hz
- `/cloud_map` (PointCloud2) — 保留水平表面的 3D 点云地图
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
