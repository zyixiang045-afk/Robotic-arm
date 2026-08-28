# SLAM 建图模块

本目录包含 MuJoCo 仿真环境中的 2D/3D SLAM 建图、自主探索和导航相关代码。

## 快速启动

```bash
source /opt/ros/foxy/setup.bash
cd worker_scene/

# 固定航点巡视建图（默认）
./slam/run_slam_3d.sh --view --fresh

# Frontier 自主探索建图（推荐，无需航点）
./slam/run_slam_3d.sh --view --fresh --explore

# 手动遥控建图
./slam/run_slam_3d.sh --view --fresh --teleop
```

## run_slam_3d.sh 参数说明

| 参数 | 说明 |
|------|------|
| `--view` | 同时打开 MuJoCo 查看器 |
| `--fresh` | 清理遗留进程后启动（推荐首次运行时使用） |
| `--explore` | 使用 Frontier Exploration 自主探索（不走固定航点） |
| `--teleop` | 手动遥控模式，不自动巡视 |
| `--no-rviz` | 不启动 RViz |
| `--scene ariac` | 使用 ARIAC 2025 实验室（默认） |
| `--scene lab` | 使用实验室场景 |
| `--scene warehouse` | 使用旧仓库场景 |
| `--laps N` | 固定航点模式下巡视圈数（默认 1） |
| `--odom-noise` | 显式注入里程计随机游走，仅用于回环抗漂移压力测试；默认关闭 |

## 场景切换

默认场景为 `ariac`。切换方式：

```bash
# ARIAC 主场景（省略 --scene 时也是此场景）
./slam/run_slam_3d.sh --view --fresh --explore --scene ariac

# 仓库场景（20m x 20m，含货架、桌子等障碍物）
./slam/run_slam_3d.sh --view --fresh --explore --scene warehouse

# 实验室场景（15m x 6m，桌椅设备）
./slam/run_slam_3d.sh --view --fresh --explore --scene lab
```

场景对应的桥接脚本：
- `ariac` → `slam/bridge/bridge_ariac.py`
- `warehouse` → `slam/bridge/bridge_warehouse.py`
- `lab` → `slam/bridge/bridge_lab.py`

如需新增场景：
1. 在 `model/scenes/` 创建场景 XML
2. 参考 `model/robot/gen_ariac_robot.py` 注入机器人和 3D lidar
3. 在 `slam/bridge/` 创建对应的桥接脚本（可复制 `bridge_warehouse.py` 修改）
4. 在 `run_slam_3d.sh` 的 `SCENE` 分支中添加新场景名

## 建图模式

### 模式 1：固定航点巡视（默认）

机器人按预设航点路径巡视，使用反应式避障（前方 ±30° 扇区检测）。

航点定义在 `bridge_warehouse.py` 的 `PATROL_WAYPOINTS` 列表中。修改航点：

```python
PATROL_WAYPOINTS = [
    (-8.5, -8.5),   # (x, y) 世界坐标
    (-8.5, 4.0),
    ...
]
```

适合：已知环境布局，想快速覆盖指定区域。

### 模式 2：Frontier 自主探索（--explore）

使用 `slam/frontier_explorer.py` 自主探索建图：

1. 订阅 rtabmap 输出的 2D 栅格地图（`/rtabmap/grid_map` 或 `/rtabmap/map`）
2. 检测已知区域与未知区域的边界（frontier）
3. 对地图障碍做 EDT 膨胀，从机器人位置跑 Dijkstra，只把【可达】的
   frontier 作为候选（墙另一侧不可达的目标自动过滤，避免撞墙/沿墙滑行）
4. 生成可达路径，沿 waypoints 纯追踪跟随
5. 复用 `slam/bridge/local_avoidance.py` 的 BACKUP->TURN->CLEAR->IDLE
   状态机做局部安全层（急停/减速/脱困，避免撞障碍与角落卡死）
6. 进度式卡死检测：目标距离不再下降才判卡住，多次脱困无效则拉黑目标重选
7. 到达后原地旋转 360° 扫描，重复直到无 frontier 可探索

适合：陌生环境，不知道布局，需要完整覆盖。

### 模式 3：手动遥控（--teleop）

不自动移动，通过 `/cmd_vel` 话题手动控制：

```bash
# 另开终端
source /opt/ros/foxy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 关键参数调整

### Frontier Explorer 参数（`slam/frontier_explorer.py`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `FRONTIER_MIN_SIZE` | 15 | 最小 frontier 聚类（栅格数），小于此的忽略 |
| `FRONTIER_GOAL_BIAS` | 0.6 | 目标选择偏好：0=纯距离优先，1=纯大小优先 |
| `FRONTIER_REACHED_DIST` | 0.8m | 认为到达 frontier 的距离阈值 |
| `PLAN_INFLATE` | 0.45m | 路径规划障碍膨胀半径（大于狗碰撞盒半对角线 0.55m 的常用值） |
| `PLAN_WAYPOINT_SPACING` | 0.4m | 路径点间距 |
| `PLAN_MAX_GOAL_DIST` | 25m | 超过此 Dijkstra 距离视为不可达 |
| `GOAL_BLACKLIST_COUNT/RADIUS` | 3 / 2m | 失败目标拉黑，避免死磕同一目标/角落 |
| `STUCK_NO_PROGRESS_SEC` | 4s | 目标距离无下降的卡死判定窗口 |
| `NAV_SPEED_MAX` | 0.30 m/s | 最大前进速度 |
| `NAV_TURN_MAX` | 1.0 rad/s | 最大角速度 |
| `AVOID_EMERGENCY_DIST` | 0.75m | 局部避障紧急净距（同 bridge 巡逻模式） |
| `AVOID_SLOW_DIST` | 1.50m | 局部避障减速区（同 bridge 巡逻模式） |

路径规划核心逻辑在 `slam/bridge/explore_planner.py`（无 ROS 依赖，可单测）；
局部避障复用 `slam/bridge/local_avoidance.py`。两者与 `--patrol` 模式共用同一套
脱困状态机，行为一致。

### 巡视模式参数（`bridge_warehouse.py`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `PATROL_V` | 0.40 m/s | 巡视线速度 |
| `PATROL_W` | 0.6 rad/s | 巡视角速度 |
| `OBSTACLE_DIST` | 0.60m | 紧急避障触发距离 |
| `OBSTACLE_SLOW_DIST` | 1.20m | 开始减速距离 |

### rtabmap 参数（`rtabmap_params.yaml`）

ICP-SLAM 核心参数，通常不需要修改。如需调整建图质量：
- 增大 `Grid/CellSize` 可降低分辨率但加速建图
- 调整 `Reg/Strategy` 改变帧间配准策略

默认启动使用无噪声仿真里程计，并限制 ICP 的对应距离和最大修正量，减少后期
回环时点云整体跳变。不要把 `--odom-noise` 用于正常建图；它只用于测试回环
在有随机游走时的容错能力。

## 建图输出

建图完成后保存：

```bash
./slam/save_map_3d.sh                    # 默认保存 ARIAC
./slam/save_map_3d.sh --scene ariac
./slam/save_map_3d.sh --scene warehouse
```

保存 ARIAC 地图后可直接启动定位与点到点导航：

```bash
./slam/run_nav_saved.sh --scene ariac --view
python3.8 send_goal.py --scene ariac 2.0 1.0 --wait
```

输出文件（保存到 `maps/` 目录）：

| 文件 | 用途 |
|------|------|
| `maps/*_3d.yaml` + `.pgm` | 2D 投影栅格地图（nav2 路径规划用） |
| `maps/rtabmap.db` | 完整 3D 地图数据库（含位姿图，可离线导出 .pcd） |

## ROS 话题

运行时的关键话题：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/pointcloud` | PointCloud2 | 3D lidar 点云（10 Hz） |
| `/odom` | Odometry | 里程计（50 Hz） |
| `/cmd_vel` | Twist | 速度指令 |
| `/rtabmap/map` | OccupancyGrid | 2D 投影栅格 |
| `/cloud_map` | PointCloud2 | 保留水平表面的 3D 点云地图 |
| `/frontier_markers` | MarkerArray | Frontier 可视化（--explore 模式） |

## 依赖

- Python 3.8 + mujoco 3.2.3
- ROS 2 Foxy
- `ros-foxy-rtabmap-ros`
- `scipy`（frontier_explorer.py 的聚类检测需要）
- `numpy`

```bash
pip3.8 install scipy numpy
```

## 文件说明

```
slam/
├── README.md                 # 本文件
├── run_slam.sh               # 2D SLAM 启动脚本
├── run_slam_3d.sh            # 3D SLAM 启动脚本（主入口）
├── frontier_explorer.py      # Frontier 自主探索 + 全局规划 + 局部避障节点
├── rtabmap_3d.launch.py      # rtabmap ROS2 launch 文件
├── rtabmap_params.yaml       # rtabmap ICP-SLAM 参数
├── save_map_3d.sh            # 保存地图脚本
├── run_nav.sh                # 导航启动（在线建图）
├── run_nav_saved.sh          # 导航启动（加载已保存地图）
├── restart_nav.sh            # 重启导航栈
├── view_map.sh               # 离线查看地图
├── slam.rviz                 # 2D SLAM RViz 配置
├── slam_3d.rviz              # 3D SLAM RViz 配置
├── view_map.rviz             # 地图查看 RViz 配置
├── mapper_params_online_async.yaml  # slam_toolbox 参数
└── bridge/                   # MuJoCo → ROS 桥接
    ├── bridge_lab.py         # 实验室场景桥接
    ├── bridge_warehouse.py   # 仓库场景桥接
    ├── local_avoidance.py    # 局部避障 + 脱困状态机（巡逻/探索共用）
    ├── explore_planner.py    # 全局路径规划（无 ROS 依赖，可单测）
    └── legacy_2d/            # 旧版 2D 桥接（已弃用）

无 ROS 快速回归（`./test/run_avoidance_test.sh quick`）：
  python3.8 test/verify_avoidance_code.py   # 局部避障逻辑
  python3.8 test/test_explore_planner.py    # 全局规划器 + 局部避障接线
```
