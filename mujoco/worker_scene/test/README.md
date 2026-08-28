# 避障与建图测试

本目录集中存放避障、规划与建图相关的自动化测试。所有命令在
`worker_scene/` 根目录下执行。

## 一键入口

```bash
./test/sh/avoidance/run_avoidance_test.sh [quick|regression|ros]
```

| 模式 | 作用 | 耗时 |
| --- | --- | --- |
| `quick` | 无 ROS 逻辑检查 + 规划器单测（编译、水平净距、恢复状态机、全局规划、点云→净距→急停） | 秒级 |
| `regression` | 完整无界面 MuJoCo 巡逻/接触/桌面回归（默认模式） | 约两分钟 |
| `ros` | 启动 ROS 2 + RTAB-Map + MuJoCo 查看器（`--view --fresh` 巡逻） | 一直运行 |

## 测试脚本

### ARIAC 模型与默认场景回归

```bash
./test/sh/model/run_ariac_scene_test.sh
```

该测试无需 ROS 或图形界面，使用项目要求的 Python 3.8 + MuJoCo 3.2.3：

- 检查 ARIAC 文件已按 `model/scenes`、`model/assets`、`model/robot` 分类。
- 检查 XML 引用的全部网格存在。
- 实际加载纯 ARIAC 和 ARIAC + 机器人组合模型。
- 检查 `dog_base`、`lidar3d_frame`、执行器和雷达传感器。
- 检查 `run_slam_3d.sh` 默认选择 `ariac`，且旧场景仍可显式选择。

### 1. 快速逻辑检查 — `test/py/avoidance/verify_avoidance_code.py`

无需 ROS 2 或图形界面，验证：

```bash
python3.8 test/py/avoidance/verify_avoidance_code.py
```

- 共享模块、3D bridge、`explore_planner.py`、`frontier_explorer.py` 均可编译。
- `0.90 m` 向下斜距正确换算为约 `0.45 m` 水平净距。
- 恢复过程按 `BACKUP -> TURN -> CLEAR -> IDLE` 退出。

### 2. 全局规划器单测 — `test/py/navigation/test_explore_planner.py`

无需 ROS，验证探索导航的核心逻辑：

```bash
python3.8 test/py/navigation/test_explore_planner.py
```

- EDT 障碍膨胀（`0.45 m`）与地图边界强制不可走。
- Dijkstra 可达性：墙另一侧目标不可达并被过滤。
- 绕墙路径：全程落在可走格、穿过缺口到达另一侧。
- 目标拉黑：失败目标不会被再次选中。
- 点云 → 每方位角净距 → `LocalAvoidance` 前方净距与急停（`BACKUP`）。

### 3. 完整物理回归 — `test/py/avoidance/test_avoidance_simple.py`

直接运行仓库 MuJoCo 模型，无图形界面：

```bash
python3.8 test/py/avoidance/test_avoidance_simple.py
```

通过条件：

1. 17 个仓库航点全部完成，不能通过跳过航点伪装成功。
2. 每个物理步都不能发生机器狗与非地面障碍的接触。
3. 单次恢复状态持续时间必须小于 8 秒。
4. 矮障碍必须触发水平净距紧急判断。
5. 仓库桌面在固定位置必须至少有 250 个直接激光命中点。

当前基线：

```text
PASS sim_time=204.66s waypoints=17 recoveries=1 max_recovery=4.20s obstacle_contacts=0 tabletop_hits=343 scans=2048
```

## 实测运行（联调）

### 巡逻建图

```bash
./slam/run_slam_3d.sh --view --fresh
```

重点观察：

- 终端 `[避障状态]` 应按 后退 → 转向 → 越障 → 恢复导航 推进，不持续停在同
  一状态。
- 点云发布规格应为 `64 layers x 360 rays = 23040 total`。
- RViz 累积三维点云话题为 `/cloud_map`；桌面应为连续水平点云而非只有桌腿。

无界面冒烟检查：

```bash
timeout 35s ./slam/run_slam_3d.sh --fresh --no-rviz --teleop
```

启动初期短暂出现 `base_footprint frame does not exist` 属正常，TF 建立后应
恢复；持续出现才表示链路有问题。

### Frontier 自主探索建图

```bash
./slam/run_slam_3d.sh --view --fresh --explore
```

探索模式使用 `slam/frontier_explorer.py`（全局可达路径规划 + 复用
`local_avoidance.py` 局部避障）。重点观察：

- RViz 中 `/frontier_markers`（绿色 frontier、红色当前目标）与 `/explore_path`
  （规划路径）是否正确。
- 目标应沿规划路径绕开障碍，不直接撞墙、不沿整面墙滑行。
- 到达 frontier 后原地旋转扫描，然后继续下一个目标；地图逐步扩张。
- 遇到角落/新障碍时能自动脱困，多次失败后拉黑该目标改选其他 frontier。

### 建图进度检查

```bash
python3.8 check_map.py 15     # 采样 15 秒，打印占据/自由/未知比例
```

未知比例 < 30% 时地图基本建完，可保存。
