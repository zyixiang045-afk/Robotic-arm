# 实验室内景场景

一个 MuJoCo 仿真场景，模拟一间实验室内部：

- 四面墙围出的房间（约 15 m × 6 m，墙高 2.8 m）
- 多张实验台：主工作台 + 3 张靠墙实验台，另有置物架、方凳、台面器皿（烧杯/试剂瓶/仪器）等细节
- 主工作台上一红一蓝两把螺丝刀（`freejoint`，可推动/拾取）
- 一名静态工人（人体模型），站在走廊一端、面朝主工作台
- 走廊中**两个规则运动**的物体，供机械狗做动态避障：
  - 一台横穿走廊的往复料车（`shuttle_a`，沿 y 平移）
  - 一根绕竖直轴扫掠的挡杆（`sweep_1`，在机械狗身位高度扫过）
- 走廊中 3 个**静态货箱**障碍物（`crate_1..3`），供机械狗绕行

两个动态物体均由**弹簧驱动、阻尼为 0**，无需任何控制器即可在查看器中
自行做无衰减往复运动（已验证 30 s 内幅度不衰减）。

## 标定板

- 纹理 `assets/calib_board.png`（从 `model/camera` 拷来）：9×7 格 + 白色 quiet-zone 白边，
  → **8×6 内角点**。行列格数故意不等，避免方形棋盘的 4 重旋转对称让
  `findChessboardCorners` 把角点顺序转 90°/180°。白边是必需的，MuJoCo 内建 checker
  纹理没有白边、外圈黑格会和背景糊在一起导致检测时好时坏。
- geom `calib_board`：半尺寸 `0.15 0.15 0.004`（板真实边长 0.30 m），
  `contype=0 conaffinity=0` 不参与碰撞，纯视觉基准。
- 已验证从场景里渲染出来能稳定检测到全部 48 个内角点。

> 注意：`ready` 姿态下两个手背相机**看不到**这块板 —— 狗在 `x=-8.5`，板在 `(2.25,2.35)`，
> 相隔约 10.8 m，光轴偏离板心 84°。板目前是场景摆设/备用基准。若要在本场景真做手眼标定，
> 需要把狗挪到桌前（或把桌子挪到狗前）并重新解一组瞄准姿态，参考
> `model/camera` 的 `AIM_POSE_L/R` 与 `calib/` 那套流程。

## 机器人（四足狗 + 双 RM65 臂 + 双 dexhand 手）

由 `build_robot.py` 装配，含 52 关节 / 52 个 position 执行器与 `ready` 前伸工作姿态 keyframe。
两只手的**手背**各装一个随臂运动的相机（eye-in-hand），与 `model/camera` 项目一致：

- `cam_hand_l` / `cam_hand_r`，挂在 `hand_l/right_hand_base`、`hand_r/left_hand_base` 上；
  `pos=[-0.06,0,0.06]`（手背侧、沿手指前伸），光轴沿手基座 +Z，fovy=58，640×480。
- 手基座局部系：+X=掌心，−X=手背法向，+Z=手指/reach 方向。

仿真稳定性调参（`build_robot.py` 顶部常量）：

- `HAND_ARMATURE=0.002` —— 手部小关节加转动惯量，抑制高频抖动。
- `ARM_KV=15` —— 双臂位置执行器阻尼。原先 kv=4 时 `ready` 姿态会过冲到 1.72 rad
  并来回振荡十几次；kv=15 零过冲。
- `ROBOT_GRAVCOMP=1.0` —— **只对臂/手 body** 开重力补偿，场景其余部分（螺丝刀、
  料车、扫掠杆）保持真实重力。不开时 `kp=40` 顶不住臂+手自重，`ready` 姿态下
  `joint_2` 会下坠约 22°、手背相机跟着偏；开了之后双臂精确保持指令姿态
  （实测 10 s 后跟踪误差 0）。设 0.0 可恢复真实下坠。
  注：MuJoCo 的 `ngravcomp` 在编译期统计，必须在 spec 上设，运行时改 `body_gravcomp` 无效。

## 2D 激光 SLAM（slam_toolbox）

在狗身上加了移动底盘和一圈 2D 激光雷达，配 `slam_toolbox` 做在线建图。

### 移动底盘

狗原本是 `worldbody` 下的固定 body（`dofnum=0`），**根本没法巡视**。现在给
`dog_base` 加了三个自由度 + 速度执行器，等效差速轮式底盘（可原地转向）：

| 关节 | 类型 | 执行器 | 说明 |
|---|---|---|---|
| `base_x` | slide | `act_base_x` | 沿世界 +x（狗局部 −x） |
| `base_y` | slide | `act_base_y` | 沿世界 +y（狗局部 −y） |
| `base_yaw` | hinge | `act_base_yaw` | 绕竖直轴 |

`ctrl` 直接是速度指令（m/s、rad/s）。SLAM 只要里程计+激光，不必模拟步态。

两个必须显式设的参数，否则底盘不可用：

- **`DOG_MASS=60`**：默认 `inertiafromgeom` 会把 6 块视觉网格的体积都算进去，
  得出 **1494 kg** —— `vx=0.5` 指令 4 s 只走 0.73 m，停指令后还滑行十几度。
- **`BASE_DAMPING=6` / `BASE_VEL_KV=1200`**：速度执行器和关节阻尼是**对抗**的，
  稳态 `v/v_cmd = kv/(kv+damping)`。原先 400/60 只跑到指令的 87%，里程计
  系统性偏小 13%，SLAM 位姿会一路累积漂移。现在是 99.5%。

### 激光雷达

`LIDAR_POS=(0.454, 0, 1.0)`（狗局部系，机身后方），360 束 rangefinder、
1°/束、12 m 量程，sensor 名 `lidar_r000..r359`，参考系 site `lidar_frame`。

**站位是实测选出来的**，不是随手放的。用 720 束射线扫了一批候选点：

| 候选 | 视野净空 | 非墙面特征点 |
|---|---|---|
| 低位后方 z≈0.45 | 94~97% | **约 180** |
| 背顶 z≥1.70 | 100% | 约 25（几乎只有墙） |

背顶完全无自遮挡，但光束平面高过绝大多数家具，scan matching 和回环检测都会
很弱。低位能打到实验台腿/货箱/方凳/工人，这才是位姿图需要的约束。

两个坑：

- **雷达 site 必须直接挂在 `dog_base` 上**，不能另建子 body。MuJoCo 的
  rangefinder 只排除「site 所属那一个 body」的 geom；另建子 body 的话，减面
  网格里那 22 个离群顶点会在 0.33 m 处形成 **20 束假回波**，slam_toolbox 会
  当成贴着车身的障碍物。
- **`dog_collision` 尺寸不能用 STL 包围盒**。那 22 个离群顶点把 y 方向拉到
  ±1.44 m，碰撞盒长 2.88 m —— 比真实狗身长 4.6 倍，在 6 m 宽的房间里转不开身。
  现在用顶点 0.05/99.95 分位的稳健包围盒：0.91 × 0.63 × 1.29 m。

### 巡视航点

`PATROL_WAYPOINTS`（21 点，绕房间一圈回到起点）按当前机械狗正向为世界 `+x`
重新排过。第一段朝 `+x` 方向进入南侧通道；`x=-6.0` 的 `barrier_1` 横跨
`y∈[-1.2,1.2]`，所以从南侧绕过。北侧 `x≈-3.5` 有 `shuttle_a` 往复料车扫掠线，
自动巡视没有避障层，闭环必经段不再从那里硬穿，而是从东侧下行后走南侧回到西侧。
MuJoCo 级仿真实测约 86 s 跑完约 33.4 m 全程并回到起点。

### 数据质量（旧路线实测，决定 SLAM 能不能收敛）

下表是调整狗朝向和巡视线路前的 16 点路线数据。当前 21 点路线已经过 MuJoCo
闭环仿真，但 SLAM 质量指标需要重新跑 `slam_toolbox` 后复测。

| 指标 | 实测值 | 判据 |
|---|---|---|
| 里程计漂移 | 0.096 m / 28.7 m（0.33%） | 要看得见但可纠正 |
| 相邻关键帧扫描重叠 | 91.6%（最低 84.2%） | scan matching 需 >40% |
| 首尾帧重叠 | 88.3% | 高 = 回环可检测 |
| 回环闭合间隙 | 0.257 m | — |
| 关键帧数 | 217 | — |

里程计**故意加了噪声**（每米 0.01 m、每弧度 0.006 rad）。完全无噪声的里程计会
让位姿图退化成纯推算，回环检测和 scan matching 都得不到检验。激光另加
0.012 m 测距噪声（Hokuyo/RPLidar 量级）。

### 两个 Python 环境（重要）

Foxy 的 `rclpy` 只有 **cpython-38** 扩展，而本项目的 mujoco 3.10 只支持
**py3.11**（py3.8 能装的最高版是 mujoco 3.2.3）。所以按 XML 边界劈开：

- **建模**（`build_scene.py` / `build_robot.py`）跑 py3.11 + mujoco 3.10；
- **ROS 桥接**（`slam_bridge.py`）跑 py3.8 + mujoco 3.2.3。

`build_robot.py` 会额外输出一份 `*_py38.xml`，去掉 mujoco 3.10 写出、3.2.3
不认识的 `texture colorspace` 属性（其余逐字节相同）。**不要**把
`scene_with_robot.xml` 直接喂给 py3.8。

### SmolVLA 本地推理

SmolVLA 推理使用 `openpi/.venv` 里的 Python 3.11 环境。这个环境已经装有
LeRobot / Torch，并将 Python `mujoco` 包切到 3.8.1；离屏渲染走 OSMesa，
需要在启动进程前预加载 `/lib/x86_64-linux-gnu/libOSMesa.so`。直接用包装脚本即可：

```bash
./run_smolvla_infer.sh --scene-check-only --preview
```

上面只编译 `smolvla_scene.xml`、渲染三路 256×256 RGB 观测，并保存
`smolvla_preview.png`，不会加载 SmolVLA 权重。完整推理需要两个本地目录：

- `../../../models/smolvla_base`：SmolVLA policy checkpoint，当前已在本机。
- `../../../models/SmolVLM2-500M-Video-Instruct`：VLM backbone/tokenizer。

网络可用时下载 backbone。脚本默认走 `https://hf-mirror.com`，并只下载
PyTorch 推理需要的 config/tokenizer/processor/`model.safetensors`，不会拉 ONNX 变体：

```bash
./download_smolvlm.sh
```

如果需要改回官方站或加代理，先设置 `HF_ENDPOINT` / `HTTPS_PROXY` 再运行。
下载完成后完整跑一步推理：

```bash
./run_smolvla_infer.sh --device cpu --steps 1
```

有可用 CUDA 时可把 `--device cpu` 改为 `--device cuda`。如果 backbone 放在其他目录，
可传参覆盖：

```bash
./run_smolvla_infer.sh --vlm-model-dir /path/to/SmolVLM2-500M-Video-Instruct --steps 1
```

### SmolVLA 可视化 trace

如果想看 SmolVLA 在 MuJoCo 场景里的可观测推理/执行过程，打开 `--trace`：

```bash
./run_smolvla_infer.sh --device cpu --steps 1 --apply --trace --preview
```

输出在 `smolvla_trace/`：

- `index.html`：可直接用浏览器打开的报告；
- `trace.json`：同一份数据的机器可读版本；
- `images/step_000_obs.png` / `images/step_000_after.png`：推理前后三路 MuJoCo 相机拼图。

报告会展示：

- 三路 256×256 RGB 观测图；
- 当前语言任务、模型目录、设备、动作块长度；
- SmolVLA 的 action queue 状态（例如 `0 -> 49` 表示刚生成 50 步动作块并弹出第 1 步）；
- 当前步 6 个关节目标、执行前关节状态、执行后关节状态；
- 50 步 action chunk 的折线图和前若干行动作表；
- preprocess / policy / postprocess / MuJoCo step 的耗时。

这份 trace 展示的是模型和仿真的**可观测信号**，不是模型隐藏的逐字思维链。
要看连续执行过程，可增加步数：

```bash
./run_smolvla_infer.sh --device cpu --steps 10 --apply --trace --sim-steps-per-action 20
```

### ROS 接口

`slam_bridge.py` 发布 `/scan`（10 Hz LaserScan）、`/odom`、`/clock`，
TF 链 `odom → base_footprint → base_link → laser`；订阅 `/cmd_vel`
（收到外部指令就停止自动巡视，交给 Nav2）。坐标系遵循 REP-103/REP-105：
`base_link` 的 +x 为前进方向，laser 的 θ=0 也是正前方。

### MuJoCo + RViz 实时建图与保存地图

下面流程会同时打开 MuJoCo 仿真界面、启动 `slam_toolbox` 在线建图，并用 RViz
实时显示 `/map`、`/scan` 和 TF。需要图形界面（桌面环境、X11 或 WSLg）。

```bash
# 只需装一次；需要你自己输 sudo 密码
sudo apt update
sudo apt install -y ros-foxy-slam-toolbox ros-foxy-nav2-map-server ros-foxy-rviz2

source /opt/ros/foxy/setup.bash
python3 build_robot.py --scene scene.xml --out scene_with_robot.xml
```

上面会生成 `scene_with_robot.xml` 和 `scene_with_robot_py38.xml`。后者是
`slam_bridge.py` 在 ROS Foxy / Python 3.8 下要加载的兼容版。
修改 `build_robot.py` 里的 `LIDAR_POS`、底盘、执行器或机器人资源后，也要重新执行
这条带 `--scene` 的命令；只运行 `python3 build_robot.py --out robot.xml` 只会更新
独立机器人模型，不会更新 SLAM 实际加载的场景模型。

终端 1：从干净状态一键启动自动巡视、MuJoCo 查看器、SLAM 和 RViz：

```bash
source /opt/ros/foxy/setup.bash
./slam/run_slam.sh --fresh --view --rviz
```

启动后会出现两个窗口：

- MuJoCo viewer：显示狗在实验室里按航点自动巡视。
- RViz：加载 `slam/slam.rviz`，实时显示激光、TF 和逐步生成的 `/map`。

终端 2：确认数据正常发布：

```bash
source /opt/ros/foxy/setup.bash
ros2 topic hz /scan     # 应约为 10 Hz
ros2 topic hz /map      # 应约为 1 Hz
```

等狗至少跑完一圈、RViz 中地图基本闭合后保存地图：

```bash
source /opt/ros/foxy/setup.bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/lab_map
```

保存成功后会得到：

- `maps/lab_map.yaml`：地图元数据
- `maps/lab_map.pgm`：占据栅格图片

如果还想保存 `slam_toolbox` 的位姿图，便于之后继续优化或定位，也可以执行：

```bash
source /opt/ros/foxy/setup.bash
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: maps/lab_posegraph}"
```

`use_sim_time` 必须为 `true`（桥接发 `/clock`），否则 TF 查找全部超时。

如果不加 `--fresh`，脚本会先检查 ROS graph 中是否还有旧的
`/mujoco_slam_bridge`、`/slam_toolbox` 或 `/rviz2` 节点；检测到就拒绝启动。
旧节点没退出时，新旧 `/map`、`/scan`、TF 会混在同一个 ROS_DOMAIN_ID 里，
RViz 看起来就像“接着上一次建图”。正常退出请在运行脚本的终端按 `Ctrl-C`；
如果上次终端已关闭或节点残留，用 `--fresh` 清理后再启动。

其他启动方式：

```bash
./slam/run_slam.sh              # 自动巡视建图，不开 MuJoCo/RViz 窗口
./slam/run_slam.sh --view       # 自动巡视建图，只额外开 MuJoCo 查看器
./slam/run_slam.sh --rviz       # 自动巡视建图，只额外开 RViz
./slam/run_slam.sh --fresh --view --rviz  # 清理旧节点后重新建图
./slam/run_slam.sh --teleop     # 不巡视，等待外部 /cmd_vel，例如 Nav2 或 teleop
```

## 文件
- `build_scene.py` —— 场景生成器，写出 `scene.xml`
- `slam_bridge.py` —— MuJoCo → ROS 2 桥接（**必须 python3.8**）
- `slam/mapper_params_online_async.yaml` —— slam_toolbox 参数
- `slam/slam.launch.py` —— 起 slam_toolbox 异步建图节点
- `slam/run_slam.sh` —— 一键起桥接 + 建图
- `scene.xml` —— 场景模型（不含机器人）
- `build_robot.py` —— 机器人装配；可单独输出，也可装进场景
- `robot.xml` / `scene_with_robot.xml` —— 最近一次生成的机器人 / 场景+机器人
- `interactive_view.py` —— 交互查看：viewer 主窗口 + 双手相机实时同步画面
- `render_view.py` —— EGL 离屏出图 / 录像

## 用法
```bash
python3 build_scene.py            # 生成 scene.xml 并校验编译
python3 build_scene.py --view     # 生成后打开交互式查看器（需图形界面/WSLg）
python3 build_scene.py --seed 0   # 固定随机种子（仅影响台面小道具摆放）

# 机器人装配
python3 build_robot.py --out robot.xml                              # 独立机器人；同时写 robot_py38.xml
python3 build_robot.py --scene scene.xml --out scene_with_robot.xml  # 装入场景；同时写 scene_with_robot_py38.xml，SLAM 用这个

# 改 LIDAR_POS / 底盘 / 机器人资源后，用这条刷新 SLAM 实际加载的两个场景 XML：
python3 build_robot.py --scene scene.xml --out scene_with_robot.xml

# 交互查看（viewer 主窗口 + 两手相机 cv2 窗口）
python3 interactive_view.py                # 空格=暂停 W=摆臂 R=复位ready ESC/q=退出
python3 interactive_view.py --no-cams      # 只开 viewer 主窗口
python3 interactive_view.py --offscreen    # 无显示环境：EGL 离屏 + 手写鼠标控制

# 离屏出图 / 录像（相机名可换 overview / top / cam_hand_l / cam_hand_r）
python3 render_view.py
python3 render_view.py --cam cam_hand_l --video 5
```

> **`scene.xml` 是手工调过的，它才是准的，`build_scene.py` 的输出不等于它。**
> 直接重跑 `build_scene.py --out scene.xml` 会覆盖掉下面这些手工改动：
> - `cabinet`（含双开门）和 `barrier_1` —— 生成器不产出；
> - 生成器会额外产出 `sweep_1` 扫掠杆 —— 手工版把它换成了同位置(x=-6.0)的静态 `barrier_1`；
> - `shuttle_a` 导轨与弹簧参数（导轨中心 y=1.5 半长 1.8、`springref=1.58`）——
>   生成器的 `rail_shuttle(-3.5,-0.9,0.9)` 给的是另一组值；
> - 工人腿粗细 `size=0.085` —— 生成器是 0.065。
>
> 要改场景，直接编辑 `scene.xml`；若同时想让生成器保持可复现，两边都改
> （`corner_table` / `calib_board` 就是两边都加了的，已验证生成结果逐字节一致）。

也可用官方工具直接查看已生成的模型：
```bash
python3 -m mujoco.viewer --mjcf=scene.xml
```

## 布局与调参说明（见 build_scene.py 顶部常量与 build_xml 中的列表）
- 主工作台在原点，工人在 `x=-10` 处面朝 +x，中间是 10 m 走廊。
- 静态家具在 `furniture` 列表、静态障碍物在 `statics` 列表，均逐个显式摆放。
- 动态物体在 `movers` 列表（当前 2 个）。往复运动靠 `slide/hinge` 关节的
  `stiffness` + `springref` 实现：平衡点设在振幅处、起点在 0，故在 `[0, 2·振幅]`
  间往复；周期 `T ≈ 2π·sqrt(m/k)`，调 `mass` / `stiffness` 即可改变快慢。
- 注：`build_scene.py` 中仍保留了 `pendulum` / `conveyor` 等辅助函数，
  如需增加运动障碍可直接在 `movers` 列表中调用。
- 每个运动部件与其自身静态支架通过 `<contact><exclude>` 关闭碰撞
  （支架只是结构，不是障碍物）。
