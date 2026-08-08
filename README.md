# JBGS 
## 技术路线
- mujoco 3.？.?仿真环境，测试机器人在实验室环境的运行状况
- 使用VLA模型端到端控制机械臂
- slam扫描点云，规划路径

## 项目进度
### 仿真环境
- mujoco环境配置完成，初步构建了
- 场景通过SolidWorks完成建模并转化为URDF与xml的语言
- 状态：初步完成，需要进一步细化螺丝刀、扳手等场景

### VLA模型
- 找到huggingface上使用同类型机械臂训练的SmolVLA权重，并在mujoco环境下测试了SmolVLA模型的推理
- 配置好了openpi pi0的推理环境并进行了推理
### Slam
- 问zyx去 


## 项目结构

| 路径 | 作用 |
| --- | --- |
| `README.md` | 当前文件，说明整个工作区有什么、怎么跑、哪些目录是核心。 |
| `mujoco-3.10.0/` | 本地 MuJoCo 3.10.0 发行包，包含官方二进制、库、示例模型，以及本项目新增的 `worker_scene`。 |
| `mujoco-3.10.0/model/worker_scene/` | 主仿真场景目录。包含实验室内景、四足狗 + 双 RM65 臂 + 双 dexhand 手、2D 激光 SLAM、SmolVLA 桌面推理场景和 trace 回放脚本。 |
| `mujoco-3.10.0/model.test/worker_scene/` | ROS Foxy / Python 3.8 测试副本。重点是 2D/3D SLAM、RTAB-Map、Nav2 地图产物和导航脚本。 |
| `models/` | 本地 Hugging Face / LeRobot 模型权重目录。当前含 SmolVLA policy checkpoint 与 SmolVLM2 backbone/tokenizer。 |
| `openpi/` | Physical Intelligence `openpi` 仓库副本和 Python 环境。这里的 `.venv` 被 SmolVLA 推理脚本直接使用。 |
| `claude/` | 项目本地 Claude Code 安装、插件、会话和配置目录。不是机器人业务源码，可能包含本机认证或历史状态。 |
| `codex/` | 项目本地 Codex 运行环境、Node/npm、缓存、技能、会话和配置目录。不是机器人业务源码，可能包含本机状态。 |
| `.hf_cache/` | Hugging Face 缓存位置，`run_smolvla_infer.sh` 默认把 `HF_HOME` 指到这里。 |
| `.uv-cache/` | uv 依赖缓存。 |

根目录本身不是一个标准 Git 仓库；`openpi/` 目录内部才是独立 Git 仓库。这个工作区更接近一份可运行的本机实验环境快照，而不是单一 Python 包或库。

## 核心目录详解

### `mujoco-3.10.0/`

这是本地 MuJoCo 3.10.0 发行目录，包含：

- `bin/`：`simulate`、`compile`、`mujoco_studio`、`record` 等 MuJoCo 工具。
- `lib/`：MuJoCo 共享库。
- `include/`：C/C++ 头文件。
- `sample/`：官方 C++ 示例。
- `wasm/`：MuJoCo WebAssembly 产物。
- `model/`：官方示例模型和本项目的 `worker_scene`。
- `model.test/`：本项目用于 ROS/SLAM 的测试模型副本。

官方示例模型包括 car、hammock、humanoid、flex、tendon_arm、replicate 等；本项目新增内容主要集中在 `model/worker_scene` 和 `model.test/worker_scene`。

### `mujoco-3.10.0/model/worker_scene/`

主工作目录，包含三类内容。

第一类是大实验室场景：

- `scene.xml`：手工调过的实验室内景模型。它是当前准文件，直接重跑 `build_scene.py --out scene.xml` 会覆盖手工改动。
- `build_scene.py`：实验室内景生成器，包含房间、实验台、置物架、工人、货箱、动态障碍物、角落标定桌等元素。
- `view_overview.png`、`view_top.png`：场景预览图。

第二类是机器人装配与查看：

- `build_robot.py`：装配四足狗、双 RM65-6F 机械臂、双 dexhand021 灵巧手、手背相机、移动底盘和 2D 激光雷达。
- `robot.xml` / `robot_py38.xml`：最近一次生成的独立机器人模型，以及兼容 Python 3.8 / MuJoCo 3.2.3 的版本。
- `scene_with_robot.xml` / `scene_with_robot_py38.xml`：场景 + 机器人组合模型；`*_py38.xml` 供 ROS Foxy 桥接使用。
- `interactive_view.py`：交互查看器，可同步显示 MuJoCo viewer 和双手相机画面。
- `render_view.py`：离屏渲染出图或录像。
- `assets/`：URDF、STL mesh、标定板纹理等模型资源。

第三类是 SmolVLA 推理与 trace：

- `build_smolvla_scene.py`：生成紧凑桌面推理场景 `smolvla_scene.xml`。该场景包含角落桌、红苹果、标定板、RM65 六轴臂、三路 256x256 固定 RGB 相机和 6 个 position actuator。
- `smolvla_scene.xml`：当前用于本地 SmolVLA 推理的 MuJoCo XML。
- `smolvla_infer.py`：加载 MuJoCo 场景和 SmolVLA，构造 LeRobot frame，执行 `select_action`，可选择写回 MuJoCo 并记录 trace。
- `run_smolvla_infer.sh`：推理包装脚本，自动选择 `openpi/.venv/bin/python`、设置模型目录、OSMesa 离屏渲染和 `HF_HOME`。
- `replay_smolvla_trace.py`：把保存的 `trace.json` 动作序列在 MuJoCo viewer 中回放。
- `download_smolvlm.sh`：下载 SmolVLM2 backbone/tokenizer 的辅助脚本，默认使用 Hugging Face 镜像。
- `smolvla_preview.png`：三路相机预览图。
- `smolvla_trace/`：最近一次推理或场景检查生成的 HTML/JSON/图片报告。

### `mujoco-3.10.0/model/worker_scene/slam/`

主场景的 2D SLAM 配置：

- `run_slam.sh`：一键启动 MuJoCo 桥接、`slam_toolbox` 和可选 RViz / viewer。
- `slam.launch.py`：启动 `slam_toolbox` 异步在线建图节点。
- `mapper_params_online_async.yaml`：`slam_toolbox` 参数。
- `slam.rviz`：RViz 配置。

桥接脚本是 `../slam_bridge.py`。它发布 `/scan`、`/odom`、`/clock` 和 TF，订阅 `/cmd_vel`。ROS Foxy 的 `rclpy` 需要 Python 3.8，因此桥接加载的是 `scene_with_robot_py38.xml`。

### `mujoco-3.10.0/model.test/worker_scene/`

这个目录是偏 ROS Foxy / Python 3.8 的 SLAM 测试区，和主 `model/worker_scene` 有大量相似资源，但额外包含 3D 建图与导航内容：

- `slam_bridge.py`：2D LaserScan 桥接。
- `slam_bridge_3d.py`：3D PointCloud2 桥接。
- `gen_3d_xml.py`：从 2D XML 生成 16 层 x 180 束的 3D rangefinder 场景。
- `nav_p2p.py`、`send_goal.py`：点到点导航和目标发送辅助脚本。
- `slam/run_slam.sh`：2D `slam_toolbox` 流程。
- `slam/run_slam_3d.sh`：3D RTAB-Map 流程。
- `slam/save_map_3d.sh`：保存 3D 地图。
- `maps/`：保存的 2D/3D 栅格地图与 `rtabmap.db`。

如果目标是跑 SmolVLA 推理，优先使用 `mujoco-3.10.0/model/worker_scene/`。如果目标是 ROS 2 Foxy 下验证 SLAM、RTAB-Map 或 Nav2 地图，使用 `mujoco-3.10.0/model.test/worker_scene/`。

### `models/`

当前有两套本地模型：

| 路径 | 内容 |
| --- | --- |
| `models/smolvla_base/` | LeRobot SmolVLA base policy checkpoint。真正的 policy 权重在 `model.safetensors`，`config.json` 定义 policy 结构，`policy_preprocessor.json` / `policy_preprocessor_step_5_normalizer_processor.safetensors` 负责输入归一化，`policy_postprocessor.json` / `policy_postprocessor_step_0_unnormalizer_processor.safetensors` 负责输出反归一化。 |
| `models/SmolVLM2-500M-Video-Instruct/` | SmolVLA 使用的 VLM backbone/tokenizer。核心权重在 `model.safetensors`，其余 `tokenizer*.json`、`processor_config.json`、`preprocessor_config.json`、`generation_config.json` 等文件共同组成本地推理目录。 |

`smolvla_infer.py` 默认使用这两个目录，并开启 `local_files_only=True`，因此完整推理不依赖在线拉取权重。缺少 VLM backbone 时，可以在 `worker_scene` 里运行 `./download_smolvlm.sh` 下载。

### `openpi/`

这是 Physical Intelligence 的 `openpi` 仓库副本，包含 π0、π0-FAST、π0.5 等机器人 VLA 模型代码、训练/推理脚本和示例。当前工作区主要使用它的虚拟环境：

```text
openpi/.venv/bin/python
```

`run_smolvla_infer.sh` 默认使用这个 Python 解释器。这个环境已用于 LeRobot / Torch / MuJoCo 推理。完整 `openpi` 项目本身还有：

- `src/openpi/`：核心模型、policy、训练配置。
- `examples/`：DROID、ALOHA、LIBERO、UR5 等示例。
- `scripts/`：训练、服务、统计等脚本。
- `docs/`：Docker、远程推理、归一化统计等说明。
- `pyproject.toml` / `uv.lock`：uv 管理的依赖配置。

### SmolVLA 部署方式

SmolVLA 在这个工作区里是“本地 checkpoint + 本地 VLM backbone + 本地 MuJoCo 场景”三件套，不依赖外部推理服务。

1. 启动入口是 `mujoco-3.10.0/model/worker_scene/run_smolvla_infer.sh`。
   - 默认解释器：`openpi/.venv/bin/python`
   - 默认 policy 目录：`models/smolvla_base`
   - 默认 VLM 目录：`models/SmolVLM2-500M-Video-Instruct`
   - 默认离屏参数：`MUJOCO_GL=osmesa`、`PYOPENGL_PLATFORM=osmesa`
   - 默认缓存：`HF_HOME=.hf_cache`
2. 真正的推理逻辑在 `mujoco-3.10.0/model/worker_scene/smolvla_infer.py`。
   - `PreTrainedConfig.from_pretrained(..., local_files_only=True)`
   - `SmolVLAPolicy.from_pretrained(..., local_files_only=True)`
   - `make_pre_post_processors(...)` 会从同一个 `model_dir` 读取预处理和后处理器
3. policy 参数文件都放在 `models/smolvla_base/`。
   - `model.safetensors` 是实际权重
   - `config.json` 定义 policy 结构
   - `policy_preprocessor.json` 和 `policy_preprocessor_step_5_normalizer_processor.safetensors` 负责输入归一化
   - `policy_postprocessor.json` 和 `policy_postprocessor_step_0_unnormalizer_processor.safetensors` 负责输出反归一化
4. VLM 参数文件都放在 `models/SmolVLM2-500M-Video-Instruct/`。
   - `model.safetensors` 是 backbone 权重
   - tokenizer / processor / generation 配置文件都在同一个目录里
5. 如果需要改目录，可以直接传 `--model-dir` 和 `--vlm-model-dir`；如果目录缺失，脚本会直接报错，不会偷偷联网下载。

## 主要工作流

### 1. 检查 SmolVLA 场景和三路相机

这个命令只编译 `smolvla_scene.xml`、渲染三路 RGB 观测并保存预览，不加载 SmolVLA 权重，适合先检查 MuJoCo、OSMesa 和相机是否正常。

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model/worker_scene

./run_smolvla_infer.sh \
  --scene-check-only \
  --preview \
  --trace
```

输出：

```text
smolvla_preview.png
smolvla_trace/index.html
smolvla_trace/trace.json
smolvla_trace/images/step_000_obs.png
```

### 2. CPU 跑 SmolVLA 推理

CPU 模式适合稳定复现和 smoke test。它会比较慢，看到 `No accelerated backend detected` 之类提示属于正常情况。

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model/worker_scene

./run_smolvla_infer.sh \
  --device cpu \
  --steps 10 \
  --apply \
  --trace \
  --sim-steps-per-action 20
```

输出会覆盖刷新：

```text
smolvla_trace/index.html
smolvla_trace/trace.json
smolvla_trace/images/step_000_obs.png
smolvla_trace/images/step_000_after.png
...
```

默认任务文本是：

```text
Pick up the apple on the corner table.
```

当前紧凑 SmolVLA 场景只有 6 个机械臂关节控制量，没有 gripper/夹爪。如果只是观察机械臂靠近苹果的行为，更建议使用：

```bash
./run_smolvla_infer.sh \
  --device cpu \
  --steps 10 \
  --apply \
  --trace \
  --task "Reach the red apple."
```

### 3. GPU 跑完整 action chunk

SmolVLA 通常一次生成 50 步 action chunk，并通过内部 action queue 逐步消费。GPU 跑 50 步可以完整观察第一个 chunk 的生成与消费过程。

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model/worker_scene

./run_smolvla_infer.sh \
  --device cuda \
  --steps 50 \
  --apply \
  --trace \
  --sim-steps-per-action 20
```

典型 queue 变化：

```text
step 0:  queue 0  -> 49
step 1:  queue 49 -> 48
...
step 49: queue 1  -> 0
```

第 0 步会触发模型生成完整 action chunk，耗时最高；后续 step 主要从 queue 中取动作，通常快很多。

### 4. 回放最近一次 SmolVLA trace

回放脚本读取最新 `smolvla_trace/trace.json`，把保存下来的动作序列重新写入 MuJoCo actuator。可以开 viewer 回放，也可以直接导出 MP4。

```bash
cd /home/ee304/jbgs

/home/ee304/jbgs/mujoco-3.10.0/model/worker_scene/run_replay_smolvla_trace.sh \
  --video-out mujoco-3.10.0/model/worker_scene/smolvla_trace/replay.mp4
```

默认固定视角是 `camera2`。如果想开窗口，直接运行
`mujoco-3.10.0/model/worker_scene/replay_smolvla_trace.py`，关闭 MuJoCo viewer 窗口即可退出。

### 5. 重新生成 SmolVLA 桌面场景

如果修改了 `build_smolvla_scene.py`、RM65 URDF 或相关 mesh，可以重新生成 `smolvla_scene.xml`：

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model/worker_scene

openpi/.venv/bin/python build_smolvla_scene.py
```

生成后建议先做场景检查：

```bash
./run_smolvla_infer.sh --scene-check-only --preview --trace
```

### 6. 查看或渲染大实验室场景

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model/worker_scene

python3 interactive_view.py
python3 interactive_view.py --no-cams
python3 render_view.py
python3 render_view.py --cam cam_hand_l --video 5
```

如果只想用 MuJoCo 官方 viewer 打开 XML：

```bash
python3 -m mujoco.viewer --mjcf=scene.xml
```

注意：`scene.xml` 是手工调过的准文件。直接执行 `python3 build_scene.py --out scene.xml` 会覆盖手工修改。需要改大场景时，优先直接编辑 `scene.xml`，或者确保生成器和手工 XML 同步更新。

### 7. 2D SLAM 建图

主场景的 2D SLAM 流程位于 `mujoco-3.10.0/model/worker_scene/slam/`。需要 ROS 2 Foxy、`slam_toolbox`、RViz 和 Python 3.8 环境。

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model/worker_scene

source /opt/ros/foxy/setup.bash
./slam/run_slam.sh --fresh --view --rviz
```

启动后会同时有：

- MuJoCo viewer：显示四足狗在实验室中自动巡视。
- RViz：显示 `/scan`、TF 和逐步生成的 `/map`。

检查话题：

```bash
ros2 topic hz /scan
ros2 topic hz /map
```

保存地图：

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/lab_map
```

输出：

```text
maps/lab_map.yaml
maps/lab_map.pgm
```

### 8. 3D SLAM / RTAB-Map 测试

3D 建图流程在测试副本中：

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene

source /opt/ros/foxy/setup.bash
python3.8 gen_3d_xml.py
./slam/run_slam_3d.sh --view --fresh
```

保存 3D 地图：

```bash
./slam/save_map_3d.sh
```

主要产物：

```text
maps/lab_map_3d.yaml
maps/lab_map_3d.pgm
maps/rtabmap.db
```

## SmolVLA 输入和输出

每一步推理输入由 5 部分组成：

```text
observation.images.camera1   256x256 RGB
observation.images.camera2   256x256 RGB
observation.images.camera3   256x256 RGB
observation.state            6 维关节状态
task                         语言任务指令
```

输出会被后处理成 6 个具名关节目标：

```text
joint_1
joint_2
joint_3
joint_4
joint_5
joint_6
```

如果传入 `--apply`，脚本会把动作写入：

```text
act_joint_1 ... act_joint_6
```

然后每个动作推进固定数量的 MuJoCo step：

```text
--sim-steps-per-action 20
```

动作会被裁剪到 MuJoCo XML 中定义的关节范围内。

## Trace 报告

启用 `--trace` 后，`smolvla_infer.py` 会写出：

```text
smolvla_trace/index.html
smolvla_trace/trace.json
smolvla_trace/images/step_XXX_obs.png
smolvla_trace/images/step_XXX_after.png
```

`obs.png` 是动作前的三路相机拼图，`after.png` 是动作应用后的三路相机拼图。

`trace.json` 的核心结构：

```text
meta:
  xml
  model_dir
  vlm_model_dir
  task
  device
  steps
  apply
  sim_steps_per_action
  scene_check_only
  chunk_size
  n_action_steps
  camera_names

steps:
  step
  generated_chunk
  queue_len_before
  queue_len_after
  obs_image
  after_image
  state_before
  action
  action_chunk
  state_after
  stages
```

字段含义：

- `state_before`：当前 step 推理前的 6 个关节角。
- `action`：SmolVLA 当前弹出的 6 维关节目标。
- `state_after`：MuJoCo 应用动作后的 6 个关节角。
- `action_chunk`：只有新生成 chunk 的 step 会记录完整动作块。
- `generated_chunk`：是否在该 step 触发模型生成新 action chunk。
- `queue_len_before` / `queue_len_after`：action queue 消费情况。
- `stages`：render、preprocess、policy、postprocess、MuJoCo step 的耗时。

`index.html` 会把这些信息可视化成：

- 三路相机输入。
- 动作应用后的三路相机画面。
- 关节状态条。
- 关节目标条。
- action chunk 曲线和动作表。
- 各阶段耗时。

## 环境约束

这个工作区同时涉及多个 Python / MuJoCo / ROS 组合，需要按用途区分。

| 用途 | 推荐环境 | 说明 |
| --- | --- | --- |
| SmolVLA 推理 | `openpi/.venv/bin/python`，Python 3.11 | `run_smolvla_infer.sh` 默认使用；离屏渲染走 OSMesa。 |
| MuJoCo 3.10 建模 | Python 3.11 + MuJoCo 3.10 | `build_scene.py`、`build_robot.py` 使用较新的 MuJoCo API。 |
| ROS Foxy 桥接 | Python 3.8 + MuJoCo 3.2.3 | Foxy 的 `rclpy` 绑定是 cpython-38；桥接使用 `*_py38.xml`。 |
| 2D SLAM | ROS 2 Foxy + `slam_toolbox` | 发布 `/scan`、`/odom`、`/clock`，RViz 使用 `use_sim_time=true`。 |
| 3D SLAM | ROS 2 Foxy + RTAB-Map | 位于 `model.test/worker_scene`。 |

`run_smolvla_infer.sh` 默认设置：

```text
SMOLVLA_MODEL_DIR=models/smolvla_base
SMOLVLA_VLM_MODEL_DIR=models/SmolVLM2-500M-Video-Instruct
MUJOCO_GL=osmesa
PYOPENGL_PLATFORM=osmesa
HF_HOME=/home/ee304/jbgs/.hf_cache
```

如果 OSMesa 路径不同，可以用环境变量覆盖：

```bash
SMOLVLA_OSMESA_LIB=/path/to/libOSMesa.so ./run_smolvla_infer.sh --scene-check-only
```

## 重要注意事项

- 当前根目录是本机实验工作区快照，包含模型权重、缓存、trace 图片、代理工具状态和可能的认证配置。不要直接把整个目录当作可公开发布的干净源码仓库。
- `claude/`、`codex/`、`.hf_cache/`、`.uv-cache/`、`.claude/`、`.codex/`、`.agents/` 是本地工具/缓存/状态目录，不是机器人仿真核心代码。
- `models/` 体积较大，属于运行时权重依赖；如果迁移到新机器，需要确认这两个模型目录完整存在。
- `smolvla_trace/` 会被新的推理命令覆盖刷新；需要保留某次结果时，应先另存该目录。
- `scene.xml` 是手工调过的实验室场景准文件；重跑生成器可能覆盖手工细节。
- 当前 SmolVLA 桌面场景没有 gripper，`Pick up the apple...` 更像语言输入 smoke test，不代表物理上能完成抓取。
- GPU 推理需要普通终端能访问 NVIDIA 设备；受限沙箱或无图形环境可能看不到 GPU 或 viewer。
- 运行时出现 Transformers / LeRobot 的 preprocessor deprecation 警告，通常是上游兼容性提示，不影响当前推理和 trace 生成。

## 快速定位

常用入口：

```text
根说明                         README.md
主场景说明                     mujoco-3.10.0/model/worker_scene/README.md
SmolVLA 推理脚本               mujoco-3.10.0/model/worker_scene/smolvla_infer.py
SmolVLA 包装脚本               mujoco-3.10.0/model/worker_scene/run_smolvla_infer.sh
SmolVLA 场景                   mujoco-3.10.0/model/worker_scene/smolvla_scene.xml
SmolVLA trace HTML             mujoco-3.10.0/model/worker_scene/smolvla_trace/index.html
trace 回放                     mujoco-3.10.0/model/worker_scene/replay_smolvla_trace.py
SmolVLA 回放视频               mujoco-3.10.0/model/worker_scene/run_replay_smolvla_trace.sh
大实验室场景                   mujoco-3.10.0/model/worker_scene/scene.xml
机器人装配                     mujoco-3.10.0/model/worker_scene/build_robot.py
2D SLAM 启动                   mujoco-3.10.0/model/worker_scene/slam/run_slam.sh
3D SLAM 测试                   mujoco-3.10.0/model.test/worker_scene/slam/run_slam_3d.sh
OpenPI 环境                    openpi/.venv/bin/python
SmolVLA checkpoint             models/smolvla_base
SmolVLM2 backbone              models/SmolVLM2-500M-Video-Instruct
```

## 环境配置与部署步骤

先把系统依赖和 Python 环境搭起来，再放模型权重，最后做功能验证。

### 1. 目录前提

这个工作区默认把 `mujoco-3.10.0/`、`openpi/`、`models/` 放在同一级目录下。SmolVLA 相关脚本默认也在这个根目录下运行，因此如果你换了路径，最好保持这三个目录的相对位置不变。

### 2. 系统依赖

SmolVLA 和 MuJoCo 这套流程会同时碰到 GUI OpenGL、离屏渲染和视频导出，所以基础系统依赖至少要覆盖以下几类：

- `libgl1`、`libglfw3`：MuJoCo viewer / GLFW 相关
- `libosmesa6`：无窗口离屏渲染
- `ffmpeg`：MP4 导出
- ROS 2 Foxy 相关包：只有在跑 SLAM / Nav2 时才需要

一个比较直接的安装方式是：

```bash
sudo apt update
sudo apt install -y libgl1 libglfw3 libosmesa6 ffmpeg
sudo apt install -y ros-foxy-slam-toolbox ros-foxy-nav2-map-server ros-foxy-rviz2
```

如果你只做 SmolVLA 推理，不跑 ROS SLAM，可以先不装 Foxy 相关包。

### 3. Python 环境

这个项目实际上分成三套 Python 运行环境：

| 用途 | 推荐环境 | 说明 |
| --- | --- | --- |
| SmolVLA 推理 | `openpi/.venv/bin/python`，Python 3.11 | `run_smolvla_infer.sh` 默认使用这个解释器。 |
| MuJoCo 3.10 建模 | 系统 Python 3.11 + MuJoCo 3.10 | `build_scene.py`、`build_robot.py`、`build_smolvla_scene.py` 使用这一套。 |
| ROS Foxy 桥接 | Python 3.8 + MuJoCo 3.2.3 | Foxy 的 `rclpy` 绑定是 cpython-38，桥接要加载 `*_py38.xml`。 |

如果你需要在新机器上复现 `openpi/.venv`，进入 `openpi/` 后按它自己的安装流程来：

```bash
cd /home/ee304/jbgs/openpi
GIT_LFS_SKIP_SMUDGE=1 uv sync
GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .
```

`GIT_LFS_SKIP_SMUDGE=1` 是为了避免依赖拉取时把大体积权重文件一起抓下来。完成后，SmolVLA 相关脚本就统一使用 `openpi/.venv/bin/python`。

### 4. 模型权重

这套部署依赖两个本地模型目录：

- `models/smolvla_base/`：SmolVLA policy checkpoint
- `models/SmolVLM2-500M-Video-Instruct/`：SmolVLM2 backbone / tokenizer

如果第二个目录不在本机，可以在 `worker_scene` 目录里运行：

```bash
./download_smolvlm.sh
```

推理脚本默认是本地加载，所以这两个目录都应该完整存在于磁盘上。

### 5. 环境变量

`run_smolvla_infer.sh` 会替你设好最关键的运行参数：

- `SMOLVLA_MODEL_DIR=models/smolvla_base`
- `SMOLVLA_VLM_MODEL_DIR=models/SmolVLM2-500M-Video-Instruct`
- `MUJOCO_GL=osmesa`
- `PYOPENGL_PLATFORM=osmesa`
- `HF_HOME=.hf_cache`

如果机器上的 OSMesa 库路径不同，可以显式覆盖：

```bash
SMOLVLA_OSMESA_LIB=/path/to/libOSMesa.so ./mujoco-3.10.0/model/worker_scene/run_smolvla_infer.sh --scene-check-only
```

### 6. 功能验证

建议按下面顺序做验证，这样出问题时能快速定位是环境、模型还是场景本身有问题：

```bash
# 先验证场景编译和三路相机预览
./mujoco-3.10.0/model/worker_scene/run_smolvla_infer.sh --scene-check-only --preview

# 再验证完整推理
./mujoco-3.10.0/model/worker_scene/run_smolvla_infer.sh --device cpu --steps 1 --apply --trace --preview

# 需要时再导出回放视频
./mujoco-3.10.0/model/worker_scene/run_replay_smolvla_trace.sh --video-out mujoco-3.10.0/model/worker_scene/smolvla_trace/replay.mp4

# 如果要跑 ROS SLAM，再切到 Foxy 环境
source /opt/ros/foxy/setup.bash
./mujoco-3.10.0/model/worker_scene/slam/run_slam.sh --fresh --view --rviz
```

### 7. 部署时最容易弄错的地方

- `models/smolvla_base/model.safetensors` 才是 policy 权重，不是 `smolvla_scene.xml`
- `openpi/.venv/bin/python` 和系统 `python3` 不是一回事，别混用
- ROS Foxy 只在 SLAM / Nav2 这条线里需要，SmolVLA 推理本身不依赖 ROS
- 无窗口导出视频时要走 OSMesa，不要直接硬开 viewer
