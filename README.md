# JBGS Robotic Arm

这个仓库主要做三件事：

1. MuJoCo 仿真场景
2. SmolVLA 推理与回放
3. ROS 2 Foxy / SLAM / Nav2 测试

`README_NOTES.md` 里记录了本 README 没有写全、但部署时很容易踩坑的内容。先看这里，再看 notes。

## 一眼看懂目录

| 路径 | 作用 |
| --- | --- |
| `mujoco-3.10.0/model/worker_scene/` | 主场景目录，包含实验室场景、SmolVLA 推理、trace 回放、2D SLAM。 |
| `mujoco-3.10.0/model.test/worker_scene/` | ROS 2 Foxy 测试副本，主要用于 3D SLAM、RTAB-Map、Nav2。 |
| `models/` | 本地模型目录，放 SmolVLA policy checkpoint 和 SmolVLM2 backbone。 |
| `openpi/` | `openpi` 仓库副本，推理时优先使用其中的 Python 环境。 |
| `.hf_cache/` | Hugging Face 缓存目录。 |

## 先装什么

### 系统依赖

SmolVLA 和 MuJoCo 相关流程通常需要这些库：

```bash
sudo apt update
sudo apt install -y libgl1 libglfw3 libosmesa6 ffmpeg
```

如果你还要跑 ROS SLAM，再补 ROS 2 Foxy 相关包。

### Python 环境

这个仓库实际分成两条线：

| 场景 | 推荐 Python |
| --- | --- |
| SmolVLA 推理 | `openpi/.venv/bin/python` |
| ROS Foxy / SLAM | Python 3.8 |

不要把这两条线混在一起。

如果你想直接用 pip 装一版当前工作区需要的 Python 包，可以先看根目录的 `requirements.txt`。

### 模型权重

推理至少需要这两个目录都在：

- `models/smolvla_base/`
- `models/SmolVLM2-500M-Video-Instruct/`

其中：

- `models/smolvla_base/model.safetensors` 是 SmolVLA policy 权重
- `models/SmolVLM2-500M-Video-Instruct/model.safetensors` 是 VLM backbone 权重

如果只缺 VLM backbone，可以在 `mujoco-3.10.0/model/worker_scene/` 下运行：

```bash
./download_smolvlm.sh
```

注意：这只下载 SmolVLM2 backbone，不会替你下载 SmolVLA policy。

## SmolVLA 怎么跑

### 默认入口

```bash
cd mujoco-3.10.0/model/worker_scene
./run_smolvla_infer.sh
```

这个脚本会自动处理：

- Python 解释器选择
- 模型目录
- `HF_HOME`
- 无窗口渲染

### 最小验证

先做场景检查：

```bash
cd mujoco-3.10.0/model/worker_scene
./run_smolvla_infer.sh --scene-check-only --preview --trace
```

再跑一次完整推理：

```bash
cd mujoco-3.10.0/model/worker_scene
./run_smolvla_infer.sh --device cpu --steps 10 --apply --trace --sim-steps-per-action 20
```

如果你只想做轻量 smoke test，可以先把 `--steps` 改成 `1`。

### 任务文本

默认任务是：

```text
Pick up the apple on the corner table.
```

这个场景目前没有夹爪，任务更适合写成靠近目标一类的描述，比如：

```bash
./run_smolvla_infer.sh --device cpu --steps 10 --apply --trace --task "Reach the red apple."
```

## 主要脚本

### SmolVLA

- `mujoco-3.10.0/model/worker_scene/smolvla_infer.py`
- `mujoco-3.10.0/model/worker_scene/run_smolvla_infer.sh`
- `mujoco-3.10.0/model/worker_scene/replay_smolvla_trace.py`
- `mujoco-3.10.0/model/worker_scene/run_replay_smolvla_trace.sh`

### 场景生成

- `mujoco-3.10.0/model/worker_scene/build_smolvla_scene.py`
- `mujoco-3.10.0/model/worker_scene/build_scene.py`
- `mujoco-3.10.0/model/worker_scene/build_robot.py`

### SLAM

- `mujoco-3.10.0/model/worker_scene/slam/run_slam.sh`
- `mujoco-3.10.0/model.test/worker_scene/slam/run_slam_3d.sh`

## 典型工作流

### 1. 检查场景和相机

```bash
cd mujoco-3.10.0/model/worker_scene
./run_smolvla_infer.sh --scene-check-only --preview --trace
```

输出会生成：

- `smolvla_preview.png`
- `smolvla_trace/index.html`
- `smolvla_trace/trace.json`

### 2. 跑 SmolVLA 推理

```bash
cd mujoco-3.10.0/model/worker_scene
./run_smolvla_infer.sh --device cpu --steps 10 --apply --trace --sim-steps-per-action 20
```

推理结果和 trace 会写到 `smolvla_trace/`。

### 3. 回放 trace

```bash
cd mujoco-3.10.0/model/worker_scene
./run_replay_smolvla_trace.sh --video-out smolvla_trace/replay.mp4
```

这个回放脚本会直接读取 `smolvla_trace/trace.json` 里的真实动作字段，兼容当前的 tactile 22 维 trace；如果 trace 里记录的是 Windows 路径，它也会自动找本地对应 XML。

### 4. 重新生成 SmolVLA 场景

```bash
cd mujoco-3.10.0/model/worker_scene
openpi/.venv/bin/python build_smolvla_scene.py
```

生成后建议再跑一次场景检查命令。

### 5. 跑 2D SLAM

```bash
cd mujoco-3.10.0/model/worker_scene
source /opt/ros/foxy/setup.bash
./slam/run_slam.sh --fresh --view --rviz
```

### 6. 跑 3D SLAM

```bash
cd mujoco-3.10.0/model.test/worker_scene
source /opt/ros/foxy/setup.bash
python3.8 gen_3d_xml.py
./slam/run_slam_3d.sh --view --fresh
```

## 运行环境提醒

- `openpi/.venv/bin/python` 和系统 `python3` 不是一回事
- `models/smolvla_base/` 才是 policy 权重，不是 XML
- `download_smolvlm.sh` 只管 VLM backbone
- `scene.xml` 是手工调整过的，别随手覆盖
- 当前 SmolVLA 场景没有 gripper，别把“抓取”理解得太满

## 已验证情况

- SmolVLA 推理可跑通
- 本地 policy 权重可加载
- 本地 SmolVLM2 backbone 可加载
- trace 可正常生成

更多补充说明和容易误解的地方，放在 `README_NOTES.md`。
