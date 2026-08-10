# README 补充说明

这份文档专门记录主 `README.md` 里没有写透、但实际配置环境时很容易卡住的地方。

## 已确认但容易误解的点

1. 这个仓库不是单一环境。
   - `mujoco-3.10.0/` 负责 MuJoCo 场景和 SmolVLA 推理。
   - `mujoco-3.10.0/model.test/worker_scene/` 负责 ROS 2 Foxy / SLAM / Nav2 / RTAB-Map。
   - `openpi/.venv` 是 SmolVLA 推理脚本默认使用的 Python 3.11 环境。
   - 另外还有仓库根目录下的 `.venv`，不要把它和 `openpi/.venv` 混掉。

2. README 里的很多命令用了固定路径。
   - 示例里常见的是 `/home/ee304/jbgs/...`
   - 迁移到别的机器时，要把这些绝对路径替换成你自己的工作区路径。

3. `build_robot.py` 和 `scene_with_robot*.xml` 不是一回事。
   - `build_robot.py` 是生成/重建机器人装配的脚本。
   - `scene_with_robot.xml` / `scene_with_robot_py38.xml` 是生成后的场景文件。
   - 跑 ROS Foxy 的时候通常要用 `*_py38.xml`。

## 权重相关

### 1. SmolVLA policy 权重

- 目录：`models/smolvla_base/`
- 作用：SmolVLA 的 policy checkpoint
- 关键文件：`model.safetensors`

README 目前默认这个目录已经存在，但没有给出单独的下载脚本。
也就是说，这部分更像是“把已有权重放到本地目录”，而不是“README 一键拉取”。

### 2. SmolVLM2 backbone / tokenizer

- 目录：`models/SmolVLM2-500M-Video-Instruct/`
- 下载脚本：`mujoco-3.10.0/model/worker_scene/download_smolvlm.sh`

这个脚本只负责 SmolVLM2 backbone/tokenizer，不会下载 SmolVLA policy 权重。
脚本默认使用 Hugging Face 镜像，并把文件拉到本地 `models/SmolVLM2-500M-Video-Instruct/`。
权重在- `https://huggingface.co/datasets/KrisYoung/SmolVLA-DexHand-Tactile-Data`



## 环境相关

1. 如果只跑 SmolVLA 推理，不需要 ROS Foxy。
2. 如果要跑 SLAM / Nav2 / RTAB-Map，就要有 ROS 2 Foxy 和 Python 3.8 兼容环境。
3. README 里同时出现了 MuJoCo 3.10 和 ROS 3.8 兼容路径，这不是矛盾，而是因为这套仓库本来就分了两条线。

## 还建议补进 README 的地方

- 明确写出 `models/smolvla_base/` 里的权重来源。
- 明确写出 `download_smolvlm.sh` 只下载 backbone，不下载 policy。
- 把所有示例路径改成“工作区根目录相对路径”或变量形式。
- 说明 `openpi/.venv` 和根目录 `.venv` 的职责边界。
- 说明 ROS Foxy 只对 SLAM 线必需，SmolVLA 推理不依赖它。

## 额外缺失项记录

4. WSL 里直接跑 `run_smolvla_infer.sh` 还缺 Python 依赖。
   - 不能直接用 Windows 侧的 `.venv`。
   - 当前最明显缺的是 `torch`，没有它会在 `smolvla_infer.py` 里直接报 `ModuleNotFoundError: No module named 'torch'`。
   - 需要在 WSL 中单独建环境并安装依赖，至少包括 `torch`、`mujoco`、`imageio`、`numpy`。
## Environment Verification (2026-08-09)

- WSL is Ubuntu 20.04 with Python 3.8.10. LeRobot 0.6.1 requires Python >=3.12 and cannot be imported there.
- Windows Python 3.13 has lerobot 0.6.1 and CPU PyTorch installed. Real tactile inference completed successfully for 1 step and produced 22-dimensional actions plus a trace.
- Run inference from PowerShell:
  ```powershell
  $env:MUJOCO_GL="glfw"
  py -3.13 mujoco-3.10.0/model/worker_scene/smolvla_infer.py --model-dir models/smolvla_base --vlm-model-dir models/SmolVLM2-500M-Video-Instruct --device cpu --steps 10 --apply --trace --sim-steps-per-action 20
  ```
- `run_smolvla_infer.sh` is the Linux/WSL launcher. It cannot run SmolVLA in the current WSL Python 3.8 environment without installing Python 3.12+ and matching LeRobot dependencies.
## WSL launcher fix (2026-08-09)

- The error `TypeError: 'type' object is not subscriptable` came from running LeRobot 0.6.1 with WSL Python 3.8. LeRobot 0.6.1 requires Python 3.12 or newer.
- `run_smolvla_infer.sh` now detects WSL and automatically uses the configured Windows Python 3.13 at `/mnt/d/Program Files/Python313/python.exe`.
- The launcher also converts `/mnt/d/...` paths to Windows paths and uses the `glfw` MuJoCo backend for the Windows runtime.
- Verified command from WSL:
  ```bash
  ./run_smolvla_infer.sh --device cpu --steps 1 --trace --sim-steps-per-action 1
  ```
  It completed successfully with tactile input shape `(20,)`, state shape `(22,)`, and a 22-dimensional action output.

## Replay fix (2026-08-09)

- `run_replay_smolvla_trace.sh` now survives CRLF issues in WSL.
- `replay_smolvla_trace.py` now reads the real trace action keys instead of assuming a 6-joint layout.
- The replay loader can resolve the Windows `meta.xml` path in the tactile trace and fall back to the local tactile scene XML.
- The replay script now picks an available camera automatically from `trace["meta"]["camera_names"]`, so tactile traces use `image` instead of the old `camera2` default.
- MP4 export now uses PyAV/libx264 directly; `imageio`'s ffmpeg plugin was not available in the local Windows Python runtime.
- Verified replay command from Windows Python 3.13:
  ```powershell
  py -3.13 mujoco-3.10.0/model/worker_scene/replay_smolvla_trace.py --video-out mujoco-3.10.0/model/worker_scene/smolvla_trace/replay.mp4
  ```
  It completed successfully and wrote `smolvla_trace/replay.mp4`.
