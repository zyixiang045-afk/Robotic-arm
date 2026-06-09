# ROS 2 Humble 本地仿真环境

本目录是团队统一使用的 ROS 2 本地开发与仿真环境模板。当前路线固定为：

```text
Windows 10/11
  -> WSL2
    -> Ubuntu 22.04
      -> ROS 2 Humble
      -> VS Code Remote - WSL
      -> turtlesim / RViz2 / Gazebo / MoveIt 2 / ros2_control
```

Ubuntu 24.04 + ROS 2 Jazzy 路线已不再作为当前仓库目标。

## 快速开始

在 Windows PowerShell 或 Windows Terminal 中安装并进入 Ubuntu 22.04：

```powershell
wsl --install -d Ubuntu-22.04
wsl -d Ubuntu-22.04
```

在 Ubuntu 22.04 中进入本目录并执行环境安装脚本：

```bash
cd /mnt/c/Users/18707/Documents/ros2/ros2-humble-sim
bash scripts/setup_wsl_ros2_humble.sh
```

打开新的 Ubuntu 22.04 终端，验证 ROS 2 通信：

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

再打开另一个 Ubuntu 22.04 终端：

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

如果 listener 能持续收到 `Hello World` 消息，说明 ROS 2 基础通信正常。

也可以运行基础测试脚本：

```bash
bash scripts/test_ros2_basics.sh
```

## VS Code 打开方式

推荐在 Windows 版 VS Code 中安装 `WSL` 或 `Remote Development` 扩展，然后：

1. 点击左下角 `><`。
2. 选择 `Connect to WSL using Distro...`。
3. 选择 `Ubuntu-22.04`。
4. 打开目录 `/home/<你的用户名>/ros2_ws`，或打开本仓库目录。

如果在 WSL 中执行 `code .` 出现 `Exec format error`，直接使用 VS Code 的 Remote - WSL 入口即可，不影响 ROS 2 环境。

## turtlesim 图形化验证

启动 turtlesim：

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

另开终端发布速度指令：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"
```

如果小乌龟开始画圆，说明 WSL 图形化窗口、ROS 2 topic 和仿真节点都工作正常。

## 目录结构

```text
docker/                     # 后续团队 Docker 镜像支持
docs/                       # 本地部署、测试和完整配置说明
scripts/                    # WSL 安装与验证脚本
src/                        # ROS 2 工作空间源码包
```

推荐的 ROS 2 包结构：

```text
src/
  robot_description/        # URDF/Xacro, meshes, RViz 配置
  robot_bringup/            # launch 文件
  robot_gazebo/             # Gazebo world 与仿真 launch 文件
  robot_moveit_config/      # MoveIt 2 配置
  robot_control/            # ros2_control 控制器配置
```

## 工作空间构建

在 `src/` 中加入 ROS 2 包后执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 文档

- `docs/WSL_UBUNTU22_ROS2_HUMBLE_SETUP.zh-CN.md`：从安装 WSL 到 VS Code 打开 Ubuntu 22.04 并完成 turtlesim 仿真的完整流程。
- `docs/LOCAL_ROS2_SETUP.zh-CN.md`：本地 ROS 2 Humble 安装与验证说明。
- `docs/ROS2_FUNCTION_TEST_PLAN.zh-CN.md`：基础功能测试计划。
- `docs/DEPLOYMENT.md`：部署说明与后续 Docker 镜像路径。

## 在liunx系统中配置codex与claude

按照教程https://docs.right.codes/docs/rc_cli_config/wsl.html即可
推荐使用ccswitch导入配置文件

### Codex 在 WSL 中的常见提示处理

#### 1. bubblewrap 未安装

如果启动 Codex 时出现：

```text
Codex could not find bubblewrap on PATH.
Install bubblewrap with your OS package manager.
Codex will use the bundled bubblewrap in the meantime.
```

原因是 Ubuntu/WSL 中没有安装系统版 `bubblewrap`。`bubblewrap` 用于 Codex 的 Linux sandbox 隔离。

解决：

```bash
sudo apt update
sudo apt install -y bubblewrap
```

验证：

```bash
command -v bwrap
bwrap --version
```

安装完成后重启 VS Code WSL 窗口或 Codex。

#### 2. node_repl MCP 启动失败

如果启动 Codex 时出现：

```text
MCP client for `node_repl` failed to start:
MCP startup failed: No such file or directory (os error 2)

MCP startup incomplete (failed: node_repl)
```

通常是因为 `~/.codex/config.toml` 中的 `node_repl` 配置指向了 Windows 路径，例如：

```toml
[mcp_servers.node_repl]
command = 'C:\Users\...\node_repl.exe'
```

但当前 Codex 运行在 WSL Ubuntu 中，无法直接识别这个 Windows 路径，因此启动失败。

如果暂时不需要 `node_repl`，最简单稳定的处理方式是直接禁用它。

先备份配置：

```bash
cp ~/.codex/config.toml ~/.codex/config.toml.bak
```

打开配置：

```bash
code ~/.codex/config.toml
```

删除或注释掉以下相关段落：

```toml
[mcp_servers.node_repl]
...

[mcp_servers.node_repl.env]
...
```

保存后重启 VS Code WSL 窗口或 Codex。

说明：禁用 `node_repl` 只会少一个 JavaScript REPL MCP 工具，不影响 Codex 的主要功能，例如阅读仓库、修改文件、运行命令和协助开发。

## Docker 镜像

Docker 不是当前本地开发的主要方式，只保留给后续团队服务器或统一镜像使用。

构建镜像：

```bash
bash docker/build.sh
```

通过 WSLg 图形转发运行：

```bash
bash docker/run_wsl_gui.sh
```
