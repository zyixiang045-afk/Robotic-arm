# 本地 ROS 2 Humble 部署步骤

当前团队本地开发环境统一为：

```text
Windows 10/11
  -> WSL2
    -> Ubuntu 22.04
      -> ROS 2 Humble
      -> VS Code Remote - WSL
```

Docker 暂时不作为当前主要运行方式，只保留给后续团队服务器镜像化使用。

## 1. 安装 WSL2 和 Ubuntu 22.04

在 Windows PowerShell 或 Windows Terminal 中执行：

```powershell
wsl --install -d Ubuntu-22.04
```

如果提示需要启用虚拟化，请检查：

1. BIOS/UEFI 中 CPU virtualization 是否开启。
2. Windows 功能中是否启用：
   - Windows Subsystem for Linux
   - Virtual Machine Platform
3. 安装完成后按提示重启 Windows。

第一次启动 Ubuntu 22.04 时，需要设置 Linux 用户名和密码。

确认版本：

```bash
lsb_release -a
```

应显示：

```text
Release: 22.04
Codename: jammy
```

## 2. 进入当前项目目录

在 Ubuntu 22.04 终端中执行：

```bash
cd /mnt/c/Users/18707/Documents/ros2/ros2-humble-sim
```

## 3. 安装 ROS 2 Humble 本地开发环境

执行项目内脚本：

```bash
bash scripts/setup_wsl_ros2_humble.sh
```

该脚本会安装：

- ROS 2 Humble desktop
- ROS development tools
- colcon
- rosdep
- turtlesim
- Gazebo ROS 集成
- MoveIt 2
- ros2_control
- ros2_controllers
- xacro
- joint_state_publisher_gui

脚本使用 `ros2-apt-source` 配置 ROS 2 apt 源，避免旧 `ros.key` 过期和 GitHub raw 下载超时问题。

## 4. 用 VS Code 打开 Ubuntu 22.04

在 Windows 版 VS Code 中安装 `WSL` 或 `Remote Development` 扩展。

推荐打开方式：

1. 点击 VS Code 左下角 `><`。
2. 选择 `Connect to WSL using Distro...`。
3. 选择 `Ubuntu-22.04`。
4. 打开目录 `/home/<你的用户名>/ros2_ws` 或本仓库目录。

如果 `code .` 报 `Exec format error`，直接使用 Remote - WSL 入口打开即可。

## 5. 验证 ROS 2

打开一个 Ubuntu 22.04 终端：

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

再打开另一个 Ubuntu 22.04 终端：

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

如果 listener 能收到 talker 的消息，说明 ROS 2 基础通信正常。

也可以运行非图形测试脚本：

```bash
bash scripts/test_ros2_basics.sh
```

## 6. 验证图形化仿真

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

如果小乌龟开始画圆，说明 WSL 图形化仿真正常。

可继续测试 RViz2：

```bash
rviz2
```

Gazebo 测试：

```bash
gz sim shapes.sdf
```

## 7. 后续工作区规划

建议后续按下面结构创建 ROS 2 包：

```text
src/
  robot_description/      # URDF/Xacro, mesh, RViz 配置
  robot_bringup/          # launch 文件
  robot_gazebo/           # Gazebo world 和仿真插件
  robot_moveit_config/    # MoveIt 2 配置
  robot_control/          # ros2_control 配置
```

本地开发稳定之后，再把同一套依赖转入 `docker/Dockerfile.ros2-humble`，用于团队服务器镜像。
