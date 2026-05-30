# 本地 ROS 2 Jazzy 部署步骤

当前目标是先在这台 Windows 机器上搭建本地 ROS 2 仿真环境：

```text
Windows 10/11
  -> WSL2
    -> Ubuntu 24.04
      -> ROS 2 Jazzy
      -> Gazebo / RViz2 / MoveIt 2 / ros2_control
```

Docker 暂时不作为当前目标，只保留给团队后续服务器镜像化使用。

## 1. 安装 WSL2 和 Ubuntu 24.04

请在 Windows 的 PowerShell 或 Windows Terminal 中手动执行：

```powershell
wsl --install -d Ubuntu-24.04
```

如果提示需要启用虚拟化，请检查：

1. BIOS/UEFI 中 CPU virtualization 是否开启。
2. Windows 功能中是否启用：
   - Windows Subsystem for Linux
   - Virtual Machine Platform
3. 安装完成后按提示重启 Windows。

重启后第一次打开 Ubuntu 24.04，会要求设置 Linux 用户名和密码。

## 2. 进入当前项目目录

在 Ubuntu 24.04 终端中执行：

```bash
cd /mnt/c/Users/18707/Documents/ros2/ros2-jazzy-sim
```

## 3. 安装 ROS 2 Jazzy 本地开发环境

执行项目内脚本：

```bash
bash scripts/setup_wsl_ros2_jazzy.sh
```

该脚本会安装：

- ROS 2 Jazzy desktop
- ROS development tools
- colcon
- rosdep
- Gazebo ROS 集成
- MoveIt 2
- ros2_control
- ros2_controllers
- xacro
- joint_state_publisher_gui

## 4. 验证 ROS 2

打开一个 Ubuntu 终端：

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

再打开另一个 Ubuntu 终端：

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

如果 listener 能收到 talker 的消息，说明 ROS 2 基础通信正常。

## 5. 验证图形仿真

在 Ubuntu 终端中执行：

```bash
rviz2
```

再测试 Gazebo：

```bash
gz sim
```

如果窗口能正常打开，就可以继续做机器人和机械臂仿真。

## 6. 后续工作区规划

建议后续按下面结构创建 ROS 2 包：

```text
src/
  robot_description/      # URDF/Xacro、mesh、RViz 配置
  robot_bringup/          # launch 文件
  robot_gazebo/           # Gazebo world 和仿真插件
  robot_moveit_config/    # MoveIt 2 配置
  robot_control/          # ros2_control 配置
```

本地开发稳定之后，再把同一套依赖转入 `docker/Dockerfile.ros2-jazzy`，用于团队服务器镜像。
