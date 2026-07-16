# 本地 ROS 2 Foxy 部署步骤

当前目标是在 Windows + WSL2 中搭建 Ubuntu 20.04 + ROS 2 Foxy 环境：

```text
Windows 10/11
  -> WSL2
    -> Ubuntu 20.04
      -> ROS 2 Foxy
      -> Gazebo classic / RViz2 / ros2_control
```

> Foxy 已 EOL，仅建议用于旧项目复现或兼容性测试。

## 1. 安装 Ubuntu 20.04

PowerShell：

```powershell
wsl --install -d Ubuntu-20.04
wsl --set-default Ubuntu-20.04
```

如果在线列表中没有 Ubuntu 20.04，请从 Microsoft Store 安装 `Ubuntu 20.04 LTS`。

## 2. 进入项目目录

Ubuntu 20.04 终端：

```bash
git clone https://github.com/zyixiang045-afk/Robotic-arm.git ~/Robotic-arm
cd ~/Robotic-arm/ros2-foxy-sim
```

如果已经 clone 过仓库，只需要执行：

```bash
cd ~/Robotic-arm/ros2-foxy-sim
```

## 3. 安装 ROS 2 Foxy

```bash
bash scripts/setup_wsl_ros2_foxy.sh
```

## 4. 验证 ROS 2

终端 1：

```bash
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker
```

终端 2：

```bash
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_py listener
```

如果 listener 能收到 talker 的消息，说明 ROS 2 基础通信正常。

## 5. 验证图形工具

RViz2：

```bash
rviz2
```

Gazebo classic：

```bash
gazebo
```

## 6. 后续工作区规划

```text
src/
  robot_description/      # URDF/Xacro、mesh、RViz 配置
  robot_bringup/          # launch 文件
  robot_gazebo/           # Gazebo world 和仿真插件
  robot_moveit_config/    # MoveIt 2 配置
  robot_control/          # ros2_control 配置
```
