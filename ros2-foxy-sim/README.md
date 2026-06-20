# ROS 2 Foxy 本地仿真环境

本目录用于复现 Ubuntu 20.04 + ROS 2 Foxy 的旧版 ROS 2 开发与仿真环境。

目标环境：

```text
Windows 10/11
  -> WSL2
    -> Ubuntu 20.04
      -> ROS 2 Foxy
      -> VS Code Remote - WSL
      -> turtlesim / RViz2 / Gazebo / ros2_control
```

## 快速开始

在 Windows PowerShell 或 Windows Terminal 中安装并进入 Ubuntu 20.04：

```powershell
wsl --install -d Ubuntu-20.04
wsl -d Ubuntu-20.04
```

如果在线列表中没有 `Ubuntu-20.04`，可从 Microsoft Store 安装 `Ubuntu 20.04 LTS`，或手动导入 Ubuntu 20.04 rootfs。

在 Ubuntu 20.04 中执行：

```bash
cd /mnt/c/Users/18707/Documents/ros2/ros2-foxy-sim
bash scripts/setup_wsl_ros2_foxy.sh
```

打开新的 Ubuntu 20.04 终端，验证 ROS 2 通信：

```bash
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker
```

再打开另一个终端：

```bash
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_py listener
```

如果 listener 能持续收到 `Hello World` 消息，说明 ROS 2 基础通信正常。

也可以运行基础测试脚本：

```bash
bash scripts/test_ros2_basics.sh
```

## VS Code 打开方式

推荐在 Windows 版 VS Code 中安装 `WSL` 或 `Remote Development` 扩展，然后：

1. 点击左下角远程连接按钮。
2. 选择 `Connect to WSL using Distro...`。
3. 选择 `Ubuntu-20.04`。
4. 打开 `/home/<你的用户名>/ros2_ws`，或打开本仓库目录。

## turtlesim 图形验证

启动 turtlesim：

```bash
source /opt/ros/foxy/setup.bash
ros2 run turtlesim turtlesim_node
```

另开终端发布速度指令：

```bash
source /opt/ros/foxy/setup.bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"
```

如果小乌龟开始画圆，说明 WSL 图形窗口、ROS 2 topic 和仿真节点都正常。

## 目录结构

```text
docker/                     # 后续团队 Docker 镜像支持
docs/                       # 本地部署、测试和故障处理文档
scripts/                    # WSL 安装与验证脚本
src/                        # ROS 2 工作空间源码包
```

推荐 ROS 2 包结构：

```text
src/
  robot_description/        # URDF/Xacro、mesh、RViz 配置
  robot_bringup/            # launch 文件
  robot_gazebo/             # Gazebo world 与仿真 launch 文件
  robot_moveit_config/      # MoveIt 2 配置
  robot_control/            # ros2_control 控制器配置
```

## 工作空间构建

在 `src/` 中加入 ROS 2 包后执行：

```bash
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 文档

- `docs/WSL_UBUNTU20_ROS2_FOXY_SETUP.zh-CN.md`：Ubuntu 20.04 + ROS 2 Foxy 完整部署流程。
- `docs/LOCAL_ROS2_SETUP.zh-CN.md`：本地 Foxy 安装与验证说明。
- `docs/ROS2_FUNCTION_TEST_PLAN.zh-CN.md`：基础功能测试计划。
- `docs/DEPLOYMENT.md`：部署说明与后续 Docker 镜像路径。

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
