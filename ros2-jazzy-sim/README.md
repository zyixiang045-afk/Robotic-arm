# ROS 2 Jazzy 本地仿真环境

本目录是一个面向机器人与机械臂仿真的 ROS 2 本地开发环境模板。当前阶段优先支持本机开发与调试，后续可迁移为团队服务器 Docker 镜像。

当前目标环境：

```text
Windows
  -> WSL2
    -> Ubuntu 24.04
      -> ROS 2 Jazzy
      -> RViz2 / Gazebo / MoveIt 2 / ros2_control
```

Docker 相关文件已预留，但当前不作为主要运行方式，只用于后续团队服务器部署准备。

## 快速开始

在 Ubuntu 24.04 WSL 终端中执行：

```bash
cd /mnt/c/Users/18707/Documents/ros2/ros2-jazzy-sim
bash scripts/setup_wsl_ros2_jazzy.sh
```

打开一个新的 Ubuntu 终端，验证 talker：

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

再打开另一个 Ubuntu 终端，验证 listener：

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

如果 listener 能持续收到 `Hello World` 消息，说明 ROS 2 节点通信正常。

也可以运行本地环境检查脚本：

```bash
bash scripts/verify_local_ros2.sh
```

## 目录结构

```text
docker/                     # 未来团队 Docker 镜像支持
docs/                       # 本地部署说明
scripts/                    # WSL 安装与验证脚本
src/                        # ROS 2 工作区源码包
```

推荐的 ROS 2 包结构：

```text
src/
  robot_description/        # URDF/Xacro、mesh、RViz 配置
  robot_bringup/            # launch 文件
  robot_gazebo/             # Gazebo world 与仿真 launch 文件
  robot_moveit_config/      # MoveIt 2 生成的配置
  robot_control/            # ros2_control 控制器配置
```

## 构建工作区

在 `src/` 下添加 ROS 2 包之后，执行：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 文档

- `docs/LOCAL_ROS2_SETUP.zh-CN.md`：本地 ROS 2 Jazzy 安装与验证说明。
- `docs/DEPLOYMENT.md`：部署说明与未来 Docker 镜像路径。

## 未来 Docker 镜像

团队准备迁移到服务器或统一镜像时，可在 Ubuntu/WSL 中构建：

```bash
bash docker/build.sh
```

通过 WSLg 图形转发运行：

```bash
bash docker/run_wsl_gui.sh
```

## 当前状态

本地 ROS 2 Jazzy 已完成基础验证：

- `demo_nodes_cpp talker` 可以正常发布消息。
- `demo_nodes_py listener` 可以正常接收消息。

下一步可以在 `src/` 中逐步加入机械臂描述、Gazebo 仿真场景、MoveIt 2 配置和 ros2_control 控制器配置。
