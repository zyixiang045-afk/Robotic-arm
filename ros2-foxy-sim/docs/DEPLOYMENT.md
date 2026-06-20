# ROS 2 Foxy 部署计划

本目录用于 Ubuntu 20.04 + ROS 2 Foxy 环境复现。

当前用途：

1. 在 WSL2 + Ubuntu 20.04 中进行本地 Foxy 兼容测试。
2. 保留 Docker 文件，供后续服务器或团队镜像复现旧环境。

> Foxy 已经过官方维护期。除非项目明确依赖 Foxy，否则新开发应优先使用 Humble 或更新版本。

## 本地开发

Ubuntu 20.04：

```bash
cd /mnt/c/Users/18707/Documents/ros2/ros2-foxy-sim
bash scripts/setup_wsl_ros2_foxy.sh
```

验证：

```bash
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker
```

另一个终端：

```bash
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_py listener
```

## Docker 镜像

构建：

```bash
bash docker/build.sh
```

运行：

```bash
bash docker/run_wsl_gui.sh
```

## 工作区结构

```text
src/
  robot_description/      # URDF/Xacro、mesh、RViz 配置
  robot_bringup/          # launch 文件
  robot_gazebo/           # Gazebo world 和仿真插件
  robot_moveit_config/    # MoveIt 2 配置
  robot_control/          # ros2_control 配置
docker/
  Dockerfile.ros2-foxy
  compose.yaml
  build.sh
  run_wsl_gui.sh
scripts/
  setup_wsl_ros2_foxy.sh
  test_ros2_basics.sh
  verify_local_ros2.sh
```
