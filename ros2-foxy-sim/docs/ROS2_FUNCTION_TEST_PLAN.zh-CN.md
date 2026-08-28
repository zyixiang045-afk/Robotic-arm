# ROS 2 Foxy 功能测试计划

本测试计划用于验证 Ubuntu 20.04 + ROS 2 Foxy 环境是否可用于旧项目复现和基础仿真。

## 1. 基础环境测试

```bash
cd ~/Robotic-arm/ros2-foxy-sim
bash scripts/test_ros2_basics.sh
```

通过标准：

```text
Basic ROS 2 Foxy tests passed.
```

## 2. 节点通信测试

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

通过标准：

- listener 持续输出 `I heard: [Hello World: ...]`

## 3. Topic 测试

在 talker 运行时执行：

```bash
source /opt/ros/foxy/setup.bash
ros2 node list
ros2 topic list
ros2 topic echo /chatter
ros2 topic info /chatter
```

注意：Foxy 不支持 `ros2 topic echo /chatter --once`，收到消息后手动 `Ctrl+C`。

通过标准：

- 能看到 `/talker`。
- 能看到 `/chatter`。
- `topic echo` 能收到消息。

## 4. RViz2 测试

```bash
source /opt/ros/foxy/setup.bash
rviz2
```

通过标准：

- RViz2 窗口可以打开。
- 没有 OpenGL 或 Qt 初始化失败。

## 5. Gazebo classic 测试

```bash
source /opt/ros/foxy/setup.bash
gazebo
```

通过标准：

- Gazebo classic 窗口可以打开。
- 可加载默认空世界。

## 6. 后续机械臂测试

基础环境通过后，再进入：

- URDF/Xacro 模型加载。
- `robot_state_publisher` 发布 TF。
- RViz2 显示机械臂模型。
- ros2_control 控制器加载。
- Gazebo 中执行关节控制。
