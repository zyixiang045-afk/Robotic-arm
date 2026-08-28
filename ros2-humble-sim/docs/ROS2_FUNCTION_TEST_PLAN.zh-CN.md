# ROS 2 Humble 功能测试计划

本测试计划用于验证 Ubuntu 22.04 WSL 中的 ROS 2 Humble 仿真环境是否可用于机器人与机械臂开发。

## 1. 基础环境测试

目标：

- ROS 2 Humble 环境变量加载正常。
- `ros2`、`rosdep`、`colcon` 可用。
- demo 节点可以启动。

命令：

```bash
cd /mnt/c/Users/18707/Documents/ros2/ros2-humble-sim
bash scripts/test_ros2_basics.sh
```

通过标准：

- 脚本输出 `Basic ROS 2 tests passed.`

## 2. 节点通信测试

终端 1：

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

通过标准：

- listener 持续输出 `I heard: [Hello World: ...]`。

## 3. ROS topic 测试

在 talker 运行时执行：

```bash
source /opt/ros/humble/setup.bash
ros2 node list
ros2 topic list
ros2 topic echo /chatter --once
ros2 topic info /chatter
```

通过标准：

- 能看到 `/talker` 节点。
- 能看到 `/chatter` topic。
- `topic echo` 能收到一条消息。

## 4. turtlesim 图形化仿真测试

终端 1：

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"
```

通过标准：

- turtlesim 窗口可以打开。
- 小乌龟开始绕圈运动。

## 5. RViz2 图形测试

命令：

```bash
source /opt/ros/humble/setup.bash
rviz2
```

通过标准：

- RViz2 窗口可以打开。
- 没有 OpenGL 或 Qt 初始化失败。

## 6. Gazebo 测试

命令：

```bash
source /opt/ros/humble/setup.bash
gz sim shapes.sdf
```

通过标准：

- Gazebo 窗口可以打开。
- 示例场景可以加载。

## 7. 后续机械臂测试

基础环境通过后，再进入以下内容：

- URDF/Xacro 模型加载。
- `robot_state_publisher` 发布 TF。
- RViz2 显示机械臂模型。
- MoveIt 2 配置生成与规划测试。
- ros2_control 控制器加载。
- Gazebo 中执行关节轨迹。
