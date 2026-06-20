# ROS 2 工作空间源码包

在这里添加项目 ROS 2 包。

推荐包结构：

```text
robot_description/        # URDF/Xacro、mesh、RViz 配置
robot_bringup/            # launch 文件
robot_gazebo/             # Gazebo world 与仿真 launch 文件
robot_moveit_config/      # MoveIt 2 配置
robot_control/            # ros2_control 控制器配置
```

建议将仿真专用 launch 文件与可复用的机器人描述、控制器配置分开，方便后续迁移到真实机械臂或 Docker 环境。
