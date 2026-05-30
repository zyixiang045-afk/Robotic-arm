# ROS 2 workspace source packages

Add project packages here.

Suggested package groups:

```text
robot_description/        # URDF/Xacro, meshes, RViz config
robot_bringup/            # launch files
robot_gazebo/             # Gazebo worlds and simulation launch files
robot_moveit_config/      # MoveIt 2 generated config
robot_control/            # ros2_control controller config
```

Keep simulation-specific launch files separate from reusable robot description and control configuration.
