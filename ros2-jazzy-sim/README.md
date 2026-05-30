# ROS 2 Jazzy local simulation environment

This directory is a local-first ROS 2 simulation environment template for robot and manipulator work.

Current target:

```text
Windows
  -> WSL2
    -> Ubuntu 24.04
      -> ROS 2 Jazzy
      -> RViz2 / Gazebo / MoveIt 2 / ros2_control
```

Docker support is included only as preparation for a later team server image.

## Quick start

Run inside Ubuntu 24.04 WSL:

```bash
cd /mnt/c/Users/18707/Documents/ros2/ros2-jazzy-sim
bash scripts/setup_wsl_ros2_jazzy.sh
```

Open a new Ubuntu terminal and verify:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

In another Ubuntu terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

Run the local verification script:

```bash
bash scripts/verify_local_ros2.sh
```

## Directory layout

```text
docker/                     # Future team image support
docs/                       # Local setup notes
scripts/                    # WSL setup and verification scripts
src/                        # ROS 2 workspace source packages
```

Recommended ROS 2 package layout:

```text
src/
  robot_description/        # URDF/Xacro, meshes, RViz configs
  robot_bringup/            # launch files
  robot_gazebo/             # Gazebo worlds and simulation launch files
  robot_moveit_config/      # MoveIt 2 generated config
  robot_control/            # ros2_control config and controllers
```

## Build workspace

After adding ROS 2 packages under `src/`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Documentation

- `docs/LOCAL_ROS2_SETUP.zh-CN.md`: Chinese local setup guide.
- `docs/DEPLOYMENT.md`: Deployment notes and future Docker path.

## Future Docker image

Build from Ubuntu/WSL when the team is ready:

```bash
bash docker/build.sh
```

Run with WSLg GUI forwarding:

```bash
bash docker/run_wsl_gui.sh
```
