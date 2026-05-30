# ROS 2 robot and arm simulation deployment plan

This workspace is prepared for a local-first workflow:

1. Use WSL2 + Ubuntu 24.04 for the current local ROS 2 development environment.
2. Keep Docker files only as a future server/team-image path.

The recommended ROS distribution is ROS 2 Jazzy on Ubuntu 24.04. Jazzy is a stable LTS choice for robot simulation, Gazebo, MoveIt 2, and ros2_control.

## Host prerequisites

On Windows, install these manually or allow Codex to run them when system approvals are available:

```powershell
wsl --install -d Ubuntu-24.04
```

Then install Docker Desktop and enable WSL integration for the Ubuntu 24.04 distribution.

After Ubuntu starts, update it:

```bash
sudo apt update
sudo apt upgrade -y
```

## Option A: WSL native development

Inside Ubuntu 24.04:

```bash
cd /mnt/c/Users/18707/Documents/ros2/ros2-jazzy-sim
bash scripts/setup_wsl_ros2_jazzy.sh
```

Open a new shell, then verify:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

In another shell:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

Graphics checks:

```bash
rviz2
gz sim
```

## Future option: Docker team image

This is not required for the current local setup. Keep it for the later server stage.

Build the image from Ubuntu/WSL when the team is ready:

```bash
bash docker/build.sh
```

Run an interactive container with GUI forwarding through WSLg:

```bash
bash docker/run_wsl_gui.sh
```

Inside the container:

```bash
ros2 --help
rviz2
gz sim
```

## Workspace layout

```text
src/
  robot_description/      # URDF/Xacro, meshes, RViz configs
  robot_bringup/          # launch files
  robot_gazebo/           # Gazebo worlds and simulation launch files
  robot_moveit_config/    # MoveIt 2 generated config
  robot_control/          # ros2_control config and controllers
docker/
  Dockerfile.ros2-jazzy
  compose.yaml
  build.sh
  run_wsl_gui.sh
scripts/
  setup_wsl_ros2_jazzy.sh
```

## Suggested execution order

1. Enable WSL2 and install Ubuntu 24.04.
2. Run `scripts/setup_wsl_ros2_jazzy.sh` inside Ubuntu.
3. Verify ROS 2 CLI, RViz2, and Gazebo.
4. Create or import the robot URDF/Xacro under `src/robot_description`.
5. Add ros2_control controller config under `src/robot_control`.
6. Generate MoveIt 2 config under `src/robot_moveit_config`.
7. After the local environment is stable, build and test the Docker image for the future server.
8. Pin team image tags and publish to the team's registry.

## Notes for later real hardware integration

Keep simulation packages separate from robot description and control packages. The same URDF/Xacro and MoveIt configuration should be reusable when swapping Gazebo plugins for real drivers.
