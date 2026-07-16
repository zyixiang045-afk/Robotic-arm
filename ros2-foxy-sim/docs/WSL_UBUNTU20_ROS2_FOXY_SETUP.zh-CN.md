# WSL2 + Ubuntu 20.04 + ROS 2 Foxy 部署与问题记录

本文档记录本项目的 ROS 2 Foxy 复现环境。安装过程中最好科学上网。

目标环境：

```text
Windows 10/11
  -> WSL2
    -> Ubuntu 20.04
      -> ROS 2 Foxy
      -> VS Code Remote - WSL
      -> RViz2 / Gazebo / ros2_control
```

## 1. 安装 WSL2

在 Windows PowerShell 或 Windows Terminal 中执行：

```powershell
wsl --install
wsl --set-default-version 2
```

查看状态：

```powershell
wsl --status
wsl -l -v
```

如果提示需要启用虚拟化，需要检查 BIOS/UEFI 中 CPU virtualization 是否开启，并确认 Windows 功能中启用了：

- Windows Subsystem for Linux
- Virtual Machine Platform

## 2. 安装 Ubuntu 20.04

优先尝试：

```powershell
wsl --install -d Ubuntu-20.04
```

如果 `wsl --list --online` 中没有 `Ubuntu-20.04`，可使用 Microsoft Store 安装 `Ubuntu 20.04 LTS`。

安装后启动：

```powershell
wsl -d Ubuntu-20.04
```

设置为默认发行版：

```powershell
wsl --set-default Ubuntu-20.04
```

验证：

```powershell
wsl -l -v
```

默认发行版前会显示 `*`。

## 3. 安装 ROS 2 Foxy

进入 Ubuntu 20.04 后执行：

```bash
cd /mnt/c/Users/<你的用户代号>/Documents/ros2/ros2-foxy-sim
bash scripts/setup_wsl_ros2_foxy.sh
```

脚本会安装：

- ROS 2 Foxy desktop
- colcon
- rosdep
- Gazebo ROS packages
- ros2_control
- ros2_controllers
- turtlesim
- xacro
- joint_state_publisher_gui
- MoveIt 2，如果 apt 源中可用

Foxy 已 EOL，因此脚本使用：

```bash
rosdep update --include-eol-distros
```

## 4. 基础通信测试

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

如果 listener 能收到 `Hello World`，说明基础通信正常。

也可以运行：

```bash
cd /mnt/c/Users/18707/Documents/ros2/ros2-foxy-sim
bash scripts/test_ros2_basics.sh
```

通过时应输出：

```text
Basic ROS 2 Foxy tests passed.
```

## 5. Topic 测试

保持 talker 运行，在另一个终端执行：

```bash
source /opt/ros/foxy/setup.bash
ros2 node list
ros2 topic list
ros2 topic echo /chatter
ros2 topic info /chatter
```

注意：Foxy 中 `ros2 topic echo` 不支持 `--once` 参数。收到一条消息后手动按 `Ctrl+C` 停止。

如果提示：

```text
Unknown topic '/chatter'
```

通常是 talker 没有运行，或不同终端的 `ROS_DOMAIN_ID` 不一致。

检查：

```bash
echo $ROS_DOMAIN_ID
```

两个终端应保持一致，默认可以为空。

## 6. RViz2 与 Gazebo 测试

测试 RViz2：

```bash
source /opt/ros/foxy/setup.bash
rviz2
```

如果看到：

```text
Stereo is NOT SUPPORTED
OpenGl version: ...
```

这通常不是错误。`Stereo is NOT SUPPORTED` 只表示当前图形环境不支持立体渲染。

测试 Gazebo classic：

```bash
source /opt/ros/foxy/setup.bash
gazebo
```

Foxy 使用 Gazebo classic，和 Humble/Jazzy 中常见的 `gz sim` 命令不同。

如果窗口无法显示，可检查 WSLg：

```bash
echo $DISPLAY
echo $WAYLAND_DISPLAY
echo $XDG_RUNTIME_DIR
ls /mnt/wslg
```

也可以安装图形测试工具：

```bash
sudo apt install -y x11-apps mesa-utils
xeyes
glxgears
```

## 7. turtlesim 图形验证

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

## 8. VS Code 打开 Ubuntu 20.04

推荐使用 VS Code Remote - WSL。

在 Ubuntu 终端中：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
code .
```

或在 VS Code 中：

1. 点击左下角远程连接按钮。
2. 选择 `Connect to WSL using Distro...`。
3. 选择 `Ubuntu-20.04`。
4. 打开 `/home/<你的用户名>/` 或本仓库目录。

## 9. Codex 与 Node.js

请按照https://docs.right.codes/docs/rc_cli_config/wsl.html 中的步骤安装Node.js 和 npm，并选择通过Windows下的cc-switch导入。
在终端中输入codex，如果出现对话框则安装成功。

此时可能会出现很多warning，但实际上不影响使用。

# 问题记录

## 1. WSL 代理问题

WSL 启动时可能出现：

```text
wsl: 检测到 localhost 代理配置，但未镜像到 WSL。NAT 模式下的 WSL 不支持 localhost 代理。
```

这不是致命错误。如果 `apt update` 和 `rosdep update` 能正常完成，可以忽略。

如果访问 GitHub 或 raw.githubusercontent.com 超时，假设 Windows 代理端口是 `7890`，可在 Ubuntu 中执行：

```bash
export WINDOWS_HOST=$(ip route | awk '/default/ {print $3}')
export http_proxy=http://$WINDOWS_HOST:7890
export https_proxy=http://$WINDOWS_HOST:7890
```

测试：

```bash
curl -I https://github.com
curl -I https://raw.githubusercontent.com
```

## 2. rosdep 常见问题

如果出现：

```text
ERROR: no sources directory exists on the system meaning rosdep has not yet been initialized.
```

执行：

```bash
sudo rosdep init
rosdep update --include-eol-distros
```

如果 `rosdep update` 超时，通常是 raw.githubusercontent.com 访问问题，按第 7 节配置代理。

如果误用了：

```bash
rosdep version
```

会报 unsupported command。正确命令是：

```bash
rosdep --version
```
