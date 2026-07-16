# WSL2 + Ubuntu 20.04 + ROS 2 Foxy 部署与验证指南

目标环境：

```text
Windows 10/11
  -> WSL2
    -> Ubuntu 20.04
      -> ROS 2 Foxy
      -> VS Code Remote - WSL
      -> RViz2 / Gazebo classic / ros2_control
```

## 1. 安装 WSL2

在 Windows PowerShell 或 Windows Terminal 中执行：

```powershell
wsl --install --no-distribution
wsl --set-default-version 2
```

如果系统提示重启，请重启 Windows。

查看 WSL 状态：

```powershell
wsl --status
wsl -l -v
```

## 2. 安装 Ubuntu 20.04

尝试：

```powershell
wsl --install -d Ubuntu-20.04
```

启动 Ubuntu 20.04：

```powershell
wsl -d Ubuntu-20.04
```

第一次启动时会要求创建 Linux 用户名和密码。密码输入时不会显示字符，这是正常现象。

设置 Ubuntu 20.04 为默认发行版：

```powershell
wsl --set-default Ubuntu-20.04
```

验证：

```powershell
wsl -l -v
```

默认发行版前会显示 `*`。

**此后在终端中输入wsl就能直接跳转到此版本的ubuntu终端。**

## 3. 获取项目仓库

进入 Ubuntu 20.04 后，建议将仓库 clone 到 Linux home 目录，而不是放在某个固定的 Windows 用户路径下：

```bash
sudo apt update
sudo apt install -y git
git clone https://github.com/zyixiang045-afk/Robotic-arm.git ~/Robotic-arm
cd ~/Robotic-arm/ros2-foxy-sim
```

如果已经 clone 过仓库，后续直接进入：

```bash
cd ~/Robotic-arm/ros2-foxy-sim
git pull
```

## 4. 安装 ROS 2 Foxy

在 `~/Robotic-arm/ros2-foxy-sim` 中执行：

```bash
bash scripts/setup_wsl_ros2_foxy.sh
```

该脚本会安装：

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

安装完成后，打开一个新的 Ubuntu 终端，或手动执行：

```bash
source /opt/ros/foxy/setup.bash
```

检查：

```bash
echo $ROS_DISTRO
which ros2
```

期望输出包含：

```text
foxy
/opt/ros/foxy/bin/ros2
```

## 5. VS Code 打开 Ubuntu 20.04 项目

推荐使用 VS Code Remote - WSL。

方式一：在 Ubuntu 终端中：

```bash
cd ~/Robotic-arm/ros2-foxy-sim
code .
```

方式二：在 VS Code 中：

1. 点击左下角远程连接按钮。
2. 选择 `Connect to WSL using Distro...`。
3. 选择 `Ubuntu-20.04`。
4. 打开 `/home/<你的Linux用户名>/Robotic-arm/ros2-foxy-sim`。

此后在Windows系统中打开vscode就能直接远程操控ubuntu系统。

## 6. Linux系统中Codex与Claude配置

按照https://docs.right.codes/docs/rc_cli_config/wsl.html 的流程先安装 Node.js 和 npm，然后依照 通过Windows下的cc-switch导入 进行配置。

在ubuntu系统中输入codex出现对话框则证明配置成功，会出现很多warning但是不会影响使用。

# 验证测试

实际上如果你部署好了Linux端的codex，可以直接输入“帮我验证此系统中的ros2功能完整性”然后等待即可。

## 1. 基础通信测试

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

如果 listener 能持续收到 `Hello World`，说明 ROS 2 基础通信正常。

也可以运行项目测试脚本：

```bash
cd ~/Robotic-arm/ros2-foxy-sim
bash scripts/test_ros2_basics.sh
```

通过时应输出：

```text
Basic ROS 2 Foxy tests passed.
```

## 2. RViz2 与 Gazebo 测试

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

Foxy 使用 Gazebo classic，不使用 Humble/Jazzy 中常见的 `gz sim` 命令。

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

## 3. turtlesim 图形验证

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

# 常见问腿

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

如果代理端口不是 `7890`，请替换为实际端口。

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

如果 `rosdep update` 超时，通常是 raw.githubusercontent.com 访问问题，按第 10 节配置代理。

如果误用了：

```bash
rosdep version
```

会报 unsupported command。正确命令是：

```bash
rosdep --version
```

# 最小复现流程

PowerShell：

```powershell
wsl --install --no-distribution
wsl --set-default-version 2
wsl --install -d Ubuntu-20.04
wsl --set-default Ubuntu-20.04
wsl -d Ubuntu-20.04
```

Ubuntu：

```bash
sudo apt update
sudo apt install -y git
git clone https://github.com/zyixiang045-afk/Robotic-arm.git ~/Robotic-arm
cd ~/Robotic-arm/ros2-foxy-sim
bash scripts/setup_wsl_ros2_foxy.sh
bash scripts/test_ros2_basics.sh
```

如果 `test_ros2_basics.sh` 输出 `Basic ROS 2 Foxy tests passed.`，说明基础环境配置完成。
