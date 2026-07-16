# WSL2 + Ubuntu 20.04 + ROS 2 Foxy 部署与验证指南

本文档面向团队成员，用于在各自电脑上复现 Ubuntu 20.04 + ROS 2 Foxy 环境。所有路径均使用通用写法，不依赖某个队员的 Windows 用户目录。

目标环境：

```text
Windows 10/11
  -> WSL2
    -> Ubuntu 20.04
      -> ROS 2 Foxy
      -> VS Code Remote - WSL
      -> RViz2 / Gazebo classic / ros2_control
```

> 注意：ROS 2 Foxy 已经过官方维护期，主要用于旧项目复现或兼容性测试。新项目建议优先使用 ROS 2 Humble 或更新版本。

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

如果提示需要启用虚拟化，请检查：

- BIOS/UEFI 中 CPU virtualization 是否开启。
- Windows 功能中是否启用 `Windows Subsystem for Linux`。
- Windows 功能中是否启用 `Virtual Machine Platform`。

## 2. 安装 Ubuntu 20.04

优先尝试：

```powershell
wsl --install -d Ubuntu-20.04
```

如果在线列表中没有 `Ubuntu-20.04`，请从 Microsoft Store 安装：

```text
Ubuntu 20.04 LTS
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

不建议队员照抄类似 `cd /mnt/c/Users/<某个Windows用户名>/Documents/...` 这样的个人路径。这类路径只适用于特定电脑和特定 Windows 用户名。

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

## 5. 基础通信测试

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

## 6. Topic 测试

保持 talker 运行，在另一个终端执行：

```bash
source /opt/ros/foxy/setup.bash
ros2 node list
ros2 topic list
ros2 topic echo /chatter
ros2 topic info /chatter
```

注意：Foxy 中 `ros2 topic echo` 不支持 `--once` 参数。收到消息后手动按 `Ctrl+C` 停止。

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

## 7. RViz2 与 Gazebo 测试

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

## 8. turtlesim 图形验证

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

## 9. VS Code 打开 Ubuntu 20.04 项目

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

## 10. WSL 代理问题

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

## 11. rosdep 常见问题

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

## 12. Codex 与 Node.js

如果需要在 Ubuntu 20.04 中运行 Codex，建议先安装 `bubblewrap`：

```bash
sudo apt update
sudo apt install -y bubblewrap
```

Node.js 推荐使用 nvm。若安装 nvm 卡在 `Cloning into ~/.nvm`，可使用 script 模式：

```bash
rm -rf ~/.nvm
curl -fsSL https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.5/install.sh -o /tmp/install_nvm.sh
METHOD=script PROFILE="$HOME/.bashrc" bash /tmp/install_nvm.sh
source ~/.bashrc
```

安装 Node.js 22：

```bash
export NVM_NODEJS_ORG_MIRROR=https://npmmirror.com/mirrors/node
nvm install 22
nvm alias default 22
nvm use 22
```

## 13. 最小复现流程

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
