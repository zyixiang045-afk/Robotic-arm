# Windows WSL2 + Ubuntu 22.04 + ROS 2 Humble 本地仿真环境流程

本文记录从 Windows 安装 WSL、添加 Ubuntu 22.04、安装 ROS 2 Humble、用 VS Code 打开 WSL 工作空间，到运行 turtlesim 图形化仿真验证的完整流程。

目标环境：

```text
Windows 10/11
  -> WSL2
    -> Ubuntu 22.04
      -> ROS 2 Humble
      -> VS Code Remote - WSL
      -> turtlesim / rqt / rviz2
```

## 1. 在 Windows 中安装 WSL

打开 Windows PowerShell 或 Windows Terminal，执行：

```powershell
wsl --install
```

如果系统提示需要重启，请先重启 Windows。

查看 WSL 状态：

```powershell
wsl -l -v
```

如果还没有 Ubuntu 22.04，安装 Ubuntu 22.04：

```powershell
wsl --install -d Ubuntu-22.04
```

安装完成后进入 Ubuntu 22.04：

```powershell
wsl -d Ubuntu-22.04
```

第一次进入时会要求设置 Linux 用户名和密码。

如果希望以后默认进入 Ubuntu 22.04：

```powershell
wsl --set-default Ubuntu-22.04
```

之后只需要执行：

```powershell
wsl
```

## 2. 确认 Ubuntu 版本

进入 Ubuntu 后执行：

```bash
lsb_release -a
```

正确环境应显示：

```text
Release: 22.04
Codename: jammy
```

Ubuntu 22.04 对应推荐安装 ROS 2 Humble。请确认当前 WSL 发行版是 `jammy`，再继续执行 Humble 安装步骤。

## 3. 初始化 Ubuntu 基础环境

在 Ubuntu 22.04 终端中执行：

```bash
sudo apt update
sudo apt install locales software-properties-common curl gnupg lsb-release -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe -y
```

## 4. 添加 ROS 2 Humble 软件源

ROS 官方旧 key `F42ED6FBAB17C654` 已经过期，如果直接使用旧的 `ros.key`，可能出现：

```text
EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
E: The repository 'http://packages.ros.org/ros2/ubuntu jammy InRelease' is not signed.
```

因此建议使用新的 `ros2-apt-source` 包配置 ROS 2 源。

先清理可能已经添加过的旧源和旧 key：

```bash
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
```

下载并安装 ROS 2 apt source 包：

```bash
curl -L -o /tmp/ros2-apt-source.deb http://repo.ros2.org/ubuntu/main/pool/main/r/ros-apt-source/ros2-apt-source_1.2.0~jammy_all.deb
sudo dpkg -i /tmp/ros2-apt-source.deb
```

如果下载 GitHub 相关资源一直卡在 `0%`，优先使用上面的 `repo.ros2.org` 直链方式，避免依赖 GitHub API。

## 5. 安装 ROS 2 Humble

更新 apt 并安装 ROS 2 桌面版：

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop ros-dev-tools -y
```

写入 shell 环境：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

注意：`ros2` 命令没有 `--version` 参数，下面这个报错不代表安装失败：

```bash
ros2 --version
```

```text
ros2: error: unrecognized arguments: --version
```

可以用下面方式验证：

```bash
ros2 --help
```

如果可以看到 `run`、`topic`、`node`、`launch` 等子命令，说明 ROS 2 命令已经生效。

也可以检查 Humble 桌面包：

```bash
dpkg -l | grep ros-humble-desktop
```

## 6. 创建 ROS 2 工作空间

建议把 ROS 工作空间放在 Ubuntu 文件系统中，不要优先放在 `/mnt/c` 或 `/mnt/d` 下。这样编译更快，也更少遇到权限和符号链接问题。

创建工作空间：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

刚创建时，`~/ros2_ws` 目录下只有一个 `src` 文件夹是正常的：

```text
~/ros2_ws/
└── src/
```

运行 `colcon build` 后才会出现：

```text
~/ros2_ws/
├── build/
├── install/
├── log/
└── src/
```

## 7. 用 VS Code 打开 Ubuntu 22.04 工作空间

推荐方式是在 Windows 版 VS Code 中使用 Remote - WSL。

1. 打开 Windows 版 VS Code。
2. 安装扩展 `WSL` 或 `Remote Development`。
3. 点击左下角 `><`。
4. 选择 `Connect to WSL using Distro...`。
5. 选择 `Ubuntu-22.04`。
6. 打开文件夹：

```text
/home/<你的用户名>/ros2_ws
```

例如本机用户为 `nightflower` 时：

```text
/home/nightflower/ros2_ws
```

也可以在 Windows 的 VS Code 打开路径：

```text
\\wsl$\Ubuntu-22.04\home\nightflower\ros2_ws
```

如果在 WSL 里执行 `code .` 出现：

```text
/mnt/d/Microsoft VS Code/bin/code: ... Code.exe: Exec format error
```

可以先绕过 `code .`，直接从 Windows 版 VS Code 通过 Remote - WSL 打开 Ubuntu 22.04。这个问题通常和 WSL 调用 Windows 可执行文件的 interop 或 VS Code 安装路径有关，不影响 ROS 2 本身。

## 8. 创建一个测试 ROS 2 包

进入工作空间的 `src`：

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_first_pkg --dependencies rclpy std_msgs
```

回到工作空间编译：

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

如果 `colcon` 不存在，安装：

```bash
sudo apt install python3-colcon-common-extensions -y
```

## 9. 验证 ROS 2 通信

打开第一个 Ubuntu 22.04 终端：

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

打开第二个 Ubuntu 22.04 终端：

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

如果 listener 能持续收到 `Hello World` 消息，说明 ROS 2 节点通信正常。

## 10. 运行 turtlesim 图形化仿真

安装 turtlesim：

```bash
sudo apt install ros-humble-turtlesim -y
```

打开第一个 Ubuntu 22.04 终端，启动仿真窗口：

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

正常情况下会弹出一个小乌龟窗口。

打开第二个 Ubuntu 22.04 终端，发布速度指令：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"
```

小乌龟会开始绕圈运动。

## 11. 把 turtlesim 控制做成自己的 ROS 2 小项目

创建新包：

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python turtle_circle --dependencies rclpy geometry_msgs
```

新建文件：

```text
~/ros2_ws/src/turtle_circle/turtle_circle/circle_controller.py
```

写入：

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CircleController(Node):
    def __init__(self):
        super().__init__("circle_controller")
        self.publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

编辑：

```text
~/ros2_ws/src/turtle_circle/setup.py
```

找到 `entry_points`，改成：

```python
entry_points={
    "console_scripts": [
        "circle_controller = turtle_circle.circle_controller:main",
    ],
},
```

编译：

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

运行仿真窗口：

```bash
ros2 run turtlesim turtlesim_node
```

再开一个终端运行控制节点：

```bash
source ~/ros2_ws/install/setup.bash
ros2 run turtle_circle circle_controller
```

如果小乌龟持续画圆，说明：

- WSL2 正常。
- Ubuntu 22.04 正常。
- ROS 2 Humble 正常。
- VS Code Remote - WSL 可以编辑 WSL 工作空间。
- ROS 2 topic 发布和节点运行正常。
- WSL 图形化仿真窗口正常。

## 12. 常见问题

### GitHub 下载一直 0%

现象：

```text
% Total    % Received % Xferd
0     0    0     0
```

原因通常是当前网络访问 GitHub API 或 raw.githubusercontent.com 超时。优先使用本文中的 `repo.ros2.org` 直链方式安装 `ros2-apt-source`。

### packages.ros.org HTTPS 证书不匹配

现象：

```text
curl: (60) SSL: no alternative certificate subject name matches target host name 'packages.ros.org'
```

可以避免直接用 `https://packages.ros.org/ros.key`，改用 `ros2-apt-source` 包配置源。

### apt 报 EXPKEYSIG

现象：

```text
EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
```

说明旧 key 过期，清理旧源和旧 key 后重新安装 `ros2-apt-source`。

### 只有 src 文件夹是否正常

正常。`~/ros2_ws` 刚创建时只有 `src`。执行 `colcon build` 后才会生成 `build`、`install`、`log`。

### turtlesim 没有弹窗

如果使用 Windows 11，WSLg 通常会自动支持 Linux GUI。可以检查：

```bash
echo $DISPLAY
```

如果没有输出，说明当前 WSL 图形环境没有准备好。可以先从 Windows Terminal 重新进入 Ubuntu 22.04，再运行：

```bash
ros2 run turtlesim turtlesim_node
```

### VS Code 的 code . 报 Exec format error

可以不使用 `code .`，直接从 Windows 版 VS Code 左下角 Remote - WSL 连接 Ubuntu 22.04，并打开：

```text
/home/nightflower/ros2_ws
```
