# Ubuntu 20.04 安装 ROS1 Noetic 指南

## 1. 进入 Ubuntu 20.04

在 Windows PowerShell 中启动 Ubuntu 20.04：

```powershell
wsl -d Ubuntu-20.04
```

确认系统版本：

```bash
lsb_release -a
```

应看到：

```text
Ubuntu 20.04
focal
```

## 2. 进入 root shell

先进入 root：

```bash
sudo -i
```

进入后提示符通常会变成：

```text
root@主机名:~#
```

后续命令均在 root shell 中执行。

## 3. 添加 ROS1 清华软件源

新建 `/etc/apt/sources.list.d/ros-latest.list`：

```bash
echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main" > /etc/apt/sources.list.d/ros-latest.list
```

检查内容：

```bash
cat /etc/apt/sources.list.d/ros-latest.list
```

应输出：

```text
deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main
```

## 4. 导入 ROS GPG Key

按页面命令导入 ROS 的 GPG key：

```bash
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

如果出现 `apt-key is deprecated`，可以先忽略；这是旧版 ROS1 安装流程中的常见提示。

## 5. 更新软件索引

```bash
apt update
```

如果能够正常读取 `mirrors.tuna.tsinghua.edu.cn/ros/ubuntu`，说明软件源配置成功。

## 6. 安装 ROS1 Noetic

安装完整桌面版：

```bash
apt install -y ros-noetic-desktop-full
```

如果只需要基础版本，可改用：

```bash
apt install -y ros-noetic-desktop
```

一般建议使用 `ros-noetic-desktop-full`，包含 RViz、rqt、常用仿真和可视化组件。

## 7. 初始化 rosdep

安装 rosdep：

```bash
apt install -y python3-rosdep
```

初始化：

```bash
rosdep init
```

如果提示已经存在：

```text
ERROR: default sources list file already exists
```

说明已经初始化过，可以继续下一步。

更新 rosdep：

```bash
rosdep update
```

## 8. 配置 ROS1 环境变量

仍在 root shell 中时，建议不要把 ROS 环境写入 root 的 `.bashrc`，而是写入普通用户的 `.bashrc`。

假设普通用户名是 `nightflower`：

```bash
echo "source /opt/ros/noetic/setup.bash" >> /home/nightflower/.bashrc
```

如果你的用户名不同，先退出 root，查看用户名：

```bash
exit
whoami
```

然后重新进入 root，并替换路径中的用户名。

也可以在普通用户终端中执行：

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 9. 退出 root 并验证

退出 root：

```bash
exit
```

重新加载环境：

```bash
source ~/.bashrc
```

检查 ROS 版本：

```bash
echo $ROS_DISTRO
```

应输出：

```text
noetic
```

启动 ROS master：

```bash
roscore
```

如果能看到 ROS master 启动日志，说明 ROS1 Noetic 安装成功。

## 10. turtlesim 测试

打开第一个终端：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

打开第二个终端：

```bash
source /opt/ros/noetic/setup.bash
rosrun turtlesim turtlesim_node
```

打开第三个终端：

```bash
source /opt/ros/noetic/setup.bash
rosrun turtlesim turtle_teleop_key
```

如果可以通过键盘控制小乌龟移动，说明 ROS1 图形和节点通信正常。

## 11. ROS1 与 ROS2 共存注意事项

如果同一台 Ubuntu 中也安装了 ROS2 Foxy，不要在同一个终端同时执行：

```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
```

建议：

- ROS1 Noetic 使用单独终端。
- ROS2 Foxy 使用单独终端。
- 每个终端只 source 一个 ROS 版本。

ROS1 Noetic：

```bash
source /opt/ros/noetic/setup.bash
```

ROS2 Foxy：

```bash
source /opt/ros/foxy/setup.bash
```
roscore
```
