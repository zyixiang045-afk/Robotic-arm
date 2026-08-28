# Windows 终端安装 WSL2 与 Ubuntu 20.04 指南

本文档用于在 Windows 电脑上通过终端安装和配置 WSL2 + Ubuntu 20.04，适合作为后续 ROS 2 Foxy、Linux 开发工具链、VS Code Remote WSL 的基础环境。

参考：

- [Microsoft WSL 安装文档](https://learn.microsoft.com/windows/wsl/install)
- [Microsoft WSL 常用命令](https://learn.microsoft.com/windows/wsl/basic-commands)

## 1. 前置要求

建议环境：

```text
Windows 10 版本 2004 及以上，或 Windows 11
PowerShell / Windows Terminal
CPU 虚拟化已开启
```

检查 WSL 状态：

```powershell
wsl --status
```

如果提示虚拟化未开启，需要进入 BIOS/UEFI，开启 Intel VT-x / AMD-V / SVM 等虚拟化选项。

## 2. 安装 WSL2

在 PowerShell 中执行：

```powershell
wsl --install --no-distribution
wsl --set-default-version 2
```

执行完成后，如果系统提示重启，请重启 Windows。

## 3. 安装 Ubuntu 20.04

优先尝试：

```powershell
wsl --install -d Ubuntu-20.04
```

如果提示列表中没有 `Ubuntu-20.04`，先查看可安装发行版：

```powershell
wsl --list --online
```

如果仍然没有，请从 Microsoft Store 搜索并安装：

```text
Ubuntu 20.04 LTS
```

安装后启动：

```powershell
wsl -d Ubuntu-20.04
```

第一次启动会要求创建 Linux 用户名和密码。密码输入时不会显示字符，这是正常现象。

## 4. 设置 Ubuntu 20.04 为默认发行版

查看已安装发行版：

```powershell
wsl -l -v
```

设置默认发行版：

```powershell
wsl --set-default Ubuntu-20.04
```

确认默认发行版：

```powershell
wsl -l -v
```

默认发行版前会显示 `*`：

```text
  NAME             STATE           VERSION
* Ubuntu-20.04     Stopped         2
```

之后直接执行：

```powershell
wsl
```

就会进入 Ubuntu 20.04。

## 5. Ubuntu 20.04 基础初始化

进入 Ubuntu 后执行：

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y curl wget git ca-certificates gnupg lsb-release software-properties-common build-essential
```

检查系统版本：

```bash
lsb_release -a
uname -a
echo $WSL_DISTRO_NAME
```

期望 `lsb_release -a` 中包含：

```text
Ubuntu 20.04
focal
```

## 6. Windows 与 WSL 路径

WSL 中访问 Windows 文件：

```bash
cd /mnt/c/Users/<你的Windows用户名>
```

Windows 资源管理器访问 WSL：

```text
\\wsl$\Ubuntu-20.04\home\<你的Linux用户名>
```

建议：

- 文档、README、轻量脚本可以放在 `/mnt/c/...`。
- ROS 2、CMake、colcon 等大量编译项目建议放在 Linux home 下，例如 `~/ros2_ws`。
- 本项目推荐 clone 到 Linux home，避免绑定某个队员的 Windows 用户路径：

```bash
git clone https://github.com/zyixiang045-afk/Robotic-arm.git ~/Robotic-arm
cd ~/Robotic-arm/ros2-foxy-sim
```

## 7. VS Code Remote WSL

在 Windows 安装 VS Code，并安装扩展：

```text
WSL
```

方式一：从 Ubuntu 终端打开：

```bash
cd ~
code .
```

方式二：从 VS Code 打开：

1. 点击左下角远程连接按钮。
2. 选择 `Connect to WSL using Distro...`。
3. 选择 `Ubuntu-20.04`。
4. 打开 `/home/<你的Linux用户名>` 或项目目录。

确认当前终端在 WSL 中：

```bash
echo $WSL_DISTRO_NAME
```

## 8. 代理与网络问题

如果启动 WSL 时出现：

```text
wsl: 检测到 localhost 代理配置，但未镜像到 WSL。NAT 模式下的 WSL 不支持 localhost 代理。
```

这不是致命错误。如果下面命令正常，可以忽略：

```bash
sudo apt update
curl -I https://github.com
```

如果访问 GitHub 或 raw.githubusercontent.com 超时，假设 Windows 代理端口是 `7890`，在 Ubuntu 中执行：

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

## 9. 常用 WSL 命令

查看发行版：

```powershell
wsl -l -v
```

进入 Ubuntu 20.04：

```powershell
wsl -d Ubuntu-20.04
```

关闭 Ubuntu 20.04：

```powershell
wsl --terminate Ubuntu-20.04
```

关闭所有 WSL：

```powershell
wsl --shutdown
```

更新 WSL：

```powershell
wsl --update
```

卸载 Ubuntu 20.04：

```powershell
wsl --unregister Ubuntu-20.04
```

注意：`wsl --unregister` 会删除 Ubuntu 20.04 内的所有文件。执行前请先备份重要数据。

## 10. 备份 Ubuntu 20.04

导出备份：

```powershell
wsl --shutdown
wsl --export Ubuntu-20.04 C:\Users\<你的Windows用户名>\Documents\ubuntu-20.04-backup.tar
```

从备份导入：

```powershell
mkdir C:\WSL\Ubuntu-20.04
wsl --import Ubuntu-20.04 C:\WSL\Ubuntu-20.04 C:\Users\<你的Windows用户名>\Documents\ubuntu-20.04-backup.tar --version 2
```

## 11. 常见问题

### 11.1 `WSL_E_DISTRO_NOT_FOUND`

说明发行版名称不对或还没有安装。

先查看实际名称：

```powershell
wsl -l -v
```

再使用实际名称启动：

```powershell
wsl -d Ubuntu-20.04
```

### 11.2 `wsl --list --online` 没有 Ubuntu 20.04

处理方式：

1. 打开 Microsoft Store。
2. 搜索 `Ubuntu 20.04 LTS`。
3. 安装后执行：

```powershell
wsl -l -v
```

### 11.3 `apt update` 超时

先测试网络：

```bash
curl -I https://github.com
```

如果 GitHub 也超时，按第 8 节配置代理。

### 11.4 忘记 Linux 密码

可以临时用 root 进入：

```powershell
wsl -d Ubuntu-20.04 -u root
```

重置用户密码：

```bash
passwd <你的Linux用户名>
```

## 12. 最小流程汇总

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
sudo apt upgrade -y
sudo apt install -y curl wget git build-essential ca-certificates
mkdir -p ~/ros2_ws/src
```

验证：

```bash
lsb_release -a
echo $WSL_DISTRO_NAME
curl -I https://github.com
```

至此，WSL2 + Ubuntu 20.04 基础环境完成。
