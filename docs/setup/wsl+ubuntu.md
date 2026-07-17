# WSL2 + Ubuntu 20.04 部署指南

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

会显示系统中已有的ubuntu版本。

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
git clone https://github.com/zyixiang045-afk/Robotic-arm
```

## 4. VS Code 打开 Ubuntu 20.04 项目

推荐使用 VS Code Remote - WSL。

方式一：在 Ubuntu 终端中：

打开用户目录（之后linux系统中下载的内容存放在此目录）
```bash
cd ~
code .
```

此过程中vscode会提示下载wsl扩展，按照提示下载此系列扩展。

下载完成后点击左侧电脑形状的任务栏，会显示下载完成的ubuntu版本，点击”→“即可进入所选定的目录。在扩展市场中，原先下载好的扩展上会显示“在wsl中安装”，按需求点击必要的扩展进行安装。

方式二：在 VS Code 中：（前提是已下载wsl扩展）

1. 点击左下角远程连接按钮。
2. 选择 `Connect to WSL using Distro...`。
3. 选择 `Ubuntu-20.04`。
4. 打开 `/home/<你的Linux用户名>`。

此后在Windows系统中打开vscode默认远程连接ubuntu系统。在vscode左下角显示。

## 6. Linux系统中Codex与Claude配置

按照https://docs.right.codes/docs/rc_cli_config/wsl.html 的流程先安装 Node.js 和 npm，然后依照 通过Windows下的cc-switch导入 进行配置。

在ubuntu系统中输入codex出现对话框则证明配置成功，会出现很多warning但是不会影响使用。
