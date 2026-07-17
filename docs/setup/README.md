# 环境安装指南索引

**按照顺序安装。安装过程需要科学上网。**

## 推荐安装顺序

### 1. 安装 WSL2 与 Ubuntu 20.04

先阅读：

[wsl+ubuntu.md](./wsl+ubuntu.md)

该文档用于完成 Windows 侧基础环境配置：


### 2. 安装 ROS1 Noetic

然后阅读：

[ros1-noetic.md](./ros1-noetic.md)

该文档用于在 Ubuntu 20.04 中安装 ROS1 Noetic：

### 3. 安装 MuJoCo

最后阅读：

[mujoco.md](./mujoco.md)

## 快速检查清单

按顺序完成后，建议检查：

```bash
echo $WSL_DISTRO_NAME
lsb_release -a
echo $ROS_DISTRO
roscore
```

期望结果：

```text
Ubuntu-20.04
Ubuntu 20.04 / focal
noetic
roscore 正常启动
```

## 注意事项

- 不要跳过 WSL/Ubuntu，后续 ROS 和 MuJoCo 都依赖该基础环境。
- ROS1 Noetic 适配 Ubuntu 20.04，不建议混用 Ubuntu 22.04。
- 如果同一台机器也安装了 ROS2，请分开终端使用 ROS1 和 ROS2，不要在同一个终端里同时 source 两套 ROS 环境。
- 大型编译或仿真项目建议放在 Linux home 目录，例如 `~/Robotic-arm` 或 `~/ros_ws`，不要长期放在 `/mnt/c/...` 下。
