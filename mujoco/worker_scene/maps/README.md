# Maps 目录说明

本目录按场景分开存放 SLAM 建图产生的地图文件。

## 目录结构

```
maps/
├── ariac/                  # 默认 ARIAC 主场景地图（首次保存时创建）
│   ├── ariac_map_3d.pgm
│   ├── ariac_map_3d.yaml
│   └── rtabmap.db
├── lab/                    # 实验室场景地图
│   ├── lab_map_3d.pgm      # 3D SLAM 投影 2D 栅格图像
│   ├── lab_map_3d.yaml     # 对应配置（分辨率、原点）
│   ├── lab_map.pgm         # 2D SLAM 栅格图像
│   ├── lab_map.yaml        # 对应配置
│   └── rtabmap.db          # rtabmap 完整 3D 数据库
├── warehouse/              # 仓库场景地图
│   ├── warehouse_map_3d.pgm
│   ├── warehouse_map_3d.yaml
│   └── rtabmap.db
└── README.md
```

## 保存地图

```bash
./slam/save_map_3d.sh                    # 默认保存 ARIAC 地图
./slam/save_map_3d.sh --scene ariac      # 保存到 maps/ariac/
./slam/save_map_3d.sh --scene lab        # 保存实验室地图
./slam/save_map_3d.sh --scene warehouse  # 保存仓库地图
```

## 查看地图

```bash
./slam/view_map.sh maps/ariac/ariac_map_3d.yaml
./slam/view_map.sh maps/lab/lab_map_3d.yaml
./slam/view_map.sh maps/warehouse/warehouse_map_3d.yaml
```

## 路径规划

```bash
# ARIAC（默认；需先完成并保存 ARIAC 地图）
./slam/run_nav_saved.sh --view
./slam/run_nav_saved.sh --scene ariac --view

# 实验室
./slam/run_nav_saved.sh --scene lab --view

# 仓库
./slam/run_nav_saved.sh --scene warehouse --view
```

## 动态行人避障测试

使用仓库保存的静态地图进行 Lazy Theta* 全局规划，同时启用实时点云和
条件式全向 DWA。多个测试行人会在仓库不同通道交错横穿并短暂停留；只有行人进入
当前路径走廊时，导航状态才会切换到 `DYNAMIC_AVOID`。

```bash
cd /home/ee304/jbgs/mujoco-3.10.0/model.test/worker_scene
./slam/run_nav_saved.sh --scene warehouse --view --dynamic-person
```

在 RViz2 中沿西侧通道设置目标，或发送以下目标点：

```bash
ros2 topic pub -1 /nav_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 0.0, y: 10.0}, orientation: {w: 1.0}}}"
```

观察导航状态：

```bash
ros2 topic echo /nav_status
```

正常情况下可以观察到：

```text
PLANNING -> FOLLOWING -> DYNAMIC_AVOID -> FOLLOWING -> ARRIVED
```

动态行人默认停在场外；不传 `--dynamic-person` 时，不会影响普通静态地图导航。
MuJoCo 查看器会隐藏黄色雷达射线，但实时雷达计算和 `/pointcloud` 仍保持启用。

## 坐标系说明

| 场景 | 狗起点(世界系) | map 原点 |
|------|---------------|----------|
| ariac | (-4, 0) | 狗起点 |
| lab | (-8.5, 0) | 狗起点 |
| warehouse | (-9, -9) | 狗起点 |

世界系 → map 系：`(x - origin_x, y - origin_y)`
