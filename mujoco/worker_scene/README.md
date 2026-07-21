# 实验室内景场景

一个 MuJoCo 仿真场景，模拟一间实验室内部：

- 四面墙围出的房间（约 15 m × 6 m，墙高 2.8 m）
- 多张实验台：主工作台 + 3 张靠墙实验台，另有置物架、方凳、台面器皿（烧杯/试剂瓶/仪器）等细节
- 主工作台上一红一蓝两把螺丝刀（`freejoint`，可推动/拾取）
- 一名静态工人（人体模型），站在走廊一端、面朝主工作台
- 走廊中**两个规则运动**的物体，供机械狗做动态避障：
  - 一台横穿走廊的往复料车（`shuttle_a`，沿 y 平移）
  - 一根绕竖直轴扫掠的挡杆（`sweep_1`，在机械狗身位高度扫过）
- 走廊中 3 个**静态货箱**障碍物（`crate_1..3`），供机械狗绕行

两个动态物体均由**弹簧驱动、阻尼为 0**，无需任何控制器即可在查看器中
自行做无衰减往复运动（已验证 30 s 内幅度不衰减）。

## 文件
- `build_scene.py` —— 场景生成器，写出 `scene.xml`
- `scene.xml` —— 最近一次生成的模型（可直接加载）
- `preview.png` —— 生成场景的离屏渲染预览

## 用法
```bash
python3 build_scene.py            # 生成 scene.xml 并校验编译
python3 build_scene.py --view     # 生成后打开交互式查看器（需图形界面/WSLg）
python3 build_scene.py --seed 0   # 固定随机种子（仅影响台面小道具摆放）
```

也可用官方工具直接查看已生成的模型：
```bash
python3 -m mujoco.viewer --mjcf=scene.xml
```

## 布局与调参说明（见 build_scene.py 顶部常量与 build_xml 中的列表）
- 主工作台在原点，工人在 `x=-10` 处面朝 +x，中间是 10 m 走廊。
- 静态家具在 `furniture` 列表、静态障碍物在 `statics` 列表，均逐个显式摆放。
- 动态物体在 `movers` 列表（当前 2 个）。往复运动靠 `slide/hinge` 关节的
  `stiffness` + `springref` 实现：平衡点设在振幅处、起点在 0，故在 `[0, 2·振幅]`
  间往复；周期 `T ≈ 2π·sqrt(m/k)`，调 `mass` / `stiffness` 即可改变快慢。
- 注：`build_scene.py` 中仍保留了 `pendulum` / `conveyor` 等辅助函数，
  如需增加运动障碍可直接在 `movers` 列表中调用。
- 每个运动部件与其自身静态支架通过 `<contact><exclude>` 关闭碰撞
  （支架只是结构，不是障碍物）。
