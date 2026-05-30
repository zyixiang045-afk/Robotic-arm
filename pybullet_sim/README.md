# PyBullet Simulation

本目录是一个本地 Python 虚拟环境 + PyBullet 的最小仿真项目。

## 目录

```text
pybullet_sim/
  .venv/
  requirements.txt
  src/
    pybullet_sim/
      run_sim.py
```

## 运行方式

先进入项目目录：

```powershell
cd C:\Users\18707\pybullet_sim
```

无窗口快速验证，适合自动测试和日志调试：

```powershell
.\.venv\Scripts\python.exe -m src.pybullet_sim.run_sim --mode direct --seconds 3 --no-realtime
```

打开 PyBullet 图形窗口：

```powershell
.\.venv\Scripts\python.exe -m src.pybullet_sim.run_sim --mode gui --seconds 15
```

运行 KUKA 机械臂示例：

```powershell
.\.venv\Scripts\python.exe -m src.pybullet_sim.run_sim --mode gui --robot kuka
```

## 调试参数

打印机器人关节表：

```powershell
.\.venv\Scripts\python.exe -m src.pybullet_sim.run_sim --mode direct --robot kuka --inspect-joints --seconds 1 --no-realtime
```

每 0.5 秒打印一次机器人位姿：

```powershell
.\.venv\Scripts\python.exe -m src.pybullet_sim.run_sim --mode direct --log-every 0.5 --seconds 3 --no-realtime
```

GUI 运行结束后保留窗口，按 Enter 再退出：

```powershell
.\.venv\Scripts\python.exe -m src.pybullet_sim.run_sim --mode gui --pause-at-end
```

## 常用参数

- `--mode gui`：打开图形窗口。
- `--mode direct`：后台仿真，不打开窗口。
- `--seconds 10`：仿真运行秒数。
- `--robot r2d2`：加载 R2D2 移动机器人。
- `--robot kuka`：加载 KUKA iiwa 机械臂。
- `--time-step 0.0041666667`：设置仿真步长，默认约等于 240 Hz。
- `--inspect-joints`：输出关节编号、名称、类型和关节限位。
- `--log-every 0.5`：每隔指定仿真秒数输出机器人位姿。
- `--no-realtime`：尽可能快地运行，适合无窗口批量测试。
- `--pause-at-end`：GUI 模式结束后暂停，便于观察最终状态。
