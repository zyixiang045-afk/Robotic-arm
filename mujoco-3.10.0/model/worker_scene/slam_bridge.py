#!/usr/bin/env python3.8
"""把 MuJoCo 仿真桥接到 ROS 2，喂给 slam_toolbox 做 2D 激光 SLAM。

必须用 python3.8 跑：Foxy 的 rclpy 只有 cpython-38 扩展。相应地这里只能用
mujoco 3.2.3（py3.8 可装的最高版），所以加载的是 build_robot.py 额外输出的
scene_with_robot_py38.xml（去掉了 3.2.3 不认识的 texture colorspace 属性）。

发布:
  /scan        sensor_msgs/LaserScan            360 束、1°/束、12 m
  /odom        nav_msgs/Odometry                底盘里程计
  /clock       rosgraph_msgs/Clock              仿真时间（配 use_sim_time:=true）
  TF  odom -> base_footprint -> base_link -> laser
订阅:
  /cmd_vel     geometry_msgs/Twist              底盘速度指令（含 Nav2 输出）

坐标系遵循 REP-103/REP-105：base_link 的 +x 为前进方向，laser 的 θ=0 也是正前方。

用法:
  source /opt/ros/foxy/setup.bash
  python3.8 slam_bridge.py                 # headless，纯发数据
  python3.8 slam_bridge.py --view          # 同时开 MuJoCo 查看器
  python3.8 slam_bridge.py --patrol        # 自动沿航点巡视（建图用）
"""
import argparse
import math
import os

import numpy as np

import mujoco

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "scene_with_robot_py38.xml")

# 与 build_robot.py 的 LIDAR_* 常量保持一致
NUM_RAYS = 360
RANGE_MIN = 0.12
RANGE_MAX = 12.0
SCAN_HZ = 10.0                 # 典型 2D 雷达转速
ODOM_HZ = 50.0

# 里程计噪声：完全无噪声的里程计会让 slam_toolbox 的位姿图退化成纯里程计推算，
# 回环检测和 scan matching 都得不到检验。加一点标定残差量级的噪声更接近真机。
ODOM_NOISE = True
ODOM_TRANS_NOISE = 0.01        # 每米行程的位置噪声标准差(m)
ODOM_ROT_NOISE = 0.006         # 每弧度转动的角度噪声标准差(rad)

# 激光测距噪声（Hokuyo/RPLidar 量级）
SCAN_NOISE_STD = 0.012

# 巡视航点（世界系 x,y）：绕房间走一圈再回到起点，给回环检测创造机会。
# 狗初始朝世界 +x，因此第一段也朝 +x 方向进入南侧通道。北侧 x≈-3.5 有
# shuttle_a 动态扫掠线；自动巡视没有避障层，所以闭环必经段避开那里，改走东侧
# 下行后从南侧穿回西侧。MuJoCo 级仿真实测约 86 s 跑完全程 21 点。
PATROL_WAYPOINTS = [
    (-8.50, 0.00),
    (-8.00, -0.55),
    (-7.20, -1.30),
    (-6.50, -1.80),
    (-5.00, -1.80),
    (-4.00, -1.30),
    (-1.00, -1.40),
    (1.20, -1.10),
    (1.50, 0.00),
    (1.30, 0.90),
    (1.30, 1.30),
    (1.40, 0.20),
    (1.40, -0.80),
    (1.20, -1.40),
    (-2.00, -1.40),
    (-5.00, -1.80),
    (-8.60, -1.30),
    (-10.60, -1.10),
    (-10.70, 0.70),
    (-9.20, 1.90),
    (-8.50, 0.00),
]
PATROL_V = 0.45                # 巡视线速度(m/s)
PATROL_W = 0.8                 # 巡视角速度上限(rad/s)
PATROL_TOL = 0.22              # 到点判定半径(m)
PATROL_LOOP = True             # 走完一圈后从头再来（建图时多跑几圈更收敛）


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class SlamBridge(Node):
    def __init__(self, view=False, patrol=False, seed=0):
        super().__init__("mujoco_slam_bridge")
        self.model = mujoco.MjModel.from_xml_path(XML)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)   # ready 姿态
        mujoco.mj_forward(self.model, self.data)
        self.rng = np.random.RandomState(seed)

        m = self.model
        self.dt = m.opt.timestep

        # --- 传感器/执行器索引 ---
        self.scan_adr = np.array([
            m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR,
                                           "lidar_r%03d" % i)]
            for i in range(NUM_RAYS)])
        self.act = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_" + n)
                    for n in ("base_x", "base_y", "base_yaw")}
        self.qadr = {n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
                     for n in ("base_x", "base_y", "base_yaw")}
        self.vadr = {n: m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
                     for n in ("base_x", "base_y", "base_yaw")}

        # 狗 body 的初始世界位姿：base_x/base_y 是相对它的位移，
        # 而 odom 原点就取机器人启动位置（REP-105 要求 odom 连续且以启动位姿为原点）。
        dogb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "dog_base")
        self.dog_home = self.data.xpos[dogb][:2].copy()

        # 激光相对 base_link 的静态外参：直接从模型里量，避免手写错。
        sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "lidar_frame")
        lp_world = self.data.site_xpos[sid].copy()
        # base_link 定义在狗 body 原点、朝向为狗正面（世界 +x at t=0）。
        # 故 laser 在 base_link 系中的位置 = R_base^T (lp_world - dog_world)
        yaw0 = self._base_yaw_world()
        Rb = np.array([[math.cos(yaw0), -math.sin(yaw0)],
                       [math.sin(yaw0), math.cos(yaw0)]])
        rel = Rb.T @ (lp_world[:2] - self.dog_home)
        self.laser_xyz = (float(rel[0]), float(rel[1]), float(lp_world[2]))

        # --- ROS 接口 ---
        # /scan 用 best-effort：雷达数据是可丢的流，slam_toolbox 默认也订阅 sensor QoS
        scan_qos = QoSProfile(depth=5,
                              reliability=QoSReliabilityPolicy.BEST_EFFORT,
                              history=QoSHistoryPolicy.KEEP_LAST)
        self.pub_scan = self.create_publisher(LaserScan, "scan", scan_qos)
        self.pub_odom = self.create_publisher(Odometry, "odom", 20)
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)

        self._publish_static_tf()

        self.cmd = (0.0, 0.0, 0.0)      # (vx_body, vy_body, wz)
        self.patrol = patrol
        self.wp = 1                      # 航点 0 就是起点，直接奔第 1 个
        self.lap = 0
        # 里程计累积量（带噪声，与真值分离）
        self.odom_xy = np.zeros(2)
        self.odom_yaw = 0.0
        self._last_truth = self._truth_pose()

        self.viewer = None
        if view:
            # 用 importlib 而不是 `import mujoco.viewer`：后者会在函数作用域里
            # 把 mujoco 绑成局部名，导致同一函数中前面的 mujoco.* 全部报
            # UnboundLocalError。
            import importlib
            mj_viewer = importlib.import_module("mujoco.viewer")
            self.viewer = mj_viewer.launch_passive(self.model, self.data)

        # 物理按 dt 推进，用一个定时器驱动；发布按各自频率抽取。
        self.create_timer(1.0 / ODOM_HZ, self.tick)
        self._scan_every = max(1, int(round(ODOM_HZ / SCAN_HZ)))
        self._n = 0
        self.get_logger().info(
            "bridge up: laser at base_link (%.3f, %.3f, %.3f), %d rays, "
            "patrol=%s" % (self.laser_xyz + (NUM_RAYS, patrol)))

    # ---------------- 位姿 ----------------
    def _base_yaw_world(self):
        """狗正面在世界系中的方位角。t=0 时狗朝世界 +x，故 yaw0 = 0。"""
        return float(self.data.qpos[self.qadr["base_yaw"]])

    def _truth_pose(self):
        """底盘真值位姿 (x, y, yaw)，世界系。"""
        return np.array([
            self.dog_home[0] + float(self.data.qpos[self.qadr["base_x"]]),
            self.dog_home[1] + float(self.data.qpos[self.qadr["base_y"]]),
            self._base_yaw_world()])

    def _integrate_odom(self):
        """把真值增量加噪后积到里程计上 —— 模拟轮式里程计的累积漂移。"""
        cur = self._truth_pose()
        d = cur - self._last_truth
        self._last_truth = cur
        dyaw = math.atan2(math.sin(d[2]), math.cos(d[2]))
        # 世界增量转到上一时刻的车体系
        c, s = math.cos(self.odom_yaw), math.sin(self.odom_yaw)
        # 真值增量在世界系；先转到真值车体系，再按里程计朝向放回去
        yaw_prev = cur[2] - dyaw
        cb, sb = math.cos(yaw_prev), math.sin(yaw_prev)
        dbody = np.array([cb * d[0] + sb * d[1], -sb * d[0] + cb * d[1]])
        if ODOM_NOISE:
            dist = float(np.linalg.norm(dbody))
            dbody = dbody + self.rng.randn(2) * ODOM_TRANS_NOISE * math.sqrt(max(dist, 1e-9))
            dyaw = dyaw + self.rng.randn() * ODOM_ROT_NOISE * math.sqrt(abs(dyaw) + 1e-9)
        self.odom_xy += np.array([c * dbody[0] - s * dbody[1],
                                  s * dbody[0] + c * dbody[1]])
        self.odom_yaw = math.atan2(math.sin(self.odom_yaw + dyaw),
                                   math.cos(self.odom_yaw + dyaw))

    # ---------------- ROS 输出 ----------------
    def _stamp(self):
        t = self.data.time
        msg_sec = int(t)
        msg_nsec = int((t - msg_sec) * 1e9)
        return msg_sec, msg_nsec

    def _publish_static_tf(self):
        """base_footprint -> base_link -> laser 的固定外参。"""
        out = []
        s, ns = 0, 0
        t1 = TransformStamped()
        t1.header.stamp.sec, t1.header.stamp.nanosec = s, ns
        t1.header.frame_id = "base_footprint"
        t1.child_frame_id = "base_link"
        t1.transform.rotation.w = 1.0
        out.append(t1)

        t2 = TransformStamped()
        t2.header.stamp.sec, t2.header.stamp.nanosec = s, ns
        t2.header.frame_id = "base_link"
        t2.child_frame_id = "laser"
        t2.transform.translation.x = self.laser_xyz[0]
        t2.transform.translation.y = self.laser_xyz[1]
        t2.transform.translation.z = self.laser_xyz[2]
        t2.transform.rotation.w = 1.0
        out.append(t2)
        self.tf_static.sendTransform(out)

    def publish_odom(self):
        s, ns = self._stamp()
        vx = float(self.data.qvel[self.vadr["base_x"]])
        vy = float(self.data.qvel[self.vadr["base_y"]])
        wz = float(self.data.qvel[self.vadr["base_yaw"]])
        yaw = self._base_yaw_world()
        c, sn = math.cos(yaw), math.sin(yaw)
        vxb = c * vx + sn * vy          # 世界速度 -> 车体系
        vyb = -sn * vx + c * vy

        od = Odometry()
        od.header.stamp.sec, od.header.stamp.nanosec = s, ns
        od.header.frame_id = "odom"
        od.child_frame_id = "base_footprint"
        od.pose.pose.position.x = float(self.odom_xy[0])
        od.pose.pose.position.y = float(self.odom_xy[1])
        od.pose.pose.orientation = yaw_to_quat(self.odom_yaw)
        od.twist.twist.linear.x = vxb
        od.twist.twist.linear.y = vyb
        od.twist.twist.angular.z = wz
        # 对角协方差，量级和上面的噪声一致；slam_toolbox 自己做 scan match，
        # 这里只是让下游滤波器有个合理的不确定度。
        for i, v in ((0, 0.02), (7, 0.02), (35, 0.05)):
            od.pose.covariance[i] = v
        self.pub_odom.publish(od)

        tf = TransformStamped()
        tf.header.stamp.sec, tf.header.stamp.nanosec = s, ns
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = float(self.odom_xy[0])
        tf.transform.translation.y = float(self.odom_xy[1])
        tf.transform.rotation = yaw_to_quat(self.odom_yaw)
        self.tf.sendTransform(tf)

    def publish_scan(self):
        s, ns = self._stamp()
        r = self.data.sensordata[self.scan_adr].astype(np.float64).copy()
        # rangefinder 无命中返回 -1；LaserScan 约定用 inf 表示无回波
        miss = r < 0
        if SCAN_NOISE_STD > 0:
            r = r + self.rng.randn(NUM_RAYS) * SCAN_NOISE_STD
        r[miss] = float("inf")
        r[~miss] = np.clip(r[~miss], RANGE_MIN, RANGE_MAX)

        sc = LaserScan()
        sc.header.stamp.sec, sc.header.stamp.nanosec = s, ns
        sc.header.frame_id = "laser"
        # site 布置方式：i=0 对应 θ=-π，逆时针每束 +2π/N（见 build_robot._add_lidar）
        sc.angle_min = -math.pi
        sc.angle_max = math.pi - 2 * math.pi / NUM_RAYS
        sc.angle_increment = 2 * math.pi / NUM_RAYS
        sc.time_increment = 0.0
        sc.scan_time = 1.0 / SCAN_HZ
        sc.range_min = RANGE_MIN
        sc.range_max = RANGE_MAX
        sc.ranges = [float(x) for x in r]
        self.pub_scan.publish(sc)

    # ---------------- 控制 ----------------
    def on_cmd_vel(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.patrol = False          # 收到外部指令就交出控制权（Nav2 接管）

    def _patrol_cmd(self):
        """航点跟随：航向误差大时原地转，误差小时按 (1-|err|) 减速前进。

        减速项很重要 —— 全速冲着大航向误差走会切内弯撞上障碍物；
        这套参数配 PATROL_WAYPOINTS 的 MuJoCo 级仿真实测能闭环跑完全程。
        """
        if self.wp >= len(PATROL_WAYPOINTS):
            if not PATROL_LOOP:
                return (0.0, 0.0, 0.0)
            self.wp = 1                      # 起点就是航点 0，循环时从 1 开始
            self.lap += 1
            self.get_logger().info("lap %d done, looping" % self.lap)
        p = self._truth_pose()
        tx, ty = PATROL_WAYPOINTS[self.wp]
        dx, dy = tx - p[0], ty - p[1]
        dist = math.hypot(dx, dy)
        if dist < PATROL_TOL:
            self.wp += 1
            self.get_logger().info("waypoint %d/%d reached"
                                   % (self.wp, len(PATROL_WAYPOINTS)))
            return (0.0, 0.0, 0.0)
        want = math.atan2(dy, dx)
        err = math.atan2(math.sin(want - p[2]), math.cos(want - p[2]))
        wz = max(-PATROL_W, min(PATROL_W, 1.8 * err))
        if abs(err) > 0.35:
            return (0.0, 0.0, wz)            # 先摆正航向
        return (PATROL_V * max(0.25, 1.0 - abs(err)), 0.0, wz)

    def tick(self):
        if self.patrol:
            self.cmd = self._patrol_cmd()
        vxb, vyb, wz = self.cmd
        # 车体速度 -> 世界速度（底盘的 slide 关节是沿世界轴的）
        yaw = self._base_yaw_world()
        c, s = math.cos(yaw), math.sin(yaw)
        self.data.ctrl[self.act["base_x"]] = c * vxb - s * vyb
        self.data.ctrl[self.act["base_y"]] = s * vxb + c * vyb
        self.data.ctrl[self.act["base_yaw"]] = wz

        for _ in range(max(1, int(round((1.0 / ODOM_HZ) / self.dt)))):
            mujoco.mj_step(self.model, self.data)

        self._integrate_odom()
        ck = Clock()
        ck.clock.sec, ck.clock.nanosec = self._stamp()
        self.pub_clock.publish(ck)
        self.publish_odom()
        self._n += 1
        if self._n % self._scan_every == 0:
            self.publish_scan()
        if self.viewer is not None:
            self.viewer.sync()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true", help="同时开 MuJoCo 查看器")
    ap.add_argument("--patrol", action="store_true", help="自动沿航点巡视")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    rclpy.init()
    node = SlamBridge(view=args.view, patrol=args.patrol, seed=args.seed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.viewer is not None:
            node.viewer.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
