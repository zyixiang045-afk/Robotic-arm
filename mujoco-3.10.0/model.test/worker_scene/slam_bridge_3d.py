#!/usr/bin/env python3.8
"""把 MuJoCo 仿真桥接到 ROS 2，发布 3D 激光点云（PointCloud2）。

与 slam_bridge.py 类似，但使用多层 3D 激光雷达（类 VLP-16）替代单层 2D 雷达。
发布:
  /pointcloud  sensor_msgs/PointCloud2       16层×180束 = 2880 点/帧
  /odom        nav_msgs/Odometry             底盘里程计
  /clock       rosgraph_msgs/Clock           仿真时间
  TF  odom -> base_footprint -> base_link -> lidar3d
订阅:
  /cmd_vel     geometry_msgs/Twist           底盘速度指令

用法:
  source /opt/ros/foxy/setup.bash
  python3.8 slam_bridge_3d.py                 # headless
  python3.8 slam_bridge_3d.py --view          # 开 MuJoCo 查看器
  python3.8 slam_bridge_3d.py --patrol        # 自动巡视
"""
import argparse
import math
import os
import struct

import numpy as np

import mujoco

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

HERE = os.path.dirname(os.path.abspath(__file__))
XML_3D = os.path.join(HERE, "scene_with_robot_3d_py38.xml")
XML_FALLBACK = os.path.join(HERE, "scene_with_robot_py38.xml")
XML = XML_3D if os.path.exists(XML_3D) else XML_FALLBACK

# 3D 雷达参数（须与 build_robot.py 中 LIDAR3D_* 常量保持一致）
NUM_LAYERS = 16
NUM_H_RAYS = 180
NUM_TOTAL_RAYS = NUM_LAYERS * NUM_H_RAYS  # 2880
RANGE_MIN = 0.12
RANGE_MAX = 12.0
V_MIN = math.radians(-15.0)
V_MAX = math.radians(15.0)
SCAN_HZ = 10.0
ODOM_HZ = 50.0

# 噪声参数（同 2D 版）
ODOM_NOISE = True
ODOM_TRANS_NOISE = 0.01
ODOM_ROT_NOISE = 0.006
SCAN_NOISE_STD = 0.012

# 巡视航点（同 2D 版 slam_bridge.py）
PATROL_WAYPOINTS = [
    (-8.50, 0.00), (-8.00, -0.55), (-7.20, -1.30), (-6.50, -1.80),
    (-5.00, -1.80), (-4.00, -1.30), (-1.00, -1.40), (1.20, -1.10),
    (1.50, 0.00), (1.30, 0.90), (1.30, 1.30), (1.40, 0.20),
    (1.40, -0.80), (1.20, -1.40), (-2.00, -1.40), (-5.00, -1.80),
    (-8.60, -1.30), (-10.60, -1.10), (-10.70, 0.70), (-9.20, 1.90),
    (-8.50, 0.00),
]
PATROL_V = 0.45
PATROL_W = 0.8
PATROL_TOL = 0.22
PATROL_LOOP = True
PATROL_MAX_LAPS = 0            # 巡视几圈后自动停止（0 = 无限循环）


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class SlamBridge3D(Node):
    def __init__(self, view=False, patrol=False, seed=0, no_lidar=False, max_laps=PATROL_MAX_LAPS):
        super().__init__("mujoco_slam_bridge_3d")
        self.no_lidar = no_lidar
        self.model = mujoco.MjModel.from_xml_path(XML)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.rng = np.random.RandomState(seed)

        m = self.model
        self.dt = m.opt.timestep

        # --- 3D 雷达传感器索引（layer × azimuth）---
        self.scan_adr = np.array([
            m.sensor_adr[mujoco.mj_name2id(
                m, mujoco.mjtObj.mjOBJ_SENSOR,
                "lidar3d_r%02d_%03d" % (layer, az))]
            for layer in range(NUM_LAYERS)
            for az in range(NUM_H_RAYS)
        ]).reshape(NUM_LAYERS, NUM_H_RAYS)

        # 预计算每根射线的方向向量（雷达局部系）
        self._build_direction_table()

        # --- 执行器/关节索引 ---
        self.act = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_" + n)
                    for n in ("base_x", "base_y", "base_yaw")}
        self.qadr = {n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
                     for n in ("base_x", "base_y", "base_yaw")}
        self.vadr = {n: m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
                     for n in ("base_x", "base_y", "base_yaw")}

        # 狗初始世界位置
        dogb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "dog_base")
        self.dog_home = self.data.xpos[dogb][:2].copy()

        # 3D 雷达相对 base_link 的静态外参
        sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "lidar3d_frame")
        lp_world = self.data.site_xpos[sid].copy()
        yaw0 = self._base_yaw_world()
        Rb = np.array([[math.cos(yaw0), -math.sin(yaw0)],
                       [math.sin(yaw0), math.cos(yaw0)]])
        rel = Rb.T @ (lp_world[:2] - self.dog_home)
        self.lidar_xyz = (float(rel[0]), float(rel[1]), float(lp_world[2]))

        # --- ROS 接口 ---
        pc_qos = QoSProfile(depth=5,
                            reliability=QoSReliabilityPolicy.RELIABLE,
                            history=QoSHistoryPolicy.KEEP_LAST)
        self.pub_pc = self.create_publisher(PointCloud2, "pointcloud", pc_qos)
        self.pub_odom = self.create_publisher(Odometry, "odom", 20)
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)

        self._publish_static_tf()

        self.cmd = (0.0, 0.0, 0.0)
        self.patrol = patrol
        self.max_laps = max_laps
        self._last_cmd_t = -10.0   # 看门狗：最近一次收到外部 /cmd_vel 的仿真时间
        self.wp = 1
        self.lap = 0
        self.odom_xy = np.zeros(2)
        self.odom_yaw = 0.0
        self._last_truth = self._truth_pose()

        self.viewer = None
        if view:
            import importlib
            mj_viewer = importlib.import_module("mujoco.viewer")
            self.viewer = mj_viewer.launch_passive(self.model, self.data)
            if self.no_lidar:
                # 关闭 rangefinder 黄色射线可视化
                try:
                    self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
                except Exception:
                    pass

        self._scan_every = max(1, int(round(ODOM_HZ / SCAN_HZ)))
        self._n = 0

        # 预发 clock 让其他 use_sim_time 节点先同步仿真时间，避免 TF_OLD_DATA
        for _ in range(10):
            ck = Clock()
            ck.clock.sec, ck.clock.nanosec = self._stamp()
            self.pub_clock.publish(ck)

        self.create_timer(1.0 / ODOM_HZ, self.tick)
        self.get_logger().info(
            "3D bridge up: lidar3d at base_link (%.3f, %.3f, %.3f), "
            "%d layers x %d rays = %d total, patrol=%s"
            % (self.lidar_xyz + (NUM_LAYERS, NUM_H_RAYS, NUM_TOTAL_RAYS, patrol)))

    def _build_direction_table(self):
        """预计算每根射线在雷达局部系中的单位方向向量。"""
        v_step = (V_MAX - V_MIN) / (NUM_LAYERS - 1) if NUM_LAYERS > 1 else 0.0
        dirs = np.zeros((NUM_LAYERS, NUM_H_RAYS, 3), dtype=np.float32)
        for layer in range(NUM_LAYERS):
            phi = V_MIN + layer * v_step
            cp, sp = math.cos(phi), math.sin(phi)
            for az in range(NUM_H_RAYS):
                theta = -math.pi + 2 * math.pi * az / NUM_H_RAYS
                dirs[layer, az] = [cp * math.cos(theta),
                                   cp * math.sin(theta), sp]
        self._dirs = dirs  # (16, 180, 3)

    # ---------------- 位姿 ----------------
    def _base_yaw_world(self):
        return float(self.data.qpos[self.qadr["base_yaw"]])

    def _truth_pose(self):
        return np.array([
            self.dog_home[0] + float(self.data.qpos[self.qadr["base_x"]]),
            self.dog_home[1] + float(self.data.qpos[self.qadr["base_y"]]),
            self._base_yaw_world()])

    def _integrate_odom(self):
        cur = self._truth_pose()
        d = cur - self._last_truth
        self._last_truth = cur
        dyaw = math.atan2(math.sin(d[2]), math.cos(d[2]))
        c, s = math.cos(self.odom_yaw), math.sin(self.odom_yaw)
        yaw_prev = cur[2] - dyaw
        cb, sb = math.cos(yaw_prev), math.sin(yaw_prev)
        dbody = np.array([cb * d[0] + sb * d[1], -sb * d[0] + cb * d[1]])
        if ODOM_NOISE:
            dist = float(np.linalg.norm(dbody))
            dbody += self.rng.randn(2) * ODOM_TRANS_NOISE * math.sqrt(max(dist, 1e-9))
            dyaw += self.rng.randn() * ODOM_ROT_NOISE * math.sqrt(abs(dyaw) + 1e-9)
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
        t2.child_frame_id = "lidar3d"
        t2.transform.translation.x = self.lidar_xyz[0]
        t2.transform.translation.y = self.lidar_xyz[1]
        t2.transform.translation.z = self.lidar_xyz[2]
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
        vxb = c * vx + sn * vy
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

    def publish_pointcloud(self):
        s, ns = self._stamp()
        # 读取全部 2880 个距离值
        ranges = np.empty((NUM_LAYERS, NUM_H_RAYS), dtype=np.float64)
        for layer in range(NUM_LAYERS):
            for az in range(NUM_H_RAYS):
                ranges[layer, az] = self.data.sensordata[
                    self.scan_adr[layer, az]]

        # 加噪声
        if SCAN_NOISE_STD > 0:
            ranges += self.rng.randn(NUM_LAYERS, NUM_H_RAYS) * SCAN_NOISE_STD

        # 过滤无效点：rangefinder 无命中返回 -1
        valid = (ranges > RANGE_MIN) & (ranges < RANGE_MAX)

        # 距离 × 方向 = XYZ（雷达局部系）
        pts = (ranges[:, :, np.newaxis] * self._dirs)  # (16, 180, 3)
        pts_valid = pts[valid]  # (N, 3)

        # 构建 PointCloud2 消息
        msg = PointCloud2()
        msg.header.stamp.sec, msg.header.stamp.nanosec = s, ns
        msg.header.frame_id = "lidar3d"
        msg.height = 1
        msg.width = len(pts_valid)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        # 打包二进制数据
        if msg.width > 0:
            buf = np.zeros((msg.width, 4), dtype=np.float32)
            buf[:, :3] = pts_valid.astype(np.float32)
            buf[:, 3] = 1.0  # intensity 固定为 1.0
            msg.data = buf.tobytes()
        else:
            msg.data = b''

        self.pub_pc.publish(msg)

    # ---------------- 控制 ----------------
    def on_cmd_vel(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.patrol = False
        self._last_cmd_t = self.data.time

    def _patrol_cmd(self):
        if self.wp >= len(PATROL_WAYPOINTS):
            if not PATROL_LOOP:
                return (0.0, 0.0, 0.0)
            self.wp = 1
            self.lap += 1
            self.get_logger().info("lap %d done, looping" % self.lap)
            if self.max_laps > 0 and self.lap >= self.max_laps:
                self.patrol = False
                self.cmd = (0.0, 0.0, 0.0)
                self.get_logger().info("巡视 %d 圈完成，自动停止，可以保存地图" % self.max_laps)
                return (0.0, 0.0, 0.0)
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
            return (0.0, 0.0, wz)
        return (PATROL_V * max(0.25, 1.0 - abs(err)), 0.0, wz)

    def tick(self):
        if self.patrol:
            self.cmd = self._patrol_cmd()
        elif self.data.time - self._last_cmd_t > 0.5:
            # 看门狗：超过 0.5s 没有新速度指令就停车，避免狗带着最后一条指令跑飞
            self.cmd = (0.0, 0.0, 0.0)
        vxb, vyb, wz = self.cmd
        yaw = self._base_yaw_world()
        c, s = math.cos(yaw), math.sin(yaw)
        self.data.ctrl[self.act["base_x"]] = c * vxb - s * vyb
        self.data.ctrl[self.act["base_y"]] = s * vxb + c * vyb
        self.data.ctrl[self.act["base_yaw"]] = wz

        for _ in range(max(1, int(round((1.0 / ODOM_HZ) / self.dt)))):
            mujoco.mj_step(self.model, self.data)

        self._integrate_odom()

        # 关键：先发 TF 和传感器数据，最后发 clock。
        # 原因：rtabmap 用 use_sim_time，收到 clock 才更新内部时间。
        # 如果先发 clock 再发 TF，rtabmap 时间已经推进到 T，
        # 而 TF(T) 还没到 buffer 里，下一轮 clock(T+dt) 到了后
        # TF(T) 才到，就变成 "来自过去的数据" → TF_OLD_DATA。
        # 先发 TF 确保 buffer 里已有数据，再用 clock 推进时间。
        self.publish_odom()
        self._n += 1
        if not self.no_lidar and self._n % self._scan_every == 0:
            self.publish_pointcloud()

        # clock 最后发，让所有 use_sim_time 订阅者在时间推进前已收到数据
        ck = Clock()
        ck.clock.sec, ck.clock.nanosec = self._stamp()
        self.pub_clock.publish(ck)

        if self.viewer is not None:
            self.viewer.sync()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true", help="同时开 MuJoCo 查看器")
    ap.add_argument("--patrol", action="store_true", help="自动沿航点巡视")
    ap.add_argument("--no-lidar", action="store_true", help="不发布点云（用于离线导航）")
    ap.add_argument("--laps", type=int, default=PATROL_MAX_LAPS,
                    help="巡视几圈后自动停止并提示（0=无限循环）")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    rclpy.init()
    node = SlamBridge3D(view=args.view, patrol=args.patrol, seed=args.seed,
                        no_lidar=args.no_lidar, max_laps=args.laps)
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
