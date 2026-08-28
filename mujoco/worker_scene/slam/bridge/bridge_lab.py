#!/usr/bin/env python3.8
"""把 MuJoCo 仿真桥接到 ROS 2，发布 3D 激光点云（PointCloud2）。

  /pointcloud  sensor_msgs/PointCloud2       64层×360束（过滤后为有效命中点）
  /pointcloud_visual sensor_msgs/PointCloud2 RViz 实时层（使用最新 TF，不供 SLAM）
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
import subprocess
import threading

import numpy as np

import mujoco

from local_avoidance import AvoidanceConfig, LocalAvoidance

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.join(HERE, "..", "..")
XML_3D = os.path.join(PROJECT_ROOT, "model", "robot", "scene_with_robot_3d_py38.xml")
XML = XML_3D

# 3D 雷达参数。
#
# 射线不再走 XML 里的 2880 个 <rangefinder>，而是每帧用 mj_multiRay 批量
# 投射（见 _scan）。理由同 bridge_warehouse.py：
#   1) 性能：2880 个 rangefinder 占 89% 的 mj_step 时间；mj_multiRay 投同样
#      的射线只要 1.3 ms，禁用传感器后 mj_step 快 10 倍。
#   2) 视场：±15° 对地面机器人不够——雷达在 0.95 m 高，最低层要到 3.67 m
#      外才碰地面，近处地面和矮障碍全在盲区，而避障阈值就在盲区里。
VERTICAL_ANGLES_DEG = np.concatenate([
    np.linspace(-60.0, -20.0, 12, endpoint=False),
    np.linspace(-20.0, 5.0, 42, endpoint=False),
    np.linspace(5.0, 20.0, 10),
])
NUM_LAYERS = len(VERTICAL_ANGLES_DEG)
NUM_H_RAYS = 360
NUM_TOTAL_RAYS = NUM_LAYERS * NUM_H_RAYS  # 23040
RANGE_MIN = 0.12
RANGE_MAX = 8.0
V_MIN = math.radians(float(VERTICAL_ANGLES_DEG[0]))
V_MAX = math.radians(float(VERTICAL_ANGLES_DEG[-1]))
SCAN_HZ = 10.0
ODOM_HZ = 50.0

# 点云高度裁剪（世界系，米）。必须保留地板点：rtabmap 靠地面点标记空闲
# 栅格（RViz 里的白色区域），滤掉地面会让地面随狗移动而闪烁/消失。
Z_MIN_WORLD = -0.05
Z_MAX_WORLD = 2.60

# 噪声参数。默认使用仿真底盘的无噪声里程计，避免长路径随机游走在回环时
# 被 ICP 一次性拉回。需要验证回环抗漂移能力时显式传 --odom-noise。
ODOM_NOISE_DEFAULT = False
ODOM_TRANS_NOISE = 0.01
ODOM_ROT_NOISE = 0.006
SCAN_NOISE_STD = 0.012

# 避障参数
OBSTACLE_DIST = 0.70       # 雷达到障碍的水平净距阈值（米）
OBSTACLE_SLOW_DIST = 1.35  # 开始减速的水平净距
OBSTACLE_FRONT_AZ = 42     # 前方扇区半角（单位：度，与射线数无关）

# 巡视航点（实验室场景）
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
PATROL_TOL = 0.30
PATROL_LOOP = False
PATROL_MAX_LAPS = 1            # 巡视 1 圈后自动停止

STUCK_WINDOW = 8.0
PROGRESS_DIST = 0.25
MAX_RECOVERY_ATTEMPTS = 3


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class SlamBridge3D(Node):
    def __init__(self, view=False, patrol=False, seed=0, no_lidar=False,
                 max_laps=PATROL_MAX_LAPS, odom_noise=ODOM_NOISE_DEFAULT):
        super().__init__("mujoco_slam_bridge_3d")
        self.no_lidar = no_lidar
        self.odom_noise = bool(odom_noise)
        self.model = mujoco.MjModel.from_xml_path(XML)
        self.data = mujoco.MjData(self.model)
        if self.model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.rng = np.random.RandomState(seed)

        m = self.model
        self.dt = m.opt.timestep

        # XML 里的 <rangefinder> 不再使用（我们自己投射射线），禁用传感器
        # 计算让 mj_step 快 10 倍。
        m.opt.disableflags = (int(m.opt.disableflags)
                              | int(mujoco.mjtDisableBit.mjDSBL_SENSOR))

        # 预计算每根射线的方向向量（雷达局部系）
        self._build_direction_table()

        # mj_multiRay 的输出缓冲 + 排除狗自身的 body
        self._ray_geomid = np.zeros(NUM_TOTAL_RAYS, dtype=np.int32)
        self._ray_dist = np.zeros(NUM_TOTAL_RAYS, dtype=np.float64)
        self._dog_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "dog_base")
        self._ranges = np.full((NUM_LAYERS, NUM_H_RAYS), -1.0)

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
        self._lidar_site = sid
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
        visual_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.pub_pc_visual = self.create_publisher(
            PointCloud2, "pointcloud_visual", visual_qos)
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
        self._avoider = LocalAvoidance(AvoidanceConfig(
            range_min=RANGE_MIN,
            range_max=RANGE_MAX,
            emergency_dist=OBSTACLE_DIST,
            slow_dist=OBSTACLE_SLOW_DIST,
            front_half_angle_deg=OBSTACLE_FRONT_AZ,
            max_turn_rate=PATROL_W,
            clear_speed=0.20,
        ))
        self._progress_wp = self.wp
        self._progress_best = float("inf")
        self._progress_t = self.data.time
        self._recovery_attempts = 0
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

        # 先扫一帧，避免第一个 tick 的避障读到全 -1 的空矩阵而误判为"无障碍"
        self._scan()

        # 预发 clock 让其他 use_sim_time 节点先同步仿真时间，避免 TF_OLD_DATA
        for _ in range(10):
            ck = Clock()
            ck.clock.sec, ck.clock.nanosec = self._stamp()
            self.pub_clock.publish(ck)

        self.create_timer(1.0 / ODOM_HZ, self.tick)
        self.get_logger().info(
            "3D bridge up: lidar3d at base_link (%.3f, %.3f, %.3f), "
            "%d layers x %d rays = %d total, patrol=%s, odom_noise=%s"
            % (self.lidar_xyz + (NUM_LAYERS, NUM_H_RAYS, NUM_TOTAL_RAYS,
                                 patrol, self.odom_noise)))

    def _build_direction_table(self):
        """预计算每根射线在雷达局部系中的单位方向向量。"""
        dirs = np.zeros((NUM_LAYERS, NUM_H_RAYS, 3), dtype=np.float32)
        for layer in range(NUM_LAYERS):
            phi = math.radians(float(VERTICAL_ANGLES_DEG[layer]))
            cp, sp = math.cos(phi), math.sin(phi)
            for az in range(NUM_H_RAYS):
                theta = -math.pi + 2 * math.pi * az / NUM_H_RAYS
                dirs[layer, az] = [cp * math.cos(theta),
                                   cp * math.sin(theta), sp]
        self._dirs = dirs                      # (NUM_LAYERS, NUM_H_RAYS, 3)
        self._dirs_flat = dirs.reshape(-1, 3).astype(np.float64)

    def _scan(self):
        """用 mj_multiRay 投射全部射线，返回 (NUM_LAYERS, NUM_H_RAYS) 距离矩阵。

        未命中或超出 RANGE_MAX 的射线置 -1（与原 rangefinder 的约定一致）。
        """
        d = self.data
        origin = d.site_xpos[self._lidar_site].copy()
        yaw = self._base_yaw_world()
        c, s = math.cos(yaw), math.sin(yaw)
        world = np.empty((NUM_TOTAL_RAYS, 3))
        world[:, 0] = c * self._dirs_flat[:, 0] - s * self._dirs_flat[:, 1]
        world[:, 1] = s * self._dirs_flat[:, 0] + c * self._dirs_flat[:, 1]
        world[:, 2] = self._dirs_flat[:, 2]

        mujoco.mj_multiRay(
            self.model, d, origin, world.flatten(),
            None,               # geomgroup: 全部组
            1,                  # flg_static: 包含静态几何体
            self._dog_body,     # 排除狗自身，避免自打击
            self._ray_geomid, self._ray_dist,
            NUM_TOTAL_RAYS, RANGE_MAX)

        # cutoff 只用于加速剪枝，返回值仍可能 > cutoff；未命中返回 -1。
        r = self._ray_dist.reshape(NUM_LAYERS, NUM_H_RAYS)
        np.copyto(self._ranges, r)
        self._ranges[(r < 0) | (r > RANGE_MAX)] = -1.0
        return self._ranges

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
        if self.odom_noise:
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
        """把 self._ranges（由 tick 刷新）打包成 PointCloud2 发布。"""
        s, ns = self._stamp()
        ranges = self._ranges

        # 无效射线为 -1；必须在加噪声前判定，否则噪声会把 -1 抬进有效区间。
        valid = (ranges > RANGE_MIN) & (ranges < RANGE_MAX)

        # 加噪声（只对有效点）
        if SCAN_NOISE_STD > 0:
            noise = self.rng.randn(NUM_LAYERS, NUM_H_RAYS) * SCAN_NOISE_STD
            ranges = ranges + noise * valid.astype(float)

        # 距离 × 方向 = XYZ（雷达局部系）
        pts = (ranges[:, :, np.newaxis] * self._dirs)  # (layers, rays, 3)
        pts_valid = pts[valid]  # (N, 3)

        # 高度裁剪：把世界系阈值换算到雷达系（雷达装在 z=lidar_z 处，无俯仰）。
        # 保留地板点——rtabmap 需要它们来标记空闲栅格，见 Z_MIN_WORLD 注释。
        lidar_z = self.lidar_xyz[2]
        z = pts_valid[:, 2]
        pts_valid = pts_valid[(z > Z_MIN_WORLD - lidar_z)
                              & (z < Z_MAX_WORLD - lidar_z)]

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

        # 建图输入保留精确采样时间；RViz 实时层只显示最新一帧。零时间戳
        # 让 RViz 使用最新完整 TF，避免低频 map->odom 更新挤满其 10 帧
        # MessageFilter 队列。原始 /pointcloud 不受影响。
        self.pub_pc.publish(msg)
        visual = PointCloud2()
        visual.header.frame_id = msg.header.frame_id
        visual.height = msg.height
        visual.width = msg.width
        visual.fields = msg.fields
        visual.is_bigendian = msg.is_bigendian
        visual.point_step = msg.point_step
        visual.row_step = msg.row_step
        visual.data = msg.data
        visual.is_dense = msg.is_dense
        self.pub_pc_visual.publish(visual)

    # ---------------- 控制 ----------------
    def on_cmd_vel(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.patrol = False
        self._last_cmd_t = self.data.time

    def _auto_save_map(self):
        """巡视完成后自动保存地图（异步执行，不阻塞仿真）。"""
        if getattr(self, "_map_saved", False):
            return
        self._map_saved = True
        self.get_logger().info("开始自动保存地图 ...")

        def _save():
            script = os.path.join(PROJECT_ROOT, "slam", "save_map_3d.sh")
            if os.path.isfile(script):
                subprocess.run(["bash", script, "--scene", "lab"], cwd=PROJECT_ROOT)
                self.get_logger().info("地图已自动保存到 maps/lab/ 目录")
            else:
                self.get_logger().warn("找不到 save_map_3d.sh，请手动保存")

        threading.Thread(target=_save, daemon=True).start()

    def _front_obstacle_dist(self):
        """Return horizontal front clearance and obstacle-side indication."""
        scan = self._avoider.analyze(
            self._ranges, self._dirs, self.lidar_xyz[2])
        return scan.front, scan.left_score - scan.right_score

    def _reset_progress(self, dist=float("inf")):
        self._progress_wp = self.wp
        self._progress_best = dist
        self._progress_t = self.data.time
        self._recovery_attempts = 0

    def _patrol_cmd(self):
        if self.wp >= len(PATROL_WAYPOINTS):
            if not PATROL_LOOP:
                self._auto_save_map()
                return (0.0, 0.0, 0.0)
            self.wp = 1
            self.lap += 1
            self.get_logger().info("lap %d done, looping" % self.lap)
            if self.max_laps > 0 and self.lap >= self.max_laps:
                self.patrol = False
                self.cmd = (0.0, 0.0, 0.0)
                self.get_logger().info("巡视 %d 圈完成，自动保存地图 ..." % self.max_laps)
                self._auto_save_map()
                return (0.0, 0.0, 0.0)
        p = self._truth_pose()
        tx, ty = PATROL_WAYPOINTS[self.wp]
        dx, dy = tx - p[0], ty - p[1]
        dist = math.hypot(dx, dy)
        if dist < PATROL_TOL:
            self.wp += 1
            self._avoider.reset()
            self._reset_progress()
            self.get_logger().info("waypoint %d/%d reached"
                                   % (self.wp, len(PATROL_WAYPOINTS)))
            return (0.0, 0.0, 0.0)
        want = math.atan2(dy, dx)
        err = math.atan2(math.sin(want - p[2]), math.cos(want - p[2]))
        wz = max(-PATROL_W, min(PATROL_W, 1.8 * err))

        now = self.data.time
        scan = self._avoider.analyze(
            self._ranges, self._dirs, self.lidar_xyz[2])

        if self._progress_wp != self.wp:
            self._reset_progress(dist)
        elif dist < self._progress_best - PROGRESS_DIST:
            self._progress_best = dist
            self._progress_t = now
            self._recovery_attempts = 0

        force_recovery = False
        if (not self._avoider.active
                and now - self._progress_t >= STUCK_WINDOW):
            if self._recovery_attempts >= MAX_RECOVERY_ATTEMPTS:
                self.get_logger().warn(
                    "航点 %d(%.1f,%.1f) 连续 %d 次脱困后仍无进展，安全跳过"
                    % (self.wp, tx, ty, self._recovery_attempts))
                self.wp += 1
                self._avoider.reset()
                self._reset_progress()
                return (0.0, 0.0, 0.0)
            self._recovery_attempts += 1
            self._progress_t = now
            force_recovery = True
            self.get_logger().warn(
                "到航点距离 %.2fm 已有 %.0fs 未下降，启动脱困 %d/%d"
                % (dist, STUCK_WINDOW, self._recovery_attempts,
                   MAX_RECOVERY_ATTEMPTS))

        preferred = 1.0 if err >= 0.0 else -1.0
        previous_phase = self._avoider.phase
        recovery = self._avoider.recovery_command(
            now, scan, preferred=preferred, force=force_recovery)
        if self._avoider.phase != previous_phase:
            phase_text = {
                LocalAvoidance.IDLE: "恢复航点导航",
                LocalAvoidance.BACKUP: "后退拉开距离",
                LocalAvoidance.TURN: "锁定方向转向",
                LocalAvoidance.CLEAR: "沿新方向越过障碍",
            }[self._avoider.phase]
            self.get_logger().info(
                "[避障状态] %s，前方水平净距 %.2fm，绕行方向=%s"
                % (phase_text, scan.front,
                   "左" if self._avoider.turn_dir > 0.0 else "右"))
        if recovery is not None:
            return recovery

        caution = self._avoider.caution_command(
            now, scan, goal_error=err, cruise_speed=PATROL_V)
        if caution is not None:
            return caution

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
        if self._n % self._scan_every == 0:
            # 在 mj_step 之后扫描，保证点云与本帧发布的 TF/时间戳严格对应。
            self._scan()
            if not self.no_lidar:
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
    ap.add_argument("--odom-noise", action="store_true",
                    help="注入里程计随机游走（仅用于回环/漂移压力测试）")
    args = ap.parse_args()

    rclpy.init()
    node = SlamBridge3D(view=args.view, patrol=args.patrol, seed=args.seed,
                        no_lidar=args.no_lidar, max_laps=args.laps,
                        odom_noise=args.odom_noise)
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
