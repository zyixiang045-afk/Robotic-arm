#!/usr/bin/env python3.8
"""点对点导航节点：3D SLAM 地图 + TF 定位 + Lazy Theta* 规划 + 路径跟随。"""

# Mrs.yoki,Keep up the good work!

"""链路:
  /map(占据栅格) + TF(map->odom->base_footprint)  ->  Lazy Theta* 规划
  /nav_goal(目标点)  ->  触发规划
  Lazy Theta* 路径(any-angle)  ->  纯追踪跟随(全向底盘, 可侧移)  ->  /cmd_vel
  slam_bridge.py 收到外部 /cmd_vel 会自动停止巡视、交出控制权。

必须 python3.8 跑（同 slam_bridge.py，Foxy 的 rclpy 只有 cpython-38 扩展），
use_sim_time 必须 true（桥接发布 /clock）。

地图优先级: 优先订阅 slam_toolbox 在线 /map（最新，含动态障碍）；若一直收不到
则回退加载已保存的 maps/lab_map.pgm（静态，可离线测试）。

订阅:
  /map        nav_msgs/OccupancyGrid      占据栅格地图
  /nav_goal   geometry_msgs/PoseStamped   目标点 (map 系)
  TF           map -> odom -> base_footprint
发布:
  /cmd_vel    geometry_msgs/Twist
  /nav_path   nav_msgs/Path
  /nav_status std_msgs/String              状态: IDLE/PLANNING/FOLLOWING/ARRIVED/NO_MAP/NO_POSE/STUCK/UNREACHABLE

用法:
  # 终端1: 先跑建图(slam_toolbox) + 桥接，让狗巡视建图:
  ./slam/run_slam.sh --patrol
  # 终端2: 起导航节点（先 source /opt/ros/foxy/setup.bash）:
  python3.8 nav_p2p.py
  # 终端3: 发目标点（map 系）。狗起点是 map 原点，即世界(-8.5, 0)；
  #        世界系 -> map系 转换: (x_world+8.5, y_world)。
  #        例: 世界(-4, 1) == map(4.5, 1)
  ros2 topic pub -1 /nav_goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: 4.5, y: 1.0}, orientation: {w: 1.0}}}"
"""
import heapq
import math
import os

import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
SAVED_MAP = os.path.join(HERE, "maps", "lab_map_3d.pgm")
SAVED_YAML = os.path.join(HERE, "maps", "lab_map_3d.yaml")

# --- 规划/跟随参数 ---
ROBOT_RADIUS = 0.32          # 底盘碰撞盒半宽 0.315 + 余量
INFLATE = 0.05               # 额外膨胀余量(m)
GOAL_TOL = 0.22              # 到点判定半径(m)
LOOKAHEAD = 0.45             # 纯追踪前瞻距离(m)
V_MAX = 0.45                 # 最大线速度(m/s)
W_MAX = 0.9                  # 最大角速度(rad/s)
DEVIATE_REPLAN = 0.6         # 偏离路径多远重规划(m)
STUCK_TIMEOUT = 3.0          # 被堵判定时间(s)
STUCK_DIST = 0.03            # 该时间内移动不足(m) 视为被堵
MAX_PLAN_FAIL = 3            # 连续规划失败几次后报 UNREACHABLE

# --- 状态 ---
S_IDLE, S_PLANNING, S_FOLLOWING = "IDLE", "PLANNING", "FOLLOWING"
S_ARRIVED, S_NO_MAP, S_NO_POSE, S_STUCK, S_UNREACH = "ARRIVED", "NO_MAP", "NO_POSE", "STUCK", "UNREACHABLE"


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class NavP2P(Node):
    def __init__(self, use_saved=False):
        super().__init__("nav_p2p")
        self.set_parameters([Parameter("use_sim_time", value=True)])

        self._use_saved = use_saved
        self.map = None            # 当前 OccupancyGrid（在线或保存的）
        self.cost = None           # 膨胀后代价栅格(np.ndarray float, 0 可走/1 不可走)
        self.grid_shape = None
        self.path = []             # list of (x, y) map 系
        self.goal = None           # (x, y) map 系
        self.path_idx = 0
        self.status = S_IDLE
        self.plan_fail = 0
        self.last_pose = None
        self._stuck_t0 = None
        self._stuck_pos = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_path = self.create_publisher(Path, "nav_path", 10)
        self.pub_status = self.create_publisher(String, "nav_status", 10)

        # 用 best_effort + KEEP_LAST 订阅 /map 更稳：slam_toolbox 的 map 发布
        # 若带 volatile 也能收到。
        from rclpy.qos import (QoSProfile, QoSHistoryPolicy,
                               QoSReliabilityPolicy, QoSDurabilityPolicy)
        planning_map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_planning_map = self.create_publisher(
            OccupancyGrid, "planning_map", planning_map_qos)

        map_qos = QoSProfile(depth=2,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub_map = self.create_subscription(OccupancyGrid, "map",
                                                self.on_map, map_qos)
        self.sub_goal = self.create_subscription(PoseStamped, "nav_goal",
                                                 self.on_goal, 10)

        self.create_timer(0.02, self.tick)     # 50 Hz 跟随
        self._fallback_timer = self.create_timer(3.0, self._try_saved_map)

        self.get_logger().info(
            "nav_p2p up: robot_radius=%.2f inflate=%.2f lookahead=%.2f "
            "saved_map=%s" % (ROBOT_RADIUS, INFLATE, LOOKAHEAD,
                              os.path.basename(SAVED_MAP) if os.path.exists(SAVED_MAP) else "无"))

    # ---------------- 地图 ----------------
    def on_map(self, msg):
        if self._use_saved:
            return
        self.map = msg
        self._rebuild_cost()
        self._publish_planning_map()
        if self.goal is not None and self.status in (S_FOLLOWING, S_IDLE, S_STUCK, S_UNREACH):
            self._plan()

    def _try_saved_map(self):
        """收不到在线 /map 时回退到已保存地图（离线测试用）。"""
        if self.map is not None:
            self._fallback_timer.cancel()
            return
        if not os.path.exists(SAVED_MAP):
            return
        self.get_logger().warn("3s 内未收到 /map，回退加载 %s" % SAVED_MAP)
        self.map = self._load_saved_map()
        if self.map is not None:
            self._rebuild_cost()
            self._publish_planning_map()
            self._fallback_timer.cancel()
            if self.goal is not None:
                self._plan()

    def _load_saved_map(self):
        """解析 maps/lab_map.pgm + lab_map.yaml 成 OccupancyGrid。"""
        try:
            with open(SAVED_MAP, "rb") as f:
                magic = f.readline().strip()
                if magic != b"P5":
                    raise ValueError("不是 P5 PGM")
                while True:
                    line = f.readline()
                    if line.startswith(b"#"):
                        continue
                    parts = line.split()
                    if len(parts) >= 2 and len(parts) < 3:
                        w, h = int(parts[0]), int(parts[1])
                        continue
                    if len(parts) == 1:
                        break
                data = np.frombuffer(f.read(), dtype=np.uint8).astype(np.int16)
                if data.size != w * h:
                    raise ValueError("PGM 数据长度不符")
                img = data.reshape(h, w)
            # PGM 从上到下存储，OccupancyGrid 从下到上（origin 在左下角），需翻转
            img = np.flipud(img)
            res, origin = 0.05, [-2.97, -11.5, 0.0]
            if os.path.exists(SAVED_YAML):
                y = {}
                for ln in open(SAVED_YAML, encoding="utf-8"):
                    if ":" in ln and not ln.strip().startswith("#"):
                        k, v = ln.split(":", 1)
                        y[k.strip()] = v.strip()
                res = float(y.get("resolution", res))
                origin = [float(x) for x in
                          y.get("origin", "[-2.97, -11.5, 0]").strip("[]").split(",")]
            g = OccupancyGrid()
            g.header.frame_id = "map"
            g.header.stamp = self.get_clock().now().to_msg()
            g.info.resolution = res
            g.info.width = w
            g.info.height = h
            g.info.origin.position.x, g.info.origin.position.y = origin[0], origin[1]
            # 黑(0)=占据, 白(255)=空闲 → 换算成 0..100 占据概率
            occ = (255 - img) * 100 // 255
            g.data = [int(v) for v in occ.flatten()]
            self.get_logger().info(
                "加载保存地图 %dx%d res=%.3f origin=(%.2f, %.2f)"
                % (w, h, res, origin[0], origin[1]))
            return g
        except Exception as e:
            self.get_logger().error("加载保存地图失败: %s" % e)
            return None

    def _rebuild_cost(self):
        """占据栅格 -> EDT 距离场 -> 膨胀后代价栅格。"""
        m = self.map
        occ = np.asarray(m.data, dtype=np.float64).reshape(m.info.height, m.info.width)
        occ[occ < 0] = 0.0                      # unknown 当自由
        blocked = occ > 50.0
        self.grid_shape = (m.info.height, m.info.width)

        # scipy EDT：精确欧氏距离（网格单位），比 chamfer 快几百倍
        free = ~blocked
        dist = distance_transform_edt(free)      # 自由格到最近障碍的距离

        r_infl = (ROBOT_RADIUS + INFLATE) / m.info.resolution
        self.cost = np.where(dist < r_infl, 1.0, 0.0)

        # 房间四面墙：rtabmap 栅格常漏扫墙体，若不强制设障碍，
        # 规划器会画"穿墙直线"，狗撞墙后原地卡死。
        self._add_room_walls()

    def _add_room_walls(self):
        """把实验室四面墙(静物)写进代价栅格。坐标均为 map 系。"""
        if self.cost is None or self.map is None:
            return
        o = self.map.info.origin.position
        r = self.map.info.resolution
        h, w = self.grid_shape
        xs = o.x + (np.arange(w) + 0.5) * r     # 每列中心 x (map系)
        ys = o.y + (np.arange(h) + 0.5) * r     # 每行中心 y (map系)
        X, Y = np.meshgrid(xs, ys)
        d = np.minimum.reduce([
            np.abs(X - (-3.5)),     # 西墙 x=-3.5
            np.abs(X - 11.5),       # 东墙 x=11.5
            np.abs(Y - (-3.0)),     # 南墙 y=-3
            np.abs(Y - 3.0),        # 北墙 y=3
        ])
        self.cost[d < 0.15] = 1.0

    # ---------------- 目标与规划 ----------------
    def on_goal(self, msg):
        p = msg.pose.position
        self.goal = (p.x, p.y)
        self.plan_fail = 0
        self._stuck_pos, self._stuck_t0 = None, None
        self.get_logger().info("收到目标 (%.2f, %.2f), 开始规划" % self.goal)
        if self.map is None:
            self._set_status(S_NO_MAP)
            return
        self._plan()

    def _world_to_grid(self, x, y):
        o = self.map.info.origin.position
        r = self.map.info.resolution
        i = int(math.floor((x - o.x) / r))
        j = int(math.floor((y - o.y) / r))
        return i, j

    def _grid_to_world(self, i, j):
        o = self.map.info.origin.position
        r = self.map.info.resolution
        return o.x + (i + 0.5) * r, o.y + (j + 0.5) * r

    def _plan(self):
        if self.map is None or self.cost is None or self.goal is None:
            return
        pose = self._get_pose()
        if pose is None:
            self._set_status(S_NO_MAP)
            return
        self._set_status(S_PLANNING)
        sx, sy = self._world_to_grid(pose[0], pose[1])
        gx, gy = self._world_to_grid(self.goal[0], self.goal[1])
        h, w = self.grid_shape
        if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
            self.plan_fail += 1
            self.get_logger().warn("目标超出地图范围")
            self._check_plan_fail()
            return
        path = self._astar(sx, sy, gx, gy)
        if path is None:
            self.plan_fail += 1
            self.get_logger().warn("Lazy Theta* 规划失败 (起点或目标被障碍包围?)")
            self._check_plan_fail()
            return
        self.plan_fail = 0
        self.path = [self._grid_to_world(i, j) for i, j in path]
        self.path_idx = 0
        self.get_logger().info("规划成功: %d 个路径点" % len(self.path))
        self._set_status(S_FOLLOWING)
        self._publish_path()

    def _check_plan_fail(self):
        if self.plan_fail >= MAX_PLAN_FAIL:
            self._set_status(S_UNREACH)

    def _astar(self, sx, sy, gx, gy):
        """Lazy Theta* — any-angle path planning on inflated cost grid."""
        h, w = self.grid_shape
        if self.cost[sy, sx] > 0 or self.cost[gy, gx] > 0:
            return None

        SQRT2 = 1.4142135623730951
        # 8-connected neighbors
        DIRS = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2))

        start_key = sx * h + sy
        goal_key = gx * h + gy
        gcost = {start_key: 0.0}
        parent = {start_key: (sx, sy)}
        openq = [(math.hypot(sx - gx, sy - gy), sx, sy)]
        closed = set()

        while openq:
            _, cx, cy = heapq.heappop(openq)
            ckey = cx * h + cy
            if ckey in closed:
                continue

            # Lazy check: verify LOS to parent (修复: 原实现校验祖父点, 且父点为起点时
            # 跳过校验, 导致穿过障碍的"起点->终点"捷径永远不被发现)
            px, py = parent[ckey]
            if (px, py) != (cx, cy):
                if not self._line_of_sight(px, py, cx, cy):
                    # LOS failed — fallback: find best g among neighbors
                    best_g = 1e18
                    best_p = (px, py)
                    for dx, dy, cost in DIRS:
                        nx, ny = cx + dx, cy + dy
                        nkey = nx * h + ny
                        if nkey in closed and nkey in gcost:
                            d = math.hypot(cx - nx, cy - ny)
                            g_via = gcost[nkey] + d
                            if g_via < best_g:
                                best_g = g_via
                                best_p = (nx, ny)
                        parent[ckey] = best_p
                        gcost[ckey] = best_g

            if (cx, cy) == (gx, gy):
                closed.add(ckey)
                break
            closed.add(ckey)

            for dx, dy, step_cost in DIRS:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < w and 0 <= ny < h):
                    continue
                if self.cost[ny, nx] > 0:
                    continue
                nkey = nx * h + ny
                if nkey in closed:
                    continue

                # Try path through grandparent (Theta* update)
                px, py = parent[ckey]
                pkey = px * h + py
                g_pp = gcost[pkey] + math.hypot(nx - px, ny - py)
                g_c = gcost[ckey] + step_cost

                if g_pp < g_c:
                    # Lazy: assume LOS to grandparent passes
                    new_g = g_pp
                    new_parent = (px, py)
                else:
                    new_g = g_c
                    new_parent = (cx, cy)

                if new_g < gcost.get(nkey, 1e18):
                    gcost[nkey] = new_g
                    parent[nkey] = new_parent
                    hh = math.hypot(nx - gx, ny - gy)
                    heapq.heappush(openq, (new_g + hh, nx, ny))

        if goal_key not in parent:
            return None

        # Reconstruct path
        rev = []
        cur = (gx, gy)
        while cur != (sx, sy):
            rev.append(cur)
            ckey = cur[0] * h + cur[1]
            cur = parent[ckey]
        rev.append((sx, sy))
        return list(reversed(rev))

    def _line_of_sight(self, x0, y0, x1, y1):
        """Bresenham line check on inflated cost grid. Returns True if clear."""
        h, w = self.grid_shape
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        err = dx - dy
        cx, cy = x0, y0
        while True:
            if not (0 <= cx < w and 0 <= cy < h):
                return False
            if self.cost[cy, cx] > 0:
                return False
            if cx == x1 and cy == y1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy

    # ---------------- 位姿与跟随 ----------------
    def _get_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_footprint", Time(), Duration(seconds=1.0))
        except Exception:
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)
        return (x, y, yaw)

    def tick(self):
        if self.goal is None:
            return                          # 空闲不发指令，让巡视接管
        if self.map is None or self.cost is None or not self.path:
            self._set_status(S_NO_MAP)
            return
        pose = self._get_pose()
        if pose is None:
            self._set_status(S_NO_POSE)
            return

        # 到点判定
        gx, gy = self.path[-1]
        if math.hypot(gx - pose[0], gy - pose[1]) < GOAL_TOL:
            self._stop()
            self.get_logger().info("到达目标 (%.2f, %.2f)" % self.goal)
            self.goal = None
            self._set_status(S_ARRIVED)
            return

        # 被堵判定
        if self._stuck_pos is None:
            self._stuck_pos, self._stuck_t0 = pose, self.get_clock().now()
        else:
            moved = math.hypot(pose[0] - self._stuck_pos[0],
                               pose[1] - self._stuck_pos[1])
            dt = (self.get_clock().now() - self._stuck_t0).nanoseconds / 1e9
            if dt > STUCK_TIMEOUT and moved < STUCK_DIST:
                self._stuck_pos, self._stuck_t0 = pose, self.get_clock().now()
                self.get_logger().warn("疑似被堵, 重规划")
                self._set_status(S_STUCK)
                self._plan()
                if self.status == S_UNREACH:
                    return
            if moved > STUCK_DIST:
                self._stuck_pos, self._stuck_t0 = pose, self.get_clock().now()

        # 偏离重规划
        if self._path_deviation(pose) > DEVIATE_REPLAN:
            self.get_logger().warn("偏离路径 %.2f m, 重规划" % self._path_deviation(pose))
            self._plan()
            if self.status == S_UNREACH:
                return

        # 纯追踪：选前瞻点
        idx = self.path_idx
        best = idx
        for k in range(idx, len(self.path)):
            dx = self.path[k][0] - pose[0]
            dy = self.path[k][1] - pose[1]
            if math.hypot(dx, dy) <= LOOKAHEAD:
                best = k
            else:
                break
        # 修复: 当选中点就是狗自己脚下(距离≈0, 常见于2点直线路径+终点>LOOKAHEAD)
        # 时, 指向下一个路径点, 否则线速度为0、狗只原地转向导致"被堵"死循环
        if (best == idx and idx + 1 < len(self.path)
                and math.hypot(self.path[idx][0] - pose[0],
                               self.path[idx][1] - pose[1]) < 0.1):
            best = idx + 1
        self.path_idx = best
        tx, ty = self.path[best]

        # 到目标还差多少（用于减速）
        rem = math.hypot(self.path[-1][0] - pose[0], self.path[-1][1] - pose[1])

        # 全向底盘：直接把速度指向前瞻点（可侧移），航向单独回正
        dx, dy = tx - pose[0], ty - pose[1]
        dist = max(math.hypot(dx, dy), 1e-3)
        want = math.atan2(dy, dx)
        err = _wrap(want - pose[2])
        c, s = math.cos(pose[2]), math.sin(pose[2])
        uxb = (c * dx + s * dy) / dist
        uyb = (-s * dx + c * dy) / dist
        scale = V_MAX * min(1.0, rem / 0.4)
        wz = max(-W_MAX, min(W_MAX, 1.6 * err))

        tw = Twist()
        tw.linear.x = uxb * scale
        tw.linear.y = uyb * scale
        tw.angular.z = wz
        self.pub_cmd.publish(tw)
        self.last_pose = pose

    def _path_deviation(self, pose):
        if len(self.path) < 2:
            return 0.0
        best = 1e18
        for k in range(self.path_idx, min(self.path_idx + 5, len(self.path))):
            x1, y1 = self.path[k - 1]
            x2, y2 = self.path[k]
            vx, vy = x2 - x1, y2 - y1
            wx, wy = pose[0] - x1, pose[1] - y1
            t = (wx * vx + wy * vy) / max(vx * vx + vy * vy, 1e-9)
            t = max(0.0, min(1.0, t))
            px, py = x1 + t * vx, y1 + t * vy
            best = min(best, math.hypot(pose[0] - px, pose[1] - py))
        return best

    def _stop(self):
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = 0.0, 0.0, 0.0
        self.pub_cmd.publish(tw)

    # ---------------- 输出 ----------------
    def _set_status(self, s):
        if s != self.status:
            self.status = s
            msg = String()
            msg.data = s
            self.pub_status.publish(msg)
            self.get_logger().info("状态 -> %s" % s)

    def _publish_path(self):
        if self.map is None:
            return
        p = Path()
        p.header.frame_id = "map"
        for x, y in self.path:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.pub_path.publish(p)

    def _publish_planning_map(self):
        if self.map is not None:
            self.pub_planning_map.publish(self.map)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--use-saved", action="store_true",
                    help="只用保存的 lab_map.pgm，不订阅在线 /map")
    args = ap.parse_args()

    rclpy.init()
    node = NavP2P(use_saved=args.use_saved)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
