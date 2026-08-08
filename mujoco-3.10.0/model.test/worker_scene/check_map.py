#!/usr/bin/env python3.8
"""检查当前 /map 建图进度：打印地图尺寸、占据/自由/未知格子比例。

用法:
  python3.8 check_map.py            # 采样 5 秒
  python3.8 check_map.py 15         # 采样 15 秒

未知比例越小说明地图越完整；巡视结束后再跑一次对比即可。
"""
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    node = rclpy.create_node("map_check")
    qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                     reliability=QoSReliabilityPolicy.BEST_EFFORT)
    got = []

    def cb(m):
        arr = np.asarray(m.data, dtype=np.int8)
        got.append((m.info.width, m.info.height,
                    int((arr > 50).sum()), int((arr == 0).sum()), int((arr < 0).sum())))

    node.create_subscription(OccupancyGrid, "map", cb, qos)
    t0 = time.time()
    while time.time() - t0 < secs and not got:
        rclpy.spin_once(node, timeout_sec=0.1)

    if got:
        w, h, occ, free, unk = got[-1]
        total = w * h
        print("地图 %dx%d (%.2fm x %.2fm)  占据=%d(%.0f%%) 自由=%d(%.0f%%) 未知=%d(%.0f%%)"
              % (w, h, w * 0.05, h * 0.05,
                 occ, occ / total * 100, free, free / total * 100, unk, unk / total * 100))
        if unk / total > 0.3:
            print(">> 未知区还很多，建图未完成，等狗多巡视几圈")
        else:
            print(">> 未知区很少，地图基本建完，可以保存了")
    else:
        print("%ds 内没收到 /map（建图可能没在跑）" % secs)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
