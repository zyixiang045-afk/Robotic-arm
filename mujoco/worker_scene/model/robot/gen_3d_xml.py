#!/usr/bin/env python3.8
"""从现有 scene_with_robot_py38.xml 生成 3D-only 雷达版本。

不依赖 MjSpec 构建 API（3.2.3 / 3.10 差异太大），直接用 XML 操作：
1. 删除所有 2D lidar site 和 sensor（名字前缀 lidar_s/lidar_r）
2. 注入 3D lidar sites 和 sensors（lidar3d_*）
3. 输出 scene_with_robot_3d_py38.xml

用法:
    python3.8 gen_3d_xml.py
    python3.8 gen_3d_xml.py --keep-2d   # 保留 2D 雷达（两种都有）
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
INPUT_XML = os.path.join(HERE, "scene_with_robot_py38.xml")
OUTPUT_XML = os.path.join(HERE, "scene_with_robot_3d_py38.xml")

# 3D 雷达参数（与 build_robot.py LIDAR3D_* 一致）
LIDAR3D_NAME = "lidar3d"
LIDAR3D_POS = "-0.454 0.0 0.95"
H_RAYS = 180
V_LAYERS = 16
V_MIN = math.radians(-15.0)
V_MAX = math.radians(15.0)
RANGE_MAX = 8.0


def quat_z(angle):
    """绕 z 轴旋转的四元数 [w,x,y,z]。"""
    return [math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)]


def quat_mul(a, b):
    """四元数乘法 [w,x,y,z]。"""
    w = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
    x = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2]
    y = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1]
    z = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    return [w, x, y, z]


def fmt_quat(q):
    return " ".join("%.8f" % v for v in q)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--keep-2d", action="store_true",
                    help="保留 2D 雷达（输出两种都有的模型）")
    args = ap.parse_args()

    tree = ET.parse(INPUT_XML)
    root = tree.getroot()

    # --- 删除 2D lidar（除非 --keep-2d）---
    if not args.keep_2d:
        # 删 site（在 dog_base body 下）
        for body in root.iter("body"):
            if body.get("name") == "dog_base":
                dog_body = body
                break
        else:
            raise RuntimeError("找不到 dog_base body")

        to_remove = []
        for site in dog_body.findall("site"):
            name = site.get("name", "")
            if name.startswith("lidar_s") or name == "lidar_frame":
                to_remove.append(site)
        for geom in dog_body.findall("geom"):
            if geom.get("name") == "lidar_case":
                to_remove.append(geom)
        for elem in to_remove:
            dog_body.remove(elem)

        # 删 sensor
        sensor_elem = root.find("sensor")
        if sensor_elem is not None:
            to_remove = []
            for sen in sensor_elem:
                name = sen.get("name", "")
                if name.startswith("lidar_r"):
                    to_remove.append(sen)
            for elem in to_remove:
                sensor_elem.remove(elem)

    # --- 注入 3D lidar ---
    # hub 朝向修正（狗 body 绕 z 转 180°）
    q_hub = quat_z(math.pi)

    # 外壳 geom
    case = ET.SubElement(dog_body, "geom")
    case.set("name", LIDAR3D_NAME + "_case")
    case.set("type", "cylinder")
    case.set("size", "0.05 0.04")
    case.set("pos", LIDAR3D_POS)
    case.set("rgba", "0.08 0.08 0.10 1")
    case.set("contype", "0")
    case.set("conaffinity", "0")
    case.set("group", "2")

    # 参考 site
    ref = ET.SubElement(dog_body, "site")
    ref.set("name", LIDAR3D_NAME + "_frame")
    ref.set("pos", LIDAR3D_POS)
    ref.set("quat", fmt_quat(q_hub))
    ref.set("size", "0.01 0.01 0.01")
    ref.set("rgba", "0 1 1 0")

    # 确保有 sensor 元素
    if sensor_elem is None:
        sensor_elem = ET.SubElement(root, "sensor")

    # 生成射线
    v_step = (V_MAX - V_MIN) / (V_LAYERS - 1) if V_LAYERS > 1 else 0.0

    def quat_y(a):
        return [math.cos(a / 2), 0.0, math.sin(a / 2), 0.0]

    for layer in range(V_LAYERS):
        phi = V_MIN + layer * v_step
        for az in range(H_RAYS):
            theta = -math.pi + 2 * math.pi * az / H_RAYS
            # Rz(θ) × Ry(90°-φ)
            q_elev = quat_y(math.pi / 2 - phi)
            q_az = quat_z(theta)
            q_local = quat_mul(q_az, q_elev)
            q_final = quat_mul(q_hub, q_local)

            sname = "%s_s%02d_%03d" % (LIDAR3D_NAME, layer, az)
            rname = "%s_r%02d_%03d" % (LIDAR3D_NAME, layer, az)

            s = ET.SubElement(dog_body, "site")
            s.set("name", sname)
            s.set("pos", LIDAR3D_POS)
            s.set("quat", fmt_quat(q_final))
            s.set("size", "0.001 0.001 0.001")
            s.set("rgba", "0 0 1 0")

            sen = ET.SubElement(sensor_elem, "rangefinder")
            sen.set("name", rname)
            sen.set("site", sname)
            sen.set("cutoff", str(RANGE_MAX))

    # 写出
    tree.write(OUTPUT_XML, encoding="unicode", xml_declaration=True)
    print("已写出 %s" % OUTPUT_XML)

    # 验证能加载
    m = mujoco.MjModel.from_xml_path(OUTPUT_XML)
    n3d = sum(1 for i in range(m.nsensor)
              if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, i) or "")
              .startswith("lidar3d_r"))
    n2d = sum(1 for i in range(m.nsensor)
              if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, i) or "")
              .startswith("lidar_r"))
    print("验证: nsensor=%d (3D=%d, 2D=%d), nbody=%d"
          % (m.nsensor, n3d, n2d, m.nbody))


if __name__ == "__main__":
    main()
