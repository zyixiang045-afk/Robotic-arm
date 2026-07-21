#!/usr/bin/env python3
"""把「四足机械狗 + 双机械臂(RM65-6F) + 双灵巧手(dexhand021)」组装成一个 MuJoCo 机器人模型。

结构（父 → 子）:
    dog_base (固定, 无自由度)
      ├─ arm_l/base_link → link_1..6   (左臂, 6 DOF)  → l_hand/right_hand_base → 20 关节
      └─ arm_r/base_link → link_1..6   (右臂, 6 DOF)  → r_hand/left_hand_base  → 20 关节

要点:
  - 狗 STL 单位是 mm，缩放到 m；平移使足底贴 z=0、脚印中心对齐 body 原点。
  - 狗视觉用原始高精 STL；碰撞用一个简化长方体（避免 390 万面拖垮仿真）。
  - 两臂在狗背顶面(约 1.46 m)左右对称肩装，base 竖直朝上。
  - 用 MjSpec.attach 把各 URDF 子模型按前缀合并进主模型，避免手写坐标错配。
  - 每个转动关节配 position 执行器，便于后续控制。

用法:
    python3 build_robot.py            # 写出 robot.xml 并校验编译
    python3 build_robot.py --view     # 组装后打开查看器
"""
import argparse
import os
import re

import math

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))


def quat_x(angle):
    """绕 x 轴旋转 angle(rad) 的四元数 [w,x,y,z]。"""
    return [math.cos(angle / 2), math.sin(angle / 2), 0.0, 0.0]


def quat_z(angle):
    """绕 z 轴旋转 angle(rad) 的四元数 [w,x,y,z]。"""
    return [math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)]
MESHES = "/home/nightflower/meshes"

# --- 狗几何参数（由 STL 包围盒算出，单位换算见下）-------------------------
DOG_SCALE = 0.001                      # mm -> m
# 原始 STL 包围盒(mm): min(532.97,28.81,21.17) max(1674.38,2909.43,1484.58)
DOG_BB_MIN = (532.9708, 28.81336, 21.168306)
DOG_BB_MAX = (1674.3787, 2909.4346, 1484.5803)


def _dog_mesh_offset():
    """让脚印中心对齐 body 原点、足底贴 z=0 的网格平移量(m)。"""
    cx = (DOG_BB_MIN[0] + DOG_BB_MAX[0]) / 2 * DOG_SCALE
    cy = (DOG_BB_MIN[1] + DOG_BB_MAX[1]) / 2 * DOG_SCALE
    tz = -DOG_BB_MIN[2] * DOG_SCALE
    return (-cx, -cy, tz)


def _dog_size():
    return tuple((DOG_BB_MAX[i] - DOG_BB_MIN[i]) * DOG_SCALE for i in range(3))


# ---------------------------------------------------------------------------
# URDF -> MjSpec：修正 mesh 路径后加载
# ---------------------------------------------------------------------------
def load_arm_spec():
    """RM65-6F 臂。原 URDF 无 <mujoco> 标签且用 package:// 引用，需注入 meshdir。"""
    src = os.path.join(MESHES, "RM65-6F/urdf/RM65-6F.urdf")
    txt = open(src, encoding="utf-8").read()
    txt = txt.replace("package://RM65-6F/meshes/", "")
    meshdir = os.path.join(HERE, "assets", "arm") + "/"
    inject = (f'<robot name="RM65-6F">\n'
              f'  <mujoco><compiler meshdir="{meshdir}" '
              f'balanceinertia="true" discardvisual="false"/></mujoco>')
    txt = re.sub(r'<robot\s+name="RM65-6F">', inject, txt, count=1)
    return mujoco.MjSpec.from_string(txt)


def load_hand_spec(side):
    """dexhand021 手。原 meshdir 指向不存在的目录，改指 assets/hand。
    side='right' → 右手模型；side='left' → 左手模型。"""
    src = os.path.join(MESHES, f"dexhand021 urdf/dexhand021_{side}_simplified.urdf")
    txt = open(src, encoding="utf-8").read()
    meshdir = os.path.join(HERE, "assets", "hand") + "/"
    txt = re.sub(r'meshdir="[^"]*"', f'meshdir="{meshdir}"', txt)
    txt = txt.replace("../meshes/dexhand021_simplified/", "")
    return mujoco.MjSpec.from_string(txt)


# ---------------------------------------------------------------------------
# 主装配
# ---------------------------------------------------------------------------
def build(spec=None):
    """组装机器人。spec 为 None 时新建独立模型；否则把机器人装入已有场景 spec。"""
    standalone = spec is None
    if standalone:
        spec = mujoco.MjSpec()
        spec.modelname = "dog_dual_arm"
    spec.compiler.degree = False          # 用弧度，和 URDF 一致

    # 狗视觉网格作为 asset：原 STL 有 389 万面，超 MuJoCo 单网格 20 万面上限，
    # 已减面并按 <20 万面切成多块（见 decimate 步骤），逐块作为独立 mesh。
    import glob
    dog_parts = sorted(glob.glob(os.path.join(HERE, "assets", "dog", "dog_visual_*.STL")))
    if not dog_parts:
        raise FileNotFoundError("缺少减面后的狗视觉网格 assets/dog/dog_visual_*.STL")
    for i, fp in enumerate(dog_parts):
        m = spec.add_mesh()
        m.name = f"dog_mesh_{i}"
        m.file = fp
        m.scale = [DOG_SCALE, DOG_SCALE, DOG_SCALE]

    mat = spec.add_material()
    mat.name = "dog_mat"
    mat.rgba = [0.35, 0.37, 0.40, 1.0]

    # --- 狗本体：固定 body（无自由度）---
    dog = spec.worldbody.add_body()
    dog.name = "dog_base"
    # 站到走廊起点、工人附近。狗 STL 长轴是本体局部 Y(2.88m=前后身长)，
    # 绕 z 转 -90° 使身长轴对齐世界 +x（面向走廊/主工作台方向）。
    dog.pos = [-8.5, 0.0, 0.0]
    dog.quat = quat_z(-math.pi / 2)

    ox, oy, oz = _dog_mesh_offset()
    dsx, dsy, dsz = _dog_size()

    # 视觉：减面后的多块网格（不参与碰撞）
    for i in range(len(dog_parts)):
        gv = dog.add_geom()
        gv.name = f"dog_visual_{i}"
        gv.type = mujoco.mjtGeom.mjGEOM_MESH
        gv.meshname = f"dog_mesh_{i}"
        gv.pos = [ox, oy, oz]
        gv.material = "dog_mat"
        gv.contype = 0
        gv.conaffinity = 0
        gv.group = 2

    # 碰撞：简化长方体（贴合狗身，足底 z=0 → 顶面 dsz）
    gc = dog.add_geom()
    gc.name = "dog_collision"
    gc.type = mujoco.mjtGeom.mjGEOM_BOX
    gc.size = [dsx / 2, dsy / 2, dsz / 2]
    gc.pos = [0, 0, dsz / 2]
    gc.rgba = [0.4, 0.4, 0.45, 0.0]       # 透明，仅用于碰撞
    gc.group = 3
    gc.contype = 2                        # 机器人碰撞组：只与环境(conaffinity 含 bit0)相碰
    gc.conaffinity = 1

    # --- 肩部安装点（均在狗 body 局部坐标系，数值来自减面网格实测 AABB）---
    # 狗 body 绕 z 转 -90°：狗局部 X → 世界 -y，狗局部 Y → 世界 +x。
    # 狗正朝向是世界 +y，故“左右”= 世界 x = 狗局部 Y。双臂应沿【局部 Y】左右对称分开，
    # 使两臂构成的平面为世界 xOz；前伸方向 reach 对正世界 +y(狗正面)。
    # 实测背部上表面(狗局部)：X∈[-0.47,0.47], Y∈[-0.29,0.29], 顶面 z≈1.35~1.46。
    shoulder_z = 1.30                     # 贴合背部上表面
    shoulder_sep = 0.17                   # 沿局部 Y 左右各偏(→世界 x 方向左右分开)
    shoulder_ctr = 0.08                   # 局部 X 居中(背部前后中央)
    ARM_TILT = 0.0                        # 肩部外倾角(rad)，0=竖直朝上
    SHOULDER_YAW = 0.0                    # reach 保持 +y(狗正面)

    def mount_arm(prefix, sep_local_y, hand_spec, hand_prefix, tilt_sign):
        arm = load_arm_spec()            # 每臂新建，attach 会消费子 spec
        fr = dog.add_frame()
        fr.pos = [shoulder_ctr, sep_local_y, shoulder_z]
        fr.quat = quat_z(SHOULDER_YAW)
        spec.attach(arm, prefix=prefix + "/", frame=fr)
        # 臂末端 link_6 接手
        tip = spec.body(prefix + "/link_6")
        hf = tip.add_frame()
        hf.pos = [0, 0, 0.0]
        hf.quat = [1, 0, 0, 0]
        spec.attach(hand_spec, prefix=hand_prefix + "/", frame=hf)

    mount_arm("arm_l", +shoulder_sep, load_hand_spec("right"), "hand_l", +1)
    mount_arm("arm_r", -shoulder_sep, load_hand_spec("left"), "hand_r", -1)

    # attach 后，子模型 mesh 的 file 只剩相对名（顶层 compiler 无对应 meshdir），
    # 逐个解析成绝对路径：臂网格在 assets/arm，手网格在 assets/hand。
    arm_dir = os.path.join(HERE, "assets", "arm")
    hand_dir = os.path.join(HERE, "assets", "hand")
    for m in spec.meshes:
        if os.path.isabs(m.file):
            continue                      # 狗网格已是绝对路径
        fn = os.path.basename(m.file)
        if os.path.exists(os.path.join(arm_dir, fn)):
            m.file = os.path.join(arm_dir, fn)
        elif os.path.exists(os.path.join(hand_dir, fn)):
            m.file = os.path.join(hand_dir, fn)
        else:
            raise FileNotFoundError(f"找不到 mesh: {fn}")

    # --- 执行器：仅给机器人（臂/手）关节配 position 控制，不碰场景里已有关节 ---
    robot_prefixes = ("arm_l/", "arm_r/", "hand_l/", "hand_r/")
    for j in spec.joints:
        if not j.name.startswith(robot_prefixes):
            continue
        if j.type in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            a = spec.add_actuator()
            a.name = "act_" + j.name
            a.set_to_position(kp=40.0, kv=4.0)
            a.trntype = mujoco.mjtTrn.mjTRN_JOINT
            a.target = j.name

    # --- 碰撞分组：臂/手所有 geom 设为“机器人组”，彼此不自碰，只与环境相撞 ---
    # contype=2, conaffinity=1 → 机器人 geom 之间(2&2 无 bit0)不接触；
    # 与环境默认 geom(contype=1,conaffinity=1)：2&1? 机器人conaff=1 含 bit0，环境contype=1 → 相撞。
    robot_body_prefixes = ("arm_l/", "arm_r/", "hand_l/", "hand_r/")
    for g in spec.geoms:
        b = g.parent
        bn = b.name if b is not None else ""
        if bn.startswith(robot_body_prefixes):
            g.contype = 2
            g.conaffinity = 1

    # --- 前伸工作预备姿态：双臂低头前伸、肘部小幅弯曲 ---
    # RM65 六关节：joint_2=肩俯仰, joint_3=肘, joint_5=腕俯仰（其余保持 0）。
    # base 竖直朝上，joint_2 转 ~1.2rad 使大臂前倒、joint_3/5 补偿让手朝前下方。
    arm_pose = {"joint_1": 0.0, "joint_2": 1.2, "joint_3": 0.6,
                "joint_4": 0.0, "joint_5": 0.9, "joint_6": 0.0}
    _add_ready_keyframe(spec, arm_pose)

    return spec


def _add_ready_keyframe(spec, arm_pose):
    """按 arm_pose 给两臂关节设初值，写成 keyframe，并让执行器 ctrl 保持该姿态。"""
    # 先编译一份拿到关节/执行器地址布局
    model = spec.compile()
    qpos = model.qpos0.copy()
    ctrl = None
    import numpy as np
    ctrl = np.zeros(model.nu)
    for side in ("arm_l", "arm_r"):
        for jn, val in arm_pose.items():
            full = f"{side}/{jn}"
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, full)
            if jid < 0:
                continue
            qadr = model.jnt_qposadr[jid]
            qpos[qadr] = val
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_" + full)
            if aid >= 0:
                ctrl[aid] = val
    key = spec.add_key()
    key.name = "ready"
    key.qpos = qpos.tolist()
    key.ctrl = ctrl.tolist()


def build_into_scene(scene_xml):
    """把机器人装进已有场景 XML，返回合并后的 spec。"""
    spec = mujoco.MjSpec.from_file(scene_xml)
    return build(spec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "robot.xml"))
    ap.add_argument("--scene", default=None,
                    help="已有场景 XML；给出则把机器人装入该场景后输出")
    ap.add_argument("--view", action="store_true")
    args = ap.parse_args()

    if args.scene:
        spec = build_into_scene(args.scene)
    else:
        spec = build()
    model = spec.compile()
    print(f"组装成功: nbody={model.nbody}, njnt={model.njnt}, "
          f"nu={model.nu}, ngeom={model.ngeom}, nmesh={model.nmesh}")

    xml = spec.to_xml()
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(xml)
    print(f"已写出 {args.out}")

    if args.view:
        import mujoco.viewer
        data = mujoco.MjData(model)
        mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
