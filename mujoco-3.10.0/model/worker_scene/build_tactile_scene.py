#!/usr/bin/env python3
"""Build the tactile SmolVLA tabletop scene.

This scene matches the SmolVLA DexHand Tactile checkpoint shape more closely:
- one RM65 arm
- one dexhand021 right hand
- one fixed overhead camera
- one wrist camera on the hand base
- 20 fingertip touch sensors
"""

from __future__ import annotations

import argparse
import importlib.util
import math
from copy import deepcopy
from pathlib import Path
from xml.dom import minidom
from xml.etree import ElementTree as ET

import mujoco

HERE = Path(__file__).resolve().parent
ASSET_DIR = HERE / "assets"
ARM_DIR = ASSET_DIR / "arm"
HAND_DIR = ASSET_DIR / "hand"
URDF_DIR = ASSET_DIR / "urdf"
BUILD_ROBOT_PATH = HERE / "build_robot.py"

ROOM_X = (0.0, 3.0)
ROOM_Y = (0.0, 3.0)
WALL_H = 2.8
WALL_T = 0.10

TABLE_XY = (1.75, 1.75)
TABLE_TOP_Z = 0.70
TABLE_HALF = (0.55, 0.40, 0.02)
TABLE_LEG_HALF = 0.03

ARM_MOUNT_XYZ = (1.45, 1.90, 0.59)
ARM_MOUNT_QUAT = (0.0, 0.0, 0.0, 1.0)

TOP_CAMERA_POS = (1.75, 1.75, 2.45)
TOP_CAMERA_TARGET = (1.75, 1.75, 0.98)
TOP_CAMERA_FOVY = 50.0
TOP_CAMERA_RES = (256, 256)

ROBOT_PREFIX = "arm"
HAND_PREFIX = "hand"
WRIST_CAMERA_NAME = "wrist_image"
TOP_CAMERA_NAME = "image"

STATE_JOINTS = (
    "arm/joint_1",
    "arm/joint_2",
    "arm/joint_3",
    "arm/joint_4",
    "arm/joint_5",
    "arm/joint_6",
    "hand/r_f_joint1_1",
    "hand/r_f_joint1_2",
    "hand/r_f_joint1_3",
    "hand/r_f_joint1_4",
    "hand/r_f_joint2_1",
    "hand/r_f_joint2_2",
    "hand/r_f_joint2_3",
    "hand/r_f_joint2_4",
    "hand/r_f_joint3_1",
    "hand/r_f_joint3_2",
    "hand/r_f_joint3_3",
    "hand/r_f_joint3_4",
    "hand/r_f_joint4_1",
    "hand/r_f_joint4_2",
    "hand/r_f_joint4_3",
    "hand/r_f_joint4_4",
)

TACTILE_SENSOR_NAMES = tuple(f"tactile_{i:02d}" for i in range(20))

APPLE_RADIUS = 0.036
APPLE_MASS = 0.08


def _fmt(values) -> str:
    return " ".join(f"{float(v):.10g}" for v in values)


def _lookat_quat(pos, target, up=(0.0, 0.0, 1.0)) -> tuple[float, float, float, float]:
    import numpy as np

    pos = np.asarray(pos, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    up = np.asarray(up, dtype=np.float64)
    forward = target - pos
    forward = forward / max(np.linalg.norm(forward), 1e-9)
    z_axis = -forward
    if np.linalg.norm(np.cross(up, z_axis)) < 1e-6:
        up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    x_axis = np.cross(up, z_axis)
    x_axis = x_axis / max(np.linalg.norm(x_axis), 1e-9)
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / max(np.linalg.norm(y_axis), 1e-9)
    rot = np.column_stack([x_axis, y_axis, z_axis]).astype(np.float64)
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, rot.flatten())
    return tuple(float(v) for v in quat)


def _append(parent: ET.Element, tag: str, **attrs) -> ET.Element:
    clean = {key: str(value) for key, value in attrs.items() if value is not None}
    return ET.SubElement(parent, tag, clean)


def _load_robot_helpers():
    spec = importlib.util.spec_from_file_location("build_robot", BUILD_ROBOT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load {BUILD_ROBOT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _build_robot_root() -> ET.Element:
    robot_mod = _load_robot_helpers()

    arm_spec = robot_mod.load_arm_spec()
    hand_spec = robot_mod.load_hand_spec("right")

    robot = mujoco.MjSpec()
    robot.modelname = "smolvla_tactile_robot"
    robot.compiler.degree = False

    base = robot.worldbody.add_body()
    base.name = "robot_mount"
    base.pos = list(ARM_MOUNT_XYZ)
    base.quat = list(ARM_MOUNT_QUAT)

    arm_frame = base.add_frame()
    arm_frame.pos = [0, 0, 0]
    arm_frame.quat = [1, 0, 0, 0]
    robot.attach(arm_spec, prefix=f"{ROBOT_PREFIX}/", frame=arm_frame)

    arm_tip = robot.body(f"{ROBOT_PREFIX}/link_6")
    hand_frame = arm_tip.add_frame()
    hand_frame.pos = [0, 0, 0]
    hand_frame.quat = [1, 0, 0, 0]
    robot.attach(hand_spec, prefix=f"{HAND_PREFIX}/", frame=hand_frame)

    robot.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    for joint in robot.joints:
        if joint.name.startswith(f"{HAND_PREFIX}/"):
            joint.armature = 0.002

    for actuator in robot.actuators:
        if actuator.name.startswith(f"act_{ROBOT_PREFIX}/"):
            bias = list(actuator.biasprm)
            bias[2] = -15.0
            actuator.biasprm = bias

    for body in robot.bodies:
        if body.name.startswith((f"{ROBOT_PREFIX}/", f"{HAND_PREFIX}/")):
            body.gravcomp = 1.0

    wrist = robot.body(f"{HAND_PREFIX}/right_hand_base")
    cam = wrist.add_camera()
    cam.name = WRIST_CAMERA_NAME
    cam.pos = list(robot_mod.CAM_POS)
    cam.quat = list(robot_mod._cam_local_quat())
    cam.fovy = robot_mod.CAM_FOVY
    cam.resolution = list(robot_mod.CAM_RES)

    active_joint_names = STATE_JOINTS
    for joint_name in active_joint_names:
        actuator = robot.add_actuator()
        actuator.name = f"act_{joint_name}"
        actuator.set_to_position(kp=40.0, kv=4.0)
        actuator.trntype = mujoco.mjtTrn.mjTRN_JOINT
        actuator.target = joint_name

    pad_bodies = [
        f"{HAND_PREFIX}/r_f_link1_pad",
        f"{HAND_PREFIX}/r_f_link2_pad",
        f"{HAND_PREFIX}/r_f_link3_pad",
        f"{HAND_PREFIX}/r_f_link4_pad",
        f"{HAND_PREFIX}/r_f_link5_pad",
    ]
    pad_offsets = (
        (-0.004, -0.003, 0.0),
        (-0.004, 0.003, 0.0),
        (0.004, -0.003, 0.0),
        (0.004, 0.003, 0.0),
    )
    sensor_index = 0
    for pad_body in pad_bodies:
        body = robot.body(pad_body)
        for offset in pad_offsets:
            site = body.add_site()
            site.name = f"tactile_site_{sensor_index:02d}"
            site.pos = list(offset)
            site.size = [0.003, 0.003, 0.003]
            site.rgba = [0.0, 0.0, 0.0, 0.0]
            sensor = robot.add_sensor()
            sensor.name = TACTILE_SENSOR_NAMES[sensor_index]
            sensor.type = mujoco.mjtSensor.mjSENS_TOUCH
            sensor.objtype = mujoco.mjtObj.mjOBJ_SITE
            sensor.objname = site.name
            sensor.intprm = [1, 0, 0]
            sensor_index += 1

    xml = robot.to_xml()
    return ET.fromstring(xml)


def _prepare_assets(robot_root: ET.Element) -> list[ET.Element]:
    assets = []
    for mesh in robot_root.findall("./asset/mesh"):
        copied = deepcopy(mesh)
        file_name = Path(copied.attrib["file"]).name
        if (ARM_DIR / file_name).exists():
            copied.set("file", f"assets/arm/{file_name}")
        else:
            copied.set("file", f"assets/hand/{file_name}")
        assets.append(copied)
    return assets


def _add_camera_rig(world: ET.Element, name: str, pos, target, fovy: float, res: tuple[int, int]) -> None:
    rig = _append(world, "body", name=f"{name}_rig", pos=_fmt(pos), quat=_fmt(_lookat_quat(pos, target)))
    _append(rig, "camera", name=name, pos="0 0 0", quat="1 0 0 0", fovy=fovy, resolution=_fmt(res))


def _add_world(root: ET.Element, robot_body_elems: list[ET.Element]) -> None:
    world = _append(root, "worldbody")
    _append(world, "geom", name="floor", type="plane", size="0 0 0.05", material="grid", condim="3")
    _append(world, "light", name="top1", pos="0.5 0.8 3.0", dir="0 0 -1", diffuse="0.7 0.7 0.7")
    _append(world, "light", name="top2", pos="2.5 2.2 3.0", dir="0 0 -1", diffuse="0.55 0.55 0.55")
    _append(world, "light", name="table_fill", pos="1.7 1.8 2.1", dir="0.3 0.3 -1", diffuse="0.25 0.25 0.25")

    _add_camera_rig(world, TOP_CAMERA_NAME, TOP_CAMERA_POS, TOP_CAMERA_TARGET, TOP_CAMERA_FOVY, TOP_CAMERA_RES)

    x0, x1 = ROOM_X
    y0, y1 = ROOM_Y
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    hx, hy = (x1 - x0) / 2.0, (y1 - y0) / 2.0
    z = WALL_H / 2.0
    for name, pos, size in (
        ("wall_S", (cx, y0, z), (hx, WALL_T, z)),
        ("wall_N", (cx, y1, z), (hx, WALL_T, z)),
        ("wall_W", (x0, cy, z), (WALL_T, hy, z)),
        ("wall_E", (x1, cy, z), (WALL_T, hy, z)),
    ):
        _append(world, "geom", name=name, type="box", pos=_fmt(pos), size=_fmt(size), material="wall_mat")

    _add_table(world)
    _add_props(world)
    for body in robot_body_elems:
        world.append(body)


def _add_table(world: ET.Element) -> None:
    table = _append(world, "body", name="table", pos=_fmt((1.75, 1.75, 0.0)))
    _append(
        table,
        "geom",
        name="table_top",
        type="box",
        pos=_fmt((0.0, 0.0, TABLE_TOP_Z - TABLE_HALF[2])),
        size=_fmt(TABLE_HALF),
        material="table_mat",
    )
    leg_z = (TABLE_TOP_Z - 2.0 * TABLE_HALF[2]) / 2.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            _append(
                table,
                "geom",
                name=f"table_leg_{sx}_{sy}",
                type="box",
                pos=_fmt((sx * (TABLE_HALF[0] - TABLE_LEG_HALF - 0.02), sy * (TABLE_HALF[1] - TABLE_LEG_HALF - 0.02), leg_z)),
                size=_fmt((TABLE_LEG_HALF, TABLE_LEG_HALF, leg_z)),
                material="table_mat",
            )


def _add_props(world: ET.Element) -> None:
    def add_body(name, pos, geom_type, size, material, rgba=None):
        body = _append(world, "body", name=name, pos=_fmt(pos))
        geom_attrs = {"name": f"{name}_geom", "type": geom_type, "size": _fmt(size), "material": material}
        if rgba is not None:
            geom_attrs["rgba"] = _fmt(rgba)
        _append(body, "geom", **geom_attrs)
        return body

    add_body("plate", (1.05, 1.55, TABLE_TOP_Z + 0.006), "cylinder", (0.09, 0.006, 0.0), "wall_mat", (0.95, 0.95, 0.95, 1.0))
    add_body("mug", (1.05, 1.18, TABLE_TOP_Z + 0.055), "cylinder", (0.05, 0.055, 0.0), "camera_body_mat", (0.20, 0.65, 0.60, 1.0))
    add_body("bowl", (1.48, 1.45, TABLE_TOP_Z + 0.030), "cylinder", (0.055, 0.030, 0.0), "wall_mat", (0.92, 0.92, 0.92, 1.0))
    add_body("box", (1.28, 1.67, TABLE_TOP_Z + 0.038), "box", (0.045, 0.028, 0.038), "stem_mat", (0.65, 0.35, 0.25, 1.0))
    apple = _append(world, "body", name="apple", pos=_fmt((1.95, 1.53, TABLE_TOP_Z + APPLE_RADIUS)))
    _append(apple, "joint", name="apple_free", type="free")
    _append(
        apple,
        "geom",
        name="apple_geom",
        type="sphere",
        size=f"{APPLE_RADIUS}",
        mass=f"{APPLE_MASS}",
        material="apple_mat",
        condim="3",
        friction="1.2 0.01 0.001",
    )


def _add_assets(root: ET.Element, robot_assets: list[ET.Element]) -> None:
    asset = _append(root, "asset")
    _append(asset, "texture", type="skybox", builtin="gradient", rgb1=".25 .3 .38", rgb2="0 0 0", width="32", height="512")
    _append(asset, "texture", name="grid", type="2d", builtin="checker", width="512", height="512", rgb1=".18 .20 .24", rgb2=".24 .27 .32")
    _append(asset, "material", name="grid", texture="grid", texrepeat="8 8", texuniform="true", reflectance=".15")
    _append(asset, "material", name="table_mat", rgba="0.55 0.42 0.28 1", reflectance="0.05")
    _append(asset, "material", name="wall_mat", rgba="0.86 0.87 0.88 1", reflectance="0.04")
    _append(asset, "material", name="apple_mat", rgba="0.84 0.14 0.12 1", reflectance="0.08")
    _append(asset, "material", name="stem_mat", rgba="0.32 0.20 0.10 1")
    _append(asset, "material", name="camera_body_mat", rgba="0.16 0.18 0.20 1", reflectance="0.12")
    _append(asset, "material", name="camera_lens_mat", rgba="0.12 0.56 0.96 1", reflectance="0.25")
    for mesh in robot_assets:
        asset.append(mesh)


def _serialize(root: ET.Element) -> str:
    raw = ET.tostring(root, encoding="unicode")
    pretty = minidom.parseString(raw).toprettyxml(indent="  ")
    return "\n".join(line for line in pretty.splitlines() if line.strip()) + "\n"


def build_xml() -> str:
    robot_root = _build_robot_root()
    robot_assets = _prepare_assets(robot_root)
    worldbody = robot_root.find("worldbody")
    actuator = robot_root.find("actuator")
    sensor = robot_root.find("sensor")
    if worldbody is None:
        raise RuntimeError("Robot XML has no worldbody.")
    if actuator is None:
        raise RuntimeError("Robot XML has no actuator section.")
    if sensor is None:
        raise RuntimeError("Robot XML has no sensor section.")

    robot_bodies = [deepcopy(child) for child in list(worldbody)]
    robot_actuators = [deepcopy(child) for child in list(actuator)]
    robot_sensors = [deepcopy(child) for child in list(sensor)]

    root = ET.Element("mujoco", {"model": "smolvla_tactile_table"})
    _append(root, "compiler", angle="radian")
    _append(root, "option", timestep="0.004", integrator="implicitfast")
    _append(root, "size", nkey="1")

    visual = _append(root, "visual")
    _append(visual, "global", offwidth="1280", offheight="720", azimuth="145", elevation="-20")
    _append(visual, "map", force="0.1", zfar="20")
    _append(visual, "quality", shadowsize="2048")
    _append(root, "statistic", center="1.75 1.75 0.85", extent="2.4")

    default = _append(root, "default")
    _append(default, "geom", condim="3", friction="0.9 0.05 0.005", solref="0.008 1")

    _add_assets(root, robot_assets)
    _add_world(root, robot_bodies)
    _append(root, "actuator")
    _append(root, "sensor")
    root.find("actuator").extend(robot_actuators)  # type: ignore[union-attr]
    root.find("sensor").extend(robot_sensors)  # type: ignore[union-attr]

    return _serialize(root)


def main() -> None:
    parser = argparse.ArgumentParser(description="Build the tactile SmolVLA tabletop scene.")
    parser.add_argument("--out", default=str(HERE / "smolvla_tactile_scene.xml"), help="Output XML path.")
    parser.add_argument("--view", action="store_true", help="Open a MuJoCo viewer after writing XML.")
    args = parser.parse_args()

    xml = build_xml()
    out = Path(args.out)
    out.write_text(xml, encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(out))
    print(f"Wrote {out} | nbody={model.nbody}, njnt={model.njnt}, nu={model.nu}, ngeom={model.ngeom}, ncam={model.ncam}")

    if args.view:
        import mujoco.viewer as mujoco_viewer

        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        mujoco_viewer.launch(model, data)


if __name__ == "__main__":
    main()
