#!/usr/bin/env python3
"""Build a compact MuJoCo scene for local SmolVLA inference.

The scene intentionally avoids the MjSpec.attach API so it can be loaded by the
OpenPI virtualenv's MuJoCo 2.3.7. The RM65 arm is converted from the local URDF
to MJCF, then embedded as a static XML subtree.
"""

from __future__ import annotations

import argparse
import math
import os
import re
from copy import deepcopy
from pathlib import Path
from xml.dom import minidom
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ASSET_DIR = HERE / "assets"
ARM_DIR = ASSET_DIR / "arm"
URDF = ASSET_DIR / "urdf" / "RM65-6F.urdf"

ROOM_X = (0.0, 3.0)
ROOM_Y = (0.0, 3.0)
WALL_H = 2.8
WALL_T = 0.10

TABLE_XY = (2.25, 2.35)
TABLE_TOP_Z = 0.70
TABLE_HALF = (0.45, 0.35, 0.02)
TABLE_LEG_HALF = 0.03
BOARD_HALF = (0.15, 0.15, 0.004)

ARM_MOUNT_XYZ = (1.55, 2.35, 0.59)
ARM_PEDESTAL_TOP = 0.56

APPLE_RADIUS = 0.036
APPLE_MASS = 0.08
APPLE_XYZ = (TABLE_XY[0] - 0.23, TABLE_XY[1] - 0.08, TABLE_TOP_Z + APPLE_RADIUS)

CAMERAS = (
    ("camera1", (1.00, 1.00, 1.75), (2.12, 2.30, 1.05)),
    ("camera2", (3.05, 1.55, 1.65), (2.15, 2.28, 1.05)),
    ("camera3", (2.25, 2.35, 2.25), (2.12, 2.28, 1.02)),
)
CAM_RES = (256, 256)
CAM_FOVY = 58.0
CAM_MARKER_BODY = (0.040, 0.024, 0.016)
CAM_MARKER_STEM = 0.007
CAM_MARKER_STEM_LEN = 0.052
CAM_MARKER_LENS = 0.011

JOINT_NAMES = tuple(f"joint_{i}" for i in range(1, 7))
READY_POSE = {
    "joint_1": 0.0,
    "joint_2": 1.05,
    "joint_3": 0.60,
    "joint_4": 0.0,
    "joint_5": 0.90,
    "joint_6": 0.0,
}


def _fmt(values) -> str:
    return " ".join(f"{float(v):.10g}" for v in values)


def _quat_z(angle: float) -> tuple[float, float, float, float]:
    return (math.cos(angle / 2.0), 0.0, 0.0, math.sin(angle / 2.0))


def _normalize(v: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(v)
    if norm < 1e-9:
        return v
    return v / norm


def _lookat_quat(pos, target, up=(0.0, 0.0, 1.0)) -> tuple[float, float, float, float]:
    """Return a MuJoCo camera quaternion that looks from pos to target."""
    pos = np.asarray(pos, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    up = np.asarray(up, dtype=np.float64)

    forward = _normalize(target - pos)
    z_axis = -forward
    if np.linalg.norm(np.cross(up, z_axis)) < 1e-6:
        up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    x_axis = _normalize(np.cross(up, z_axis))
    y_axis = _normalize(np.cross(z_axis, x_axis))
    rot = np.column_stack([x_axis, y_axis, z_axis]).astype(np.float64)

    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, rot.flatten())
    return tuple(float(x) for x in quat)


def _append(parent: ET.Element, tag: str, **attrs) -> ET.Element:
    clean = {key: str(value) for key, value in attrs.items() if value is not None}
    return ET.SubElement(parent, tag, clean)


def _add_camera_rig(world: ET.Element, name: str, pos, target) -> None:
    """Create a visible fixed camera rig without changing the camera name."""
    rig = _append(
        world,
        "body",
        name=f"{name}_rig",
        pos=_fmt(pos),
        quat=_fmt(_lookat_quat(pos, target)),
    )
    _append(
        rig,
        "camera",
        name=name,
        pos="0 0 0",
        quat="1 0 0 0",
        fovy=CAM_FOVY,
    )
    _append(
        rig,
        "geom",
        name=f"{name}_stem",
        type="capsule",
        fromto=_fmt((0.0, 0.0, 0.0, 0.0, 0.0, CAM_MARKER_STEM_LEN)),
        size=f"{CAM_MARKER_STEM}",
        material="camera_body_mat",
        contype="0",
        conaffinity="0",
    )
    _append(
        rig,
        "geom",
        name=f"{name}_body",
        type="box",
        pos=_fmt((0.0, 0.0, CAM_MARKER_STEM_LEN + CAM_MARKER_BODY[2])),
        size=_fmt(CAM_MARKER_BODY),
        material="camera_body_mat",
        contype="0",
        conaffinity="0",
    )
    _append(
        rig,
        "geom",
        name=f"{name}_lens",
        type="sphere",
        pos=_fmt((0.0, 0.0, CAM_MARKER_STEM_LEN + 2.0 * CAM_MARKER_BODY[2] + CAM_MARKER_LENS)),
        size=f"{CAM_MARKER_LENS}",
        material="camera_lens_mat",
        contype="0",
        conaffinity="0",
    )


def _load_arm_xml() -> ET.Element:
    """Convert the RM65 URDF to MJCF and return the parsed MJCF root."""
    text = URDF.read_text(encoding="utf-8")
    text = text.replace("package://RM65-6F/meshes/", "")
    inject = (
        '<robot name="RM65-6F">\n'
        f'  <mujoco><compiler meshdir="{ARM_DIR.as_posix()}/" '
        'balanceinertia="true" discardvisual="false"/></mujoco>'
    )
    text = re.sub(r'<robot\s+name="RM65-6F">', inject, text, count=1)

    spec = mujoco.MjSpec()
    spec.from_string(text)
    spec.compile()
    return ET.fromstring(spec.to_xml())


def _prepare_arm_assets(arm_root: ET.Element) -> list[ET.Element]:
    assets = []
    for mesh in arm_root.findall("./asset/mesh"):
        copied = deepcopy(mesh)
        file_name = Path(copied.attrib["file"]).name
        copied.set("file", f"assets/arm/{file_name}")
        assets.append(copied)
    return assets


def _strip_unsupported_attrs(elem: ET.Element) -> None:
    for key in ("actuatorfrcrange",):
        elem.attrib.pop(key, None)
    for child in elem:
        _strip_unsupported_attrs(child)


def _prepare_arm_body(arm_root: ET.Element) -> ET.Element:
    worldbody = arm_root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Converted RM65 XML has no worldbody.")

    arm = ET.Element(
        "body",
        {
            "name": "arm",
            "pos": _fmt(ARM_MOUNT_XYZ),
            "quat": _fmt(_quat_z(math.pi)),
            "gravcomp": "1",
        },
    )

    for child in list(worldbody):
        copied = deepcopy(child)
        _strip_unsupported_attrs(copied)
        if copied.tag == "geom":
            copied.set("contype", "2")
            copied.set("conaffinity", "1")
            copied.set("friction", "1 0.005 0.0001")
            copied.set("solref", "0.02 1")
        elif copied.tag == "body":
            _configure_arm_subtree(copied)
        arm.append(copied)
    return arm


def _configure_arm_subtree(elem: ET.Element) -> None:
    if elem.tag == "body":
        elem.set("gravcomp", "1")
    if elem.tag == "joint":
        elem.attrib.pop("actuatorfrcrange", None)
        if "range" in elem.attrib:
            elem.set("limited", "true")
    if elem.tag == "geom":
        elem.set("contype", "2")
        elem.set("conaffinity", "1")
        elem.set("friction", "1 0.005 0.0001")
        elem.set("solref", "0.02 1")
    for child in elem:
        _configure_arm_subtree(child)


def _add_assets(root: ET.Element, arm_assets: list[ET.Element]) -> None:
    asset = _append(root, "asset")
    _append(
        asset,
        "texture",
        type="skybox",
        builtin="gradient",
        rgb1=".25 .3 .38",
        rgb2="0 0 0",
        width="32",
        height="512",
    )
    _append(
        asset,
        "texture",
        name="grid",
        type="2d",
        builtin="checker",
        width="512",
        height="512",
        rgb1=".18 .20 .24",
        rgb2=".24 .27 .32",
    )
    _append(asset, "texture", name="board_tex", type="2d", file="assets/calib_board.png")

    _append(asset, "material", name="grid", texture="grid", texrepeat="8 8", texuniform="true", reflectance=".15")
    _append(asset, "material", name="table_mat", rgba="0.55 0.42 0.28 1", reflectance="0.05")
    _append(asset, "material", name="board_mat", texture="board_tex", texrepeat="1 1", texuniform="false")
    _append(asset, "material", name="apple_mat", rgba="0.84 0.14 0.12 1", reflectance="0.08")
    _append(asset, "material", name="stem_mat", rgba="0.32 0.20 0.10 1")
    _append(asset, "material", name="metal_mat", rgba="0.55 0.57 0.60 1", reflectance="0.25")
    _append(asset, "material", name="camera_body_mat", rgba="0.16 0.18 0.20 1", reflectance="0.12")
    _append(asset, "material", name="camera_lens_mat", rgba="0.12 0.56 0.96 1", reflectance="0.25")
    _append(asset, "material", name="wall_mat", rgba="0.86 0.87 0.88 1", reflectance="0.04")

    for mesh in arm_assets:
        asset.append(mesh)


def _add_world(root: ET.Element, arm_body: ET.Element) -> None:
    world = _append(root, "worldbody")
    _append(world, "geom", name="floor", type="plane", size="0 0 0.05", material="grid", condim="3")
    _append(world, "light", name="top1", pos="-0.5 0.7 3.0", dir="0 0 -1", diffuse="0.7 0.7 0.7")
    _append(world, "light", name="top2", pos="2.0 2.0 3.0", dir="0 0 -1", diffuse="0.55 0.55 0.55")
    _append(world, "light", name="table_fill", pos="1.5 1.8 2.1", dir="0.3 0.3 -1", diffuse="0.25 0.25 0.25")

    for name, pos, target in CAMERAS:
        _add_camera_rig(world, name, pos, target)

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

    _add_corner_table(world)
    _add_apple(world)
    _add_arm_pedestal(world)
    world.append(arm_body)


def _add_corner_table(world: ET.Element) -> None:
    table = _append(world, "body", name="corner_table", pos=_fmt((TABLE_XY[0], TABLE_XY[1], 0.0)))
    _append(
        table,
        "geom",
        name="corner_table_top",
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
                name=f"corner_table_leg_{sx}_{sy}",
                type="box",
                pos=_fmt(
                    (
                        sx * (TABLE_HALF[0] - TABLE_LEG_HALF - 0.02),
                        sy * (TABLE_HALF[1] - TABLE_LEG_HALF - 0.02),
                        leg_z,
                    )
                ),
                size=_fmt((TABLE_LEG_HALF, TABLE_LEG_HALF, leg_z)),
                material="table_mat",
            )
    _append(
        table,
        "geom",
        name="calib_board",
        type="box",
        pos=_fmt((0.0, 0.0, TABLE_TOP_Z + BOARD_HALF[2])),
        size=_fmt(BOARD_HALF),
        material="board_mat",
        contype="0",
        conaffinity="0",
    )


def _add_apple(world: ET.Element) -> None:
    apple = _append(world, "body", name="apple", pos=_fmt(APPLE_XYZ))
    _append(apple, "joint", name="apple_free", type="free")
    _append(
        apple,
        "geom",
        name="apple_body",
        type="sphere",
        size=f"{APPLE_RADIUS}",
        material="apple_mat",
        mass=f"{APPLE_MASS}",
        condim="3",
        friction="1.2 0.01 0.001",
    )
    _append(
        apple,
        "geom",
        name="apple_stem",
        type="capsule",
        fromto=_fmt((0.0, 0.0, APPLE_RADIUS * 0.65, 0.0, 0.0, APPLE_RADIUS * 1.2)),
        size="0.004",
        material="stem_mat",
        contype="0",
        conaffinity="0",
    )
    _append(apple, "site", name="apple_site", pos="0 0 0", size="0.01 0.01 0.01", rgba="0 1 1 0")


def _add_arm_pedestal(world: ET.Element) -> None:
    pedestal = _append(world, "body", name="arm_pedestal", pos=_fmt((ARM_MOUNT_XYZ[0], ARM_MOUNT_XYZ[1], 0.0)))
    _append(
        pedestal,
        "geom",
        name="arm_pedestal_base",
        type="box",
        pos=_fmt((0.0, 0.0, ARM_PEDESTAL_TOP / 2.0)),
        size=_fmt((0.14, 0.14, ARM_PEDESTAL_TOP / 2.0)),
        material="metal_mat",
    )
    _append(
        pedestal,
        "geom",
        name="arm_pedestal_cap",
        type="box",
        pos=_fmt((0.0, 0.0, ARM_PEDESTAL_TOP + 0.015)),
        size=_fmt((0.16, 0.16, 0.015)),
        material="metal_mat",
    )


def _add_actuators(root: ET.Element) -> None:
    actuator = _append(root, "actuator")
    for joint_name in JOINT_NAMES:
        _append(
            actuator,
            "general",
            name=f"act_{joint_name}",
            joint=joint_name,
            biastype="affine",
            gainprm="40",
            biasprm="0 -40 -15",
        )


def _add_ready_keyframe(root: ET.Element, xml_without_keyframe: str) -> None:
    old_cwd = os.getcwd()
    try:
        os.chdir(HERE)
        model = mujoco.MjModel.from_xml_string(xml_without_keyframe)
    finally:
        os.chdir(old_cwd)
    qpos = model.qpos0.copy()
    ctrl = np.zeros(model.nu, dtype=np.float64)

    for joint_name, value in READY_POSE.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid < 0:
            raise RuntimeError(f"Missing joint in generated scene: {joint_name}")
        qpos[model.jnt_qposadr[jid]] = value

        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{joint_name}")
        if aid < 0:
            raise RuntimeError(f"Missing actuator in generated scene: act_{joint_name}")
        ctrl[aid] = value

    keyframe = _append(root, "keyframe")
    _append(keyframe, "key", name="ready", qpos=_fmt(qpos), ctrl=_fmt(ctrl))


def build_xml() -> str:
    arm_root = _load_arm_xml()
    arm_assets = _prepare_arm_assets(arm_root)
    arm_body = _prepare_arm_body(arm_root)

    root = ET.Element("mujoco", {"model": "smolvla_corner_table"})
    _append(root, "compiler", angle="radian")
    _append(root, "option", timestep="0.004", integrator="implicitfast")
    _append(root, "size", nkey="1")

    visual = _append(root, "visual")
    _append(visual, "global", offwidth="1280", offheight="720", azimuth="145", elevation="-20")
    _append(visual, "map", force="0.1", zfar="20")
    _append(visual, "quality", shadowsize="2048")
    _append(root, "statistic", center="1.85 2.15 0.85", extent="2.4")

    default = _append(root, "default")
    _append(default, "geom", condim="3", friction="0.9 0.05 0.005", solref="0.008 1")

    _add_assets(root, arm_assets)
    _add_world(root, arm_body)
    _add_actuators(root)

    xml_without_keyframe = _serialize(root)
    _add_ready_keyframe(root, xml_without_keyframe)
    return _serialize(root)


def _serialize(root: ET.Element) -> str:
    raw = ET.tostring(root, encoding="unicode")
    pretty = minidom.parseString(raw).toprettyxml(indent="  ")
    lines = [line for line in pretty.splitlines() if line.strip()]
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="Build the SmolVLA tabletop scene.")
    parser.add_argument("--out", default=str(HERE / "smolvla_scene.xml"), help="Output XML path.")
    parser.add_argument("--view", action="store_true", help="Open a MuJoCo viewer after writing XML.")
    args = parser.parse_args()

    xml = build_xml()
    out = Path(args.out)
    out.write_text(xml, encoding="utf-8")

    model = mujoco.MjModel.from_xml_path(str(out))
    print(
        f"Wrote {out} | nbody={model.nbody}, njnt={model.njnt}, nu={model.nu}, "
        f"ngeom={model.ngeom}, ncam={model.ncam}"
    )

    if args.view:
        import mujoco.viewer as mujoco_viewer

        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        mujoco_viewer.launch(model, data)


if __name__ == "__main__":
    main()
