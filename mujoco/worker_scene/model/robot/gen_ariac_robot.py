#!/usr/bin/env python3
"""Inject the mobile robot and 3D lidar into the ARIAC lab scene."""

import argparse
import re
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROBOT_SOURCE = HERE / "scene_with_robot_3d_py38.xml"
SCENE_SOURCE = HERE.parent / "scenes" / "ariac_lab.xml"
DEFAULT_OUTPUT = HERE / "ariac_lab_with_robot_3d.xml"


def section(text, tag):
    match = re.search(r"(?s)  <%s(?:\s[^>]*)?>.*?</%s>" % (tag, tag), text)
    if not match:
        raise ValueError("missing <%s> section" % tag)
    return match.group(0)


def build(start_x, start_y):
    robot = ROBOT_SOURCE.read_text(encoding="utf-8")
    scene = SCENE_SOURCE.read_text(encoding="utf-8")

    defaults = re.findall(
        r"(?ms)^    <default class=\"(?:arm|hand)_[lr]/main\">.*?^    </default>",
        section(robot, "default"))
    default_block = "  <default>\n%s\n  </default>\n\n" % "\n".join(defaults)
    scene = scene.replace("  <asset>\n", default_block + "  <asset>\n", 1)

    asset_lines = []
    for line in section(robot, "asset").splitlines()[1:-1]:
        if ('mesh name="dog_' in line or 'mesh name="arm_' in line or
                'mesh name="hand_' in line or 'material name="dog_mat"' in line):
            line = re.sub(r'file="[^"]*/model/assets/', 'file="../assets/', line)
            asset_lines.append(line)
    scene = scene.replace("  </asset>", "\n".join(asset_lines) + "\n  </asset>", 1)

    worldbody = section(robot, "worldbody")
    dog_start = worldbody.index('    <body name="dog_base"')
    dog = worldbody[dog_start:worldbody.rfind("  </worldbody>")].rstrip()
    dog = re.sub(r'(<body name="dog_base"\s+pos=")[^"]*"',
                 r'\g<1>%.6g %.6g 0"' % (start_x, start_y), dog, count=1)
    scene = scene.replace("  </worldbody>", dog + "\n  </worldbody>", 1)

    tail = section(robot, "actuator") + "\n\n" + section(robot, "sensor")
    scene = scene.replace("</mujoco>", tail + "\n</mujoco>", 1)
    return scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-x", type=float, default=-4.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    args.output.write_text(build(args.start_x, args.start_y), encoding="utf-8")
    print("generated %s (start: %.3f, %.3f)" %
          (args.output, args.start_x, args.start_y))


if __name__ == "__main__":
    main()
