#!/usr/bin/env python3.8
"""Regression checks for the organized ARIAC scene and SLAM integration."""

import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[3]
SCENE_XML = ROOT / "model" / "scenes" / "ariac_lab.xml"
ROBOT_XML = ROOT / "model" / "robot" / "ariac_lab_with_robot_3d.xml"
RUN_SLAM = ROOT / "slam" / "run_slam_3d.sh"
RUN_NAV = ROOT / "slam" / "run_nav_saved.sh"


class AriacSceneTest(unittest.TestCase):
    def test_scene_assets_are_organized_and_resolvable(self):
        self.assertFalse((ROOT / "ariac_mujoco").exists())
        self.assertTrue((ROOT / "model" / "assets" / "ariac" /
                         "conversion_report.json").is_file())
        for xml_path in (SCENE_XML, ROBOT_XML):
            tree = ET.parse(str(xml_path))
            for mesh in tree.findall("./asset/mesh"):
                mesh_path = (xml_path.parent / mesh.attrib["file"]).resolve()
                self.assertTrue(mesh_path.is_file(), str(mesh_path))

    def test_mujoco_323_loads_static_and_robot_scenes(self):
        self.assertEqual(mujoco.__version__, "3.2.3")
        static_model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
        robot_model = mujoco.MjModel.from_xml_path(str(ROBOT_XML))
        self.assertGreaterEqual(static_model.nmesh, 95)
        self.assertGreater(robot_model.nbody, static_model.nbody)
        self.assertGreater(robot_model.nu, 0)
        self.assertGreater(robot_model.nsensor, 0)
        self.assertGreaterEqual(mujoco.mj_name2id(
            robot_model, mujoco.mjtObj.mjOBJ_BODY, "dog_base"), 0)
        self.assertGreaterEqual(mujoco.mj_name2id(
            robot_model, mujoco.mjtObj.mjOBJ_SITE, "lidar3d_frame"), 0)

    def test_slam_defaults_to_ariac_bridge(self):
        script = RUN_SLAM.read_text(encoding="utf-8")
        self.assertRegex(script, r'(?m)^SCENE="ariac"$')
        self.assertRegex(
            script,
            r'if \[ "\$SCENE" = "ariac" \]; then\s+' +
            r'BRIDGE_SCRIPT="slam/bridge/bridge_ariac.py"')
        self.assertIn("ariac|lab|warehouse", script)

        nav_script = RUN_NAV.read_text(encoding="utf-8")
        self.assertRegex(nav_script, r'(?m)^SCENE="ariac"$')
        self.assertIn('BRIDGE_SCRIPT="slam/bridge/bridge_ariac.py"', nav_script)


if __name__ == "__main__":
    unittest.main(verbosity=2)
