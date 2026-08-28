#!/usr/bin/env python3.8
"""Launch the shared 3D SLAM bridge with the ARIAC lab model."""

import os
import runpy


HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
os.environ.setdefault("SLAM_SCENE_NAME", "ariac")
os.environ.setdefault(
    "MUJOCO_SCENE_XML",
    os.path.join(PROJECT_ROOT, "model", "robot", "ariac_lab_with_robot_3d.xml"))

runpy.run_path(os.path.join(HERE, "bridge_warehouse.py"), run_name="__main__")
