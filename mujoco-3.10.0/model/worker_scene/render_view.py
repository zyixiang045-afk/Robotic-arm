#!/usr/bin/env python3
"""EGL 离屏渲染 worker_scene，绕开崩溃的 simulate 窗口。
用法:
  python3 render_view.py                 # 渲染 ready 姿态若干视角 -> PNG
  python3 render_view.py --video 5        # 渲染 5 秒物理仿真 -> MP4
"""
import os
os.environ.setdefault("MUJOCO_GL", "egl")
import argparse
import numpy as np
import mujoco
import imageio.v2 as imageio

XML = os.path.join(os.path.dirname(__file__), "scene_with_robot.xml")
W, H = 1280, 720


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", type=float, default=0, help="秒数; >0 则录制物理仿真")
    ap.add_argument("--cam", default="overview")
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)   # ready 姿态
    mujoco.mj_forward(m, d)

    with mujoco.Renderer(m, H, W) as r:
        if args.video > 0:
            fps = 30
            path = os.path.join(os.path.dirname(__file__), "worker_scene.mp4")
            with imageio.get_writer(path, fps=fps) as vid:
                n = int(args.video * fps)
                for i in range(n):
                    while d.time < i / fps:
                        mujoco.mj_step(m, d)
                    r.update_scene(d, camera=args.cam)
                    vid.append_data(r.render())
            print("saved", path)
        else:
            for cam in ("overview", "top"):
                r.update_scene(d, camera=cam)
                out = os.path.join(os.path.dirname(__file__), f"view_{cam}.png")
                imageio.imwrite(out, r.render())
                print("saved", out)


if __name__ == "__main__":
    main()
