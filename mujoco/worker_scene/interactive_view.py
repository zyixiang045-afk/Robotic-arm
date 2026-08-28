#!/usr/bin/env python3
"""交互查看 worker_scene：mujoco.viewer 真实窗口 + 两个手背相机实时同步画面。

主窗口用 mujoco.viewer.launch_passive（本机可正常开窗，崩的是 bin/simulate），
同一个 MjData 上跑物理，另用离屏 Renderer 渲染双手腕相机，显示在 cv2 窗口。

操作:
  viewer 主窗口: 鼠标交互照旧（左键转 / 右键平移 / 滚轮缩放）。
  cv2 窗口 "hand cameras": 空格=暂停/继续  W=摆臂演示  R=复位ready  ESC/q=退出

用法:
  python3 interactive_view.py              # 开窗交互（需图形界面/WSLg）
  python3 interactive_view.py --no-cams    # 只开 viewer 主窗口，不显示手相机
  python3 interactive_view.py --offscreen  # 无显示环境：EGL 离屏 + cv2 手写相机控制
"""

# 默认直接播放：一打开场景就在动（料车/扫掠杆/可选摆臂演示），
# 不会出现"窗口开了但静止不动"的错觉。暂停/继续按 空格(cv2 窗口)。
import argparse
import os

# py3.8 + mujoco 3.2.3 只认识去掉 colorspace 属性的 *_py38.xml，自动按可加载性选择。
XML_CANDIDATES = (
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "model", "robot", "scene_with_robot.xml"),
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "model", "robot", "scene_with_robot_py38.xml"),
)
for _cand in XML_CANDIDATES:
    if os.path.exists(_cand):
        try:
            import mujoco
            mujoco.MjModel.from_xml_path(_cand)
            XML = _cand
            break
        except Exception:
            continue
else:
    XML = XML_CANDIDATES[0]
HAND_CAMS = ("cam_hand_l", "cam_hand_r")
CAM_W, CAM_H = 480, 480
WIN = "hand cameras (synced)"
W, H = 1280, 720


def _wave(m, d, acts):
    """双臂 joint_2 小幅摆动，用于直观看手相机画面随臂运动实时变化。"""
    import numpy as np
    for a in acts:
        d.ctrl[a] = 1.2 + 0.25 * np.sin(2 * np.pi * 0.3 * d.time)


def run_windowed(show_cams=True):
    """launch_passive 主窗口 + （可选）双手相机 cv2 窗口。"""
    import numpy as np
    import mujoco
    import mujoco.viewer

    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)     # ready 姿态
    mujoco.mj_forward(m, d)

    cv2 = cam_r = None
    if show_cams:
        import cv2 as _cv2
        cv2 = _cv2
        cam_r = mujoco.Renderer(m, CAM_H, CAM_W)   # 与 viewer 各自持有 GL 上下文
        cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)

    wave_acts = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{s}/joint_2")
                 for s in ("arm_l", "arm_r")]
    wave_acts = [a for a in wave_acts if a >= 0]
    # 默认直接播放，避免一打开是静止画面被当成"没反应"。
    # 注意：MuJoCo viewer 自带的播放按钮不驱动物理（步进由本脚本控制），
    # 暂停/继续只能按 空格（cv2 窗口）。
    state = {"paused": False, "wave": False}
    print("就绪: 场景已开动 | viewer 窗口鼠标交互" + (
        " | cv2 窗口: 空格=暂停/继续 W=摆臂 R=复位 ESC/q=退出" if show_cams
        else "（--no-cams：空格不可用，Ctrl-C 退出）"))

    with mujoco.viewer.launch_passive(m, d) as viewer:
        step_per_frame = max(1, int(1 / m.opt.timestep / 60))
        while viewer.is_running():
            if not state["paused"]:
                for _ in range(step_per_frame):
                    if state["wave"]:
                        _wave(m, d, wave_acts)
                    mujoco.mj_step(m, d)
            else:
                mujoco.mj_forward(m, d)
            viewer.sync()

            if not show_cams:
                continue

            panes = []
            for cam in HAND_CAMS:
                cam_r.update_scene(d, camera=cam)   # 相机绑在手 body 上，随臂运动
                img = cv2.cvtColor(cam_r.render(), cv2.COLOR_RGB2BGR)
                cv2.putText(img, cam, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2)
                panes.append(img)
            frame = np.hstack(panes)
            tag = "PAUSED" if state["paused"] else "t=%.2fs" % d.time
            cv2.putText(frame, tag + ("  WAVE" if state["wave"] else ""),
                        (10, frame.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 200, 255), 2)
            cv2.imshow(WIN, frame)

            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord('q')):
                break
            elif k == ord(' '):
                state["paused"] = not state["paused"]
            elif k == ord('w'):
                state["wave"] = not state["wave"]
            elif k == ord('r'):
                mujoco.mj_resetDataKeyframe(m, d, 0)
                mujoco.mj_forward(m, d)
                state["wave"] = False

    if show_cams:
        cam_r.close()
        cv2.destroyAllWindows()


class OffscreenViewer:
    """无显示环境的回退路径：EGL 离屏渲染 + cv2 窗口里手写鼠标控制自由相机。"""

    def __init__(self):
        import mujoco
        import cv2
        self.mujoco, self.cv2 = mujoco, cv2
        self.m = mujoco.MjModel.from_xml_path(XML)
        self.d = mujoco.MjData(self.m)
        self.reset()
        self.cam = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(self.m, self.cam)
        self.opt = mujoco.MjvOption()
        self.renderer = mujoco.Renderer(self.m, H, W)
        self.paused = True
        self.last = None      # (x, y)

    def reset(self):
        self.mujoco.mj_resetDataKeyframe(self.m, self.d, 0)
        self.mujoco.mj_forward(self.m, self.d)

    def on_mouse(self, event, x, y, flags, _):
        cv2, mujoco = self.cv2, self.mujoco
        if event in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_RBUTTONDOWN):
            self.last = (x, y)
        elif event in (cv2.EVENT_LBUTTONUP, cv2.EVENT_RBUTTONUP):
            self.last = None
        elif event == cv2.EVENT_MOUSEMOVE and self.last is not None:
            dx, dy = x - self.last[0], y - self.last[1]
            self.last = (x, y)
            act = (mujoco.mjtMouse.mjMOUSE_MOVE_V if flags & cv2.EVENT_FLAG_RBUTTON
                   else mujoco.mjtMouse.mjMOUSE_ROTATE_V)
            mujoco.mjv_moveCamera(self.m, act, dx / H, dy / H,
                                  self.renderer.scene, self.cam)
        elif event == cv2.EVENT_MOUSEWHEEL:
            s = -0.05 if flags > 0 else 0.05
            mujoco.mjv_moveCamera(self.m, mujoco.mjtMouse.mjMOUSE_ZOOM,
                                  0, s, self.renderer.scene, self.cam)

    def run(self):
        cv2, mujoco = self.cv2, self.mujoco
        win = "worker_scene (EGL)"
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(win, self.on_mouse)
        print("就绪：空格=播放/暂停  左键=转  右键=平移  滚轮=缩放  R=复位  ESC=退出")
        while True:
            if not self.paused:
                for _ in range(int(1 / self.m.opt.timestep / 60)):
                    mujoco.mj_step(self.m, self.d)
            else:
                mujoco.mj_forward(self.m, self.d)
            self.renderer.update_scene(self.d, self.cam, self.opt)
            img = cv2.cvtColor(self.renderer.render(), cv2.COLOR_RGB2BGR)
            tag = "PAUSED" if self.paused else "t=%.2fs" % self.d.time
            cv2.putText(img, tag, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 0), 2)
            cv2.imshow(win, img)
            k = cv2.waitKey(16) & 0xFF
            if k in (27, ord('q')):
                break
            elif k == ord(' '):
                self.paused = not self.paused
            elif k == ord('r'):
                self.reset()
        cv2.destroyAllWindows()
        self.renderer.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-cams", action="store_true",
                    help="只开 viewer 主窗口，不显示手背相机画面")
    ap.add_argument("--offscreen", action="store_true",
                    help="无显示环境：走 EGL 离屏 + cv2 手写相机控制")
    args = ap.parse_args()

    if args.offscreen:
        os.environ.setdefault("MUJOCO_GL", "egl")
        OffscreenViewer().run()
    else:
        run_windowed(show_cams=not args.no_cams)


if __name__ == "__main__":
    main()
