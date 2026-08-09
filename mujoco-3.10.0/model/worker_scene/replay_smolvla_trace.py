#!/usr/bin/env python3
"""Replay a saved SmolVLA action trace in MuJoCo or render it to MP4."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import re
from pathlib import Path

if sys.platform.startswith("win"):
    os.environ.setdefault("MUJOCO_GL", "glfw")
    os.environ.pop("PYOPENGL_PLATFORM", None)
else:
    os.environ.setdefault("MUJOCO_GL", "osmesa")
    os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")

import imageio.v2 as imageio
import mujoco
import numpy as np
import av

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]
FALLBACK_DIR = WORKSPACE / "mujoco-3.10.0" / "model.test" / "worker_scene"
DEFAULT_XML = HERE / "smolvla_scene.xml"
DEFAULT_TRACE = HERE / "smolvla_trace" / "trace.json"
FALLBACK_XML = FALLBACK_DIR / "smolvla_scene.xml"
FALLBACK_TRACE = FALLBACK_DIR / "smolvla_trace" / "trace.json"

JOINT_NAMES = tuple(f"joint_{i}" for i in range(1, 7))
READY_POSE = {
    "joint_1": 0.0,
    "joint_2": 1.15,
    "joint_3": 0.60,
    "joint_4": 0.0,
    "joint_5": 0.90,
    "joint_6": 0.0,
}

def _joint_id(model: mujoco.MjModel, joint_name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise RuntimeError(f"Missing joint: {joint_name}")
    return joint_id


def _actuator_ids(model: mujoco.MjModel, joint_names: tuple[str, ...]) -> dict[str, int]:
    ids = {}
    for joint_name in joint_names:
        actuator_name = f"act_{joint_name}"
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        if actuator_id < 0:
            raise RuntimeError(f"Missing actuator: {actuator_name}")
        ids[joint_name] = actuator_id
    return ids


def _set_joint_qpos(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str, value: float) -> None:
    joint_id = _joint_id(model, joint_name)
    data.qpos[model.jnt_qposadr[joint_id]] = value


def _reset_ready(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    actuator_ids: dict[str, int],
    joint_names: tuple[str, ...],
) -> None:
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "ready")
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    else:
        mujoco.mj_resetData(model, data)
        for joint_name in joint_names:
            value = READY_POSE.get(joint_name, 0.0)
            _set_joint_qpos(model, data, joint_name, value)
            data.ctrl[actuator_ids[joint_name]] = value
    mujoco.mj_forward(model, data)


def _configure_camera(model: mujoco.MjModel, viewer, camera_name: str) -> None:
    if camera_name == "free":
        return
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if camera_id < 0:
        raise RuntimeError(f"Missing camera: {camera_name}")
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
    viewer.cam.fixedcamid = camera_id


def _load_trace(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"Missing trace JSON: {path}")
    with path.open("r", encoding="utf-8") as f:
        trace = json.load(f)
    if "steps" not in trace:
        raise RuntimeError(f"Trace has no steps: {path}")
    return trace


def _apply_action(
    data: mujoco.MjData,
    actuator_ids: dict[str, int],
    action: dict[str, float],
    joint_names: tuple[str, ...],
) -> None:
    for joint_name in joint_names:
        data.ctrl[actuator_ids[joint_name]] = float(action[joint_name])


def _existing_path(*paths: Path) -> Path:
    for path in paths:
        if path.exists():
            return path
    return paths[0]


def _resolve_user_path(path: Path) -> Path:
    return path.expanduser().resolve()


def _windows_path_to_wsl(raw_path: str) -> Path | None:
    match = re.match(r"^([A-Za-z]):[\\/](.+)$", raw_path)
    if not match:
        return None
    drive = match.group(1).lower()
    rest = match.group(2).replace("\\", "/")
    return Path(f"/mnt/{drive}/{rest}")


def _default_trace_path() -> Path:
    return _existing_path(DEFAULT_TRACE, FALLBACK_TRACE)


def _default_xml_path(trace: dict) -> Path:
    meta_xml = trace.get("meta", {}).get("xml")
    if meta_xml:
        meta_xml_path = Path(meta_xml).expanduser()
        if meta_xml_path.exists():
            return meta_xml_path.resolve()
        wsl_meta_xml = _windows_path_to_wsl(str(meta_xml))
        if wsl_meta_xml is not None and wsl_meta_xml.exists():
            return wsl_meta_xml.resolve()
        local_name = HERE / Path(meta_xml).name
        if local_name.exists():
            return local_name.resolve()
    return _existing_path(DEFAULT_XML, FALLBACK_XML).resolve()


def _replay_steps(trace: dict) -> list[dict]:
    replay_steps = [step for step in trace["steps"] if step.get("action")]
    if not replay_steps:
        raise RuntimeError("Trace has no actionable steps.")
    return replay_steps


def _action_joint_names(replay_steps: list[dict]) -> tuple[str, ...]:
    action = replay_steps[0]["action"]
    if not isinstance(action, dict) or not action:
        raise RuntimeError("Trace action payload is empty or invalid.")
    return tuple(action.keys())


def _available_camera_names(model: mujoco.MjModel) -> tuple[str, ...]:
    return tuple(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i) for i in range(model.ncam))


def _resolve_camera_name(model: mujoco.MjModel, trace: dict, requested: str) -> str:
    if requested != "auto":
        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, requested)
        if camera_id < 0:
            raise RuntimeError(f"Missing camera: {requested}")
        return requested

    trace_cameras = trace.get("meta", {}).get("camera_names", [])
    for camera_name in trace_cameras:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name) >= 0:
            return camera_name

    camera_names = [name for name in _available_camera_names(model) if name]
    if not camera_names:
        raise RuntimeError("No cameras found in MuJoCo model.")
    return camera_names[0]


def _sim_steps_per_action(args: argparse.Namespace, trace: dict) -> int:
    if args.sim_steps_per_action is not None:
        return args.sim_steps_per_action
    return int(trace.get("meta", {}).get("sim_steps_per_action", 20))


def _print_action(step: dict, joint_names: tuple[str, ...]) -> None:
    action = step["action"]
    preview_names = joint_names[:4]
    preview = " ".join(f"{name}={float(action[name]):.3f}" for name in preview_names if name in action)
    print(f"step {step.get('step')}: queue {step.get('queue_len_before')}->{step.get('queue_len_after')} {preview}")


def _render_video(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    actuator_ids: dict[str, int],
    joint_names: tuple[str, ...],
    replay_steps: list[dict],
    trace: dict,
    out_path: Path,
    camera_name: str,
    sim_steps_per_action: int,
    fps: int,
    width: int,
    height: int,
) -> None:
    if camera_name == "free":
        raise RuntimeError("--camera free is only supported for viewer replay, not --video-out.")
    camera_name = _resolve_camera_name(model, trace, camera_name)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"Rendering {len(replay_steps)} actions to {out_path}")
    print(f"camera={camera_name}, fps={fps}, size={width}x{height}, sim_steps_per_action={sim_steps_per_action}")

    with mujoco.Renderer(model, height, width) as renderer:
        with av.open(str(out_path), mode="w") as container:
            stream = container.add_stream("libx264", rate=fps)
            stream.width = width
            stream.height = height
            stream.pix_fmt = "yuv420p"

            renderer.update_scene(data, camera=camera_name)
            frame = av.VideoFrame.from_ndarray(np.asarray(renderer.render(), dtype=np.uint8), format="rgb24")
            for packet in stream.encode(frame):
                container.mux(packet)

            for step in replay_steps:
                _print_action(step, joint_names)
                _apply_action(data, actuator_ids, step["action"], joint_names)
                for _ in range(sim_steps_per_action):
                    mujoco.mj_step(model, data)
                    renderer.update_scene(data, camera=camera_name)
                    frame = av.VideoFrame.from_ndarray(np.asarray(renderer.render(), dtype=np.uint8), format="rgb24")
                    for packet in stream.encode(frame):
                        container.mux(packet)

            for packet in stream.encode():
                container.mux(packet)

    print(f"saved {out_path}")


def _viewer_replay(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    actuator_ids: dict[str, int],
    joint_names: tuple[str, ...],
    replay_steps: list[dict],
    trace: dict,
    args: argparse.Namespace,
    sim_steps_per_action: int,
) -> None:
    import mujoco.viewer

    print("Viewer: mouse to orbit/pan/zoom. Close the MuJoCo window to exit.")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        camera_name = _resolve_camera_name(model, trace, args.camera)
        _configure_camera(model, viewer, camera_name)
        while viewer.is_running():
            if args.reset_each_loop:
                _reset_ready(model, data, actuator_ids, joint_names)
                viewer.sync()

            for step in replay_steps:
                if not viewer.is_running():
                    break
                _print_action(step, joint_names)
                _apply_action(data, actuator_ids, step["action"], joint_names)
                for _ in range(sim_steps_per_action):
                    if not viewer.is_running():
                        break
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    if args.frame_delay > 0:
                        time.sleep(args.frame_delay)
                if args.step_hold > 0:
                    time.sleep(args.step_hold)

            if not args.loop:
                break

        while viewer.is_running() and not args.loop:
            viewer.sync()
            time.sleep(0.03)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay smolvla_trace/trace.json in MuJoCo or render MP4.")
    parser.add_argument("--xml", type=Path, default=None)
    parser.add_argument("--trace", type=Path, default=None)
    parser.add_argument("--camera", default="auto", help="Fixed MuJoCo camera name, 'auto', or 'free' for viewer mode.")
    parser.add_argument("--video-out", type=Path, default=None, help="Write replay video to this MP4 path.")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--loop", action="store_true", help="Loop the saved action trace in viewer mode.")
    parser.add_argument("--reset-each-loop", action="store_true", help="Reset to ready pose before each viewer loop.")
    parser.add_argument("--frame-delay", type=float, default=0.015, help="Wall-clock delay after each viewer mj_step.")
    parser.add_argument("--step-hold", type=float, default=0.35, help="Extra viewer pause after each saved trace step.")
    parser.add_argument(
        "--sim-steps-per-action",
        type=int,
        default=None,
        help="Override MuJoCo sim steps per saved action. Defaults to trace metadata.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    trace_path = _resolve_user_path(args.trace) if args.trace is not None else _default_trace_path().resolve()
    trace = _load_trace(trace_path)

    xml = _resolve_user_path(args.xml) if args.xml is not None else _default_xml_path(trace)
    if not xml.exists():
        raise FileNotFoundError(f"Missing MuJoCo XML: {xml}")

    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    sim_steps_per_action = _sim_steps_per_action(args, trace)
    replay_steps = _replay_steps(trace)
    joint_names = _action_joint_names(replay_steps)
    actuator_ids = _actuator_ids(model, joint_names)

    _reset_ready(model, data, actuator_ids, joint_names)
    print(f"Loaded XML: {xml}")
    print(f"Loaded trace: {trace_path}")
    print(f"Replay steps: {len(replay_steps)}, sim_steps_per_action={sim_steps_per_action}")

    if args.video_out is not None:
        _render_video(
            model=model,
            data=data,
            actuator_ids=actuator_ids,
            joint_names=joint_names,
            replay_steps=replay_steps,
            trace=trace,
            out_path=_resolve_user_path(args.video_out),
            camera_name=args.camera,
            sim_steps_per_action=sim_steps_per_action,
            fps=args.fps,
            width=args.width,
            height=args.height,
        )
    else:
        _viewer_replay(model, data, actuator_ids, joint_names, replay_steps, trace, args, sim_steps_per_action)


if __name__ == "__main__":
    main()
