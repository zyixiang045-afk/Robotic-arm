#!/usr/bin/env python3
"""Run SmolVLA inference on the worker_scene tabletop MuJoCo scene."""

from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import html
import json
import os
import time
from pathlib import Path


def _preload_osmesa() -> bool:
    candidates = [
        "/lib/x86_64-linux-gnu/libOSMesa.so",
        "/lib/x86_64-linux-gnu/libOSMesa.so.8",
        "/usr/lib/x86_64-linux-gnu/libOSMesa.so",
        "/usr/lib/x86_64-linux-gnu/libOSMesa.so.8",
    ]
    found = ctypes.util.find_library("OSMesa")
    if found:
        candidates.append(found)
    for candidate in dict.fromkeys(candidates):
        if candidate.startswith("/") and not Path(candidate).exists():
            continue
        try:
            ctypes.CDLL(candidate, mode=ctypes.RTLD_GLOBAL)
            return True
        except OSError:
            continue
    return False


def _configure_mujoco_gl() -> None:
    gl_backend = os.environ.get("MUJOCO_GL")
    if gl_backend is None:
        if _preload_osmesa():
            os.environ["MUJOCO_GL"] = "osmesa"
    elif gl_backend.lower() == "osmesa":
        _preload_osmesa()


_configure_mujoco_gl()

import imageio.v2 as imageio
import mujoco
import numpy as np
import torch

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.utils import make_robot_action

try:
    from lerobot.datasets.utils import combine_feature_dicts
except (ImportError, AttributeError):
    def combine_feature_dicts(*dicts: dict) -> dict:
        merged: dict = {}
        for item in dicts:
            merged.update(item)
        return merged

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]
DEFAULT_MODEL_DIR = WORKSPACE / "models" / "smolvla_base"
DEFAULT_VLM_MODEL_DIR = WORKSPACE / "models" / "SmolVLM2-500M-Video-Instruct"
DEFAULT_XML = HERE / "smolvla_tactile_scene.xml"

JOINT_NAMES = (
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
CAMERAS = ("image", "wrist_image")
CAM_WIDTH = 256
CAM_HEIGHT = 256
TACTILE_SENSOR_NAMES = tuple(f"tactile_{i:02d}" for i in range(20))
ACTION_KEY = "action"
READY_POSE = {
    "arm/joint_1": 0.0,
    "arm/joint_2": 1.05,
    "arm/joint_3": 0.60,
    "arm/joint_4": 0.0,
    "arm/joint_5": 0.90,
    "arm/joint_6": 0.0,
}


def _joint_id(model: mujoco.MjModel, joint_name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise RuntimeError(f"Missing joint: {joint_name}")
    return joint_id


def _actuator_ids(model: mujoco.MjModel) -> dict[str, int]:
    ids = {}
    for joint_name in JOINT_NAMES:
        actuator_name = f"act_{joint_name}"
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        if actuator_id < 0:
            raise RuntimeError(f"Missing actuator: {actuator_name}")
        ids[joint_name] = actuator_id
    return ids


def _tactile_sensor_ids(model: mujoco.MjModel) -> dict[str, int]:
    ids = {}
    for sensor_name in TACTILE_SENSOR_NAMES:
        sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
        if sensor_id < 0:
            raise RuntimeError(f"Missing tactile sensor: {sensor_name}")
        ids[sensor_name] = sensor_id
    return ids


def _set_joint_qpos(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str, value: float) -> None:
    joint_id = _joint_id(model, joint_name)
    data.qpos[model.jnt_qposadr[joint_id]] = value


def _get_joint_qpos(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str) -> float:
    joint_id = _joint_id(model, joint_name)
    return float(data.qpos[model.jnt_qposadr[joint_id]])


def _reset_ready(model: mujoco.MjModel, data: mujoco.MjData, actuator_ids: dict[str, int]) -> None:
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "ready")
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    else:
        mujoco.mj_resetData(model, data)
        for joint_name, value in READY_POSE.items():
            _set_joint_qpos(model, data, joint_name, value)
            data.ctrl[actuator_ids[joint_name]] = value
    mujoco.mj_forward(model, data)


def _build_features() -> dict[str, dict]:
    return combine_feature_dicts(
        {
            "observation.image": {
                "type": "VISUAL",
                "shape": [3, CAM_HEIGHT, CAM_WIDTH],
                "names": ["channels", "height", "width"],
            },
            "observation.wrist_image": {
                "type": "VISUAL",
                "shape": [3, CAM_HEIGHT, CAM_WIDTH],
                "names": ["channels", "height", "width"],
            },
            "observation.state": {
                "type": "STATE",
                "shape": [len(JOINT_NAMES)],
                "names": list(JOINT_NAMES),
            },
            "observation.tactile": {
                "type": "STATE",
                "shape": [len(TACTILE_SENSOR_NAMES)],
                "names": list(TACTILE_SENSOR_NAMES),
            },
        },
        {
            "action": {
                "type": "ACTION",
                "shape": [len(JOINT_NAMES)],
                "names": list(JOINT_NAMES),
            }
        },
    )


def _collect_raw_obs(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    renderer: mujoco.Renderer,
    tactile_sensor_ids: dict[str, int],
) -> dict:
    obs = {
        "state": np.asarray([_get_joint_qpos(model, data, joint_name) for joint_name in JOINT_NAMES], dtype=np.float32),
        "tactile": np.asarray(
            [float(data.sensordata[model.sensor_adr[sid]]) for sid in tactile_sensor_ids.values()],
            dtype=np.float32,
        ),
    }
    for camera_name in CAMERAS:
        renderer.update_scene(data, camera=camera_name)
        image = renderer.render()
        obs[camera_name] = np.moveaxis(image, -1, 0).astype(np.float32) / 255.0
    return obs


def build_dataset_frame(features: dict, raw_obs: dict, prefix: str = "observation") -> dict:
    frame = {
        f"{prefix}.image": raw_obs["image"],
        f"{prefix}.wrist_image": raw_obs["wrist_image"],
        f"{prefix}.state": np.asarray(raw_obs["state"], dtype=np.float32),
        f"{prefix}.tactile": np.asarray(raw_obs["tactile"], dtype=np.float32),
    }
    return frame


def _build_policy_frame(raw_obs: dict, features: dict[str, dict], task: str) -> dict:
    frame = build_dataset_frame(features, raw_obs, prefix="observation")
    frame["task"] = task
    return frame


def _ensure_batched(batch: dict, device: torch.device) -> dict:
    for key, value in list(batch.items()):
        if key.startswith("observation.") and isinstance(value, np.ndarray):
            value = torch.from_numpy(np.ascontiguousarray(value))
            batch[key] = value
        if not isinstance(value, torch.Tensor):
            continue
        if key in ("observation.image", "observation.wrist_image") and value.ndim == 3:
            value = value.unsqueeze(0)
        elif key in ("observation.state", "observation.tactile") and value.ndim == 1:
            value = value.unsqueeze(0)
        elif key in ("observation.language.tokens", "observation.language.attention_mask") and value.ndim == 1:
            value = value.unsqueeze(0)
        batch[key] = value.to(device)
    return batch


def _load_policy(model_dir: Path, device: torch.device, vlm_model_dir: Path | None):
    config = PreTrainedConfig.from_pretrained(str(model_dir), local_files_only=True)
    config.device = str(device)
    if vlm_model_dir is not None:
        config.vlm_model_name = str(vlm_model_dir)

    policy = SmolVLAPolicy.from_pretrained(
        str(model_dir),
        config=config,
        local_files_only=True,
    )
    policy = policy.to(device).eval()

    preprocessor_overrides = {"device_processor": {"device": str(device)}}
    if vlm_model_dir is not None:
        preprocessor_overrides["tokenizer_processor"] = {"tokenizer_name": str(vlm_model_dir)}

    preprocess, postprocess = make_pre_post_processors(
        policy.config,
        str(model_dir),
        preprocessor_overrides=preprocessor_overrides,
    )
    return policy, preprocess, postprocess


def _clip_action_to_joint_ranges(model: mujoco.MjModel, action: dict[str, float]) -> dict[str, float]:
    clipped = {}
    for joint_name, value in action.items():
        joint_id = _joint_id(model, joint_name)
        if model.jnt_limited[joint_id]:
            lo, hi = model.jnt_range[joint_id]
            value = float(np.clip(value, lo, hi))
        clipped[joint_name] = float(value)
    return clipped


def _save_preview(raw_obs: dict, out_path: Path) -> None:
    images = []
    for name in CAMERAS:
        image = np.moveaxis(raw_obs[name], 0, -1)
        images.append(np.clip(image * 255.0, 0, 255).astype(np.uint8))
    preview = np.concatenate(images, axis=1)
    imageio.imwrite(out_path, preview)


def _camera_strip(raw_obs: dict) -> np.ndarray:
    images = []
    for name in CAMERAS:
        image = np.moveaxis(raw_obs[name], 0, -1)
        images.append(np.clip(image * 255.0, 0, 255).astype(np.uint8))
    return np.concatenate(images, axis=1)


def _save_trace_strip(raw_obs: dict, out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    imageio.imwrite(out_path, _camera_strip(raw_obs))
    return f"{out_path.parent.name}/{out_path.name}"


def _joint_state(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
    return {joint_name: _get_joint_qpos(model, data, joint_name) for joint_name in JOINT_NAMES}


def _joint_ranges(model: mujoco.MjModel) -> dict[str, tuple[float, float]]:
    ranges = {}
    for joint_name in JOINT_NAMES:
        joint_id = _joint_id(model, joint_name)
        if model.jnt_limited[joint_id]:
            lo, hi = model.jnt_range[joint_id]
        else:
            lo, hi = -np.pi, np.pi
        ranges[joint_name] = (float(lo), float(hi))
    return ranges


def _action_rows_from_tensor(action_tensor: torch.Tensor, features: dict[str, dict]) -> list[dict[str, float]]:
    action_names = features[ACTION_KEY]["names"]
    tensor = action_tensor.detach().to("cpu")
    if tensor.ndim == 3 and tensor.shape[0] == 1:
        tensor = tensor[0]
    elif tensor.ndim == 2 and tensor.shape[0] == 1:
        tensor = tensor[0].unsqueeze(0)
    elif tensor.ndim == 1:
        tensor = tensor.unsqueeze(0)
    elif tensor.ndim > 2:
        tensor = tensor.reshape(-1, tensor.shape[-1])

    rows = []
    for row in tensor:
        rows.append({name: float(row[i]) for i, name in enumerate(action_names)})
    return rows


def _postprocess_chunk_rows(
    raw_action: torch.Tensor,
    policy,
    postprocess,
    features: dict[str, dict],
) -> list[dict[str, float]] | None:
    queues = getattr(policy, "_queues", None)
    if not isinstance(queues, dict):
        return None
    queue = queues.get(ACTION_KEY)
    if queue is None:
        return None

    try:
        pieces = [raw_action.detach(), *(item.detach() for item in list(queue))]
        raw_chunk = torch.stack(pieces, dim=1)
        chunk = postprocess(raw_chunk)
        if not isinstance(chunk, torch.Tensor):
            return None
        return _action_rows_from_tensor(chunk, features)
    except Exception:
        return None


def _json_default(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return str(value)


def _write_trace_json(trace: dict, out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "trace.json"
    path.write_text(json.dumps(trace, indent=2, default=_json_default), encoding="utf-8")
    return path


def _pct(value: float, lo: float, hi: float) -> float:
    if hi <= lo:
        return 50.0
    return float(np.clip((value - lo) / (hi - lo), 0.0, 1.0) * 100.0)


def _html_bar(name: str, value: float | None, ranges: dict[str, tuple[float, float]]) -> str:
    escaped = html.escape(name)
    if value is None:
        return f'<div class="bar-row"><span>{escaped}</span><div class="track empty"></div><code>n/a</code></div>'

    lo, hi = ranges.get(name, (-np.pi, np.pi))
    pos = _pct(value, lo, hi)
    value_text = f"{value:.4f}"
    range_text = f"{lo:.2f} .. {hi:.2f}"
    return (
        '<div class="bar-row">'
        f"<span>{escaped}</span>"
        f'<div class="track" title="{html.escape(range_text)}">'
        f'<i style="left:{pos:.2f}%"></i>'
        "</div>"
        f"<code>{html.escape(value_text)}</code>"
        "</div>"
    )


def _html_bar_group(
    title: str,
    values: dict[str, float] | None,
    ranges: dict[str, tuple[float, float]],
) -> str:
    rows = "\n".join(_html_bar(name, None if values is None else values.get(name), ranges) for name in JOINT_NAMES)
    return f"<section class=\"panel\"><h3>{html.escape(title)}</h3>{rows}</section>"


def _chunk_svg(rows: list[dict[str, float]], ranges: dict[str, tuple[float, float]]) -> str:
    if not rows:
        return ""

    width = 760
    height = 220
    left = 34
    right = 12
    top = 16
    bottom = 36
    plot_w = width - left - right
    plot_h = height - top - bottom
    colors = ["#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c", "#0891b2"]
    n = len(rows)

    polylines = []
    for index, joint_name in enumerate(JOINT_NAMES):
        lo, hi = ranges[joint_name]
        points = []
        for step, row in enumerate(rows):
            x = left + (0 if n == 1 else step / (n - 1) * plot_w)
            y = top + (100.0 - _pct(row[joint_name], lo, hi)) / 100.0 * plot_h
            points.append(f"{x:.2f},{y:.2f}")
        color = colors[index % len(colors)]
        polylines.append(
            f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" '
            'stroke-width="2.2" stroke-linejoin="round" stroke-linecap="round" />'
        )

    legend = []
    for index, joint_name in enumerate(JOINT_NAMES):
        color = colors[index % len(colors)]
        x = left + index * 112
        legend.append(
            f'<g><rect x="{x}" y="{height - 22}" width="12" height="12" fill="{color}" rx="2" />'
            f'<text x="{x + 18}" y="{height - 12}" font-size="12">{html.escape(joint_name)}</text></g>'
        )

    grid = []
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = top + frac * plot_h
        grid.append(f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_w}" y2="{y:.2f}" class="grid" />')
    grid.append(f'<line x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" y2="{top + plot_h}" class="axis" />')
    grid.append(f'<text x="{left}" y="{height - 6}" font-size="12">0</text>')
    grid.append(f'<text x="{left + plot_w - 20}" y="{height - 6}" font-size="12">{n - 1}</text>')

    return (
        f'<svg viewBox="0 0 {width} {height}" class="chunk-chart" role="img" '
        'aria-label="Predicted action chunk">'
        f'{"".join(grid)}{"".join(polylines)}{"".join(legend)}'
        "</svg>"
    )


def _chunk_table(rows: list[dict[str, float]], max_rows: int) -> str:
    if not rows:
        return ""
    shown = rows[:max_rows]
    header = "<tr><th>idx</th>" + "".join(f"<th>{html.escape(name)}</th>" for name in JOINT_NAMES) + "</tr>"
    body = []
    for index, row in enumerate(shown):
        cells = [f"<td>{index}</td>"]
        cells.extend(f"<td>{row[name]:.4f}</td>" for name in JOINT_NAMES)
        body.append("<tr>" + "".join(cells) + "</tr>")
    omitted = len(rows) - len(shown)
    note = f"<p class=\"muted\">Showing first {len(shown)} of {len(rows)} actions.</p>"
    if omitted <= 0:
        note = f"<p class=\"muted\">Showing all {len(rows)} actions.</p>"
    return note + "<table>" + header + "".join(body) + "</table>"


def _stage_html(stages: list[dict[str, str]]) -> str:
    nodes = []
    for stage in stages:
        name = html.escape(stage["name"])
        detail = html.escape(stage["detail"])
        duration = stage.get("duration_ms")
        duration_html = ""
        if duration is not None:
            duration_html = f"<code>{float(duration):.1f} ms</code>"
        nodes.append(f'<div class="stage"><strong>{name}</strong><span>{detail}</span>{duration_html}</div>')
    return '<div class="stages">' + "\n".join(nodes) + "</div>"


def _write_trace_html(
    trace: dict,
    out_dir: Path,
    ranges: dict[str, tuple[float, float]],
    max_chunk_rows: int,
) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    meta = trace["meta"]
    steps = trace["steps"]
    title = "SmolVLA MuJoCo Trace"

    meta_rows = []
    for key, value in meta.items():
        if isinstance(value, (dict, list, tuple)):
            value_text = json.dumps(value, default=_json_default)
        else:
            value_text = str(value)
        meta_rows.append(
            f"<tr><th>{html.escape(str(key))}</th><td>{html.escape(value_text)}</td></tr>"
        )

    step_sections = []
    for step in steps:
        obs_src = html.escape(step.get("obs_image") or "")
        after_src = html.escape(step.get("after_image") or "")
        media_after = ""
        if after_src:
            media_after = (
                '<figure><figcaption>After MuJoCo step</figcaption>'
                f'<img src="{after_src}" alt="Post-action camera strip" /></figure>'
            )
        media = (
            '<div class="media-grid">'
            '<figure><figcaption>Observation cameras</figcaption>'
            f'<img src="{obs_src}" alt="Observation camera strip" /></figure>'
            f"{media_after}"
            "</div>"
        )

        chunk_rows = step.get("action_chunk") or []
        chunk = ""
        if chunk_rows:
            chunk = (
                '<section class="panel wide"><h3>Action Chunk Preview</h3>'
                f"{_chunk_svg(chunk_rows, ranges)}"
                f"{_chunk_table(chunk_rows, max_chunk_rows)}"
                "</section>"
            )

        action_panel = _html_bar_group("Predicted Joint Targets", step.get("action"), ranges)
        state_before_panel = _html_bar_group("Joint State Before", step.get("state_before"), ranges)
        state_after_panel = _html_bar_group("Joint State After", step.get("state_after"), ranges)

        badge = "new action chunk" if step.get("generated_chunk") else "queued action"
        if step.get("scene_check_only"):
            badge = "scene check"
        queue = (
            f"queue {step.get('queue_len_before', 'n/a')} -> "
            f"{step.get('queue_len_after', 'n/a')}"
        )
        step_sections.append(
            '<article class="step-card">'
            f'<header><h2>Step {step["step"]}</h2><span>{html.escape(badge)}</span><code>{html.escape(queue)}</code></header>'
            f"{media}"
            f"{_stage_html(step.get('stages', []))}"
            '<div class="panels">'
            f"{state_before_panel}{action_panel}{state_after_panel}"
            "</div>"
            f"{chunk}"
            "</article>"
        )

    css = """
        :root {
            color-scheme: light;
            --ink: #172033;
            --muted: #5f6f85;
            --line: #d7dde7;
            --panel: #ffffff;
            --band: #f5f7fb;
            --accent: #2563eb;
            --ok: #15803d;
        }
        * { box-sizing: border-box; }
        body {
            margin: 0;
            font: 14px/1.45 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
            color: var(--ink);
            background: var(--band);
        }
        main { max-width: 1180px; margin: 0 auto; padding: 28px 24px 48px; }
        h1 { font-size: 28px; margin: 0 0 6px; }
        h2 { font-size: 20px; margin: 0; }
        h3 { font-size: 14px; margin: 0 0 12px; text-transform: uppercase; letter-spacing: 0; color: var(--muted); }
        p { margin: 0 0 14px; }
        .muted { color: var(--muted); }
        .summary, .step-card {
            background: var(--panel);
            border: 1px solid var(--line);
            border-radius: 8px;
            box-shadow: 0 1px 2px rgba(12, 24, 48, 0.06);
        }
        .summary { padding: 18px; margin: 18px 0 20px; }
        table { width: 100%; border-collapse: collapse; font-size: 13px; }
        th, td { border-bottom: 1px solid var(--line); padding: 7px 8px; text-align: left; vertical-align: top; }
        th { color: var(--muted); font-weight: 600; width: 180px; }
        code {
            display: inline-block;
            padding: 2px 5px;
            border-radius: 5px;
            background: #edf2f7;
            color: #1f2937;
            font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
            font-size: 12px;
        }
        .step-card { margin: 18px 0; overflow: hidden; }
        .step-card > header {
            display: flex;
            gap: 10px;
            align-items: center;
            padding: 14px 16px;
            border-bottom: 1px solid var(--line);
            background: #fafbfd;
        }
        .step-card > header span {
            border: 1px solid #b6c5dc;
            color: #24415f;
            padding: 2px 8px;
            border-radius: 999px;
            font-size: 12px;
        }
        .media-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
            gap: 14px;
            padding: 16px;
        }
        figure { margin: 0; }
        figcaption { margin: 0 0 6px; color: var(--muted); font-size: 12px; }
        img {
            display: block;
            width: 100%;
            height: auto;
            border: 1px solid var(--line);
            border-radius: 6px;
            background: #0f172a;
        }
        .stages {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 8px;
            padding: 0 16px 16px;
        }
        .stage {
            min-height: 78px;
            padding: 10px;
            border: 1px solid var(--line);
            border-radius: 6px;
            background: #fbfcff;
        }
        .stage strong, .stage span { display: block; }
        .stage strong { margin-bottom: 4px; }
        .stage span { color: var(--muted); font-size: 12px; min-height: 34px; }
        .panels {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 14px;
            padding: 0 16px 16px;
        }
        .panel {
            border: 1px solid var(--line);
            border-radius: 6px;
            padding: 12px;
            background: #fff;
        }
        .panel.wide { margin: 0 16px 16px; overflow-x: auto; }
        .bar-row {
            display: grid;
            grid-template-columns: 64px 1fr 74px;
            gap: 8px;
            align-items: center;
            min-height: 28px;
        }
        .bar-row span { color: var(--muted); }
        .track {
            position: relative;
            height: 10px;
            border-radius: 999px;
            background: linear-gradient(90deg, #dbeafe, #fee2e2);
            border: 1px solid #cbd5e1;
        }
        .track.empty { background: #edf2f7; }
        .track i {
            position: absolute;
            top: -5px;
            width: 3px;
            height: 18px;
            transform: translateX(-50%);
            border-radius: 2px;
            background: var(--accent);
        }
        .chunk-chart { width: 100%; min-width: 700px; height: auto; }
        .chunk-chart .grid { stroke: #e2e8f0; stroke-width: 1; }
        .chunk-chart .axis { stroke: #94a3b8; stroke-width: 1; }
        @media (max-width: 720px) {
            main { padding: 18px 12px 32px; }
            .step-card > header { flex-wrap: wrap; }
            .bar-row { grid-template-columns: 58px 1fr 68px; }
        }
    """

    body = (
        "<!doctype html><html><head><meta charset=\"utf-8\" />"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />"
        f"<title>{html.escape(title)}</title><style>{css}</style></head><body><main>"
        f"<h1>{html.escape(title)}</h1>"
        "<p class=\"muted\">This report visualizes observable inference signals: MuJoCo camera inputs, "
        "language/task metadata, action chunk state, joint targets, and simulated robot response. "
        "It does not expose hidden model chain-of-thought.</p>"
        "<section class=\"summary\"><h3>Run Metadata</h3><table>"
        f"{''.join(meta_rows)}</table></section>"
        f"{''.join(step_sections)}"
        "</main></body></html>"
    )

    path = out_dir / "index.html"
    path.write_text(body, encoding="utf-8")
    return path


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run SmolVLA inference on smolvla_tactile_scene.xml.")
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML, help="MuJoCo XML scene path.")
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR, help="Local SmolVLA checkpoint.")
    parser.add_argument(
        "--vlm-model-dir",
        type=Path,
        default=DEFAULT_VLM_MODEL_DIR if DEFAULT_VLM_MODEL_DIR.exists() else None,
        help="Optional local HuggingFaceTB/SmolVLM2-500M-Video-Instruct directory.",
    )
    parser.add_argument("--task", default="Pick up the apple on the corner table.")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--steps", type=int, default=1, help="Number of policy steps to run.")
    parser.add_argument("--apply", action="store_true", help="Apply predicted joint targets to MuJoCo.")
    parser.add_argument("--sim-steps-per-action", type=int, default=20)
    parser.add_argument("--preview", action="store_true", help="Write a side-by-side RGB camera preview.")
    parser.add_argument("--preview-out", type=Path, default=HERE / "smolvla_preview.png")
    parser.add_argument(
        "--trace",
        action="store_true",
        help="Write an HTML report with observations, actions, timings, and MuJoCo rollout state.",
    )
    parser.add_argument("--trace-out", type=Path, default=HERE / "smolvla_trace")
    parser.add_argument(
        "--trace-max-chunk-rows",
        type=int,
        default=12,
        help="Maximum number of action chunk rows to show in the HTML table.",
    )
    parser.add_argument(
        "--scene-check-only",
        action="store_true",
        help="Compile the scene, render observations, and skip loading SmolVLA.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    xml = args.xml.expanduser().resolve()
    model_dir = args.model_dir.expanduser().resolve()
    vlm_model_dir = args.vlm_model_dir.expanduser().resolve() if args.vlm_model_dir else None
    trace_dir = args.trace_out.expanduser().resolve()
    trace_images_dir = trace_dir / "images"

    if not xml.exists():
        raise FileNotFoundError(f"Missing scene XML: {xml}. Run build_tactile_scene.py first.")
    if not model_dir.exists():
        raise FileNotFoundError(f"Missing SmolVLA checkpoint: {model_dir}")
    if vlm_model_dir is not None and not vlm_model_dir.exists():
        raise FileNotFoundError(f"Missing local VLM directory: {vlm_model_dir}")

    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but torch.cuda.is_available() is false.")

    features = _build_features()
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    actuator_ids = _actuator_ids(model)
    _reset_ready(model, data, actuator_ids)
    joint_ranges = _joint_ranges(model)

    renderer = mujoco.Renderer(model, CAM_HEIGHT, CAM_WIDTH)
    tactile_sensor_ids = _tactile_sensor_ids(model)
    trace: dict | None = None
    try:
        raw_obs = _collect_raw_obs(model, data, renderer, tactile_sensor_ids)
        if args.preview:
            _save_preview(raw_obs, args.preview_out)
            print(f"saved preview: {args.preview_out}")

        frame = _build_policy_frame(raw_obs, features, args.task)
        state_before = _joint_state(model, data)
        print(
            "scene ready: "
            f"nbody={model.nbody}, njnt={model.njnt}, nu={model.nu}, "
            f"state={frame['observation.state'].shape}, "
            f"image={frame['observation.image'].shape}, "
            f"wrist_image={frame['observation.wrist_image'].shape}, "
            f"tactile={frame['observation.tactile'].shape}"
        )

        if args.trace:
            trace = {
                "meta": {
                    "xml": str(xml),
                    "model_dir": str(model_dir),
                    "vlm_model_dir": str(vlm_model_dir) if vlm_model_dir is not None else None,
                    "task": args.task,
                    "device": str(device),
                    "steps": args.steps,
                    "apply": bool(args.apply),
                    "sim_steps_per_action": args.sim_steps_per_action,
                    "scene_check_only": bool(args.scene_check_only),
                    "chunk_size": None,
                    "n_action_steps": None,
                    "camera_names": list(CAMERAS),
                },
                "steps": [],
            }

        if args.scene_check_only:
            if trace is not None:
                trace["steps"].append(
                    {
                        "step": 0,
                        "scene_check_only": True,
                        "generated_chunk": False,
                        "queue_len_before": None,
                        "queue_len_after": None,
                        "obs_image": _save_trace_strip(raw_obs, trace_images_dir / "step_000_obs.png"),
                        "after_image": None,
                        "state_before": state_before,
                        "state_after": None,
                        "action": None,
                        "action_chunk": None,
                        "stages": [
                            {
                                "name": "compile",
                                "detail": "Load and validate the MuJoCo XML scene.",
                            },
                            {
                                "name": "render",
                                "detail": "Capture the two RGB camera observations and tactile vector.",
                            },
                        ],
                    }
                )
                _write_trace_json(trace, trace_dir)
                html_path = _write_trace_html(trace, trace_dir, joint_ranges, args.trace_max_chunk_rows)
                print(f"saved trace: {html_path}")
            return

        try:
            policy, preprocess, postprocess = _load_policy(model_dir, device, vlm_model_dir)
        except OSError as exc:
            raise SystemExit(
                "SmolVLA checkpoint is local, but the VLM backbone/tokenizer is not available. "
                f"Put HuggingFaceTB/SmolVLM2-500M-Video-Instruct at {DEFAULT_VLM_MODEL_DIR}, "
                "or pass --vlm-model-dir /path/to/SmolVLM2-500M-Video-Instruct. "
                f"Original error: {exc}"
            ) from exc
        policy.reset()
        if trace is not None:
            trace["meta"]["chunk_size"] = int(getattr(policy.config, "chunk_size", 0) or 0)
            trace["meta"]["n_action_steps"] = int(getattr(policy.config, "n_action_steps", 0) or 0)

        for step in range(args.steps):
            if step > 0:
                raw_obs = _collect_raw_obs(model, data, renderer, tactile_sensor_ids)
                frame = _build_policy_frame(raw_obs, features, args.task)
            state_before = _joint_state(model, data)
            queue_len_before = len(getattr(policy, "_queues", {}).get(ACTION_KEY, []))
            trace_step = {
                "step": step,
                "scene_check_only": False,
                "generated_chunk": False,
                "queue_len_before": queue_len_before,
                "queue_len_after": None,
                "obs_image": None,
                "after_image": None,
                "state_before": state_before,
                "state_after": None,
                "action": None,
                "action_chunk": None,
                "stages": [],
            }
            if trace is not None:
                trace_step["obs_image"] = _save_trace_strip(
                    raw_obs, trace_images_dir / f"step_{step:03d}_obs.png"
                )

            t0 = time.perf_counter()
            batch = _ensure_batched(preprocess(frame), device)
            preprocess_ms = (time.perf_counter() - t0) * 1000.0

            t1 = time.perf_counter()
            with torch.inference_mode():
                raw_action = policy.select_action(batch)
            policy_ms = (time.perf_counter() - t1) * 1000.0

            t2 = time.perf_counter()
            action = postprocess(raw_action)
            action = make_robot_action(action, features)
            action = _clip_action_to_joint_ranges(model, action)
            postprocess_ms = (time.perf_counter() - t2) * 1000.0

            chunk_rows = None
            if trace is not None and queue_len_before == 0:
                chunk_rows = _postprocess_chunk_rows(raw_action, policy, postprocess, features)
                trace_step["generated_chunk"] = True
                trace_step["action_chunk"] = chunk_rows
            trace_step["queue_len_after"] = len(getattr(policy, "_queues", {}).get(ACTION_KEY, []))
            trace_step["action"] = action

            print(f"step {step}:")
            for joint_name in JOINT_NAMES:
                print(f"  {joint_name}: {action[joint_name]: .6f}")

            sim_ms = 0.0
            if args.apply:
                t3 = time.perf_counter()
                for joint_name, value in action.items():
                    data.ctrl[actuator_ids[joint_name]] = value
                for _ in range(args.sim_steps_per_action):
                    mujoco.mj_step(model, data)
                sim_ms = (time.perf_counter() - t3) * 1000.0
                if trace is not None:
                    after_obs = _collect_raw_obs(model, data, renderer, tactile_sensor_ids)
                    trace_step["after_image"] = _save_trace_strip(
                        after_obs, trace_images_dir / f"step_{step:03d}_after.png"
                    )
                    trace_step["state_after"] = _joint_state(model, data)
            else:
                trace_step["state_after"] = None

            trace_step["stages"] = [
                {
                    "name": "render",
                    "detail": "Capture MuJoCo camera inputs and joint state.",
                },
                {
                    "name": "preprocess",
                    "detail": "Build the policy frame and batch tensors.",
                    "duration_ms": preprocess_ms,
                },
                {
                    "name": "policy",
                    "detail": "Generate or dequeue one SmolVLA action.",
                    "duration_ms": policy_ms,
                },
                {
                    "name": "postprocess",
                    "detail": "Unnormalize the action and map it to named joints.",
                    "duration_ms": postprocess_ms,
                },
                {
                    "name": "mujoco",
                    "detail": "Apply the action and step the simulator." if args.apply else "Action not applied.",
                    "duration_ms": sim_ms if args.apply else None,
                },
            ]
            if trace is not None:
                trace["steps"].append(trace_step)

        if trace is not None:
            trace_json = _write_trace_json(trace, trace_dir)
            html_path = _write_trace_html(trace, trace_dir, joint_ranges, args.trace_max_chunk_rows)
            print(f"saved trace json: {trace_json}")
            print(f"saved trace html: {html_path}")
    finally:
        close = getattr(renderer, "close", None)
        if callable(close):
            close()


if __name__ == "__main__":
    main()
