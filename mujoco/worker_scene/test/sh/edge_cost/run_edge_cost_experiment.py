#!/usr/bin/env python3.8
"""Reproducible A/B/C edge-cost experiment on the saved warehouse map.

The planner is exercised without starting the ROS navigation loop.  The map,
grid conversion and path semantics are the same as NavP2P; the output plot is
an RViz-equivalent view of planning_map + nav_path for headless environments.
"""
import csv
import json
import math
import os
import sys
from types import SimpleNamespace

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "../../.."))
sys.path.insert(0, ROOT)

from nav_p2p import (  # noqa: E402
    HARD_CLEARANCE,
    INFLATE,
    LAMBDA_GEO,
    NavP2P,
    PREFERRED_CLEARANCE,
    ROBOT_RADIUS,
    _inflate_occupancy,
    _pgm_to_occupancy,
    _read_ply_xyz,
    _height_clearance_from_points,
    compute_height_cost,
    HARD_HEIGHT,
    PREFERRED_HEIGHT,
)

MAP_DIR = os.path.join(ROOT, "maps", "warehouse")
RESULT_DIR = os.path.join(ROOT, "test", "result", "edge_cost")
PGM = os.path.join(MAP_DIR, "warehouse_map_3d.pgm")
YAML = os.path.join(MAP_DIR, "warehouse_map_3d.yaml")

# Fixed map-frame endpoints chosen across the warehouse diagonal.  The
# segment passes the work-table region near (8.5, 8.5), making clearance and
# height penalties visible instead of producing three coincident straight paths.
START_WORLD = (18.00, 1.00)
GOAL_WORLD = (10.00, 10.00)


def _yaml():
    values = {}
    with open(YAML, encoding="utf-8") as stream:
        for line in stream:
            if ":" in line and not line.lstrip().startswith("#"):
                key, value = line.split(":", 1)
                values[key.strip()] = value.strip()
    origin = [float(v) for v in values["origin"].strip("[]").split(",")]
    return float(values["resolution"]), origin


def _load_grid():
    resolution, origin = _yaml()
    image = np.asarray(Image.open(PGM), dtype=np.uint8)
    image = np.flipud(image)
    occupancy = _pgm_to_occupancy(image, 0.65, 0.25, 0)
    cost, clearance = _inflate_occupancy(
        occupancy.ravel(), image.shape[0], image.shape[1], resolution,
        ROBOT_RADIUS + INFLATE)
    return occupancy, cost, clearance, resolution, origin


def _grid(world, resolution, origin, cost):
    x = int(math.floor((world[0] - origin[0]) / resolution))
    y = int(math.floor((world[1] - origin[1]) / resolution))
    h, w = cost.shape
    if not (0 <= x < w and 0 <= y < h):
        raise ValueError("endpoint outside map: %s" % (world,))
    return x, y


def _nearest_free(point, cost):
    if cost[point[1], point[0]] <= 0:
        return point
    h, w = cost.shape
    for radius in range(1, max(h, w)):
        for y in range(max(0, point[1] - radius), min(h, point[1] + radius + 1)):
            for x in range(max(0, point[0] - radius), min(w, point[0] + radius + 1)):
                if cost[y, x] <= 0:
                    return x, y
    raise ValueError("no free endpoint near %s" % (point,))


def _planner(cost, clearance, resolution, lambda_geo, height_clearance=None,
             lambda_height=0.0):
    planner = NavP2P.__new__(NavP2P)
    planner.cost = cost
    planner.clearance = clearance
    planner.traversable = cost <= 0.0
    planner.grid_shape = cost.shape
    planner.lambda_geo = lambda_geo
    planner.lambda_height = lambda_height
    planner.hard_height = HARD_HEIGHT
    planner.height_clearance = height_clearance
    planner.height_cost = (np.zeros(cost.shape, dtype=np.float64)
                           if height_clearance is None else
                           np.vectorize(compute_height_cost)(height_clearance))
    if height_clearance is not None:
        planner.traversable &= ((~np.isfinite(height_clearance)) |
                                (height_clearance > HARD_HEIGHT))
    planner.map = SimpleNamespace(
        info=SimpleNamespace(resolution=resolution))
    return planner


def _path_metrics(path, clearance, resolution):
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                 for a, b in zip(path, path[1:])) * resolution
    values = [float(clearance[y, x]) for x, y in path]
    return length, min(values), sum(values) / len(values)


def _path_height_profile(path, height_clearance, resolution):
    distance = [0.0]
    values = [float(height_clearance[path[0][1], path[0][0]])]
    for a, b in zip(path, path[1:]):
        distance.append(distance[-1] + math.hypot(b[0] - a[0], b[1] - a[1]) * resolution)
        values.append(float(height_clearance[b[1], b[0]]))
    return np.asarray(distance), np.asarray(values)


def _save_height_visuals(height_clearance, height_cost, occupancy, origin,
                         resolution, paths):
    extent = (origin[0], origin[0] + occupancy.shape[1] * resolution,
              origin[1], origin[1] + occupancy.shape[0] * resolution)
    layers = ((height_clearance, "height_clearance.png", "Height clearance (m)", "viridis", 0.0, 1.5),
              (height_cost, "height_cost.png", "Height soft cost (0..1)", "magma", 0.0, 1.0),
              ((np.isfinite(height_clearance) & (height_clearance <= HARD_HEIGHT)).astype(float),
               "height_blocked.png", "Hard height obstacle mask", "Reds", 0.0, 1.0))
    for data, name, title, cmap, vmin, vmax in layers:
        fig, ax = plt.subplots(figsize=(12, 10), dpi=150)
        shown = np.ma.masked_invalid(data) if name == "height_clearance.png" else data
        ax.imshow(shown, origin="lower", extent=extent, cmap=cmap, vmin=vmin, vmax=vmax)
        ax.set_title(title); ax.set_xlabel("map x (m)"); ax.set_ylabel("map y (m)"); ax.set_aspect("equal")
        fig.colorbar(ax.images[0], ax=ax); fig.tight_layout()
        fig.savefig(os.path.join(RESULT_DIR, name)); plt.close(fig)
    fig, ax = plt.subplots(figsize=(10, 5), dpi=150)
    for name, path in paths.items():
        if path:
            d, h = _path_height_profile(path, height_clearance, resolution)
            ax.plot(d, h, linewidth=2, label=name)
    ax.axhline(HARD_HEIGHT, color="red", linestyle="--", label="hard_height")
    ax.axhline(PREFERRED_HEIGHT, color="green", linestyle=":", label="preferred_height")
    ax.set(title="Vertical clearance along planned paths", xlabel="distance (m)", ylabel="clearance (m)")
    ax.grid(alpha=0.25); ax.legend(fontsize=8); fig.tight_layout()
    fig.savefig(os.path.join(RESULT_DIR, "path_height_profile.png")); plt.close(fig)
    fig, ax = plt.subplots(figsize=(12, 10), dpi=150)
    # _load_grid already converts PGM top-left storage to map bottom-left
    # storage, so do not flip a second time here.
    ax.imshow(occupancy, cmap="gray", vmin=0, vmax=100, origin="lower", extent=extent)
    colors = {"height_lambda_0": "#2563eb", "height_lambda_1": "#16a34a", "height_lambda_2": "#dc2626"}
    for name, path in paths.items():
        if path and name.startswith("height_lambda"):
            xy = np.asarray([(origin[0] + (x + .5) * resolution, origin[1] + (y + .5) * resolution) for x, y in path])
            ax.plot(xy[:, 0], xy[:, 1], color=colors[name], linewidth=2, label=name)
    ax.set_title("Height-cost path comparison"); ax.set_xlabel("map x (m)"); ax.set_ylabel("map y (m)"); ax.set_aspect("equal"); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(RESULT_DIR, "height_paths_comparison.png")); plt.close(fig)


def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    occupancy, cost, clearance, resolution, origin = _load_grid()
    points = _read_ply_xyz(os.path.join(MAP_DIR, "warehouse_map_3d_cloud.ply"))
    height_clearance = _height_clearance_from_points(
        points, cost.shape[0], cost.shape[1], resolution, (origin[0], origin[1]))
    height_cost = np.zeros(cost.shape, dtype=np.float64)
    finite = np.isfinite(height_clearance) & (height_clearance < PREFERRED_HEIGHT)
    height_cost[finite] = np.vectorize(compute_height_cost)(height_clearance[finite])
    start = _nearest_free(_grid(START_WORLD, resolution, origin, cost), cost)
    goal = _nearest_free(_grid(GOAL_WORLD, resolution, origin, cost), cost)

    # A is the pre-change distance reference; B is the unified-interface
    # distance mode.  Both must agree when lambda_geo=0.
    cases = [("A_baseline_reference", 0.0),
             ("B_unified_distance", 0.0),
             ("C_safety_aware", 2.0)]
    records = []
    paths = {}
    for name, weight in cases:
        planner = _planner(cost, clearance, resolution, weight)
        path = planner._astar(start[0], start[1], goal[0], goal[1])
        stats = dict(planner.last_plan_stats)
        if path is not None:
            length, min_c, mean_c = _path_metrics(path, clearance, resolution)
            paths[name] = path
        else:
            length, min_c, mean_c = (float("nan"),) * 3
        stats.update(case=name, lambda_geo=weight, start=list(start), goal=list(goal),
                     path_length_m=length, minimum_clearance_m=min_c,
                     mean_clearance_m=mean_c)
        records.append(stats)

    height_paths = {}
    height_records = []
    for weight in (0.0, 1.0, 2.0):
        name = "height_lambda_%g" % weight
        planner = _planner(cost, clearance, resolution, 0.0,
                           height_clearance, weight)
        path = planner._astar(start[0], start[1], goal[0], goal[1])
        height_paths[name] = path
        stats = dict(planner.last_plan_stats, case=name,
                     lambda_height=weight, start=list(start), goal=list(goal))
        if path:
            length, min_c, mean_c = _path_metrics(path, clearance, resolution)
            _, profile = _path_height_profile(path, height_clearance, resolution)
            stats.update(path_length_m=length, minimum_clearance_m=min_c,
                         mean_clearance_m=mean_c,
                         minimum_height_clearance_m=float(np.min(profile)))
        else:
            stats.update(path_length_m=float("nan"), minimum_height_clearance_m=float("nan"))
        height_records.append(stats)

    with open(os.path.join(RESULT_DIR, "metrics.json"), "w", encoding="utf-8") as stream:
        json.dump({"map": PGM, "resolution": resolution,
                   "origin": origin, "start_world": START_WORLD,
                   "goal_world": GOAL_WORLD, "start_grid": start,
                   "goal_grid": goal, "hard_clearance_m": HARD_CLEARANCE,
                   "preferred_clearance_m": PREFERRED_CLEARANCE,
                   "cases": records, "height_cases": height_records,
                   "hard_height_m": HARD_HEIGHT,
                   "preferred_height_m": PREFERRED_HEIGHT}, stream, indent=2)
    fields = ["case", "lambda_geo", "success", "planning_time_ms",
              "path_length_m", "minimum_clearance_m", "mean_clearance_m",
              "expanded_nodes", "los_checks", "los_cells_checked",
              "waypoint_count"]
    with open(os.path.join(RESULT_DIR, "metrics.csv"), "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({key: record.get(key) for key in fields} for record in records)

    # RViz-equivalent path view: grayscale planning map and map-frame paths.
    fig, ax = plt.subplots(figsize=(12, 10), dpi=150)
    ax.imshow(occupancy, cmap="gray", vmin=0, vmax=100,
              origin="lower", extent=(origin[0], origin[0] + occupancy.shape[1] * resolution,
                                       origin[1], origin[1] + occupancy.shape[0] * resolution))
    styles = [("A_baseline_reference", "#2563eb", "--"),
              ("B_unified_distance", "#16a34a", "-"),
              ("C_safety_aware", "#dc2626", "-")]
    for name, color, linestyle in styles:
        path = paths.get(name)
        if not path:
            continue
        xy = np.asarray([(origin[0] + (x + 0.5) * resolution,
                          origin[1] + (y + 0.5) * resolution) for x, y in path])
        ax.plot(xy[:, 0], xy[:, 1], linestyle=linestyle, color=color,
                linewidth=2.0, label=name)
    ax.scatter([START_WORLD[0]], [START_WORLD[1]], c="cyan", edgecolors="black",
               s=55, zorder=5, label="start")
    ax.scatter([GOAL_WORLD[0]], [GOAL_WORLD[1]], c="magenta", edgecolors="black",
               s=55, zorder=5, label="goal")
    ax.set_title("Warehouse Lazy Theta* edge-cost comparison (RViz-equivalent)")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(RESULT_DIR, "paths_comparison.png"))
    plt.close(fig)

    _save_height_visuals(height_clearance, height_cost, occupancy, origin,
                         resolution, dict(paths, **height_paths))

    with open(os.path.join(RESULT_DIR, "screenshot_note.txt"), "w", encoding="utf-8") as stream:
        stream.write("No DISPLAY/RViz session was available during this run.\n")
        stream.write("paths_comparison.png is an equivalent map-frame view of planning_map + nav_path.\n")
    print(json.dumps({"start_grid": start, "goal_grid": goal, "cases": records}, indent=2))


if __name__ == "__main__":
    main()
