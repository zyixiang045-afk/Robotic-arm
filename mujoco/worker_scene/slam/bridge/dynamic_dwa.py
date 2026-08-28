#!/usr/bin/env python3
"""Conditional holonomic DWA helpers, independent of ROS message types."""

from dataclasses import dataclass
import math

import numpy as np


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def point_segment_distances(points, start, end):
    """Return the distance from each 2D point to a finite segment."""
    points = np.asarray(points, dtype=np.float64)
    if points.size == 0:
        return np.empty(0, dtype=np.float64)
    start = np.asarray(start, dtype=np.float64)
    delta = np.asarray(end, dtype=np.float64) - start
    denom = float(np.dot(delta, delta))
    if denom < 1e-12:
        return np.linalg.norm(points - start, axis=1)
    ratio = np.clip(((points - start) @ delta) / denom, 0.0, 1.0)
    projection = start + ratio[:, None] * delta
    return np.linalg.norm(points - projection, axis=1)


def path_corridor_clearance(points, pose_xy, path, path_index,
                            forward_distance=3.0):
    """Minimum obstacle distance to the upcoming part of a sparse path."""
    points = np.asarray(points, dtype=np.float64)
    if points.size == 0 or not path:
        return float("inf")
    index = max(0, min(int(path_index), len(path) - 1))
    start = np.asarray(pose_xy, dtype=np.float64)
    remaining = max(0.0, float(forward_distance))
    best = float("inf")
    while remaining > 1e-9 and index < len(path):
        waypoint = np.asarray(path[index], dtype=np.float64)
        length = float(np.linalg.norm(waypoint - start))
        if length > remaining and length > 1e-12:
            end = start + (waypoint - start) * (remaining / length)
            remaining = 0.0
        else:
            end = waypoint
            remaining -= length
            index += 1
        distances = point_segment_distances(points, start, end)
        if distances.size:
            best = min(best, float(distances.min()))
        start = end
    return best


def path_lookahead(pose_xy, path, path_index, distance=1.2):
    """Interpolate a local target along a sparse global path."""
    if not path:
        return None
    index = max(0, min(int(path_index), len(path) - 1))
    start = np.asarray(pose_xy, dtype=np.float64)
    remaining = max(0.0, float(distance))
    while index < len(path):
        waypoint = np.asarray(path[index], dtype=np.float64)
        length = float(np.linalg.norm(waypoint - start))
        if length >= remaining and length > 1e-12:
            target = start + (waypoint - start) * (remaining / length)
            return float(target[0]), float(target[1])
        remaining -= length
        start = waypoint
        index += 1
    return tuple(path[-1])


@dataclass(frozen=True)
class DWAConfig:
    prediction_time: float = 1.8
    time_step: float = 0.12
    max_forward: float = 0.45
    max_reverse: float = 0.10
    max_lateral: float = 0.35
    max_yaw_rate: float = 0.90
    dynamic_clearance: float = 0.47
    path_weight: float = 2.2
    goal_weight: float = 3.0
    yaw_weight: float = 0.35
    obstacle_weight: float = 0.55
    smooth_weight: float = 0.45
    speed_weight: float = 0.75


class HolonomicDWA:
    """Sample constant body-frame velocities over a short local horizon."""

    def __init__(self, config=None):
        self.config = config or DWAConfig()

    def _samples(self):
        cfg = self.config
        vx = np.asarray([-cfg.max_reverse, 0.0, 0.12, 0.26,
                         cfg.max_forward])
        vy = np.linspace(-cfg.max_lateral, cfg.max_lateral, 7)
        wz = np.linspace(-cfg.max_yaw_rate, cfg.max_yaw_rate, 7)
        for x in vx:
            for y in vy:
                for yaw_rate in wz:
                    yield float(x), float(y), float(yaw_rate)

    def _simulate(self, pose, command):
        cfg = self.config
        steps = max(1, int(math.ceil(cfg.prediction_time / cfg.time_step)))
        result = np.empty((steps, 3), dtype=np.float64)
        x, y, yaw = pose
        vx, vy, wz = command
        for index in range(steps):
            c, s = math.cos(yaw), math.sin(yaw)
            x += (c * vx - s * vy) * cfg.time_step
            y += (s * vx + c * vy) * cfg.time_step
            yaw = wrap_angle(yaw + wz * cfg.time_step)
            result[index] = (x, y, yaw)
        return result

    @staticmethod
    def _path_distance(point, path, path_index):
        if not path:
            return 0.0
        first = max(0, int(path_index) - 1)
        last = min(len(path) - 1, int(path_index) + 4)
        if first >= last:
            return math.hypot(point[0] - path[last][0],
                              point[1] - path[last][1])
        sample = np.asarray([point], dtype=np.float64)
        return min(float(point_segment_distances(
            sample, path[index], path[index + 1])[0])
                   for index in range(first, last))

    def plan(self, pose, local_goal, path, path_index, dynamic_points,
             static_clearance, previous_command=(0.0, 0.0, 0.0)):
        """Return the safest scored command, or None if even stopping is unsafe.

        ``static_clearance(x, y)`` returns metric clearance on the already
        inflated static map, and a negative value outside its traversable area.
        Dynamic points are obstacle surface returns in map coordinates.
        """
        cfg = self.config
        obstacles = np.asarray(dynamic_points, dtype=np.float64).reshape(-1, 2)
        previous = np.asarray(previous_command, dtype=np.float64)
        goal_heading = math.atan2(local_goal[1] - pose[1],
                                  local_goal[0] - pose[0])
        best = None
        best_score = float("inf")
        for command in self._samples():
            trajectory = self._simulate(pose, command)
            static_values = np.asarray(
                [static_clearance(x, y) for x, y in trajectory[:, :2]],
                dtype=np.float64)
            if np.any(static_values < 0.0):
                continue

            dynamic_min = float("inf")
            if obstacles.size:
                delta = trajectory[:, None, :2] - obstacles[None, :, :]
                dynamic_min = float(np.sqrt(np.sum(delta * delta, axis=2)).min())
                if dynamic_min < cfg.dynamic_clearance:
                    continue

            end = trajectory[-1]
            goal_cost = math.hypot(end[0] - local_goal[0],
                                   end[1] - local_goal[1])
            path_cost = self._path_distance(end[:2], path, path_index)
            yaw_cost = abs(wrap_angle(goal_heading - end[2]))
            obstacle_cost = 0.0
            if math.isfinite(dynamic_min):
                obstacle_cost = 1.0 / max(dynamic_min, 0.05)
            smooth_cost = float(np.linalg.norm(
                np.asarray(command, dtype=np.float64) - previous))
            progress_speed = math.hypot(command[0], command[1])
            score = (cfg.goal_weight * goal_cost
                     + cfg.path_weight * path_cost
                     + cfg.yaw_weight * yaw_cost
                     + cfg.obstacle_weight * obstacle_cost
                     + cfg.smooth_weight * smooth_cost
                     - cfg.speed_weight * progress_speed)
            if score < best_score:
                best_score = score
                best = command
        return best
