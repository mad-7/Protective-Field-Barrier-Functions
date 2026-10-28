"""The route ahead in the robot's body frame, for the classical controllers' typed input.

Environment-side on purpose: the transform needs the pose, which a controller never gets
(`pfbnav/envs/controller_input.py`, ruled 2026-10-16). Kept out of `pfbnav/planning/lookahead.py`,
which is inside the scenario identity's import closure (A18), so that adding it moved no lock.
Body frame: +x forward, +y left, m.
"""
from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike

from pfbnav.planning.lookahead import arc_lengths, point_at


def world_to_body(points_world: ArrayLike, pose: ArrayLike) -> np.ndarray:
    """World points (n, 2) in the body frame of `pose` (x, y, yaw)."""
    p = np.atleast_2d(np.asarray(points_world, dtype=float))[:, :2]
    x, y, yaw = (float(v) for v in np.asarray(pose, dtype=float).ravel()[:3])
    c, s = math.cos(yaw), math.sin(yaw)
    d = p - np.array([x, y])
    return np.column_stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]])


def remaining_route(path: ArrayLike, s: float) -> np.ndarray:
    """The polyline from arc length `s` to the path's end, world frame: the point at `s`, then
    every vertex beyond it (at least two points; the end twice if `s` is at the end)."""
    p = np.asarray(path, dtype=float)
    cum = arc_lengths(p)
    s = float(np.clip(s, 0.0, cum[-1]))
    out = np.vstack([point_at(p, s)[None, :], p[cum > s + 1e-12]])
    return out if len(out) >= 2 else np.vstack([out, out[-1:]])
