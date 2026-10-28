"""Containment of a protective stop (A20, ruled 2026-10-18). Pure.

The safety property: everything the footprint sweeps from the stop trigger to rest lies inside the
field in force at the trigger. And its consequence for recovery: no stop ends with anything inside the
minimum margin (the at-rest field). These read a recorded run -- emulated or simulated -- given, per
stop, the decision pose at the trigger, the field in force there (as a function of bearing, in that
body frame), and the poses from the trigger to rest.
"""
from __future__ import annotations

import math
from collections.abc import Callable, Sequence

import numpy as np
from numpy.typing import ArrayLike


def footprint_boundary(extent: Callable[[np.ndarray], np.ndarray], n: int = 360) -> np.ndarray:
    """(n, 2) body-frame boundary points of a star-shaped footprint, m."""
    psi = np.linspace(-math.pi, math.pi, n, endpoint=False)
    e = np.asarray(extent(psi), dtype=float)
    return np.column_stack([e * np.cos(psi), e * np.sin(psi)])


def in_frame(points: ArrayLike, pose: ArrayLike, frame: ArrayLike) -> np.ndarray:
    """Body-frame `points` of a body at world `pose` (x, y, yaw), expressed in the body frame of
    world pose `frame`."""
    p = np.asarray(points, dtype=float)
    x, y, th = (float(v) for v in np.asarray(pose, dtype=float).ravel()[:3])
    fx, fy, fth = (float(v) for v in np.asarray(frame, dtype=float).ravel()[:3])
    c, s = math.cos(th), math.sin(th)
    world = np.column_stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]])
    dx, dy = world[:, 0] - fx, world[:, 1] - fy
    cf, sf = math.cos(fth), math.sin(fth)
    return np.column_stack([cf * dx + sf * dy, -sf * dx + cf * dy])


def containment_excess(boundary: np.ndarray, poses: Sequence[ArrayLike], frame: ArrayLike,
                       field: Callable[[np.ndarray], np.ndarray]) -> float:
    """How far, radially, the footprint swept through `poses` reaches beyond the field in force at
    `frame`, m: max over every boundary point of (range - field reach along its bearing). <= 0 means
    contained. `field(bearings)` is the boundary range per bearing in the frame's body frame."""
    pts = np.vstack([in_frame(boundary, pose, frame) for pose in poses])
    r = np.hypot(pts[:, 0], pts[:, 1])
    b = np.arctan2(pts[:, 1], pts[:, 0])
    return float(np.max(r - np.asarray(field(b), dtype=float)))


def rest_margin(ranges: ArrayLike, valid: ArrayLike, rest_field: ArrayLike) -> float:
    """At rest: the smallest (range - at-rest field) over the bins, m; < 0 means the stop ended
    with something inside the minimum margin. An unobserved bin counts as -inf (hard rule 5)."""
    r = np.asarray(ranges, dtype=float)
    f = np.asarray(rest_field, dtype=float)
    ok = np.asarray(valid, dtype=bool)
    return float(np.min(np.where(ok, r - f, -np.inf)))
