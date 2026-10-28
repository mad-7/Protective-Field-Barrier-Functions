"""Tracking a planned polyline: arc-length projection, lookahead, body-frame encoding.

Pure functions, plus one small object that remembers how far along the path the robot has
come. No simulator, no configuration read here.

**Why the projection is monotonic.** The robot's nearest point on a path is ambiguous
wherever the path doubles back or crosses itself: a warehouse route around a shelf passes
close to its own earlier leg. Taking the globally nearest point lets the tracker jump
backwards, which reads as a large negative progress and, to a policy paid for progress, as
an instruction to drive in circles. `PathTracker` searches forward from where it already
was, so progress is the arc length gained and nothing else.

**What the observation gets** (the layout ruled 2026-09-23): the lookahead point and the
goal, each as `(distance, sin bearing, cos bearing)` in the body frame. Distance and angle
separately, and the angle as a pair, because a bearing fed as one number jumps by 2 pi
across the seam while the pair is continuous everywhere.

**What the reward gets:** arc length along the polyline, `s`. The progress term is defined
on the projection (the ruling), not on straight-line distance to the goal -- a robot that
rounds a shelf earns progress for following the plan rather than losing it for moving
sideways relative to the goal.

Frames: paths and poses in world metres and radians; the encoded bearings are body-frame,
+x forward, +y left, in (-pi, pi].
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike


@dataclass(frozen=True)
class Projection:
    """Where a point falls on a polyline."""

    s: float                  # arc length from the path's start, m
    distance: float           # distance from the point to the path, m (cross-track)
    segment: int              # index of the segment the projection lies on
    point: np.ndarray         # the projected point in world metres


@dataclass(frozen=True)
class TrackerState:
    """One update's view of the path, ready for the observation and the reward."""

    s: float                  # arc length reached, m
    progress: float           # arc length gained since the previous update, m
    cross_track: float        # distance from the path, m
    lookahead_world: np.ndarray
    goal_world: np.ndarray
    lookahead_body: tuple[float, float, float]   # (d, sin beta, cos beta)
    goal_body: tuple[float, float, float]
    remaining: float          # arc length from s to the path's end, m
    at_end: bool              # the lookahead has reached the final waypoint


def arc_lengths(path: ArrayLike) -> np.ndarray:
    """Cumulative arc length at each waypoint, m; `[0]` at the first."""
    p = _as_path(path)
    steps = np.linalg.norm(np.diff(p, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(steps)])


def point_at(path: ArrayLike, s: float) -> np.ndarray:
    """The point at arc length `s`, m, clamped to the path's ends."""
    p = _as_path(path)
    cum = arc_lengths(p)
    s = float(np.clip(s, 0.0, cum[-1]))
    i = int(np.searchsorted(cum, s, side="right") - 1)
    i = min(max(i, 0), len(p) - 2)
    span = cum[i + 1] - cum[i]
    t = 0.0 if span <= 0.0 else (s - cum[i]) / span
    return p[i] + t * (p[i + 1] - p[i])


def project(path: ArrayLike, point: ArrayLike, *, from_s: float = 0.0,
            window: float | None = None) -> Projection:
    """Nearest point on the polyline within `[from_s, from_s + window]` of arc length.

    Searching forward keeps progress monotonic where a path passes near itself; the window
    keeps it from LEAPING there. On a hairpin whose legs are 0.4 m apart the return leg can
    be the nearest point on the whole remaining path, and without a window a robot that
    moved a centimetre would be projected 7 m further along and paid for it (measured by
    `tests/test_lookahead.py`, 2026-09-23). `window=None` searches to the end, which is
    what a fresh projection with no history wants.
    """
    p = _as_path(path)
    xy = np.asarray(point, dtype=float).ravel()[:2]
    cum = arc_lengths(p)
    limit = math.inf if window is None else from_s + float(window)
    best: Projection | None = None
    for i in range(len(p) - 1):
        if cum[i + 1] < from_s or cum[i] > limit:
            continue                      # behind us, or beyond the window
        a, b = p[i], p[i + 1]
        ab = b - a
        length = float(np.linalg.norm(ab))
        if length <= 0.0:
            continue
        t = float(np.clip(np.dot(xy - a, ab) / (length * length), 0.0, 1.0))
        s = float(cum[i] + t * length)
        if s < from_s:                    # the nearest part of this segment is behind us
            s = float(from_s)
            t = (s - cum[i]) / length
        elif s > limit:                   # ... or beyond the window
            s = float(limit)
            t = (s - cum[i]) / length
        proj = a + t * ab
        distance = float(np.linalg.norm(xy - proj))
        if best is None or distance < best.distance - 1e-12:
            best = Projection(s=s, distance=distance, segment=i, point=proj)
    if best is None:                      # `from_s` is at or past the end
        end = p[-1]
        return Projection(s=float(cum[-1]), distance=float(np.linalg.norm(xy - end)),
                          segment=len(p) - 2, point=end)
    return best


def to_body(point_world: ArrayLike, pose: ArrayLike) -> tuple[float, float, float]:
    """A world point as (distance, sin bearing, cos bearing) in the body frame.

    `pose` is (x, y, yaw). At zero distance the bearing is undefined and is reported as
    (0, 1) -- straight ahead -- rather than a NaN a network would propagate.
    """
    p = np.asarray(point_world, dtype=float).ravel()[:2]
    x, y, yaw = (float(v) for v in np.asarray(pose, dtype=float).ravel()[:3])
    dx, dy = p[0] - x, p[1] - y
    c, s = math.cos(-yaw), math.sin(-yaw)
    bx, by = c * dx - s * dy, s * dx + c * dy
    distance = math.hypot(bx, by)
    if distance <= 1e-12:
        return 0.0, 0.0, 1.0
    return distance, by / distance, bx / distance


class PathTracker:
    """Follows one path: monotonic arc length, lookahead point, encoded observation."""

    def __init__(self, path: ArrayLike, lookahead: float, window: float = 1.0):
        self.path = _as_path(path)
        if lookahead <= 0.0:
            raise ValueError("the lookahead distance must be positive")
        if window <= 0.0:
            raise ValueError("the search window must be positive")
        self.lookahead = float(lookahead)
        self.window = float(window)
        self.cum = arc_lengths(self.path)
        self.s = 0.0

    @property
    def length(self) -> float:
        return float(self.cum[-1])

    def reset(self, s: float = 0.0) -> None:
        self.s = float(np.clip(s, 0.0, self.length))

    def update(self, pose: ArrayLike) -> TrackerState:
        """Advance the projection to this pose and encode what the observation needs."""
        previous = self.s
        projection = project(self.path, np.asarray(pose, dtype=float).ravel()[:2],
                             from_s=self.s, window=self.window)
        self.s = projection.s
        lookahead_s = min(self.s + self.lookahead, self.length)
        lookahead_world = point_at(self.path, lookahead_s)
        goal_world = self.path[-1]
        return TrackerState(
            s=self.s, progress=self.s - previous, cross_track=projection.distance,
            lookahead_world=lookahead_world, goal_world=goal_world,
            lookahead_body=to_body(lookahead_world, pose),
            goal_body=to_body(goal_world, pose),
            remaining=self.length - self.s,
            at_end=lookahead_s >= self.length - 1e-12)


def _as_path(path: ArrayLike) -> np.ndarray:
    p = np.asarray(path, dtype=float)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 2:
        raise ValueError(f"a path is (n >= 2, 2) points in world metres, got {p.shape}")
    return p
