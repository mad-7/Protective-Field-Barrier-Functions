"""Draw the planned route and the robot's actual track in the CoppeliaSim scene (debug only).

For watching a run: the planned route (blue line), where the robot started and has to reach (green and
yellow points) and the track the robot has actually driven (red line), so a mismatch between what the
planner produced and what the robot did is visible. Drawing objects are not shapes: they have no
physics, no collision and no scan return, so a run with the overlay is the same run (the scene is
unchanged and nothing in the step loop reads it). The simulator removes them when the simulation
stops, and `end` removes them explicitly.

The overlay does nothing unless it is built with `enabled=True`, and then it needs a rendering
instance: a headless instance has no scene window to show it in (it still accepts the calls).
Layout coordinates are mapped to the world with the episode's own placement transform.
"""
from __future__ import annotations

import itertools
from typing import Any

import numpy as np

HEIGHT_M = 0.05                       # m above the floor, so the lines do not z-fight with it
PLAN_COLOUR = (0.1, 0.3, 1.0)
TRACK_COLOUR = (1.0, 0.1, 0.1)
START_COLOUR = (0.1, 0.8, 0.1)
GOAL_COLOUR = (1.0, 0.9, 0.0)
MAX_ITEMS = 100000
MIN_STEP_M = 0.01                     # m the robot must move before a new track segment is added


def segments(points_xy: np.ndarray, z: float = HEIGHT_M) -> list[list[float]]:
    """A polyline as line items (x1, y1, z, x2, y2, z), one per consecutive pair of points."""
    pts = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    return [[float(a[0]), float(a[1]), z, float(b[0]), float(b[1]), z]
            for a, b in itertools.pairwise(pts)]


class PathOverlay:
    """Observer for `pfbnav.eval.runner.run_episodes`: `begin` after each reset, `step` after each
    environment step, `end` when the episode is over."""

    def __init__(self, sim: Any, enabled: bool = True):
        self.sim = sim
        self.enabled = bool(enabled)
        self._objects: list[int] = []
        self._last_xy: np.ndarray | None = None
        self._track = None

    def _new(self, kind: int, size: float, colour: tuple[float, float, float]) -> int:
        handle = self.sim.addDrawingObject(kind, size, 0.0, -1, MAX_ITEMS, list(colour))
        self._objects.append(handle)
        return handle

    def begin(self, env: Any, info: dict[str, Any]) -> None:
        if not self.enabled:
            return
        self.end()
        path = env.transform.apply(np.asarray(env.plan.path_world, dtype=float))
        plan = self._new(self.sim.drawing_lines, 3, PLAN_COLOUR)
        for item in segments(path):
            self.sim.addDrawingObjectItem(plan, item)
        for xy, colour in ((path[0], START_COLOUR), (path[-1], GOAL_COLOUR)):
            dot = self._new(self.sim.drawing_spherepoints, 0.12, colour)
            self.sim.addDrawingObjectItem(dot, [float(xy[0]), float(xy[1]), HEIGHT_M])
        self._track = self._new(self.sim.drawing_lines, 3, TRACK_COLOUR)
        self._last_xy = np.asarray(path[0], dtype=float)

    def step(self, env: Any, info: dict[str, Any]) -> None:
        if not self.enabled or self._track is None or self._last_xy is None:
            return
        x, y, _ = env._robot_world_pose()
        here = np.array([x, y])
        if np.linalg.norm(here - self._last_xy) >= MIN_STEP_M:
            self.sim.addDrawingObjectItem(
                self._track, [float(self._last_xy[0]), float(self._last_xy[1]), HEIGHT_M,
                              float(here[0]), float(here[1]), HEIGHT_M])
            self._last_xy = here

    def end(self) -> None:
        for handle in self._objects:
            self.sim.removeDrawingObject(handle)
        self._objects = []
        self._track = None
