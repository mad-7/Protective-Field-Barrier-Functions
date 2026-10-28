"""The ideal tracker: the nominal controller of the PFB acceptance step (WP2, ruled 2026-10-23).

It follows the planned route at a fixed cruise speed: translation along the route's tangent at the
robot's projection, plus a cross-track correction toward the route; yaw toward the tangent; a linear
slow-down within `goal_slowdown` of the goal. It knows nothing of the scan. Every obstacle-related
slow-down, swerve or stop is therefore the PFB's or the certified layer's, which is what the
acceptance step must attribute. It reads only the typed classical input (route and goal, body frame).
Parameters: `baselines.ideal_tracker` in `configs/base.yaml`.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from pfbnav.envs.controller_input import ClassicalInput


class IdealTracker:
    """Tangent tracking of the planned route at a cruise speed."""

    name = "ideal_tracker"
    input_kind = "classical"

    def __init__(self, *, cruise: float, k_cross: float, k_yaw: float, goal_slowdown: float,
                 ceilings):
        if cruise <= 0.0 or k_cross < 0.0 or k_yaw < 0.0 or goal_slowdown <= 0.0:
            raise ValueError("impossible ideal-tracker parameters")
        self.cruise, self.k_cross, self.k_yaw = float(cruise), float(k_cross), float(k_yaw)
        self.goal_slowdown = float(goal_slowdown)
        self.ceilings = np.asarray(ceilings, dtype=float)

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> IdealTracker:
        t, a = cfg["baselines"]["ideal_tracker"], cfg["actuation"]
        return cls(cruise=float(t["cruise_speed"]), k_cross=float(t["k_cross"]),
                   k_yaw=float(t["k_yaw"]), goal_slowdown=float(t["goal_slowdown"]),
                   ceilings=[a["v_max"], a["vy_max"], a["w_max"]])

    def reset(self) -> None:
        """Memoryless."""

    def command(self, inp: ClassicalInput) -> np.ndarray:
        pts = np.asarray(inp.route.points, dtype=float)
        foot = pts[0]
        ahead = pts[1:][np.linalg.norm(pts[1:] - foot, axis=1) > 1e-9]
        if ahead.size == 0:
            return np.zeros(3)
        tangent = (ahead[0] - foot) / float(np.linalg.norm(ahead[0] - foot))
        goal_m = float(np.hypot(*inp.route.goal))
        speed = self.cruise * min(1.0, goal_m / self.goal_slowdown)
        v = speed * tangent + self.k_cross * foot
        n = float(np.linalg.norm(v))
        if n > self.cruise:
            v *= self.cruise / n
        w = float(np.clip(self.k_yaw * math.atan2(tangent[1], tangent[0]), -self.ceilings[2],
                          self.ceilings[2]))
        return np.array([float(v[0]), float(v[1]), w])

    def act(self, inp: ClassicalInput) -> np.ndarray:
        return np.clip(self.command(inp) / self.ceilings, -1.0, 1.0)
