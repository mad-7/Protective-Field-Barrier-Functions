"""B1 -- holonomic Pure Pursuit, and the zero-action control. (WP3; first use: G4's controls)

**Pure Pursuit works from the classical input alone** (`pfbnav/envs/controller_input.py`, ruled
2026-10-16): the lookahead point and the goal, in the body frame. No pose, no privileged state.
Each step it translates toward the lookahead point (bearing beta, body frame) at `cruise_speed`,
slowing linearly within `goal_slowdown` of the goal, and turns to face that point at `k_omega` per
radian. A mecanum base can translate in any direction, so it never has to turn before it moves; it
turns so that its leading field faces where it is going. Every parameter is in `configs/base.yaml`
under `baselines.pure_pursuit`, untuned (VERIFY) until B1's own tuning under the
equal-tuning-budget rule. (It read the same lookahead bearing and goal distance from the 82-D
observation until 2026-10-16; the arithmetic is unchanged.)

**The zero-action control** commands nothing, every step. G4 requires it to FAIL the checkpoint
criterion with all timeouts and no successes (A15): if doing nothing can reach a goal or collide,
the gate is not measuring what it claims. It takes the classical input so that the recovery
wrapper can sit over it: requesting nothing, it must never trigger recovery (ruled 2026-10-16).

Actions are normalised by each axis's own ceiling, exactly as the environment scales them back.
Both are wrapped by the declared recovery (`pfbnav/control/recovery.py`) when evaluated.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from pfbnav.envs.controller_input import ClassicalInput


def _ceilings(cfg: dict[str, Any]) -> np.ndarray:
    a = cfg["actuation"]
    return np.array([float(a["v_max"]), float(a["vy_max"]), float(a["w_max"])])


class PurePursuit:
    """Holonomic Pure Pursuit on the route's lookahead point and goal."""

    name = "B1_pure_pursuit"
    input_kind = "classical"

    def __init__(self, *, cruise_speed: float, k_omega: float, goal_slowdown: float,
                 v_max: float, vy_max: float, w_max: float):
        if not 0.0 < cruise_speed <= min(v_max, vy_max):
            raise ValueError("cruise_speed must be positive and within both translation ceilings")
        if k_omega <= 0.0 or goal_slowdown <= 0.0:
            raise ValueError("k_omega and goal_slowdown must be positive")
        self.cruise_speed, self.k_omega, self.goal_slowdown = cruise_speed, k_omega, goal_slowdown
        self.ceilings = np.array([v_max, vy_max, w_max], dtype=float)

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> PurePursuit:
        p = cfg["baselines"]["pure_pursuit"]
        v_max, vy_max, w_max = _ceilings(cfg)
        return cls(cruise_speed=float(p["cruise_speed"]), k_omega=float(p["k_omega"]),
                   goal_slowdown=float(p["goal_slowdown"]), v_max=v_max, vy_max=vy_max,
                   w_max=w_max)

    def reset(self) -> None:
        """Memoryless."""

    def command(self, inp: ClassicalInput) -> np.ndarray:
        """The body-frame command (vx, vy, wz) in m/s and rad/s."""
        if not isinstance(inp, ClassicalInput):
            raise TypeError(f"Pure Pursuit takes a ClassicalInput, got {type(inp).__name__}")
        lx, ly = (float(v) for v in inp.route.lookahead)
        beta = math.atan2(ly, lx) if math.hypot(lx, ly) > 1e-12 else 0.0
        goal_m = float(np.hypot(*inp.route.goal))
        speed = self.cruise_speed * min(1.0, goal_m / self.goal_slowdown)
        omega = float(np.clip(self.k_omega * beta, -self.ceilings[2], self.ceilings[2]))
        return np.array([speed * math.cos(beta), speed * math.sin(beta), omega])

    def act(self, inp: ClassicalInput) -> np.ndarray:
        return np.clip(self.command(inp) / self.ceilings, -1.0, 1.0)


class ZeroAction:
    """Commands nothing. G4's negative control."""

    name = "zero_action"
    input_kind = "classical"

    def __init__(self, ceilings: np.ndarray | None = None):
        self.ceilings = np.ones(3) if ceilings is None else np.asarray(ceilings, dtype=float)

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> ZeroAction:
        return cls(_ceilings(cfg))

    def reset(self) -> None:
        """Memoryless."""

    def command(self, inp: ClassicalInput) -> np.ndarray:
        return np.zeros(3)

    def act(self, inp: ClassicalInput) -> np.ndarray:
        return np.zeros(3)
