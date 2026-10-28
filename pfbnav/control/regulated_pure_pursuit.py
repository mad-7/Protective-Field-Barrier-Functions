"""B2 -- holonomic Regulated Pure Pursuit. (WP3; ruled 2026-10-24: implemented and tuned under the
equal-tuning-budget rule, with the route cap and the classical recovery; B7 = B2's tuned parameters
plus the PFB, nothing else.)

Nav2's Regulated Pure Pursuit (Macenski et al., 2023) for a mecanum base, working from the typed
classical input alone (`pfbnav/envs/controller_input.py`, ruled 2026-10-16): the route ahead in the
body frame, the 120-bin scan stream and the velocity estimate. No pose, no map, no privileged state.

Each step:

  1. **Adaptive lookahead.** L = clip(|v_hat| t_L, L_min, L_max); the lookahead point lies L of arc
     along the route ahead (`route.points`, from the robot's projection to the goal).
  2. **Desired speed** `v_d`, then three regulations, each a factor in [0, 1], the least applied
     (Nav2 applies the curvature and proximity factors together, as a minimum):
       * **curvature**: the route turns by |dtheta| over the lookahead arc, a radius R = L / |dtheta|;
         below `min_radius` the speed scales by R / min_radius;
       * **proximity**: the clearance d is the smallest gap between a valid return and the footprint
         along its bearing (r_k - e(psi_k)); below `proximity_distance` the speed scales by
         `proximity_gain` x d / proximity_distance (Nav2's `cost_scaling_dist` and
         `cost_scaling_gain`; added 2026-10-26 so the tuning covers RPP's proximity speed scaling,
         ruled). An INVALID bin is an obstacle at the footprint's boundary on its bearing (the
         shared convention, `pfbnav/safety/unobserved.py`, ruled 2026-10-27), so clearance 0, never
         free space (hard rule 5);
       * **approach**: within `approach_distance` of the goal (route arc remaining) the speed scales
         by the remaining arc over that distance.
     The regulated speed never falls below `min_speed` while the route is unfinished, except by the
     approach factor, which may bring it to `min_approach_speed`.
  3. **Steering.** Translate toward the lookahead point at the regulated speed, and turn to face it at
     `k_omega` per radian of its bearing (the yaw law B1 uses, so the two differ only in the
     regulation and the lookahead).

Every parameter is in `configs/base.yaml` under `baselines.regulated_pure_pursuit`, an initial value
(VERIFY) until B2's tuning. Actions are normalised by each axis's own ceiling, as the environment
scales them back. Evaluated under the declared route cap and the classical recovery, as every
classical baseline is (`scripts/evaluate.py`).
"""
from __future__ import annotations

import itertools
import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np

from pfbnav.envs.controller_input import ClassicalInput
from pfbnav.safety.unobserved import resolve_unobserved


@dataclass(frozen=True)
class RPPParams:
    desired_speed: float       # m/s
    lookahead_time: float      # s, t_L
    min_lookahead: float       # m
    max_lookahead: float       # m
    min_radius: float          # m, curvature regulation below this radius
    proximity_distance: float  # m, proximity regulation below this clearance to the footprint
    approach_distance: float   # m of route arc remaining at which the approach slow-down starts
    min_speed: float           # m/s, the floor of the curvature and proximity regulation
    min_approach_speed: float  # m/s, the floor of the approach slow-down
    k_omega: float             # 1/s, yaw rate per radian of the lookahead bearing
    proximity_gain: float      # 1, Nav2's cost_scaling_gain, in (0, 1]: the proximity factor's gain

    def __post_init__(self) -> None:
        if not 0.0 < self.min_lookahead <= self.max_lookahead:
            raise ValueError("need 0 < min_lookahead <= max_lookahead")
        if not 0.0 <= self.min_approach_speed <= self.min_speed <= self.desired_speed:
            raise ValueError("need 0 <= min_approach_speed <= min_speed <= desired_speed")
        if not 0.0 < self.proximity_gain <= 1.0:
            raise ValueError("need 0 < proximity_gain <= 1 (Nav2: cost_scaling_gain <= 1)")
        for name in ("lookahead_time", "min_radius", "proximity_distance", "approach_distance",
                     "k_omega"):
            if not getattr(self, name) > 0.0:
                raise ValueError(f"{name} must be positive")


def _point_along(points: np.ndarray, arc: float) -> tuple[np.ndarray, float]:
    """The point `arc` metres along a polyline from its first point (its last point if the polyline
    is shorter), and the heading of the segment it lies on, rad."""
    p = np.asarray(points, dtype=float)
    left = max(float(arc), 0.0)
    heading = 0.0
    for a, b in itertools.pairwise(p):
        d = b - a
        ln = float(np.hypot(*d))
        if ln <= 1e-12:
            continue
        heading = math.atan2(d[1], d[0])
        if left <= ln:
            return a + d * (left / ln), heading
        left -= ln
    return p[-1].copy(), heading


def _start_heading(points: np.ndarray) -> float:
    p = np.asarray(points, dtype=float)
    for a, b in itertools.pairwise(p):
        d = b - a
        if float(np.hypot(*d)) > 1e-12:
            return math.atan2(d[1], d[0])
    return 0.0


class RegulatedPurePursuit:
    """Holonomic Regulated Pure Pursuit on the route ahead, the scan stream and the estimate."""

    name = "B2_regulated_pure_pursuit"
    input_kind = "classical"

    def __init__(self, params: RPPParams, footprint_extent: Callable[[np.ndarray], np.ndarray],
                 *, v_max: float, vy_max: float, w_max: float):
        if params.desired_speed > min(v_max, vy_max):
            raise ValueError("desired_speed must be within both translation ceilings")
        self.p = params
        self.extent = footprint_extent
        self.ceilings = np.array([v_max, vy_max, w_max], dtype=float)

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> RegulatedPurePursuit:
        from pathlib import Path

        from pfbnav.safety.footprint_table import FootprintTable

        root = Path(__file__).resolve().parents[2]
        table = FootprintTable.from_csv(root / cfg["robot"]["footprint_table"])
        b, a = cfg["baselines"]["regulated_pure_pursuit"], cfg["actuation"]
        params = RPPParams(**{k: float(b[k]) for k in RPPParams.__dataclass_fields__})
        return cls(params, table.extent, v_max=float(a["v_max"]), vy_max=float(a["vy_max"]),
                   w_max=float(a["w_max"]))

    def reset(self) -> None:
        """Memoryless."""

    # ------------------------------------------------------------------ the parts
    def lookahead_distance(self, inp: ClassicalInput) -> float:
        speed = float(np.hypot(inp.velocity[0], inp.velocity[1]))
        return float(np.clip(speed * self.p.lookahead_time, self.p.min_lookahead,
                             self.p.max_lookahead))

    def clearance(self, inp: ClassicalInput) -> float:
        """The smallest gap between a return and the footprint along its bearing, m; an invalid
        bin is an obstacle at the footprint's boundary (the shared convention), so 0 with any."""
        s = inp.scan
        r, _ = resolve_unobserved(s.ranges, s.bearings, s.valid, self.extent)
        gap = r - np.asarray(self.extent(s.bearings), dtype=float)

        return max(float(gap.min()), 0.0)

    def factors(self, inp: ClassicalInput, lookahead: float) -> dict[str, float]:
        """The three regulation factors, each in [0, 1]."""
        p = self.p
        _, end_heading = _point_along(inp.route.points, lookahead)
        turn = abs(math.remainder(end_heading - _start_heading(inp.route.points), 2 * math.pi))
        radius = lookahead / turn if turn > 1e-9 else math.inf
        return {"curvature": min(1.0, radius / p.min_radius),
                "proximity": self.proximity_factor(self.clearance(inp)),
                "approach": min(1.0, float(inp.route.remaining_m) / p.approach_distance)}

    def proximity_factor(self, clearance: float) -> float:
        """Nav2's cost constraint: 1 at or beyond `proximity_distance`, below it
        gain x clearance / distance (a step down by the gain at the threshold when gain < 1)."""
        p = self.p
        if clearance >= p.proximity_distance:
            return 1.0
        return p.proximity_gain * max(clearance, 0.0) / p.proximity_distance

    def speed(self, inp: ClassicalInput, lookahead: float) -> float:
        p = self.p
        f = self.factors(inp, lookahead)
        v = max(p.desired_speed * min(f["curvature"], f["proximity"]), p.min_speed)
        return max(min(v, p.desired_speed * f["approach"]), p.min_approach_speed)

    # ------------------------------------------------------------------ the interface
    def command(self, inp: ClassicalInput) -> np.ndarray:
        """The body-frame command (vx, vy, wz) in m/s and rad/s."""
        if not isinstance(inp, ClassicalInput):
            raise TypeError(f"Regulated Pure Pursuit takes a ClassicalInput, got "
                            f"{type(inp).__name__}")
        lookahead = self.lookahead_distance(inp)
        point, _ = _point_along(inp.route.points, lookahead)
        if float(np.hypot(*point)) <= 1e-9:
            return np.zeros(3)
        beta = math.atan2(point[1], point[0])
        v = self.speed(inp, lookahead)
        omega = float(np.clip(self.p.k_omega * beta, -self.ceilings[2], self.ceilings[2]))
        return np.array([v * math.cos(beta), v * math.sin(beta), omega])

    def act(self, inp: ClassicalInput) -> np.ndarray:
        return np.clip(self.command(inp) / self.ceilings, -1.0, 1.0)
