"""B3 -- holonomic Dynamic Window Approach. (WP3; ruled 2026-10-26: implemented as code-only work
in parallel with Stage 1, tuned under the equal-tuning-budget rule, with the route cap and the
classical recovery, as every classical baseline.)

The Dynamic Window Approach (Fox, Burgard and Thrun, 1997) over a three-dimensional velocity space
(vx, vy, omega) for the mecanum base, in the form Nav2's DWB gives it for a holonomic robot: sample
the reachable velocities, roll each out at constant velocity, reject those that meet an obstacle,
and score the rest against the route. It works from the typed classical input alone
(`pfbnav/envs/controller_input.py`, ruled 2026-10-16): the route ahead in the body frame, the
120-bin scan stream and the velocity estimate. No pose, no map, no privileged state.

Each step:

  1. **The dynamic window.** Centred on the velocity ESTIMATE v_hat (the measured velocity, not the
     last command): the translations within `a_max` x `window_time` of (v_hat_x, v_hat_y) -- a disc,
     because the base's limiter bounds the MAGNITUDE of the change (`pfbnav/sim/robot.py`) -- and
     the yaw rates within `alpha_max` x `window_time` of v_hat_wz; intersected with the actuation
     box, the translation speed `max_speed` and the enforced wheel budget
     |vx| + |vy| + (l + w)|wz| <= filter_fraction x v_rim (the one share every method gets). It is
     sampled on a `vx_samples` x `vy_samples` x `wz_samples` grid over the window's bounding box;
     the grid points outside the window are dropped. `window_time` starts at DWB's, the control
     period (candidate 0, A28a); a longer window keeps the estimate's lag (filter_tau) from
     throttling acceleration below a_max, and the tuning range (0.05-0.40 s) covers both.
  2. **Rollout.** Each sample is held constant for `sim_time` from the current pose (the body frame
     at t = 0; closed form, `rollout`). States are spaced as DWB spaces them: no sample moves more
     than `sim_granularity` of translation or `angular_sim_granularity` of yaw between states, so a
     slow window rolls out fewer states.
  3. **Rejection.** Every scan point is fixed in the t = 0 frame. A state collides with a point when
     the point's clearance to the footprint along its own bearing, |q| - e(psi_q), falls below its
     group's threshold, min(`safety_margin`, the group's least clearance at t = 0); the groups are
     the valid returns and the unobserved points (below). A robot already nearer than the margin
     may therefore not come nearer to any return than its nearest return is now, rather than
     freezing: moving away is admitted. (Moving ALONG a wall inside the margin is admitted only
     where the scan's 3 deg sampling happens to allow it: the wall point it passes over may be
     a fraction of a millimetre nearer than the nearest sampled one now.) A sample with any
     colliding state within the horizon is rejected (Nav2 DWB). Because `sim_time` is at least
     the time to brake from the window's ceilings over half the distance
     (max(max_speed / 2 a_decel, w_max / 2 alpha_max)), every surviving sample is also admissible
     in Fox's sense: the robot can stop along it before any obstacle.
     **An INVALID bin is never free space (hard rule 5):** it is an obstacle at the footprint's
     boundary on its bearing -- the convention shared by every consumer of the stream (ruled
     2026-10-27, `pfbnav/safety/unobserved.py`; B3 first treated the whole ray as occupied). Its
     clearance now is 0, so its group's threshold is 0: B3 may move away from it, never toward it.
     The footprint is the measured table's e(psi), read as its maximum over 0.05 deg cells
     (`cell_max_table`): it over-states the robot by at most e's variation within a cell (~0.1 mm),
     the safe direction, and costs a lookup instead of a spline evaluation.
  4. **Scoring** of the surviving samples, a weighted sum of four costs, unnormalised (as DWB):
       * `w_path`      x the end state's distance to the route, m;
       * `w_progress`  x minus the route arc the end state's projection has reached, m;
       * `w_heading`   x |end yaw - the route tangent at that projection|, rad (face where it goes,
                         so the leading field faces the direction of travel, as B1 and B2 turn);
       * `w_clearance` x max(0, `clearance_cap` - the trajectory's smallest clearance), m.
     The route is cropped to its first 2 x max_speed x sim_time of arc, so a route that doubles
     back cannot pull a projection forward (the planner's track_window reasoning).
  5. **Command** the least-cost sample (ties to the first in grid order). **No admissible sample**:
     command zero, which the limiter brakes at a_decel_max, and count the step as blocked
     (`episode_report`; hard rule 9: an explicit, counted status).

Every parameter is in `configs/base.yaml` under `baselines.dwa`, an initial value (VERIFY) until B3's
tuning. Actions are normalised by each axis's own ceiling, as the environment scales them back.
Evaluated under the declared route cap and the classical recovery, as every classical baseline is
(`scripts/evaluate.py`).
"""
from __future__ import annotations

import itertools
import math
from collections.abc import Callable
from dataclasses import dataclass, fields
from typing import Any

import numpy as np

from pfbnav.envs.controller_input import ClassicalInput
from pfbnav.safety.unobserved import resolve_unobserved

# Numerical tolerances, not physical parameters: a sample on the window's edge stays in it, and a
# point that keeps exactly its threshold clearance does not collide through rounding.
WINDOW_TOL = 1e-12          # m/s and rad/s
CLEARANCE_TOL = 1e-9        # m
# The rollout's footprint: e(psi) as its MAXIMUM over each of EXTENT_CELLS equal bearing cells
# (0.05 deg), each cell's maximum taken over EXTENT_SUBSAMPLES evenly spaced bearings in it, ends
# included.
EXTENT_CELLS = 7200
EXTENT_SUBSAMPLES = 9
_COUNTS = ("vx_samples", "vy_samples", "wz_samples")


@dataclass(frozen=True)
class DWAParams:
    max_speed: float                # m/s, translation speed ceiling of the samples
    window_time: float              # s, the window's reach: a_max x this, alpha_max x this
    sim_time: float                 # s, rollout horizon
    sim_granularity: float          # m, most translation between rollout states
    angular_sim_granularity: float  # rad, most yaw between rollout states
    vx_samples: int                 # grid points over the window's vx extent
    vy_samples: int                 # grid points over the window's vy extent
    wz_samples: int                 # grid points over the window's wz extent
    safety_margin: float            # m, clearance below which a state collides (threshold rule)
    clearance_cap: float            # m, clearance at and beyond which the clearance cost is zero
    w_path: float                   # 1/m, the end state's distance to the route
    w_progress: float               # 1/m, the route arc reached (a reward: its cost is minus it)
    w_heading: float                # 1/rad, the end yaw's error to the route tangent
    w_clearance: float              # 1/m, the clearance shortfall below clearance_cap

    def __post_init__(self) -> None:
        for name in _COUNTS:
            n = getattr(self, name)
            if not isinstance(n, int) or n < 2:
                raise ValueError(f"{name} must be an integer >= 2")
        for name in ("max_speed", "window_time", "sim_time", "sim_granularity",
                     "angular_sim_granularity", "clearance_cap", "w_progress"):
            if not getattr(self, name) > 0.0:
                raise ValueError(f"{name} must be positive")
        for name in ("safety_margin", "w_path", "w_heading", "w_clearance"):
            if not getattr(self, name) >= 0.0:
                raise ValueError(f"{name} must be non-negative")

    @classmethod
    def from_block(cls, block: dict[str, Any]) -> DWAParams:
        """From `baselines.dwa` (or a tuning candidate), every value read as a number; a sample
        count must be whole, and is refused, never truncated, when it is not."""
        out: dict[str, Any] = {}
        for f in fields(cls):
            v = float(block[f.name])
            if f.name in _COUNTS:
                if v != int(v):
                    raise ValueError(f"{f.name} must be an integer, got {v}")
                v = int(v)
            out[f.name] = v
        return cls(**out)


def cell_max_table(extent: Callable[[np.ndarray], Any], cells: int = EXTENT_CELLS,
                   subsamples: int = EXTENT_SUBSAMPLES) -> np.ndarray:
    """e's maximum over each of `cells` equal bearing cells covering [-pi, pi), m; cell i spans
    [-pi + i h, -pi + (i + 1) h], h = 2 pi / cells, sampled at `subsamples` bearings, ends
    included."""
    h = 2.0 * math.pi / cells
    psi = -math.pi + h * (np.arange(cells)[:, None] + np.linspace(0.0, 1.0, subsamples)[None, :])
    return np.asarray(extent(psi.ravel()), dtype=float).reshape(cells, subsamples).max(axis=1)


def rollout(sample: np.ndarray, times: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Poses (x, y, theta) at `times` (s) under constant body twists from the identity pose.

    `sample` is (S, 3), (vx, vy, wz) in m/s and rad/s; each output is (S, len(times)), in the body
    frame at t = 0. Closed form, with theta = wz t:
        x = vx sin(theta) / wz - vy (1 - cos(theta)) / wz
        y = vx (1 - cos(theta)) / wz + vy sin(theta) / wz,
    written with sinc so that wz -> 0 gives the straight line exactly."""
    s = np.asarray(sample, dtype=float)
    t = np.asarray(times, dtype=float)[None, :]
    vx, vy, wz = (s[:, i:i + 1] for i in range(3))
    a = wz * t
    sin_over = t * np.sinc(a / math.pi)                                   # sin(a) / wz
    one_minus_cos_over = t * 0.5 * a * np.sinc(a / (2 * math.pi)) ** 2    # (1 - cos a) / wz
    x = vx * sin_over - vy * one_minus_cos_over
    y = vx * one_minus_cos_over + vy * sin_over
    return x, y, np.broadcast_to(a, x.shape).copy()


def crop_route(points: np.ndarray, arc: float) -> np.ndarray:
    """The polyline's first `arc` metres (all of it if shorter), zero-length segments dropped."""
    p = np.asarray(points, dtype=float)
    out = [p[0]]
    left = float(arc)
    for a, b in itertools.pairwise(p):
        ln = float(np.hypot(*(b - a)))
        if ln <= 1e-12:
            continue
        if left <= ln:
            out.append(a + (b - a) * (left / ln))
            break
        out.append(b)
        left -= ln
    if len(out) == 1:                       # a route of one point: no direction to follow
        out.append(p[0].copy())
    return np.asarray(out)


def project_route(points: np.ndarray, queries: np.ndarray
                  ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """For each query point (N, 2), body frame, m: its distance to the polyline, the arc length of
    its nearest point (from the polyline's first point), and the heading of the segment that point
    lies on, rad. Ties go to the earliest segment."""
    p = np.asarray(points, dtype=float)
    q = np.asarray(queries, dtype=float)
    a, d = p[:-1], np.diff(p, axis=0)
    length = np.hypot(d[:, 0], d[:, 1])
    keep = length > 1e-12
    if not np.any(keep):
        dist = np.hypot(q[:, 0] - p[0, 0], q[:, 1] - p[0, 1])
        return dist, np.zeros(len(q)), np.zeros(len(q))
    a, d, length = a[keep], d[keep], length[keep]
    start_arc = np.concatenate([[0.0], np.cumsum(length)[:-1]])
    rel = q[:, None, :] - a[None, :, :]                                  # (N, M, 2)
    u = np.clip((rel * d[None]).sum(-1) / length[None] ** 2, 0.0, 1.0)   # (N, M)
    gap = rel - u[..., None] * d[None]
    dist2 = (gap ** 2).sum(-1)
    m = np.argmin(dist2, axis=1)
    rows = np.arange(len(q))
    heading = np.arctan2(d[m, 1], d[m, 0])
    return np.sqrt(dist2[rows, m]), start_arc[m] + u[rows, m] * length[m], heading


@dataclass(frozen=True)
class Evaluation:
    """One step's dynamic window, every sample's fate and cost (for the tests and diagnostics)."""

    samples: np.ndarray             # (S, 3) the window's samples, m/s and rad/s
    times: np.ndarray               # (K,) the rollout states' times, s, from 0 to sim_time
    admissible: np.ndarray          # (S,) no colliding state within the horizon
    costs: dict[str, np.ndarray]    # each (S,), the UNWEIGHTED term; nan where not admissible
    total: np.ndarray               # (S,) the weighted sum; inf where not admissible
    best: int | None                # the least-cost admissible sample, None when blocked


class HolonomicDWA:
    """Holonomic DWA over (vx, vy, wz) on the route ahead, the scan stream and the estimate."""

    name = "B3_holonomic_dwa"
    input_kind = "classical"

    def __init__(self, params: DWAParams, footprint_extent: Callable[[np.ndarray], Any],
                 *, v_max: float, vy_max: float, w_max: float, a_max: float, alpha_max: float,
                 a_decel_max: float, wheel_budget: float, lever_arm: float):
        if params.max_speed > min(v_max, vy_max):
            raise ValueError("max_speed must be within both translation ceilings")
        for name, v in (("w_max", w_max), ("a_max", a_max), ("alpha_max", alpha_max),
                        ("a_decel_max", a_decel_max), ("wheel_budget", wheel_budget),
                        ("lever_arm", lever_arm)):
            if not v > 0.0:
                raise ValueError(f"{name} must be positive")
        braking = max(params.max_speed / (2.0 * a_decel_max), w_max / (2.0 * alpha_max))
        if params.sim_time < braking:
            raise ValueError(f"sim_time {params.sim_time} s is shorter than {braking:.3f} s, the "
                             "horizon over which a surviving sample is admissible (Fox)")
        self.p = params
        self.extent = footprint_extent
        self.ceilings = np.array([v_max, vy_max, w_max], dtype=float)
        self.a_max, self.alpha_max = float(a_max), float(alpha_max)
        self.wheel_budget, self.lever_arm = float(wheel_budget), float(lever_arm)
        # The rollout's footprint, tiled over enough periods that a state's bearing (in
        # [-pi - w_max sim_time, pi + w_max sim_time]) indexes it without a modulo; one period to
        # spare on each side, so the ends are never reached.
        cells = cell_max_table(footprint_extent)
        self.extent_bound = float(cells.max())
        self._periods = 2 * math.ceil(w_max * params.sim_time / (2.0 * math.pi)) + 3
        self._cells = np.tile(cells, self._periods)
        self._cell_scale = len(cells) / (2.0 * math.pi)
        self._cell_offset = math.pi * self._periods      # the tiled table starts at -this
        self.steps = 0
        self.blocked_steps = 0

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> HolonomicDWA:
        from pathlib import Path

        from pfbnav.safety.footprint_table import FootprintTable

        root = Path(__file__).resolve().parents[2]
        table = FootprintTable.from_csv(root / cfg["robot"]["footprint_table"])
        a, r = cfg["actuation"], cfg["robot"]
        budget = a["wheel_budget"]
        return cls(DWAParams.from_block(cfg["baselines"]["dwa"]), table.extent,
                   v_max=float(a["v_max"]), vy_max=float(a["vy_max"]), w_max=float(a["w_max"]),
                   a_max=float(a["a_max"]), alpha_max=float(a["alpha_max"]),
                   a_decel_max=float(a["a_decel_max"]),
                   wheel_budget=float(budget["filter_fraction"]) * float(budget["v_rim"]),
                   lever_arm=float(r["half_wheelbase"]) + float(r["half_track"]))

    def reset(self) -> None:
        """Memoryless but for the episode's counts."""
        self.steps = self.blocked_steps = 0

    def episode_report(self) -> dict[str, Any]:
        """The episode's control steps and the blocked ones (no admissible sample: zero commanded)."""
        return {"dwa_steps": self.steps, "dwa_blocked_steps": self.blocked_steps}

    # ------------------------------------------------------------------ the parts
    def window(self, velocity: np.ndarray) -> np.ndarray:
        """The dynamic window's samples, (S, 3), around the velocity estimate (vx, vy, wz)."""
        p = self.p
        v = np.asarray(velocity, dtype=float)
        dv, dw = self.a_max * p.window_time, self.alpha_max * p.window_time
        top = np.array([min(self.ceilings[0], p.max_speed), min(self.ceilings[1], p.max_speed),
                        self.ceilings[2]])
        axes = []
        for i, (half, n) in enumerate(((dv, p.vx_samples), (dv, p.vy_samples),
                                       (dw, p.wz_samples))):
            lo, hi = max(v[i] - half, -top[i]), min(v[i] + half, top[i])
            if lo > hi:                     # the estimate is beyond the box by more than the reach
                lo = hi = float(np.clip(v[i], -top[i], top[i]))
            axes.append(np.unique(np.linspace(lo, hi, n)))
        g = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
        reach = np.hypot(g[:, 0] - v[0], g[:, 1] - v[1]) <= dv + WINDOW_TOL
        speed = np.hypot(g[:, 0], g[:, 1]) <= p.max_speed + WINDOW_TOL
        wheels = (np.abs(g[:, 0]) + np.abs(g[:, 1]) + self.lever_arm * np.abs(g[:, 2])
                  <= self.wheel_budget + WINDOW_TOL)
        return g[reach & speed & wheels]

    def state_times(self, samples: np.ndarray) -> np.ndarray:
        """The rollout states' times, s: evenly spaced from 0 to sim_time, as many as the window's
        fastest sample needs so that none moves more than the granularities between states."""
        p = self.p
        if len(samples) == 0:
            return np.array([0.0, p.sim_time])
        trans = float(np.hypot(samples[:, 0], samples[:, 1]).max()) * p.sim_time
        turn = float(np.abs(samples[:, 2]).max()) * p.sim_time
        k = max(1, math.ceil(max(trans / p.sim_granularity, turn / p.angular_sim_granularity)
                             - 1e-9))
        return np.linspace(0.0, p.sim_time, k + 1)

    def obstacle_points(self, inp: ClassicalInput, reach: float | None = None
                        ) -> tuple[np.ndarray, np.ndarray]:
        """The scan as body-frame points (P, 2), and which of them are unobserved (P,): each valid
        return, and for each INVALID bin a point at the rollout footprint's boundary on its bearing
        (the shared convention; hard rule 5). Points farther than `reach` (m, the most any state's
        origin moves; default max_speed x sim_time) plus the footprint's largest extent plus the
        larger of the margin and the clearance cap can change no decision, and are dropped."""
        p = self.p
        s = inp.scan
        horizon = p.max_speed * p.sim_time if reach is None else float(reach)
        far = horizon + self.extent_bound + max(p.safety_margin, p.clearance_cap)
        bearings = np.asarray(s.bearings, dtype=float)
        valid = np.asarray(s.valid, dtype=bool)
        ranges, _ = resolve_unobserved(s.ranges, bearings, valid, self.rollout_extent)
        keep = ~valid | (ranges <= far)
        pts = np.column_stack([ranges[keep] * np.cos(bearings[keep]),
                               ranges[keep] * np.sin(bearings[keep])])
        return pts, ~valid[keep]


    def thresholds(self, clear0: np.ndarray, unobserved: np.ndarray) -> np.ndarray:
        """Each point's collision threshold, m, from the clearances at t = 0 (P,): its group's
        min(safety_margin, least clearance), the groups being the returns and the unobserved."""
        out = np.empty(len(clear0))
        for group in (~unobserved, unobserved):
            if np.any(group):
                out[group] = min(self.p.safety_margin, float(clear0[group].min()))
        return out

    def rollout_extent(self, psi: np.ndarray) -> np.ndarray:
        """The rollout footprint at bearings `psi`, rad, |psi| < pi x the tiled periods: m."""
        idx = ((np.asarray(psi, dtype=float) + self._cell_offset) * self._cell_scale).astype(np.intp)
        return self._cells[idx]

    def clearances(self, points: np.ndarray, x: np.ndarray, y: np.ndarray, th: np.ndarray
                   ) -> np.ndarray:
        """Each point's clearance to the rollout footprint at each state, |q| - e(psi_q), (S, K, P),
        m. q is the point in the state's body frame: its norm is the distance from the state's
        origin (a rotation keeps it), its bearing that distance's direction minus the state's
        yaw."""
        dx = points[None, None, :, 0] - x[..., None]
        dy = points[None, None, :, 1] - y[..., None]
        psi = np.arctan2(dy, dx)
        psi -= th[..., None]
        out = np.hypot(dx, dy)
        out -= self.rollout_extent(psi)
        return out

    def evaluate(self, inp: ClassicalInput) -> Evaluation:
        """The window, the rejection and the four costs for this step's input."""
        p = self.p
        samples = self.window(inp.velocity)
        times = self.state_times(samples)
        x, y, th = rollout(samples, times)
        n = len(samples)
        top = float(np.hypot(samples[:, 0], samples[:, 1]).max()) if n else 0.0
        points, unobserved = self.obstacle_points(inp, top * p.sim_time)
        if len(points) and n:
            clear = self.clearances(points, x, y, th)                     # (S, K, P)
            threshold = self.thresholds(clear[0, 0], unobserved)          # state 0 is shared
            collide = np.any(clear < threshold - CLEARANCE_TOL, axis=(1, 2))
            least = clear.min(axis=(1, 2))
        else:
            collide = np.zeros(n, dtype=bool)
            least = np.full(n, math.inf)
        route = crop_route(inp.route.points, 2.0 * p.max_speed * p.sim_time)
        dist, arc, tangent = project_route(route, np.column_stack([x[:, -1], y[:, -1]]))
        heading = np.abs(np.remainder(th[:, -1] - tangent + math.pi, 2 * math.pi) - math.pi)
        shortfall = np.maximum(0.0, p.clearance_cap - least)
        admissible = ~collide
        total = (p.w_path * dist - p.w_progress * arc + p.w_heading * heading
                 + p.w_clearance * shortfall)
        total = np.where(admissible, total, math.inf)
        costs = {k: np.where(admissible, v, math.nan) for k, v in
                 (("path", dist), ("progress", arc), ("heading", heading),
                  ("clearance", shortfall))}
        best = int(np.argmin(total)) if np.any(admissible) else None
        return Evaluation(samples, times, admissible, costs, total, best)

    # ------------------------------------------------------------------ the interface
    def command(self, inp: ClassicalInput) -> np.ndarray:
        """The body-frame command (vx, vy, wz) in m/s and rad/s; zero when blocked."""
        if not isinstance(inp, ClassicalInput):
            raise TypeError(f"Holonomic DWA takes a ClassicalInput, got {type(inp).__name__}")
        ev = self.evaluate(inp)
        self.steps += 1
        if ev.best is None:
            self.blocked_steps += 1
            return np.zeros(3)
        return ev.samples[ev.best].copy()

    def act(self, inp: ClassicalInput) -> np.ndarray:
        return np.clip(self.command(inp) / self.ceilings, -1.0, 1.0)
