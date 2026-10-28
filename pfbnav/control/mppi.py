"""B6 -- holonomic MPPI with a field-aware cost. (WP3; A16, ruled 2026-10-15: an H1 comparator in
retired B4's place, implemented in our stack with vectorised rollouts, not Nav2.)

The hand-designed answer to "could a controller that avoids what stops it do as well without
learning?": a sampling model-predictive controller whose cost counts the scan points that would lie
inside the **certified layer's own discrete band field** at each predicted state -- the field that
stops the robot, never the PFB's continuous field (which would make MPPI a soft PFB). Like every
classical baseline it works from the typed classical input alone (`pfbnav/envs/controller_input.py`,
ruled 2026-10-16): the route ahead in the body frame, the 120-bin filter stream and the control
period. No pose, no map, no privileged state. It never imports the PFB (hard rule 3); the field
geometry is `pfbnav/safety/certified.py`'s, read-only.

**Declared structure** (`configs/base.yaml`, `baselines.mppi`; every number an initial value until
tuned under the equal-tuning-budget rule on the tune set only):

  * **Model.** Kinematic holonomic: the state (x, y, theta) in the body frame of the present step,
    each command (vx, vy, wz) held for one `sim.dt` (integrated at the mid-step heading). Every
    command passes exactly what the base applies to it (`YouBotBase.command_body_velocity`): clamped
    to the actuation box (direction kept), projected into the wheel budget (the filter share every
    method's command meets), then the ACCELERATION LIMITER against the command before it (a_max,
    a_decel_max on the translation's change, alpha_max on the yaw rate's; `limit_acceleration`).
    Ruled 2026-10-27 (it was kinematic without the limiter until then, so a rollout could reach any
    speed in one step while the robot cannot).
  * **Where the rollouts start.** The command the base is executing, as B6 models it from its own
    commands: the limiter's step from the command executing before toward the command it issued
    last (what the base applied to it), except after a step on which the certified layer held the
    robot (`stop_active` in the input: the layer replaced that command by zero), when it is the
    limiter's step toward zero instead -- what the base then applied. The route cap and the recovery may
    also change what reaches the base; B6 does not see that, and the base's own limiter governs.
  * **Sampling.** `samples` sequences of `horizon_steps` commands: the nominal sequence plus
    Gaussian noise of `noise_std` per axis; sample 0 is the nominal itself. Each sequence is made
    feasible in the box and budget, then rolled out through the limiter (the dynamics). The update
    averages the COMMAND sequences, as MPPI's control-sequence update does (Williams et al. 2017):
    box and budget are convex, so the average stays inside them, and the limiter stays in the
    dynamics. (Averaging the limited sequences instead fed the limiter's asymmetry -- a_decel_max
    dt down, a_max dt up -- back into the plan every step, and the plan settled near 0.08 m/s on an
    open route: found 2026-10-27.) The command issued is the nominal's first; the base limits it as
    the rollout did. `iterations` passes per step; between steps the nominal is warm-started by
    shifting it one step (its last command repeated).
  * **Cost** of a rollout, its states p_1 .. p_H (state t reached under command t-1):
      - route tracking: `route_tracking` x the sum over states of the squared distance to the route;
      - goal progress: -`goal_progress` x the arc length of the last state's projection on the
        route (the route ahead starts at the robot's projection, arc 0);
      - field breach: `field_breach` x the number of (state, scan point) pairs with the point
        inside the certified field of that state, sized as the layer sizes it (ruled 2026-10-27:
        the certified composition): speed max(|estimate|, |command executing|), the yaw rate the
        larger of the two, facing the travel direction of whichever dominates; the band that speed
        selects from the speed reachable within T (`FieldGeometry.band_for`, A20), with the
        rotation sweep of the yaw rate; the at-rest field below both standstill ENTER thresholds.
        The estimate along the rollout is predicted by the estimator's own first-order filter
        (alpha = dt / (filter_tau + dt), `pfbnav/sim/velocity_estimate.py`) driven by the
        rollout's commands, from the estimate in the input;
      - smoothness: `smoothness` x the sum of squared changes between consecutive commands, the
        first against the command issued on the previous step;
      - hard rejection: a rollout with any scan point inside the footprint (`robot.footprint_table`)
        at any state costs +inf: it gets zero weight; it is rejected, not weighed.
  * **No heading term** is declared (confirmed 2026-10-27): the yaw is driven only by the field and
    smoothness costs.
  * **Weighting.** The path-integral weights exp(-(S_k - min S) / temperature) on PER-STEP costs,
    S_k = the rollout's cost / horizon_steps, so the temperature does not scale with the horizon
    (confirmed 2026-10-27; smoothness is within the rollout, also confirmed). Only the declared
    terms enter (no control-cost term lambda u' Sigma^-1 eps).
  * **Scan.** The filter stream's points are taken as static over the horizon. An INVALID bin is an
    obstacle at the footprint's boundary on its bearing -- the convention shared by every consumer
    of the stream (ruled 2026-10-27, `pfbnav/safety/unobserved.py`), placed on B6's own footprint
    (the cell maxima hard rejection tests against, the larger of the cell and its two neighbours),
    so that standing still touches it without entering it: a rollout moving toward it is rejected,
    one moving away is not. (It was a point at range_min, inside the footprint, which rejected every
    rollout while any bin was invalid.)
  * **Every rollout rejected.** The command is zero, the nominal is cleared, and the step is counted
    (`episode_report()["mppi_all_rejected"]`, reported per method and stratum; ruled 2026-10-27; hard
    rule 9: an explicit, counted status). The command executing is then the limiter's step toward
    zero, which is what the base does with a zero command.

**The field table.** Evaluating `FieldGeometry.extent` at every (rollout, state, point) is out of
reach (millions of pairs per step), so the fields are tabulated once per parameter set (`FieldTable`,
cached per process) and read by index:

  * **bearing:** cells uniform in the DIAMOND ANGLE (a monotone pseudo-angle needing one division,
    not an arctangent), each holding the largest extent sampled across it, edges included;
  * **travel heading** (5 deg grid) and **|yaw rate|** (knots every 0.1 rad/s): bilinear
    interpolation. The extent is not convex in the yaw rate and not linear in the heading, so the
    interpolant can fall slightly below the exact field as well as above it. (Taking the larger of
    the two bracketing headings instead, the first version, over-stated the field by up to 3 cm --
    2 cm abeam, where the narrowest aisle leaves about 21 mm between the top band's field and the
    wall; rounding the yaw rate UP to its knot over-stated it by up to 7.7 cm.);
  * **at rest:** one row per yaw knot, interpolated in the yaw rate;
  * **CONSERVATIVE** (ruled 2026-10-27: never smaller than the certified field): every entry carries
    `TABLE_PAD_M`, 3 mm, 1.5 x the worst undershoot of the unpadded interpolant measured against
    `FieldGeometry.select` (-2.0 mm over 5,400 random states x 2,880 bearings, moving and at rest,
    2026-10-27). `tests/test_mppi.py` asserts the padded table is never below the exact field over a
    dense sweep, and pins the over-statement.

A support grid of 360 normals over-states the exact boundary by at most 0.3 mm (the same comparison
against the layer's 1440).

**Determinism.** The sampler is seeded from the EPISODE: `begin_episode` (called by the evaluation
runner after each reset, through the recovery and the route cap) reseeds it from the scenario's
seed, so a scenario's rollouts do not depend on the order it is run in -- and B6's noise does NOT
depend on the run's `--seed` (confirmed 2026-10-27, recorded in the methods): a repeated evaluation
of B6 on the same scenarios draws the same noise. Repetitions within one scene load are bit-identical
(determinism is per scene load, ruled 2026-10-14).

**Per-step time.** The real-time criterion (ruled 2026-10-27) is p95 compute per step <= 50 ms, the
control period, on the training host, for every controller. The cost is about samples x horizon x
the scan points within reach (the horizon's travel plus the largest field). `samples` and
`horizon_steps` are therefore NOT tuned: they are fixed by a declared measurement on the training
host (`scripts/measure_realtime.py`): the largest pair of samples {256, 512, 1024} x horizon
{15, 20, 30} whose p95 in 0.7-0.9 m aisles is <= 50 ms. `iterations` stays 1. On this laptop
(2026-10-27, 1024 x 30, before the limiter): 15-45 ms in the open, 280-445 ms in a 0.7-0.9 m aisle.
The table costs about 10 s once per process. No figure from this laptop is a property of the
training host.

Evaluated under the declared route cap and the classical recovery, as every classical baseline is
(`scripts/evaluate.py`).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.envs.controller_input import ClassicalInput
from pfbnav.safety.certified import CertifiedParams, FieldGeometry
from pfbnav.safety.unobserved import resolve_unobserved

# Declared in `baselines.mppi` and not tunable: a different value is a different method (A16).
DECLARED = {"model": "holonomic", "scan": "filter", "hard_rejection": "footprint"}
WEIGHTS = ("route_tracking", "goal_progress", "field_breach", "smoothness")

# The field table's grid. Discretisation, not physics: its error against the exact field is
# measured in tests/test_mppi.py.
HEADING_CELLS = 72                 # 5 deg cells of travel direction (body frame)
DIAMOND_CELLS = 512                # bearing cells, uniform in the diamond angle (<= 0.9 deg each)
CELL_SAMPLES = 3                   # extent samples per bearing cell, both edges included
YAW_KNOT_STEP = 0.1                # rad/s between yaw-rate knots (and the yaw cap the last)
FIELD_NORMALS = 360                # support-grid normals (fewer over-state; 0.3 mm at 360)
FOOTPRINT_SAMPLES = 9              # footprint samples per bearing cell, both edges included
CHUNK = 32                         # rollouts per block of the (rollout, state, point) arrays
# Added to every table entry so the table is never below the certified field (ruled 2026-10-27):
# 1.5 x the worst undershoot of the unpadded interpolant, -2.0 mm (module docstring). Discretisation,
# not physics; tests/test_mppi.py asserts the result.
TABLE_PAD_M = 0.003

# Separates B6's random stream from any other use of a scenario's seed.
SEED_DOMAIN = 0xB6


@dataclass(frozen=True)
class MPPIParams:
    horizon_steps: int                       # steps of sim.dt per rollout
    samples: int                             # rollouts per iteration, sample 0 the nominal
    temperature: float                       # lambda, on per-step costs
    noise_std: tuple[float, ...]             # m/s, m/s, rad/s on (vx, vy, wz)
    iterations: int                          # optimisation passes per control step
    route_tracking: float                    # per m^2, per state: x squared distance to the route
    goal_progress: float                     # per m: x arc length gained along the route
    field_breach: float                      # per (state, scan point) inside the certified field
    smoothness: float                        # per (m/s)^2 or (rad/s)^2 of command change, per step

    def __post_init__(self) -> None:
        for name in ("horizon_steps", "samples", "iterations"):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, int):
                raise TypeError(f"{name} must be an integer, got {v!r}")
        if self.horizon_steps < 1 or self.iterations < 1:
            raise ValueError("horizon_steps and iterations must be at least 1")
        if self.samples < 2:
            raise ValueError("samples must be at least 2 (sample 0 is the nominal)")
        if not self.temperature > 0.0:
            raise ValueError("temperature must be positive")
        if len(self.noise_std) != 3 or not all(s > 0.0 for s in self.noise_std):
            raise ValueError("noise_std is three positive standard deviations (vx, vy, wz)")
        for name in WEIGHTS:
            if not getattr(self, name) >= 0.0:
                raise ValueError(f"weight {name} must be non-negative")

    @classmethod
    def from_block(cls, block: dict[str, Any]) -> MPPIParams:
        """From `baselines.mppi`; refuses a block whose declared structure differs (A16)."""
        for key, value in DECLARED.items():
            if block.get(key) != value:
                raise ValueError(f"baselines.mppi.{key} is declared {value!r} (A16), got "
                                 f"{block.get(key)!r}")
        w = block["weights"]
        if set(w) != set(WEIGHTS):
            raise ValueError(f"baselines.mppi.weights must be exactly {list(WEIGHTS)}")
        return cls(horizon_steps=_integer(block["horizon_steps"], "horizon_steps"),
                   samples=_integer(block["samples"], "samples"),
                   temperature=float(block["temperature"]),
                   noise_std=tuple(float(s) for s in block["noise_std"]),
                   iterations=_integer(block["iterations"], "iterations"),
                   **{k: float(w[k]) for k in WEIGHTS})

    @classmethod
    def from_flat(cls, **flat: float) -> MPPIParams:
        """From the tuning runner's flat names (`weights.route_tracking`, `noise_std.0`, ...): the
        numeric leaves of `baselines.mppi` (`scripts/tune_classical.py`)."""
        return cls.from_block({
            **DECLARED,
            "horizon_steps": flat["horizon_steps"], "samples": flat["samples"],
            "temperature": flat["temperature"], "iterations": flat["iterations"],
            "noise_std": [flat[f"noise_std.{i}"] for i in range(3)],
            "weights": {k: flat[f"weights.{k}"] for k in WEIGHTS}})


def _integer(value: Any, name: str) -> int:
    """An integer-valued config number (a tuning draw may arrive as 30.0); refuses 30.5."""
    v = float(value)
    if not v.is_integer():
        raise ValueError(f"{name} must be an integer, got {value!r}")
    return int(v)


# ------------------------------------------------------------------ actuation, vectorised
def clamp_box(cmd: np.ndarray, v_max: float, vy_max: float, w_max: float) -> np.ndarray:
    """`clamp_velocity_with_status` over the last axis: the translation scaled by one factor into
    the box (direction kept), the yaw rate clipped."""
    c = np.asarray(cmd, dtype=float)
    ax, ay = np.abs(c[..., 0]), np.abs(c[..., 1])
    with np.errstate(divide="ignore"):
        scale = np.minimum(np.where(ax > v_max, v_max / ax, 1.0),
                           np.where(ay > vy_max, vy_max / ay, 1.0))
    out = np.empty_like(c)
    out[..., 0] = np.clip(c[..., 0] * scale, -v_max, v_max)
    out[..., 1] = np.clip(c[..., 1] * scale, -vy_max, vy_max)
    out[..., 2] = np.clip(c[..., 2], -w_max, w_max)
    return out


def project_budget(cmd: np.ndarray, limit: float, lever_arm: float) -> np.ndarray:
    """`project_to_wheel_budget` over the last axis: |vx| + |vy| + (l+w)|wz| <= limit by one
    factor, the direction kept in all three components."""
    c = np.asarray(cmd, dtype=float)
    use = np.abs(c[..., 0]) + np.abs(c[..., 1]) + lever_arm * np.abs(c[..., 2])
    with np.errstate(divide="ignore", invalid="ignore"):
        scale = np.where(use > limit, limit / use, 1.0)
    return c * scale[..., None]


# ------------------------------------------------------------------ the diamond angle
def diamond(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """The diamond angle of (x, y) in [-2, 2]: monotone in the bearing atan2(y, x), equal to it at
    0, +-pi/2 and +-pi (0, +-1, +-2). One division instead of an arctangent."""
    a = np.abs(x) + np.abs(y)
    t = y / np.maximum(a, np.finfo(np.asarray(a).dtype).tiny)
    return np.where(x >= 0.0, t, np.copysign(2.0, y).astype(t.dtype) - t)


def diamond_to_bearing(d: np.ndarray) -> np.ndarray:
    """The bearing, rad, of a diamond angle (the inverse of `diamond`)."""
    d = np.asarray(d, dtype=float)
    t = np.where(d > 1.0, 2.0 - d, np.where(d < -1.0, -2.0 - d, d))
    x = np.where(np.abs(d) > 1.0, -1.0, 1.0) * (1.0 - np.abs(t))
    return np.arctan2(t, x)


def inverse_cos(bearings: np.ndarray, normals: np.ndarray) -> np.ndarray:
    """1 / cos(psi - n), +inf where the normal faces away (`FieldGeometry._inverse_cos`)."""
    cos = np.cos(np.asarray(bearings, dtype=float)[:, None] - np.asarray(normals)[None, :])
    with np.errstate(divide="ignore"):
        return np.where(cos > 1e-9, 1.0 / cos, np.inf)


def extent_of_support(support: np.ndarray, inv: np.ndarray, block: int = 8) -> np.ndarray:
    """rho(psi) = min over normals n with cos(psi - n) > 0 of h(n) / cos(psi - n), m: the boundary
    distance of a convex set containing the origin -- `FieldGeometry.extent`'s formula -- for each
    row of `support` (S, normals), with `inv` = `inverse_cos(bearings, normals)`."""
    h = np.atleast_2d(support)
    out = np.empty((len(h), inv.shape[0]))
    for i in range(0, len(h), block):
        out[i:i + block] = (h[i:i + block, None, :] * inv[None]).min(axis=2)
    return out


# ------------------------------------------------------------------ the field table
class FieldTable:
    """The certified fields, tabulated for look-up by index (module docstring).

    Rows: (band, grid heading, yaw knot) for the moving fields, then one per yaw knot at rest;
    columns: diamond-angle cells. `extent` returns the field's boundary distance, m, for arrays of
    speeds, travel headings, yaw rates and bearings; the controller reads the same table through
    `state_rows` and `cells`.
    """

    def __init__(self, geometry: FieldGeometry, *, heading_cells: int = HEADING_CELLS,
                 diamond_cells: int = DIAMOND_CELLS, cell_samples: int = CELL_SAMPLES,
                 yaw_knot_step: float = YAW_KNOT_STEP, pad: float = TABLE_PAD_M):
        p = geometry.params
        if heading_cells < 4 or diamond_cells < 64 or cell_samples < 2 or yaw_knot_step <= 0.0:
            raise ValueError("table grid too coarse")
        if pad < 0.0:
            raise ValueError("the pad makes the table conservative; it is never negative")
        self.pad = float(pad)
        self.params = p
        self.tops = np.asarray(p.band_top_speeds, dtype=float)
        self.heading_cells, self.cells = int(heading_cells), int(diamond_cells)
        n = max(1, math.ceil(p.yaw_cap / yaw_knot_step - 1e-9))
        self.yaw_knots = np.minimum(np.arange(n + 1) * yaw_knot_step, p.yaw_cap)
        n_b, n_h, n_y = len(self.tops), self.heading_cells, len(self.yaw_knots)
        k = cell_samples - 1
        self.sample_bearings = diamond_to_bearing(-2.0 + 4.0 * np.arange(self.cells * k + 1)
                                                  / (self.cells * k))
        normals = geometry.normal_angles
        inv = inverse_cos(self.sample_bearings, normals)

        def pooled(rho: np.ndarray) -> np.ndarray:            # (..., cells * k + 1) -> (..., cells)
            return np.lib.stride_tricks.sliding_window_view(rho, k + 1, axis=-1)[..., ::k, :] \
                .max(axis=-1)

        headings = -math.pi + 2.0 * math.pi * np.arange(n_h) / n_h
        moving = np.empty((n_b, n_h, n_y, self.cells))
        for b, top in enumerate(self.tops):
            for y, w in enumerate(self.yaw_knots):
                h = np.stack([geometry.support(normals, float(top), float(hd), False,
                                               yaw_rate=float(w)) for hd in headings])
                moving[b, :, y] = pooled(extent_of_support(h, inv))
        rest_h = np.stack([geometry.support(normals, 0.0, 0.0, True, yaw_rate=float(w))
                           for w in self.yaw_knots])
        rest = pooled(extent_of_support(rest_h, inv))
        self.rest_base = n_b * n_h * n_y
        self.table = np.ascontiguousarray(
            np.concatenate([moving.reshape(self.rest_base, self.cells), rest]) + self.pad,
            dtype=np.float32)

        self.max_extent = float(self.table.max())

    def state_rows(self, speed, heading, yaw_rate
                   ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Per state, for bilinear interpolation in (heading, |yaw rate|): the row at the grid
        heading at or below its heading and the yaw knot at or below its |yaw rate|; the row one
        grid heading on (wrapping; the same row at rest, where the heading does not matter); the
        fraction toward the next yaw knot; the fraction toward the next grid heading. Speed m/s
        (composed), heading rad (travel direction, body frame), yaw rate rad/s."""
        p = self.params
        s = np.asarray(speed, dtype=float)
        w = np.abs(np.asarray(yaw_rate, dtype=float))
        if np.any(w > self.yaw_knots[-1] + 1e-9):
            raise ValueError("a yaw rate above the yaw cap: the fields do not cover it")
        reach = np.maximum(s, np.minimum(s + p.a_max * p.reach_time, p.speed_cap))
        if np.any(reach > self.tops[-1]):
            raise ValueError("a speed above the top band: the fields do not cover it")
        band = np.searchsorted(self.tops, reach, side="left")            # the first top >= reach
        n_y = len(self.yaw_knots)
        lo = np.clip(np.searchsorted(self.yaw_knots, w, side="right") - 1, 0, n_y - 2)
        f_yaw = np.clip((w - self.yaw_knots[lo])
                        / (self.yaw_knots[lo + 1] - self.yaw_knots[lo]), 0.0, 1.0)
        u = np.mod(np.asarray(heading, dtype=float) + math.pi, 2.0 * math.pi) \
            * (self.heading_cells / (2.0 * math.pi))
        h = np.minimum(np.floor(u).astype(np.int64), self.heading_cells - 1)
        f_head = np.clip(u - h, 0.0, 1.0)
        at_rest = (s <= p.standstill_enter) & (w <= p.standstill_yaw_enter)
        rest_row = self.rest_base + lo
        row_a = np.where(at_rest, rest_row, (band * self.heading_cells + h) * n_y + lo)
        row_b = np.where(at_rest, rest_row,
                         (band * self.heading_cells + (h + 1) % self.heading_cells) * n_y + lo)
        return row_a, row_b, f_yaw, np.where(at_rest, 0.0, f_head)

    def interpolate(self, cell: np.ndarray, off_a: np.ndarray, off_b: np.ndarray,
                    f_yaw: np.ndarray, f_head: np.ndarray) -> np.ndarray:
        """The bilinear extent, m, at bearing cells `cell`, for states given by their row offsets
        (row x cells, from `state_rows`) and fractions; float32, broadcast."""
        t = self.table.ravel()
        n = self.cells
        ia, ib = cell + off_a, cell + off_b
        a0 = t.take(ia)
        a = t.take(ia + n)
        a -= a0
        a *= f_yaw
        a += a0                                         # heading a, yaw interpolated
        b0 = t.take(ib)
        b = t.take(ib + n)
        b -= b0
        b *= f_yaw
        b += b0                                         # heading b, yaw interpolated
        b -= a
        b *= f_head
        b += a
        return b

    def cells_of(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """The bearing cell of body-frame points (x, y)."""
        d = diamond(x, y)
        c = ((d + 2.0) * (self.cells / 4.0)).astype(np.int32)
        return np.minimum(c, self.cells - 1)

    def extent(self, speed, heading, yaw_rate, bearing) -> np.ndarray:
        """The tabulated boundary distance, m, of each state's field along each bearing
        (broadcast)."""
        row_a, row_b, f_yaw, f_head = self.state_rows(speed, heading, yaw_rate)
        b = np.asarray(bearing, dtype=float)
        cell = self.cells_of(np.cos(b), np.sin(b))
        return self.interpolate(cell, (row_a * self.cells).astype(np.int32),
                                (row_b * self.cells).astype(np.int32),
                                f_yaw.astype(np.float32), f_head.astype(np.float32))


_TABLES: dict[tuple[CertifiedParams, bytes], FieldTable] = {}


def field_table(footprint_extent, params: CertifiedParams) -> FieldTable:
    """The table for these parameters and this footprint, built once per process."""
    probe = np.asarray(footprint_extent(np.linspace(-math.pi, math.pi, 720, endpoint=False)),
                       dtype=float)
    key = (params, probe.tobytes())
    if key not in _TABLES:
        _TABLES[key] = FieldTable(FieldGeometry(footprint_extent, params,
                                                n_normals=FIELD_NORMALS))
    return _TABLES[key]


# ------------------------------------------------------------------ route geometry
def clip_route(points: np.ndarray, arc: float) -> np.ndarray:
    """The polyline's first `arc` metres (all of it if it is shorter)."""
    p = np.asarray(points, dtype=float)
    seg = np.hypot(*np.diff(p, axis=0).T)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    if cum[-1] <= arc:
        return p
    i = max(int(np.searchsorted(cum, arc, side="left")), 1)       # cum[i-1] < arc <= cum[i]
    f = (arc - cum[i - 1]) / seg[i - 1] if seg[i - 1] > 0.0 else 0.0
    return np.vstack([p[:i], p[i - 1] + f * (p[i] - p[i - 1])])


def route_projection(q: np.ndarray, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """For points q (..., 2): the squared distance to the polyline, m^2, and the arc length of the
    nearest point on it, m (the first segment's on a tie)."""
    p = np.asarray(points, dtype=float)
    a, d = p[:-1], np.diff(p, axis=0)                          # (M, 2)
    ln2 = (d * d).sum(axis=1)
    ln = np.sqrt(ln2)
    cum = np.concatenate([[0.0], np.cumsum(ln)])[:-1]
    rel = q[..., None, :] - a                                  # (..., M, 2)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(ln2 > 0.0, (rel * d).sum(axis=-1) / ln2, 0.0)
    t = np.clip(t, 0.0, 1.0)
    diff = rel - t[..., None] * d
    dist2 = (diff * diff).sum(axis=-1)
    j = np.argmin(dist2, axis=-1)[..., None]
    best = np.take_along_axis(dist2, j, axis=-1)[..., 0]
    arc = cum[j[..., 0]] + np.take_along_axis(t, j, axis=-1)[..., 0] * ln[j[..., 0]]
    return best, arc


# ------------------------------------------------------------------ the controller
class FieldAwareMPPI:
    """B6: holonomic MPPI on the route ahead and the scan stream, its cost on the certified
    fields."""

    name = "B6_field_aware_mppi"
    input_kind = "classical"

    def __init__(self, params: MPPIParams, table: FieldTable, footprint_extent, *, dt: float,
                 v_max: float, vy_max: float, w_max: float, budget_limit: float,
                 lever_arm: float, a_max: float, alpha_max: float, a_decel_max: float,
                 estimate_alpha: float):
        if min(dt, v_max, vy_max, w_max, budget_limit, lever_arm, a_max, alpha_max,
               a_decel_max) <= 0.0:
            raise ValueError("dt, ceilings, budget, lever arm and acceleration limits must be "
                             "positive")
        if not 0.0 < estimate_alpha <= 1.0:
            raise ValueError("the estimate's filter gain is in (0, 1]")
        self.p = params
        self.table = table
        self.dt = float(dt)
        self.ceilings = np.array([v_max, vy_max, w_max], dtype=float)
        self.budget_limit, self.lever_arm = float(budget_limit), float(lever_arm)
        self.a_max, self.alpha_max = float(a_max), float(alpha_max)
        self.a_decel_max = float(a_decel_max)
        self.estimate_alpha = float(estimate_alpha)
        # The footprint per bearing cell, the largest sampled extent across it, squared: hard
        # rejection errs toward rejecting.
        k = FOOTPRINT_SAMPLES - 1
        fine = diamond_to_bearing(-2.0 + 4.0 * np.arange(table.cells * k + 1) / (table.cells * k))
        e = np.asarray(footprint_extent(fine), dtype=float)
        cell_max = np.lib.stride_tricks.sliding_window_view(e, k + 1)[::k].max(axis=1)
        self.footprint2 = (cell_max ** 2).astype(np.float32)
        # Where an unobserved bin's point goes (the shared convention): on THIS footprint's
        # boundary, the larger of the cell and its two neighbours, so a point on a cell edge is not
        # inside the neighbouring cell either.
        self.edge = np.sqrt(np.maximum.reduce([np.roll(self.footprint2, s) for s in (-1, 0, 1)])
                            .astype(float))
        # Every state lies within `travel` of the robot: the commands are feasible, so no faster
        # than the layer's speed cap (box and budget). A point farther than travel plus the largest
        # field is inside no state's field, nor its footprint.
        self.travel = float(table.params.speed_cap) * params.horizon_steps * self.dt
        self.reach = self.travel + table.max_extent
        self.reset()

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> FieldAwareMPPI:
        from pfbnav.safety.footprint_table import FootprintTable
        from pfbnav.world.strata import certified_params_from_config

        root = Path(__file__).resolve().parents[2]
        fp = FootprintTable.from_csv(root / cfg["robot"]["footprint_table"])
        a, r = cfg["actuation"], cfg["robot"]
        budget = a["wheel_budget"]
        dt = float(cfg["sim"]["dt"])
        tau = float(cfg["estimation"]["velocity"]["filter_tau"])
        return cls(MPPIParams.from_block(cfg["baselines"]["mppi"]),
                   field_table(fp.extent, certified_params_from_config(cfg)), fp.extent,
                   dt=dt, v_max=float(a["v_max"]),
                   vy_max=float(a["vy_max"]), w_max=float(a["w_max"]),
                   budget_limit=float(budget["filter_fraction"]) * float(budget["v_rim"]),
                   lever_arm=float(r["half_wheelbase"]) + float(r["half_track"]),
                   a_max=float(a["a_max"]), alpha_max=float(a["alpha_max"]),
                   a_decel_max=float(a["a_decel_max"]), estimate_alpha=dt / (tau + dt))

    # ------------------------------------------------------------------ episode
    def reset(self) -> None:
        """A new episode: nominal and executing command zero, counters cleared, the sampler on the
        default key until `begin_episode` names the scenario."""
        self.nominal = np.zeros((self.p.horizon_steps, 3))
        self.previous = np.zeros(3)      # the command executing, as modelled (module docstring)
        self._before = np.zeros(3)       # the one executing before it
        self.issued = np.zeros(3)        # the last command issued (before the base's limiter)
        self.steps = 0
        self.all_rejected_steps = 0
        self.held_steps = 0
        self._seed(None)

    def begin_episode(self, episode: dict[str, Any]) -> None:
        """Seed the sampler from the scenario: the same scenario draws the same noise whatever the
        order it is run in."""
        seed = int(episode["scenario_seed"])
        if seed < 0:
            raise ValueError(f"scenario_seed must be non-negative, got {seed}")
        self._seed(seed)

    def _seed(self, scenario_seed: int | None) -> None:
        self.seed_key = [SEED_DOMAIN] if scenario_seed is None else [SEED_DOMAIN, scenario_seed]
        self.rng = np.random.default_rng(self.seed_key)

    def episode_report(self) -> dict[str, Any]:
        return {"mppi_steps": self.steps, "mppi_all_rejected": self.all_rejected_steps,
                "mppi_held_steps": self.held_steps, "mppi_seed_key": list(self.seed_key)}

    # ------------------------------------------------------------------ the parts
    def feasible(self, cmd: np.ndarray) -> np.ndarray:
        """Commands clamped to the box and projected into the wheel budget, as the base does."""
        return project_budget(clamp_box(cmd, *self.ceilings), self.budget_limit, self.lever_arm)

    def limit(self, cmd: np.ndarray, prev: np.ndarray) -> np.ndarray:
        """`limit_acceleration` over the last axis: the translation's change shortened along
        itself to a_max dt (a_decel_max dt when the command is slower than `prev`), the yaw rate's
        change clipped to alpha_max dt."""
        c, q = np.asarray(cmd, dtype=float), np.asarray(prev, dtype=float)
        d = c[..., :2] - q[..., :2]
        slowing = np.hypot(c[..., 0], c[..., 1]) < np.hypot(q[..., 0], q[..., 1])
        max_dv = np.where(slowing, self.a_decel_max, self.a_max) * self.dt
        norm = np.hypot(d[..., 0], d[..., 1])
        with np.errstate(divide="ignore", invalid="ignore"):
            scale = np.where(norm > max_dv, max_dv / norm, 1.0)
        out = np.empty(np.broadcast_shapes(c.shape, q.shape))
        out[..., :2] = q[..., :2] + d * scale[..., None]
        max_dw = self.alpha_max * self.dt
        out[..., 2] = q[..., 2] + np.clip(c[..., 2] - q[..., 2], -max_dw, max_dw)
        return out

    def applied_sequence(self, u: np.ndarray, start: np.ndarray) -> np.ndarray:
        """What the base applies to each command of u (K, H, 3), in turn, from the command executing
        `start` (3,): box, budget, then the limiter against the command before."""
        out = np.empty_like(np.asarray(u, dtype=float))
        prev = np.broadcast_to(np.asarray(start, dtype=float), (len(out), 3))
        for t in range(out.shape[1]):
            prev = self.limit(self.feasible(u[:, t]), prev)
            out[:, t] = prev
        return out

    def field_motion(self, v: np.ndarray, estimate: np.ndarray
                     ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Per state of v (K, H, 3): the speed, travel heading and yaw rate that size its field --
        the certified composition, max(|estimate|, |command executing|), with the estimate
        predicted along the rollout by the estimator's first-order filter from `estimate` (3,)."""
        est = np.empty_like(v)
        e = np.broadcast_to(np.asarray(estimate, dtype=float), (len(v), 3))
        a = self.estimate_alpha
        for t in range(v.shape[1]):
            e = e + a * (v[:, t] - e)
            est[:, t] = e
        s_est, s_cmd = np.hypot(est[..., 0], est[..., 1]), np.hypot(v[..., 0], v[..., 1])
        by_est = s_est >= s_cmd
        speed = np.where(by_est, s_est, s_cmd)
        heading = np.where(by_est, np.arctan2(est[..., 1], est[..., 0]),
                           np.arctan2(v[..., 1], v[..., 0]))
        yaw = np.maximum(np.abs(est[..., 2]), np.abs(v[..., 2]))
        return speed, heading, yaw

    def unobserved_edge(self, psi: np.ndarray) -> np.ndarray:
        """B6's own footprint boundary along bearings psi, m: where an unobserved bin's point goes."""
        p = np.asarray(psi, dtype=float)
        return self.edge[self.table.cells_of(np.cos(p), np.sin(p))]

    def scan_points(self, inp: ClassicalInput) -> np.ndarray:
        """The scan as body-frame points (P, 2) within reach of any rollout. An invalid bin is a
        point on the footprint's boundary on its bearing (the shared convention; hard rule 5)."""
        s = inp.scan
        r, _ = resolve_unobserved(s.ranges, s.bearings, s.valid, self.unobserved_edge)
        b = np.asarray(s.bearings, dtype=float)
        keep = r <= self.reach
        return np.column_stack([r[keep] * np.cos(b[keep]), r[keep] * np.sin(b[keep])])

    def rollout(self, v: np.ndarray) -> np.ndarray:
        """States (K, H, 3) = (x, y, theta) after each command of v (K, H, 3), from the origin,
        each step at its mid-step heading."""
        dt = self.dt
        theta = np.cumsum(v[..., 2] * dt, axis=1)
        mid = theta - 0.5 * v[..., 2] * dt
        c, s = np.cos(mid), np.sin(mid)
        dx = (c * v[..., 0] - s * v[..., 1]) * dt
        dy = (s * v[..., 0] + c * v[..., 1]) * dt
        return np.stack([np.cumsum(dx, axis=1), np.cumsum(dy, axis=1), theta], axis=-1)

    def cost_terms(self, v: np.ndarray, inp: ClassicalInput) -> dict[str, np.ndarray]:
        """Each declared term per rollout (K,), weighted, and `rejected` (K,), for APPLIED command
        sequences v (K, H, 3) (`applied_sequence`)."""
        p = self.p
        states = self.rollout(v)
        xy = states[..., :2]
        # A state lies within `travel` of the robot, so its nearest route point lies within
        # 2 x travel of the route's start unless the route winds back -- and progress measured on
        # a route that winds back is progress the robot never made.
        dist2, arc = route_projection(xy, clip_route(inp.route.points, 2.0 * self.travel))
        prev = np.concatenate([np.broadcast_to(self.previous, (len(v), 1, 3)), v[:, :-1]], axis=1)
        change = ((v - prev) ** 2).sum(axis=(1, 2))
        breach, rejected = self.field_counts(self.field_motion(v, inp.velocity), states,
                                             self.scan_points(inp))
        return {"route_tracking": p.route_tracking * dist2.sum(axis=1),
                "goal_progress": -p.goal_progress * arc[:, -1],
                "field_breach": p.field_breach * breach,
                "smoothness": p.smoothness * change,
                "rejected": rejected}

    def field_counts(self, motion: tuple[np.ndarray, np.ndarray, np.ndarray], states: np.ndarray,
                     pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Per rollout: the number of (state, point) pairs with the point inside the state's
        certified field (`motion`: the speed, heading and yaw rate sizing it, from `field_motion`),
        and whether any point lies inside the footprint at any state.

        Each state's view of the points -- body-frame coordinates and squared range -- is three
        small matrix products in float32: [bx, by, r^2] = [state terms] @ [point terms]."""
        k, h = states.shape[:2]
        counts, rejected = np.zeros(k, dtype=np.int64), np.zeros(k, dtype=bool)
        if len(pts) == 0:
            return counts.astype(float), rejected
        cells = self.table.cells
        row_a, row_b, f_yaw, f_head = self.table.state_rows(*motion)
        off_a = (row_a * cells).astype(np.int32).reshape(k, h, 1)
        off_b = (row_b * cells).astype(np.int32).reshape(k, h, 1)
        f_yaw = f_yaw.astype(np.float32).reshape(k, h, 1)
        f_head = f_head.astype(np.float32).reshape(k, h, 1)
        x, y, theta = states[..., 0], states[..., 1], states[..., 2]
        c, s = np.cos(theta), np.sin(theta)
        one = np.ones_like(x)
        # bx = c px + s py - (c x + s y);  by = c py - s px - (c y - s x);
        # r^2 = |p|^2 - 2 (x px + y py) + |xy|^2.
        sx = np.stack([c, s, -(c * x + s * y)], axis=-1).astype(np.float32)
        sy = np.stack([-s, c, -(c * y - s * x)], axis=-1).astype(np.float32)
        sr = np.stack([-2.0 * x, -2.0 * y, x * x + y * y, one], axis=-1).astype(np.float32)
        p3 = np.stack([pts[:, 0], pts[:, 1], np.ones(len(pts))]).astype(np.float32)
        p4 = np.stack([pts[:, 0], pts[:, 1], np.ones(len(pts)),
                       (pts * pts).sum(axis=1)]).astype(np.float32)
        for k0 in range(0, k, CHUNK):
            sl = slice(k0, k0 + CHUNK)
            bx = sx[sl] @ p3                                       # (chunk, H, P)
            by = sy[sl] @ p3
            r2 = sr[sl] @ p4
            cell = self.table.cells_of(bx, by)
            e = self.table.interpolate(cell, off_a[sl], off_b[sl], f_yaw[sl], f_head[sl])
            e *= e
            counts[sl] = np.count_nonzero(r2 < e, axis=(1, 2))
            rejected[sl] = (r2 < self.footprint2.take(cell)).any(axis=(1, 2))
        return counts.astype(float), rejected

    def weights(self, terms: dict[str, np.ndarray]) -> np.ndarray | None:
        """Normalised path-integral weights on per-step costs; None when every rollout is
        rejected."""
        total = sum(terms[k] for k in WEIGHTS) / self.p.horizon_steps
        ok = ~terms["rejected"]
        if not ok.any():
            return None
        w = np.zeros_like(total)
        w[ok] = np.exp(-(total[ok] - total[ok].min()) / self.p.temperature)
        return w / w.sum()

    # ------------------------------------------------------------------ the interface
    def command(self, inp: ClassicalInput) -> np.ndarray:
        """The body-frame command (vx, vy, wz) in m/s and rad/s."""
        if not isinstance(inp, ClassicalInput):
            raise TypeError(f"field-aware MPPI takes a ClassicalInput, got {type(inp).__name__}")
        if abs(float(inp.dt) - self.dt) > 1e-12:
            raise ValueError(f"the control period is {inp.dt} s, the rollouts step {self.dt} s")
        p = self.p
        self.steps += 1
        if inp.stop_active:                  # the layer replaced the last command by zero
            self.held_steps += 1
            self.previous = self.limit(np.zeros(3), self._before)
        start = self.previous
        sigma = np.asarray(p.noise_std, dtype=float)
        for _ in range(p.iterations):
            eps = self.rng.standard_normal((p.samples, p.horizon_steps, 3)) * sigma
            eps[0] = 0.0
            u = self.feasible(self.nominal[None] + eps)
            w = self.weights(self.cost_terms(self.applied_sequence(u, start), inp))
            if w is None:
                self.all_rejected_steps += 1
                self.nominal = np.zeros_like(self.nominal)
                self._before, self.previous = start, self.limit(np.zeros(3), start)
                self.issued = np.zeros(3)
                return np.zeros(3)
            self.nominal = np.tensordot(w, u, axes=1)
        out = self.nominal[0].copy()
        self.nominal = np.vstack([self.nominal[1:], self.nominal[-1:]])
        self._before, self.previous = start, self.limit(out, start)
        self.issued = out
        return out

    def act(self, inp: ClassicalInput) -> np.ndarray:
        return np.clip(self.command(inp) / self.ceilings, -1.0, 1.0)
