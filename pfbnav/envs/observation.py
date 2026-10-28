"""The 82-D observation, assembled and scaled. Pure functions; no simulator.

The layout was ruled on 2026-09-23 and is recorded in `docs/decision_log.md`; this module
implements it and nothing else. Changing either the channels or their scaling needs a
decision-log entry in the same change (hard rule 7).

    block          channels  contents
    -----------------------------------------------------------------------------------
    scan               36    policy ranges / scanners.range_max, in [0, 1]
    validity           36    1.0 where the bin is observed, 0.0 where it is not
    velocity            3    (vx / v_max, vy / vy_max, wz / w_max), estimated, clipped
    lookahead           3    (d_wp / lookahead, sin beta_wp, cos beta_wp)
    goal                3    (d_g / goal_scale, sin beta_g, cos beta_g)
    certified           1    stop_active, 1.0 or 0.0
    -----------------------------------------------------------------------------------
    total              82    (79 for the no-planner ablation, which drops the lookahead)

**Scaling** (VERIFY, recorded with the layout): every channel is brought to order one, so no
input dominates the first layer by units alone. Ranges divide by `range_max`; velocities by
their own axis ceilings, as the action is scaled (`actuation.v_max`, `vy_max`, `w_max`; the box is
square, 0.50 / 0.50, since A20, 2026-10-19); the
lookahead distance by the lookahead itself, so a robot tracking normally sees ~1; the goal
distance by a configured scale, since a goal may start 15 m away and a saturating channel
would make every distant goal look alike. Distances are clipped, never wrapped.

**Validity is a separate channel, not a sentinel range** (hard rule 5): "unobserved" and
"an obstacle at that range" must not be the same input. The channels are constant while
every bin is valid, which is why sensor dropout is a declared training augmentation.

**Pose-agnostic:** no absolute x, y or theta. A policy given its world pose can memorise a
layout instead of reacting to what it senses.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import ArrayLike

SCAN = "scan"
VALIDITY = "validity"
VELOCITY = "velocity"
LOOKAHEAD = "lookahead"
GOAL = "goal"
CERTIFIED = "certified"
# Order is the observation's order; the blocks and their widths are the ruled layout.
BLOCKS: tuple[tuple[str, int], ...] = (
    (SCAN, 36), (VALIDITY, 36), (VELOCITY, 3), (LOOKAHEAD, 3), (GOAL, 3), (CERTIFIED, 1),
)
FULL_DIM = sum(width for _name, width in BLOCKS)
NO_PLANNER_DIM = FULL_DIM - dict(BLOCKS)[LOOKAHEAD]


@dataclass(frozen=True)
class ObservationSpec:
    """What the observation looks like, and the scales it is built with."""

    policy_bins: int
    range_max: float
    v_max: float
    vy_max: float
    w_max: float
    lookahead: float
    goal_scale: float
    include_lookahead: bool = True

    @property
    def dim(self) -> int:
        return FULL_DIM if self.include_lookahead else NO_PLANNER_DIM

    @property
    def slices(self) -> dict[str, slice]:
        """Where each block sits, so a diagnostic can name a channel rather than an index."""
        out, start = {}, 0
        for name, width in BLOCKS:
            if name == LOOKAHEAD and not self.include_lookahead:
                continue
            out[name] = slice(start, start + width)
            start += width
        return out

    @classmethod
    def from_config(cls, cfg: dict[str, Any], *, include_lookahead: bool = True
                    ) -> ObservationSpec:
        env = cfg.get("env", {})
        obs = env.get("observation", {})
        return cls(policy_bins=int(cfg["scanners"]["policy_bins"]),
                   range_max=float(cfg["scanners"]["range_max"]),
                   v_max=float(cfg["actuation"]["v_max"]),
                   vy_max=float(cfg["actuation"]["vy_max"]),
                   w_max=float(cfg["actuation"]["w_max"]),
                   lookahead=float(cfg["planner"]["lookahead"]),
                   goal_scale=float(obs["goal_scale"]),
                   include_lookahead=include_lookahead)


def assemble(spec: ObservationSpec, *, ranges: ArrayLike, valid: ArrayLike,
             velocity: ArrayLike, lookahead_body: tuple[float, float, float],
             goal_body: tuple[float, float, float], stop_active: bool) -> np.ndarray:
    """One observation, float32, in the ruled order. Raises on a wrong-sized scan."""
    r = np.asarray(ranges, dtype=float).ravel()
    v = np.asarray(valid, dtype=bool).ravel()
    if r.size != spec.policy_bins or v.size != spec.policy_bins:
        raise ValueError(f"the policy stream has {spec.policy_bins} bins, got "
                         f"{r.size} ranges and {v.size} flags")
    vel = np.asarray(velocity, dtype=float).ravel()
    if vel.size != 3:
        raise ValueError(f"velocity is (vx, vy, wz), got {vel.size} values")

    parts = [np.clip(r / spec.range_max, 0.0, 1.0), v.astype(float),
             np.clip(vel / np.array([spec.v_max, spec.vy_max, spec.w_max]), -1.0, 1.0)]
    if spec.include_lookahead:
        parts.append(_point_block(lookahead_body, spec.lookahead))
    parts.append(_point_block(goal_body, spec.goal_scale))
    parts.append(np.array([1.0 if stop_active else 0.0]))
    out = np.concatenate(parts).astype(np.float32)
    if out.size != spec.dim:
        raise ValueError(f"assembled {out.size} channels, expected {spec.dim}")
    return out


def _point_block(body: tuple[float, float, float], scale: float) -> np.ndarray:
    """(distance / scale clipped to [0, 1], sin bearing, cos bearing)."""
    distance, sin_b, cos_b = (float(v) for v in body)
    if scale <= 0.0:
        raise ValueError("a distance scale must be positive")
    return np.array([min(distance / scale, 1.0), sin_b, cos_b])


def bounds(spec: ObservationSpec) -> tuple[np.ndarray, np.ndarray]:
    """Per-channel low and high, for the Gymnasium Box: every channel is bounded."""
    low, high = [], []
    for name, width in BLOCKS:
        if name == LOOKAHEAD and not spec.include_lookahead:
            continue
        if name in (SCAN, VALIDITY, CERTIFIED):
            low += [0.0] * width
            high += [1.0] * width
        elif name == VELOCITY:
            low += [-1.0] * width
            high += [1.0] * width
        else:                      # a point block: distance in [0, 1], then sin and cos
            low += [0.0, -1.0, -1.0]
            high += [1.0, 1.0, 1.0]
    return np.array(low, dtype=np.float32), np.array(high, dtype=np.float32)


@dataclass(frozen=True)
class DropoutSpec:
    """The declared sensor-dropout augmentation (supervisor 2026-09-26, replacing 09-23).

    Two mechanisms, because they model different failures:

      * **A blocked arc, per episode.** With probability `arc_probability` an episode begins
        with a contiguous run of `arc_bins` bins invalid, and they stay invalid for the whole
        episode. This is the failure that matters -- a smeared lens, tape, a mounting that
        sees the payload -- and a policy cannot average it away over a few steps the way it
        can with independent per-step noise.
      * **Per-bin, per-step invalidation** at `per_step_rate`, which keeps the validity
        channels moving on every episode rather than only on the sampled ones.

    Training only, and **never applied to the filter's 120-bin stream**: the certified layer
    and the PFB must see the sensor the robot really has. Degrading their input would make
    the safety layers look better than they are, which is the opposite of the point.
    """

    arc_probability: float
    arc_bins: tuple[int, int]      # inclusive range of contiguous bins to blind
    per_step_rate: float

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> DropoutSpec:
        d = cfg["env"]["observation"]["dropout"]
        lo, hi = (int(v) for v in d["arc_bins"])
        return cls(float(d["arc_probability"]), (lo, hi), float(d["per_step_rate"]))

    def __post_init__(self) -> None:
        if not 0.0 <= self.arc_probability <= 1.0:
            raise ValueError(f"arc_probability must be in [0, 1], got {self.arc_probability}")
        if not 0.0 <= self.per_step_rate <= 1.0:
            raise ValueError(f"per_step_rate must be in [0, 1], got {self.per_step_rate}")
        lo, hi = self.arc_bins
        if lo < 1 or hi < lo:
            raise ValueError(f"arc_bins must be 1 <= lo <= hi, got {self.arc_bins}")

    @property
    def active(self) -> bool:
        return self.arc_probability > 0.0 or self.per_step_rate > 0.0


def sample_blocked_arc(spec: DropoutSpec, n_bins: int, rng) -> np.ndarray:
    """The episode's persistent blocked arc as a boolean mask over the policy bins.

    Drawn once per episode at reset. The arc wraps around the 360 deg ring, because a bin
    index of 0 is not a physical boundary. An episode that does not draw an arc returns an
    all-False mask, and the draw happens either way so the stream of random numbers does not
    depend on the outcome (the failure `world.layouts.generate` hit on 2026-09-23).
    """
    if n_bins <= 0:
        raise ValueError(f"n_bins must be positive, got {n_bins}")
    draw = rng.random()
    lo, hi = spec.arc_bins
    width = int(rng.integers(lo, hi + 1))
    start = int(rng.integers(0, n_bins))
    mask = np.zeros(n_bins, dtype=bool)
    if draw < spec.arc_probability:
        mask[(start + np.arange(min(width, n_bins))) % n_bins] = True
    return mask


def dropout(ranges: ArrayLike, valid: ArrayLike, rng, spec: DropoutSpec,
            blocked_arc: ArrayLike | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Apply the augmentation: the episode's blocked arc, plus per-step invalidation.

    A dropped bin is flagged invalid and its range is set to **0.0, not range_max**
    (corrected 2026-09-26). range_max reads as "nothing out there", which is unobserved space
    presented as free and the one thing hard rule 5 forbids; 0.0 reads as "something right
    here", which is the safe direction and cannot be confused with a real return because the
    validity channel beside it is 0. The policy is told the bin is unobserved, and told it in
    the direction that does not invite it to drive into the gap.
    """
    r = np.array(ranges, dtype=float).ravel()
    v = np.array(valid, dtype=bool).ravel()
    dropped = np.zeros(r.size, dtype=bool)
    if blocked_arc is not None:
        arc = np.asarray(blocked_arc, dtype=bool).ravel()
        if arc.size != r.size:
            raise ValueError(f"blocked arc covers {arc.size} bins, the scan has {r.size}")
        dropped |= arc
    if spec.per_step_rate > 0.0:
        dropped |= rng.random(r.size) < spec.per_step_rate
    if not dropped.any():
        return r, v
    r[dropped] = 0.0
    v[dropped] = False
    return r, v
