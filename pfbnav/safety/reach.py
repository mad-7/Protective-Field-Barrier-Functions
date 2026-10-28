"""The reachable-motion field's ASSESSMENT tooling (ruled 2026-10-18, option a; adopted as A20 in
`pfbnav/safety/certified.py`, which holds the one implementation of its mathematics). This module
keeps what the assessment needed and the adopted layer does not: limiter bounds that differ from
the configuration (the alpha_max / a_max sensitivity) and the "ruled" (along-heading) variant that
A20 rejected. `tests/test_reach.py` checks that its "full" variant equals the adopted field.

The safety property is **containment**: everything the footprint sweeps from the stop trigger to
rest lies inside the field in force at the trigger. Within the reach time T = response_time + one
control step the command still passes, and the acceleration limiter lets it grow:

  * translation: the band is chosen from v + a_max T (capped by the velocity box and the wheel
    budget), and its depth is the travel over T at the band's top speed plus braking from it;
  * rotation: the sweep is the interval of headings reachable by accelerating at alpha_max for T
    from the current yaw rate (capped by w_max and the wheel budget), then braking at
    alpha_brake -- both senses, since a slow yaw rate can be reversed inside T;
  * at rest too: from rest the robot can reach a_max T in ANY direction and alpha_max T in
    either sense, so the at-rest field carries the isotropic reach a T^2/2 + (a T)^2 / (2 a_b)
    and the two-sided sweep.

Every moving field is taken no smaller than the at-rest field (pointwise maximum of support
functions), so the at-rest field stays the infimum of the moving fields (ruled 2026-09-27: a
robot that stops must not find its at-rest field larger than the field it drove in).

Two translation variants, both reported by the assessment (`variant`):

  * "ruled": the reach along the travel direction only, as the ruling words it;
  * "full": also the SWERVE an isotropic limiter allows -- the velocity may change direction by
    up to a_max T inside T, then brakes along its new direction -- which a holonomic controller
    (any learned policy) can command. Containment for every controller needs it; whether to
    adopt it is the supervisor's.

This module imports `certified` (geometry only) and nothing from `pfb.py`. It is outside the
generator's and the environment's import closures until adopted.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import ArrayLike

from pfbnav.safety.certified import (
    CertifiedParams,
    FieldGeometry,
    band_depth,
    band_index,
    box_budget_speed_cap,
    coupling,
    reach_speed,
    rest_reach,
    swerve,
    yaw_excursion,
    yaw_interval,
)

VARIANTS = ("ruled", "full")


@dataclass(frozen=True)
class ReachParams:
    reach_time: float      # s, T = response time + one control step
    a_max: float           # m/s^2, the limiter's acceleration bound (isotropic in translation)
    alpha_max: float       # rad/s^2, the limiter's yaw-acceleration bound
    a_brake: float         # m/s^2, braking the fields assume (<= the limiter's deceleration)
    alpha_brake: float     # rad/s^2, yaw braking the fields assume (the symmetric limiter)
    speed_cap: float       # m/s, the largest planar speed inside the box and the wheel budget
    yaw_cap: float         # rad/s, min(w_max, budget / lever arm)
    # m/s, the largest composed speed the layer can still call AT REST: the standstill detector's
    # exit threshold (hysteresis), so the at-rest field must contain the reach from any speed up
    # to it -- the recovery creeps at exactly this speed (A20).
    rest_speed: float

    def __post_init__(self) -> None:
        for name in ("reach_time", "a_max", "alpha_max", "a_brake", "alpha_brake", "speed_cap",
                     "yaw_cap", "rest_speed"):
            if not getattr(self, name) > 0.0:
                raise ValueError(f"{name} must be positive")

    @classmethod
    def from_config(cls, cfg: dict[str, Any], *, a_max: float | None = None,
                    alpha_max: float | None = None) -> ReachParams:
        """From the configuration; `a_max` / `alpha_max` override the limiter's bounds for the
        sensitivity the supervisor asked for. The limiter is symmetric in yaw, so the yaw
        braking follows alpha_max; translational braking stays the certified a_brake (the
        limiter's deceleration is bounded separately, a_decel_max <= a_brake)."""
        act, cert = cfg["actuation"], cfg["certified"]
        a = float(act["a_max"] if a_max is None else a_max)
        alpha = float(act["alpha_max"] if alpha_max is None else alpha_max)
        budget = cfg["actuation"]["wheel_budget"]
        limit = float(budget["filter_fraction"]) * float(budget["v_rim"])
        lever = float(cfg["robot"]["half_wheelbase"]) + float(cfg["robot"]["half_track"])
        return cls(reach_time=float(cert["response_time"]) + float(cfg["sim"]["dt"]),
                   a_max=a, alpha_max=alpha, a_brake=float(cert["a_brake"]),
                   alpha_brake=alpha,
                   speed_cap=box_budget_speed_cap(float(act["v_max"]), float(act["vy_max"]),
                                                  limit),
                   yaw_cap=min(float(act["w_max"]), limit / lever),
                   rest_speed=float(cert["standstill_exit"]))
















class ReachFields:
    """The reachable-motion field with FieldGeometry's interface (`support`, `extent`, `select`,
    `lateral_reach`), so the strata and D2 helpers apply unchanged. `params` is the certified
    layer's (margins, band tops); `reach` the limiter's bounds."""

    def __init__(self, fields: FieldGeometry, reach: ReachParams, variant: str = "ruled"):
        if variant not in VARIANTS:
            raise ValueError(f"variant must be one of {VARIANTS}")
        self.fields = fields
        self.params: CertifiedParams = fields.params
        self.reach = reach
        self.variant = variant
        self.normal_angles = fields.normal_angles

    def swept_interval(self, normal_angles: ArrayLike, lo: float, hi: float) -> np.ndarray:
        """Support of the footprint rotated through every angle in [lo, hi] (lo <= 0 <= hi),
        exactly: the hull support at the interval's ends, or a hull vertex inside it."""
        a = np.asarray(normal_angles, dtype=float)
        if not lo <= 0.0 <= hi:
            raise ValueError("the interval must contain the starting heading")
        if hi - lo == 0.0:
            return self.fields.footprint_support(a)
        f = self.fields
        if hi - lo >= 2.0 * math.pi:
            return np.full(a.shape, float(f._hull_r.max()))
        flat = a.ravel()
        ends = np.maximum(f._hull_support(flat - lo), f._hull_support(flat - hi))
        inner = f._hull_range_max(flat - hi, hi - lo)
        return np.maximum(ends, inner).reshape(a.shape)

    def rest_support(self, normal_angles: ArrayLike, yaw_rate: float = 0.0) -> np.ndarray:
        """As `FieldGeometry.rest_support`, with this module's limiter bounds."""
        lo, hi = yaw_interval(yaw_rate, self.reach)
        return (self.swept_interval(normal_angles, lo, hi) + rest_reach(self.reach)
                + self.params.lateral_margin)

    def support(self, normal_angles: ArrayLike, band_top_speed: float, heading: float,
                at_rest: bool, *, yaw_rate: float = 0.0) -> np.ndarray:
        a = np.asarray(normal_angles, dtype=float)
        if at_rest:
            return self.rest_support(a, yaw_rate)
        # Every moving field contains the at-rest field at the same yaw rate, so the at-rest
        # field stays the infimum of the moving fields (2026-09-27) at every yaw rate.
        rest = self.rest_support(a, yaw_rate)
        p = self.params
        lo, hi = yaw_interval(yaw_rate, self.reach)
        c = np.cos(a - heading)
        lead = np.maximum(0.0, c)
        travel = band_depth(band_top_speed, self.reach) * lead
        margin = (p.margin - p.lateral_margin) * lead
        if self.variant == "full":
            travel = np.maximum(travel, swerve(c, band_top_speed, self.reach))
            # S2 nesting (ruled 2026-10-19), as adopted in `FieldGeometry.support`: the yaw
            # interval carries e(0) more, and the margin term is at least the at-rest reach.
            e = yaw_excursion(abs(yaw_rate), self.reach) + yaw_excursion(0.0, self.reach)
            lo, hi = -e, e
            margin = np.maximum(margin, rest_reach(self.reach))
            travel = travel + coupling(band_top_speed, yaw_rate, self.reach)
        moving = self.swept_interval(a, lo, hi) + travel + margin + p.lateral_margin
        return np.maximum(moving, rest)

    def extent(self, bearings: ArrayLike, band_top_speed: float, heading: float,
               at_rest: bool, *, yaw_rate: float = 0.0) -> np.ndarray:
        b = np.asarray(bearings, dtype=float)
        h = self.support(self.normal_angles, band_top_speed, heading, at_rest,
                         yaw_rate=yaw_rate)
        return (h[None, :] * self.fields._inverse_cos(b)).min(axis=1)

    def lateral_reach(self, band_top_speed: float, heading: float, at_rest: bool, *,
                      yaw_rate: float = 0.0) -> float:
        sides = np.array([heading + math.pi / 2, heading - math.pi / 2])
        return float(self.support(sides, band_top_speed, heading, at_rest,
                                  yaw_rate=yaw_rate).max())

    def band_for(self, speed: float) -> float:
        """The band top chosen for a composed speed: from the reachable speed. Raises where no
        band covers it (`band_index`), which the assessment reports."""
        tops = self.params.band_top_speeds
        return tops[band_index(reach_speed(speed, self.reach), tops)]

    def select(self, bearings: ArrayLike, speed: float, heading: float, at_rest: bool, *,
               yaw_rate: float = 0.0) -> np.ndarray:
        if at_rest:
            return self.extent(bearings, 0.0, heading, True, yaw_rate=yaw_rate)
        return self.extent(bearings, self.band_for(speed), heading, False, yaw_rate=yaw_rate)
