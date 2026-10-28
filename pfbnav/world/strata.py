"""Aisle strata by per-side clearance, the D2 floor, and each stratum's traversal speed.

Pure functions.

D2 (supervisor review, 2026-09-17): aisle strata are defined by *per-side clearance* -- the
gap between the robot's side and the aisle wall when the robot is centred -- not by aisle
width, and a stationary robot centred in the narrowest aisle must trigger no protective
stop. The certified field is directional (ruling 2 on the WP1 report), so two quantities
follow, both for a robot aligned with the aisle:

  * **The at-rest floor.** At rest the field is the footprint grown by the margin m all
    round. Walls at lateral distance w breach it exactly when w is below its sideways
    support, so every stratum's clearance must exceed that support minus half_width.
  * **The traversal speed** of a stratum (ruling 3): the highest band-top speed whose
    moving field, travelling along the aisle, still fits between the walls. The field is
    the Minkowski sum of the footprint with the braking segment, offset by the margins
    (certified.py), so its sideways reach is e(+-90 deg) + lateral margin at every speed:
    an aligned aisle either admits every band or none.

The narrow stratum's floor itself is set by ruling 4 as lateral margin + G1b's peak
cross-track (the committed tracking tolerance) + slack; the at-rest floor is checked
against it.

A robot turned in a narrow aisle carries a field reaching towards its circumscribed radius,
which no narrow aisle admits; that case belongs to the planner, not to the strata.

Frames: body frame; metres; bearings in radians.
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import ArrayLike

from pfbnav.safety.certified import CertifiedParams, FieldGeometry, box_budget_speed_cap

# Bisection tolerance on the traversal speed, m/s.
SPEED_TOL = 1e-5


@dataclass(frozen=True)
class Stratum:
    name: str
    min_clearance: float   # m per side, inclusive
    max_clearance: float   # m per side, exclusive except for the widest stratum

    def aisle_width(self, half_width: float, clearance: float) -> float:
        """Aisle width, m, for a per-side clearance inside this stratum."""
        return 2.0 * (half_width + clearance)


def strata_from_config(cfg: dict[str, Any]) -> list[Stratum]:
    strata = [Stratum(str(s["name"]), float(s["min_clearance"]), float(s["max_clearance"]))
              for s in cfg["world"]["aisle_strata"]]
    for a, b in itertools.pairwise(strata):
        if b.min_clearance != a.max_clearance:
            raise ValueError(f"strata {a.name} and {b.name} are not contiguous")
    if any(s.max_clearance <= s.min_clearance for s in strata):
        raise ValueError("each stratum needs max_clearance > min_clearance")
    return strata


def certified_params_from_config(cfg: dict[str, Any]) -> CertifiedParams:
    c = cfg["certified"]
    return CertifiedParams(
        response_time=float(c["response_time"]), a_brake=float(c["a_brake"]),
        restart_delay=float(c["restart_delay"]), n_field_sets=int(c["n_field_sets"]),
        margin=float(c["margin"]), lateral_margin=float(c["lateral_margin"]),
        standstill_enter=float(c["standstill_enter"]),
        standstill_exit=float(c["standstill_exit"]),
        standstill_yaw_enter=float(c["standstill_yaw_enter"]),
        standstill_yaw_exit=float(c["standstill_yaw_exit"]),
        band_top_speeds=tuple(float(v) for v in c["band_top_speeds"]),
        warning_scale=float(c["warning_scale"]), warning_extra=float(c["warning_extra"]),
        warning_extra_abeam=float(c["warning_extra_abeam"]),
        # The yaw braking the fields assume IS the limiter's symmetric yaw bound (2026-10-16).
        alpha_brake=float(cfg["actuation"]["alpha_max"]),
        **reach_from_config(cfg))


def reach_from_config(cfg: dict[str, Any]) -> dict[str, float]:
    """The reachable-motion parameters (A20, ruled 2026-10-18): T = response time + one control
    step (`sim.dt`); the limiter's bounds; the largest planar speed inside the velocity box and the
    base's final wheel-budget ceiling (the filter share, which no command exceeds); and the yaw
    cap min(w_max, that budget / lever arm)."""
    act = cfg["actuation"]
    budget = act["wheel_budget"]
    limit = float(budget["filter_fraction"]) * float(budget["v_rim"])
    lever = float(cfg["robot"]["half_wheelbase"]) + float(cfg["robot"]["half_track"])
    return {"reach_time": float(cfg["certified"]["response_time"]) + float(cfg["sim"]["dt"]),
            "a_max": float(act["a_max"]), "alpha_max": float(act["alpha_max"]),
            "speed_cap": box_budget_speed_cap(float(act["v_max"]), float(act["vy_max"]), limit),
            "yaw_cap": min(float(act["w_max"]), limit / lever)}


def narrow_floor(lateral_margin: float, max_cross_track: float, slack: float,
                 scan_penalty: float = 0.0) -> float:
    """Ruling 4: the narrow stratum's floor, m per side = lateral margin + peak cross-track
    + slack. A robot tracking its lane within the committed cross-track tolerance keeps its
    moving field's lateral margin clear of the wall, with `slack` to spare.

    `scan_penalty` is the margin the real scan eats before the geometry is consulted at all
    -- sector-min binning, range error and sensor parallax (`sector_min_penalty` computes the
    first of the three exactly). The slack exists to absorb it, so a slack below it is a
    derivation that does not hold and raises rather than returning a floor that looks fine.
    Supervisor 2026-09-26: the derivation must name this term rather than leave it inside an
    unexplained slack.
    """
    if slack < scan_penalty:
        raise ValueError(
            f"slack {slack} is below the scan penalty {scan_penalty} it exists to absorb: "
            "the narrow floor would be stated more tightly than the scan can support")
    return lateral_margin + max_cross_track + slack


def sector_min_range(bearings: ArrayLike, wall_distance: float, wall_normal: float,
                     bin_width: float) -> np.ndarray:
    """Range each bin reports for a straight wall, under nearest-point-in-sector binning, m.

    The scanner reports the NEAREST point inside each angular sector, not the range at its
    centre (`lua/scanner_v2.lua`), which is shorter than the bin-centre range and therefore
    the direction that trips a stop. For a wall at perpendicular distance `wall_distance`
    whose inward normal points along `wall_normal`, the range along bearing psi is
    d / cos(psi - wall_normal) where that cosine is positive, so the sector minimum takes
    the LARGEST cosine in the sector: 1 where the sector contains the normal itself, and
    otherwise the better of its two edges. `bearings` are bin centres, `bin_width` the
    sector width (2 pi / filter_bins). Bins facing away from the wall carry +inf.
    """
    b = np.asarray(bearings, dtype=float)
    half = float(bin_width) / 2.0
    lo, hi = wrap_to_pi(b - half - wall_normal), wrap_to_pi(b + half - wall_normal)
    # The sector contains the normal exactly when its two edges straddle 0 in that frame.
    contains = (lo <= 0.0) & (hi >= 0.0)
    best = np.where(contains, 1.0, np.maximum(np.cos(lo), np.cos(hi)))
    out = np.full(b.shape, np.inf)
    facing = best > 1e-12
    out[facing] = float(wall_distance) / best[facing]
    return out


def wrap_to_pi(a: ArrayLike) -> np.ndarray:
    """Angles wrapped into (-pi, pi]."""
    x = np.asarray(a, dtype=float)
    return -(np.remainder(-x + math.pi, 2.0 * math.pi) - math.pi)


def binned_radial_margin(fields: FieldGeometry, bearings: ArrayLike, wall_distance: float,
                         band_top_speed: float, heading: float, at_rest: bool,
                         bin_width: float) -> float:
    """Smallest (reported range - field reach) over the bins, for two parallel walls, m.

    This is what the certified layer actually evaluates: per bin, a sector-minimum range
    against the field's reach along that bin's centre bearing. Negative means a stop.
    """
    b = np.asarray(bearings, dtype=float)
    reach = fields.extent(b, band_top_speed, heading, at_rest)
    left = sector_min_range(b, wall_distance, heading + math.pi / 2.0, bin_width)
    right = sector_min_range(b, wall_distance, heading - math.pi / 2.0, bin_width)
    return float(np.min(np.minimum(left, right) - reach))


def sector_min_penalty(fields: FieldGeometry, bearings: ArrayLike, wall_distance: float,
                       band_top_speed: float, heading: float, at_rest: bool,
                       bin_width: float) -> float:
    """Margin lost to binning alone, m: the perpendicular margin less the binned one.

    The perpendicular margin is what the geometry predicts for a wall
    (`wall_distance - lateral_reach`); the binned margin is what the layer computes from a
    sector-minimum scan. The difference is the sector-min quantisation term the supervisor
    asked to see named (2026-09-26). It is only ONE of the three terms that eat margin --
    range error and sensor parallax are measured, not derived, and are recorded with the
    narrow floor -- so it is a lower bound on the total penalty, never the whole of it.
    """
    perpendicular = float(wall_distance) - fields.lateral_reach(
        band_top_speed, heading, at_rest)
    binned = binned_radial_margin(fields, bearings, wall_distance, band_top_speed,
                                  heading, at_rest, bin_width)
    return perpendicular - binned


def rest_offset_floor(fields: FieldGeometry, half_width: float, offset: float) -> float:
    """Per-side clearance an aisle needs so a robot AT REST, displaced by `offset`, is clear.

    The at-rest field is the footprint grown by `margin` all round, which is WIDER abeam
    than any moving field (margin 0.10 against lateral_margin 0.05), so this floor sits
    above the traversal floor. It matters because a latched stop holds while the field is
    breached: a robot that stops off-centre in an aisle below this floor cannot clear its
    own stop (supervisor 2026-09-26, the traversability precondition).
    """
    return fields.lateral_reach(0.0, 0.0, at_rest=True) - half_width + float(offset)


def max_offset_at_rest(fields: FieldGeometry, half_width: float, clearance: float) -> float:
    """How far off-centre a robot may come to rest in this aisle and still be clear, m.

    Below zero the aisle does not admit a stationary robot at all. Compared against the
    committed peak cross-track, this says whether a stop anywhere the controller is allowed
    to be can be cleared.
    """
    return half_width + float(clearance) - fields.lateral_reach(0.0, 0.0, at_rest=True)


def rest_clearance_floor(fields: FieldGeometry, half_width: float) -> float:
    """Per-side clearance below which a stationary, centred robot trips a stop, m."""
    return fields.lateral_reach(0.0, 0.0, at_rest=True) - half_width


def moving_field_fits(speed: float, fields: FieldGeometry, half_width: float,
                      clearance: float, offset: float = 0.0) -> bool:
    """Whether the field for a band topped at `speed`, travelling along the aisle, fits it.

    `offset` moves the robot towards one wall (e.g. a tracking error), leaving
    clearance - offset on the near side. The field is convex, so it clears a wall parallel
    to the travel direction exactly when its support towards that wall is within reach.
    """
    return fields.lateral_reach(speed, 0.0, at_rest=False) <= half_width + clearance - offset


def max_traversal_speed(fields: FieldGeometry, half_width: float, clearance: float,
                        offset: float = 0.0) -> float:
    """Highest band-top speed whose field fits the aisle, m/s (0 if even v -> 0 does not).

    Found by bisection over the band-top speed, capped at the top band's top. With the
    Minkowski field the sideways reach does not grow with speed, so this is either 0 or
    the cap for an aisle aligned with travel; the search stays general so that a change
    of field shape cannot silently turn it into a constant.
    """
    hi = fields.params.band_top_speeds[-1]
    fits = lambda v: moving_field_fits(v, fields, half_width, clearance, offset)
    if not fits(0.0):
        return 0.0
    if fits(hi):
        return hi
    lo = 0.0
    while hi - lo > SPEED_TOL:
        mid = 0.5 * (lo + hi)
        if fits(mid):
            lo = mid
        else:
            hi = mid
    return lo


def usable_band_top(speed: float, params: CertifiedParams) -> float | None:
    """The highest band top at or below `speed`, m/s; None if no band fits at all."""
    usable = [top for top in params.band_top_speeds if top <= speed + 1e-12]
    return max(usable) if usable else None
