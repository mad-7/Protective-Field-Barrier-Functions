"""Route-relative placement of unmapped obstacles (amendment A13). Pure; no simulator.

Ruled by the supervisor on 2026-09-29, replacing uniform placement over the hall's free floor,
which put an unmapped obstacle within 0.3 m of the route in only 8-15 % of `unmapped_static`
scenarios (pilot seeds): H1's whole population would mostly not have tested what its condition
is named for. Now, once a scenario's route is planned, each unmapped obstacle is placed
relative to it, in every split (train, tune and test), so training and testing see the same
distribution:

  * **offset** to the obstacle's nearest SURFACE, never its centre, uniform on
    `[0, offset_max]`; the first obstacle may be capped lower (`first_offset_max`);
  * **side** uniform left or right of the route;
  * **position** uniform over the route's arc length, excluding the first `arc_from_start` and
    the last `arc_to_goal` metres;
  * **feasibility**: kept only if the robot can still get past every unmapped obstacle placed
    so far, jointly, without leaving a corridor of half-width W around its route and keeping
    the narrow stratum's floor beyond its footprint (revised 2026-10-02, `bypass.py`; it was
    "the goal reachable in the true world", which a stack that never replans cannot use). It
    is decided by a callback, so this module never plans. A rejected obstacle is re-sampled;
    the layout never is.

The obstacle is a square turned to face the route: its near face is parallel to the local
tangent at the sampled offset. The offset RECORDED is the one measured afterwards, exactly,
from the polygon to the whole route (`surface_offset`); it can be smaller than the sampled one
where the route bends back toward the obstacle.
"""
from __future__ import annotations

import itertools
import math
from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import ArrayLike

from pfbnav.planning.lookahead import arc_lengths, point_at, project
from pfbnav.world.baking import Obstacle, polygon_distance

# Rejection reasons, counted so the rejection rate is reported by cause, not as one number.
OUTSIDE_HALL = "outside_hall"
TOO_CLOSE_TO_MAPPED = "too_close_to_mapped"
OVERLAPS_UNMAPPED = "overlaps_other_unmapped"
NEAR_GOAL = "near_goal"
NO_BYPASS = "no_bypass_within_corridor"


@dataclass(frozen=True)
class PlacementRule:
    """The ruled distribution and its constraints. All lengths in m."""

    offset_max: float            # offsets to the nearest surface are uniform on [0, this]
    first_offset_max: float      # the first obstacle's cap; <= offset_max
    arc_from_start: float        # route arc excluded at the start
    arc_to_goal: float           # route arc excluded at the goal
    min_goal_distance: float     # preregistered: the obstacle's centre this far from the goal
    margin: float                # preregistered: kept clear of every mapped obstacle
    size: tuple[float, float]    # square side, uniform in this range
    attempts: int                # re-samples allowed per obstacle before giving up

    def __post_init__(self) -> None:
        if not 0.0 < self.offset_max:
            raise ValueError("offset_max must be positive")
        if not 0.0 < self.first_offset_max <= self.offset_max:
            raise ValueError("need 0 < first_offset_max <= offset_max")
        lo, hi = self.size
        if not 0.0 < lo <= hi:
            raise ValueError(f"size range {self.size} is not a positive interval")
        if self.attempts < 1:
            raise ValueError("attempts must be at least 1")

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> PlacementRule:
        u = cfg["world"]["layout"]["unmapped"]
        r = u["route_relative"]
        lo, hi = (float(v) for v in u["size"])
        return cls(offset_max=float(r["offset_max"]),
                   first_offset_max=float(r["first_offset_max"]),
                   arc_from_start=float(u["min_arc_from_start"]),
                   arc_to_goal=float(r["arc_to_goal"]),
                   min_goal_distance=float(u["min_distance_to_goal"]),
                   margin=float(u["margin"]), size=(lo, hi),
                   attempts=int(r["attempts_per_obstacle"]))


@dataclass
class Placement:
    """What was placed, where, and what it took."""

    obstacles: list[Obstacle] = field(default_factory=list)
    offsets_m: list[float] = field(default_factory=list)      # measured, to the nearest surface
    arcs_m: list[float] = field(default_factory=list)
    attempts: int = 0
    rejections: Counter = field(default_factory=Counter)
    complete: bool = False


# ---------------------------------------------------------------- convex geometry
def _axes(poly: np.ndarray) -> np.ndarray:
    edges = np.roll(poly, -1, axis=0) - poly
    normals = np.column_stack([-edges[:, 1], edges[:, 0]])
    lengths = np.linalg.norm(normals, axis=1)
    return normals[lengths > 1e-12] / lengths[lengths > 1e-12, None]


def convex_overlap(a: ArrayLike, b: ArrayLike) -> bool:
    """Whether two convex polygons (or a polygon and a segment) share any point.

    Separating-axis test. Touching counts as overlap, the conservative direction.
    """
    pa, pb = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    for axis in np.vstack([_axes(pa), _axes(pb)]):
        ra, rb = pa @ axis, pb @ axis
        if ra.max() < rb.min() or rb.max() < ra.min():
            return False
    return True


def convex_clearance(a: ArrayLike, b: ArrayLike) -> float:
    """Distance between two convex polygons, m; 0 if they overlap.

    For disjoint convex polygons the minimum is attained between a vertex and an edge, so the
    smaller of the two vertex-to-polygon distances is exact.
    """
    pa, pb = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    if convex_overlap(pa, pb):
        return 0.0
    return float(min(polygon_distance(pa, pb).min(), polygon_distance(pb, pa).min()))


def surface_offset(polygon: ArrayLike, path: ArrayLike) -> float:
    """Distance from the route to the obstacle's nearest SURFACE, m; 0 if the route crosses it.

    Exact for a convex polygon and a polyline: zero if any route segment meets the polygon,
    otherwise the smaller of the polygon's vertices to the route and the route's vertices to
    the polygon.
    """
    poly = np.asarray(polygon, dtype=float)
    p = np.asarray(path, dtype=float)
    for a, b in itertools.pairwise(p):
        if convex_overlap(poly, np.array([a, b])):
            return 0.0
    to_route = min(float(project(p, v).distance) for v in poly)
    return float(min(to_route, polygon_distance(p, poly).min()))


def tangent_at(path: ArrayLike, s: float, h: float = 1e-3) -> np.ndarray:
    """Unit tangent of the route at arc length `s`, by a central difference."""
    d = point_at(path, s + h) - point_at(path, s - h)
    n = float(np.linalg.norm(d))
    if n < 1e-12:
        raise ValueError(f"the route has no direction at s = {s}")
    return d / n


def square_beside(path: ArrayLike, s: float, offset: float, side: int,
                  size: float) -> np.ndarray:
    """A square whose near face is parallel to the route at `s`, `offset` from it, on `side`.

    `side` is +1 for the left of the direction of travel, -1 for the right. Counter-clockwise.
    """
    if side not in (1, -1):
        raise ValueError("side must be +1 (left) or -1 (right)")
    t = tangent_at(path, s)
    n = np.array([-t[1], t[0]]) * side
    c = point_at(path, s) + n * (offset + size / 2.0)
    h = size / 2.0
    corners = [c - h * t - h * n, c + h * t - h * n, c + h * t + h * n, c - h * t + h * n]
    poly = np.array(corners)
    # Counter-clockwise regardless of side.
    area = 0.5 * float(np.sum(poly[:, 0] * np.roll(poly[:, 1], -1)
                              - np.roll(poly[:, 0], -1) * poly[:, 1]))
    return poly if area > 0 else poly[::-1].copy()


# ---------------------------------------------------------------- placement
def place_on_route(rng, path: ArrayLike, goal_xy: ArrayLike, mapped: list[Obstacle],
                   hall: tuple[float, float, float, float], count: int, rule: PlacementRule,
                   feasible: Callable[[list[Obstacle]], bool]) -> Placement:
    """Place `count` unmapped obstacles along the route, one at a time, re-sampling each.

    Every draw is made whether or not the sample is later rejected, so the random stream does
    not branch on geometry more than it has to. `feasible(obstacles)` must say whether the robot
    can get past ALL of those unmapped obstacles at once (`Corridor.passable`). Returns a
    Placement with `complete` False if some obstacle could not be placed within `rule.attempts`.
    """
    out = Placement()
    if count <= 0:
        out.complete = True
        return out
    p = np.asarray(path, dtype=float)
    length = float(arc_lengths(p)[-1])
    s_lo, s_hi = rule.arc_from_start, length - rule.arc_to_goal
    if s_hi <= s_lo:
        return out                       # the route is too short to hold any obstacle
    goal = np.asarray(goal_xy, dtype=float)
    size_lo, size_hi = rule.size
    for k in range(count):
        cap = rule.first_offset_max if k == 0 else rule.offset_max
        placed = False
        for _ in range(rule.attempts):
            out.attempts += 1
            s = float(rng.uniform(s_lo, s_hi))
            offset = float(rng.uniform(0.0, cap))
            side = 1 if float(rng.random()) < 0.5 else -1
            size = float(rng.uniform(size_lo, size_hi))
            poly = square_beside(p, s, offset, side, size)
            if (poly[:, 0].min() < hall[0] or poly[:, 1].min() < hall[1]
                    or poly[:, 0].max() > hall[2] or poly[:, 1].max() > hall[3]):
                out.rejections[OUTSIDE_HALL] += 1
                continue
            if any(convex_clearance(poly, m.polygon) < rule.margin for m in mapped):
                out.rejections[TOO_CLOSE_TO_MAPPED] += 1
                continue
            if any(convex_overlap(poly, o.polygon) for o in out.obstacles):
                out.rejections[OVERLAPS_UNMAPPED] += 1
                continue
            if float(np.linalg.norm(poly.mean(axis=0) - goal)) < rule.min_goal_distance:
                out.rejections[NEAR_GOAL] += 1
                continue
            candidate = Obstacle(poly, mapped=False, name=f"unmapped_{k}")
            if not feasible(out.obstacles + [candidate]):
                out.rejections[NO_BYPASS] += 1
                continue
            out.obstacles.append(candidate)
            out.offsets_m.append(surface_offset(poly, p))
            out.arcs_m.append(float(project(p, poly.mean(axis=0)).s))
            placed = True
            break
        if not placed:
            return out
    out.complete = True
    return out


def expected_engaged_share(first_offset_max: float, offset_max: float, threshold: float,
                           count_range: tuple[int, int]) -> float:
    """Share of scenarios with an obstacle within `threshold` of the route, if nothing is
    rejected and offsets are exactly as sampled. Counts uniform over `count_range`, inclusive.

    The ruled rule (all offsets uniform on [0, 0.6] m, 1-2 obstacles, 0.3 m threshold) gives
    1 - (0.5 + 0.25) / 2 = 0.625: short of the ruled 90 % validity check by construction.
    """
    p_first = min(1.0, threshold / first_offset_max)
    p_other = min(1.0, threshold / offset_max)
    lo, hi = count_range
    shares = [1.0 - (1.0 - p_first) * (1.0 - p_other) ** (n - 1) for n in range(lo, hi + 1)]
    return float(np.mean(shares)) if shares else math.nan
