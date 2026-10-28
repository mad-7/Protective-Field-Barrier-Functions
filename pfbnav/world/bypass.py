"""Can the robot get past its unmapped obstacles without leaving its route's corridor? Pure.

Ruled by the supervisor on 2026-10-02, revising A13's feasibility test. "The goal remains
reachable in the true world" was the wrong test for a stack that plans once, on the MAPPED map,
and never replans: the dry run of 2026-10-02 found 29 % of pilot scenarios (at +-1.5 m, and at
only the lethal radius) whose obstacles could be passed only through another aisle. Every
controller would have stalled or taken a stop it could not avoid.

An unmapped obstacle is now kept only if, with every unmapped obstacle placed so far JOINTLY
(two individually passable obstacles can close a gap together):

  * **a bypass exists** from start to goal for the robot's centre, keeping at least
    `robot.half_width + narrow floor` from every surface, mapped or unmapped. The gap past an
    obstacle is an aisle and meets the same admissibility test as any aisle: the narrow
    stratum's per-side floor beyond the footprint, READ from the strata (0.1987 m under A21;
    the 0.13 m once written here was the WP1 floor). The lethal radius (0.03 m beside the
    robot) would put the obstacle inside the 0.05 m abeam protective field: a stop no
    controller could avoid, exactly the geometry-dominated case H1 must exclude;
  * **within the corridor**: no point of the bypass further than `W` from the planned route.
    It is local avoidance within the corridor the route uses, never a detour through another
    aisle.

**How it is decided, and its one approximation.** A grid of cell centres at `resolution` over
the corridor; a cell is admissible if its EXACT distance to every polygon is at least
`radius - resolution / 2`; the bypass exists if the start's and goal's cells share an
8-connected component. The tolerance is what lets a gap of exactly the floor through at any
alignment of the grid (a band of perpendicular width `resolution` always holds an 8-connected
digital line). Its cost: a gap up to `resolution / 2` per side narrower than the floor MAY be
admitted -- 5 mm at 0.01 m, out of the floor's deliberate 0.03 m buffer. Both bounds are tested
by a dense sweep (tests/test_bypass.py). The corridor edge is placed by a distance transform
and is good to one cell; that only moves W by a centimetre.

**Positive control** (hard rule 15): the same test on the mapped world alone, before any
obstacle is placed, must pass. A route whose own corridor fails it is not a scenario the test
can judge, and is counted, never silently used.
"""
from __future__ import annotations

import itertools
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import ArrayLike
from scipy import ndimage

from pfbnav.world.baking import polygon_distance

_EIGHT = np.ones((3, 3), dtype=bool)


@dataclass(frozen=True)
class BypassRule:
    """All lengths in m."""

    radius: float        # the robot centre's distance to any surface: half_width + narrow floor
    corridor: float      # W: the bypass stays within this distance of the planned route
    resolution: float    # the check grid

    def __post_init__(self) -> None:
        if not 0.0 < self.resolution < self.radius:
            raise ValueError("need 0 < resolution < radius")
        if self.corridor <= 0.0:
            raise ValueError("the corridor half-width W must be positive")

    @property
    def tolerance(self) -> float:
        """How much narrower than `radius` an admissible cell centre may be: half a cell."""
        return 0.5 * self.resolution

    @classmethod
    def from_config(cls, cfg: dict[str, Any], corridor: float | None = None) -> BypassRule:
        """The radius is READ from the robot and the narrow stratum, never restated."""
        narrow = next(s for s in cfg["world"]["aisle_strata"] if s["name"] == "narrow")
        b = cfg["world"]["layout"]["unmapped"]["route_relative"]["bypass"]
        return cls(radius=float(cfg["robot"]["half_width"]) + float(narrow["min_clearance"]),
                   corridor=float(b["corridor_half_width"] if corridor is None else corridor),
                   resolution=float(b["resolution"]))


def _densify(path: np.ndarray, step: float) -> np.ndarray:
    out = [path[:1]]
    for a, b in itertools.pairwise(path):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out.append(a + np.linspace(0.0, 1.0, n + 1)[1:, None] * (b - a))
    return np.vstack(out)


class Corridor:
    """The check grid for ONE route, built once; `passable` is then asked per candidate."""

    def __init__(self, route: ArrayLike, mapped: Sequence[ArrayLike], start_xy: ArrayLike,
                 goal_xy: ArrayLike, rule: BypassRule) -> None:
        self.rule = rule
        r = rule.resolution
        path = np.asarray(route, dtype=float)
        lo = path.min(axis=0) - rule.corridor - 2 * r
        hi = path.max(axis=0) + rule.corridor + 2 * r
        self.origin = lo
        cols = int(np.ceil((hi[0] - lo[0]) / r))
        rows = int(np.ceil((hi[1] - lo[1]) / r))
        self.shape = (rows, cols)
        # Distance of every cell to the route, by a distance transform of the route's cells.
        on_route = np.zeros(self.shape, dtype=bool)
        rr, cc = self._cells(_densify(path, 0.5 * r))
        on_route[rr, cc] = True
        to_route = np.asarray(ndimage.distance_transform_edt(~on_route, sampling=r))
        self.inside = to_route <= rule.corridor
        self.rows, self.cols = np.nonzero(self.inside)
        self.points = np.stack([lo[0] + (self.cols + 0.5) * r, lo[1] + (self.rows + 0.5) * r],
                               axis=1)
        self.mapped_clearance = np.full(len(self.points), np.inf)
        for poly in mapped:
            self.mapped_clearance = np.minimum(self.mapped_clearance,
                                               polygon_distance(self.points, poly))
        self.start = self._neighbourhood(start_xy)
        self.goal = self._neighbourhood(goal_xy)

    def _cells(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        r = self.rule.resolution
        c = np.clip(np.floor((xy[:, 0] - self.origin[0]) / r).astype(int), 0, self.shape[1] - 1)
        q = np.clip(np.floor((xy[:, 1] - self.origin[1]) / r).astype(int), 0, self.shape[0] - 1)
        return q, c

    def _neighbourhood(self, xy: ArrayLike) -> tuple[slice, slice]:
        """The 3 x 3 cells around a point: the robot stands at the point, not on a cell centre."""
        q, c = self._cells(np.asarray(xy, dtype=float)[None, :2])
        return slice(max(0, q[0] - 1), q[0] + 2), slice(max(0, c[0] - 1), c[0] + 2)

    def clearance(self, unmapped: Sequence[ArrayLike]) -> np.ndarray:
        """Distance from each corridor cell centre to the nearest surface, m.

        Exact wherever it is below `radius + resolution`, the only range admissibility depends
        on. Further from an unmapped polygon than that, the polygon is not measured, so the value
        there may over-state the distance -- never enough to change a verdict.
        """
        d = self.mapped_clearance.copy()
        reach = self.rule.radius + self.rule.resolution
        for poly in unmapped:
            p = np.asarray(poly, dtype=float)
            # Only cells that this polygon could bring below the threshold need computing.
            near = np.all((self.points > p.min(axis=0) - reach)
                          & (self.points < p.max(axis=0) + reach), axis=1)
            d[near] = np.minimum(d[near], polygon_distance(self.points[near], p))
        return d

    def passable(self, unmapped: Sequence[ArrayLike]) -> bool:
        """Whether a bypass exists, with ALL of `unmapped` in place at once."""
        ok = np.zeros(self.shape, dtype=bool)
        admissible = self.clearance(unmapped) >= self.rule.radius - self.rule.tolerance
        ok[self.rows[admissible], self.cols[admissible]] = True
        labels = np.asarray(ndimage.label(ok, structure=_EIGHT)[0])
        at_start = set(np.unique(labels[self.start])) - {0}
        at_goal = set(np.unique(labels[self.goal])) - {0}
        return bool(at_start & at_goal)
