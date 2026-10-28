"""8-connected A* over a baked map, with line-of-sight simplification. Pure; no simulator.

The global planner. It searches cell centres, never the robot's footprint: D1 makes the
global plan deliberately not footprint-complete, with the lethal tier keeping the centre
where no heading can collide and the local planner and the filters handling the rest.

**Failure is a status, not an exception and not a fallback.** v1 wrapped the search in
`try/except` and returned a straight line when it failed, so an unreachable goal looked
like a plan through a shelf. Here every outcome the caller must distinguish has a name
(`PlanStatus`), the path is empty unless the status is `ok`, and nothing is invented
(hard rule 9).

**Cost.** Each move costs its length in metres, scaled by the soft tier at the cell it
enters: `length * (1 + soft_weight * cost)`. Lethal cells are not entered at all. With
`planner.soft_weight` at 1, a cell at the lethal boundary costs twice a free one, so the
planner will accept a detour of up to twice the distance to leave the band -- the
"discourages, never forbids" D1 asks for, with a number attached.

**The heuristic** is the octile distance times the cheapest possible cost factor (1), which
never over-states the remaining cost, so the search is admissible and the first path found
is optimal on the grid.

**Simplification** pulls the string: a waypoint is dropped when the straight segment that
would replace it stays clear of lethal cells, tested over every cell the segment touches
(`line_cells`, a supercover traversal -- not Bresenham, which skips cells a line clips and
would let a path shave a lethal corner).

Frames: world metres for the interface, (row, col) internally, as `pfbnav/world/baking.py`
defines them.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from pfbnav.world.baking import BakedMap

# Outcomes a caller must be able to tell apart. Strings, so they travel into JSON reports.
OK = "ok"
START_OUTSIDE = "start_outside_map"
GOAL_OUTSIDE = "goal_outside_map"
START_LETHAL = "start_in_lethal"
GOAL_LETHAL = "goal_in_lethal"
UNREACHABLE = "unreachable"
EXPANSION_LIMIT = "expansion_limit"
STATUSES = (OK, START_OUTSIDE, GOAL_OUTSIDE, START_LETHAL, GOAL_LETHAL, UNREACHABLE,
            EXPANSION_LIMIT)

# (drow, dcol) and their lengths in cells: 8-connected, diagonals at sqrt(2).
NEIGHBOURS: tuple[tuple[int, int, float], ...] = (
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
)


@dataclass
class PlanResult:
    """A plan, or a named reason there is none. `path_world` is empty unless status is ok."""

    status: str
    path_world: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    path_cells: list[tuple[int, int]] = field(default_factory=list)
    cost: float = math.inf
    expansions: int = 0
    waypoints_before_simplification: int = 0

    @property
    def ok(self) -> bool:
        return self.status == OK

    @property
    def length_m(self) -> float:
        """Arc length of the returned polyline, m (0 when there is no path)."""
        if len(self.path_world) < 2:
            return 0.0
        return float(np.linalg.norm(np.diff(self.path_world, axis=0), axis=1).sum())


def octile(a: tuple[int, int], b: tuple[int, int]) -> float:
    """Cheapest 8-connected distance in cells between two cells, ignoring obstacles."""
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    lo, hi = min(dr, dc), max(dr, dc)
    return (math.sqrt(2.0) - 1.0) * lo + hi


def line_cells(start: tuple[float, float], end: tuple[float, float]) -> list[tuple[int, int]]:
    """Every cell a segment touches, in continuous (row, col) coordinates (a supercover).

    Bresenham visits one cell per step and so skips cells a line merely clips; a path
    simplified on Bresenham can therefore shave the corner of a lethal cell. This walks the
    grid lines instead and includes both cells at every crossing.
    """
    r0, c0 = float(start[0]), float(start[1])
    r1, c1 = float(end[0]), float(end[1])
    cell = [math.floor(r0), math.floor(c0)]
    last = (math.floor(r1), math.floor(c1))
    dr, dc = r1 - r0, c1 - c0
    step_r = 0 if dr == 0 else (1 if dr > 0 else -1)
    step_c = 0 if dc == 0 else (1 if dc > 0 else -1)
    # Parameter t in [0, 1] at which the next grid line in each axis is crossed.
    def first_crossing(value: float, step: int, delta: float) -> float:
        if step == 0:
            return math.inf
        boundary = math.floor(value) + (1 if step > 0 else 0)
        return (boundary - value) / delta
    t_r = first_crossing(r0, step_r, dr)
    t_c = first_crossing(c0, step_c, dc)
    d_r = math.inf if step_r == 0 else abs(1.0 / dr)
    d_c = math.inf if step_c == 0 else abs(1.0 / dc)
    out = [(cell[0], cell[1])]
    guard = 4 * (abs(last[0] - cell[0]) + abs(last[1] - cell[1])) + 8
    while (cell[0], cell[1]) != last and guard > 0:
        guard -= 1
        if t_r < t_c:
            cell[0] += step_r
            t_r += d_r
        elif t_c < t_r:
            cell[1] += step_c
            t_c += d_c
        else:                      # exactly through a corner: both cells it touches
            out.append((cell[0] + step_r, cell[1]))
            out.append((cell[0], cell[1] + step_c))
            cell[0] += step_r
            cell[1] += step_c
            t_r += d_r
            t_c += d_c
        out.append((cell[0], cell[1]))
    return out


def segment_is_clear(baked: BakedMap, a: tuple[int, int], b: tuple[int, int]) -> bool:
    """Whether the straight segment between two cell centres stays out of lethal cells."""
    rows, cols = baked.shape
    for r, c in line_cells((a[0] + 0.5, a[1] + 0.5), (b[0] + 0.5, b[1] + 0.5)):
        if not (0 <= r < rows and 0 <= c < cols):
            return False
        if baked.lethal[r, c]:
            return False
    return True


def simplify(baked: BakedMap, cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Drop every waypoint whose removal leaves a segment clear of lethal cells."""
    if len(cells) < 3:
        return list(cells)
    out = [cells[0]]
    anchor = 0
    for i in range(1, len(cells)):
        if not segment_is_clear(baked, cells[anchor], cells[i]):
            out.append(cells[i - 1])
            anchor = i - 1
    out.append(cells[-1])
    return out


def plan(baked: BakedMap, start_xy, goal_xy, *, soft_weight: float,
         max_expansions: int | None = None, simplify_path: bool = True) -> PlanResult:
    """Plan from `start_xy` to `goal_xy` in world metres. Never raises for a bad request."""
    if soft_weight < 0.0:
        raise ValueError("soft_weight must be non-negative")
    try:
        srow, scol = (int(v[0]) for v in baked.cell_of([start_xy[0]], [start_xy[1]]))
    except ValueError:
        return PlanResult(status=START_OUTSIDE)
    try:
        grow, gcol = (int(v[0]) for v in baked.cell_of([goal_xy[0]], [goal_xy[1]]))
    except ValueError:
        return PlanResult(status=GOAL_OUTSIDE)
    if baked.lethal[srow, scol]:
        return PlanResult(status=START_LETHAL)
    if baked.lethal[grow, gcol]:
        return PlanResult(status=GOAL_LETHAL)

    start, goal = (srow, scol), (grow, gcol)
    res = baked.resolution
    cost = baked.cost
    lethal = baked.lethal
    rows, cols = baked.shape
    g = {start: 0.0}
    came: dict[tuple[int, int], tuple[int, int]] = {}
    closed: set[tuple[int, int]] = set()
    # The counter keeps ties in insertion order, so one map and one request always give one
    # plan: determinism within a process is claimed, and a heap without it would not.
    counter = 0
    heap: list[tuple[float, int, tuple[int, int]]] = [(octile(start, goal) * res, 0, start)]
    expansions = 0

    while heap:
        _, _, current = heapq.heappop(heap)
        if current in closed:
            continue
        closed.add(current)
        expansions += 1
        if current == goal:
            break
        if max_expansions is not None and expansions >= max_expansions:
            return PlanResult(status=EXPANSION_LIMIT, expansions=expansions)
        r, c = current
        for dr, dc, step in NEIGHBOURS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols) or lethal[nr, nc]:
                continue
            if dr != 0 and dc != 0 and (lethal[r, nc] or lethal[nr, c]):
                continue          # no squeezing diagonally between two lethal cells
            tentative = g[current] + step * res * (1.0 + soft_weight * float(cost[nr, nc]))
            if tentative < g.get((nr, nc), math.inf) - 1e-12:
                g[(nr, nc)] = tentative
                came[(nr, nc)] = current
                counter += 1
                heapq.heappush(
                    heap, (tentative + octile((nr, nc), goal) * res, counter, (nr, nc)))

    if goal not in closed:
        return PlanResult(status=UNREACHABLE, expansions=expansions)

    cells = [goal]
    while cells[-1] != start:
        cells.append(came[cells[-1]])
    cells.reverse()
    before = len(cells)
    if simplify_path:
        cells = simplify(baked, cells)
    path = np.array([baked.world_of(r, c) for r, c in cells], dtype=float)
    return PlanResult(status=OK, path_world=path, path_cells=cells, cost=g[goal],
                      expansions=expansions, waypoints_before_simplification=before)


def plan_from_config(baked: BakedMap, start_xy, goal_xy, cfg: dict[str, Any],
                     **kwargs) -> PlanResult:
    """`plan` with the search's parameters taken from the configuration."""
    return plan(baked, start_xy, goal_xy, soft_weight=float(cfg["planner"]["soft_weight"]),
                **kwargs)
