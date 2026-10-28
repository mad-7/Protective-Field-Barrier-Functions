"""Bake a layout into the grid the global planner searches. Pure functions, no simulator.

The planner's map is built here, from the layout's obstacle polygons rather than from the
scene: what the planner knows is a deliberate choice, and reading it back from the
simulator would make "what the robot was told" and "what is there" the same thing by
construction.

**Exact footprints, not bounding boxes.** v1 marked each obstacle's axis-aligned bounding
box. A shelf at 30 degrees then blocked cells no shelf occupies, which closes aisles the
robot can drive and quietly changes the layout the experiment is about. A cell here is
occupied when the obstacle polygon actually meets the cell's square, by the separating-axis
test (`cell_overlap`).

**Unmapped obstacles are excluded, and recorded.** A layout may carry obstacles the planner
is not told about -- the case the local planner and the safety layers exist for. They are
dropped from every layer of the bake and listed in the result, so a run can state how many
there were rather than leaving the omission invisible.

**Two tiers, from D1** (`docs/decision_log.md`): the global planner is intentionally not
footprint-complete.

  * **lethal** -- cells whose centre is within `inscribed + lethal_margin` of an obstacle.
    The inscribed radius is `robot.half_width`: no heading lets the robot's centre sit
    there. The planner must never route through these.
  * **soft** -- a cost decaying from 1 at the lethal boundary to 0 at
    `circumscribed + soft_margin`, where the circumscribed radius is the measured footprint
    table's largest extent. It discourages, never forbids.

Radii are computed from the configuration, never stored: a copied number goes stale the
next time the envelope is measured (D1).

Frames: world metres throughout; grid indices are (row, col) with row along +y and column
along +x, so `grid[r, c]` is the cell whose centre is `origin + (c + 0.5, r + 0.5) * res`.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import ArrayLike


@dataclass(frozen=True)
class Obstacle:
    """One obstacle as a convex polygon in world metres, counter-clockwise.

    `mapped` False means the planner is not told about it: it is excluded from the bake and
    counted in the result. `name` travels into the bake's provenance.
    """

    polygon: np.ndarray
    mapped: bool = True
    name: str = ""

    def __post_init__(self) -> None:
        p = np.asarray(self.polygon, dtype=float)
        if p.ndim != 2 or p.shape[1] != 2 or len(p) < 3:
            raise ValueError(f"a polygon is (n >= 3, 2) points, got shape {p.shape}")
        object.__setattr__(self, "polygon", p)


@dataclass
class BakedMap:
    """The planner's view of a layout."""

    origin: tuple[float, float]      # world coordinates of the grid's (0, 0) corner, m
    resolution: float                # m per cell
    occupied: np.ndarray             # bool, cells an obstacle actually meets
    lethal: np.ndarray               # bool, cells the robot's centre may never occupy
    cost: np.ndarray                 # float in [0, 1], the soft tier; 1 on lethal cells
    clearance_m: np.ndarray          # distance from each cell centre to the nearest mapped
                                     #   obstacle, m (inf where there are none)
    radii: dict[str, float]          # inscribed, circumscribed, lethal, soft: all derived
    unmapped: list[str] = field(default_factory=list)

    @property
    def shape(self) -> tuple[int, int]:
        return self.occupied.shape

    def world_of(self, row: ArrayLike, col: ArrayLike) -> np.ndarray:
        """Cell centres in world metres, (..., 2)."""
        r = np.asarray(row, dtype=float)
        c = np.asarray(col, dtype=float)
        return np.stack([self.origin[0] + (c + 0.5) * self.resolution,
                         self.origin[1] + (r + 0.5) * self.resolution], axis=-1)

    def cell_of(self, x: ArrayLike, y: ArrayLike) -> tuple[np.ndarray, np.ndarray]:
        """The (row, col) containing each world point. Points outside the grid are an error."""
        col = np.floor((np.asarray(x, dtype=float) - self.origin[0]) / self.resolution)
        row = np.floor((np.asarray(y, dtype=float) - self.origin[1]) / self.resolution)
        rows, cols = self.shape
        if np.any(col < 0) or np.any(col >= cols) or np.any(row < 0) or np.any(row >= rows):
            raise ValueError("a point lies outside the baked map")
        return row.astype(int), col.astype(int)

    def free_for_centre(self, x: ArrayLike, y: ArrayLike) -> np.ndarray:
        """Whether the robot's centre may occupy each world point (not lethal)."""
        row, col = self.cell_of(x, y)
        return ~self.lethal[row, col]


# ---------------------------------------------------------------------------------------
# Geometry (pure)
# ---------------------------------------------------------------------------------------
def polygon_distance(points: ArrayLike, polygon: ArrayLike) -> np.ndarray:
    """Distance from each point to a convex polygon, m; 0 inside it.

    Exact: the minimum distance to the polygon's edge segments, and zero wherever the point
    lies inside. Vectorised over points.
    """
    p = np.atleast_2d(np.asarray(points, dtype=float))
    poly = np.asarray(polygon, dtype=float)
    a = poly
    b = np.roll(poly, -1, axis=0)
    ab = b - a                                              # (m, 2)
    ap = p[:, None, :] - a[None, :, :]                      # (n, m, 2)
    denom = np.einsum("ij,ij->i", ab, ab)
    denom = np.where(denom > 0.0, denom, 1.0)
    t = np.clip(np.einsum("nmj,mj->nm", ap, ab) / denom, 0.0, 1.0)
    closest = a[None, :, :] + t[:, :, None] * ab[None, :, :]
    edge = np.linalg.norm(p[:, None, :] - closest, axis=2).min(axis=1)
    return np.where(inside_convex(p, poly), 0.0, edge)


def inside_convex(points: ArrayLike, polygon: ArrayLike) -> np.ndarray:
    """Whether each point is inside a convex polygon (boundary counts as inside)."""
    p = np.atleast_2d(np.asarray(points, dtype=float))
    poly = np.asarray(polygon, dtype=float)
    a = poly
    b = np.roll(poly, -1, axis=0)
    edge = b - a
    rel = p[:, None, :] - a[None, :, :]
    cross = edge[None, :, 0] * rel[:, :, 1] - edge[None, :, 1] * rel[:, :, 0]
    return np.all(cross >= -1e-12, axis=1) | np.all(cross <= 1e-12, axis=1)


def cell_overlap(polygon: ArrayLike, centres: ArrayLike, resolution: float) -> np.ndarray:
    """Whether a convex polygon meets each axis-aligned cell square. Exact (SAT).

    Two convex shapes are disjoint exactly when some axis separates them; for a polygon
    against an axis-aligned square the candidate axes are x, y and each edge normal. This
    is what "exact footprints" means: a rotated shelf marks the cells it covers and no
    others, where a bounding box would mark the corners of its box as well.
    """
    poly = np.asarray(polygon, dtype=float)
    c = np.atleast_2d(np.asarray(centres, dtype=float))
    half = 0.5 * float(resolution)
    axes = [np.array([1.0, 0.0]), np.array([0.0, 1.0])]
    edges = np.roll(poly, -1, axis=0) - poly
    for e in edges:
        norm = math.hypot(e[0], e[1])
        if norm > 0.0:
            axes.append(np.array([-e[1] / norm, e[0] / norm]))
    separated = np.zeros(len(c), dtype=bool)
    for axis in axes:
        proj_poly = poly @ axis
        lo_p, hi_p = float(proj_poly.min()), float(proj_poly.max())
        proj_c = c @ axis
        radius = half * (abs(float(axis[0])) + abs(float(axis[1])))   # square's support
        # `<=`: a cell that only shares an edge with the obstacle is NOT occupied. Touching
        # counted as overlap would inflate every obstacle by a ring of cells, which is the
        # bounding-box error in another form; clearance and the lethal tier carry safety.
        separated |= (proj_c + radius <= lo_p + 1e-12) | (proj_c - radius >= hi_p - 1e-12)
    return ~separated


def soft_cost(clearance: ArrayLike, lethal_radius: float, soft_radius: float) -> np.ndarray:
    """The soft tier: 1 at or inside the lethal radius, falling linearly to 0 at `soft`.

    Linear because nothing measured says otherwise; it discourages the band between the
    inscribed and circumscribed radii and forbids nothing (D1).
    """
    if soft_radius <= lethal_radius:
        raise ValueError("the soft radius must exceed the lethal radius")
    d = np.asarray(clearance, dtype=float)
    return np.clip((soft_radius - d) / (soft_radius - lethal_radius), 0.0, 1.0)


# ---------------------------------------------------------------------------------------
def planner_radii(cfg: dict[str, Any], circumscribed: float) -> dict[str, float]:
    """The four radii D1 defines, derived from the configuration and the measured footprint.

    `circumscribed` is the footprint table's largest extent, passed in so this module never
    reads a file. Nothing here is stored in the config: radii are margins plus measurements.
    """
    inscribed = float(cfg["robot"]["half_width"])
    lethal_margin = float(cfg["planner"]["lethal_margin"])
    soft_margin = float(cfg["planner"]["soft_margin"])
    return {"inscribed": inscribed, "circumscribed": float(circumscribed),
            "lethal": inscribed + lethal_margin,
            "soft": float(circumscribed) + soft_margin}


def bake(obstacles: Sequence[Obstacle], bounds: tuple[float, float, float, float],
         resolution: float, radii: dict[str, float]) -> BakedMap:
    """Rasterise a layout into the planner's grid.

    `bounds` is (x_min, y_min, x_max, y_max) in world metres; the grid covers it, rounding
    outwards so no requested area is left off the map.
    """
    x_min, y_min, x_max, y_max = (float(v) for v in bounds)
    if not (x_max > x_min and y_max > y_min):
        raise ValueError(f"empty bounds {bounds}")
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    if radii["soft"] <= radii["lethal"]:
        raise ValueError("the soft radius must exceed the lethal radius")

    cols = math.ceil((x_max - x_min) / resolution)
    rows = math.ceil((y_max - y_min) / resolution)
    origin = (x_min, y_min)
    grid_r, grid_c = np.meshgrid(np.arange(rows), np.arange(cols), indexing="ij")
    centres = np.stack([x_min + (grid_c + 0.5) * resolution,
                        y_min + (grid_r + 0.5) * resolution], axis=-1).reshape(-1, 2)

    occupied = np.zeros(len(centres), dtype=bool)
    clearance = np.full(len(centres), np.inf)
    unmapped = [o.name for o in obstacles if not o.mapped]
    for obstacle in obstacles:
        if not obstacle.mapped:
            continue                     # the planner is not told; the robot must react
        occupied |= cell_overlap(obstacle.polygon, centres, resolution)
        clearance = np.minimum(clearance, polygon_distance(centres, obstacle.polygon))

    lethal = clearance <= radii["lethal"]
    cost = soft_cost(clearance, radii["lethal"], radii["soft"])
    return BakedMap(origin=origin, resolution=float(resolution),
                    occupied=occupied.reshape(rows, cols),
                    lethal=lethal.reshape(rows, cols), cost=cost.reshape(rows, cols),
                    clearance_m=clearance.reshape(rows, cols), radii=dict(radii),
                    unmapped=unmapped)
