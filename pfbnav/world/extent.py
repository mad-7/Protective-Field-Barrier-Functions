"""Does every placed world fit on the scene's floor, with nothing else on it? Pure functions.

Found 2026-10-09 (the researcher): each episode moves and rotates the hall so the scenario's
start lands on the robot, so a hall reaches up to its diagonal from the robot -- and the
scene's floor ended at 7.5 m while halls reached 14.2 m. Nothing checked it, because every
probe and gate ran near the origin. Ruled 2026-10-10 (supervisor): the floor becomes 35 x 35 m
(`sim.floor_half_extent_m: 17.5`), and a guard test checks every built set -- the test sets
too once built -- and every unmapped obstacle, not only the hall bounds. Hard rule 18 is the
general form.

**The check is rotation-free.** A point `r` from the scenario's start lands `r` from the robot
whatever yaw the placement takes, so it lands within `spawn + r` of the origin, where `spawn`
is how far the robot's scene pose is from the origin. That disc lies inside the floor's square
when `spawn + reach <= half_extent`: no assumption about the robot's heading, and the robot's
spawn is read back on the simulator against `sim.robot_spawn_radius_m`.

**The floor's half of the guarantee** (`floor_covers`, `foreign_shapes_on`) is checked on the
simulator against the scene (`tests/test_floor_sim.py`): the tiles must tile the declared square
without gaps, and no shape outside the robot and the floor may stand on it, because a hall can
be placed anywhere on it.
"""
from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike

from pfbnav.world.layouts import Layout


def world_reach(layout: Layout, start_xy: ArrayLike) -> float:
    """The farthest any part of the world lies from the scenario's start, in m.

    Over the hall's bounds (its four corners), every mapped obstacle and every unmapped
    obstacle (each polygon's vertices; all are convex polygons, so the farthest point is a
    vertex).
    """
    x0, y0, x1, y1 = (float(v) for v in layout.bounds)
    points = [np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])]
    points += [np.asarray(o.polygon, dtype=float) for o in (*layout.mapped, *layout.unmapped)]
    p = np.vstack(points)
    return float(np.max(np.linalg.norm(p - np.asarray(start_xy, dtype=float)[:2], axis=1)))


def fits_on_floor(reach: float, spawn_radius: float, half_extent: float) -> bool:
    """Whether a world reaching `reach` from its start lands on a floor [-h, h]^2, any yaw."""
    return spawn_radius + reach <= half_extent


def floor_covers(tiles: Iterable[tuple[float, float, float, float]], half_extent: float,
                 tol: float = 1e-6) -> list[str]:
    """Problems with a set of axis-aligned floor tiles as a cover of [-h, h]^2 (empty = covers).

    Each tile is (cx, cy, half_x, half_y), world frame, m. The tiles must lie inside the square
    and their areas must add up to its area with no two overlapping, which together mean they
    tile it exactly: no gap a hall could stand over, no tile beyond the declared floor.
    """
    boxes = [tuple(float(v) for v in t) for t in tiles]
    h = float(half_extent)
    problems = []
    for cx, cy, hx, hy in boxes:
        if cx - hx < -h - tol or cx + hx > h + tol or cy - hy < -h - tol or cy + hy > h + tol:
            problems.append(f"tile at ({cx:+.3f}, {cy:+.3f}) reaches past +-{h} m")
    for i, (ax, ay, ahx, ahy) in enumerate(boxes):
        for bx, by, bhx, bhy in boxes[i + 1:]:
            ox = min(ax + ahx, bx + bhx) - max(ax - ahx, bx - bhx)
            oy = min(ay + ahy, by + bhy) - max(ay - ahy, by - bhy)
            if ox > tol and oy > tol:
                problems.append(f"tiles at ({ax:+.3f}, {ay:+.3f}) and ({bx:+.3f}, {by:+.3f}) "
                                "overlap")
    area = sum(4.0 * b[2] * b[3] for b in boxes)
    if not problems and abs(area - 4.0 * h * h) > tol * max(1.0, 4.0 * h * h):
        problems.append(f"the tiles cover {area:.6f} m^2 of the floor's {4.0 * h * h:.6f} m^2")
    return problems


def triangle_overlaps_square(triangle: ArrayLike, half_extent: float) -> bool:
    """Whether a triangle's plan view (xy, m) overlaps the open square (-h, h)^2.

    Separating-axis test: two convex shapes are disjoint exactly when one of their edge
    normals separates them, here x, y and the triangle's three edge normals. Touching at an
    edge or corner does not count as overlap.
    """
    t = np.asarray(triangle, dtype=float)[:, :2]
    h = float(half_extent)
    square = np.array([[-h, -h], [h, -h], [h, h], [-h, h]])
    axes = [np.array([1.0, 0.0]), np.array([0.0, 1.0])]
    for i in range(3):
        edge = t[(i + 1) % 3] - t[i]
        if np.hypot(*edge) > 0.0:
            axes.append(np.array([-edge[1], edge[0]]))
    for axis in axes:
        a, b = t @ axis, square @ axis
        if a.max() <= b.min() or b.max() <= a.min():
            return False
    return True


def foreign_shapes_on(shapes: Mapping[str, Sequence[ArrayLike]],
                      half_extent: float) -> list[str]:
    """Names of the shapes with any triangle over the floor's square, in plan.

    `shapes` maps a name to its triangles in world coordinates. A hall can be placed anywhere
    on the floor, so anything standing on it is an obstacle no scenario recorded.
    """
    return [name for name, triangles in shapes.items()
            if any(triangle_overlaps_square(tri, half_extent) for tri in triangles)]

