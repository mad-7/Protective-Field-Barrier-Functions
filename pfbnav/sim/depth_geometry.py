"""Vision-sensor depth pixels to points in the robot frame. Pure functions.

The scanners are one-row perspective vision sensors. Turning a pixel's depth into a point
takes two conventions this project has not yet established from the simulator, and got
wrong on the first try (2026-09-21, `SCANCHECK_20260921T144640Z`): a wall 1.0 m to the
left came back as a straight line tilted ~60 deg -- what a mirrored pixel order would do
to a sensor whose axis sits at 30 deg.

  * **pixel order** (`mirrored`): whether pixel 0 lies towards the sensor's +x or -x.
  * **depth meaning** (`euclidean`): whether a pixel's depth is the perpendicular distance
    along the sensor's +z, or the euclidean distance along the pixel's own ray.

Both are parameters here, so the scanner check can reconstruct the same raw capture under
every combination and let the known wall decide; the Lua then uses whichever the
simulator confirms, through `scanners.pixel_mirror` and `scanners.depth_is_euclidean`.

Sensor frame (CoppeliaSim): the sensor looks along its +z; the image's horizontal axis is
its x. Matrices are CoppeliaSim's 12-float row-major 3x4 form
[Vx0 Vy0 Vz0 P0 Vx1 Vy1 Vz1 P1 Vx2 Vy2 Vz2 P2], mapping sensor coordinates into the frame
they were read relative to. Units: metres and radians.
"""
from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike


def pixel_tangents(resolution: int, fov: float, *, mirrored: bool) -> np.ndarray:
    """tan of each pixel's horizontal angle off the sensor axis, pixel 0 first.

    Pixel i's centre sits at (2 (i + 0.5) / n - 1) of the half-width of the image plane,
    whose half-width is tan(fov / 2). `mirrored` flips which side pixel 0 is on.
    """
    if resolution < 1:
        raise ValueError(f"resolution must be at least 1, got {resolution}")
    if not 0.0 < fov < math.pi:
        raise ValueError(f"a perspective field of view must be in (0, pi), got {fov}")
    fractions = 2.0 * (np.arange(resolution) + 0.5) / resolution - 1.0
    t = math.tan(0.5 * fov) * fractions
    return -t if mirrored else t


def sensor_points(depth: ArrayLike, fov: float, *, mirrored: bool,
                  euclidean: bool) -> np.ndarray:
    """Points in the sensor frame, (n, 3), from one row of depths in metres.

    Perpendicular depth z puts the point at (z t, 0, z). Euclidean depth D is measured
    along the pixel's ray, whose direction is (t, 0, 1) / sqrt(1 + t^2).
    """
    d = np.asarray(depth, dtype=float).ravel()
    t = pixel_tangents(d.size, fov, mirrored=mirrored)
    if euclidean:
        norm = np.sqrt(1.0 + t * t)
        return np.column_stack([d * t / norm, np.zeros_like(d), d / norm])
    return np.column_stack([d * t, np.zeros_like(d), d])


def apply_matrix(matrix: ArrayLike, points: ArrayLike) -> np.ndarray:
    """Apply a CoppeliaSim 12-float 3x4 matrix to (n, 3) points."""
    m = np.asarray(matrix, dtype=float).ravel()
    if m.size != 12:
        raise ValueError(f"a CoppeliaSim matrix has 12 values, got {m.size}")
    r = m.reshape(3, 4)
    p = np.asarray(points, dtype=float)
    return p @ r[:, :3].T + r[:, 3]


def matrix_from_pose(yaw: float, position: ArrayLike) -> list[float]:
    """The 12-float matrix of a horizontal sensor whose +z points along `yaw`.

    Its +x points to the right of the view (yaw - pi/2) and +y up, the arrangement a
    scanner pod's sensor has. For building known answers in tests; the scanner itself
    always reads the real matrix from the simulator.
    """
    z = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    x = np.array([math.cos(yaw - math.pi / 2), math.sin(yaw - math.pi / 2), 0.0])
    y = np.cross(z, x)
    p = np.asarray(position, dtype=float)
    rows = np.column_stack([x, y, z, p])
    return rows.ravel().tolist()


def wall_residuals(points: ArrayLike, normal_angle: float, distance: float) -> np.ndarray:
    """n . p - d, m, for points against a wall at `distance` along the unit normal n.

    Zero for every point on the wall, whatever sensor saw it and wherever that sensor
    sits -- which is why the scanner check uses a plane rather than per-bearing distances.
    """
    p = np.asarray(points, dtype=float)
    n = np.array([math.cos(normal_angle), math.sin(normal_angle)])
    return p[:, :2] @ n - distance


def cast_to_wall(fov: float, resolution: int, matrix: ArrayLike, normal_angle: float,
                 distance: float, *, mirrored: bool, euclidean: bool,
                 far: float = math.inf) -> np.ndarray:
    """Synthetic depths a sensor would report looking at a wall, under a convention.

    The inverse of `sensor_points`: each pixel's ray is intersected with the wall
    n . p = d, and the depth written in the requested convention. Rays that miss, or hit
    beyond `far`, read `far` -- as the simulator reports no return at its far clipping.
    """
    t = pixel_tangents(resolution, fov, mirrored=mirrored)
    m = np.asarray(matrix, dtype=float).reshape(3, 4)
    origin = m[:, 3]
    n = np.array([math.cos(normal_angle), math.sin(normal_angle), 0.0])
    depths = np.full(resolution, far, dtype=float)
    for i, ti in enumerate(t):
        ray_sensor = np.array([ti, 0.0, 1.0])            # direction scaled to unit z
        ray = m[:, :3] @ ray_sensor
        denom = float(n @ ray)
        if denom <= 1e-12:
            continue
        s = (distance - float(n @ origin)) / denom       # z-depth along this ray
        if s <= 0.0:
            continue
        depth = s * math.sqrt(1.0 + ti * ti) if euclidean else s
        if depth < far:
            depths[i] = depth
    return depths
