"""Exact 2-D ray casting against polygons: a synthetic scan for offline checks. Pure.

Used where a probe must predict, without the simulator, what the scanner will see: the synthetic
recovery control's known-answer check (`pfbnav/eval/recovery_control.py`) and the controller tests.
Rays start at the body origin; bearings are body-frame, rad; polygons are body-frame vertex lists,
m. A ray that hits nothing reads `range_max` and is valid (an observed, empty bearing).
"""
from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np
from numpy.typing import ArrayLike


def bin_bearings(n: int) -> np.ndarray:
    """`n` evenly spaced bin centres in (-pi, pi], ascending."""
    return np.array([math.remainder(-math.pi + 2.0 * math.pi * (k + 0.5) / n, 2.0 * math.pi)
                     for k in range(n)])


def ray_hit(theta: float, polygon: ArrayLike) -> float:
    """Distance from the origin along bearing `theta` to the polygon's boundary, or inf."""
    p = np.asarray(polygon, dtype=float)
    dx, dy = math.cos(theta), math.sin(theta)
    best = math.inf
    for a, b in zip(p, np.roll(p, -1, axis=0), strict=True):
        ex, ey = b[0] - a[0], b[1] - a[1]
        den = dx * (-ey) - dy * (-ex)
        if abs(den) < 1e-15:
            continue
        t = (a[0] * (-ey) - a[1] * (-ex)) / den          # origin + t d = a + u e
        u = (dx * a[1] - dy * a[0]) / den
        if t > 0.0 and 0.0 <= u <= 1.0:
            best = min(best, t)
    return best


def ray_ranges(bearings: ArrayLike, polygons: Sequence[ArrayLike], range_max: float) -> np.ndarray:
    """The nearest hit per bearing over all polygons, capped at `range_max`."""
    b = np.asarray(bearings, dtype=float)
    r = np.full(b.shape, float(range_max))
    for poly in polygons:
        r = np.minimum(r, [ray_hit(float(t), poly) for t in b])
    return r


def rays_hit(thetas: ArrayLike, polygons: Sequence[ArrayLike], range_max: float) -> np.ndarray:
    """`ray_ranges`, vectorised over rays and every polygon edge at once (same semantics: the
    nearest boundary hit at t > 0, capped at `range_max`)."""
    th = np.asarray(thetas, dtype=float)
    r = np.full(th.shape, float(range_max))
    if not polygons:
        return r
    a = np.vstack([np.asarray(p, dtype=float)[:, :2] for p in polygons])
    b = np.vstack([np.roll(np.asarray(p, dtype=float)[:, :2], -1, axis=0) for p in polygons])
    e = b - a                                                     # (m, 2) edges
    dx, dy = np.cos(th)[:, None], np.sin(th)[:, None]             # (n, 1)
    den = dx * (-e[None, :, 1]) - dy * (-e[None, :, 0])           # (n, m)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (a[None, :, 0] * (-e[None, :, 1]) - a[None, :, 1] * (-e[None, :, 0])) / den
        u = (dx * a[None, :, 1] - dy * a[None, :, 0]) / den
    hit = (np.abs(den) >= 1e-15) & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
    t = np.where(hit, t, np.inf)
    return np.minimum(r, t.min(axis=1))


def sector_scan(bins: int, polygons: Sequence[ArrayLike], range_max: float,
                rays_per_bin: int) -> tuple[np.ndarray, np.ndarray]:
    """The FILTER stream as the simulator sends it (`pfbnav/sim/scan_packet.py`): per sector of
    `bins` uniform sectors, the nearest hit over `rays_per_bin` evenly spaced rays, at the bearing it
    was measured at; a sector with no hit reads `range_max` at its centre. Returns (ranges, bearings),
    bearings strictly ascending in (-pi, pi].

    A single ray per bin centre (`ray_ranges` on `bin_bearings`) leaves the surface between centres
    unobserved: a corner there appears only once rotation brings it under a ray (found 2026-10-24,
    the PFB acceptance run's one unattributed stop)."""
    centres = bin_bearings(bins)
    width = 2.0 * math.pi / bins
    offsets = ((np.arange(rays_per_bin) + 0.5) / rays_per_bin - 0.5) * width
    r = rays_hit((centres[:, None] + offsets[None, :]).ravel(), polygons,
                 range_max).reshape(bins, rays_per_bin)
    k = np.argmin(r, axis=1)
    ranges = r[np.arange(bins), k]
    bearings = np.where(ranges < range_max, centres + offsets[k], centres)
    return ranges, bearings


def to_body_frame(polygon: ArrayLike, pose: ArrayLike) -> np.ndarray:
    """A world (or start-frame) polygon in the body frame of `pose` (x, y, yaw)."""
    p = np.asarray(polygon, dtype=float)
    x, y, yaw = (float(v) for v in np.asarray(pose, dtype=float).ravel()[:3])
    c, s = math.cos(yaw), math.sin(yaw)
    d = p - np.array([x, y])
    return np.column_stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]])
