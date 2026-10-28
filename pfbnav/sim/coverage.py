"""Which scan bins are actually observed: position-aware coverage. Pure functions.

A bin is **valid** only if every point of its sector, from the robot's footprint edge out
to `depth` beyond it, is seen by some sensor: inside that sensor's field of view, within
its range, and not hidden behind the robot itself. Anything less and the bin carries
unobserved space, which is never treated as free (hard rule 5).

Why this exists (2026-09-21): the first scanner marked a bin valid when the bin's bearing
from the body origin fell inside a sensor's field of view -- as if every sensor sat at the
origin. The two centreline pods sit 0.325 m from it, and beside the robot a zone out to
0.563 m was seen by neither while its bins read free space. Coverage has to be judged from
where each sensor is.

Occlusion is modelled in the plane, by the footprint (`FootprintTable`) as a polygon: a
sight line that has left the footprint and re-enters it is blocked. A sensor mounted
inside the envelope -- as a scanner body always is -- starts its rays inside, so the rule
is "exits and comes back", not "touches": counted as boundary crossings, a sight line from
inside may cross once (out), one from outside not at all. The footprint includes parts below and above the scan plane (the
wheels, for one), so this over-states occlusion rather than under-stating it; which
direction the error runs is recorded against the scan-plane height in `configs/base.yaml`.

Frames: body frame, +x forward, +y left; bearings in (-pi, pi]; metres, radians.
"""
from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np

ExtentFn = Callable[[np.ndarray], np.ndarray]

# Sampling of the region a mask is judged over. Blind zones are regions tens of
# centimetres across, so this is ample for the mask; whether a 20 mm post can hide is
# what the empirical coverage map (scripts/diag_coverage_map.py) tests, not this.
BEARING_STEP_DEG = 1.0
RADIAL_STEP_M = 0.01
# The footprint polygon used for occlusion: one vertex per degree, as the table is.
POLYGON_STEP_DEG = 1.0
# A point this close outside the footprint edge is where the region starts.
EDGE_CLEARANCE_M = 0.002
# Points are tested against the polygon in chunks of this many, to bound memory.
CHUNK = 4000


@dataclass(frozen=True)
class SensorView:
    """One scanner sensor as seen in the body frame."""

    x: float                 # m, sensor origin
    y: float                 # m
    heading: float           # rad, optical axis
    fov: float               # rad, full horizontal field of view
    range_max: float         # m


def _wrap(a: np.ndarray) -> np.ndarray:
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi


def inside_footprint(points: np.ndarray, extent: ExtentFn) -> np.ndarray:
    """Whether each (n, 2) point lies strictly inside the footprint's boundary."""
    p = np.asarray(points, dtype=float)
    r = np.hypot(p[:, 0], p[:, 1])
    return r < np.asarray(extent(np.arctan2(p[:, 1], p[:, 0])), dtype=float)


def footprint_polygon(extent: ExtentFn, step_deg: float = POLYGON_STEP_DEG) -> np.ndarray:
    """The footprint boundary as an (n, 2) polygon, one vertex per `step_deg`."""
    b = np.radians(np.arange(-180.0, 180.0, step_deg))
    e = np.asarray(extent(b), dtype=float)
    return np.column_stack([e * np.cos(b), e * np.sin(b)])


def _crossings(start: np.ndarray, ends: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """How many polygon edges each segment start->end crosses (proper intersections)."""
    a = polygon
    b = np.roll(polygon, -1, axis=0)
    e = b - a                                          # (m, 2) edges
    out = np.zeros(len(ends), dtype=int)
    for lo in range(0, len(ends), CHUNK):
        q = ends[lo:lo + CHUNK]
        d = q - start                                  # (k, 2) segments
        denom = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]
        w = a[None, :, :] - start                      # (1, m, 2)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (w[..., 0] * e[None, :, 1] - w[..., 1] * e[None, :, 0]) / denom
            u = (w[..., 0] * d[:, None, 1] - w[..., 1] * d[:, None, 0]) / denom
        hit = (np.abs(denom) > 1e-15) & (t > 1e-9) & (t < 1 - 1e-9) & (u >= 0) & (u < 1)
        out[lo:lo + CHUNK] = hit.sum(axis=1)
    return out


def visible(sensor: SensorView, points: np.ndarray, extent: ExtentFn | None = None) -> np.ndarray:
    """Whether `sensor` sees each (n, 2) point: in its field of view, in range, unoccluded.

    Occlusion (when `extent` is given): the sight line from the sensor to the point may
    cross the footprint boundary once if the sensor is inside the footprint (leaving it)
    and not at all if outside; any further crossing means it re-entered the robot.
    """
    p = np.asarray(points, dtype=float)
    d = p - np.array([sensor.x, sensor.y])
    dist = np.hypot(d[:, 0], d[:, 1])
    off = np.abs(_wrap(np.arctan2(d[:, 1], d[:, 0]) - sensor.heading))
    ok = (off <= 0.5 * sensor.fov + 1e-9) & (dist <= sensor.range_max + 1e-9)
    if extent is None or not ok.any():
        return ok
    start = np.array([sensor.x, sensor.y])
    allowed = 1 if inside_footprint(start[None, :], extent)[0] else 0
    idx = np.flatnonzero(ok)
    ok[idx] = _crossings(start, p[idx], footprint_polygon(extent)) <= allowed
    return ok


def region_points(extent: ExtentFn, depth: float,
                  bearing_step_deg: float = BEARING_STEP_DEG,
                  radial_step: float = RADIAL_STEP_M) -> tuple[np.ndarray, np.ndarray]:
    """Sample points from the footprint edge out to `depth` beyond it, with their bearings."""
    if depth <= 0.0:
        raise ValueError(f"depth must be positive, got {depth}")
    bearings = np.radians(np.arange(-180.0 + 0.5 * bearing_step_deg, 180.0, bearing_step_deg))
    pts, brg = [], []
    for b, e in zip(bearings, np.asarray(extent(bearings), dtype=float), strict=True):
        radii = np.arange(e + EDGE_CLEARANCE_M, e + depth + 1e-12, radial_step)
        pts.append(np.column_stack([radii * math.cos(b), radii * math.sin(b)]))
        brg.append(np.full(radii.size, b))
    return np.vstack(pts), np.concatenate(brg)


def observed(sensors: Sequence[SensorView], points: np.ndarray,
             extent: ExtentFn | None = None) -> np.ndarray:
    """Whether each point is seen by at least one sensor."""
    seen = np.zeros(len(points), dtype=bool)
    for s in sensors:
        todo = ~seen
        if todo.any():
            seen[todo] = visible(s, points[todo], extent)
    return seen


def bin_mask(sensors: Sequence[SensorView], n_bins: int, extent: ExtentFn, depth: float,
             *, occlusion: bool = True) -> np.ndarray:
    """Per-bin validity over a uniform binning of the full turn (bin 0 starts at -pi).

    A bin is valid iff every sampled point of its sector, from the footprint edge to
    `depth` beyond it, is observed. One unseen point makes the whole bin invalid: the
    packet has one validity flag per bin, and a bin that is partly unobserved cannot be
    reported as free.
    """
    if n_bins < 1:
        raise ValueError(f"need at least one bin, got {n_bins}")
    pts, brg = region_points(extent, depth)
    seen = observed(sensors, pts, extent if occlusion else None)
    idx = np.clip(np.floor((brg + math.pi) / (2 * math.pi) * n_bins).astype(int), 0, n_bins - 1)
    unseen_bins = np.unique(idx[~seen])
    mask = np.ones(n_bins, dtype=bool)
    mask[unseen_bins] = False
    return mask


def bin_masks(sensors: Sequence[SensorView], bin_counts: Sequence[int], extent: ExtentFn,
              depth: float, *, occlusion: bool = True) -> dict[int, np.ndarray]:
    """`bin_mask` for several binnings at once, from one visibility computation."""
    if any(n < 1 for n in bin_counts):
        raise ValueError("need at least one bin per stream")
    pts, brg = region_points(extent, depth)
    seen = observed(sensors, pts, extent if occlusion else None)
    out = {}
    for n in bin_counts:
        idx = np.clip(np.floor((brg + math.pi) / (2 * math.pi) * n).astype(int), 0, n - 1)
        mask = np.ones(n, dtype=bool)
        mask[np.unique(idx[~seen])] = False
        out[int(n)] = mask
    return out


def unobserved_extent(sensors: Sequence[SensorView], extent: ExtentFn, depth: float,
                      *, occlusion: bool = True) -> dict[str, float]:
    """Area (m^2) and furthest reach beyond the footprint (m) of what no sensor sees."""
    pts, brg = region_points(extent, depth)
    seen = observed(sensors, pts, extent if occlusion else None)
    if seen.all():
        return {"area_m2": 0.0, "reach_m": 0.0}
    r = np.hypot(pts[:, 0], pts[:, 1])
    e = np.asarray(extent(brg), dtype=float)
    # Area of polar samples: r * dr * dtheta.
    cell = r * RADIAL_STEP_M * math.radians(BEARING_STEP_DEG)
    return {"area_m2": float(cell[~seen].sum()), "reach_m": float((r - e)[~seen].max())}
