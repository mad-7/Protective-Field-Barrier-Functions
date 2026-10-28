"""Classifying one cell of the coverage map. Pure functions.

`scripts/diag_coverage_map.py` puts a thin post at each cell of a dense grid around the
robot and reads the published scan.

A post is not a point: it spans several bins at close range, so each cell is judged over
every bin its angular extent overlaps (`bins_spanned`), not over the bin holding its axis
alone. Each cell is then one of:

  * **observed** -- some valid spanned bin reads the post (or nearer);
  * **invalid** -- every spanned bin is flagged unobserved, which the safety layers treat
    as breached;
  * **violation** -- a valid spanned bin could have reported the post and none did: free
    space reported where there is an obstacle, the failure hard rule 5 forbids;
  * **confounded** -- no valid spanned bin can tell, each already reading nearer than the
    post without it; reported, never counted as observed.

Pass criterion (supervisor, 2026-09-21): no violations, on either stream.
"""
from __future__ import annotations

import math
from collections.abc import Iterable, Sequence

import numpy as np

OBSERVED, INVALID, VIOLATION, CONFOUNDED = "observed", "invalid", "violation", "confounded"
CLASSES = (OBSERVED, INVALID, VIOLATION, CONFOUNDED)


def bin_index(bearing: float, n_bins: int) -> int:
    """Index of the uniform bin (bin 0 starts at -pi) containing `bearing` (rad)."""
    i = math.floor((bearing + math.pi) / (2 * math.pi) * n_bins)
    return min(max(i, 0), n_bins - 1)


def bins_spanned(bearing: float, post_distance: float, post_radius: float,
                 n_bins: int) -> list[int]:
    """Every bin the post's angular extent overlaps, nearest bin first.

    A post of radius r at distance d subtends +-asin(r / d) about its axis: 2.4 deg for a
    20 mm post at 0.24 m, against 3 deg filter bins. Its visible near face therefore often
    sits in the bin next to the one holding its axis, and asking only the axis bin scores
    that as free space where the scan in fact reports the post (measured 2026-09-22:
    two cells of COVMAP-s300_corners_20260922T212716Z, both within 0.01 deg of a bin edge,
    read range_max in the axis bin while the policy stream saw the post at 0.228 m).
    """
    if post_distance <= 0.0 or post_radius < 0.0:
        raise ValueError("post distance must be positive and its radius non-negative")
    half = math.asin(min(1.0, post_radius / post_distance))
    wrap = lambda b: math.atan2(math.sin(b), math.cos(b))
    first = bin_index(wrap(bearing - half), n_bins)
    last = bin_index(wrap(bearing + half), n_bins)
    span = [(first + k) % n_bins for k in range((last - first) % n_bins + 1)]
    centre = bin_index(bearing, n_bins)
    # The axis bin first, so a caller reading one bin still reads the most likely one.
    return [centre] + [i for i in span if i != centre] if centre in span else span


def classify(valid: Sequence[bool], range_with_post: Sequence[float],
             range_without: Sequence[float], post_distance: float,
             tolerance: float) -> str:
    """One cell's class, from every bin the post spans, with and without it.

    The sequences hold one value per spanned bin (`bins_spanned`), the bin holding the
    post's axis first. `post_distance` is from the body origin to that axis (m); the near
    face is a little closer, so reading up to `post_distance + tolerance` counts as seeing
    the post. A cell is **observed** when any valid spanned bin reports it, **invalid**
    when every spanned bin is flagged unobserved, **confounded** when no valid bin can tell
    (each already read short without the post), and a **violation** when a valid bin could
    have reported it and none did.
    """
    if not (len(valid) == len(range_with_post) == len(range_without)) or not len(valid):
        raise ValueError("one range, baseline and flag per spanned bin")
    reach = post_distance + tolerance
    seen = usable = False
    for ok, with_post, without in zip(valid, range_with_post, range_without, strict=True):
        if not ok:
            continue
        if without <= reach:            # already short without the post: tells us nothing
            continue
        usable = True
        if with_post <= reach:
            seen = True
    if seen:
        return OBSERVED
    if not any(valid):
        return INVALID
    return VIOLATION if usable else CONFOUNDED


def grid_cells(extent, depth: float, spacing: float, clearance: float) -> np.ndarray:
    """(n, 2) body-frame cells from the footprint edge (+ clearance) to `depth` beyond it.

    A square lattice, kept where a post of the given clearance would not touch the robot
    and lies within `depth` of the footprint edge.
    """
    if spacing <= 0.0 or depth <= 0.0:
        raise ValueError("spacing and depth must be positive")
    reach = float(np.max(extent(np.radians(np.arange(-180.0, 180.0, 1.0))))) + depth
    axis = np.arange(-reach, reach + 1e-12, spacing)
    xs, ys = np.meshgrid(axis, axis, indexing="ij")
    pts = np.column_stack([xs.ravel(), ys.ravel()])
    r = np.hypot(pts[:, 0], pts[:, 1])
    e = np.asarray(extent(np.arctan2(pts[:, 1], pts[:, 0])), dtype=float)
    keep = (r > e + clearance) & (r <= e + depth)
    return pts[keep]


def tally(classes: Iterable[str]) -> dict[str, int]:
    """Count of each class; unknown classes are an error, not a new bucket."""
    out: dict[str, int] = {c: 0 for c in CLASSES}
    for c in classes:
        if c not in out:
            raise ValueError(f"unknown class {c!r}")
        out[c] += 1
    return out
