"""Reading per-step traces: the step-0 comparison the supervisor asked for (2026-10-10). Pure.

The hypothesis to test was a heading mismatch in hall placement, the kind the youBot_ref quarter
turn once caused, so a step-0 yaw error is classified by the quarter turn it is nearest. A
mismatch from a frame convention lands near +-90 or 180 deg; noise and settling land near 0.
"""
from __future__ import annotations

import math
from collections.abc import Callable, Sequence

import numpy as np

# Degrees within which an error counts as "at" a quarter turn. Wide enough for settling and a
# few degrees of drift, narrow enough that a 45-degree error is not called either.
QUARTER_TURN_TOL_DEG = 15.0


def signed_angle_deg(a: float, b: float) -> float:
    """a - b in degrees, wrapped to (-180, 180]; a and b in rad."""
    d = math.degrees(math.atan2(math.sin(a - b), math.cos(a - b)))
    return 180.0 if d == -180.0 else d


def quarter_turn_class(error_deg: float, tol_deg: float = QUARTER_TURN_TOL_DEG) -> str:
    """'0', '+90', '-90' or '180' if `error_deg` is within `tol_deg` of it, else 'other'."""
    e = signed_angle_deg(math.radians(error_deg), 0.0)
    for label, centre in (("0", 0.0), ("+90", 90.0), ("-90", -90.0), ("180", 180.0)):
        if abs(signed_angle_deg(math.radians(e), math.radians(centre))) <= tol_deg:
            return label
    return "other"


def first_segment_bearing_body_deg(path: Sequence[Sequence[float]], yaw: float,
                                   min_length: float = 1e-9) -> float | None:
    """The route's first segment's direction relative to the robot's heading, degrees.

    `path` is the plan's vertices in the layout frame, `yaw` the robot's measured heading in the
    same frame (rad). The first segment is from the first vertex to the first one at least
    `min_length` from it. None for a path with no such vertex.
    """
    x0, y0 = (float(v) for v in path[0][:2])
    for p in path[1:]:
        dx, dy = float(p[0]) - x0, float(p[1]) - y0
        if math.hypot(dx, dy) >= min_length:
            return signed_angle_deg(math.atan2(dy, dx), yaw)
    return None


def scan_points(ranges, bearings, valid, range_min: float) -> np.ndarray:
    """Body-frame points (N, 2), m, one per scan bin. An unobserved bin is a point at `range_min`
    on its bearing (hard rule 5: unobserved space is never free), as recovery treats it."""
    r = np.where(np.asarray(valid, dtype=bool), np.asarray(ranges, dtype=float), range_min)
    b = np.asarray(bearings, dtype=float)
    return np.column_stack([r * np.cos(b), r * np.sin(b)])


def nearest_to_footprint(distance: Callable[[np.ndarray], np.ndarray], ranges, bearings, valid,
                         range_min: float) -> dict:
    """The scan point nearest the footprint: its signed distance (m; negative inside) given by
    `distance` (body-frame points -> signed distances, e.g. `RecoveryGeometry.footprint_distance`),
    its bearing (deg) and whether its bin was observed."""
    d = np.asarray(distance(scan_points(ranges, bearings, valid, range_min)), dtype=float)
    i = int(np.argmin(d))
    return {"distance_m": float(d[i]), "bearing_deg": math.degrees(float(bearings[i])),
            "valid": bool(np.asarray(valid, dtype=bool)[i])}


def bins_inside(ranges, bearings, valid, field) -> list[dict]:
    """Every bin inside `field` (the boundary range per bearing, m): an observed range below it,
    or an unobserved bin (which breaches by rule). `by_m` is how far inside; None if unobserved."""
    out = []
    for b, r, v, f in zip(bearings, ranges, valid, field, strict=True):
        if not v or r < f:
            out.append({"bearing_deg": math.degrees(float(b)), "range_m": float(r),
                        "field_m": float(f), "by_m": float(f - r) if v else None,
                        "valid": bool(v)})
    return out

