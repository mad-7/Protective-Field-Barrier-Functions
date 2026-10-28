"""Polygon footprint of the robot, measured from scene geometry. Pure functions.

`pfbnav/safety/pfb.py` models the footprint as a rectangle, which is exact only while the
robot *is* a rectangle. Measurement showed it is not: the wheels stand proud sideways and
the sensor housings stand proud fore and aft, so the true outline is closer to a cross.
A rectangle covering all of it over-states the corners; one fitted to the corners
under-states the protrusions.

These functions build the convex hull of the collidable geometry in the body frame and
report, per quadrant, the radius of the largest circle that fits inside the outline and the
smallest that contains it. Those are the two numbers a planner needs: the inscribed radius
bounds what can never collide, the circumscribed radius bounds what always might.

Frames: body frame, +x forward, +y left, metres. Bearings in (-pi, pi], measured from +x.
"""
from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from typing import TypeAlias

import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection

Points: TypeAlias = Sequence[Sequence[float]] | np.ndarray

# The four body-frame quadrants, as (name, bearing range in radians).
QUADRANTS: tuple[tuple[str, float, float], ...] = (
    ("front-left", 0.0, math.pi / 2),
    ("rear-left", math.pi / 2, math.pi),
    ("rear-right", -math.pi, -math.pi / 2),
    ("front-right", -math.pi / 2, 0.0),
)


def convex_hull(points: Points) -> np.ndarray:
    """Vertices of the 2-D convex hull, counter-clockwise, as an (N, 2) array."""
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError(f"points must be (N, 2), got shape {pts.shape}")
    if len(pts) < 3:
        raise ValueError(f"a hull needs at least 3 points, got {len(pts)}")
    hull = ConvexHull(pts)
    return pts[hull.vertices]


def contains_origin(hull: Points) -> bool:
    """Whether the body origin lies strictly inside the hull.

    Everything below measures radii *from the origin*, which is meaningless if the origin
    is outside the outline. For a robot whose reference frame sits inside its own body this
    is always true, and checking it turns a silent nonsense into a loud one.
    """
    poly = np.asarray(hull, dtype=float)
    n = len(poly)
    signs = []
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        edge, to_origin = b - a, -a
        signs.append(math.copysign(1.0, edge[0] * to_origin[1] - edge[1] * to_origin[0]))
    return all(s > 0 for s in signs) or all(s < 0 for s in signs)


def radial_extent(hull: Points, bearing: float) -> float:
    """Distance from the body origin to the hull boundary along `bearing`, in metres.

    The hull is convex and contains the origin, so the ray crosses the boundary exactly
    once and the answer is unique.
    """
    poly = np.asarray(hull, dtype=float)
    direction = np.array([math.cos(bearing), math.sin(bearing)])

    def cross(u, v) -> float:
        return float(u[0] * v[1] - u[1] * v[0])

    best: float | None = None
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        edge = b - a
        # Solve t * direction = a + s * edge for t >= 0 and s in [0, 1]. Crossing both
        # sides with `edge` isolates t, and with `direction` isolates s.
        denom = cross(direction, edge)
        if abs(denom) < 1e-15:
            continue  # ray parallel to this edge
        t = cross(a, edge) / denom
        s = cross(a, direction) / denom
        if not -1e-12 <= s <= 1 + 1e-12:
            continue
        if t >= 0 and (best is None or t > best):
            best = t
    if best is None:
        raise ValueError(f"no hull edge crosses the ray at bearing {bearing}")
    return best


def radial_extents(hull: Points, bearings: Sequence[float] | np.ndarray) -> np.ndarray:
    """`radial_extent` at many bearings at once, metres. Vectorised; same answer.

    For a convex polygon containing the origin, each edge lies on a line at distance d
    with outward normal n, and the ray along u leaves the polygon through the edge that
    minimises d / (n . u) over edges facing it (n . u > 0).
    """
    poly = np.asarray(hull, dtype=float)
    if not contains_origin(poly):
        raise ValueError("the body origin is outside the polygon")
    edges = np.roll(poly, -1, axis=0) - poly
    normals = np.column_stack([edges[:, 1], -edges[:, 0]])
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    # Orient outward: the origin is inside, so every support distance must be positive.
    dist = np.einsum("ij,ij->i", normals, poly)
    flip = dist < 0
    normals[flip] *= -1.0
    dist[flip] *= -1.0
    b = np.asarray(bearings, dtype=float)
    u = np.column_stack([np.cos(b), np.sin(b)])
    facing = u @ normals.T                          # (n_bearings, n_edges)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(facing > 1e-15, dist[None, :] / facing, np.inf)
    return t.min(axis=1)


def quadrant_radii(hull: Points, *, samples: int = 181) -> dict[str, dict[str, float]]:
    """Inscribed and circumscribed radius of the outline within each body quadrant.

    `samples` bearings are taken across each quadrant, inclusive of both bounds so that
    the axes belong to both neighbouring quadrants. The inscribed radius is the smallest
    radial extent found and the circumscribed the largest, each accurate to the angular
    resolution; a sharp corner between two samples is under-reported, which is why the
    default resolution is one degree.
    """
    if samples < 2:
        raise ValueError(f"samples must be at least 2, got {samples}")
    poly = np.asarray(hull, dtype=float)
    if not contains_origin(poly):
        raise ValueError(
            "the body origin is outside the footprint outline, so radii measured from it "
            "are meaningless; check the frame the points were expressed in"
        )
    out: dict[str, dict[str, float]] = {}
    for name, lo, hi in QUADRANTS:
        extents = [radial_extent(poly, b) for b in np.linspace(lo, hi, samples)]
        out[name] = {
            "inscribed": float(min(extents)),
            "circumscribed": float(max(extents)),
            "inscribed_bearing_deg": float(math.degrees(
                np.linspace(lo, hi, samples)[int(np.argmin(extents))])),
            "circumscribed_bearing_deg": float(math.degrees(
                np.linspace(lo, hi, samples)[int(np.argmax(extents))])),
        }
    return out


def symmetry_report(radii: dict[str, dict[str, float]]) -> dict[str, float]:
    """How far the outline departs from fore-aft and left-right symmetry, in metres.

    Reported rather than asserted: an asymmetric robot is a fact to record, not an error.
    """
    def circ(name: str) -> float:
        return radii[name]["circumscribed"]

    return {
        "fore_aft_difference": abs((circ("front-left") + circ("front-right"))
                                   - (circ("rear-left") + circ("rear-right"))) / 2.0,
        "left_right_difference": abs((circ("front-left") + circ("rear-left"))
                                     - (circ("front-right") + circ("rear-right"))) / 2.0,
    }


def outermost_contact(contact_at: Callable[[float], bool], start: float, stop: float,
                      step: float, tol: float) -> tuple[float, float] | None:
    """Outermost offset at which a thin probe plate touches, bracketed as (lo, hi), metres.

    `contact_at(d)` says whether a plate of thickness t centred at offset d touches the
    geometry. `sim.checkCollision` tests *surface* intersection, so a plate touches a
    connected part exactly when d lies within t/2 of the part's projection interval, and
    a solid that swallows a part whole does not touch it at all -- which is why a
    half-space block cannot be used, and was not, after it reported nothing on
    2026-09-18.

    The scan walks inward from `start` in steps of `step` until the first contact, then
    bisects between that offset and the previous one, where the contact function is
    monotone. The returned bracket holds the outer contact boundary, which is the
    geometry's support value plus t/2; the caller subtracts t/2.

    **What the scan can miss.** A part at least as thick as the plate along the probe
    direction registers over a contiguous band at least 2t wide, so with `step` <= t it
    cannot be skipped. A part thinner than the plate, w < t, can sit wholly inside it,
    where no surface is cut, and registers only in two bands each w wide: the scan skips
    it whenever `step` > w. The earlier docstring claimed no part could be skipped; that
    held only without surface semantics, and an arm cylinder disproved it on 2026-09-18.
    Callers retry a miss with a finer step and fall back to a bounding box.

    Returns None if nothing is touched down to `stop`; raises if there is contact already
    at `start`, because then `start` was not outside the geometry and the scan would
    under-state it.
    """
    if step <= 0.0 or tol <= 0.0 or start <= stop:
        raise ValueError(f"need start > stop, step > 0, tol > 0; got {start}, {stop}, "
                         f"{step}, {tol}")
    if contact_at(start):
        raise ValueError(f"contact at the scan start ({start:.4f} m): the start is not "
                         "outside the geometry, so the scan would under-state it")
    outside = start
    d = start - step
    while d >= stop:
        if contact_at(d):
            inside = d
            while outside - inside > tol:
                mid = 0.5 * (inside + outside)
                if contact_at(mid):
                    inside = mid
                else:
                    outside = mid
            return inside, outside
        outside = d
        d -= step
    return None


def polygon_from_support(bearings: Sequence[float] | np.ndarray,
                         support: Sequence[float] | np.ndarray) -> np.ndarray:
    """Convex outline from a sampled support function, as counter-clockwise (N, 2) vertices.

    The outline is the intersection of the half-planes {p : p . n(b) <= h(b)} over the
    sampled bearings b. Between samples it can only over-state the true convex outline, so
    it is conservative, and it converges to the convex hull as the sampling densifies.
    Needs every h > 0 -- the body origin must be strictly inside -- and bearings spanning
    the full circle, or the intersection is unbounded.
    """
    b = np.asarray(bearings, dtype=float)
    h = np.asarray(support, dtype=float)
    if b.shape != h.shape or b.ndim != 1 or len(b) < 3:
        raise ValueError("bearings and support must be matching 1-D arrays of length >= 3")
    if np.any(h <= 0.0):
        raise ValueError("every support value must be positive: the origin must be inside")
    gaps = np.diff(np.sort(np.mod(b, 2.0 * math.pi)))
    widest = max(float(np.max(gaps, initial=0.0)),
                 2.0 * math.pi - float(np.mod(b, 2.0 * math.pi).max()
                                       - np.mod(b, 2.0 * math.pi).min()))
    if widest >= math.pi:
        raise ValueError("bearings leave a gap of pi or more; the outline would be unbounded")
    halfspaces = np.column_stack([np.cos(b), np.sin(b), -h])
    vertices = HalfspaceIntersection(halfspaces, np.zeros(2)).intersections
    return convex_hull(vertices)


def support_of_points(points: Points, bearings: Sequence[float] | np.ndarray) -> np.ndarray:
    """Support function max_p (p . n(b)) of a point set at each bearing, metres."""
    pts = np.asarray(points, dtype=float)
    b = np.asarray(bearings, dtype=float)
    normals = np.column_stack([np.cos(b), np.sin(b)])
    return (pts[:, :2] @ normals.T).max(axis=0)


# Added to every footprint-table knot on top of the measured undershoot, metres.
SPLINE_GUARD_M = 1e-6


def footprint_table_from_polygon(polygon: Points, *, check_samples: int = 7200):
    """A 1-degree footprint table whose spline never falls below the polygon's e(psi).

    Knots are the polygon's radial extent at each whole degree. A cubic spline through
    them can dip below the true e(psi) between knots -- at a vertex, where e has a kink,
    it does -- and under-stating e is the unsafe direction. So the spline is compared with
    the true extent at `check_samples` bearings plus every vertex's bearing, and every
    knot is raised by the worst undershoot; since the spline is linear in the knot values, raising all knots by c
    raises the spline by exactly c. Returns (table, margin_added_m).
    """
    from pfbnav.safety.footprint_table import FootprintTable

    poly = np.asarray(polygon, dtype=float)
    knots = np.radians(np.arange(-180.0, 180.0, 1.0))
    table = FootprintTable(radial_extents(poly, knots))
    # The true e(psi) peaks exactly at the polygon's vertices, where it has a kink, and a
    # uniform grid can miss a peak by enough to matter (0.17 mm on a 0.35 x 0.21 m
    # rectangle at 0.05 deg spacing). So each vertex's own bearing is always checked.
    vertex_bearings = np.arctan2(poly[:, 1], poly[:, 0])
    dense = np.concatenate([np.linspace(-math.pi, math.pi, check_samples, endpoint=False),
                            vertex_bearings])
    truth = radial_extents(poly, dense)
    undershoot = float(np.max(truth - table.extent(dense)))
    # Between check bearings the spline can still dip by a sub-micron amount (0.12 um on a
    # 12-gon at 0.05 deg spacing); a fixed 1 um guard covers it, three orders of magnitude
    # below the probe's own 0.5 mm resolution.
    margin = max(0.0, undershoot) + SPLINE_GUARD_M
    if margin > 0.0:
        table = FootprintTable(table.extents + margin)
    return table, margin
