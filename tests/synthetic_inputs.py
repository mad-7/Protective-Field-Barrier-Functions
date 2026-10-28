"""Synthetic `ClassicalInput`s for the controller and recovery tests: a 120-bin scan ray-cast against
body-frame polygons, and a route given by its lookahead point and goal. No simulator."""
import math

import numpy as np

from pfbnav.envs.controller_input import ClassicalInput, RouteView, ScanStream

BINS = 120
RANGE_MAX = 5.0


def bearings(n: int = BINS) -> np.ndarray:
    """Evenly spaced bin centres in (-pi, pi], ascending."""
    return np.array([math.remainder(-math.pi + 2.0 * math.pi * (k + 0.5) / n, 2.0 * math.pi)
                     for k in range(n)])


def _ray_hit(theta: float, polygon: np.ndarray) -> float:
    """Distance from the origin along bearing theta to a polygon's boundary, or inf."""
    d = np.array([math.cos(theta), math.sin(theta)])
    best = math.inf
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0), strict=True):
        e = b - a
        den = d[0] * (-e[1]) - d[1] * (-e[0])
        if abs(den) < 1e-15:
            continue
        # origin + t d = a + u e
        t = (a[0] * (-e[1]) - a[1] * (-e[0])) / den
        u = (d[0] * a[1] - d[1] * a[0]) / den
        if t > 0.0 and 0.0 <= u <= 1.0:
            best = min(best, t)
    return best


def ray_scan(polygons=(), n: int = BINS, range_max: float = RANGE_MAX,
             invalid=()) -> ScanStream:
    """A scan of body-frame polygons: the nearest hit per bin, range_max where nothing is hit
    (valid), and the listed bin indices marked invalid (hard rule 5)."""
    b = bearings(n)
    r = np.full(n, range_max)
    for poly in polygons:
        p = np.asarray(poly, dtype=float)
        r = np.minimum(r, [_ray_hit(t, p) for t in b])
    valid = np.ones(n, dtype=bool)
    valid[list(invalid)] = False
    return ScanStream.of(r, b, valid)


def box(x0: float, y0: float, x1: float, y1: float) -> np.ndarray:
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=float)


def make_input(*, stop: bool = False, velocity=(0.0, 0.0, 0.0), scan: ScanStream | None = None,
               lookahead=(0.8, 0.0), goal=(10.0, 0.0), dt: float = 0.05) -> ClassicalInput:
    look = np.asarray(lookahead, dtype=float)
    g = np.asarray(goal, dtype=float)
    route = RouteView.of(np.vstack([[0.0, 0.0], look, g]), look, g,
                         float(np.hypot(*look) + np.hypot(*(g - look))))
    return ClassicalInput.of(stop_active=stop, velocity=velocity,
                             scan=scan if scan is not None else ray_scan(), route=route, dt=dt)
