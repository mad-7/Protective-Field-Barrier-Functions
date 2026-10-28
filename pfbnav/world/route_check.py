"""Route-level traversability: does an ideal tracker on a planned route ever breach its field?
Pure functions.

Ruled 2026-10-13 (supervisor, item 3): the planner's lethal radius (`half_width + lethal_margin`,
0.2386 m) lies inside the at-rest field (`half_width + lateral_margin`, 0.2586 m) -- an
environment defect from the D1 ruling. New lethal and soft margins are chosen BY CRITERION:

  **an ideal tracker following every planned route exactly, at its stratum's traversal speed,
  with the field oriented along the path tangent, produces zero field breaches**, and no stratum
  loses plannability (grid quantisation included).

This module is the criterion's left half. The ideal tracker stands on the route with its body
yaw along the tangent, so the field it carries is the moving field of the stratum's band with
the travel direction straight ahead (`heading` 0 in the body frame). At a route vertex both
tangents are checked, the one arriving and the one leaving.

**The breach test is exact for the emulator's field.** The certified layer's field is the convex
set {p : p . n <= h(n)} over `FieldGeometry.normal_angles` (its boundary distances are computed
from exactly that grid), and every layout obstacle is a convex polygon. Two convex polygons are
disjoint exactly when one of their edge normals separates them, so the test checks the field's
grid normals and the obstacle's own edge normals. The larger separating gap over those axes is
reported as the pose's clearance to the field (negative = the depth of the deepest overlap along
the best axis): 0 is touching, which is not a breach -- the scan would have to read inside the
boundary.

Only the MAPPED world counts for the criterion: an unmapped obstacle is placed within 0.3 m of
the route by design (A13) so that the route meets it; that is an encounter, not a planning
defect.

**Information only** (asked for in the same ruling): the same check with the tracker displaced
50 mm toward the inside of the route's bends, the cross-track a real tracker cutting corners
would carry. `inside_offsets` gives each sample its direction.
"""
from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike


@dataclass(frozen=True)
class RouteSamples:
    """Poses along a route: x, y (m, layout frame), yaw (rad, the tangent), and for each the
    unit normal pointing to the inside of the nearest bend ahead (zeros on a straight route)."""

    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    inside: np.ndarray        # (N, 2)

    def offset(self, distance: float) -> RouteSamples:
        """The same samples displaced `distance` m toward the inside of the bends."""
        return RouteSamples(self.x + distance * self.inside[:, 0],
                            self.y + distance * self.inside[:, 1], self.yaw, self.inside)


def _segments(path: np.ndarray) -> np.ndarray:
    """The path's vertices with zero-length steps removed."""
    keep = [0]
    for i in range(1, len(path)):
        if np.hypot(*(path[i] - path[keep[-1]])) > 1e-12:
            keep.append(i)
    return path[keep]


def inside_normals(path: ArrayLike) -> np.ndarray:
    """For each segment, the unit normal toward the inside of the next bend the route makes
    (a left turn's inside is the left side); the last segments, with no bend ahead, take the
    inside of the bend just made; a route with no bend at all gets zeros."""
    p = _segments(np.asarray(path, dtype=float))
    d = np.diff(p, axis=0)
    d = d / np.linalg.norm(d, axis=1)[:, None]
    left = np.column_stack([-d[:, 1], d[:, 0]])
    turn = np.zeros(len(d))            # turn[i]: the bend between segment i and i + 1
    for i in range(len(d) - 1):
        c = d[i, 0] * d[i + 1, 1] - d[i, 1] * d[i + 1, 0]
        turn[i] = np.sign(c) if abs(c) > 1e-9 else 0.0
    sides = np.zeros(len(d))
    for i in range(len(d)):
        ahead = [t for t in turn[i:len(d) - 1] if t != 0.0]
        behind = [t for t in turn[:i][::-1] if t != 0.0]
        sides[i] = ahead[0] if ahead else (behind[0] if behind else 0.0)
    return left * sides[:, None]


def sample_route(path: ArrayLike, ds: float) -> RouteSamples:
    """Poses every `ds` m along the route, every vertex included twice (arriving and leaving
    tangents). The goal vertex is included with the last tangent."""
    p = _segments(np.asarray(path, dtype=float))
    if len(p) < 2:
        raise ValueError("a route needs two distinct vertices")
    normals = inside_normals(p)
    xs, ys, yaws, ins = [], [], [], []
    for i in range(len(p) - 1):
        a, b = p[i], p[i + 1]
        length = float(np.hypot(*(b - a)))
        n = max(1, int(np.ceil(length / ds)))
        t = np.linspace(0.0, 1.0, n + 1)          # both ends: the vertex is sampled per segment
        pts = a + t[:, None] * (b - a)
        xs.append(pts[:, 0])
        ys.append(pts[:, 1])
        yaws.append(np.full(len(t), np.arctan2(b[1] - a[1], b[0] - a[0])))
        ins.append(np.repeat(normals[i][None, :], len(t), axis=0))
    return RouteSamples(np.concatenate(xs), np.concatenate(ys), np.concatenate(yaws),
                        np.concatenate(ins))


@dataclass(frozen=True)
class FieldShape:
    """The moving field of one band, travel straight ahead, as its support on the emulator's
    grid of normals (body frame) and a function for its support at any other angle."""

    normal_angles: np.ndarray     # rad, body frame
    support: np.ndarray           # m, h(n) on that grid
    support_at: Callable[[np.ndarray], np.ndarray]   # angles (rad, body) -> h, m
    reach: float                  # m, the farthest boundary point from the body origin

    @classmethod
    def moving(cls, fields, band_top_speed: float, heading: float = 0.0,
               yaw_rate: float = 0.0) -> FieldShape:
        """From a `pfbnav.safety.certified.FieldGeometry`. `heading` is the travel direction in the
        body frame (rad): 0 when the body faces along the route, as the oracle assumes; a holonomic
        base may travel at any heading to its body. `yaw_rate` (rad/s) adds the rotation sweep
        (ruled 2026-10-16); 0 gives the rotation-blind shape exactly."""
        angles = np.asarray(fields.normal_angles, dtype=float)
        h = np.asarray(fields.support(angles, band_top_speed, heading, False,
                                      yaw_rate=yaw_rate), dtype=float)
        bearings = np.linspace(-np.pi, np.pi, 720, endpoint=False)
        reach = float(np.max(fields.extent(bearings, band_top_speed, heading, False,
                                           yaw_rate=yaw_rate)))
        return cls(angles, h, lambda a: np.asarray(
            fields.support(np.asarray(a, dtype=float), band_top_speed, heading, False,
                           yaw_rate=yaw_rate), dtype=float), reach)


def _field_axes(x: np.ndarray, y: np.ndarray, t: np.ndarray, v: np.ndarray,
                normals: np.ndarray, support: np.ndarray) -> np.ndarray:
    """Separating gap along the field's own normals (body-frame `normals`, world after rotating by
    each pose's yaw `t`): obstacle min minus field max, the best axis per pose.

    Poses sharing a yaw share the normals' world directions, so the obstacle's projection onto
    them is computed once per distinct yaw -- a route has a handful of yaws, and a state table one
    per call -- instead of once per pose."""
    out = np.empty(x.size)
    keys = np.round(t, 12)
    for k in np.unique(keys):
        rows = np.flatnonzero(keys == k)
        phi = t[rows[0]] + normals
        c, s = np.cos(phi), np.sin(phi)
        obs_min = np.min(np.outer(v[:, 0], c) + np.outer(v[:, 1], s), axis=0)      # (G,)
        field_max = np.outer(x[rows], c) + np.outer(y[rows], s) + support[None, :]  # (n, G)
        out[rows] = np.max(obs_min[None, :] - field_max, axis=1)
    return out


def _edge_axes(x: np.ndarray, y: np.ndarray, t: np.ndarray, m: np.ndarray, obs_max: np.ndarray,
               field: FieldShape) -> np.ndarray:
    """Separating gap along the obstacle's outward edge normals `m`: field min minus obstacle max,
    the field's support evaluated once per distinct yaw."""
    out = np.empty(x.size)
    keys = np.round(t, 12)
    back_world = np.arctan2(-m[:, 1], -m[:, 0])                     # -m, world frame
    for k in np.unique(keys):
        rows = np.flatnonzero(keys == k)
        h_back = field.support_at(back_world - t[rows[0]])           # (E,), body frame
        field_min = (np.outer(x[rows], m[:, 0]) + np.outer(y[rows], m[:, 1])
                     - h_back[None, :])
        out[rows] = np.max(field_min - obs_max[None, :], axis=1)
    return out


def clearance_to_field(samples: RouteSamples, obstacles: Sequence[ArrayLike],
                       field: FieldShape, coarse_stride: int = 1) -> np.ndarray:
    """For each pose, the separating gap between its field and the nearest obstacle, m.

    Negative means the obstacle is inside the field: a breach. +inf when no obstacle is near
    enough to matter (every obstacle farther than the field's reach from the pose).

    `coarse_stride` > 1 is a faster, still exact test of WHETHER a pose is breached: any axis that
    separates the field from an obstacle proves them disjoint, so the obstacle's edge normals and
    every `coarse_stride`-th field normal are tried first, and the full set of field normals only
    for the pose-obstacle pairs those leave unseparated. Breach depths (negative values) are then
    exact; a clear pose's value is a lower bound (>= 0) rather than its exact clearance.
    """
    px, py, th = samples.x, samples.y, samples.yaw
    best = np.full(len(px), np.inf)
    stride = max(1, int(coarse_stride))
    for polygon in obstacles:
        v = np.asarray(polygon, dtype=float)[:, :2]
        edges = np.roll(v, -1, axis=0) - v
        m = np.column_stack([edges[:, 1], -edges[:, 0]])
        m /= np.linalg.norm(m, axis=1)[:, None]
        area2 = float(np.sum(v[:, 0] * np.roll(v[:, 1], -1) - np.roll(v[:, 0], -1) * v[:, 1]))
        if area2 < 0:                                         # clockwise polygon: flip
            m = -m
        obs_max = np.max(v @ m.T, axis=0)                     # (E,) support along each normal
        # The largest signed distance to an edge line is a LOWER bound on the distance to a
        # convex polygon, so a pose beyond the field's reach by that bound cannot be breached.
        lower = np.max(px[:, None] * m[:, 0] + py[:, None] * m[:, 1] - obs_max[None, :], axis=1)
        near = np.flatnonzero(lower <= field.reach + 1e-9)
        if near.size == 0:
            continue
        x, y, t = px[near], py[near], th[near]
        gap = _edge_axes(x, y, t, m, obs_max, field)
        if stride > 1:
            gap = np.maximum(gap, _field_axes(x, y, t, v, field.normal_angles[::stride],
                                              field.support[::stride]))
            todo = np.flatnonzero(gap < 0)
        else:
            todo = np.arange(near.size)
        if todo.size:
            gap[todo] = np.maximum(gap[todo], _field_axes(x[todo], y[todo], t[todo], v,
                                                          field.normal_angles, field.support))
        best[near] = np.minimum(best[near], gap)
    return best


def location_classes(samples: RouteSamples, *, start_window: float, goal_window: float,
                     bend_window: float) -> np.ndarray:
    """Where each sample lies on its route, for reporting where breaches happen: 'start' (within
    `start_window` m of arc length from the start), 'goal' (within `goal_window` m of the goal),
    'bend' (within `bend_window` m of an interior vertex), else 'straight'. First match wins."""
    step = np.hypot(np.diff(samples.x), np.diff(samples.y))
    arc = np.concatenate([[0.0], np.cumsum(step)])
    to_goal = arc[-1] - arc
    # Interior vertices: where the tangent changes between consecutive samples.
    turn = np.flatnonzero(np.abs(np.diff(np.unwrap(samples.yaw))) > 1e-9)
    corners = np.column_stack([samples.x[turn], samples.y[turn]])
    if len(corners):
        to_bend = np.min(np.hypot(samples.x[:, None] - corners[None, :, 0],
                                  samples.y[:, None] - corners[None, :, 1]), axis=1)
    else:
        to_bend = np.full(arc.size, np.inf)
    return np.where(arc <= start_window, "start",
                    np.where(to_goal <= goal_window, "goal",
                             np.where(to_bend <= bend_window, "bend", "straight")))


# --- the oracle tracker (criterion amended 2026-10-15) -------------------------------------------
#
# The ideal tracker held at traversal speed was the wrong model: no controller takes a corner or a
# goal at top speed. The oracle follows the route exactly and travels, at each point, in the fastest
# band whose field (oriented along the tangent) is clear there, slowing in time under the
# deceleration limit. Fields are nested in speed (a faster band's field contains a slower band's,
# asserted by tests/test_certified_layer.py), so any speed up to the fastest clear band's top keeps
# the field the robot actually carries clear. Where even band 0 is breached the oracle must stop:
# that stretch is an UNAVOIDABLE stop, and the route is not traversable without one.

def arc_length(samples: RouteSamples) -> np.ndarray:
    """Arc length (m) at each sample from the route's start; a vertex's two samples share one."""
    step = np.hypot(np.diff(samples.x), np.diff(samples.y))
    return np.concatenate([[0.0], np.cumsum(step)])


def band_caps(clear: np.ndarray, band_tops: Sequence[float]) -> np.ndarray:
    """Per sample, the top speed of the fastest band whose field is clear (m/s); 0 where none is.

    `clear` is (n_bands, N) booleans, bands ascending. Nesting means a clear band implies every
    slower band clear; the fastest clear one is taken regardless.
    """
    clear = np.asarray(clear, dtype=bool)
    tops = np.asarray(band_tops, dtype=float)
    caps = np.zeros(clear.shape[1])
    for k in range(len(tops)):
        caps = np.where(clear[k], tops[k], caps)
    return caps


def oracle_speeds(arc: ArrayLike, caps: ArrayLike, decel: float, v_goal: float = 0.0
                  ) -> np.ndarray:
    """The fastest speed profile under per-sample caps and a deceleration limit, ending at
    `v_goal` (m/s): the backward pass v_i = min(cap_i, sqrt(v_{i+1}^2 + 2 decel ds_i)).
    No acceleration limit is applied (as ruled), so the profile -- and the mission time from it --
    is the best case."""
    s = np.asarray(arc, dtype=float)
    v = np.asarray(caps, dtype=float).copy()
    v[-1] = min(v[-1], v_goal)
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], float(np.sqrt(v[i + 1] ** 2 + 2.0 * decel * (s[i + 1] - s[i]))))
    return v


def stop_runs(breached: ArrayLike) -> int:
    """How many separate stretches of the route are breached: each forces one stop."""
    b = np.asarray(breached, dtype=bool)
    return int(np.sum(b[1:] & ~b[:-1]) + (1 if b.size and b[0] else 0))


def traverse_time(arc: ArrayLike, speeds: ArrayLike) -> float:
    """Time (s) to follow the profile: each step at constant acceleration between its end speeds,
    2 ds / (v_i + v_{i+1}). Infinite if the profile stops anywhere a step has length."""
    s = np.asarray(arc, dtype=float)
    v = np.asarray(speeds, dtype=float)
    ds = np.diff(s)
    vsum = v[:-1] + v[1:]
    moving = ds > 0
    if np.any(moving & (vsum <= 0)):
        return float("inf")
    return float(np.sum(2.0 * ds[moving] / vsum[moving]))
