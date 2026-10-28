"""The PFB's protective field, rho(psi; V, theta, w): WP2, ruled 2026-10-23. Pure functions.

**Independent of the certified layer** (hard rule 3): this module imports neither
`pfbnav.safety.certified` nor anything that does (`pfbnav.world.strata` does). It builds the SAME
physical field from the SAME configuration keys (rule 16; Q2), written separately, and
`tests/test_pfb_field.py` cross-checks the two -- a test may import both.

**The field** (the settled construction, A20 and A21). For travel at speed V in direction theta
(body frame) with yaw rate w, the support in a direction n, with c = cos(n - theta) and
lead = max(0, c), is

    max( S(n; +-(E(|w|) + E(0))) + max(D(V) lead, swerve(c, V)) + K(V, w)
           + max((margin - lateral_margin) lead, R) + lateral_margin,
         S(n; +-E(|w|)) + R + lateral_margin )

  * S(n; +-e): the footprint's support swept over headings in [-e, e];
  * E(w): the heading change reachable within the reach time T from yaw rate w, then braking;
  * D(V) = V T + V^2 / (2 a_b): travel over the reach time and braking;
  * swerve: the displacement an isotropic limiter allows off the travel direction;
  * K: the drift of a body that turns while it translates;
  * R: the at-rest reach (the creep allowed while still called at rest, then braking).

The second term is the at-rest field, the infimum of the moving fields. At rest the field is that
term alone. The boundary distance along a bearing is min over normals of h(n) / cos(psi - n) on the
same normal grid the certified layer uses (1440), so the two discretisations agree exactly.

**The rotation sweep is derived differently from the certified layer's.** The swept footprint is
the union of the hull rotated through [lo, hi]. Its support in n is the maximum over hull vertices
(radius r_v, angle a_v) of r_v * max over phi in [lo, hi] of cos(n - phi - a_v): that is r_v when
n - a_v lies in [lo, hi] (mod 2 pi), and otherwise the better end. It is evaluated over all vertices
directly; the certified layer uses supporting vertices and a range-maximum table. The answers must
agree; the methods share no code.

**V is any speed** (the continuous field, PFB-(a)), or a band's top (PFB-(b), the adopted one).
`band_of` picks the band from the reachable speed, as the certified layer does.
"""
from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import ArrayLike

N_NORMALS = 1440       # the support grid; the certified layer's, so the discretisations agree


@dataclass(frozen=True)
class FieldParams:
    """The physical constants of the field, read from the certified configuration keys (rule 16)."""

    reach_time: float        # s, T: the certified response time plus one control step
    a_brake: float           # m/s^2, the braking the fields assume
    a_max: float             # m/s^2, the limiter's acceleration bound
    alpha_max: float         # rad/s^2, the limiter's yaw-acceleration bound
    alpha_brake: float       # rad/s^2, the yaw braking assumed (= actuation.alpha_max)
    speed_cap: float         # m/s, the largest planar speed in the box and the filter's budget
    yaw_cap: float           # rad/s, min(w_max, budget / lever arm)
    margin: float            # m, ahead when moving
    lateral_margin: float    # m, abeam and behind when moving; all round at rest
    rest_speed: float        # m/s, the standstill EXIT threshold: the fastest speed still at rest
    rest_yaw: float          # rad/s, the rotational standstill EXIT threshold
    rest_enter: float        # m/s, the standstill ENTER threshold (hysteresis)
    rest_yaw_enter: float    # rad/s, the rotational standstill ENTER threshold
    band_tops: tuple[float, ...]   # m/s, ascending


def planar_speed_cap(vx_max: float, vy_max: float, budget: float) -> float:
    """The largest |(vx, vy)| with |vx| <= vx_max, |vy| <= vy_max and |vx| + |vy| <= budget, m/s.

    The feasible set is a convex polygon symmetric about both axes; the norm, being convex, peaks
    at one of its vertices in the first quadrant: the axis intercepts, or the corners where the
    budget line meets the box."""
    if min(vx_max, vy_max, budget) <= 0.0:
        raise ValueError("limits must be positive")
    candidates = [(min(vx_max, budget), 0.0), (0.0, min(vy_max, budget))]
    if vx_max + vy_max > budget:
        x_corner = max(budget - vy_max, 0.0)                # on the budget line at y = vy_max
        y_corner = max(budget - vx_max, 0.0)                # on the budget line at x = vx_max
        candidates += [(min(x_corner, vx_max), min(vy_max, budget)),
                       (min(vx_max, budget), min(y_corner, vy_max))]
    else:
        candidates.append((vx_max, vy_max))
    return max(math.hypot(x, y) for x, y in candidates)


def field_params_from_config(cfg: dict[str, Any]) -> FieldParams:
    """The SAME keys the certified layer reads (rule 16; WP2 Q2), read here independently."""
    c, act = cfg["certified"], cfg["actuation"]
    budget = float(act["wheel_budget"]["filter_fraction"]) * float(act["wheel_budget"]["v_rim"])
    lever = float(cfg["robot"]["half_wheelbase"]) + float(cfg["robot"]["half_track"])
    return FieldParams(
        reach_time=float(c["response_time"]) + float(cfg["sim"]["dt"]),
        a_brake=float(c["a_brake"]), a_max=float(act["a_max"]),
        alpha_max=float(act["alpha_max"]), alpha_brake=float(act["alpha_max"]),
        speed_cap=planar_speed_cap(float(act["v_max"]), float(act["vy_max"]), budget),
        yaw_cap=min(float(act["w_max"]), budget / lever),
        margin=float(c["margin"]), lateral_margin=float(c["lateral_margin"]),
        rest_speed=float(c["standstill_exit"]), rest_yaw=float(c["standstill_yaw_exit"]),
        rest_enter=float(c["standstill_enter"]), rest_yaw_enter=float(c["standstill_yaw_enter"]),
        band_tops=tuple(float(v) for v in c["band_top_speeds"]))


# ------------------------------------------------------------------------- scalar terms
def reachable(speed: float, p: FieldParams) -> float:
    """The planar speed reachable within T from `speed` (m/s): speed + a_max T, capped, never below
    the speed itself."""
    if speed < 0.0:
        raise ValueError("speed must be non-negative")
    return max(float(speed), min(float(speed) + p.a_max * p.reach_time, p.speed_cap))


def band_of(speed: float, p: FieldParams) -> int:
    """The band the REACHABLE speed selects: the first whose top covers it. Above the top band is an
    error: no field covers it."""
    v = reachable(speed, p)
    for i, top in enumerate(p.band_tops):
        if v <= top:
            return i
    raise ValueError(f"reachable speed {v} m/s exceeds the top band ({p.band_tops[-1]} m/s)")


def travel_depth(v: float, p: FieldParams) -> float:
    """D(V) = V T + V^2 / (2 a_b), m."""
    return v * p.reach_time + v * v / (2.0 * p.a_brake)


def heading_reach_rate(yaw_rate: float, p: FieldParams) -> float:
    """dE/d|w|, rad per rad/s: T + w_end / alpha_brake while the rate stays below the cap over T;
    the rise time once it reaches the cap within T (the braking term is then constant); 0 at or
    above the cap (the rate is clipped there)."""
    w = abs(float(yaw_rate))
    if w >= p.yaw_cap:
        return 0.0
    t_rise = (p.yaw_cap - w) / p.alpha_max
    if t_rise >= p.reach_time:
        return p.reach_time + (w + p.alpha_max * p.reach_time) / p.alpha_brake
    return t_rise


def heading_reach(yaw_rate: float, p: FieldParams) -> float:
    """E(w): the largest heading change, in the sense of rotation, reachable from |w| within T --
    accelerating at alpha_max up to yaw_cap -- and then braking at alpha_brake, rad."""
    w = min(abs(float(yaw_rate)), p.yaw_cap)
    t_rise = min(p.reach_time, (p.yaw_cap - w) / p.alpha_max)
    w_end = w + p.alpha_max * t_rise
    angle = w * t_rise + 0.5 * p.alpha_max * t_rise ** 2 + w_end * (p.reach_time - t_rise)
    return angle + w_end ** 2 / (2.0 * p.alpha_brake)


def rest_creep_reach(p: FieldParams) -> float:
    """R: from any speed still called at rest (<= the standstill exit), in any direction: the reach
    over T, then braking, m."""
    s = p.rest_speed
    v_end = min(s + p.a_max * p.reach_time, p.speed_cap)
    return s * p.reach_time + 0.5 * p.a_max * p.reach_time ** 2 + v_end ** 2 / (2.0 * p.a_brake)


def turning_drift(v: float, yaw_rate: float, p: FieldParams) -> float:
    """K: a translation that turns with the body departs from the straight one by at most its length
    (at most D(V)) times the chord of the heading change, 2 sin(E / 2), m."""
    return travel_depth(v, p) * 2.0 * math.sin(0.5 * min(heading_reach(yaw_rate, p), math.pi))


def swerve_slope(cos_nu: ArrayLike, v: float, p: FieldParams) -> np.ndarray:
    """d swerve / d c: (V - a T)+ T [c > 0] for the reach term, plus s (s + a T) / (2 a_b) at the
    maximising s of the braking quadratic (the envelope theorem; zero where that term is clipped
    at zero)."""
    c = np.asarray(cos_nu, dtype=float)
    da = min(p.a_max * p.reach_time, p.speed_cap)
    top = max(v - da, 0.0)
    vertex = np.zeros_like(c)
    neg = c < 0.0
    vertex[neg] = np.clip(-da * (1.0 + c[neg]) / (2.0 * c[neg]), 0.0, top)
    cands = np.stack([np.zeros_like(c), np.full_like(c, top), vertex])
    vals = (cands * c[None, :] + da) * (cands + da)
    arg = np.argmax(vals, axis=0)
    s_best = np.take_along_axis(cands, arg[None, :], axis=0)[0]
    best = vals.max(axis=0)
    return (top * (c > 0.0) * p.reach_time
            + np.where(best > 0.0, s_best * (s_best + da), 0.0) / (2.0 * p.a_brake))


def swerve_reach(cos_nu: ArrayLike, v: float, p: FieldParams) -> np.ndarray:
    """The displacement, along a direction at cos_nu = c to the travel, an isotropic limiter allows a
    robot in a band topped at V: over T, s c T + a T^2 / 2 (s <= V - a T); then braking along the new
    velocity, at most max over s of (s c + a T)(s + a T) / (2 a_b). That quadratic in s is maximised
    over [0, V - a T] at an end or, when c < 0, at its vertex s* = -a T (1 + c) / (2 c), m."""
    c = np.asarray(cos_nu, dtype=float)
    da = min(p.a_max * p.reach_time, p.speed_cap)
    top = max(v - da, 0.0)

    def quad(s):
        return (s * c + da) * (s + da)
    vertex = np.zeros_like(c)
    neg = c < 0.0
    vertex[neg] = np.clip(-da * (1.0 + c[neg]) / (2.0 * c[neg]), 0.0, top)
    best = np.maximum(np.maximum(quad(0.0), quad(top)), quad(vertex))
    return (top * np.maximum(c, 0.0) * p.reach_time + 0.5 * da * p.reach_time
            + np.maximum(best, 0.0) / (2.0 * p.a_brake))


# ------------------------------------------------------------------------- the footprint
class SweptFootprint:
    """The footprint's convex hull, in polar form, and its support swept over a heading interval.

    The swept support in a normal n is the maximum over hull vertices (radius r_v, angle a_v) of
    r_v * max over phi in [-e, e] of cos(n - phi - a_v) (the module docstring). A vertex whose angle
    lies within e of n contributes its radius; any other contributes at most its value at one END of
    the sweep, and over all vertices that is the hull's ordinary support at n - e or n + e. So

        S(n; e) = max( h(n - e), h(n + e), max{ r_v : a_v in [n - e, n + e] } ),

    each part answered fast: h(d) by its supporting vertex, found by binary search on the hull's
    edge-normal angles; the range maximum by a sparse table over the vertices in angle order.
    `support_all_vertices` is the same quantity by brute force, the reference the tests hold it to.
    """

    def __init__(self, extent: Callable[[np.ndarray], np.ndarray], n_boundary: int = 2880):
        from scipy.spatial import ConvexHull

        psi = np.linspace(-math.pi, math.pi, n_boundary, endpoint=False)
        e = np.asarray(extent(psi), dtype=float)
        pts = np.column_stack([e * np.cos(psi), e * np.sin(psi)])
        self.extent = extent                                   # e(psi), for the receding test
        self.boundary = pts                                    # the footprint's boundary samples
        hull = pts[ConvexHull(pts).vertices]                   # counter-clockwise
        ang = np.arctan2(hull[:, 1], hull[:, 0])
        k0 = int(np.argmin(ang))
        hull, ang = np.roll(hull, -k0, axis=0), np.roll(ang, -k0)
        self.radius = np.hypot(hull[:, 0], hull[:, 1])
        self.angle = np.unwrap(ang)                            # increasing, span < 2 pi
        self.outer_radius = float(self.radius.max())
        # The outward normal of the edge from vertex k to k + 1; vertex k + 1 supports every
        # direction from that normal to the next edge's.
        edge = np.roll(hull, -1, axis=0) - hull
        self._normal = np.unwrap(np.arctan2(-edge[:, 0], edge[:, 1]))
        # A sparse table of radius maxima over runs of 2^j vertices, on the doubled sequence so a
        # circular run is one contiguous run.
        level = [np.concatenate([self.radius, self.radius])]
        while (2 << (len(level) - 1)) <= level[0].size:
            prev, step = level[-1], 1 << (len(level) - 1)
            level.append(np.maximum(prev[:-step], prev[step:]))
        self._table = level
        self._angle2 = np.concatenate([self.angle, self.angle + 2.0 * math.pi])

    def _supporting(self, directions: np.ndarray) -> np.ndarray:
        b0 = self._normal[0]
        d = b0 + np.mod(directions - b0, 2.0 * math.pi)
        k = np.searchsorted(self._normal, d, side="right") - 1
        return (k + 1) % self.radius.size

    def plain_support(self, directions: np.ndarray) -> np.ndarray:
        """h(d): the hull's support, through each direction's supporting vertex."""
        v = self._supporting(directions)
        return self.radius[v] * np.cos(directions - self.angle[v])

    def support_and_slope(self, normals: ArrayLike, half_sweep: float
                          ) -> tuple[np.ndarray, np.ndarray]:
        """S(n; e) and dS/de. At the end n - e the support r_v cos(n - e - a_v) changes at
        r_v sin(n - e - a_v) per unit e; at n + e at -r_v sin(n + e - a_v); a vertex inside the sweep
        is a plateau (slope 0). The slope is the active branch's."""
        n = np.asarray(normals, dtype=float).ravel()
        e = float(half_sweep)
        if 2.0 * e >= 2.0 * math.pi:
            return np.full(n.shape, self.outer_radius), np.zeros(n.shape)
        va, vb = self._supporting(n - e), self._supporting(n + e)
        ha = self.radius[va] * np.cos(n - e - self.angle[va])
        hb = self.radius[vb] * np.cos(n + e - self.angle[vb])
        sa = self.radius[va] * np.sin(n - e - self.angle[va])
        sb = -self.radius[vb] * np.sin(n + e - self.angle[vb])
        val = np.maximum(ha, hb)
        slope = np.where(ha >= hb, sa, sb)
        if e > 0.0:
            inner = self._largest_radius_between(n - e, 2.0 * e)
            plateau = inner > val
            val = np.where(plateau, inner, val)
            slope = np.where(plateau, 0.0, slope)
        return val, slope

    def _largest_radius_between(self, lo: np.ndarray, width: float) -> np.ndarray:
        """max{ r_v : a_v in [lo, lo + width] } (circular), -inf where no vertex lies there."""
        a0 = self.angle[0]
        start = a0 + np.mod(lo - a0, 2.0 * math.pi)
        i = np.searchsorted(self._angle2, start, side="left")
        j = np.searchsorted(self._angle2, start + width, side="right")
        count = j - i
        out = np.full(lo.shape, -np.inf)
        has = count > 0
        if has.any():
            lv = np.floor(np.log2(count[has])).astype(int)
            ii, jj = i[has], j[has]
            vals = np.empty(lv.size)
            for level in np.unique(lv):
                sel = lv == level
                t = self._table[level]
                vals[sel] = np.maximum(t[ii[sel]], t[jj[sel] - (1 << level)])
            out[has] = vals
        return out

    def support(self, normals: ArrayLike, half_sweep: float) -> np.ndarray:
        """max over phi in [-half_sweep, half_sweep] of the rotated hull's support in each normal."""
        n = np.asarray(normals, dtype=float)
        e = float(half_sweep)
        if e < 0.0:
            raise ValueError("half_sweep must be non-negative")
        if 2.0 * e >= 2.0 * math.pi:
            return np.full(n.shape, self.outer_radius)
        flat = n.ravel()
        ends = np.maximum(self.plain_support(flat - e), self.plain_support(flat + e))
        if e == 0.0:
            return ends.reshape(n.shape)
        inner = self._largest_radius_between(flat - e, 2.0 * e)
        return np.maximum(ends, inner).reshape(n.shape)

    def support_all_vertices(self, normals: ArrayLike, half_sweep: float) -> np.ndarray:
        """The same quantity over every vertex by brute force: the tests' reference."""
        n = np.asarray(normals, dtype=float)
        e = float(half_sweep)
        if 2.0 * e >= 2.0 * math.pi:
            return np.full(n.shape, self.outer_radius)
        rel = n.reshape(-1, 1) - self.angle[None, :]
        inside = np.mod(rel + e, 2.0 * math.pi) <= 2.0 * e
        best = np.where(inside, 1.0, np.maximum(np.cos(rel - e), np.cos(rel + e)))
        return (best * self.radius[None, :]).max(axis=1).reshape(n.shape)


class PFBField:
    """rho(psi; V, theta, w) and its support, for one footprint and parameter set."""

    _CACHE_SIZE = 64      # fields kept by `support_and_partials`

    def __init__(self, extent: Callable[[np.ndarray], np.ndarray], params: FieldParams,
                 n_normals: int = N_NORMALS):
        self.params = params
        self.footprint = SweptFootprint(extent)
        self.normals = np.linspace(-math.pi, math.pi, n_normals, endpoint=False)
        self._cos, self._sin = np.cos(self.normals), np.sin(self.normals)
        nxt = np.roll(self.normals, -1)
        self._cos1, self._sin1 = np.cos(nxt), np.sin(nxt)
        self._det = self._cos * self._sin1 - self._sin * self._cos1
        self.brute_force_evaluations = 0   # supports whose boundary was not a valid polygon
        self._support_cache: dict[tuple[float, float, float, bool],
                                  tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        self._sweep_cache: dict[float, tuple[np.ndarray, np.ndarray]] = {}

    def rest_support(self, normals: ArrayLike, yaw_rate: float = 0.0) -> np.ndarray:
        p = self.params
        return (self.footprint.support(normals, heading_reach(yaw_rate, p))
                + rest_creep_reach(p) + p.lateral_margin)

    def support(self, normals: ArrayLike, v: float, heading: float, yaw_rate: float = 0.0,
                at_rest: bool = False) -> np.ndarray:
        """The field's support in each normal (rad, body frame), m. `v` is the speed the field is
        sized for: a band top (PFB-(b)) or the reachable speed itself (PFB-(a))."""
        n = np.asarray(normals, dtype=float)
        p = self.params
        rest = self.rest_support(n, yaw_rate)
        if at_rest:
            return rest
        c = np.cos(n - heading)
        lead = np.maximum(c, 0.0)
        sweep = heading_reach(yaw_rate, p) + heading_reach(0.0, p)
        moving = (self.footprint.support(n, sweep)
                  + np.maximum(travel_depth(v, p) * lead, swerve_reach(c, v, p))
                  + turning_drift(v, yaw_rate, p)
                  + np.maximum((p.margin - p.lateral_margin) * lead, rest_creep_reach(p))
                  + p.lateral_margin)
        return np.maximum(moving, rest)

    def support_and_partials(self, v: float, heading: float, yaw_rate: float,
                             at_rest: bool = False
                             ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """On the normal grid: h, dh/d(heading) and dh/d|w| (the field depends on |w| only). The
        active branch of every max is differentiated (the envelope theorem at the kinks).

        **Cached by its exact arguments** (2026-10-26, the per-step budget): one filter step asks for
        the same field many times -- the chord rows, the true-row checks, every regime -- and the
        answer is a pure function of (v, heading, |w|, at_rest). The cache holds the last
        `_CACHE_SIZE` fields and returns read-only arrays, so a caller cannot alter a cached field.
        """
        key = (float(v), float(heading), abs(float(yaw_rate)), bool(at_rest))
        hit = self._support_cache.get(key)
        if hit is not None:
            return hit
        out = self._support_and_partials(v, heading, yaw_rate, at_rest)
        for a in out:
            a.setflags(write=False)
        if self._CACHE_SIZE <= 0:
            return out
        if len(self._support_cache) >= self._CACHE_SIZE:
            self._support_cache.pop(next(iter(self._support_cache)))
        self._support_cache[key] = out
        return out

    def _sweep(self, half_sweep: float) -> tuple[np.ndarray, np.ndarray]:
        """`SweptFootprint.support_and_slope` on the normal grid, cached by its exact half-sweep
        (2026-10-26): the sweep depends on the yaw rate alone, so it repeats across headings, bands
        and envelope cases where the whole field does not. Read-only arrays."""
        key = float(half_sweep)
        hit = self._sweep_cache.get(key)
        if hit is None:
            hit = self.footprint.support_and_slope(self.normals, key)
            for a in hit:
                a.setflags(write=False)
            if self._CACHE_SIZE <= 0:
                return hit
            if len(self._sweep_cache) >= self._CACHE_SIZE:
                self._sweep_cache.pop(next(iter(self._sweep_cache)))
            self._sweep_cache[key] = hit
        return hit

    def _support_and_partials(self, v: float, heading: float, yaw_rate: float,
                              at_rest: bool = False
                              ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        p = self.params
        n = self.normals
        er = heading_reach(yaw_rate, p)
        de = heading_reach_rate(yaw_rate, p)
        s_r, ds_r = self._sweep(er)
        rest = s_r + rest_creep_reach(p) + p.lateral_margin
        d_rest_w = ds_r * de
        if at_rest:
            return rest, np.zeros_like(rest), d_rest_w
        c = np.cos(n - heading)
        dc = np.sin(n - heading)                                   # dc / d heading
        lead = np.maximum(c, 0.0)
        e_m = er + heading_reach(0.0, p)
        s_m, ds_m = self._sweep(e_m)
        depth = travel_depth(v, p)
        sw = swerve_reach(c, v, p)
        t1 = np.maximum(depth * lead, sw)
        d_t1 = np.where(depth * lead >= sw, depth * (c > 0.0), swerve_slope(c, v, p))
        r = rest_creep_reach(p)
        fwd = p.margin - p.lateral_margin
        t2 = np.maximum(fwd * lead, r)
        d_t2 = np.where(fwd * lead >= r, fwd * (c > 0.0), 0.0)
        # The turning drift K takes the reachable turn E(|w|) alone, as `turning_drift` and the
        # certified layer's `coupling` do -- not the sweep's E(|w|) + E(0). (Corrected 2026-10-26: the
        # partials path used e_m here, a field up to 6 mm larger than the certified one; found by
        # the rows' equivalence test, now pinned by tests/test_pfb_field.py.)
        k = depth * 2.0 * math.sin(0.5 * min(er, math.pi))
        dk = depth * math.cos(0.5 * er) * de if er < math.pi else 0.0
        moving = s_m + t1 + k + t2 + p.lateral_margin
        use_moving = moving >= rest
        h = np.where(use_moving, moving, rest)
        dh_heading = np.where(use_moving, (d_t1 + d_t2) * dc, 0.0)
        dh_w = np.where(use_moving, ds_m * de + dk, d_rest_w)
        return h, dh_heading, dh_w

    def extent_and_partials(self, bearings: ArrayLike, v: float, heading: float,
                            yaw_rate: float, at_rest: bool = False
                            ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """rho along arbitrary bearings, with d rho / d bearing, d heading and d |w|, through each
        bearing's active normal n*: rho = h(n*) / cos(psi - n*), so d rho / d psi =
        h(n*) sin(psi - n*) / cos^2(psi - n*) and d rho / dX = (dh/dX)(n*) / cos(psi - n*)."""
        b = np.asarray(bearings, dtype=float)
        h, dh_t, dh_w = self.support_and_partials(v, heading, yaw_rate, at_rest)
        k = self._active_normal(b, h)
        rel = b - self.normals[k]
        ck, sk = np.cos(rel), np.sin(rel)
        return h[k] / ck, h[k] * sk / ck ** 2, dh_t[k] / ck, dh_w[k] / ck

    def _active_normal(self, bearings: np.ndarray, h: np.ndarray) -> np.ndarray:
        """For each bearing, the normal n* minimising h(n) / cos(psi - n) over normals facing it.

        **Through the boundary polygon** (2026-10-24, for the per-step budget). The field is the
        intersection of the half-planes n . x <= h(n), and h is a support function (sums and maxima
        of support functions), so every line touches the set. Its boundary is then the polygon whose
        vertex k is where lines k and k+1 meet, and the ray at psi leaves through the edge on the
        line whose two vertices bracket psi in polar angle -- the minimiser, found by one sorted
        search instead of a bearings x normals minimum. Where the vertices are not a valid convex
        polygon about the origin (a line not touching, or angles out of order), the brute-force
        minimum is used and counted in `brute_force_evaluations`."""
        h1 = np.roll(h, -1)
        x = (h * self._sin1 - h1 * self._sin) / self._det
        y = (self._cos * h1 - self._cos1 * h) / self._det
        edge = (x - np.roll(x, 1)) * (-self._sin) + (y - np.roll(y, 1)) * self._cos
        ang = np.arctan2(y, x)
        j0 = int(np.argmin(ang))
        order = np.roll(np.arange(ang.size), -j0)
        sorted_ang = ang[order]
        if edge.min() < -1e-12 or np.any(np.diff(sorted_ang) < 0.0):
            self.brute_force_evaluations += 1
            return self._active_normal_brute(bearings, h)
        i = np.searchsorted(sorted_ang, bearings, side="left")
        return order[i % ang.size]

    def _active_normal_brute(self, bearings: np.ndarray, h: np.ndarray) -> np.ndarray:
        """The reference: the minimum over every facing normal."""
        cos = np.outer(np.cos(bearings), self._cos) + np.outer(np.sin(bearings), self._sin)
        ratio = np.full(cos.shape, np.inf)
        np.divide(h[None, :], cos, out=ratio, where=cos > 1e-9)
        return np.argmin(ratio, axis=1)

    def extent(self, bearings: ArrayLike, v: float, heading: float, yaw_rate: float = 0.0,
               at_rest: bool = False) -> np.ndarray:
        """rho along each bearing, m: min over normals facing it of h(n) / cos(psi - n)."""
        b = np.asarray(bearings, dtype=float)
        h = self.support(self.normals, v, heading, yaw_rate, at_rest)
        k = self._active_normal(b, h)
        return h[k] / np.cos(b - self.normals[k])

    def band_extent(self, bearings: ArrayLike, band: int, heading: float,
                    yaw_rate: float = 0.0) -> np.ndarray:
        """PFB-(b): the field of `band`, sized for its top speed (ruled 2026-10-23, Q1)."""
        return self.extent(bearings, self.params.band_tops[band], heading, yaw_rate)



def bands(p: FieldParams) -> Sequence[float]:
    return p.band_tops
