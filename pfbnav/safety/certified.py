"""Certified-layer emulator: discrete speed-banded protective fields.

Independent implementation. This module must not import pfbnav.safety.pfb.
It emulates the behaviour of an industrial safety scanner + safety controller:
field selection by speed band and direction, response latency, braking to a
latched stop, and a restart delay. It does not claim functional-safety compliance.
"""
from __future__ import annotations

import itertools
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import ArrayLike

# cos(angle) below which a point counts as abeam of, not ahead of, the intended
# motion in the restart condition: floating-point noise, nothing more.
RESTART_ABEAM_TOL = 1e-9

# e(psi) in metres for bearings in radians; takes and returns arrays (or floats).
FootprintFn = Callable[..., Any]


@dataclass(frozen=True)
class CertifiedParams:
    response_time: float       # s, scanner + controller response
    a_brake: float             # m/s^2, braking assumed by the fields
    restart_delay: float       # s, latched stop before restart
    n_field_sets: int
    margin: float              # m, AHEAD when moving, only. Behind and abeam a moving field
                               #   carries `lateral_margin` (trailing margin ruled 2026-09-28),
                               #   and at rest `lateral_margin` all round (the infimum)
    lateral_margin: float      # m, abeam of the travel direction, when moving
    standstill_enter: float    # m/s, estimated speed at or below which the robot comes to rest
    standstill_exit: float     # m/s, estimated speed above which a resting robot is moving
    band_top_speeds: tuple[float, ...]   # m/s, ascending; one per field set
    # The WARNING field, which a real scanner carries separately from its protective field
    # (supervisor 2026-09-26). Depth D_w = warning_scale * D(v) + warning_extra; at rest,
    # where there is no depth to scale, the margin is extended by `warning_extra` all round.
    # It must be strictly larger than the protective field or it can never warn.
    # No defaults: a field size that decides when the robot is warned may not be inherited
    # silently from a dataclass default. Every construction site states it.
    warning_scale: float           # x the protective stopping depth
    warning_extra: float           # m, the warning field's extension AHEAD (depth and margin)
    # m, its extension ABEAM, behind, and at rest (supervisor 2026-09-28, D2). Separate from
    # `warning_extra` because one number could not serve both: 0.05 abeam put a robot tracking
    # at the committed 50 mm bound in the narrowest aisle inside the warning field for 100 % of
    # its traverse, so the near-miss metric said nothing there. It must stay strictly below the
    # 29.1 mm gap between the protective field's abeam reach and that wall (asserted in
    # tests/test_d2_aisle_strata.py).
    warning_extra_abeam: float
    # rad/s^2, the yaw braking the fields assume (ruled 2026-10-16). It is the actuation's
    # `alpha_max`, which the acceleration limiter applies symmetrically -- the same bound on a
    # yaw-rate increase and decrease -- so a stop's yaw ramps down at exactly this rate and the
    # assumption holds by construction (`pfbnav/sim/robot.py::limit_acceleration`).
    alpha_brake: float
    # **Reachable motion (A20, ruled 2026-10-18).** Within the reach time the command still
    # passes and the limiter lets it grow, so every field contains what the body can reach
    # before braking acts: the band is chosen from v + a_max T, the rotation sweep is the
    # interval reachable at alpha_max, and the swerve an isotropic limiter allows is included.
    reach_time: float          # s, T = response_time + one control step
    a_max: float               # m/s^2, the limiter's acceleration bound (isotropic, translation)
    alpha_max: float           # rad/s^2, the limiter's yaw-acceleration bound
    speed_cap: float           # m/s, the largest planar speed in the velocity box and wheel budget
    yaw_cap: float             # rad/s, min(w_max, wheel budget / lever arm)
    # rad/s, the rotational standstill thresholds (ruled 2026-10-19: AT REST needs both the
    # estimate and the applied command below the standstill threshold, rotation included, so a
    # spin is a moving state): the translational thresholds over the footprint's outermost
    # radius, so at rest no body point moves faster than the translational threshold.
    standstill_yaw_enter: float
    standstill_yaw_exit: float

    @property
    def rest_speed(self) -> float:
        """m/s, the largest composed speed still called at rest: the standstill EXIT threshold
        (hysteresis). The at-rest field contains the reach from any speed up to it."""
        return self.standstill_exit

    def __post_init__(self) -> None:
        tops = list(self.band_top_speeds)
        if len(tops) != self.n_field_sets:
            raise ValueError(f"{self.n_field_sets} field sets but {len(tops)} band tops")
        if not tops or tops[0] <= 0.0 or any(b <= a for a, b in itertools.pairwise(tops)):
            raise ValueError("band top speeds must be positive and strictly ascending")
        if not 0.0 < self.lateral_margin <= self.margin:
            raise ValueError("need 0 < lateral_margin <= margin")
        if not 0.0 <= self.standstill_enter < self.standstill_exit < tops[0]:
            raise ValueError("need 0 <= standstill_enter < standstill_exit < first band top")
        if self.warning_scale < 1.0 or self.warning_extra < 0.0:
            raise ValueError("the warning field may not be smaller than the protective one")
        if self.alpha_brake <= 0.0:
            raise ValueError("alpha_brake must be positive")
        if self.warning_extra_abeam <= 0.0:
            raise ValueError(
                "warning_extra_abeam must be positive, or abeam the warning field equals the "
                "protective field and can never warn before it")
        if self.warning_extra < self.warning_extra_abeam:
            raise ValueError(
                "warning_extra (ahead) below warning_extra_abeam would make the warning field "
                "reach less far ahead of the robot than to its side")
        for name in ("reach_time", "a_max", "alpha_max", "speed_cap", "yaw_cap"):
            if not getattr(self, name) > 0.0:
                raise ValueError(f"{name} must be positive")
        if not 0.0 <= self.standstill_yaw_enter < self.standstill_yaw_exit:
            raise ValueError("need 0 <= standstill_yaw_enter < standstill_yaw_exit")
        if self.reach_time < self.response_time:
            raise ValueError("reach_time is the response time plus a control step")
        # Band coherence (A20): what the limiter can add inside T must be less than the
        # narrowest band, or a band could be skipped within one reach time.
        widths = [tops[0]] + [b - a for a, b in itertools.pairwise(tops)]
        if not self.a_max * self.reach_time < min(widths):
            raise ValueError(f"a_max T = {self.a_max * self.reach_time} m/s is not below the "
                             f"narrowest band width {min(widths)} m/s (A20 band coherence)")
        # The top band covers the largest reachable composed speed (A20).
        if self.speed_cap > tops[-1]:
            raise ValueError(f"the top band ({tops[-1]} m/s) does not cover the largest "
                             f"reachable speed ({self.speed_cap} m/s)")


def band_index(speed: float, band_top_speeds: Sequence[float]) -> int:
    """Index of the speed band that `speed` (m/s, >= 0) selects: the first whose top covers it.

    A speed above the last band's top is an error, not the last band: the fields do not
    cover it, and a certified layer must not pretend they do.
    """
    if speed < 0.0:
        raise ValueError(f"speed must be non-negative, got {speed}")
    for i, top in enumerate(band_top_speeds):
        if speed <= top:
            return i
    raise ValueError(f"speed {speed} m/s exceeds the top band ({band_top_speeds[-1]} m/s)")


def box_budget_speed_cap(vx_max: float, vy_max: float, budget: float) -> float:
    """The largest |(vx, vy)| with |vx| <= vx_max, |vy| <= vy_max and |vx| + |vy| <= budget
    (no rotation), m/s: the norm is convex, so it peaks at a vertex of that polygon."""
    if min(vx_max, vy_max, budget) <= 0.0:
        raise ValueError("limits must be positive")
    x, y = min(vx_max, budget), min(vy_max, budget)
    vertices = [(x, 0.0), (0.0, y), (x, min(y, budget - x)), (min(x, budget - y), y)]
    return max(math.hypot(a, b) for a, b in vertices if a >= 0.0 and b >= 0.0)


def reach_speed(speed: float, p: Any) -> float:
    """The planar speed reachable within T from the composed `speed` (m/s): speed + a_max T,
    capped by the box and the budget -- and never below the speed itself (A20)."""
    if speed < 0.0:
        raise ValueError(f"speed must be non-negative, got {speed}")
    s = float(speed)
    return max(s, min(s + p.a_max * p.reach_time, p.speed_cap))


def band_speed_limit(index: int, p: Any) -> float:
    """The highest composed speed that keeps the robot in band `index` under reach (m/s): the
    band's top less a_max T -- except the top band, which covers every reachable speed up to the
    cap. What a tracker may drive at while its field is that band's."""
    tops = p.band_top_speeds
    if index == len(tops) - 1:
        return float(p.speed_cap)
    return max(float(tops[index]) - p.a_max * p.reach_time, 0.0)


def rest_reach(p: Any) -> float:
    """How far the body can travel, in ANY direction, while the layer still calls it at rest, and
    then braking, m. The composed speed s is at most `rest_speed` and its direction is not trusted,
    so the bound is isotropic: s T + a T^2 / 2 over the reach, then braking from at most s + a T."""
    s = p.rest_speed
    v = min(s + p.a_max * p.reach_time, p.speed_cap)
    return s * p.reach_time + 0.5 * p.a_max * p.reach_time ** 2 + v * v / (2.0 * p.a_brake)


def band_depth(band_top_speed: float, p: Any) -> float:
    """D(V) = V T + V^2 / (2 a_b): travel ahead for a band topped at V -- over the reach time
    at most V (the band's choice guarantees it), then braking from V, m."""
    v = float(band_top_speed)
    return v * p.reach_time + v * v / (2.0 * p.a_brake)


def yaw_excursion(yaw_rate: float, p: Any) -> float:
    """The largest heading change in the POSITIVE sense reachable from `yaw_rate` (rad/s): the rate
    accelerates at alpha_max for T (capped at yaw_cap), then brakes at alpha_brake. Never negative:
    the starting heading is part of the sweep."""
    w0 = float(np.clip(yaw_rate, -p.yaw_cap, p.yaw_cap))
    T, a = p.reach_time, p.alpha_max
    if w0 + a * T <= p.yaw_cap:
        theta_T, w_T = w0 * T + 0.5 * a * T * T, w0 + a * T
    else:
        tc = (p.yaw_cap - w0) / a
        theta_T, w_T = w0 * tc + 0.5 * a * tc * tc + p.yaw_cap * (T - tc), p.yaw_cap
    braking = w_T * w_T / (2.0 * p.alpha_brake) if w_T > 0.0 else 0.0
    return max(0.0, theta_T + braking)


def yaw_interval(yaw_rate: float, p: Any) -> tuple[float, float]:
    """(-e, e), rad, e = the excursion reachable from |yaw_rate| (A20: the sweep is sized "from
    |omega| + alpha_max T"). Symmetric, so it does not depend on the sense of rotation and never
    shrinks as |omega| grows (the 2026-10-16 requirement) -- conservative on the side the body is
    not turning toward, where the reachable excursion is smaller."""
    e = yaw_excursion(abs(float(yaw_rate)), p)
    return -e, e


def coupling(band_top_speed: float, yaw_rate: float, p: Any) -> float:
    """The sideways drift of a body that TURNS while it translates, m (ruled necessary 2026-10-19
    for nesting; recorded under rule 7). A holonomic base's commanded velocity turns with the body,
    so the displacement integral of R(theta(t)) u(t) dt departs from the straight one by at most
    integral of |u| |R(theta) - I| dt <= L * 2 sin(Theta / 2): L the path length, at most the band
    depth D(V), and Theta the heading change, at most the reachable excursion e(|omega|).
    Isotropic and conservative."""
    theta = min(yaw_excursion(abs(float(yaw_rate)), p), math.pi)
    return band_depth(band_top_speed, p) * 2.0 * math.sin(0.5 * theta)


def swerve(cos_nu: ArrayLike, band_top_speed: float, p: Any) -> np.ndarray:
    """Support, m, of the displacement an isotropic limiter allows in a direction at cos_nu = c to
    the travel direction, for a band topped at V (A20, the FULL construction). With rho = a T, the
    current speed s in [0, b], b = (V - rho)+, and the velocity after T v + delta, |delta| <= rho:

      over the reach:  s c T + rho T / 2 <= b max(0, c) T + rho T / 2;
      braking along the new velocity:  (n . v_T)|v_T| / (2 a_b) <= g / (2 a_b),
        g = max over s in [0, b] of (s c + rho)(s + rho)

    -- the slowest robot is carried furthest BEHIND (an interior s for -1 < c < 0)."""
    v = float(band_top_speed)
    rho = min(p.a_max * p.reach_time, p.speed_cap)
    b = max(v - rho, 0.0)
    c = np.asarray(cos_nu, dtype=float)

    def g(speed):
        return (speed * c + rho) * (speed + rho)

    with np.errstate(divide="ignore", invalid="ignore"):
        s_star = np.where(c < 0.0, -rho * (1.0 + c) / (2.0 * c), b)
    s_star = np.clip(np.nan_to_num(s_star, nan=b), 0.0, b)
    peak = np.maximum.reduce([g(0.0) * np.ones_like(c), g(b), g(s_star)])
    return (b * np.maximum(0.0, c) * p.reach_time + 0.5 * rho * p.reach_time
            + np.maximum(0.0, peak) / (2.0 * p.a_brake))


class StandstillDetector:
    """At rest or moving, from the estimated speed, with hysteresis; transitions logged.

    A robot at rest leaves rest when its speed exceeds `standstill_exit`; a moving robot
    comes to rest when its speed falls to `standstill_enter` or below. The gap keeps a
    speed estimate that hovers near one threshold from flipping the field every step. The
    robot starts at rest, as it does at every episode start.
    """

    def __init__(self, params: CertifiedParams):
        self.enter = params.standstill_enter
        self.exit = params.standstill_exit
        self.yaw_enter = params.standstill_yaw_enter
        self.yaw_exit = params.standstill_yaw_exit
        self.reset()

    def reset(self) -> None:
        self.at_rest = True
        self.transitions: list[tuple[int, str]] = []
        self._step = 0

    def update(self, speed: float, yaw_rate: float = 0.0) -> bool:
        """Feed one step's composed speed (m/s) and yaw rate (rad/s); returns whether the robot
        is at rest. Since 2026-10-19 rest needs BOTH below their thresholds: a spin on the spot
        is a moving state (its stops must nest, S2)."""
        if speed < 0.0:
            raise ValueError(f"speed must be non-negative, got {speed}")
        w = abs(float(yaw_rate))
        if self.at_rest and (speed > self.exit or w > self.yaw_exit):
            self.at_rest = False
            self.transitions.append((self._step, "moving"))
        elif not self.at_rest and speed <= self.enter and w <= self.yaw_enter:
            self.at_rest = True
            self.transitions.append((self._step, "rest"))
        self._step += 1
        return self.at_rest


# ---------------------------------------------------------------------------------------
# Field geometry (pure). The shape descends from ruling 2 on the WP1 report and its correction --
# the Minkowski sum of the footprint with the braking segment along the travel direction u, offset
# by `margin` ahead and `lateral_margin` abeam and behind -- and since A20 (2026-10-18) contains the
# motion REACHABLE before braking acts. In support-function form, for a unit normal n,
#
#   moving:  h(n) = max( h_sweep(n) + max(D(V) max(0, n.u), swerve(n.u, V))
#                          + (margin - lateral_margin) max(0, n.u) + lateral_margin,
#                        h_rest(n) )
#   at rest: h_rest(n) = h_sweep_rest(n) + rest_reach + lateral_margin
#
# h_sweep: the footprint rotated through the yaw interval reachable within T; D(V) = V T + V^2/2a_b;
# swerve: the sideways and rearward displacement an isotropic limiter allows; rest_reach: the
# isotropic reach from the standstill speed. Travel direction u is the direction of the composed
# velocity, so rearward travel puts the depth behind the body. (An early form added depth radially
# at every bearing -- a shape error, not a property of the robot; supervisor, WP1 report.)
#
# The footprint is passed in as a function, so this module imports nothing from the rest
# of the package (and nothing from pfb.py, which it must never import).
# ---------------------------------------------------------------------------------------
class FieldGeometry:
    """Protective fields for one footprint and parameter set, via support functions.

    `n_normals` sets the angular resolution of the support grid used to turn support values
    into boundary distances; taking the minimum over a finite set of normals can only
    over-state a boundary distance, so the discretisation errs towards a larger field.

    **Reachable motion** (A20, ruled 2026-10-18; it replaced the constant-rate stopping sweep of
    2026-10-16, which replaced the rotation-blind field). The field contains everything the
    footprint can sweep from the stop trigger to rest: over the reach time T the command may keep
    accelerating within the limiter's bounds -- faster, turning harder, or swerving -- and then
    brakes. The body turns through the interval `yaw_interval(yaw_rate)`; the union of the
    footprint rotated through every such angle has support max over phi of h_fp(n - phi),
    computed exactly in closed form (`swept_interval`), continuous in the yaw rate. The
    translation reach (`band_depth`, `swerve`) and the margins are added to it (`support`). WP2's
    PFB adopts the same construction.
    """

    def __init__(self, footprint_extent: FootprintFn, params: CertifiedParams,
                 n_normals: int = 1440, n_boundary: int = 2880):
        self.params = params
        psi = np.linspace(-math.pi, math.pi, n_boundary, endpoint=False)
        e = np.asarray(footprint_extent(psi), dtype=float)
        self._footprint_extent = footprint_extent        # e(psi), for the restart condition
        self._boundary = np.column_stack([e * np.cos(psi), e * np.sin(psi)])
        self._prepare_hull()
        self.normal_angles = np.linspace(-math.pi, math.pi, n_normals, endpoint=False)
        self._h_fp = self.footprint_support(self.normal_angles)
        # 1 / cos(psi - n) for the bearing grids actually asked for, which in the step loop
        # are the two scan streams and never change. Rebuilding it cost most of what was
        # left of the 124 ms step (2026-09-23). Entries where the normal faces away carry
        # +inf, so the ratio there is +inf and loses the minimum, as the mask did.
        self._inv_cos: dict[bytes, np.ndarray] = {}
        self._inv_cos_max = 4

    def footprint_support(self, normal_angles: ArrayLike) -> np.ndarray:
        """h_fp(n) = max over the footprint boundary of p . n, m.

        The footprint does not change, so its support on the support grid is computed once
        in `__init__` and returned here rather than recomputed. Measured 2026-09-23: every
        call rebuilt a 2880 x 1440 product, and the emulator cost 124 ms p95 per step in
        `tests/test_step_loop_sim.py` -- more than the physics -- because `support` asks for
        exactly that grid twice a step. The returned array is a copy, so a caller cannot
        write through it into the cache.
        """
        a = np.asarray(normal_angles, dtype=float)
        cached = getattr(self, "_h_fp", None)          # absent while __init__ fills it
        if cached is not None and (a is self.normal_angles
                                   or (a.shape == self.normal_angles.shape
                                       and np.array_equal(a, self.normal_angles))):
            return cached.copy()
        n = np.column_stack([np.cos(a), np.sin(a)])
        return (self._boundary @ n.T).max(axis=0)

    def _prepare_hull(self) -> None:
        """The footprint's convex hull, for the exact rotation sweep: its vertices sorted by polar
        angle (the origin is inside, so counter-clockwise order is angle order), each vertex's
        radius and angle, the outward normal angles of its edges, and a sparse table of radii for
        range maxima over a circular run of vertices."""
        from scipy.spatial import ConvexHull

        hull = self._boundary[ConvexHull(self._boundary).vertices]      # counter-clockwise
        alpha = np.arctan2(hull[:, 1], hull[:, 0])
        first = int(np.argmin(alpha))
        hull, alpha = np.roll(hull, -first, axis=0), np.roll(alpha, -first)
        self._hull = hull
        self._hull_r = np.hypot(hull[:, 0], hull[:, 1])
        self._hull_alpha = np.unwrap(alpha)                               # increasing, < 2 pi span
        # Edge i runs from vertex i to i+1; its outward normal (e_y, -e_x). Vertex i+1 supports
        # every direction between the normals of edges i and i+1.
        edge = np.roll(hull, -1, axis=0) - hull
        beta = np.unwrap(np.arctan2(-edge[:, 0], edge[:, 1]))
        self._edge_beta = beta
        r2 = np.concatenate([self._hull_r, self._hull_r])
        table = [r2]
        while (1 << len(table)) <= r2.size:
            prev, half = table[-1], 1 << (len(table) - 1)
            table.append(np.maximum(prev[:-half], prev[half:]))
        self._hull_table = table

    def _hull_support(self, psi: np.ndarray) -> np.ndarray:
        """The footprint's support at angles `psi`, exactly, through each angle's supporting hull
        vertex: r_v cos(psi - alpha_v)."""
        beta0 = self._edge_beta[0]
        rel = beta0 + np.mod(psi - beta0, 2.0 * math.pi)
        k = np.searchsorted(self._edge_beta, rel, side="right") - 1        # edge k <= rel < k+1
        v = (k + 1) % self._hull_r.size
        return self._hull_r[v] * np.cos(psi - self._hull_alpha[v])

    def _hull_range_max(self, psi_lo: np.ndarray, span: float) -> np.ndarray:
        """The largest vertex radius whose angle lies in [psi_lo, psi_lo + span] (circular);
        -inf where none does."""
        a0 = self._hull_alpha[0]
        lo = a0 + np.mod(psi_lo - a0, 2.0 * math.pi)
        alpha2 = np.concatenate([self._hull_alpha, self._hull_alpha + 2.0 * math.pi])
        i0 = np.searchsorted(alpha2, lo, side="left")
        i1 = np.searchsorted(alpha2, lo + span, side="right")
        length = i1 - i0
        out = np.full(psi_lo.shape, -np.inf)
        ok = length > 0
        if ok.any():
            level = np.floor(np.log2(length[ok])).astype(int)
            out_ok = np.empty(level.size)
            for lv in np.unique(level):
                sel = level == lv
                t = self._hull_table[lv]
                a = i0[ok][sel]
                b = i1[ok][sel] - (1 << lv)
                out_ok[sel] = np.maximum(t[a], t[b])
            out[ok] = out_ok
        return out

    def swept_interval(self, normal_angles: ArrayLike, lo: float, hi: float) -> np.ndarray:
        """Support of the footprint rotated through every angle in [lo, hi] (rad, lo <= 0 <= hi),
        exactly (see `swept_support`). The A20 rotation sweep is such an interval."""
        a = np.asarray(normal_angles, dtype=float)
        if not lo <= 0.0 <= hi:
            raise ValueError("the interval must contain the starting heading")
        if hi - lo == 0.0:
            return self.footprint_support(a)
        if hi - lo >= 2.0 * math.pi:
            return np.full(a.shape, float(self._hull_r.max()))
        flat = a.ravel()
        ends = np.maximum(self._hull_support(flat - lo), self._hull_support(flat - hi))
        inner = self._hull_range_max(flat - hi, hi - lo)
        return np.maximum(ends, inner).reshape(a.shape)

    def swept_support(self, normal_angles: ArrayLike, sweep: float) -> np.ndarray:
        """Support of the footprint rotated through every angle between 0 and `sweep` (rad,
        signed): max over phi of h_fp(n - phi), EXACTLY. `sweep` = 0 returns `footprint_support`.

        For a convex polygon the support over an interval of directions is largest either at the
        interval's ends or at a vertex direction inside it -- a vertex direction is a local
        maximum, worth the vertex's radius, and the support's corners (edge normals) never are,
        since h + h'' >= 0 there. So the swept support is
        max(h(n - lo), h(n - hi), the largest vertex radius with its angle in [n - hi, n - lo]):
        exact, continuous in the sweep (a noise-level yaw rate moves the field by a noise-level
        amount), non-decreasing as the sweep grows, and cheap. (A first, sampled version padded
        every non-zero sweep by 0.74 mm, so a robot at rest with a yaw-rate estimate of 1e-5 rad/s
        saw its field jump; the recovery, judging at zero yaw rate, disagreed with the emulator
        and was not released -- the simulator control of 2026-10-16.)
        """
        a = np.asarray(normal_angles, dtype=float)
        if sweep == 0.0:
            return self.footprint_support(a)
        lo, hi = (0.0, float(sweep)) if sweep > 0.0 else (float(sweep), 0.0)
        span = hi - lo
        if span >= 2.0 * math.pi:
            return np.full(a.shape, float(self._hull_r.max()))
        flat = a.ravel()
        ends = np.maximum(self._hull_support(flat - lo), self._hull_support(flat - hi))
        inner = self._hull_range_max(flat - hi, span)
        return np.maximum(ends, inner).reshape(a.shape)

    def rest_support(self, normal_angles: ArrayLike, yaw_rate: float = 0.0) -> np.ndarray:
        """The at-rest field's support (A20): the footprint swept through every heading reachable
        from the composed yaw rate (a robot turning on the spot is at rest by speed; the interval
        never shrinks as |omega| grows), grown by the isotropic reach and the lateral margin."""
        lo, hi = yaw_interval(yaw_rate, self.params)
        return (self.swept_interval(normal_angles, lo, hi) + rest_reach(self.params)
                + self.params.lateral_margin)

    def support(self, normal_angles: ArrayLike, band_top_speed: float, heading: float,
                at_rest: bool, *, warning: bool = False, yaw_rate: float = 0.0) -> np.ndarray:
        """Support function of the selected field at the given normals, m (A20, ruled 2026-10-18).

        **Containment** is the property: everything the footprint sweeps from the stop trigger to
        rest lies inside the field in force at the trigger. With `nu = cos(n - heading)` and
        `lead = max(0, nu)`, a moving field in a band topped at V is

            max( h_sweep(n) + max(D(V) lead, swerve(nu, V)) + (margin - lateral_margin) lead
                   + lateral_margin,
                 the at-rest support at the same yaw rate )

        -- h_sweep the footprint rotated through the yaw interval reachable within T, D(V) the
        travel over T and braking, `swerve` the displacement an isotropic limiter allows sideways
        and behind. So it carries `margin` AHEAD and `lateral_margin` abeam and behind (trailing
        margin ruled 2026-09-28), and the at-rest field (`rest_support`) stays the infimum of the
        moving fields (2026-09-27). The present construction replaced the constant-rate one, which
        did not contain a command that kept accelerating through the response time (2026-10-18:
        the traced turns rotated up to twice the sweep, and stops ended inside the minimum margin).

        `warning=True` returns the band's WARNING field: depth `warning_scale * D + warning_extra`
        ahead, and an extension that is `warning_extra` ahead and `warning_extra_abeam` abeam,
        behind and at rest. It is strictly the larger set, so it is entered first.
        """
        a = np.asarray(normal_angles, dtype=float)
        p = self.params
        extra = p.warning_extra_abeam if warning else 0.0
        if at_rest:
            return self.rest_support(a, yaw_rate) + extra
        depth = band_depth(band_top_speed, p)
        c = np.cos(a - heading)
        lead = np.maximum(0.0, c)
        # NESTING (S2, ruled 2026-10-19): a stop that begins while moving must end with the
        # at-rest field -- footprint swept by +-e(0), grown by the at-rest reach and the lateral
        # margin -- inside the field in force at the trigger. So the moving field's yaw interval
        # carries e(0) on top of the reachable turn, and its margin term is at least the at-rest
        # reach in every direction (ahead the forward margin already exceeds it).
        e = yaw_excursion(abs(yaw_rate), p) + yaw_excursion(0.0, p)
        travel = np.maximum(depth * lead, swerve(c, band_top_speed, p))
        # And the drift of a body that turns while it translates (`coupling`), in every direction.
        support = np.maximum(
            self.swept_interval(a, -e, e) + travel + coupling(band_top_speed, yaw_rate, p)
            + np.maximum((p.margin - p.lateral_margin) * lead, rest_reach(p)) + p.lateral_margin,
            self.rest_support(a, yaw_rate))
        if warning:
            support = (support + ((p.warning_scale - 1.0) * depth + p.warning_extra) * lead
                       + extra + (p.warning_extra - extra) * lead)
        return support

    def extent(self, bearings: ArrayLike, band_top_speed: float, heading: float,
               at_rest: bool, *, warning: bool = False, yaw_rate: float = 0.0) -> np.ndarray:
        """Distance from the body origin to the field boundary along each bearing, m.

        For a convex set containing the origin, rho(psi) = min over normals n with
        n . u_psi > 0 of h(n) / (n . u_psi). `yaw_rate` (rad/s) adds the rotation sweep.
        """
        b = np.asarray(bearings, dtype=float)
        h = self.support(self.normal_angles, band_top_speed, heading, at_rest,
                         warning=warning, yaw_rate=yaw_rate)
        return (h[None, :] * self._inverse_cos(b)).min(axis=1)

    def _inverse_cos(self, bearings: np.ndarray) -> np.ndarray:
        """1 / cos(psi - n) for these bearings against the support grid, cached.

        Normals facing away from a bearing carry +inf, so multiplying by a (positive)
        support puts them out of the minimum -- the same answer the explicit mask gave.
        """
        key = bearings.tobytes()
        cached = self._inv_cos.get(key)
        if cached is None:
            cos = np.cos(bearings[:, None] - self.normal_angles[None, :])
            with np.errstate(divide="ignore"):
                cached = np.where(cos > 1e-9, 1.0 / cos, np.inf)
            if len(self._inv_cos) >= self._inv_cos_max:
                self._inv_cos.clear()      # bounded: a few MB, not an unbounded leak
            self._inv_cos[key] = cached
        return cached

    def lateral_reach(self, band_top_speed: float, heading: float, at_rest: bool, *,
                      yaw_rate: float = 0.0) -> float:
        """How far the field reaches to either side of the travel direction, m (exact).

        A wall parallel to the travel direction at distance w breaches the field exactly
        when w is below the support in the wall's normal direction.
        """
        sides = np.array([heading + math.pi / 2, heading - math.pi / 2])
        return float(self.support(sides, band_top_speed, heading, at_rest,
                                  yaw_rate=yaw_rate).max())

    def band_for(self, speed: float) -> int:
        """The band a composed speed selects: from the speed reachable within T (A20)."""
        return band_index(reach_speed(speed, self.params), self.params.band_top_speeds)

    def select(self, bearings: ArrayLike, speed: float, heading: float,
               at_rest: bool, *, yaw_rate: float = 0.0) -> np.ndarray:
        """The field the certified layer carries: at rest, or the band the REACHABLE speed
        selects (A20), with the yaw interval reachable from the yaw rate (rad/s)."""
        if at_rest:
            return self.extent(bearings, 0.0, heading, True, yaw_rate=yaw_rate)
        top = self.params.band_top_speeds[self.band_for(speed)]
        return self.extent(bearings, top, heading, False, yaw_rate=yaw_rate)


def field_breached(ranges: ArrayLike, valid: ArrayLike, field: ArrayLike) -> bool:
    """Whether any valid range falls inside the field (the boundary at each bearing).

    An invalid bin is unobserved space, and unobserved space is never treated as free
    (hard rule 5): it breaches the field.
    """
    r = np.asarray(ranges, dtype=float)
    ok = np.asarray(valid, dtype=bool)
    f = np.asarray(field, dtype=float)
    if not (r.shape == ok.shape == f.shape):
        raise ValueError("ranges, valid and field must have the same shape")
    if not ok.all():
        return True
    return bool(np.any(r < f))


class CertifiedLayer:
    """The emulator itself: field selection, response latency, latched stop, restart delay.

    One instance per robot, stepped once per physics step after sensing. It owns no
    simulator handles and no geometry beyond the footprint it is given, so it is exercised
    entirely on synthetic scans.

    **The sequence it emulates** (implemented WP1, 2026-09-23):

      1. the speed band is chosen from the *estimated* velocity, and with it the field;
      2. a valid range inside that field, or any unobserved bin (hard rule 5), breaches it;
      3. a breach **triggers** a stop; the stop becomes **active** `response_time` later,
         which is the scanner-plus-controller latency the field is sized to cover -- the
         robot keeps its command during that window, exactly as a real one does;
      4. an active stop is **latched**: it holds while the field is breached, and for
         `restart_delay` after the field last came clear;
      5. only then does the robot run again.

    **The receding release** (A24, ruled 2026-10-21). A creep stop ends with the robot a few mm
    closer to what stopped it, so the intrusion lies INSIDE the at-rest field -- the smallest field
    there is -- and under the whole-field restart (A22) no twist could ever be released: a
    permanent deadlock. So a robot held at rest with its at-rest field breached is released, after
    `restart_delay`, for a CREEP twist under which every point inside the at-rest field strictly
    recedes -- the centre rule and the closest-point rule, both strict, and every footprint part
    within the at-rest field's thickness of the point receding too, so a rotation swinging any part
    toward it is blocked. An unobserved bin still blocks. The points inside the field when it
    released are then EXEMPT from triggering -- only those (tracked bin to bin), only while the
    command stays receding and at creep speed; any other breach, a non-receding command or a speed
    above creep ends the exemption and triggers normally, and it ends by itself when those points
    have left the at-rest field. Its industrial counterpart is a reset and a reduced-speed
    manoeuvring mode; this one releases automatically (`docs/realism_audit.md`). Without a creep
    stop nothing here engages, and the layer is bit-identical to A22's.

    A stop already triggered is not re-triggered by the same breach, so counting
    `stop_triggered` counts protective stops -- the primary metric, per 100 m.

    `field_id` is the index of the field set in force: the speed band, or 0 at rest, whose
    field is the footprint grown by `margin` all round.

    `warning` reports the band's own WARNING field being breached while its protective field
    is not: a real scanner carries a separate, larger warning field, and until 2026-09-26
    this emulator approximated one with "the field of the band above", which warns not at all
    in the top band and not at all at rest. The warning field is now configured
    (`warning_scale`, `warning_extra`) and exists in every band. It is reported and never
    acts. `warning_entries` counts RISING EDGES -- how many times the robot entered the
    warning field -- which is the secondary near-miss metric (supervisor 2026-09-26); the
    step-by-step flag is not a count and must not be summed into one.
    """

    def __init__(self, params: CertifiedParams, half_length: float, half_width: float,
                 footprint_extent: FootprintFn | None = None):
        self.params = params
        self.half_length = half_length
        self.half_width = half_width
        # Without a measured footprint the emulator falls back to the configured rectangle's
        # circumscribed radius: conservative, and never silently a different robot.
        if footprint_extent is None:
            radius = math.hypot(half_length, half_width)
            footprint_extent = lambda psi: np.full_like(
                np.asarray(psi, dtype=float), radius)
        self.fields = FieldGeometry(footprint_extent, params)
        self.standstill = StandstillDetector(params)
        self.reset()

    def reset(self) -> None:
        """Back to the start of an episode: no stop, at rest, nothing counted."""
        self.standstill.reset()
        self.stop_active = False
        self.stops = 0
        self.warning_active = False
        self._held_since = None
        self._recede = None
        self.receding_releases = 0
        self.warning_entries = 0
        # Stops triggered AT REST (ruled 2026-10-19): in a static world a stationary robot with a
        # clear at-rest field cannot be triggered by static obstacles, so every one is reported as
        # a construction gap (expected zero in every static-world run).
        self.at_rest_triggers = 0
        # ... of which STATIONARY: the composed speed and yaw rate at or below the standstill ENTER
        # thresholds. Only these are the construction gap; a trigger while CREEPING (at rest by
        # classification, moving below the exit thresholds) is an at-rest stop by ruling, reported
        # with its outcome, not engineered for (2026-10-19).
        self.stationary_triggers = 0
        self._triggered_at: float | None = None
        self._clear_since: float | None = None
        self._heading = 0.0
        # The receding release (A24): when the held stop began, and the bins exempt while receding.
        self._held_since: float | None = None
        self._recede: np.ndarray | None = None
        self.receding_releases = 0

    def step(self, t: float, velocity, ranges, bearings, valid,
             intended_direction=None, applied=None) -> dict:
        """Advance the emulator one physics step, after sensing. Returns its state.

        `intended_direction` is the body-frame direction the controller is ASKING to travel
        in -- the requested command, not the achieved velocity. It is used only by the
        restart condition below; a stop is detected without it.

        Returns {'stop_active', 'stop_triggered', 'field_id', 'warning', 'breached', 'at_rest',
        'speed', 'field_speed', 'field_yaw_rate', 'reach_speed', 'yaw_interval',
        'restart_blocked'}.

        **The field is never sized below true motion** (ruled 2026-10-16). `velocity` is the
        *estimated actual* body velocity (vx, vy, wz) -- never wheel odometry, which hides
        ground slip (ruling 3 on the D7/G1b report) -- and `applied` the command the base is
        executing, after the limiter (vx, vy, wz). The band is chosen from
        max(estimated speed, applied speed), and the rotation sweep from the larger of
        |estimated yaw rate| and |applied yaw rate|. The command alone understates motion while
        decelerating (the robot moves faster than it is told); the low-pass estimate alone
        understates it while accelerating (it lags), which let a robot drive at band-1 speed
        under the band-0 field (found in emulation, 2026-10-16). The maximum is conservative in
        both. `speed` reports the estimate; `field_speed` and `field_yaw_rate` what sized the
        field. Without `applied` the estimate alone sizes it, as before. WP2's PFB uses the
        same composition.
        """
        v = np.asarray(velocity, dtype=float).ravel()
        if v.size < 2:
            raise ValueError("velocity must be at least (vx, vy)")
        estimate = float(math.hypot(v[0], v[1]))
        est_yaw = float(v[2]) if v.size > 2 else 0.0      # a 2-component velocity has none
        if applied is None:
            a = np.zeros(3)
        else:
            a = np.asarray(applied, dtype=float).ravel()
            if a.size != 3:
                raise ValueError("applied must be (vx, vy, wz)")
        commanded = float(math.hypot(a[0], a[1]))
        speed = max(estimate, commanded)                   # never below true motion
        yaw_rate = est_yaw if abs(est_yaw) >= abs(float(a[2])) else float(a[2])
        at_rest = self.standstill.update(speed, yaw_rate)
        # The field faces the travel direction of whichever signal dominates; below the
        # standstill threshold neither carries a reliable direction, so the last is kept.
        if speed > self.params.standstill_enter:
            src = v if estimate >= commanded else a
            self._heading = float(math.atan2(src[1], src[0]))
        field_id = 0 if at_rest else self.fields.band_for(speed)
        field = self.fields.select(bearings, speed, self._heading, at_rest=at_rest,
                                   yaw_rate=yaw_rate)
        breached = field_breached(ranges, valid, field)
        inside = None
        receding = False
        if self._recede is not None:
            # The exemption (A24): only while still at creep (so the at-rest field is the one in
            # force), only the bins inside it when it released (tracked to their neighbours), and
            # only while the command recedes from every one of them. Anything else ends it here, and
            # the breach -- if any -- triggers below as any other would.
            inside = ~np.asarray(valid, dtype=bool) | (np.asarray(ranges, dtype=float) < field)
            tracked = self._recede | np.roll(self._recede, 1) | np.roll(self._recede, -1)
            if (at_rest and bool(np.all(np.asarray(valid, dtype=bool)[inside]))
                    and not bool((inside & ~tracked).any())
                    and (not inside.any() or self._recedes(intended_direction, ranges, bearings,
                                                           valid, field, inside))):
                if inside.any():
                    self._recede = inside
                    receding = True
                    breached = False
                else:
                    self._recede = None            # the points have left the at-rest field
            else:
                self._recede = None

        triggered = False
        if breached:
            self._clear_since = None
            if self._triggered_at is None:
                self._triggered_at = t
                triggered = True
                self.stops += 1
                self.at_rest_triggers += int(at_rest)
                self.stationary_triggers += int(
                    at_rest and speed <= self.params.standstill_enter
                    and abs(yaw_rate) <= self.params.standstill_yaw_enter)
        elif self._triggered_at is not None and self._clear_since is None:
            self._clear_since = t

        if self._triggered_at is not None and t >= self._triggered_at + self.params.response_time:
            self.stop_active = True
        # **The restart condition** (supervisor 2026-09-27, ruling 3). Clearing the field the
        # robot is standing in is not enough to move: the band-0 field ALONG THE INTENDED
        # TRAVEL DIRECTION must also be clear. A stopped robot may be approached to the
        # minimum margin -- that is what makes the at-rest field the infimum of the moving
        # fields safe -- but it must never drive toward an intrusion. Without this, shrinking
        # the at-rest field would have bought the robot the right to set off at something
        # standing 0.05 m away.
        # The latch releases only while the field the robot STANDS in is clear (`not breached`)
        # as well as when `_restart_allowed` holds. That is stricter than the 2026-09-28 ruling's
        # words, deliberately: with something inside the minimum margin behind the intended
        # motion, the ruled condition alone would release the robot, and on the very next step
        # -- still at rest -- the breach would fire a new stop. It would be released and
        # re-stopped every `restart_delay`, each time counted as a protective stop. Closer than
        # the minimum margin is a real breach; recovery from it is supervised (ruling 4).
        restart_blocked = False
        if self.stop_active and not breached:
            restart_blocked = not self._restart_allowed(
                ranges, bearings, valid, intended_direction)
        if (self.stop_active and not breached and not restart_blocked
                and self._clear_since is not None
                and t >= self._clear_since + self.params.restart_delay):
            self.stop_active = False
            self._triggered_at = None
            self._clear_since = None
        # Held at rest with the at-rest field breached: only the receding release can free it (A24).
        if self.stop_active and breached and at_rest:
            if self._held_since is None:
                self._held_since = t
            inside = ~np.asarray(valid, dtype=bool) | (np.asarray(ranges, dtype=float) < field)
            release = (t >= self._held_since + self.params.restart_delay
                       and self._recedes(intended_direction, ranges, bearings, valid, field,
                                         inside))
            restart_blocked = not release
            if release:
                self.stop_active = False
                self._triggered_at = None
                self._clear_since = None
                self._held_since = None
                self._recede = inside
                self.receding_releases += 1
                receding = True
        else:
            self._held_since = None

        # The band's own warning field, which is strictly larger than its protective field,
        # so a breach of the protective field is always also a warning. Reported as a
        # warning only while the stop is not yet warranted, which is what "warning" means.
        band_top = self.params.band_top_speeds[field_id]
        in_warning = field_breached(ranges, valid, self.fields.extent(
            bearings, band_top, self._heading, at_rest, warning=True, yaw_rate=yaw_rate))
        warning = bool(in_warning and not breached)
        entered = bool(in_warning and not self.warning_active)
        if entered:
            self.warning_entries += 1
        self.warning_active = bool(in_warning)
        return {"stop_active": self.stop_active, "stop_triggered": triggered,
                "field_id": int(field_id), "warning": warning,
                "warning_entered": entered, "warning_entries": self.warning_entries,
                "breached": bool(breached), "at_rest": bool(at_rest), "speed": estimate,
                "field_speed": speed, "field_yaw_rate": yaw_rate,
                "reach_speed": reach_speed(speed, self.params),
                "triggered_at_rest": bool(triggered and at_rest),
                "triggered_stationary": bool(
                    triggered and at_rest and speed <= self.params.standstill_enter
                    and abs(yaw_rate) <= self.params.standstill_yaw_enter),
                "at_rest_triggers": self.at_rest_triggers,
                "stationary_triggers": self.stationary_triggers,
                "yaw_interval": yaw_interval(yaw_rate, self.params),
                "restart_blocked": bool(restart_blocked),
                "receding": bool(receding), "receding_releases": self.receding_releases}

    def _restart_allowed(self, ranges, bearings, valid, intended_direction) -> bool:
        """The restart condition (ruled 2026-10-19). `intended_direction` is the commanded twist
        (vx, vy, wz), or (vx, vy) for a translation. A restart is permitted only if the WHOLE field
        the commanded twist will select is clear:

          * at or below both standstill EXIT thresholds (translational and rotational) the robot
            stays at rest, so the twist keeps the AT-REST field -- the creep regime;
          * faster, the band its reachable speed selects, along its translation (a spin on the
            spot: the field's present heading), with its yaw interval.

        Containment plus nesting leave a departure shell in every direction: a nested stop leaves the
        obstacle just outside the at-rest field, inside every moving field. A departure whose own
        field reaches it would re-trigger at once, so it is BLOCKED rather than released into a
        release-retrigger loop. The creep regime leaves the shell: the classical recovery's creep is
        at the standstill speed, and a learned policy can learn the same. This mirrors industrial
        manoeuvring field sets. The two approach rules of 2026-10-19 (b) -- the centre rule
        v . p <= 0, and the closest-point rule (v + w x q) . (p - q) <= 0 -- are kept after the
        whole-field test; with the whole field clear they cannot bind. Their sense-dependence (a spin
        swinging a corner away permitted) is superseded wherever the intrusion lies inside the
        spin's own, sense-symmetric, field: spinning, that field is in force and would re-trigger.
        An unobserved bin breaches (hard rule 5).

        Stopping is unaffected: a protective stop while moving uses the FULL field. With no
        twist, or a zero one, the robot is not asking to move and nothing is blocked.
        """
        if intended_direction is None:
            return True
        twist = np.asarray(intended_direction, dtype=float).ravel()
        if twist.size < 2:
            return True
        vx, vy = float(twist[0]), float(twist[1])
        w = float(twist[2]) if twist.size > 2 else 0.0
        speed = float(math.hypot(vx, vy))
        if speed < 1e-9 and abs(w) < 1e-9:
            return True
        b = np.asarray(bearings, dtype=float)
        r = np.asarray(ranges, dtype=float)
        ok = np.asarray(valid, dtype=bool)
        # The WHOLE field the commanded twist will select (ruled 2026-10-19). Containment plus
        # nesting leave a departure shell in every direction; a departure whose own field
        # reaches into it would re-trigger at once, so it is blocked instead of released into a
        # loop. A twist at or below both standstill EXIT thresholds keeps the robot at rest, so
        # it keeps the at-rest field -- the creep regime that leaves the shell (the recovery's
        # creep; a learned policy's own). Anything faster selects the band its reachable speed
        # selects, along its translation (a spin on the spot: the field's present heading).
        p = self.params
        if speed <= p.standstill_exit and abs(w) <= p.standstill_yaw_exit:
            field = self.fields.extent(b, 0.0, 0.0, at_rest=True, yaw_rate=w)
        else:
            heading = float(math.atan2(vy, vx)) if speed >= 1e-9 else self._heading
            top = p.band_top_speeds[self.fields.band_for(speed)]
            field = self.fields.extent(b, top, heading, at_rest=False, yaw_rate=w)
        breaching = ~ok | (r < field)
        # Both approach rules are kept, as ruled, after the whole-field test: with the whole
        # selected field required clear no point breaches it, so they cannot bind; they stay as
        # the second line they were in 2026-09-28, should the field ever be narrowed.
        return (not bool(breaching.any())
                and self._approach_rules_permit(vx, vy, w, b, r, ok, field, breaching))

    def _approach_rules_permit(self, vx: float, vy: float, w: float, b: np.ndarray, r: np.ndarray,
                               ok: np.ndarray, field: np.ndarray, breaching: np.ndarray) -> bool:
        """The two approach rules (ruled 2026-10-19, (b)) over the breaching points: the centre
        rule and the closest-point rigid-body rule. Kept for the restart condition's second line;
        with the whole selected field required clear it is not reached."""
        speed = float(math.hypot(vx, vy))
        bb, seen = b[breaching], ok[breaching]
        u = np.column_stack([np.cos(bb), np.sin(bb)])
        # (i) The translation rule of 2026-09-28: the commanded translation has no component
        # toward any breaching point, along its bearing from the body origin (an unobserved bin:
        # its bearing). Vacuous for a spin on the spot.
        if speed >= 1e-9 and np.any(u @ np.array([vx, vy]) > RESTART_ABEAM_TOL * speed):
            return False
        # (ii) Rigid body: the velocity of the closest footprint point q, v + w x q, has no
        # component toward p, measured from q. An unobserved bin's range is unknown, so its
        # bearing is tested from just outside the footprint out to the field boundary.
        rng = r[breaching]
        edge = np.asarray(self.fields._footprint_extent(bb), dtype=float)
        far = field[breaching]
        samples = [np.where(seen, rng, edge + 1e-3)]
        samples += [np.where(seen, rng, edge + f * (far - edge)) for f in (0.5, 1.0)]
        for reach in samples:
            pts = reach[:, None] * u
            q = self._closest_footprint_points(pts)
            vq = np.column_stack([vx - w * q[:, 1], vy + w * q[:, 0]])
            gap = pts - q
            approach = np.sum(vq * gap, axis=1)
            scale = np.hypot(vq[:, 0], vq[:, 1]) * np.hypot(gap[:, 0], gap[:, 1])
            if np.any(approach > RESTART_ABEAM_TOL * scale):
                return False
        return True

    def _recedes(self, intended_direction, ranges, bearings, valid, field: np.ndarray,
                 inside: np.ndarray) -> bool:
        """Whether the commanded twist is a CREEP under which every point inside the at-rest field
        strictly recedes (A24). Strict centre rule: the translation has a component AWAY from every
        point (so a spin on the spot never qualifies). Strict rigid-body rule: every footprint
        boundary point within the at-rest field's thickness of the point -- the closest included --
        moves away from it, so a rotation swinging any part toward it is blocked. An unobserved bin
        inside blocks."""
        if intended_direction is None:
            return False
        twist = np.asarray(intended_direction, dtype=float).ravel()
        if twist.size < 2:
            return False
        vx, vy = float(twist[0]), float(twist[1])
        w = float(twist[2]) if twist.size > 2 else 0.0
        speed = float(math.hypot(vx, vy))
        p = self.params
        if speed < 1e-9 or speed > p.standstill_exit or abs(w) > p.standstill_yaw_exit:
            return False
        ok = np.asarray(valid, dtype=bool)
        if not bool(inside.any()) or not bool(np.all(ok[inside])):
            return False
        b = np.asarray(bearings, dtype=float)[inside]
        u = np.column_stack([np.cos(b), np.sin(b)])
        if np.any(u @ np.array([vx, vy]) >= -RESTART_ABEAM_TOL * speed):
            return False
        pts = np.asarray(ranges, dtype=float)[inside][:, None] * u
        boundary = self.fields._boundary
        # The at-rest field's thickness beyond the footprint along each point's bearing.
        thickness = np.asarray(field, dtype=float)[inside] - np.asarray(
            self.fields._footprint_extent(b), dtype=float)
        for point, reach in zip(pts, thickness, strict=True):
            gap = point[None, :] - boundary
            dist = np.hypot(gap[:, 0], gap[:, 1])
            near = dist <= float(dist.min()) + max(float(reach), 0.0)
            q = boundary[near]
            vq = np.column_stack([vx - w * q[:, 1], vy + w * q[:, 0]])
            if np.any(np.sum(vq * gap[near], axis=1) >= 0.0):
                return False
        return True

    def _closest_footprint_points(self, pts: np.ndarray) -> np.ndarray:
        """For each body-frame point, the nearest sample of the footprint boundary."""
        boundary = self.fields._boundary
        d2 = ((pts[:, None, :] - boundary[None, :, :]) ** 2).sum(axis=2)
        return boundary[np.argmin(d2, axis=1)]

    def override(self, command):
        """The command the robot may act on: zero while a stop is active, else unchanged.

        A protective stop removes the command; how fast the base then decelerates is the
        platform's business (the acceleration limiter), not the emulator's. `a_brake` sizes
        the fields and is not a command.
        """
        c = np.asarray(command, dtype=float)
        return np.zeros_like(c) if self.stop_active else c
