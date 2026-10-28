"""The classical baselines' declared route cap, and the privileged oracle tracker's schedule
(ruled 2026-10-20). Pure functions, computed OFFLINE per scenario, before its episode.

**Why.** Under the A20-A22 fields a controller that holds one speed (B1: 0.25 m/s, up to 1.4 rad/s
at bends) is stopped at the first tight spot and never released: the whole-field restart judges the
field of the twist it commands, and that field never clears. Both oracles pass the same routes with
zero stops, because they slow BEFORE the spot with knowledge of the whole route
(`docs/decision_log.md`, 2026-10-20). Ruled:

**The route cap** -- every classical baseline (B1, B2, B3, B6, B7) gets the same declared, map-based
speed and yaw-rate cap along its route; learned arms do not (the asymmetry is stated in the methods).
It reuses the oracle restricted to the MAP: unmapped obstacles never enter it (non-privileged). At
each route sample, with the body along the route tangent (as a path tracker holds it):

  * the fastest band whose field -- carrying the rotation sweep of tangent tracking at that band's
    speed limit (`tracking_yaw_rates`, the turn spread over `window`) -- is clear of the mapped
    obstacles, both on the route and displaced toward the inside of the bends by the committed
    tracking bound (`inside_offset`), gives the speed cap (that band's speed limit, A20) and the
    yaw-rate cap (the sweep's rate, rounded up to `rate_step`);
  * where no band survives the offset, the fastest band clear ON the route (the oracle's own test;
    found 2026-10-20: on three tune routes the offset alone left 1.5-2.6 m stretches where nothing
    was clear, while on the route itself band 0 or better was clear everywhere -- the planner's
    margins let a route pass within the tracking bound of band 0's field, and the cap would have
    held the robot at creep for minutes);
  * where no band is clear, the creep regime: both caps at 0.9 x the standstill exits;
  * then both caps are ANTICIPATED: each sample takes the minimum over the next `lead` m, and the
    speed is deceleration-limited backward at `decel`, so the cap falls ahead of a tight spot.

A controller may go slower, never faster (`pfbnav/control/route_cap.py`).

**The privileged schedule** -- G4's positive control only (ruled 2026-10-20, pre-declared
2026-10-17): the free-orientation oracle over ALL obstacles, mapped and unmapped, with bands 1 and 2
added to its tables, and the fastest continuous yaw path through them (`fastest_schedule`): a body
yaw and a speed per sample. A holonomic base need not turn its body at a bend, so the schedule's
body turns at most the route's yaw-rate ALLOWANCE -- the largest rate of a declared ladder at which
band 0, every state swept at that rate, stays traversable (ruled 2026-10-20, rounded down) -- in the
dynamic programme's turning and in-place rotation alike, and every table carries exactly that
rate's sweep; the tracker caps its yaw rate there. (Found 2026-10-20: tables swept at TANGENT-tracking rates -- about 1 rad/s at a 90 deg
bend in the top band -- left 0.5-1.6 m of every route to band 0, whose usable speed is 0.025 m/s,
and scheduled 56-127 s for routes the episode gives 75 s.) The tracker follows the route with
feedback, so its schedule is tested on the route (`inside_offset` 0 by default for it).
Where no continuous path exists (an unmapped obstacle across the route), the schedule falls back to
the tangent-locked cap over all obstacles, and says so.

`pfbnav/world/oracle.py` is reused, never edited: it sits in the scenario identity's closure (A18),
so `fastest_schedule` repeats `fastest_path_time`'s dynamic programme with the path kept, and a
test pins the two to the same mission time.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import ArrayLike

from pfbnav.safety.certified import CertifiedParams, FieldGeometry, band_speed_limit
from pfbnav.world.oracle import (
    Limits,
    rate_levels,
    reach,
    rotation_time,
    state_tables,
    tracking_yaw_rates,
    turning_speed_limit,
    yaw_set,
)
from pfbnav.world.route_check import (
    FieldShape,
    RouteSamples,
    arc_length,
    clearance_to_field,
    sample_route,
)

PROFILE_VERSION = 1


@dataclass(frozen=True)
class ProfileSpec:
    """What a route profile is computed with (`baselines.route_cap` in `configs/base.yaml`)."""

    sample_spacing: float     # m between route samples
    window: float             # m, the turn of tangent tracking is spread over this
    inside_offset: float      # m, the committed tracking bound, toward the inside of bends
    rate_step: float          # rad/s, sweep rates rounded UP to multiples of this
    lead: float               # m, each cap is the minimum over the next `lead` m
    decel: float              # m/s^2, the backward deceleration pass
    coarse_stride: int        # the exact coarse-first separating-axis test (route_check)
    creep_speed: float        # m/s, nothing clear: 0.9 x the translational standstill exit
    creep_yaw_rate: float     # rad/s, nothing clear: 0.9 x the rotational standstill exit

    @classmethod
    def from_config(cls, cfg: dict[str, Any], params: CertifiedParams) -> ProfileSpec:
        rc = cfg["baselines"]["route_cap"]
        return cls(sample_spacing=float(rc["sample_spacing_m"]),
                   window=float(cfg["planner"]["lookahead"]),
                   inside_offset=float(rc["inside_offset_m"]),
                   rate_step=float(cfg["world"]["layout"]["validity"]["yaw_rate_step"]),
                   lead=float(rc["lead_m"]), decel=float(rc["decel"]),
                   coarse_stride=int(cfg["world"]["layout"]["validity"]["coarse_stride"]),
                   creep_speed=float(rc["creep_fraction"]) * params.rest_speed,
                   creep_yaw_rate=float(rc["creep_fraction"]) * params.standstill_yaw_exit)


@dataclass(frozen=True)
class RouteProfile:
    """Caps along one route, by arc length. `yaw_offset` (body yaw minus the route's tangent, rad)
    exists only for the privileged schedule; `traversable` says whether the schedule found a
    continuous path (None for a cap)."""

    kind: str                    # "cap" or "schedule"
    arc: np.ndarray              # m, non-decreasing (route vertices are sampled twice)
    speed: np.ndarray            # m/s
    yaw_rate: np.ndarray         # rad/s, >= 0
    band: np.ndarray             # the band the speed comes from; -1 = the creep regime
    yaw_offset: np.ndarray | None = None
    traversable: bool | None = None
    yaw_allowance: float | None = None     # rad/s, the schedule's (None for a cap)
    schedule_time: float | None = None     # s, the oracle programme's mission time (schedule)

    @property
    def length(self) -> float:
        return float(self.arc[-1])

    def index(self, s: float) -> int:
        """The last sample at or before arc length `s` (m), clipped to the route."""
        return int(np.clip(np.searchsorted(self.arc, s, side="right") - 1, 0, self.arc.size - 1))

    def caps(self, s: float) -> tuple[float, float]:
        """(speed m/s, yaw rate rad/s) in force at arc length `s`: the smaller of the samples
        either side, so a position between two samples never takes the larger."""
        i = self.index(s)
        j = min(i + 1, self.arc.size - 1)
        return (float(min(self.speed[i], self.speed[j])),
                float(min(self.yaw_rate[i], self.yaw_rate[j])))

    def to_dict(self) -> dict[str, Any]:
        out = {"version": PROFILE_VERSION, "kind": self.kind, "arc": self.arc.tolist(),
               "speed": self.speed.tolist(), "yaw_rate": self.yaw_rate.tolist(),
               "band": self.band.astype(int).tolist(), "traversable": self.traversable,
               "yaw_allowance": self.yaw_allowance, "schedule_time": self.schedule_time}
        if self.yaw_offset is not None:
            out["yaw_offset"] = self.yaw_offset.tolist()
        return out

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> RouteProfile:
        if d.get("version") != PROFILE_VERSION:
            raise ValueError(f"route profile version {d.get('version')}, expected "
                             f"{PROFILE_VERSION}")
        off = d.get("yaw_offset")
        return cls(kind=str(d["kind"]), arc=np.asarray(d["arc"], dtype=float),
                   speed=np.asarray(d["speed"], dtype=float),
                   yaw_rate=np.asarray(d["yaw_rate"], dtype=float),
                   band=np.asarray(d["band"], dtype=int),
                   yaw_offset=None if off is None else np.asarray(off, dtype=float),
                   traversable=d.get("traversable"), yaw_allowance=d.get("yaw_allowance"),
                   schedule_time=d.get("schedule_time"))


def anticipate(arc: ArrayLike, values: ArrayLike, lead: float) -> np.ndarray:
    """Each sample's value replaced by the minimum over samples within the next `lead` m (itself
    included). Known answer in `tests/test_route_profile.py`."""
    a = np.asarray(arc, dtype=float)
    v = np.asarray(values, dtype=float)
    if lead < 0.0:
        raise ValueError("lead must be non-negative")
    end = np.searchsorted(a, a + lead, side="right")
    return np.array([float(np.min(v[i:max(int(end[i]), i + 1)])) for i in range(v.size)])


def decelerate_backward(arc: ArrayLike, speed: ArrayLike, decel: float) -> np.ndarray:
    """The deceleration-limited backward pass: v_i <= sqrt(v_{i+1}^2 + 2 a (s_{i+1} - s_i)), so a
    robot at the cap can always brake to the next sample's cap at `decel`."""
    a = np.asarray(arc, dtype=float)
    v = np.asarray(speed, dtype=float).copy()
    if decel <= 0.0:
        raise ValueError("decel must be positive")
    for i in range(v.size - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * decel * max(0.0, a[i + 1] - a[i])))
    return v


def _sub(samples: RouteSamples, idx: np.ndarray) -> RouteSamples:
    return RouteSamples(samples.x[idx], samples.y[idx], samples.yaw[idx], samples.inside[idx])


def _band_clear_tangent(samples: RouteSamples, obstacles, fields: FieldGeometry, top: float,
                        rates: np.ndarray, todo: np.ndarray, spec: ProfileSpec
                        ) -> tuple[np.ndarray, np.ndarray]:
    """Whether the band's field (body along the tangent, swept at each sample's rate level) is clear
    of `obstacles` for the samples in `todo`: (on the route AND at the inside offset, on the
    route)."""
    both = np.zeros(samples.x.size, dtype=bool)
    on = np.zeros(samples.x.size, dtype=bool)
    shifted = samples.offset(spec.inside_offset)
    for level in np.unique(rates[todo]):
        idx = np.flatnonzero(todo & (rates == level))
        shape = FieldShape.moving(fields, top, heading=0.0, yaw_rate=float(level))
        here = clearance_to_field(_sub(samples, idx), obstacles, shape,
                                  coarse_stride=spec.coarse_stride) >= 0.0
        moved = clearance_to_field(_sub(shifted, idx), obstacles, shape,
                                   coarse_stride=spec.coarse_stride) >= 0.0
        on[idx] = here
        both[idx] = here & moved
    return both, on


def route_cap(path: ArrayLike, obstacles: Sequence[ArrayLike], fields: FieldGeometry,
              params: CertifiedParams, spec: ProfileSpec, *, kind: str = "cap") -> RouteProfile:
    """The declared route cap for `path` against `obstacles` -- the MAPPED ones for a baseline's
    cap (non-privileged); every obstacle for the schedule's fallback. See the module docstring."""
    samples = sample_route(path, spec.sample_spacing)
    arc = arc_length(samples)
    n = arc.size
    tops = [float(t) for t in params.band_top_speeds]
    limits = [band_speed_limit(b, params) for b in range(len(tops))]
    band_both = np.full(n, -1, dtype=int)
    band_on = np.full(n, -1, dtype=int)
    rate_of: list[np.ndarray] = [np.zeros(n)] * len(tops)
    for b in reversed(range(len(tops))):              # fastest first
        rates = rate_levels(tracking_yaw_rates(samples, spec.window, limits[b]), spec.rate_step)
        rate_of[b] = rates
        todo = band_on < 0
        if not todo.any():
            break
        both, on = _band_clear_tangent(samples, obstacles, fields, tops[b], rates, todo, spec)
        band_both[todo & both & (band_both < 0)] = b
        band_on[todo & on] = b
    band = np.where(band_both >= 0, band_both, band_on)
    yaw_cap = np.full(n, spec.creep_yaw_rate)
    for b in range(len(tops)):
        yaw_cap = np.where(band == b, np.maximum(rate_of[b], spec.creep_yaw_rate), yaw_cap)
    speed = np.where(band >= 0, np.asarray(limits)[np.maximum(band, 0)], spec.creep_speed)
    speed = decelerate_backward(arc, anticipate(arc, speed, spec.lead), spec.decel)
    return RouteProfile(kind=kind, arc=arc, speed=speed,
                        yaw_rate=anticipate(arc, yaw_cap, spec.lead),
                        band=anticipate(arc, band, spec.lead).astype(int))


@dataclass(frozen=True)
class Schedule:
    """The fastest continuous yaw path: per sample, the yaw index arrived with and departed with
    (they differ where it rotates in place), and the speed after the deceleration pass."""

    time: float
    arrive: np.ndarray
    depart: np.ndarray
    speed: np.ndarray


def fastest_schedule(arc: ArrayLike, tangents: ArrayLike, yaws: ArrayLike, reachable: ArrayLike,
                     caps: ArrayLike, lim: Limits, start_index: int = 0) -> Schedule | None:
    """`pfbnav.world.oracle.fastest_path_time`'s dynamic programme, step for step, keeping the
    path. None where no continuous path reaches the goal. `tests/test_route_profile.py` pins
    `time` to `fastest_path_time` on the same inputs."""
    s = np.asarray(arc, dtype=float)
    phi = np.asarray(tangents, dtype=float)
    th = np.asarray(yaws, dtype=float)
    R = np.asarray(reachable, dtype=bool)
    cap = np.asarray(caps, dtype=float)
    n, nj = R.shape
    gaps = (np.roll(th, -1) - th) % (2.0 * math.pi)
    if nj == 1:
        gaps = np.array([2.0 * math.pi])
    cum = np.concatenate([[0.0], np.cumsum(np.concatenate([gaps, gaps]))])
    inf = math.inf
    cost = np.full((n, nj), inf)
    back_move = np.full((n, nj), -1, dtype=int)
    back_rot = np.full((n, nj), -1, dtype=int)
    speed = np.zeros((n, nj))
    if R[0, start_index]:
        cost[0, start_index] = 0.0
    dist = np.arange(nj)
    kk, jj_ = np.meshgrid(np.arange(nj), np.arange(nj), indexing="ij")
    up = (jj_ - kk) % nj
    ccw_angle = cum[kk + up] - cum[kk]
    ccw_short = ccw_angle <= math.pi
    short_angle = np.where(ccw_short, ccw_angle, 2.0 * math.pi - ccw_angle)
    rot_of = np.vectorize(lambda a: rotation_time(float(a), lim))(short_angle)

    def relax_in_place(i: int) -> None:
        base = cost[i].copy()
        froms = np.flatnonzero(np.isfinite(base))
        if froms.size == 0:
            return
        blocked = ~R[i]
        pre = np.concatenate([[0], np.cumsum(np.concatenate([blocked, blocked]))])
        for k in froms:
            j = dist
            u = up[k]
            lo = np.where(ccw_short[k], k, (k - (nj - u)) % nj)
            length = np.where(ccw_short[k], u, nj - u)
            hi = lo + length + 1
            n_blocked = pre[hi] - pre[lo]
            ok = (n_blocked == 0) & R[i] & (j != k)
            c = base[k] + rot_of[k]
            better = ok & (c < cost[i])
            cost[i][better] = c[better]
            back_rot[i][better] = k

    relax_in_place(0)
    for i in range(n - 1):
        ds = s[i + 1] - s[i]
        for j in np.flatnonzero(np.isfinite(cost[i])):
            for dj in ((0,) if ds <= 0.0 else (-1, 0, 1)):
                jj = (j + dj) % nj
                if not R[i + 1, jj]:
                    continue
                if ds <= 0.0:
                    t, v = 0.0, speed[i, j]
                else:
                    v = min(cap[i, j], cap[i + 1, jj],
                            turning_speed_limit(phi[i] - th[j],
                                                (gaps[j] if dj > 0 else gaps[jj] if dj < 0
                                                 else 0.0) / ds, lim, ds))
                    if v <= 0.0:
                        continue
                    t = ds / v
                if cost[i, j] + t < cost[i + 1, jj]:
                    cost[i + 1, jj], speed[i + 1, jj] = cost[i, j] + t, v
                    back_move[i + 1, jj], back_rot[i + 1, jj] = j, -1
        relax_in_place(i + 1)
    if not np.isfinite(cost[-1]).any():
        return None

    j = int(np.argmin(cost[-1]))
    arrive = np.zeros(n, dtype=int)
    depart = np.zeros(n, dtype=int)
    v_path = np.zeros(n)
    stop_at = np.zeros(n, dtype=bool)
    rot_time = 0.0
    for i in range(n - 1, -1, -1):
        depart[i] = j
        k = back_rot[i, j]
        if k >= 0:
            rot_time += float(rot_of[k, j])
            stop_at[i] = True
            j = int(k)
        arrive[i] = j
        v_path[i] = speed[i, j]
        if i > 0:
            j = int(back_move[i, j])
    v_path[stop_at] = 0.0
    v_path[0] = 0.0
    v_path[-1] = 0.0
    for i in range(n - 2, -1, -1):
        v_path[i] = min(v_path[i], math.sqrt(v_path[i + 1] ** 2 + 2.0 * lim.decel * (s[i + 1] - s[i])))
    total = rot_time
    for i in range(n - 1):
        ds = s[i + 1] - s[i]
        if ds <= 0.0:
            continue
        vsum = v_path[i] + v_path[i + 1]
        if vsum <= 0.0:
            vsum = 2.0 * math.sqrt(lim.decel * ds)
        total += 2.0 * ds / vsum
    return Schedule(time=float(total), arrive=arrive, depart=depart, speed=v_path)


def oracle_schedule(path: ArrayLike, obstacles: Sequence[ArrayLike], start_yaw: float,
                    fields: FieldGeometry, params: CertifiedParams, lim: Limits,
                    spec: ProfileSpec, *, yaw_step: float, heading_round: float,
                    yaw_rates: Sequence[float], inside_offset: float = 0.0) -> RouteProfile:
    """The privileged tracker's schedule over `obstacles` (ALL of them: privileged). See the module
    docstring.

    **The yaw-rate allowance** (ruled 2026-10-20, "the largest yaw rate keeping band 0 admissible,
    rounded down"): the rates of `yaw_rates` are tried from the largest down, and the first at which
    band 0 -- every state's field carrying that rate's sweep -- admits a continuous yaw path is the
    route's allowance. The programme then turns no faster, and the tracker's yaw-rate cap is that
    rate. Where none admits one (an unmapped obstacle across the mapped route), the tangent-locked cap
    over all obstacles, and `traversable` False."""
    ladder = sorted({float(r) for r in yaw_rates}, reverse=True)
    if not ladder or ladder[-1] <= 0.0:
        raise ValueError("the yaw-rate ladder must hold positive rates")
    samples = sample_route(path, spec.sample_spacing)
    arc = arc_length(samples)
    n = arc.size
    yaws, start = yaw_set(float(start_yaw), yaw_step, extra=np.unique(np.round(samples.yaw, 12)))
    tops = [float(t) for t in params.band_top_speeds]
    limits = [band_speed_limit(b, params) for b in range(len(tops))]
    shifted = samples.offset(inside_offset)
    cache: dict[tuple, FieldShape] = {}

    def shape_for(top: float):
        def get(heading: float, yaw_rate: float = 0.0) -> FieldShape:
            key = (top, round(heading, 9), round(yaw_rate, 9))
            if key not in cache:
                cache[key] = FieldShape.moving(fields, top, heading=heading, yaw_rate=yaw_rate)
            return cache[key]
        return get

    def table(b: int, rate: float, mask) -> np.ndarray:
        ok = np.ones((n, yaws.size), dtype=bool) if mask is None else mask.copy()
        # Band 0 on the route only -- the oracle's own test, so a route the generator accepted stays
        # traversable; the faster bands also at the inside offset when one is given.
        variants = (samples,) if b == 0 or inside_offset == 0.0 else (samples, shifted)
        for variant in variants:
            g = state_tables(variant, obstacles, yaws, shape_for(tops[b]), mask=ok,
                             heading_round=heading_round, coarse_stride=spec.coarse_stride,
                             yaw_rates=np.full(n, rate), rate_step=spec.rate_step)
            ok &= np.nan_to_num(g, nan=-1.0) >= 0.0
        return ok

    for rate in ladder:
        band0 = table(0, rate, None)
        r = reach(band0, start_index=start)
        if not r.traversable:
            continue
        clear = [band0]
        for b in range(1, len(tops)):
            clear.append(table(b, rate, clear[-1]))
        caps = np.where(r.reachable, limits[0], 0.0)
        for b in range(1, len(tops)):
            caps = np.where(r.reachable & clear[b], limits[b], caps)
        turning = Limits(v_max=lim.v_max, vy_max=lim.vy_max, w_max=min(lim.w_max, rate),
                         alpha_max=lim.alpha_max, budget=lim.budget, lever_arm=lim.lever_arm,
                         decel=lim.decel)
        sched = fastest_schedule(arc, samples.yaw, yaws, r.reachable, caps, turning,
                                 start_index=start)
        if sched is None:
            continue
        j = sched.depart
        band = np.full(n, -1, dtype=int)
        for b in range(len(tops)):
            here = clear[b][np.arange(n), j] & r.reachable[np.arange(n), j]
            band = np.where(here, b, band)
        speed = decelerate_backward(arc, anticipate(arc, sched.speed, spec.lead), spec.decel)
        offset = np.arctan2(np.sin(yaws[j] - samples.yaw), np.cos(yaws[j] - samples.yaw))
        return RouteProfile(kind="schedule", arc=arc, speed=speed,
                            yaw_rate=np.full(n, rate), band=band, yaw_offset=offset,
                            traversable=True, yaw_allowance=rate, schedule_time=sched.time)
    fallback = route_cap(path, obstacles, fields, params, spec, kind="schedule")
    return RouteProfile(kind="schedule", arc=fallback.arc, speed=fallback.speed,
                        yaw_rate=fallback.yaw_rate, band=fallback.band,
                        yaw_offset=np.zeros(n), traversable=False)
