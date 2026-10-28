"""The oracle tracker with FREE body orientation (ruled 2026-10-15). Pure functions.

The first oracle turned the robot's body to face the route's tangent -- an assumption the criterion
never made, and wrong for a mecanum base, which may travel at any heading to its body. With one body
offset held through each breached stretch, 142 of the 143 residual routes cleared. Ruled: formalise
the oracle with free orientation, constrained by

  * **yaw continuity** along the route;
  * **yaw rate** <= `actuation.w_max` and **yaw acceleration** <= `actuation.alpha_max` (VERIFY);
  * **the wheel budget at the nominal share** (|vx| + |vy| + (l+w)|w| <= 0.80 v_rim): body rotation
    consumes budget, and when it binds the oracle drops to a slower band;
  * **the field oriented along the velocity direction**, the footprint (`FootprintTable`) rotating
    with the body.

**The model.** Body yaw lives on a grid anchored at the scenario's START yaw (`yaw_grid`), so the
robot's real starting orientation is a grid point and the start is checked exactly. At each route
sample the robot either keeps its yaw while moving to the next sample, turns one grid step while
moving (`turning_speed_limit`), or rotates in place through orientations whose band-0 field is clear
there (`closure`) -- continuity means it can never jump across a breached orientation.

**Feasibility does not depend on the rate, acceleration or budget limits**, and this is a result,
not an omission: every one of them scales down with speed (yaw rate = curvature x speed; yaw
acceleration and the budget's rotation term likewise), and band 0 admits any speed up to its top,
so any continuous yaw path is executable slowly enough. A route is therefore traversable without a
stop iff a continuous yaw path through band-0-clear states exists (`reach`). The limits decide the
SPEED along that path -- which band, so the mission time (`fastest_path_time`).

**The unavoidable-stop floor** counts separate stretches where no continuous path survives (`reach`,
`stops`). It is platform-specific -- a holonomic base with full knowledge of its route -- and is said
so wherever it is reported (ruled 2026-10-15).
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike


def yaw_grid(start_yaw: float, step: float) -> np.ndarray:
    """Body yaws (rad, world frame) every `step` round the full turn, index 0 = the start yaw."""
    n = round(2.0 * math.pi / step)
    if n < 4 or abs(n * step - 2.0 * math.pi) > 1e-9:
        raise ValueError(f"step {step} rad must divide the full turn into at least 4")
    th = start_yaw + step * np.arange(n)
    return np.arctan2(np.sin(th), np.cos(th))


def yaw_set(start_yaw: float, step: float, extra: ArrayLike = ()) -> tuple[np.ndarray, int]:
    """The orientations the oracle may take: the grid every `step` from the start yaw, PLUS the
    `extra` yaws -- the route's own segment directions -- in circular (ascending) order, and the
    start yaw's index.

    Why the extras (found 2026-10-15): a grid anchored at the start yaw rarely contains the route's
    direction exactly, and in a tight aisle a body turned even 7.5 deg off the aisle swings its
    corners into the field. All 50 residual routes of the first free-orientation run were straights
    the tangent-locked oracle drove clear: the orientation along the route must be available.
    """
    grid = yaw_grid(start_yaw, step)
    every = np.concatenate([grid, np.asarray(extra, dtype=float).ravel()])
    every = np.arctan2(np.sin(every), np.cos(every))
    order = np.sort(every)
    keep = np.concatenate([[True], np.diff(order) > 1e-9])
    yaws = order[keep]
    if yaws.size > 1 and (yaws[0] + 2.0 * math.pi - yaws[-1]) <= 1e-9:
        yaws = yaws[:-1]                            # -pi and pi are one orientation
    start = float(grid[0])
    gap = np.abs(np.arctan2(np.sin(yaws - start), np.cos(yaws - start)))
    return yaws, int(np.argmin(gap))


def closure(clear: ArrayLike, seeds: ArrayLike) -> np.ndarray:
    """The orientations reachable by rotating in place from `seeds` through `clear` ones.

    The grid is circular: index 0 neighbours index J-1. A seed that is not clear reaches nothing.
    """
    c = np.asarray(clear, dtype=bool)
    s = np.asarray(seeds, dtype=bool) & c
    out = s.copy()
    if not s.any():
        return out
    if c.all():
        return c.copy()
    n = c.size
    # Walk each circular run of clear orientations; mark it if it holds a seed.
    start = int(np.flatnonzero(~c)[0])          # begin just after a blocked orientation
    run: list[int] = []
    for k in range(1, n + 1):
        j = (start + k) % n
        if c[j]:
            run.append(j)
        if not c[j] or k == n:
            if run and s[run].any():
                out[run] = True
            run = []
    return out


@dataclass(frozen=True)
class Reach:
    start_valid: bool        # the start yaw's band-0 field is clear at the start
    goal_valid: bool         # some orientation's band-0 field is clear on arrival
    traversable: bool        # a continuous band-0-clear yaw path runs start to goal
    stops: int               # separate stretches no continuous path survives: the floor
    first_death: int | None  # sample index where the path first dies (None if traversable)
    deaths: tuple[int, ...]  # the sample index where each stop's stretch begins
    reachable: np.ndarray    # (N, J) reachable orientations; after a stop, re-seeded


def reach(clear0: ArrayLike, start_index: int = 0) -> Reach:
    """Continuous yaw paths through band-0-clear states, start to goal.

    `clear0` is (N samples, J yaws). From the start yaw the robot may rotate in place through clear
    orientations; it moves to the next sample keeping its yaw, then may rotate again. Where every
    path dies the oracle must stop: one stop per separate stretch. After a stop it may restart in
    any clear orientation (the floor is therefore a lower bound on the stops a real controller,
    with less freedom after a latched stop, would make).
    """
    c = np.asarray(clear0, dtype=bool)
    n = c.shape[0]
    reached = np.zeros_like(c)
    start_valid = bool(c[0, start_index])
    goal_valid = bool(c[-1].any())
    seeds = np.zeros(c.shape[1], dtype=bool)
    seeds[start_index] = True
    stops, dead = 0, False
    deaths: list[int] = []
    r = closure(c[0], seeds)
    if not r.any():
        stops = 1
        deaths.append(0)
        r = c[0].copy()
        dead = not r.any()
    reached[0] = r
    for i in range(1, n):
        if dead:
            if c[i].any():
                r, dead = c[i].copy(), False
            reached[i] = r if not dead else False
            continue
        r = closure(c[i], r)
        if not r.any():
            stops += 1
            deaths.append(i)
            r = c[i].copy()
            dead = not r.any()
        reached[i] = r
    return Reach(start_valid=start_valid, goal_valid=goal_valid,
                 traversable=start_valid and stops == 0, stops=stops,
                 first_death=deaths[0] if deaths else None, deaths=tuple(deaths),
                 reachable=reached)


@dataclass(frozen=True)
class Limits:
    """What bounds the oracle's speed and turning (all from `configs/base.yaml`)."""

    v_max: float            # m/s, body-x axis ceiling
    vy_max: float           # m/s, body-y axis ceiling
    w_max: float            # rad/s
    alpha_max: float        # rad/s^2 (VERIFY in the config)
    budget: float           # m/s, nominal share of the rim speed: 0.80 x v_rim
    lever_arm: float        # m, half_wheelbase + half_track
    decel: float            # m/s^2, actuation.a_decel_max


def turning_speed_limit(heading: float, turn_per_m: float, lim: Limits, ds: float) -> float:
    """The fastest speed (m/s) at which the body can travel at `heading` (rad, body frame) while
    turning `turn_per_m` (rad per metre of travel), within the box, the wheel budget, the yaw rate
    and -- when turning -- the yaw acceleration needed to start the turn within one step `ds`.

    Budget: v(|cos h| + |sin h|) + (l+w) kappa v <= budget. Rate: kappa v <= w_max. Acceleration:
    reaching kappa v from rest within ds / v needs alpha ds / v >= kappa v.
    """
    c, s = abs(math.cos(heading)), abs(math.sin(heading))
    k = abs(turn_per_m)
    caps = [lim.budget / (c + s + lim.lever_arm * k)]
    if c > 1e-12:
        caps.append(lim.v_max / c)
    if s > 1e-12:
        caps.append(lim.vy_max / s)
    if k > 0.0:
        caps.append(lim.w_max / k)
        caps.append(math.sqrt(lim.alpha_max * ds / k))
    return float(min(caps))


def rotation_time(angle: float, lim: Limits) -> float:
    """Seconds to rotate in place by `angle` (rad) from rest to rest: the rate limited by w_max
    and by the wheel budget (all of it spent on rotation), trapezoidal under alpha_max."""
    a = abs(angle)
    if a == 0.0:
        return 0.0
    w = min(lim.w_max, lim.budget / lim.lever_arm)
    if a >= w * w / lim.alpha_max:
        return a / w + w / lim.alpha_max
    return 2.0 * math.sqrt(a / lim.alpha_max)


def fastest_path_time(arc: ArrayLike, tangents: ArrayLike, yaws: ArrayLike,
                      reachable: ArrayLike, caps: ArrayLike, lim: Limits,
                      start_index: int = 0) -> float:
    """Mission time (s) of the fastest continuous yaw path through the reachable states.

    `caps` (N, J): the top speed of the fastest clear band at each state (0 where none is). A move
    to the next sample keeps the yaw or turns one grid step, at most `turning_speed_limit`; at any
    sample the robot may also rotate in place (from rest, `rotation_time`) to another reachable
    orientation the shorter way round, every orientation passed reachable. Dynamic programming
    over (sample, yaw) minimises the summed time. Rotations start only from states reached by a
    move: rest-to-rest rotation time is subadditive, so chaining two never beats one. The chosen
    path's speeds then get the deceleration-limited backward pass, stopping for each in-place
    rotation and at the goal. No acceleration limit (as for the first oracle): a best case.
    Infinite if no continuous path reaches the goal.
    """
    s = np.asarray(arc, dtype=float)
    phi = np.asarray(tangents, dtype=float)
    th = np.asarray(yaws, dtype=float)
    R = np.asarray(reachable, dtype=bool)
    cap = np.asarray(caps, dtype=float)
    n, nj = R.shape
    # The orientations need not be evenly spaced (`yaw_set` adds the route's directions): angles
    # come from the gaps between circular neighbours, not from a uniform step.
    gaps = (np.roll(th, -1) - th) % (2.0 * math.pi)       # from yaw k to yaw k+1, counter-clockwise
    if nj == 1:
        gaps = np.array([2.0 * math.pi])
    cum = np.concatenate([[0.0], np.cumsum(np.concatenate([gaps, gaps]))])
    inf = math.inf
    cost = np.full((n, nj), inf)
    back_move = np.full((n, nj), -1, dtype=int)   # yaw at the previous sample
    back_rot = np.full((n, nj), -1, dtype=int)    # yaw at THIS sample before rotating in place
    speed = np.zeros((n, nj))                     # speed of the move into (i, j)
    if R[0, start_index]:
        cost[0, start_index] = 0.0

    # For each (from k, to j): the counter-clockwise step count and angle, which way round is
    # shorter by ANGLE, and the rotation time -- the same for every sample, so computed once.
    dist = np.arange(nj)
    kk, jj_ = np.meshgrid(np.arange(nj), np.arange(nj), indexing="ij")
    up = (jj_ - kk) % nj                                   # steps counter-clockwise from k to j
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
        # prefix sums over the doubled circle: blocked count on any arc in O(1)
        pre = np.concatenate([[0], np.cumsum(np.concatenate([blocked, blocked]))])
        for k in froms:
            j = dist
            u = up[k]
            # counter-clockwise arc k..k+u, or clockwise arc k-d..k with d = nj - u
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
                if ds <= 0.0:                     # a vertex's second sample: the same place
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
        return inf

    j = int(np.argmin(cost[-1]))
    v_path = np.zeros(n)
    stop_at = np.zeros(n, dtype=bool)
    rot_time = 0.0
    for i in range(n - 1, -1, -1):
        k = back_rot[i, j]
        if k >= 0:                                # rotated in place here, from yaw k
            rot_time += float(rot_of[k, j])
            stop_at[i] = True
            j = int(k)
        v_path[i] = speed[i, j]
        if i > 0:
            j = int(back_move[i, j])
    v_path[stop_at] = 0.0
    v_path[0] = 0.0                               # starts from rest
    v_path[-1] = 0.0                              # arrives at rest
    for i in range(n - 2, -1, -1):                # deceleration-limited backward pass
        v_path[i] = min(v_path[i],
                        math.sqrt(v_path[i + 1] ** 2 + 2.0 * lim.decel * (s[i + 1] - s[i])))
    total = rot_time
    for i in range(n - 1):
        ds = s[i + 1] - s[i]
        if ds <= 0.0:
            continue
        vsum = v_path[i] + v_path[i + 1]
        if vsum <= 0.0:                           # rest to rest over one step
            vsum = 2.0 * math.sqrt(lim.decel * ds)
        total += 2.0 * ds / vsum
    return float(total)


def tracking_yaw_rates(samples, window: float, speed: float) -> np.ndarray:
    """The yaw rate, rad/s, of a body tracking the route's tangent at `speed` (m/s): speed x
    kappa, kappa the tangent's turn over a `window` (m) centred on each sample, divided by the
    window (ruled 2026-10-16: "tangent tracking at a bend implies omega = v kappa"). A polyline's
    curvature sits at its vertices; the window -- the tracker's lookahead -- spreads each turn
    over the distance a tracker takes to make it. Zero on straights."""
    from pfbnav.world.route_check import arc_length

    if window <= 0.0 or speed < 0.0:
        raise ValueError("window must be positive and speed non-negative")
    arc = arc_length(samples)
    th = np.unwrap(np.asarray(samples.yaw, dtype=float))
    lo = np.clip(np.searchsorted(arc, arc - window / 2.0, side="left"), 0, arc.size - 1)
    hi = np.clip(np.searchsorted(arc, arc + window / 2.0, side="right") - 1, 0, arc.size - 1)
    return speed * np.abs(th[hi] - th[lo]) / window


def rate_levels(yaw_rates: ArrayLike, step: float) -> np.ndarray:
    """Yaw rates rounded UP to multiples of `step` (rad/s), so field shapes are shared and the
    rounding only ever enlarges the field."""
    r = np.asarray(yaw_rates, dtype=float)
    return np.ceil(r / step - 1e-12) * step


def state_tables(samples, obstacles: Sequence[ArrayLike], yaws: ArrayLike, shape_for,
                 mask: ArrayLike | None = None, heading_round: float = math.radians(1.0),
                 coarse_stride: int = 1, yaw_rates: ArrayLike | None = None,
                 rate_step: float = 0.05) -> np.ndarray:
    """Clearance (m; negative = breach) of every (sample, yaw) state's field, NaN where `mask` is
    False. The field travels along the sample's tangent with the body at the state's yaw:
    `shape_for(heading)` returns the `FieldShape` for a body-frame travel heading, rounded to
    `heading_round` (the lead term moves well under a millimetre for half a degree) so shapes are
    shared across routes. `coarse_stride` > 1 decides breaches exactly but reports a clear state's
    clearance as a lower bound (`clearance_to_field`). `yaw_rates` (rad/s per sample, e.g.
    `tracking_yaw_rates`) add the rotation sweep, rounded up to `rate_step`; then
    `shape_for(heading, yaw_rate)` is asked for the swept shape. Without them every shape is the
    rotation-blind one, exactly as before."""
    from pfbnav.world.route_check import RouteSamples, clearance_to_field

    th = np.asarray(yaws, dtype=float)
    n = samples.x.size
    out = np.full((n, th.size), np.nan)
    m = np.ones((n, th.size), dtype=bool) if mask is None else np.asarray(mask, dtype=bool)
    tangent_key = np.round(samples.yaw, 9)
    levels = (np.zeros(n) if yaw_rates is None
              else rate_levels(np.asarray(yaw_rates, dtype=float), rate_step))
    for t in np.unique(tangent_key):
        for level in np.unique(levels[tangent_key == t]):
            rows = np.flatnonzero((tangent_key == t) & (levels == level))
            for j, yaw in enumerate(th):
                sel = rows[m[rows, j]]
                if sel.size == 0:
                    continue
                heading = math.atan2(math.sin(t - yaw), math.cos(t - yaw))
                heading = round(heading / heading_round) * heading_round
                shape = (shape_for(heading) if level == 0.0
                         else shape_for(heading, float(level)))
                sub = RouteSamples(samples.x[sel], samples.y[sel], np.full(sel.size, yaw),
                                   samples.inside[sel])
                out[sel, j] = clearance_to_field(sub, obstacles, shape,
                                                 coarse_stride=coarse_stride)
    return out


# --- the generator's validity rules (ruled 2026-10-15) --------------------------------------------
#
# A built route must be traversable by the free-orientation oracle without a stop. The generator
# classifies each candidate route and, on any failure, re-samples the GOAL -- never the layout --
# logging the rejection by cause:
#   * START_INVALID -- the band-0 field along the route's initial direction is breached at the start,
#     with the body at its start yaw (the generator sets it along the route's first segment);
#   * GOAL_INVALID  -- no orientation's band-0 field along the final approach is clear on arrival;
#   * RESIDUAL      -- valid at both ends, but no continuous orientation path keeps band 0 clear
#     between them (the residual rule: <= 2 % per stratum in the quantification, re-sampled here).

ROUTE_OK = "ok"
START_INVALID = "start_invalid"
GOAL_INVALID = "goal_invalid"
RESIDUAL = "residual"
VALIDITY_FAILURES = (START_INVALID, GOAL_INVALID, RESIDUAL)


def route_validity(path: ArrayLike, obstacles: Sequence[ArrayLike], start_yaw: float, shape_for,
                   *, sample_spacing: float, yaw_step: float, heading_round: float,
                   coarse_stride: int, tracking_window: float | None = None,
                   tracking_speed: float = 0.0, rate_step: float = 0.05) -> str:
    """Classify a planned route against the mapped obstacles: `ROUTE_OK` or the first failure in
    the order start, goal, residual. `shape_for(heading[, yaw_rate])` gives the band-0
    `FieldShape` for a body-frame travel heading. With `tracking_window`, every state's field
    carries the rotation sweep of tangent tracking at `tracking_speed` (ruled 2026-10-16)."""
    from pfbnav.world.route_check import sample_route

    samples = sample_route(path, sample_spacing)
    yaws, start = yaw_set(start_yaw, yaw_step, extra=np.unique(np.round(samples.yaw, 12)))
    rates = (None if tracking_window is None
             else tracking_yaw_rates(samples, tracking_window, tracking_speed))
    gaps = state_tables(samples, obstacles, yaws, shape_for, heading_round=heading_round,
                        coarse_stride=coarse_stride, yaw_rates=rates, rate_step=rate_step)
    r = reach(gaps >= 0, start_index=start)
    if not r.start_valid:
        return START_INVALID
    if not r.goal_valid:
        return GOAL_INVALID
    return ROUTE_OK if r.traversable else RESIDUAL
