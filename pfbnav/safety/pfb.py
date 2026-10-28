"""The Protective-Field Barrier (PFB) safety filter (WP2, ruled 2026-10-23). Pure functions.

This module must not import the simulator, the environment or `pfbnav.safety.certified` (hard rule
3, `tests/test_architecture.py`); its field is `pfbnav/safety/pfb_field.py`, independent too.

**C1** (restated 2026-10-23): a control barrier function whose safe set is the complement of the
certified safety function's own trigger set. Every scan return must stay outside the field of the
REGIME the robot will be in -- the at-rest field while creeping, else the field of the band its
reachable speed selects (Q1, option (b)) -- sized from the same configuration keys (Q2).

**Per row** (one per observed bin within reach), for a candidate command u = (vx, vy, w):

    g_i(u) = r_i(u) - rho(psi_i(u); theta(u), W(u)) - delta   >=   (1 - alpha dt) max(h_i(now), 0)

  * r_i(u), psi_i(u): the return's position in the body frame after tau = dt + the lag allowance of
    travel at u (Q2: the PFB's own latency sets only this horizon);
  * theta(u): the field's heading, along u where u dominates the estimate, else the estimate's;
  * W(u) = max(|w_hat|, |w|): the yaw rate the field is swept for (estimate and command composed,
    lag-safe);
  * delta: the binning penalty, the measured scan residual at its upper bound (Q6, 0.50 mm);
  * h_i(now): the same row's value at the current motion. Clipped at zero: a regime whose field
    is breached now must be clear after one step (never entered breached; 2026-10-23).

**The QP**, one per regime (Q1): minimise (u - u_nom)^T W (u - u_nom), W = diag(1, 1, rho_out^2)
(Q5), subject to the rows linearised about a REACHABLE point near u_nom (tangents in translation,
chords over a yaw-rate trust interval, tightened by `lin_margin`), the wheel polytope (the filter's
share of v_rim), the axis box, the limiter's one-step envelope in its two convex cases (hold or
accelerate within a_max dt; else brake within a_decel dt without gaining speed; inscribed polygons),
and the regime's speed range. The TRUE rows are checked at the solution with no tolerance; one
re-linearisation, about the solution, or about the braking command if the first QP was infeasible;
otherwise the regime is infeasible (Q3). The best feasible regime wins. None feasible: the
maximum-braking command, labelled `brake` if it satisfies the true rows of an admissible regime,
else the fallback flagged `infeasible` (counted per method and stratum, Q3).

**Under the certified latch** (Q7) only a command the restart condition could release passes -- a
creep whose at-rest field is clear or that strictly recedes from every point inside it (A22/A24),
or a twist whose own band field is clear -- otherwise zero.

**An unobserved bin** (ruled 2026-10-27) is an obstacle at the footprint's boundary on its bearing,
the convention shared with every classical consumer of the stream (`pfbnav/safety/unobserved.py`):
it enters the rows as a return like any other, so a command moving away from it can be admissible.
(Until 2026-10-27 any unobserved bin sent the filter straight to the fallback, labelled
"unobserved".) In effect the command is the same: a point on the footprint's boundary lies inside
every regime's field and cannot leave it in one step, so the step is infeasible and falls back to
maximum braking, still labelled "unobserved" (`tests/test_pfb.py`). The count of such bins this
step is reported (`PFBResult.unobserved`). The latch
test `releasable` still treats an unobserved bin as blocking: it predicts the certified layer's
restart condition, and the layer keeps its own rule (an invalid bin breaches every field).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np
import scipy.sparse as sp

from pfbnav.safety.pfb_field import PFBField, band_of
from pfbnav.safety.unobserved import resolve_unobserved

REST = -1             # the regime index of the at-rest (creep) field
_RING = 16            # sides of the inscribed polygons for the speed and envelope discs


@dataclass(frozen=True)
class PFBConfig:
    """The filter's own settings (`filter:` in the config) and the actuation limits it respects."""

    dt: float                # s, the control step
    lag: float               # s, the declared lag allowance added to the prediction horizon
    motion_fractions: tuple[float, ...]  # the realised fractions of the predicted motion each
                                         #   row must hold over (ruled 2026-10-26)
    motion_grid_margin: float  # m, subtracted from every row: the grid's gap to the dense minimum
    alpha: float             # 1/s, the class-K gain (Q4: tuned with B7 on the tune layouts)
    delta: float             # m, the binning penalty (Q6)
    budget: float            # m/s, the filter's share of the rim speed
    lever: float             # m, half wheelbase + half track
    v_max: float
    vy_max: float
    w_max: float
    a_max: float             # m/s^2, the limiter's translational bound: the one-step envelope
    alpha_max: float         # rad/s^2, the limiter's yaw bound
    a_decel: float           # m/s^2, the limiter's deceleration bound: the fallback
    weight_yaw: float        # m^2, rho_out^2 (Q5), rho_out read from the footprint table
    creep_fraction: float    # the creep regime's speed and yaw rate: this x the standstill exits
    row_window: float        # m, rows whose linearised slack exceeds this are left out
    linearisations: int      # 2: the first, and one re-linearisation (Q3)
    lin_margin: float        # m, the linearised rows are tightened by this; the TRUE rows are
                             #   checked with none
    yaw_trust: float         # rad/s, the yaw rate's trust interval half-width per linearisation
    band_margin: float       # m/s, kept below each band's speed limit (the estimate's excursions)
    qp_solver: str = "clarabel"   # "clarabel" (interior point) or "osqp" (ADMM), same QP

    @classmethod
    def from_config(cls, cfg: dict[str, Any], field: PFBField) -> PFBConfig:
        f, act = cfg["filter"], cfg["actuation"]
        return cls(
            dt=float(cfg["sim"]["dt"]), lag=float(f["lag_allowance_s"]),
            motion_fractions=tuple(float(x) for x in f["motion_fractions"]),
            motion_grid_margin=float(f["motion_grid_margin_m"]),
            alpha=float(f["alpha"]), delta=float(f["binning_penalty_m"]),
            budget=float(act["wheel_budget"]["filter_fraction"]) * float(
                act["wheel_budget"]["v_rim"]),
            lever=float(cfg["robot"]["half_wheelbase"]) + float(cfg["robot"]["half_track"]),
            v_max=float(act["v_max"]), vy_max=float(act["vy_max"]), w_max=float(act["w_max"]),
            a_max=float(act["a_max"]), alpha_max=float(act["alpha_max"]),
            a_decel=float(act["a_decel_max"]),
            weight_yaw=field.footprint.outer_radius ** 2,
            creep_fraction=float(f["creep_fraction"]), row_window=float(f["row_window_m"]),
            linearisations=int(f["linearisations"]),
            lin_margin=float(f["linearisation_margin_m"]),
            yaw_trust=float(f["yaw_trust_rad_s"]),
            band_margin=float(f["band_speed_margin_m_s"]),
            qp_solver=str(f["qp_solver"]))


@dataclass
class PFBResult:
    """What the filter did this step."""

    u: np.ndarray            # the command to apply, (vx, vy, w)
    mode: str                # "nominal", "filtered", "brake", "fallback" or "latched"
    regime: int | None       # REST, a band index, or None
    infeasible: bool         # the fallback was used
    reason: str | None       # why it fell back, or why a latched command was zeroed
    solves: int              # QPs solved
    heading: float | None = None    # rad, the field heading the chosen command assumed
    yaw_field: float | None = None  # rad/s, the |yaw rate| the field was swept for
    unobserved: int = 0             # bins unobserved this step, entered at the footprint boundary


class PFB:
    """The filter, for one footprint, parameter set and configuration."""

    def __init__(self, field: PFBField, config: PFBConfig):
        self.field = field
        self.p = field.params
        self.c = config
        self._heading = 0.0
        self._at_rest = True
        self._memo: dict | None = None          # the rows of the current `filter` call
        self._clarabel = None                   # (cost matrix, settings), built on first use
        self._ring = np.column_stack([np.cos(2 * math.pi * np.arange(_RING) / _RING),
                                      np.sin(2 * math.pi * np.arange(_RING) / _RING)])

    def reset(self) -> None:
        self._heading = 0.0
        self._at_rest = True

    # ------------------------------------------------------------------ regimes
    def speed_limit(self, regime: int) -> float:
        """The fastest planar command that keeps the robot in `regime` under reach, less
        `band_margin` below a band edge (found 2026-10-23: driven exactly at band 0's limit, an
        estimate a hair above the command put the certified layer in band 1 and it stopped)."""
        if regime == REST:
            return self.c.creep_fraction * self.p.rest_speed
        tops = self.p.band_tops
        if regime == len(tops) - 1:
            return float(self.p.speed_cap)
        return max(float(tops[regime]) - self.p.a_max * self.p.reach_time - self.c.band_margin,
                   0.0)

    def standstill(self, v_hat: np.ndarray, u_prev: np.ndarray) -> bool:
        """The certified layer's standstill rule, MIRRORED from the same keys (found 2026-10-23:
        offering the creep regime whenever the estimate was below the EXIT threshold planned with
        the at-rest field while the layer, by hysteresis, still used band 0's). On the composed
        speed and yaw rate -- the larger of the estimate and the previous command, as the layer
        composes them -- once per step: at rest, moving above either exit threshold; moving, at
        rest at or below both enter thresholds."""
        speed = max(math.hypot(v_hat[0], v_hat[1]), math.hypot(u_prev[0], u_prev[1]))
        w = max(abs(float(v_hat[2])), abs(float(u_prev[2])))
        p = self.p
        if self._at_rest and (speed > p.rest_speed or w > p.rest_yaw):
            self._at_rest = False
        elif not self._at_rest and speed <= p.rest_enter and w <= p.rest_yaw_enter:
            self._at_rest = True
        return self._at_rest

    def _regimes(self, speed_hat: float, w_hat: float) -> list[int]:
        """The regimes a candidate may be in: never below the band the ESTIMATE selects (it lags,
        and the field in force composes it with the command); the creep regime only while the
        mirrored standstill state is at rest."""
        out = []
        if self._at_rest:
            out.append(REST)
        out += list(range(band_of(speed_hat, self.p), len(self.p.band_tops)))
        return out

    def _field(self, regime: int, bearings, heading: float, w: float):
        if regime == REST:
            return self.field.extent_and_partials(bearings, 0.0, heading, w, at_rest=True)
        return self.field.extent_and_partials(bearings, self.p.band_tops[regime], heading, w)

    # ------------------------------------------------------------------ the rows
    def _rows(self, u: np.ndarray, pts: np.ndarray, regime: int, speed_hat: float,
              heading_hat: float, w_hat: float) -> tuple[np.ndarray, np.ndarray]:
        """`_rows_at`, memoised within one `filter` call (2026-10-26, the per-step budget): the scan
        is fixed for the call, and a linearisation point is often the previous solution, whose
        true rows were just checked. Exact: the same arguments give the same arrays, returned
        read-only. Outside a `filter` call nothing is memoised."""
        memo = self._memo
        if memo is None:
            return self._rows_at(u, pts, regime, speed_hat, heading_hat, w_hat)
        key = (tuple(float(x) for x in u), regime, float(speed_hat), float(heading_hat),
               float(w_hat))
        hit = memo.get(key)
        if hit is None:
            hit = self._rows_at(u, pts, regime, speed_hat, heading_hat, w_hat)
            for a in hit:
                a.setflags(write=False)
            memo[key] = hit
        return hit

    def _rows_at(self, u: np.ndarray, pts: np.ndarray, regime: int, speed_hat: float,
                 heading_hat: float, w_hat: float) -> tuple[np.ndarray, np.ndarray]:
        """g_i(u) and its gradient (n, 3) for every point.

        **Over the realised fraction of the motion** (ruled 2026-10-26, rule 7; amends Q2's
        horizon). The robot realises some fraction lambda in [0, 1] of the predicted motion over
        tau = dt + lag -- slip, the lag, a stop -- and a row must hold whichever: the motion
        predicted at the full horizon is not the conservative case for a return the robot moves
        away from or sweeps with its turn (the simulator acceptance step's unattributed stop,
        2026-10-26). So g_i is the minimum over the declared grid `motion_fractions` of the row at
        p'(lambda) = R(-lambda w tau)(p - lambda tau v); its gradient is the minimising fraction's.
        The field (heading, swept yaw rate) depends on u, not on lambda, so its support is
        evaluated once for all the fractions. `tests/test_pfb.py` bounds the grid's gap to a dense
        minimum over lambda, and `motion_grid_margin` (the bound, subtracted from every row) makes the
        grid's minimum a lower bound of the minimum over all lambda in [0, 1]."""
        tau = self.c.dt + self.c.lag
        vx, vy, w = (float(x) for x in u)
        n = pts.shape[0]
        lam = np.asarray(self.c.motion_fractions, dtype=float)
        jay = np.array([[0.0, -1.0], [1.0, 0.0]])
        phi = -w * lam * tau
        cos_p, sin_p = np.cos(phi), np.sin(phi)
        rots = np.stack([np.stack([cos_p, -sin_p], axis=-1),
                         np.stack([sin_p, cos_p], axis=-1)], axis=1)    # (k, 2, 2)
        shifts = pts[None, :, :] - (lam * tau)[:, None, None] * np.array([vx, vy])[None, None, :]
        q_all = np.einsum("kij,knj->kni", rots, shifts).reshape(-1, 2)  # p' = R(-w f tau)(p - v f tau)
        r_all = np.hypot(q_all[:, 0], q_all[:, 1])
        speed = math.hypot(vx, vy)
        dominant = speed >= speed_hat and speed > 1e-9
        heading = math.atan2(vy, vx) if dominant else heading_hat
        w_field = max(abs(w_hat), abs(w))
        rho, d_psi, d_head, d_w = self._field(regime, np.arctan2(q_all[:, 1], q_all[:, 0]),
                                              heading, w_field)
        g_all = (r_all - rho - self.c.delta - self.c.motion_grid_margin).reshape(len(lam), n)
        best = np.argmin(g_all, axis=0)                                # the minimising fraction
        idx = best * n + np.arange(n)
        g = g_all[best, np.arange(n)]
        q, r = q_all[idx], r_all[idx]
        scale = -lam[best] * tau                                       # (n,)
        rot_sel = rots[best]                                           # each point's own rotation
        grad = np.zeros((n, 3))
        for j in range(2):
            # d p' / d v_j = -f tau R e_j
            dq = scale[:, None] * rot_sel[:, :, j]
            dr = (q * dq).sum(axis=1) / r
            dpsi = (q[:, 0] * dq[:, 1] - q[:, 1] * dq[:, 0]) / r ** 2
            grad[:, j] = dr - d_psi[idx] * dpsi
        # d p' / d w = -f tau (R J) shifted
        dq_dw = scale[:, None] * np.einsum("nij,nj->ni", rot_sel @ jay, shifts[best, np.arange(n)])
        dr = (q * dq_dw).sum(axis=1) / r
        dpsi = (q[:, 0] * dq_dw[:, 1] - q[:, 1] * dq_dw[:, 0]) / r ** 2
        grad[:, 2] = dr - d_psi[idx] * dpsi
        if dominant:
            grad[:, 0] -= d_head[idx] * (-vy / speed ** 2)
            grad[:, 1] -= d_head[idx] * (vx / speed ** 2)
        if abs(w) > abs(w_hat):
            grad[:, 2] -= d_w[idx] * math.copysign(1.0, w)
        return g, grad

    def _linear_rows(self, lin: np.ndarray, pts: np.ndarray, regime: int, rhs: np.ndarray,
                     speed_hat: float, heading_hat: float, w_hat: float
                     ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """The rows about `lin` as A u <= b, each tightened by `lin_margin`, with the yaw rate held
        to a TRUST interval [w - yaw_trust, w + yaw_trust] and described by CHORDS.

        **Why chords** (found 2026-10-23). The field grows convexly in |w| (the sweep's braking term
        is quadratic in the rate) and has a kink where |w| meets |w_hat|, so g is concave in w: a
        tangent over-estimates g away from its point, and the QP chasing a small yaw rate overshot,
        re-linearised, and overshot the other way (3-15 mm misses). On the trust interval the chord
        from w to each end lies BELOW a concave g, and the two chords together describe it
        conservatively and convexly -- the kink included. Translation keeps its tangent (its
        curvature over one step is negligible). Returns (A, b, keep)."""
        eps, dw = self.c.lin_margin, self.c.yaw_trust
        g0, grad = self._rows(lin, pts, regime, speed_hat, heading_hat, w_hat)
        step = np.array([0.0, 0.0, dw])
        gp, _ = self._rows(lin + step, pts, regime, speed_hat, heading_hat, w_hat)
        gm, _ = self._rows(lin - step, pts, regime, speed_hat, heading_hat, w_hat)
        keep = (np.minimum(np.minimum(g0, gp), gm) - rhs) < self.c.row_window
        g0, gp, gm, gxy = g0[keep], gp[keep], gm[keep], grad[keep, :2]
        base = g0 - gxy @ lin[:2] - rhs[keep] - eps
        rows, b = [], []
        for slope in ((gp - g0) / dw, (g0 - gm) / dw):
            rows.append(np.column_stack([-gxy, -slope]))
            b.append(base - slope * lin[2])
        trust = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, -1.0]])
        return (np.vstack([*rows, trust]),
                np.concatenate([*b, [lin[2] + dw, -(lin[2] - dw)]]), keep)

    def _now(self, pts: np.ndarray, regime: int, heading_now: float, w_now: float) -> np.ndarray:
        """h_i at the current motion, in the candidate regime's field."""
        b = np.arctan2(pts[:, 1], pts[:, 0])
        rho = self._field(regime, b, heading_now, w_now)[0]
        # The same offset as the predicted rows, so that holding still keeps its row (h = g at
        # lambda = 0 with the field unchanged) and the decay condition compares like with like.
        return np.hypot(pts[:, 0], pts[:, 1]) - rho - self.c.delta - self.c.motion_grid_margin

    # ------------------------------------------------------------------ the constraints
    def _static(self, regime: int, u_prev: np.ndarray, braking: bool = False
                ) -> tuple[np.ndarray, np.ndarray]:
        """The polytope, the box, the one-step envelope and the regime's speed range: A u <= b.

        **The envelope is the limiter's, in two convex cases** (found 2026-10-23: a single a_max ball
        could not brake hard enough near walls at 0.47 m/s, and fell back). The limiter bounds a step
        that does not reduce speed by a_max dt, and one that does by a_decel dt: so (hold or
        accelerate) |u - u_prev| <= a_max dt, or (`braking`) |u| <= |u_prev| and
        |u - u_prev| <= a_decel dt. Both are inscribed polygons, so every point is reachable."""
        c = self.c
        rows, rhs = [], []
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sw in (-1.0, 1.0):
                    rows.append([sx, sy, sw * c.lever])
                    rhs.append(c.budget)
        for j, lim in enumerate((c.v_max, c.vy_max, c.w_max)):
            e = np.zeros(3)
            e[j] = 1.0
            rows += [e, -e]
            rhs += [lim, lim]
        inscribed = math.cos(math.pi / _RING)
        env = (c.a_decel if braking else c.a_max) * c.dt * inscribed
        lim = self.speed_limit(regime) * inscribed
        if braking:
            lim = min(lim, math.hypot(u_prev[0], u_prev[1]) * inscribed)
        for d in self._ring:
            rows.append([d[0], d[1], 0.0])
            rhs.append(env + d @ u_prev[:2])
            rows.append([d[0], d[1], 0.0])
            rhs.append(lim)
        yaw_env = c.alpha_max * c.dt
        rows += [[0.0, 0.0, 1.0], [0.0, 0.0, -1.0]]
        rhs += [u_prev[2] + yaw_env, -(u_prev[2] - yaw_env)]
        if regime == REST:
            creep_yaw = c.creep_fraction * self.p.rest_yaw
            rows += [[0.0, 0.0, 1.0], [0.0, 0.0, -1.0]]
            rhs += [creep_yaw, creep_yaw]
        return np.asarray(rows, dtype=float), np.asarray(rhs, dtype=float)

    def _reachable_point(self, u_nom: np.ndarray, regime: int, u_prev: np.ndarray,
                         braking: bool = False) -> np.ndarray:
        """A point of the regime's static set near the nominal: the step from u_prev clipped to the
        one-step envelope, then the speed to the regime's limit and the yaw rate to the box, then
        scaled into the wheel polytope (each step keeps the earlier ones satisfied: the sets are
        star-shaped about the origin or about u_prev as used)."""
        c = self.c
        inscribed = math.cos(math.pi / _RING)
        step = u_nom[:2] - u_prev[:2]
        n = float(np.linalg.norm(step))
        cap = (c.a_decel if braking else c.a_max) * c.dt * inscribed * 0.999
        xy = u_prev[:2] + (step * (cap / n) if n > cap else step)
        lim = self.speed_limit(regime) * inscribed * 0.999
        if braking:
            lim = min(lim, math.hypot(u_prev[0], u_prev[1]) * inscribed * 0.999)
        s = float(np.linalg.norm(xy))
        if s > lim:
            xy = xy * (lim / s)
        yaw_env = c.alpha_max * c.dt * 0.999
        w = float(np.clip(u_nom[2], u_prev[2] - yaw_env, u_prev[2] + yaw_env))
        w = float(np.clip(w, -c.w_max, c.w_max))
        if regime == REST:
            cy = c.creep_fraction * self.p.rest_yaw * 0.999
            w = float(np.clip(w, -cy, cy))
        xy = np.clip(xy, [-c.v_max, -c.vy_max], [c.v_max, c.vy_max])
        rim = abs(xy[0]) + abs(xy[1]) + c.lever * abs(w)
        if rim > c.budget:
            k = c.budget / rim * 0.999
            xy, w = xy * k, w * k
        return np.array([xy[0], xy[1], w])

    def _feasible(self, u: np.ndarray, a: np.ndarray, b: np.ndarray, tol: float = 1e-7) -> bool:
        return bool(np.all(a @ u <= b + tol))

    def _solve(self, u_nom: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray | None:
        """min (u - u_nom)' W (u - u_nom) s.t. a u <= b; None if the solver finds it infeasible.
        Clarabel by default (2026-10-24: about 1 ms against OSQP's 12 ms on these 3-variable QPs,
        mostly OSQP's setup; the same answers, `tests/test_pfb.py`). Every answer is then checked
        against the TRUE rows and constraints by the caller, whichever solver produced it."""
        w = np.array([1.0, 1.0, self.c.weight_yaw])
        if self.c.qp_solver == "clarabel":
            import clarabel

            if self._clarabel is None:            # built once: the cost matrix and the settings
                settings = clarabel.DefaultSettings()
                settings.verbose = False
                settings.tol_gap_abs = settings.tol_gap_rel = settings.tol_feas = 1e-10
                self._clarabel = (sp.csc_matrix(np.diag(2.0 * w)), settings)
            cost, settings = self._clarabel
            sol = clarabel.DefaultSolver(cost, -2.0 * w * u_nom, sp.csc_matrix(a), b,
                                         [clarabel.NonnegativeConeT(b.size)], settings).solve()
            if str(sol.status) not in ("Solved", "AlmostSolved"):
                return None
            return np.asarray(sol.x, dtype=float)
        if self.c.qp_solver != "osqp":
            raise ValueError(f"filter.qp_solver must be clarabel or osqp, got {self.c.qp_solver!r}")
        import osqp

        prob = osqp.OSQP()
        prob.setup(P=sp.csc_matrix(np.diag(2.0 * w)), q=-2.0 * w * u_nom,
                   A=sp.csc_matrix(a), l=np.full(b.size, -np.inf), u=b,
                   verbose=False, eps_abs=1e-9, eps_rel=1e-9, max_iter=20000, polishing=True)
        res = prob.solve(raise_error=False)
        if res.info.status not in ("solved", "solved inaccurate"):
            return None
        return np.asarray(res.x, dtype=float)

    # ------------------------------------------------------------------ the filter
    def fallback(self, u_prev: np.ndarray) -> np.ndarray:
        """Maximum braking toward rest within the limiter: translation shortened by a_decel dt
        along itself, yaw rate toward zero by alpha_max dt."""
        c = self.c
        out = np.asarray(u_prev, dtype=float).copy()
        s = math.hypot(out[0], out[1])
        if s > 0.0:
            out[:2] *= max(0.0, 1.0 - c.a_decel * c.dt / s)
        out[2] = math.copysign(max(0.0, abs(out[2]) - c.alpha_max * c.dt), out[2])
        return out

    def filter(self, u_nom, v_hat, u_prev, ranges, bearings, valid, *,
               latched: bool = False) -> PFBResult:
        """One step. Body frame; m/s, rad/s, m, rad. The rows are memoised for this call only."""
        self._memo = {}
        try:
            return self._filter(u_nom, v_hat, u_prev, ranges, bearings, valid, latched=latched)
        finally:
            self._memo = None

    def _filter(self, u_nom, v_hat, u_prev, ranges, bearings, valid, *,
                latched: bool = False) -> PFBResult:
        u_nom = np.asarray(u_nom, dtype=float)
        u_prev = np.asarray(u_prev, dtype=float)
        v_hat = np.asarray(v_hat, dtype=float)
        r = np.asarray(ranges, dtype=float)
        b = np.asarray(bearings, dtype=float)
        ok = np.asarray(valid, dtype=bool)
        self.standstill(v_hat, u_prev)
        if latched:
            if not np.any(u_nom) or self.releasable(u_nom, r, b, ok):
                return PFBResult(u_nom.copy(), "latched", None, False, None, 0,
                                 unobserved=int((~ok).sum()))
            return PFBResult(np.zeros(3), "latched", None, False, "not_releasable", 0,
                             unobserved=int((~ok).sum()))
        res = self._filter_observed(u_nom, v_hat, u_prev, *resolve_unobserved(
            r, b, ok, self.field.footprint.extent)[:1], b)
        res.unobserved = int((~ok).sum())
        if res.infeasible and res.unobserved:
            # A point on the footprint's boundary lies inside every regime's field and cannot leave
            # it in one step, so the step is infeasible because of it: labelled as before.
            res.reason = "unobserved"
        return res

    def _filter_observed(self, u_nom, v_hat, u_prev, r, b) -> PFBResult:
        """The filter on returns every one of which is observed (the unobserved already placed on
        the footprint's boundary)."""
        pts = r[:, None] * np.column_stack([np.cos(b), np.sin(b)])

        speed_hat = math.hypot(v_hat[0], v_hat[1])
        w_hat = float(v_hat[2])
        if speed_hat > 1e-9:
            self._heading = math.atan2(v_hat[1], v_hat[0])
        heading_hat = self._heading
        prev_speed = math.hypot(u_prev[0], u_prev[1])
        heading_now = (math.atan2(u_prev[1], u_prev[0])
                       if prev_speed > speed_hat and prev_speed > 1e-9 else heading_hat)
        w_now = max(abs(w_hat), abs(float(u_prev[2])))
        best: tuple[float, np.ndarray, int] | None = None
        solves = 0
        tried: list[tuple[int, np.ndarray]] = []
        wts = np.array([1.0, 1.0, self.c.weight_yaw])
        for regime in self._regimes(speed_hat, w_hat):
            # (1 - alpha dt) max(h_now, 0): within a field the clearance may decay at most at that
            # rate; INTO a field already breached it must be clear next step -- the decay form alone
            # let a robot at rest switch into band 0 with band 0's field already holding the
            # obstacle, and the certified layer stopped it (found 2026-10-23 by the acceptance
            # machinery: an unattributed stop; rule 7 entry in docs/decision_log.md).
            rhs = (1.0 - self.c.alpha * self.c.dt) * np.maximum(
                self._now(pts, regime, heading_now, w_now), 0.0)
            tried.append((regime, rhs))
            a_s, b_s = self._static(regime, u_prev)
            # The nominal itself, if it already satisfies every TRUE row and constraint here.
            if self._feasible(u_nom, a_s, b_s):
                g, _ = self._rows(u_nom, pts, regime, speed_hat, heading_hat, w_hat)
                if np.all(g >= rhs - 1e-9):
                    head, yaw = self._assumed(u_nom, speed_hat, heading_hat, w_hat)
                    return PFBResult(u_nom.copy(), "nominal", regime, False, None, solves,
                                     head, yaw)
            found = None
            for braking in (False, True):           # the braking case only if holding fails
                a_s, b_s = self._static(regime, u_prev, braking)
                found, n_solves = self._solve_regime(u_nom, pts, regime, rhs, a_s, b_s, u_prev,
                                                     speed_hat, heading_hat, w_hat, braking)
                solves += n_solves
                if found is not None:
                    break
            if found is not None:
                cost = float(((found - u_nom) ** 2 * wts).sum())
                if best is None or cost < best[0]:
                    best = (cost, found, regime)
        if best is None:
            # Before declaring the step infeasible, the maximum-braking command itself against the
            # TRUE rows: the linearised rows are tightened by `lin_margin`, which exceeds the decay
            # the CBF allows, alpha dt h_now, once h_now < lin_margin / (alpha dt) (5 mm), so the QP
            # can miss that holding still is safe (found 2026-10-23: at rest 6 mm from the at-rest
            # field, 62 % of a blocked route's steps were labelled infeasible). Same command as the
            # fallback; labelled by whether it satisfies the CBF, so `pfb_infeasible` counts only
            # steps where no command checked did.
            u_brake = self.fallback(u_prev)
            for regime, rhs in tried:
                a_s, b_s = self._static(regime, u_prev, braking=True)
                if not self._feasible(u_brake, a_s, b_s):
                    continue
                g, _ = self._rows(u_brake, pts, regime, speed_hat, heading_hat, w_hat)
                if np.all(g >= rhs):
                    head, yaw = self._assumed(u_brake, speed_hat, heading_hat, w_hat)
                    return PFBResult(u_brake, "brake", regime, False, None, solves, head, yaw)
            return PFBResult(u_brake, "fallback", None, True, "infeasible", solves)
        head, yaw = self._assumed(best[1], speed_hat, heading_hat, w_hat)
        return PFBResult(best[1], "filtered", best[2], False, None, solves, head, yaw)

    def _solve_regime(self, u_nom, pts, regime, rhs, a_s, b_s, u_prev, speed_hat, heading_hat,
                      w_hat, braking) -> tuple[np.ndarray | None, int]:
        """One regime and envelope case: linearise about the nominal PROJECTED onto the case's
        reachable set (the rows are accurate near their linearisation point, and the feasible
        region lies around the previous command -- found 2026-10-23: linearised at a nominal
        0.17 m/s outside the envelope, the tightened rows excluded it), solve, check the TRUE rows,
        re-linearise once (Q3): about the solution, or, if the first QP was infeasible, about the
        braking command."""
        lin = self._reachable_point(u_nom, regime, u_prev, braking)
        solves = 0
        for _ in range(self.c.linearisations):
            a_rows, b_rows, _keep = self._linear_rows(lin, pts, regime, rhs, speed_hat,
                                                      heading_hat, w_hat)
            a, bb = np.vstack([a_s, a_rows]), np.concatenate([b_s, b_rows])
            sol = self._solve(u_nom, a, bb)
            solves += 1
            if sol is None:
                # The rows linearised toward the nominal can exclude a safe region on the far side
                # (found 2026-10-23: at rest beside an obstacle, rows about a creep INTO it ruled out
                # backing away). The one re-linearisation is then about the braking command, the
                # state the fallback would reach, instead of giving up.
                lin = self._reachable_point(self.fallback(u_prev), regime, u_prev, braking)
                continue
            g_true, _ = self._rows(sol, pts, regime, speed_hat, heading_hat, w_hat)
            if np.all(g_true >= rhs) and self._feasible(sol, a_s, b_s):
                return sol, solves
            lin = sol
        return None, solves

    @staticmethod
    def _assumed(u: np.ndarray, speed_hat: float, heading_hat: float, w_hat: float
                 ) -> tuple[float, float]:
        """The field heading and |yaw rate| a command's rows assumed (as `_rows` composes them)."""
        speed = math.hypot(float(u[0]), float(u[1]))
        heading = (math.atan2(float(u[1]), float(u[0])) if speed >= speed_hat and speed > 1e-9
                   else heading_hat)
        return heading, max(abs(w_hat), abs(float(u[2])))

    # ------------------------------------------------------------------ under the latch
    def releasable(self, u, ranges, bearings, valid) -> bool:
        """Whether the restart condition could release `u` (Q7): a creep whose at-rest field is clear
        or that strictly recedes from every point inside it (A24), or a faster twist whose own band
        field -- along it, swept for its yaw rate -- is clear. An unobserved bin blocks."""
        ok = np.asarray(valid, dtype=bool)
        if not ok.all():
            return False
        r = np.asarray(ranges, dtype=float)
        b = np.asarray(bearings, dtype=float)
        vx, vy, w = (float(x) for x in u)
        speed = math.hypot(vx, vy)
        p = self.p
        if speed <= p.rest_speed and abs(w) <= p.rest_yaw:
            field = self.field.extent(b, 0.0, 0.0, abs(w), at_rest=True)
            inside = r < field
            if not inside.any():
                return True
            return self._recedes(vx, vy, w, r[inside], b[inside], field[inside])
        heading = math.atan2(vy, vx) if speed > 1e-9 else self._heading
        band = band_of(speed, p)
        return not bool(np.any(r < self.field.band_extent(b, band, heading, abs(w))))

    def _recedes(self, vx, vy, w, r, b, field) -> bool:
        """Strictly receding from every point inside the at-rest field (A24): the translation has a
        component away from each, and every footprint boundary point within the field's thickness
        of each moves away from it (a rotation swinging any part toward it fails)."""
        speed = math.hypot(vx, vy)
        if speed < 1e-9:
            return False
        u = np.column_stack([np.cos(b), np.sin(b)])
        if np.any(u @ np.array([vx, vy]) >= -1e-9 * speed):
            return False
        boundary = self.field.footprint.boundary
        edge = np.asarray(self.field.footprint.extent(b), dtype=float)
        for point, reach in zip(r[:, None] * u, field - edge, strict=True):
            gap = point[None, :] - boundary
            dist = np.hypot(gap[:, 0], gap[:, 1])
            near = dist <= dist.min() + max(float(reach), 0.0)
            q = boundary[near]
            vq = np.column_stack([vx - w * q[:, 1], vy + w * q[:, 0]])
            if np.any((vq * gap[near]).sum(axis=1) >= 0.0):
                return False
        return True
