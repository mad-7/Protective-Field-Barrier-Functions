"""A generic discrete-time control barrier function filter. Pure functions, no simulator.

This is the reference form of the construction the PFB specialises (`docs/theory/02_control_
barrier_function.md`): given a nominal command and barrier rows that are linear in the command,
return the closest admissible command. The PFB (`pfb.py`) does not call this module -- it builds
its own rows from the scan and solves them with extra structure (regimes, chords in yaw rate, the
limiter's envelope) -- so the two stay independent; this module is the known-answer check on the
idea itself and is what a new barrier would start from.

**Discrete-time condition.** A barrier h keeps the safe set {h >= 0} forward invariant if, at
every step of length dt, h(x_next) >= (1 - alpha dt) h(x_now), with 0 < alpha dt <= 1.

**The filter.** Minimise (u - u_nom)^T W (u - u_nom) subject to A u >= b, with W positive
diagonal. Solved exactly by active-set enumeration, which is exact and cheap for the few rows and
at most three command components used here (vx, vy, w).
"""
from __future__ import annotations

import itertools

import numpy as np


def barrier_floor(h_now: float, alpha: float, dt: float) -> float:
    """The value h(x_next) must reach: (1 - alpha dt) * max(h_now, 0). Clipped at zero, so a state
    that is unsafe now must be safe after one step (never entered unsafe)."""
    if not 0.0 < alpha * dt <= 1.0:
        raise ValueError(f"alpha * dt must lie in (0, 1], got {alpha * dt}")
    return (1.0 - alpha * dt) * max(float(h_now), 0.0)


def filter_command(u_nom: np.ndarray, rows: np.ndarray, bounds: np.ndarray,
                   weights: np.ndarray | None = None, tol: float = 1e-9) -> tuple[np.ndarray, bool]:
    """The command closest to `u_nom` (in the W-norm) with `rows @ u >= bounds`.

    Returns (u, feasible). If u_nom already satisfies every row it is returned unchanged. If no
    command satisfies them all, returns (u_nom, False): the caller decides the fallback (the PFB
    brakes), this function never invents one.
    """
    u_nom = np.asarray(u_nom, dtype=float)
    rows = np.atleast_2d(np.asarray(rows, dtype=float))
    bounds = np.asarray(bounds, dtype=float).reshape(-1)
    n = u_nom.size
    w = np.ones(n) if weights is None else np.asarray(weights, dtype=float)
    if rows.shape != (bounds.size, n) or w.shape != (n,) or np.any(w <= 0):
        raise ValueError("rows must be (m, n), bounds (m,), weights (n,) positive")
    if np.all(rows @ u_nom >= bounds - tol):
        return u_nom.copy(), True
    best, best_cost = None, np.inf
    for k in range(1, min(n, rows.shape[0]) + 1):
        for active in itertools.combinations(range(rows.shape[0]), k):
            a, b = rows[list(active)], bounds[list(active)]
            # Project onto the intersection of the active planes in the W-metric.
            winv = 1.0 / w
            gram = (a * winv) @ a.T
            if abs(np.linalg.det(gram)) < 1e-14:
                continue
            lam = np.linalg.solve(gram, b - a @ u_nom)
            if np.any(lam < -tol):
                continue
            u = u_nom + winv * (a.T @ lam)
            if np.all(rows @ u >= bounds - tol):
                cost = float(np.sum(w * (u - u_nom) ** 2))
                if cost < best_cost:
                    best, best_cost = u, cost
    if best is None:
        return u_nom.copy(), False
    return best, True
