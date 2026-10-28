"""Shared pieces of the power simulations for H1 and H2 (corrected two-level analysis, 2026-10-03).

H1's arms and stop counts follow the supervisor's model (`pfbnav/eval/rate_ratio.py`). H2's
success model extends the one behind the 2026-09-29 H2 figures -- L4's success probability per
scenario uniform on [0.3, 0.8], L5 adding the effect delta on the probability scale -- with a
per-seed shift on the LOGIT scale, constant across scenarios like H1's seed multiplier. Its SD is a
choice of Claude's, flagged for the supervisor: it is swept, not assumed.
"""
from __future__ import annotations

from collections.abc import Iterator, Sequence
from typing import Any

import numpy as np
from scipy.special import expit, logit

from pfbnav.eval.power import Arm, gamma_multiplier
from pfbnav.eval.rate_ratio import stop_counts

SCENARIOS_H1 = ("all_at_margin", "one_at_margin_classical", "one_at_margin_learned")


def h1_arms(scenario: str, rate: float, p: dict[str, Any]) -> tuple[Arm, list[Arm]]:
    """L5 and the seven comparators for one H1 cell; `equal_rates` gives every arm `rate`."""
    names = [(n, False) for n in p["classical"]] + [(n, True) for n in p["learned"]]
    if scenario == "equal_rates":
        return Arm("L5", rate, True), [Arm(n, rate, k) for n, k in names]
    l5 = p["l5_ratio"] * rate
    other = p["secondary_other_ratio_to_l5"] * l5
    margin = {"all_at_margin": {n for n, _ in names},
              "one_at_margin_classical": {p["classical"][0]},
              "one_at_margin_learned": {p["learned"][0]}}[scenario]
    return Arm("L5", l5, True), [Arm(n, rate if n in margin else other, k) for n, k in names]


def h1_chunks(rng: np.random.Generator, lengths: Sequence[float], n_pairs: int, l5: Arm,
              comps: list[Arm], difficulty_cv: float, seed_cv: float, max_seeds: int,
              reps: int, chunk: int = 500) -> Iterator[tuple[np.ndarray, dict[str, np.ndarray]]]:
    """(route lengths, {arm: stop counts}) in chunks; learned arms carry `max_seeds` seeds."""
    pool = np.asarray(lengths, dtype=float)
    done = 0
    while done < reps:
        r = min(chunk, reps - done)
        d = rng.choice(pool, size=(r, n_pairs), replace=True)
        diff = gamma_multiplier(rng, difficulty_cv, (r, n_pairs))
        yield d, {a.name: stop_counts(rng, a, d, diff, max_seeds, seed_cv) for a in [l5, *comps]}
        done += r


def success_outcomes(rng: np.random.Generator, base: np.ndarray, delta: float, seed_sd: float,
                     n_seeds: int) -> np.ndarray:
    """Success indicators (reps, n_seeds, n) for one learned arm.

    `base` (reps, n) is L4's per-scenario success probability; the arm's is `base + delta`,
    clipped to [0.01, 0.99], then shifted on the logit scale by one normal draw per seed
    (SD `seed_sd`), the same draw on every scenario.
    """
    p = np.clip(base + delta, 0.01, 0.99)
    shift = (rng.normal(0.0, seed_sd, size=(base.shape[0], n_seeds, 1)) if seed_sd > 0
             else np.zeros((base.shape[0], n_seeds, 1)))
    prob = expit(logit(p)[:, None, :] + shift)
    return (rng.random(prob.shape) < prob).astype(np.float64)
