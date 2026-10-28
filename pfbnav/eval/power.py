"""Monte Carlo power of H1 under the frozen analysis (supervisor's model, 2026-10-02). Pure.

SUPERSEDED for H1's inference by `pfbnav/eval/rate_ratio.py` (ruled 2026-10-02 after this module's
run, `runs/power/H1_POWER_20261002T210848Z_seed0`, showed the signed-rank test unfit). Kept because
that run's provenance points here, and because `Arm` and `gamma_multiplier` are shared.

H1 (preregistration/endpoints.yaml): L5 triggers fewer protective stops per 100 m than EVERY
comparator, by intersection-union -- each of seven paired, two-sided Wilcoxon signed-rank tests
(unit: scenario; learned methods averaged over their seeds) must reject in the predicted direction.

The data model, as ruled (2026-10-02):
  * a scenario's route length is drawn from the pilot set's planned route lengths;
  * its shared difficulty is a gamma multiplier, mean 1, coefficient of variation `difficulty_cv`,
    applied to every arm alike;
  * a protective-stop count is Poisson with mean `rate * difficulty * length / 100`;
  * a LEARNED arm also carries one gamma multiplier per training seed (mean 1, CV `seed_cv`),
    constant across scenarios -- a seed's policy is better or worse everywhere -- and its value on a
    scenario is the mean over its seeds of count / (length / 100); a CLASSICAL arm runs once;
  * every comparator is simulated jointly against the SAME L5 values, as the comparisons share them.

Zero differences use **Pratt's method** (zeros kept in the ranking; ruled 2026-10-02 as a
clarification of the frozen "Wilcoxon signed-rank", to be recorded in A14). The p-value is SciPy's
normal approximation with the tie correction and no continuity correction, which is what SciPy uses
by default at these sizes; a known-answer test pins it against a hand calculation.

**Not modelled:** H1's success also requires "each effect exceeds its paired detectability floor",
and that floor is not defined anywhere in the repository. The power here is the probability that all
seven comparisons are significant in the predicted direction -- an UPPER bound on the probability of
H1's success once the floor is defined.
"""
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike
from scipy import stats


def gamma_multiplier(rng: np.random.Generator, cv: float, size) -> np.ndarray:
    """Gamma draws with mean 1 and coefficient of variation `cv` (shape 1/cv^2, scale cv^2).
    `cv` 0 gives exactly 1."""
    if cv < 0.0:
        raise ValueError("cv must be non-negative")
    if cv == 0.0:
        return np.ones(size)
    return rng.gamma(1.0 / cv**2, cv**2, size=size)


@dataclass(frozen=True)
class Arm:
    name: str
    rate_per_100m: float     # the arm's true protective-stop rate on a scenario of difficulty 1
    learned: bool            # learned: seed-averaged over n_seeds; classical: one run


def stops_per_100m(rng: np.random.Generator, arm: Arm, lengths_m: np.ndarray,
                   difficulty: np.ndarray, n_seeds: int, seed_cv: float) -> np.ndarray:
    """The endpoint per (replicate, scenario), for one arm. `lengths_m` and `difficulty` are
    (reps, n)."""
    per_100 = lengths_m / 100.0
    if not arm.learned:
        return rng.poisson(arm.rate_per_100m * difficulty * per_100) / per_100
    reps = lengths_m.shape[0]
    seeds = gamma_multiplier(rng, seed_cv, (reps, 1, n_seeds))      # constant across scenarios
    lam = arm.rate_per_100m * (difficulty * per_100)[:, :, None] * seeds
    return (rng.poisson(lam) / per_100[:, :, None]).mean(axis=2)


def wilcoxon_pratt(differences: ArrayLike) -> tuple[np.ndarray, np.ndarray]:
    """Two-sided paired Wilcoxon signed-rank along the last axis, Pratt's zero handling.

    Returns `(p, favours_positive)`: the two-sided p-value, and whether the signed-rank sum is
    larger for positive differences (comparator minus L5 > 0: the predicted direction).
    """
    d = np.asarray(differences, dtype=float)
    res = stats.wilcoxon(d, axis=-1, zero_method="pratt", method="approx", correction=False)
    pvalue = np.asarray(res.pvalue)  # type: ignore[attr-defined]
    ranks = stats.rankdata(np.abs(d), axis=-1)
    r_plus = np.where(d > 0, ranks, 0.0).sum(axis=-1)
    r_minus = np.where(d < 0, ranks, 0.0).sum(axis=-1)
    return pvalue, r_plus > r_minus


def simulate_h1(rng: np.random.Generator, pilot_lengths_m: Sequence[float], n_pairs: int,
                l5: Arm, comparators: Sequence[Arm], *, difficulty_cv: float, seed_cv: float,
                n_seeds: int, levels: Sequence[float], reps: int) -> dict:
    """Power of H1's intersection-union test at each significance level in `levels`.

    Returns {"power": {level: P(all comparators reject in the predicted direction)},
             "marginal": {level: {name: P(that comparator rejects in the predicted direction)}},
             "opposite": {level: {name: P(it rejects in the OPPOSITE direction)}}}.
    """
    pool = np.asarray(pilot_lengths_m, dtype=float)
    if pool.size == 0 or np.any(pool <= 0.0):
        raise ValueError("route lengths must be positive")
    lengths = rng.choice(pool, size=(reps, n_pairs), replace=True)
    difficulty = gamma_multiplier(rng, difficulty_cv, (reps, n_pairs))
    base = stops_per_100m(rng, l5, lengths, difficulty, n_seeds, seed_cv)
    rejected = {lv: np.ones(reps, dtype=bool) for lv in levels}
    marginal: dict[float, dict[str, float]] = {lv: {} for lv in levels}
    opposite: dict[float, dict[str, float]] = {lv: {} for lv in levels}
    for arm in comparators:
        other = stops_per_100m(rng, arm, lengths, difficulty, n_seeds, seed_cv)
        p, right_way = wilcoxon_pratt(other - base)
        for lv in levels:
            hit = (p <= lv) & right_way
            marginal[lv][arm.name] = float(hit.mean())
            opposite[lv][arm.name] = float(((p <= lv) & ~right_way).mean())
            rejected[lv] &= hit
    return {"power": {lv: float(rejected[lv].mean()) for lv in levels}, "marginal": marginal,
            "opposite": opposite}


def smallest_sufficient(power_by_size: dict[int, float], target: float) -> int | None:
    """The smallest size whose power reaches `target`, or None if none does."""
    for size in sorted(power_by_size):
        if power_by_size[size] >= target:
            return size
    return None
