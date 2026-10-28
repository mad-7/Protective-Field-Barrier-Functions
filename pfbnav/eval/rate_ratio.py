"""H1's estimand and its inference, as ruled on 2026-10-02 (A14). Pure.

**Estimand:** the rate ratio of protective stops, total stops / total distance per arm, L5 over a
comparator. **Inference:** a bootstrap confidence interval on its log; scenarios are resampled
JOINTLY across arms (the pairing), and

  * **two-level** (confirmatory, method-level): a learned arm's seeds are also resampled, within the
    arm, independently of every other arm's; a classical arm resamples scenarios only;
  * **policy-scoped** (secondary): seeds are held fixed; scenarios only.

A comparison rejects in L5's favour when the CI's upper end is below 0, against L5 when its lower end
is above 0. This replaces the paired signed-rank test, which leaned against a seed-averaged L5 when
the comparator ran once (decision log, 2026-10-02), and makes the Pratt ruling moot.

**Continuity correction, declared:** 0.5 is added to each arm's TOTAL stops in every estimate, so a
zero total has a finite log. At FIXED counts it shifts arms of unequal exposure differently
(`continuity_bias`: L5 summed over K seeds against one classical run). But averaged over Poisson
counts it does the opposite of biasing: log of a Poisson total is biased low by about 1/(2S), and the
0.5 cancels that, for each arm whatever its exposure. Measured at equal rates and seed CV 0
(`runs/power/H1_RR_20261002T220803Z_seed0`): the mean log rate ratio is within +-0.002 of zero in
every cell, against classical and learned comparators alike. The fixed-count prediction (-0.006 to
-0.012 there) was wrong as a prediction of bias; corrected 2026-10-02.

**The normal approximation** used for the power grid is centred on the point estimate with the
variance the bootstrap itself would have for the LINEARISED statistic -- exact formulas below, checked
against brute-force resampling by the tests and against the full bootstrap by the power script at
selected cells. Arrays: stops `s` are (reps, K, n) -- K seeds (1 for a classical arm), n scenarios;
route lengths `d` are (reps, n), the same for every seed of every arm (each episode covers its route).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import stats

from pfbnav.eval.power import Arm, gamma_multiplier

CONTINUITY = 0.5   # stops added to each arm's total; ruled (supervisor's sketch, 2026-10-02)
POLICY, TWO_LEVEL = "policy_scoped", "two_level"


def stop_counts(rng: np.random.Generator, arm: Arm, lengths_m: np.ndarray,
                difficulty: np.ndarray, n_seeds: int, seed_cv: float) -> np.ndarray:
    """Protective-stop counts per (replicate, seed, scenario). A classical arm has one 'seed'.

    Poisson with mean rate x difficulty x seed multiplier x length / 100; a learned arm's seed
    multiplier is gamma (mean 1, CV `seed_cv`), constant across scenarios (ruled 2026-10-02).
    """
    k = n_seeds if arm.learned else 1
    lam = arm.rate_per_100m * (difficulty * lengths_m / 100.0)[:, None, :]
    if arm.learned:
        lam = lam * gamma_multiplier(rng, seed_cv, (lengths_m.shape[0], k, 1))
    return rng.poisson(np.broadcast_to(lam, (lengths_m.shape[0], k, lengths_m.shape[1])))


@dataclass(frozen=True)
class _Terms:
    total: np.ndarray       # (reps,) stops + continuity
    distance: np.ndarray    # (reps,) metres over all seeds and scenarios
    col: np.ndarray         # (reps, n) residuals summed over seeds
    row: np.ndarray         # (reps, K) residuals summed over scenarios
    frob: np.ndarray        # (reps,) sum of squared residuals
    k: int


def _terms(s: np.ndarray, d: np.ndarray) -> _Terms:
    k = s.shape[1]
    total = s.sum(axis=(1, 2)) + CONTINUITY
    distance = k * d.sum(axis=1)
    e = s / total[:, None, None] - d[:, None, :] / distance[:, None, None]
    return _Terms(total, distance, e.sum(axis=1), e.sum(axis=2), (e**2).sum(axis=(1, 2)), k)


def arm_terms(s: np.ndarray, d: np.ndarray) -> _Terms:
    """One arm's totals and residuals; computed once per arm and reused for every comparison."""
    return _terms(s, d)


def log_rate_ratio_terms(a: _Terms, b: _Terms) -> np.ndarray:
    return np.log(a.total / a.distance) - np.log(b.total / b.distance)


def log_rate_ratio(s_a: np.ndarray, s_b: np.ndarray, d: np.ndarray) -> np.ndarray:
    """log((S_a + 0.5) / D_a) - log((S_b + 0.5) / D_b), per replicate."""
    return log_rate_ratio_terms(_terms(s_a, d), _terms(s_b, d))


def variance_terms(a: _Terms, b: _Terms, mode: str) -> np.ndarray:
    """`bootstrap_variance` from precomputed terms."""
    if mode not in (POLICY, TWO_LEVEL):
        raise ValueError(f"unknown mode {mode!r}")
    n = a.col.shape[1]

    def var(t: _Terms) -> np.ndarray:
        scen = (t.col**2).sum(axis=1)
        total = t.col.sum(axis=1)
        if mode == POLICY:
            return scen - total**2 / n
        c = 1.0 - 1.0 / t.k
        return (t.frob + c * scen + (1.0 - 1.0 / n) * ((t.row**2).sum(axis=1) + c * total**2)
                - total**2)

    cov = (a.col * b.col).sum(axis=1) - a.col.sum(axis=1) * b.col.sum(axis=1) / n
    return var(a) + var(b) - 2.0 * cov


def bootstrap_variance(s_a: np.ndarray, s_b: np.ndarray, d: np.ndarray, mode: str) -> np.ndarray:
    """The bootstrap variance of the linearised log rate ratio, per replicate.

    One arm's linearised term under resampling is T = sum_k sum_i w_k u_i e_ki, with seed counts w
    (multinomial K, 1/K; or all 1 when seeds are fixed) and scenario counts u (multinomial n, 1/n),
    shared by every arm. With col = e summed over seeds, row = e summed over scenarios, and
    t = the sum of all e (not zero: the continuity correction makes it -0.5 / total), exactly:
        seeds fixed:     Var T = |col|^2 - t^2 / n
        seeds resampled: Var T = |e|^2 + c |col|^2 + (1 - 1/n) (|row|^2 + c t^2) - t^2,
                         c = 1 - 1/K
    and two arms covary only through the shared scenarios: col_a . col_b - t_a t_b / n.
    """
    if mode not in (POLICY, TWO_LEVEL):
        raise ValueError(f"unknown mode {mode!r}")
    return variance_terms(_terms(s_a, d), _terms(s_b, d), mode)


def verdict_terms(a: _Terms, b: _Terms, mode: str, level: float,
                  estimate: np.ndarray | None = None):
    """`normal_verdict` from precomputed terms. `estimate` defaults to the log rate ratio; pass
    the difference of means for a success rate."""
    est = log_rate_ratio_terms(a, b) if estimate is None else estimate
    half = stats.norm.isf(level / 2.0) * np.sqrt(variance_terms(a, b, mode))
    return est + half < 0.0, est - half > 0.0


def normal_verdict(s_a, s_b, d, mode: str, level: float) -> tuple[np.ndarray, np.ndarray]:
    """(rejects with arm a LOWER, rejects with arm a HIGHER) at two-sided `level`, per replicate,
    by the normal approximation on the log rate ratio of a over b."""
    return verdict_terms(_terms(s_a, d), _terms(s_b, d), mode, level)


def bootstrap_log_rr(rng: np.random.Generator, arms: dict[str, np.ndarray], d: np.ndarray,
                     mode: str, draws: int) -> dict[str, np.ndarray]:
    """The full bootstrap for ONE replicate: each arm's log rate (stops + 0.5 over distance) in
    `draws` resamples. `arms` maps names to (K, n) counts; `d` is (n,). Scenarios are drawn once per
    resample and shared by every arm; seeds are drawn per arm (two-level) or kept (policy-scoped).
    Differences of the returned arrays are bootstrap draws of log rate ratios."""
    n = d.shape[0]
    u = rng.multinomial(n, np.full(n, 1.0 / n), size=draws)          # (draws, n), shared
    out = {}
    for name, s in arms.items():
        k = s.shape[0]
        if mode == TWO_LEVEL and k > 1:
            w = rng.multinomial(k, np.full(k, 1.0 / k), size=draws)  # (draws, K), this arm only
        else:
            w = np.ones((draws, k))
        total = ((w @ s) * u).sum(axis=1) + CONTINUITY
        distance = w.sum(axis=1) * (u @ d)
        out[name] = np.log(total / distance)
    return out


def percentile_verdict(draws: np.ndarray, level: float) -> tuple[bool, bool]:
    """(upper end < 0, lower end > 0) of the percentile CI at two-sided `level`."""
    lo, hi = np.quantile(draws, [level / 2.0, 1.0 - level / 2.0])
    return bool(hi < 0.0), bool(lo > 0.0)


def continuity_bias(stops_per_run: float, k_a: int, k_b: int) -> float:
    """The log rate ratio the correction alone produces at FIXED counts, when both arms record
    `stops_per_run` in total per run over the same routes, arm a summed over `k_a` runs and arm b
    over `k_b`. Not the expected bias: over Poisson counts the correction cancels the log's own
    downward bias (module docstring), and the measured bias at equal rates is about zero."""
    return float(np.log((k_a * stops_per_run + CONTINUITY) / (k_a * stops_per_run))
                 - np.log((k_b * stops_per_run + CONTINUITY) / (k_b * stops_per_run)))


# ---------------------------------------------------------------------------------------------
# The small-seed correction (ruled 2026-10-03): seed-level variance x K/(K-1), t quantiles,
# Welch-Satterthwaite degrees of freedom.
# ---------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Components:
    """The two-level bootstrap variance of a comparison, split by its source, per replicate.

    `scenario` is the paired variance from resampling scenarios with seeds held (the
    policy-scoped variance); `seed_a` and `seed_b` are each arm's expected variance from
    resampling its own seeds, E_u Var_w, which is 0 for an arm run once. Their sum is exactly the
    uncorrected two-level variance (`variance_terms(..., TWO_LEVEL)`).
    """

    scenario: np.ndarray
    seed_a: np.ndarray
    seed_b: np.ndarray
    k_a: int
    k_b: int
    n: int


def _seed_component(t: _Terms, n: int) -> np.ndarray:
    """E_u Var_w of one arm's linearised term: |e|^2 + (1 - 1/n)|row|^2
    - (|col|^2 + (1 - 1/n) t^2) / K. Zero for K = 1 (then |e|^2 = |col|^2 and row = t)."""
    if t.k == 1:
        return np.zeros_like(t.frob)
    total = t.col.sum(axis=1)
    c = 1.0 - 1.0 / n
    v = t.frob + c * (t.row**2).sum(axis=1) - ((t.col**2).sum(axis=1) + c * total**2) / t.k
    return np.maximum(v, 0.0)


def components(a: _Terms, b: _Terms) -> Components:
    n = a.col.shape[1]
    return Components(variance_terms(a, b, POLICY), _seed_component(a, n),
                      _seed_component(b, n), a.k, b.k, n)


def welch_satterthwaite(parts: list[tuple[np.ndarray, int]]) -> np.ndarray:
    """Degrees of freedom of a sum of independent variance estimates, each with its own df:
    (sum v)^2 / sum(v^2 / df). Parts that are exactly zero contribute nothing."""
    total = sum(v for v, _ in parts)
    denominator = sum(v**2 / df for v, df in parts)
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(denominator > 0.0, total**2 / denominator, np.inf)


WS, FLAT = "welch_satterthwaite", "flat_k_minus_1"


def corrected_interval(c: Components, level: float,
                       classical_df: str = WS) -> tuple[np.ndarray, np.ndarray]:
    """(variance, half-width) of the corrected interval at two-sided `level`: each seed part
    scaled by K/(K-1); t quantile with Welch-Satterthwaite df over the scenario part (n - 1 df)
    and each learned arm's seed part (K - 1 df).

    `classical_df` decides only a comparison between a learned and a classical arm (ruled
    2026-10-03): Welch-Satterthwaite as above, adopted if it passes calibration across the grid;
    otherwise the fallback, df = K - 1 of the learned arm outright. Learned against learned is
    always Welch-Satterthwaite.
    """
    if classical_df not in (WS, FLAT):
        raise ValueError(f"unknown df rule {classical_df!r}")
    parts: list[tuple[np.ndarray, int]] = [(c.scenario, c.n - 1)]
    for seed, k in ((c.seed_a, c.k_a), (c.seed_b, c.k_b)):
        if k > 1:
            parts.append((seed * k / (k - 1.0), k - 1))
    var = sum(v for v, _ in parts)
    learned = [k for k in (c.k_a, c.k_b) if k > 1]
    if classical_df == FLAT and len(learned) == 1:
        df = np.full_like(var, float(learned[0] - 1))
    else:
        df = welch_satterthwaite(parts)
    return var, stats.t.isf(level / 2.0, df) * np.sqrt(var)


def corrected_verdict(a: _Terms, b: _Terms, level: float,
                      estimate: np.ndarray | None = None,
                      classical_df: str = WS) -> tuple[np.ndarray, np.ndarray]:
    """(rejects with a LOWER, rejects with a HIGHER), corrected two-level. `estimate` defaults to
    the log rate ratio of a over b; pass the difference of means for a success rate (H2)."""
    est = log_rate_ratio_terms(a, b) if estimate is None else estimate
    _, half = corrected_interval(components(a, b), level, classical_df)
    return est + half < 0.0, est - half > 0.0


def mean_terms(s: np.ndarray) -> _Terms:
    """Terms for a MEAN of per-(seed, scenario) outcomes, e.g. success (H2): the statistic
    sum(w u s) / sum(w u), linearised as e_ki = (s_ki - M) / (K n). The denominator is the same in
    every resample (K n), so the statistic is exactly linear and the variance formulas are exact.
    `total` and `distance` hold sum(s) and K n, so total / distance is the mean."""
    s = np.asarray(s, dtype=float)
    k, n = s.shape[1], s.shape[2]
    total = s.sum(axis=(1, 2))
    count = np.full(s.shape[0], float(k * n))
    e = (s - (total / count)[:, None, None]) / count[:, None, None]
    return _Terms(total, count, e.sum(axis=1), e.sum(axis=2), (e**2).sum(axis=(1, 2)), k)


def mean_difference(a: _Terms, b: _Terms) -> np.ndarray:
    return a.total / a.distance - b.total / b.distance


# ---------------------------------------------------------------------------------------------
# The confirmatory interval (A14a; ruled 2026-10-26: the closed-form t interval on the log rate
# ratio, scenario and seed variance combined with K/(K-1), the ruled df; the bootstrap was
# verification only). tests/test_a14a_interval.py: equal to the code that passed calibration.
# ---------------------------------------------------------------------------------------------
def confirmatory_interval(a: _Terms, b: _Terms, level: float,
                          estimate: np.ndarray | None = None) -> dict[str, np.ndarray | str]:
    """The A14a interval of arm a over arm b at two-sided `level` (the comparison's Holm level):
    estimate +- t(df) sqrt(V_scenario + sum K/(K-1) V_seed). df: K - 1 of the learned arm against a
    classical arm (flat, calibrated 2026-10-03, CORRECTED_20261003T181634Z); Welch-Satterthwaite
    between two learned arms (and for H2's success-rate difference, passed as `estimate` with
    `mean_terms`). Two classical arms are not a confirmatory comparison and are refused."""
    learned = [t.k for t in (a, b) if t.k > 1]
    if not learned:
        raise ValueError("no learned arm: two classical arms are not a confirmatory comparison")
    rule = FLAT if len(learned) == 1 else WS
    est = log_rate_ratio_terms(a, b) if estimate is None else estimate
    var, half = corrected_interval(components(a, b), level, rule)
    return {"estimate": est, "lower": est - half, "upper": est + half, "variance": var,
            "df_rule": rule}
