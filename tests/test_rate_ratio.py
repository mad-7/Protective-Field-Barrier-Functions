"""Known-answer tests for H1's rate-ratio inference (A14 rulings, 2026-10-02; hard rule 6)."""
import math

import numpy as np
import pytest

from pfbnav.eval.power import Arm, gamma_multiplier
from pfbnav.eval.rate_ratio import (
    CONTINUITY,
    POLICY,
    TWO_LEVEL,
    _terms,
    arm_terms,
    bootstrap_log_rr,
    bootstrap_variance,
    components,
    continuity_bias,
    corrected_interval,
    corrected_verdict,
    log_rate_ratio,
    mean_difference,
    mean_terms,
    normal_verdict,
    percentile_verdict,
    stop_counts,
    variance_terms,
    verdict_terms,
    welch_satterthwaite,
)


def test_the_log_rate_ratio_known_answer():
    """Arm a: two seeds, 3 + 1 and 0 + 2 stops over routes of 10 and 30 m -> (6 + 0.5) / 80 m.
    Arm b: one run, 2 and 2 stops -> (4 + 0.5) / 40 m."""
    d = np.array([[10.0, 30.0]])
    s_a = np.array([[[3, 1], [0, 2]]])
    s_b = np.array([[[2, 2]]])
    want = math.log(6.5 / 80.0) - math.log(4.5 / 40.0)
    assert log_rate_ratio(s_a, s_b, d)[0] == pytest.approx(want, abs=1e-12)


def test_the_continuity_bias_known_answer_and_its_direction():
    """2 stops per run, L5 over 5 seeds against one classical run: log(10.5/10) - log(2.5/2)
    = 0.048790 - 0.223144 = -0.174353, a lean TOWARD the five-seed arm. Equal exposure: none."""
    assert continuity_bias(2.0, 5, 1) == pytest.approx(-0.174353, abs=1e-6)
    assert continuity_bias(2.0, 5, 5) == 0.0
    assert continuity_bias(200.0, 5, 1) == pytest.approx(-0.002, abs=1e-4)


def _data(seed=0, reps=1, k_a=3, k_b=1, n=12, rate=20.0):
    rng = np.random.default_rng(seed)
    d = rng.uniform(8.0, 15.0, (reps, n))
    diff = gamma_multiplier(rng, 1.0, (reps, n))
    s_a = stop_counts(rng, Arm("a", rate, k_a > 1), d, diff, k_a, 0.3)
    s_b = stop_counts(rng, Arm("b", rate, k_b > 1), d, diff, k_b, 0.3)
    return s_a, s_b, d


@pytest.mark.parametrize("mode,k_a,k_b", [(POLICY, 3, 1), (TWO_LEVEL, 3, 1), (TWO_LEVEL, 3, 4)])
def test_the_variance_formula_matches_brute_force_resampling_of_the_linear_statistic(mode, k_a, k_b):
    """Draw the resampling weights themselves -- scenario counts shared, seed counts per arm --
    and take the variance of sum w_k u_i e_ki (arm a) minus the same for arm b."""
    s_a, s_b, d = _data(k_a=k_a, k_b=k_b)
    a, b = _terms(s_a, d), _terms(s_b, d)
    ea = s_a[0] / a.total[0] - d[0] / a.distance[0]
    eb = s_b[0] / b.total[0] - d[0] / b.distance[0]
    rng = np.random.default_rng(9)
    m, n = 400_000, d.shape[1]
    u = rng.multinomial(n, np.full(n, 1 / n), size=m)

    def weights(k):
        if mode == POLICY or k == 1:
            return np.ones((m, k))
        return rng.multinomial(k, np.full(k, 1 / k), size=m)

    t = ((weights(k_a) @ ea) * u).sum(axis=1) - ((weights(k_b) @ eb) * u).sum(axis=1)
    assert bootstrap_variance(s_a, s_b, d, mode)[0] == pytest.approx(t.var(), rel=0.01)


def test_the_full_bootstrap_has_the_variance_the_grid_uses():
    """The linearised variance against the full bootstrap's, over six datasets. Seeds fixed: within
    3 % each (measured 0.3-1.1 %). Seeds resampled: five seeds give only 126 distinct resamples and
    the log is curved across them, so single datasets scatter (measured -2 % to +7 %); the average
    is within 3 % and none is off by more than 10 %. The power script checks the effect of this on
    rejections against the full bootstrap at selected cells."""
    ratios = {POLICY: [], TWO_LEVEL: []}
    for seed in range(6):
        s_a, s_b, d = _data(seed=seed, k_a=5, k_b=1, n=150, rate=20.0)
        for mode, found in ratios.items():
            boot = bootstrap_log_rr(np.random.default_rng(100 + seed),
                                    {"a": s_a[0], "b": s_b[0]}, d[0], mode, 40_000)
            found.append(np.var(boot["a"] - boot["b"]) / bootstrap_variance(s_a, s_b, d, mode)[0])
    assert np.all(np.abs(np.array(ratios[POLICY]) - 1.0) < 0.03), ratios
    assert abs(np.mean(ratios[TWO_LEVEL]) - 1.0) < 0.03, ratios
    assert np.all(np.abs(np.array(ratios[TWO_LEVEL]) - 1.0) < 0.10), ratios


def test_a_classical_pair_is_the_same_in_both_modes_and_seeds_add_variance():
    s_a, s_b, d = _data(k_a=1, k_b=1, reps=20)
    assert np.allclose(bootstrap_variance(s_a, s_b, d, POLICY),
                       bootstrap_variance(s_a, s_b, d, TWO_LEVEL), rtol=1e-3)
    s_a, s_b, d = _data(k_a=5, k_b=1, reps=20)
    assert np.all(bootstrap_variance(s_a, s_b, d, TWO_LEVEL)
                  > bootstrap_variance(s_a, s_b, d, POLICY))


def test_an_unknown_mode_is_refused():
    s_a, s_b, d = _data()
    with pytest.raises(ValueError):
        bootstrap_variance(s_a, s_b, d, "neither")


def test_positive_control_two_classical_arms_at_equal_rates_hold_the_level_each_way():
    """Like against like, plenty of stops: about 2.5 % each way at 0.05."""
    rng = np.random.default_rng(3)
    reps, n = 6000, 300
    d = rng.uniform(8.0, 15.0, (reps, n))
    diff = gamma_multiplier(rng, 1.0, (reps, n))
    s_a = stop_counts(rng, Arm("a", 10.0, False), d, diff, 1, 0.0)
    s_b = stop_counts(rng, Arm("b", 10.0, False), d, diff, 1, 0.0)
    low, high = normal_verdict(s_a, s_b, d, POLICY, 0.05)
    assert 0.018 <= low.mean() <= 0.032 and 0.018 <= high.mean() <= 0.032


def test_a_large_reduction_is_detected_the_right_way():
    rng = np.random.default_rng(4)
    reps, n = 500, 300
    d = rng.uniform(8.0, 15.0, (reps, n))
    diff = gamma_multiplier(rng, 1.0, (reps, n))
    s_a = stop_counts(rng, Arm("L5", 3.0, True), d, diff, 5, 0.1)
    s_b = stop_counts(rng, Arm("B1", 10.0, False), d, diff, 1, 0.0)
    low, high = normal_verdict(s_a, s_b, d, TWO_LEVEL, 0.05 / 3)
    assert low.mean() > 0.95 and high.mean() == 0.0


def test_percentile_verdict_known_answers():
    assert percentile_verdict(np.linspace(-2.0, -1.0, 1001), 0.05) == (True, False)
    assert percentile_verdict(np.linspace(1.0, 2.0, 1001), 0.05) == (False, True)
    assert percentile_verdict(np.linspace(-1.0, 1.0, 1001), 0.05) == (False, False)


def test_the_bootstrap_shares_scenarios_across_arms():
    """Two arms with IDENTICAL data and seeds fixed: every resample gives them the same log rate,
    so the ratio is exactly 0 in every draw -- which only holds if the scenario draw is shared."""
    s = np.array([[1, 0, 3, 2, 0, 1]])
    d = np.array([10.0, 12.0, 9.0, 14.0, 11.0, 8.0])
    boot = bootstrap_log_rr(np.random.default_rng(0), {"a": s, "b": s.copy()}, d, POLICY, 500)
    assert np.all(boot["a"] - boot["b"] == 0.0)
    assert CONTINUITY == 0.5


# ---------------------------------------------------------------- the small-seed correction


def test_the_components_add_up_to_the_uncorrected_two_level_variance():
    for k_a, k_b in ((5, 1), (5, 5), (3, 10)):
        s_a, s_b, d = _data(reps=30, k_a=k_a, k_b=k_b, n=40)
        a, b = arm_terms(s_a, d), arm_terms(s_b, d)
        c = components(a, b)
        assert np.allclose(c.scenario + c.seed_a + c.seed_b,
                           variance_terms(a, b, TWO_LEVEL), rtol=1e-10, atol=1e-15)
        assert np.allclose(c.scenario, variance_terms(a, b, POLICY))
        if k_b == 1:
            assert np.all(c.seed_b == 0.0)


def test_welch_satterthwaite_known_answers():
    v = np.array([1.0])
    assert welch_satterthwaite([(v, 4), (v, 4)])[0] == pytest.approx(8.0)
    assert welch_satterthwaite([(v, 4)])[0] == pytest.approx(4.0)
    # (2 + 0.5)^2 / (4/4 + 0.25/9) = 6.25 / 1.027778 = 6.0811
    assert welch_satterthwaite([(2 * v, 4), (0.5 * v, 9)])[0] == pytest.approx(6.0811, abs=1e-4)
    assert welch_satterthwaite([(1e-12 * v, 4), (v, 299)])[0] == pytest.approx(299.0, rel=1e-6)


def test_the_corrected_interval_scales_the_seed_part_and_widens_with_few_seeds():
    s_a, s_b, d = _data(reps=10, k_a=5, k_b=1, n=150, rate=10.0)
    a, b = arm_terms(s_a, d), arm_terms(s_b, d)
    c = components(a, b)
    var, half = corrected_interval(c, 0.05)
    assert np.allclose(var, c.scenario + 1.25 * c.seed_a)            # K/(K-1) = 5/4
    assert np.all(half > 1.959964 * np.sqrt(c.scenario + c.seed_a))   # t and the scaling widen


def _equal_learned(seed_cv, k, reps=3000, n=150, rate=5.0, seed=21):
    rng = np.random.default_rng(seed)
    d = rng.uniform(8.0, 15.0, (reps, n))
    diff = gamma_multiplier(rng, 1.0, (reps, n))
    s_a = stop_counts(rng, Arm("L5", rate, True), d, diff, k, seed_cv)
    s_b = stop_counts(rng, Arm("L1", rate, True), d, diff, k, seed_cv)
    return arm_terms(s_a, d), arm_terms(s_b, d)


def test_positive_control_the_correction_holds_the_level_where_the_uncorrected_test_does_not():
    """Learned vs learned at EQUAL rates, seed CV 0.3, five seeds each, 3000 replicates (MC SE at
    the nominal 0.025: 0.0029). Measured: uncorrected 0.039 / 0.037 per side, 4-5 SE above
    nominal; corrected 0.015 / 0.015. Asserted at nominal +- 2 SE."""
    a, b = _equal_learned(0.3, 5)
    low_u, high_u = verdict_terms(a, b, TWO_LEVEL, 0.05)
    low_c, high_c = corrected_verdict(a, b, 0.05)
    assert low_u.mean() > 0.031 and high_u.mean() > 0.031
    assert low_c.mean() <= 0.031 and high_c.mean() <= 0.031


def test_the_mean_terms_give_the_exact_two_level_variance_of_a_success_rate_difference():
    """Binary outcomes, two learned arms: the variance formula against brute-force resampling of
    seeds (per arm) and scenarios (shared). The mean is exactly linear here, so they agree to MC."""
    rng = np.random.default_rng(5)
    k, n = 4, 30
    s_a = (rng.random((1, k, n)) < rng.uniform(0.3, 0.9, (1, 1, n))).astype(float)
    s_b = (rng.random((1, k, n)) < rng.uniform(0.3, 0.9, (1, 1, n))).astype(float)
    a, b = mean_terms(s_a), mean_terms(s_b)
    c = components(a, b)
    m = 300_000
    u = rng.multinomial(n, np.full(n, 1 / n), size=m)
    wa = rng.multinomial(k, np.full(k, 1 / k), size=m)
    wb = rng.multinomial(k, np.full(k, 1 / k), size=m)
    diff = (((wa @ s_a[0]) * u).sum(1) - ((wb @ s_b[0]) * u).sum(1)) / (k * n)
    assert c.scenario[0] + c.seed_a[0] + c.seed_b[0] == pytest.approx(diff.var(), rel=0.01)
    assert mean_difference(a, b)[0] == pytest.approx(s_a.mean() - s_b.mean(), abs=1e-12)


def test_the_flat_fallback_uses_k_minus_1_against_a_classical_arm_only():
    """Against a classical arm, df = K - 1 = 4 outright: the half-width is t_4 x sqrt(var). Learned
    against learned is Welch-Satterthwaite whatever the rule."""
    from scipy import stats as st

    from pfbnav.eval.rate_ratio import FLAT, WS
    s_a, s_b, d = _data(reps=8, k_a=5, k_b=1, n=150, rate=10.0)
    c = components(arm_terms(s_a, d), arm_terms(s_b, d))
    var, half = corrected_interval(c, 0.05, FLAT)
    assert np.allclose(half, st.t.isf(0.025, 4) * np.sqrt(var))
    _, half_ws = corrected_interval(c, 0.05, WS)
    assert np.all(half_ws <= half + 1e-12)                  # WS df >= 4 here, so never wider
    s_a, s_b, d = _data(reps=8, k_a=5, k_b=5, n=150, rate=10.0)
    c = components(arm_terms(s_a, d), arm_terms(s_b, d))
    assert np.allclose(corrected_interval(c, 0.05, FLAT)[1], corrected_interval(c, 0.05, WS)[1])
    with pytest.raises(ValueError):
        corrected_interval(c, 0.05, "neither")

