"""Known-answer tests for the H1 power simulation (hard rule 6). Pure; synthetic data only."""
import math

import numpy as np
import pytest
from scipy import stats

from pfbnav.eval.power import (
    Arm,
    gamma_multiplier,
    simulate_h1,
    smallest_sufficient,
    stops_per_100m,
    wilcoxon_pratt,
)

LENGTHS = [8.0, 10.0, 12.0, 15.0]


@pytest.mark.parametrize("cv", [0.3, 0.5, 1.0, 1.5])
def test_the_gamma_multiplier_has_mean_one_and_the_asked_cv(cv):
    x = gamma_multiplier(np.random.default_rng(0), cv, 400_000)
    assert x.mean() == pytest.approx(1.0, abs=0.01)
    assert x.std() / x.mean() == pytest.approx(cv, rel=0.03)


def test_a_zero_cv_is_exactly_one_and_a_negative_one_is_refused():
    assert np.all(gamma_multiplier(np.random.default_rng(0), 0.0, 5) == 1.0)
    with pytest.raises(ValueError):
        gamma_multiplier(np.random.default_rng(0), -0.1, 5)


@pytest.mark.parametrize("learned", [False, True])
def test_the_endpoint_averages_to_the_arm_rate(learned):
    rng = np.random.default_rng(1)
    lengths = np.full((400, 300), 10.0)
    got = stops_per_100m(rng, Arm("x", 5.0, learned), lengths, np.ones_like(lengths), 5, 0.3)
    assert got.mean() == pytest.approx(5.0, rel=0.01)


def test_a_learned_arms_seed_effect_is_shared_across_scenarios():
    """A replicate's mean over 2000 scenarios of 100 m at rate 5: the classical arm keeps only
    Poisson noise, CV sqrt(5 / 2000) / 5 = 0.0100; the learned arm adds its five seeds' mean
    multiplier, CV 0.3 / sqrt(5) = 0.134, which does NOT average out over scenarios."""
    rng = np.random.default_rng(2)
    lengths = np.full((3000, 2000), 100.0)
    ones = np.ones_like(lengths)
    learned = stops_per_100m(rng, Arm("L", 5.0, True), lengths, ones, 5, 0.3).mean(axis=1)
    classical = stops_per_100m(rng, Arm("B", 5.0, False), lengths, ones, 5, 0.3).mean(axis=1)
    assert learned.std() / learned.mean() == pytest.approx(0.3 / math.sqrt(5), rel=0.08)
    assert classical.std() / classical.mean() == pytest.approx(0.0100, rel=0.08)


def test_wilcoxon_pratt_against_a_hand_calculation():
    """d = [0, 1, -2, 3, 3]. Pratt ranks |d| WITH the zero: 1, 2, 3, 4.5, 4.5, so R+ = 11 and
    R- = 3. Mean 5*6/4 - 1*2/4 = 7; variance (5*6*11 - 1*2*3)/24 - (2^3 - 2)/48 = 13.375;
    z = 4 / sqrt(13.375); two-sided p = 0.274070."""
    p, right_way = wilcoxon_pratt(np.array([0.0, 1.0, -2.0, 3.0, 3.0]))
    want = 2 * stats.norm.sf(4 / math.sqrt(13.375))
    assert float(p) == pytest.approx(want, abs=1e-12)
    assert float(want) == pytest.approx(0.274070, abs=1e-6)
    assert bool(right_way)
    _, other_way = wilcoxon_pratt(-np.array([0.0, 1.0, -2.0, 3.0, 3.0]))
    assert not bool(other_way)


def test_it_is_vectorised_row_by_row():
    rng = np.random.default_rng(3)
    d = (rng.poisson(1.0, (30, 200)) - rng.poisson(1.0, (30, 200))).astype(float)
    p, _ = wilcoxon_pratt(d)
    for i in range(30):
        want = stats.wilcoxon(d[i], zero_method="pratt", method="approx", correction=False)
        assert p[i] == pytest.approx(want.pvalue, abs=1e-12)


def simulate(l5_rate, comparator_rates, reps=3000, n=150, levels=(0.05,), cv=1.0, seed=4,
             seed_cv=0.3, learned=None, l5_learned=True):
    kinds = learned if learned is not None else [bool(i % 2) for i in range(len(comparator_rates))]
    comps = [Arm(f"C{i}", r, k) for i, (r, k) in enumerate(zip(comparator_rates, kinds, strict=True))]
    return simulate_h1(np.random.default_rng(seed), LENGTHS, n, Arm("L5", l5_rate, l5_learned),
                       comps, difficulty_cv=cv, seed_cv=seed_cv, n_seeds=5, levels=levels,
                       reps=reps)


def test_positive_control_like_against_like_the_test_holds_its_size():
    """Equal rates and arms of the same kind (learned without seed effects, or classical against
    classical): a two-sided test at 0.05 rejects in EACH direction about 2.5 % of the time."""
    for got in (simulate(5.0, [5.0], reps=4000, seed_cv=0.0, learned=[True]),
                simulate(5.0, [5.0], reps=4000, learned=[False], l5_learned=False)):
        assert 0.015 <= got["marginal"][0.05]["C0"] <= 0.035, got
        assert 0.015 <= got["opposite"][0.05]["C0"] <= 0.035, got


def test_property_shared_seed_effects_inflate_the_null_rejection_rate():
    """A property of the frozen analysis under the ruled model, recorded 2026-10-02: the unit is
    the scenario, but a learned arm's seed effect is shared by every scenario, so the test sees
    THESE five seeds, not the method. Equal rates, seed CV 0.3: far above 2.5 % each way."""
    got = simulate(5.0, [5.0], reps=2000, n=225, learned=[True])
    assert got["marginal"][0.05]["C0"] > 0.15 and got["opposite"][0.05]["C0"] > 0.15


def test_property_a_single_run_against_a_seed_average_leans_against_l5():
    """The second property, recorded 2026-10-02: at equal MEAN rates, a classical arm's single
    Poisson count (mostly zero at these rates) against L5's five-seed average gives differences
    that are not symmetric about zero, and the signed-rank test -- not a test of means -- reads
    L5 as worse. Seed effects off, so only the shape is at work."""
    got = simulate(5.0, [5.0], reps=2000, n=225, learned=[False], seed_cv=0.0)
    assert got["marginal"][0.05]["C0"] < 0.005
    assert got["opposite"][0.05]["C0"] > 0.3


def test_a_large_effect_has_power_near_one():
    assert simulate(0.5, [5.0] * 7, reps=500)["power"][0.05] > 0.99


def test_l5_worse_than_its_comparators_never_succeeds():
    assert simulate(7.0, [5.0] * 3, reps=500)["power"][0.05] == 0.0


def test_intersection_union_is_at_most_the_weakest_comparison_and_falls_with_alpha():
    got = simulate(3.5, [5.0, 5.0, 7.0], reps=2000, levels=(0.05, 0.025))
    for lv in (0.05, 0.025):
        assert got["power"][lv] <= min(got["marginal"][lv].values()) + 1e-12
    assert got["power"][0.025] <= got["power"][0.05]


def test_power_rises_with_the_number_of_pairs():
    small = simulate(3.5, [5.0] * 2, n=40, reps=2000, seed_cv=0.0, learned=[True, True])["power"][0.05]
    large = simulate(3.5, [5.0] * 2, n=300, reps=2000, seed_cv=0.0, learned=[True, True])["power"][0.05]
    assert large > small + 0.2


def test_lengths_must_be_positive():
    with pytest.raises(ValueError):
        simulate_h1(np.random.default_rng(0), [10.0, 0.0], 10, Arm("L5", 1, True), [],
                    difficulty_cv=1.0, seed_cv=0.3, n_seeds=5, levels=(0.05,), reps=2)


def test_smallest_sufficient_known_answers():
    assert smallest_sufficient({50: 0.7, 75: 0.81, 100: 0.9}, 0.80) == 75
    assert smallest_sufficient({100: 0.85, 50: 0.82}, 0.80) == 50
    assert smallest_sufficient({50: 0.5, 100: 0.79}, 0.80) is None
    assert smallest_sufficient({50: 0.80}, 0.80) == 50
