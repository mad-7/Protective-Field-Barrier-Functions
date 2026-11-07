"""The refined A14b grid's pieces (ruled 2026-11-07, item 2): known answers (hard rule 6)."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest
from scipy import stats

from pfbnav.eval.power_grid import h1_arms_measured, h1_chunks, h1prime_arms, three_way
from pfbnav.eval.rate_ratio import WS, arm_terms, components, corrected_interval, corrected_verdict

ROOT = Path(__file__).resolve().parents[1]
P = {"classical": ["B1", "B2", "B3", "B6"], "learned": ["L1", "L2", "L3"],
     "h1prime_comparators": ["B1", "B2", "B6"],
     "measured_rates_per_100m": {"B1": 64.0, "B2": 52.8, "B3": 20.5}, "learned_ratio_to_b6": 1.0}


def test_h1_arms_take_the_measured_rates_and_the_b6_band():
    l5, comps = h1_arms_measured(0.5, 0.7, P)
    rates = {c.name: (c.rate_per_100m, c.learned) for c in comps}
    assert l5.rate_per_100m == pytest.approx(0.35) and l5.learned
    assert rates == {"B1": (64.0, False), "B2": (52.8, False), "B3": (20.5, False),
                     "B6": (0.5, False), "L1": (0.5, True), "L2": (0.5, True), "L3": (0.5, True)}
    with pytest.raises(ValueError):
        h1_arms_measured(0.5, 0.7, {**P, "measured_rates_per_100m": {"B1": 64.0}})


def test_h1prime_arms_are_all_classical():
    b7, comps = h1prime_arms(1.0, 2.0, P)
    assert (b7.name, b7.rate_per_100m, b7.learned) == ("B7", 2.0, False)
    assert [(c.name, c.rate_per_100m, c.learned) for c in comps] == [
        ("B1", 64.0, False), ("B2", 52.8, False), ("B6", 1.0, False)]


def test_three_way_shares():
    s = three_way(np.array([1, 0, 0, 1], bool), np.array([0, 1, 0, 0], bool))
    assert s == {"favours": 0.5, "against": 0.25, "inconclusive": 0.25}
    with pytest.raises(ValueError):
        three_way(np.array([True]), np.array([True]))


def test_two_classical_arms_use_the_scenario_part_and_n_minus_1_df():
    """endpoints.yaml H1prime: every arm runs once, so the interval is t(n - 1) x the scenario part."""
    rng = np.random.default_rng(1)
    d = rng.uniform(5, 15, size=(3, 40))
    a, b = (arm_terms(rng.poisson(1.0, size=(3, 1, 40)).astype(float), d) for _ in range(2))
    c = components(a, b)
    var, half = corrected_interval(c, 0.05, WS)
    assert np.allclose(var, c.scenario)
    assert np.allclose(half, stats.t.isf(0.025, 39) * np.sqrt(c.scenario))


def test_h1prime_at_equal_rates_rejects_near_its_nominal_one_sided_rate():
    """The calibration H1prime must pass before it is tested (A14a): two classical arms at the same
    rate, 2,000 replications of 150 routes; each side should reject at about level / 2 = 0.025
    (MC SE 0.0035; the band 0.010-0.045 allows the t approximation's small-count slack)."""
    rng = np.random.default_rng(7)
    lengths = np.linspace(4.0, 20.0, 50)
    b7, comps = h1prime_arms(2.0, 1.0, P)
    lower = higher = 0
    for d, counts in h1_chunks(rng, lengths, 150, b7, [comps[2]], 1.0, 0.0, 1, 2000):
        lo, hi = corrected_verdict(arm_terms(counts["B7"], d), arm_terms(counts["B6"], d), 0.05,
                                   classical_df=WS)
        lower, higher = lower + int(lo.sum()), higher + int(hi.sum())
    assert 0.010 <= lower / 2000 <= 0.045 and 0.010 <= higher / 2000 <= 0.045


def test_wall_clock_counts_every_arm_and_k():
    spec = importlib.util.spec_from_file_location("par", ROOT / "scripts" / "power_a14b_refined.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    p = {"episode_wall_s": {"B": 100, "L": 50}, "classical_arms_evaluated": ["B"],
         "learned_arms_evaluated": ["L"], "instances": 2}
    # 36 scenarios x (100 + 10 x 50) s / 2 instances = 10,800 s = 3 h
    assert m.wall_clock_h(36, 10, p) == pytest.approx(3.0)
