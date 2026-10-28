"""Known answers for the shared power-simulation pieces (hard rule 6). Synthetic data only."""
import numpy as np
import pytest
from scipy.special import expit

from pfbnav.eval.power_grid import h1_arms, success_outcomes

P = {"classical": ["B1", "B2", "B3", "B6"], "learned": ["L1", "L2", "L3"], "l5_ratio": 0.7,
     "secondary_other_ratio_to_l5": 2.0}


def test_h1_arms_known_answers():
    l5, comps = h1_arms("all_at_margin", 5.0, P)
    assert l5.rate_per_100m == pytest.approx(3.5) and l5.learned
    assert [c.rate_per_100m for c in comps] == [5.0] * 7
    assert [c.learned for c in comps] == [False] * 4 + [True] * 3
    _, comps = h1_arms("one_at_margin_classical", 5.0, P)
    assert [c.rate_per_100m for c in comps] == pytest.approx([5.0] + [7.0] * 6)
    _, comps = h1_arms("one_at_margin_learned", 5.0, P)
    assert comps[4].rate_per_100m == 5.0 and comps[0].rate_per_100m == pytest.approx(7.0)
    l5, comps = h1_arms("equal_rates", 5.0, P)
    assert l5.rate_per_100m == 5.0 and all(c.rate_per_100m == 5.0 for c in comps)


def test_success_without_seed_effects_has_the_asked_probability():
    rng = np.random.default_rng(0)
    base = np.full((200, 500), 0.4)
    s = success_outcomes(rng, base, 0.2, 0.0, 5)
    assert s.shape == (200, 5, 500) and s.mean() == pytest.approx(0.6, abs=0.005)


def test_a_seed_shift_is_shared_by_every_scenario_and_clipping_holds():
    """With a large seed SD, a seed's success rate over many scenarios is far from the others':
    the shift is per seed, not per scenario. The mean over seeds follows E[expit(logit p + z)]."""
    rng = np.random.default_rng(1)
    base = np.full((400, 2000), 0.5)
    s = success_outcomes(rng, base, 0.0, 1.0, 5)
    per_seed = s.mean(axis=2)                                        # (reps, seeds)
    assert per_seed.std() > 0.15                                     # spread between seeds
    want = expit(np.random.default_rng(2).normal(0, 1.0, 400_000)).mean()
    assert s.mean() == pytest.approx(want, abs=0.01)
    hi = success_outcomes(rng, np.full((50, 100), 0.95), 0.2, 0.0, 3)
    assert hi.mean() == pytest.approx(0.99, abs=0.01)
