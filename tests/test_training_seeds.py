"""Which seeds may train after A35 (ruled 2026-11-07): known answers (hard rule 6)."""
from pfbnav.training.seeds import training_seed_problem

A35 = {"G4_learned_pilot": {"pilot_training_range": [9000000, 9000099],
                            "confirmatory_training_seeds": list(range(9100001, 9100011))}}
OLD = {"G4_learned_pilot": {}}


def test_runs_take_a_confirmatory_or_a_pilot_range_seed():
    assert training_seed_problem(9100001, A35, diagnostic=False) is None
    assert training_seed_problem(9100010, A35, diagnostic=False) is None
    assert training_seed_problem(9000003, A35, diagnostic=False) is None
    assert "neither" in training_seed_problem(9100011, A35, diagnostic=False)
    assert "neither" in training_seed_problem(9000100, A35, diagnostic=False)
    assert "neither" in training_seed_problem(0, A35, diagnostic=False)


def test_diagnostics_never_touch_the_confirmatory_block():
    assert "confirmatory block" in training_seed_problem(9100005, A35, diagnostic=True)
    assert "confirmatory block" in training_seed_problem(9100999, A35, diagnostic=True)
    assert training_seed_problem(9200000, A35, diagnostic=True) is None


def test_before_a35_nothing_applies():
    assert training_seed_problem(123, OLD, diagnostic=False) is None
