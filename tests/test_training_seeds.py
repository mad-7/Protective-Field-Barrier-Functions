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



def test_only_the_two_declared_ranges_are_reportable_and_only_confirmatory_seeds_enter_h1_h2():
    """Ruled 2026-11-07 (item 4): no seed in [9000000, 9999999] outside the pilot training range and
    the confirmatory block appears in a reported result."""
    from pfbnav.training.seeds import confirmatory_seed_problem, reported_training_seed_problem
    for ok in (9000000, 9000003, 9000099, 9100000, 9100001, 9100010, 9100999):
        assert reported_training_seed_problem(ok, A35) is None, ok
    for bad in (9000100, 9000149, 9099999, 9101000, 9200000, 9999999):
        assert "outside both declared ranges" in reported_training_seed_problem(bad, A35), bad
    assert "outside the pilot range" in reported_training_seed_problem(0, A35)
    assert confirmatory_seed_problem(9100005, A35) is None
    assert "never enters" in confirmatory_seed_problem(9000002, A35)
    assert "never enters" in confirmatory_seed_problem(9100011, A35)


def test_every_training_seed_the_preregistration_names_is_reportable():
    """The frozen gates as they stand: every pilot and confirmatory training seed lies in a
    reportable range, the ten confirmatory seeds are 9100001-9100010, and training refuses any seed
    of the pilot range outside the two ranges (so no other seed can produce a run to report)."""
    from pathlib import Path

    import yaml

    from pfbnav.training.seeds import reported_training_seed_problem
    gates = yaml.safe_load((Path(__file__).resolve().parents[1] / "preregistration" / "gates.yaml")
                           .read_text(encoding="utf-8"))
    g4 = gates["G4_learned_pilot"]
    if "confirmatory_training_seeds" not in g4:
        return                                   # before A35: nothing declared yet
    assert g4["confirmatory_training_seeds"] == list(range(9100001, 9100011))
    for s in g4["pilot_training_seeds"] + g4["confirmatory_training_seeds"]:
        assert reported_training_seed_problem(int(s), gates) is None, s
    assert training_seed_problem(9000100, gates, diagnostic=False) is not None
    assert training_seed_problem(9100500, gates, diagnostic=False) is not None
