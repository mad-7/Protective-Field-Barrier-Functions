"""Known answers for the rebuild comparison (hard rule 6)."""
from pfbnav.eval.set_compare import compare_sets


def _set(*recs):
    return {"scenarios": [dict(r) for r in recs]}


def test_a_rebuild_differing_only_in_the_module_hash_is_identical():
    a = _set({"seed": 1, "goal": [1.0, 2.0], "scenarios_sha256": "old"})
    b = _set({"seed": 1, "goal": [1.0, 2.0], "scenarios_sha256": "new"})
    assert compare_sets(a, b)["identical"]


def test_any_other_difference_or_a_missing_seed_is_reported():
    a = _set({"seed": 1, "goal": [1.0, 2.0]}, {"seed": 2, "goal": [0.0, 0.0]})
    b = _set({"seed": 1, "goal": [1.0, 2.5]}, {"seed": 3, "goal": [0.0, 0.0]})
    out = compare_sets(a, b)
    assert not out["identical"]
    assert {"seed": 1, "field": "goal"} in out["differences"]
    assert {"seed": 2, "field": "present in one build only"} in out["differences"]
    assert {"seed": 3, "field": "present in one build only"} in out["differences"]
