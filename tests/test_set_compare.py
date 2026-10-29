"""Known answers for the rebuild comparison (hard rule 6)."""
import pytest

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
    assert not out["identical"] and not out["accepted"]
    found = {(d["seed"], d["field"]) for d in out["differences"]}
    assert found == {(1, "goal"), (2, "present in one build only"),
                     (3, "present in one build only")}


# ------------------------------------------- ruled 2026-10-29: exact identities, floats within 1e-9 m
TOL = 1e-9


def test_the_measured_cross_host_rounding_is_accepted_and_not_identical():
    """The null test (hard rule 19): values recorded on 2026-10-29 by the same commit on the laptop
    and on the training host -- one unit in the last place, and the largest difference measured
    (1.887e-15 m). Not identical, accepted, with the max reported."""
    a = _set({"seed": 1000011, "unmapped_offsets_m": [0.03635409691696784, 0.4176325322437606],
              "scenario_sha256": "abc"},
             {"seed": 1000023, "unmapped_offsets_m": [0.024028546115694258, 0.5250550088122602]})
    b = _set({"seed": 1000011, "unmapped_offsets_m": [0.03635409691696784, 0.41763253224376057],
              "scenario_sha256": "abc"},
             {"seed": 1000023, "unmapped_offsets_m": [0.024028546115694254, 0.5250550088122602]})
    out = compare_sets(a, b, float_tol=TOL)
    assert not out["identical"] and out["accepted"]
    assert 0.0 < out["max_float_difference"] < 1e-15
    worst = _set({"seed": 9000070, "unmapped_offsets_m": [0.5]})
    shifted = _set({"seed": 9000070, "unmapped_offsets_m": [0.5 + 1.887379141862766e-15]})
    assert compare_sets(worst, shifted, float_tol=TOL)["accepted"]


def test_a_float_beyond_the_tolerance_is_refused():
    """Positive control: the same comparison refuses a real geometric change, at 1e-8 m and up."""
    for delta in (1e-8, 1e-6, 0.01):
        a = _set({"seed": 1, "unmapped_polygons": [[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]]})
        b = _set({"seed": 1, "unmapped_polygons": [[[0.0, 0.0], [1.0 + delta, 0.0], [1.0, 1.0]]]})
        out = compare_sets(a, b, float_tol=TOL)
        assert not out["accepted"] and out["max_float_difference"] == pytest.approx(delta)


def test_identities_and_structure_are_exact_whatever_the_tolerance():
    """Hashes, seeds, integers, booleans, keys and list lengths never fall under the tolerance, even
    a huge one; nor does an integer compared with a float, or a NaN."""
    base = {"seed": 1, "layout_sha256": "aa", "n": 3, "flag": True, "xs": [0.5, 0.25]}
    for changed in ({"layout_sha256": "ab"}, {"n": 4}, {"flag": False}, {"xs": [0.5]},
                    {"xs": [0.5, 0.25, 0.0]}, {"n": 3.0}, {"xs": [0.5, float("nan")]},
                    {"extra": 1.0}):
        out = compare_sets(_set(base), _set({**base, **changed}), float_tol=1e3)
        assert not out["accepted"], changed
    missing = compare_sets(_set(base, {"seed": 2}), _set(base), float_tol=1e3)
    assert not missing["accepted"]
    with pytest.raises(ValueError, match="non-negative"):
        compare_sets(_set(base), _set(base), float_tol=-1.0)


def test_the_tolerance_is_the_ruled_one_in_the_config():
    from pathlib import Path

    from pfbnav.config import load_config
    cfg = load_config(Path(__file__).resolve().parents[1] / "configs" / "base.yaml")
    assert cfg["world"]["set_verification"]["float_abs_tol_m"] == TOL
