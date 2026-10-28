"""Known-answer tests for pfbnav/eval/tracking_spread.py on synthetic runs (hard rule 6)."""
import pytest

from pfbnav.eval.tracking_spread import (
    cell_spread,
    margin_statement,
    regression,
    spread_verdict,
    within_invocation_identical,
)

CRITERIA = {"max_cross_track": 0.05, "rms_cross_track": 0.02,
            "max_final_heading_error_deg": 5.0,
            "profiles": ["forward", "lateral", "translate_while_rotating"],
            "speeds": [0.10, 0.20]}


def run(profile, speed, heading=0.1, peak=0.001, rms=0.0005, rep=0):
    return {"profile": profile, "speed": speed, "repetition": rep,
            "max_abs_cross_track": peak, "rms_cross_track": rms,
            "final_heading_error_deg": heading}


def invocation(twr_heading_at_020, reps=3):
    return [run(p, v, heading=(twr_heading_at_020 if (p, v) == ("translate_while_rotating",
                                                              0.20) else 0.1), rep=k)
            for k in range(reps) for v in (0.10, 0.20)
            for p in ("forward", "lateral", "translate_while_rotating")]


def test_cell_spread_known_answer():
    s = cell_spread([4.9, 5.053, 4.95], 5.0)
    assert s["min"] == pytest.approx(4.9) and s["max"] == pytest.approx(5.053)
    assert s["spread"] == pytest.approx(0.153)
    assert s["margin_worst"] == pytest.approx(-0.053)
    assert s["margin_best"] == pytest.approx(0.1)
    assert (s["runs_within"], s["runs"], s["status"]) == (2, 3, "straddle")


def test_cell_spread_limit_is_inclusive():
    assert cell_spread([5.0, 5.0], 5.0)["status"] == "pass"
    assert cell_spread([5.1, 5.2], 5.0)["status"] == "fail"


def test_all_invocations_within_gives_the_top_speed():
    r = spread_verdict([invocation(4.0), invocation(4.2), invocation(4.1)], CRITERIA)
    assert r["v_max_from_gate"] == 0.20 and r["result"] == "PASS"
    assert r["within_invocation_identical"] == [True, True, True]
    assert r["cells"]["translate_while_rotating@0.20"]["metrics"][
        "final_heading_error_deg"]["spread"] == pytest.approx(0.2)


def test_straddling_invocations_fail_the_speed_with_the_margin_stated():
    r = spread_verdict([invocation(4.95), invocation(5.053), invocation(4.98)], CRITERIA)
    assert r["per_speed"][0.20]["all_pass"] is False
    assert r["per_speed"][0.20]["straddling"] == ["translate_while_rotating"]
    assert r["v_max_from_gate"] == 0.10
    cell = r["cells"]["translate_while_rotating@0.20"]
    assert cell["status"] == "straddle"
    text = margin_statement(cell)
    assert "2/3 invocations within" in text and "-0.053" in text


def test_a_passing_speed_above_a_failure_is_not_a_ceiling():
    runs = [run(p, v, heading=(6.0 if v == 0.10 else 0.1))
            for v in (0.10, 0.20) for p in ("forward", "lateral", "translate_while_rotating")]
    r = spread_verdict([runs], CRITERIA)
    assert r["v_max_from_gate"] is None and r["result"] == "FAIL"


def test_a_missing_profile_fails_the_speed():
    runs = [run(p, v) for v in (0.10, 0.20) for p in ("forward", "lateral")]
    r = spread_verdict([runs], CRITERIA)
    assert r["per_speed"][0.10]["missing"] == ["translate_while_rotating"]
    assert r["v_max_from_gate"] is None


def test_vy_max_comes_from_the_lateral_cells_including_extra_speeds():
    runs = invocation(0.1, reps=1) + [run("lateral", 0.15), run("lateral", 0.25, peak=0.06)]
    r = spread_verdict([runs], CRITERIA)
    assert r["vy_max_from_gate"] == 0.20


def test_within_invocation_identity_detects_a_differing_repetition():
    inv = invocation(4.0)
    assert within_invocation_identical(inv)
    inv[-1] = dict(inv[-1], final_heading_error_deg=4.0000001)
    assert not within_invocation_identical(inv)


def test_rejects_an_empty_invocation():
    with pytest.raises(ValueError):
        spread_verdict([[]], CRITERIA)


def _cell(status):
    return {"status": status}


def test_regression_reports_no_change_when_verdicts_agree():
    ref = {"a": _cell("pass"), "b": _cell("fail")}
    r = regression(ref, {"a": _cell("pass"), "b": _cell("straddle")}, ["a", "b"])
    # fail -> straddle is still FAIL: the verdict did not change.
    assert r["changed"] == [] and not r["full_rerun_required"]


def test_regression_flags_a_changed_verdict_either_way():
    ref = {"a": _cell("pass"), "b": _cell("fail")}
    r = regression(ref, {"a": _cell("straddle"), "b": _cell("pass")}, ["a", "b"])
    assert r["changed"] == ["a", "b"] and r["full_rerun_required"]


def test_regression_refuses_a_missing_cell():
    with pytest.raises(ValueError):
        regression({"a": _cell("pass")}, {}, ["a"])
