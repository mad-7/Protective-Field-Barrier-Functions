"""Known-answer tests for gate G1b's verdict and `v_max` derivation (hard rule 6).

No simulator. D8 makes `v_max` an output of this gate and the input to a single commit
updating every dependent constant, so the rule that derives it is a decision routine and
gets the same treatment as a metric.
"""
import importlib.util
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]


def _load_script():
    spec = importlib.util.spec_from_file_location(
        "g1b_script", REPO_ROOT / "scripts" / "gate_g1b_tracking.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules["g1b_script"] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def g1b():
    return _load_script()


@pytest.fixture(scope="module")
def criteria():
    committed = yaml.safe_load(
        (REPO_ROOT / "preregistration" / "gates.yaml").read_text()
    )["G1b_closed_loop_tracking"]
    # The lateral extension (A6) may not be in gates.yaml yet; the verdict logic is tested
    # against the values A6 proposes either way.
    return {"lateral_extra_speeds": [0.35, 0.45], **committed}


def _runs(criteria, *, peak_at):
    """One result per profile, speed and repetition; `peak_at` gives the peak by speed."""
    return [
        {"profile": p, "speed": s, "repetition": r,
         "max_abs_cross_track": peak_at(s), "rms_cross_track": 0.005,
         "final_heading_error_deg": 1.0}
        for p in criteria["profiles"] for s in criteria["speeds"]
        for r in range(int(criteria["repetitions"]))
    ]


def test_v_max_is_the_highest_speed_passing_everything(g1b, criteria):
    report = g1b.verdict(_runs(criteria, peak_at=lambda s: 0.01 if s <= 0.30 else 0.09),
                         criteria)
    assert report["result"] == "PASS"
    assert report["v_max_from_gate"] == pytest.approx(0.30)


def test_a_single_failing_repetition_disqualifies_its_speed(g1b, criteria):
    runs = _runs(criteria, peak_at=lambda s: 0.01)
    # One repetition of one profile at the top speed exceeds the peak criterion.
    top = max(criteria["speeds"])
    for r in runs:
        if r["speed"] == top and r["profile"] == criteria["profiles"][0] \
                and r["repetition"] == 2:
            r["max_abs_cross_track"] = 0.09
    report = g1b.verdict(runs, criteria)
    assert report["v_max_from_gate"] == pytest.approx(sorted(criteria["speeds"])[-2])


def test_failing_everywhere_yields_no_v_max_and_names_the_fallback(g1b, criteria):
    report = g1b.verdict(_runs(criteria, peak_at=lambda s: 0.5), criteria)
    assert report["result"] == "FAIL"
    assert report["v_max_from_gate"] is None
    assert "unicycle primary" in report["fallback"]


def test_a_speed_missing_a_profile_cannot_pass(g1b, criteria):
    """A partial run must not be read as a pass; absence of evidence is not a pass."""
    runs = [r for r in _runs(criteria, peak_at=lambda s: 0.01)
            if not (r["speed"] == max(criteria["speeds"])
                    and r["profile"] == criteria["profiles"][-1])]
    report = g1b.verdict(runs, criteria)
    assert report["v_max_from_gate"] == pytest.approx(sorted(criteria["speeds"])[-2])


def test_rms_and_heading_criteria_are_applied_too(g1b, criteria):
    runs = _runs(criteria, peak_at=lambda s: 0.01)
    for r in runs:
        r["rms_cross_track"] = 0.5 * (r["speed"] > 0.2) + 0.005
        r["final_heading_error_deg"] = 90.0 if r["speed"] > 0.4 else 1.0
    report = g1b.verdict(runs, criteria)
    assert report["v_max_from_gate"] == pytest.approx(0.20)


def test_a_pass_above_a_failure_does_not_raise_v_max(g1b, criteria):
    """A6: the ceiling is the highest speed with every lower speed passing too."""
    runs = _runs(criteria, peak_at=lambda s: 0.09 if abs(s - 0.30) < 1e-9 else 0.01)
    report = g1b.verdict(runs, criteria)
    assert report["per_speed"][0.40]["all_pass"]
    assert report["v_max_from_gate"] == pytest.approx(0.20)


def _lateral_extension(criteria, *, peak_at):
    return [{"profile": "lateral", "speed": s, "repetition": r,
             "max_abs_cross_track": peak_at(s), "rms_cross_track": 0.005,
             "final_heading_error_deg": 1.0}
            for s in criteria["lateral_extra_speeds"]
            for r in range(int(criteria["repetitions"]))]


def test_vy_max_uses_the_lateral_extension_and_ignores_other_profiles(g1b, criteria):
    # Everything passes to 0.30; lateral passes to 0.35 and fails from 0.40. Diagonal
    # fails at 0.50, which must lower v_max but not vy_max.
    lateral_peak = lambda s: 0.01 if s <= 0.35 + 1e-9 else 0.09
    runs = _runs(criteria, peak_at=lambda s: 0.01)
    for r in runs:
        if r["profile"] == "lateral":
            r["max_abs_cross_track"] = lateral_peak(r["speed"])
        if r["profile"] == "diagonal_45" and r["speed"] == 0.50:
            r["max_abs_cross_track"] = 0.09
    runs += _lateral_extension(criteria, peak_at=lateral_peak)
    report = g1b.verdict(runs, criteria)
    assert report["vy_max_from_gate"] == pytest.approx(0.35)
    assert report["v_max_from_gate"] == pytest.approx(0.30)
    # Lateral-only speeds never enter the all-profile grid.
    assert set(report["per_speed"]) == {float(s) for s in criteria["speeds"]}


def test_realised_speed_map_averages_repetitions_and_states_the_shortfall(g1b):
    runs = [{"profile": "forward", "speed": 0.40, "realised_speed": v}
            for v in (0.36, 0.36, 0.366)]
    (row,) = g1b.realised_speed_map(runs)
    assert row["realised_mean_m_s"] == pytest.approx(0.362)
    assert row["realised_min_m_s"] == pytest.approx(0.36)
    assert row["shortfall"] == pytest.approx(0.095)
    assert row["repetitions"] == 3


def test_the_gate_overlay_keeps_the_box_above_every_speed_the_gate_tests():
    """The clamp must not decide the result (A6), so the overlay sits at or above the grid.

    This is what `configs/gates/g1b_rerun.yaml` is for, and why it is NOT a hard rule 16
    violation to be retired (rule 15 until the 2026-09-27 renumbering): 0.50 is a measurement ceiling, not a measured constant.
    PRE-A20 `base.yaml` carried v_max 0.40 -- an output of this gate -- below the 0.50 the grid
    tests, so running the gate from base would have measured the clamp. Since 2026-10-19 (A20
    limits) base carries the gate's own outcome, 0.50, which equals the grid's top. The overlay
    stays: it is the declared measurement ceiling, so the gate's result never depends on what
    base's box was last ruled to be (A6 re-read for this change). A 2026-09-23 note proposing to
    retire the overlay rested on the mistaken premise that it merely repeated base's box; this
    test fails if anyone acts on it.
    """
    from pfbnav.config import load_config
    from pfbnav.sim.robot import ActuationLimits

    committed = yaml.safe_load(
        (REPO_ROOT / "preregistration" / "gates.yaml").read_text()
    )["G1b_closed_loop_tracking"]
    speeds = [float(s) for s in committed["speeds"]]
    lateral = speeds + [float(s) for s in committed.get("lateral_extra_speeds", [])]

    overlay = ActuationLimits.from_config(
        load_config(REPO_ROOT / "configs" / "gates" / "g1b_rerun.yaml"))
    assert overlay.v_max >= max(speeds), (
        f"the overlay's v_max {overlay.v_max} is below the tested grid {max(speeds)}")
    assert overlay.vy_max >= max(lateral), (
        f"the overlay's vy_max {overlay.vy_max} is below the tested lateral "
        f"speeds {max(lateral)}")

    base = ActuationLimits.from_config(load_config(REPO_ROOT / "configs" / "base.yaml"))
    assert base.v_max <= overlay.v_max and base.vy_max <= overlay.vy_max, (
        "base.yaml's box now exceeds the overlay's measurement ceiling; re-read A6 before "
        "changing either")
