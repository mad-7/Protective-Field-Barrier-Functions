"""Known-answer tests for the timing summary used by the profiling pass (rule 6)."""
import pytest

from pfbnav.eval.profiling import spikes, summarise_timings


def test_summary_of_a_known_sample():
    s = summarise_timings([1.0] * 19 + [21.0])      # 20 samples, one slow step
    assert s["n"] == 20
    assert s["mean_ms"] == pytest.approx(2.0)
    assert s["p50_ms"] == pytest.approx(1.0)
    assert s["max_ms"] == pytest.approx(21.0)
    assert s["p95_ms"] == pytest.approx(1.0 + 0.05 * 20.0)   # numpy linear interpolation


def test_flag_is_on_the_mean_and_the_tail_is_reported_separately():
    cheap_with_tail = summarise_timings([0.5] * 90 + [5.0] * 10)
    assert not cheap_with_tail["flagged"] and cheap_with_tail["p95_over"]
    expensive = summarise_timings([2.5] * 10)
    assert expensive["flagged"] and expensive["p95_over"]


def test_bad_samples_are_refused():
    for bad in ([], [1.0, -0.1], [float("nan")]):
        with pytest.raises(ValueError):
            summarise_timings(bad)


def test_spikes_are_located_by_step():
    samples = [3.0] * 10
    samples[0] = 604.0      # the first check after collections are built, if that is it
    samples[7] = 51.0
    assert spikes(samples) == [0, 7]
    assert spikes(samples, threshold_ms=100.0) == [0]
    assert spikes([1.0, 2.0]) == []


# ----------------------------------------------------------------------
# floor_decision (the step-floor rule, set before the diagnosis runs)
# ----------------------------------------------------------------------
def test_floor_decision_takes_the_cheapest_representative_configuration():
    from pfbnav.eval.profiling import floor_decision
    measured = {"as_loaded": 48.0, "realtime_off": 12.0, "empty_scene": 1.0}
    d = floor_decision(measured, ["as_loaded", "realtime_off"])
    assert (d["floor_ms"], d["floor_config"], d["decision"]) == (12.0, "realtime_off",
                                                                 "batched_step")


def test_floor_decision_ignores_stripped_scenes_when_setting_the_floor():
    from pfbnav.eval.profiling import floor_decision
    d = floor_decision({"as_loaded": 45.0, "empty_scene": 1.0}, ["as_loaded"])
    assert (d["floor_ms"], d["decision"]) == (45.0, "replan_matrix")


def test_floor_decision_threshold_is_strict():
    from pfbnav.eval.profiling import floor_decision
    assert floor_decision({"a": 15.0}, ["a"])["decision"] == "replan_matrix"
    assert floor_decision({"a": 14.99}, ["a"])["decision"] == "batched_step"


def test_floor_decision_rejects_missing_measurements():
    from pfbnav.eval.profiling import floor_decision
    with pytest.raises(ValueError):
        floor_decision({"a": 1.0}, ["b"])
    with pytest.raises(ValueError):
        floor_decision({"a": 1.0}, [])
