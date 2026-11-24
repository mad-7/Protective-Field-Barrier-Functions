"""The offline outcome classes (2026-11-24): known answers on synthetic episodes, before any real run is read."""
import pytest

from pfbnav.eval.outcome_classes import classify_stops, classify_timeouts, progress_class


@pytest.mark.parametrize("dist, cls", [(0.0, "under_10"), (0.99, "under_10"), (1.0, "10_to_50"),
                                       (4.99, "10_to_50"), (5.0, "50_to_90"), (8.99, "50_to_90"),
                                       (9.0, "over_90"), (12.0, "over_90")])
def test_progress_class_edges_on_a_ten_metre_plan(dist, cls):
    assert progress_class(dist, 10.0) == cls


def test_a_nonpositive_plan_is_refused():
    with pytest.raises(ValueError):
        progress_class(1.0, 0.0)


def _ep(outcome="timeout", dist=2.0, plan=10.0, unmapped=0, stops=0, fb=0, steps=100, brake=0,
        stationary=0, at_rest=0, rec=0):
    return {"outcome": outcome, "distance_m": dist, "plan_length_m": plan, "unmapped_count": unmapped,
            "protective_stops": stops, "pfb_infeasible": fb, "steps": steps,
            "pfb_modes": {"brake": brake}, "stationary_triggers": stationary,
            "at_rest_triggers": at_rest, "recovery_attempts": rec}


def test_timeouts_are_counted_by_class_and_condition_and_successes_are_ignored():
    eps = [_ep(dist=0.5), _ep(dist=0.5, unmapped=1, brake=50), _ep(dist=6.0, unmapped=2),
           _ep(dist=9.5, rec=1), _ep(outcome="success", dist=10.0)]
    out = classify_timeouts(eps)
    assert out["timeouts"] == 4 and out["with_recovery_attempts"] == 1
    bp = out["by_progress"]
    assert bp["under_10"] == {"mapped_only": 1, "unmapped": 1, "mean_brake_share": 0.25}
    assert bp["50_to_90"]["unmapped"] == 1 and bp["over_90"]["mapped_only"] == 1
    assert bp["10_to_50"]["mean_brake_share"] is None


def test_stops_are_attributed_to_their_episode_conditions():
    eps = [_ep(outcome="success", stops=2, fb=0), _ep(stops=3, fb=1, unmapped=1, stationary=1, at_rest=2),
           _ep(stops=0, fb=5)]
    out = classify_stops(eps)
    assert out["stops"] == 5 and out["episodes_with_stops"] == 2
    assert out["in_episodes_with_fallback"] == 3 and out["in_episodes_without_fallback"] == 2
    assert out["by_route_condition"] == {"mapped_only": 2, "unmapped": 3}
    assert out["by_outcome"] == {"success": 2, "timeout": 3}
    assert out["stationary_triggers"] == 1 and out["at_rest_triggers"] == 2


def test_the_real_episode_schema_is_accepted():
    """The keys the classifier reads exist in a recorded filtered-run line (one validation episode)."""
    ep = {"outcome": "timeout", "steps": 4000, "distance_m": 7.0043, "plan_length_m": 9.5096,
          "protective_stops": 0, "stationary_triggers": 0, "at_rest_triggers": 0, "pfb_infeasible": 3,
          "pfb_modes": {"filtered": 546, "nominal": 2240, "brake": 1211, "fallback": 3},
          "recovery_attempts": 0, "unmapped_count": 2}
    out = classify_timeouts([ep])
    assert out["by_progress"]["50_to_90"]["unmapped"] == 1
    assert out["by_progress"]["50_to_90"]["mean_brake_share"] == pytest.approx(1211 / 4000)
