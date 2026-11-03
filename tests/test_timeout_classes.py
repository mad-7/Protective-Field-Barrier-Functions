"""B7's timeout classes against the oracle (ruled 2026-11-03): known answers (hard rule 6)."""
import pytest

from pfbnav.eval.timeout_classes import (
    BLOCKED,
    DEADLOCK,
    UNCLASSIFIED,
    class_counts,
    classify_timeouts,
)


def _ep(seed, outcome, stratum="narrow", unmapped=0, distance=5.0, plan=10.0, offsets=()):
    return {"scenario_seed": seed, "outcome": outcome, "stratum": stratum,
            "unmapped_count": unmapped, "unmapped_offsets_m": list(offsets),
            "plan_length_m": plan, "distance_m": distance, "steps": 4000, "pfb_modified": 1000,
            "protective_stops": 0}


B7 = [_ep(1, "timeout", unmapped=1, offsets=[0.2, 0.6]),   # oracle fails, obstacle -> blocked
      _ep(2, "timeout"),                                    # oracle succeeds -> deadlock
      _ep(3, "timeout", stratum="wide"),                    # oracle fails, no obstacle -> unclassified
      _ep(4, "success")]                                    # not a timeout: no row
ORACLE = [_ep(1, "timeout"), _ep(2, "success"), _ep(3, "unrecoverable_stop", stratum="wide"),
          _ep(4, "success")]


def test_each_class_is_decided_by_the_oracle_on_the_same_scenario():
    rows = classify_timeouts(B7, ORACLE, dt=0.05)
    assert [(r["scenario_seed"], r["class"]) for r in rows] == [
        (1, BLOCKED), (2, DEADLOCK), (3, UNCLASSIFIED)]
    assert rows[0]["nearest_unmapped_offset_m"] == 0.2 and rows[1]["nearest_unmapped_offset_m"] is None
    assert rows[0]["travelled_share"] == 0.5 and rows[0]["mean_speed_mps"] == 0.025
    assert rows[0]["pfb_modified_share"] == 0.25


def test_counts_per_stratum_with_ids():
    got = class_counts(classify_timeouts(B7, ORACLE, dt=0.05))
    assert got["narrow"][BLOCKED] == {"count": 1, "ids": [1]}
    assert got["narrow"][DEADLOCK] == {"count": 1, "ids": [2]}
    assert got["wide"][UNCLASSIFIED] == {"count": 1, "ids": [3]}
    assert sum(v["count"] for v in got["all"].values()) == 3


def test_refuses_an_oracle_run_on_another_set():
    with pytest.raises(ValueError, match="one per scenario"):
        classify_timeouts(B7, ORACLE[:3], dt=0.05)
    with pytest.raises(ValueError, match="one per scenario"):
        classify_timeouts(B7, ORACLE + [_ep(4, "timeout")], dt=0.05)


def test_route_speeds_separate_latched_time_from_driving_speed():
    from pfbnav.eval.timeout_classes import route_speeds
    eps = [{"scenario_seed": 7, "outcome": "unrecoverable_stop", "distance_m": 6.0, "steps": 4000,
            "stopped_steps": 2000, "protective_stops": 3},
           {"scenario_seed": 8, "outcome": "timeout", "distance_m": 10.0, "steps": 4000},
           {"scenario_seed": 9, "outcome": "success", "distance_m": 1.0, "steps": 10}]
    rows = route_speeds(eps, [8, 7], dt=0.05)
    assert [r["scenario_seed"] for r in rows] == [8, 7]
    assert rows[0]["mean_speed_mps"] == 0.05 and rows[0]["moving_speed_mps"] == 0.05
    assert rows[1]["mean_speed_mps"] == 0.03 and rows[1]["moving_speed_mps"] == 0.06
    with pytest.raises(ValueError, match="without an episode"):
        route_speeds(eps, [8, 1], dt=0.05)
    with pytest.raises(ValueError, match="twice"):
        route_speeds(eps + [eps[1]], [8], dt=0.05)
