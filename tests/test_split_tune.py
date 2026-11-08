"""The tune-set split by obstacle presence (2026-11-08): known answers on synthetic data (hard rule 6)."""
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("split", ROOT / "scripts" / "split_tune_by_obstacles.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def _ep(sha, outcome, stops, plan=10.0, dist=5.0):
    return {"scenario_sha256": sha, "outcome": outcome, "episode_return": 0.0, "protective_stops": stops,
            "plan_length_m": plan, "distance_m": dist, "steps": 100, "stopped_steps": 50}


MANIFEST = {"scenarios": [{"scenario_sha256": "a", "unmapped_count": 2},
                          {"scenario_sha256": "b", "unmapped_count": 0},
                          {"scenario_sha256": "c", "unmapped_count": 1}]}


def test_flags_and_split():
    flags = m.obstacle_flags([MANIFEST])
    assert flags == {"a": True, "b": False, "c": True}
    with pytest.raises(ValueError):
        m.obstacle_flags([MANIFEST, MANIFEST])
    parts = m.split_records([_ep("a", "timeout", 3), _ep("b", "success", 0), _ep("c", "timeout", 1)], flags)
    assert [r["scenario_sha256"] for r in parts["obstacles"]] == ["a", "c"]
    assert [r["scenario_sha256"] for r in parts["clear"]] == ["b"]


def test_a_run_of_another_set_is_refused():
    flags = m.obstacle_flags([MANIFEST])
    with pytest.raises(ValueError):
        m.split_records([_ep("a", "timeout", 3)], flags)                     # scenarios missing
    with pytest.raises(ValueError):
        m.split_records([_ep("a", "timeout", 3), _ep("a", "timeout", 3), _ep("b", "success", 0)], flags)


def test_the_row_known_answer():
    """Two episodes, 4 stops over 20 m planned = 20 per 100 m; 10 m driven = 0.5 of planned; half the
    steps latched; moving 10 m over 100 moving steps at 0.05 s = 2 m/s."""
    r = m.row([_ep("a", "timeout", 3), _ep("c", "success", 1)], 0.05)
    assert r["episodes"] == 2 and r["success_rate"] == 0.5 and r["stops"] == 4
    assert r["per_100m_planned"] == pytest.approx(20.0)
    assert r["driven_over_planned"] == pytest.approx(0.5)
    assert r["latched"] == pytest.approx(0.5)
    assert r["moving_mps"] == pytest.approx(2.0)
