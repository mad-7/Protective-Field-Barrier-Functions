"""The oracle quantification's per-scenario path, run end to end on one tune scenario.

Added 2026-10-16 after the decide phase crashed in its workers (a loop variable shadowed the
list of result rows when A1's floor gained the rotation sweep): the fast suite exercised the
oracle's pieces but never the script that joins them. Tune layouts only (hard rule 2).
"""
import importlib.util
import json
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def _script():
    spec = importlib.util.spec_from_file_location(
        "quantify_oracle_free", REPO / "scripts" / "quantify_oracle_free.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_decide_phase_produces_one_complete_row_per_margin_pair():
    from pfbnav.config import load_config
    q = _script()
    cfg = load_config(REPO / "configs" / "analysis" / "planner_oracle_free_rebuilt.yaml")
    q._init(cfg)
    record = json.loads((REPO / "scenarios" / "tune" / "tune_wide.json")
                        .read_text(encoding="utf-8"))["scenarios"][0]
    rows = q.decide(("tune_wide", record))
    assert len(rows) == 1
    row = rows[0]
    assert row["status"] == "ok"
    for key in ("start_valid", "goal_valid", "traversable", "unavoidable_stops",
                "tangent_stops", "stops_bend", "path_length_m"):
        assert key in row, key
    assert isinstance(row["tangent_stops"], int)
