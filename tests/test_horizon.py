"""The episode horizon's derivation rule (ruled 2026-10-20, A23): known answers on synthetic
schedules, then the rule itself over every built set (`tests/data/route_schedules.json`, from
`scripts/route_schedules.py`)."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from pfbnav.config import load_config
from pfbnav.eval.horizon import horizon_problems
from pfbnav.utils.hashing import text_sha256
from pfbnav.world.scenarios import load_set

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
DATA = REPO / "tests" / "data" / "route_schedules.json"


def _rows(times, traversable=True):
    return [{"set": "a.json", "seed": i, "scenario_sha256": f"h{i}", "traversable": traversable,
             "schedule_time_s": t} for i, t in enumerate(times)]


def test_known_answers():
    sets = {"a.json": {"sha256": "x", "scenarios": ["h0", "h1"]}}
    ok = {"sets": {"a.json": "x"}, "rows": _rows([100.0, 150.0])}
    assert horizon_problems(ok, sets, max_steps=4000, dt=0.05, margin=1.3) == []
    # 1.3 x 160 = 208 s > 200 s
    over = {"sets": {"a.json": "x"}, "rows": _rows([100.0, 160.0])}
    assert len(horizon_problems(over, sets, max_steps=4000, dt=0.05, margin=1.3)) == 1
    # A route with no schedule (an unmapped obstacle across it) is not covered.
    blocked = {"sets": {"a.json": "x"}, "rows": _rows([None, 999.0], traversable=False)}
    assert horizon_problems(blocked, sets, max_steps=4000, dt=0.05, margin=1.3) == []
    # A built set with no schedules, a changed set, a missing row.
    assert "no schedules" in horizon_problems({"sets": {}, "rows": []}, sets, max_steps=4000,
                                              dt=0.05, margin=1.3)[0]
    assert "changed" in horizon_problems({"sets": {"a.json": "y"}, "rows": _rows([1.0, 1.0])},
                                         sets, max_steps=4000, dt=0.05, margin=1.3)[0]
    assert "without a schedule" in horizon_problems(
        {"sets": {"a.json": "x"}, "rows": _rows([1.0])}, sets, max_steps=4000, dt=0.05,
        margin=1.3)[0]
    with pytest.raises(ValueError):
        horizon_problems(ok, sets, max_steps=4000, dt=0.05, margin=0.9)


def _built() -> dict[str, dict]:
    out = {}
    for split in ("tune", "pilot", "train", "test"):
        for p in sorted((REPO / "scenarios" / split).glob("*.json")):
            out[p.relative_to(REPO).as_posix()] = {
                "sha256": text_sha256(p),
                "scenarios": [s["scenario_sha256"] for s in load_set(p)["scenarios"]]}
    return out


def test_the_horizon_covers_every_traversable_built_route():
    """The rule itself, over every built set (the test sets too once they are built). Fails until
    `scripts/route_schedules.py` has been run on the sets as they stand and its output committed."""
    assert DATA.is_file(), ("no tests/data/route_schedules.json: run "
                            "`python scripts/route_schedules.py --config configs/base.yaml --seed 0`")
    schedules = json.loads(DATA.read_text(encoding="utf-8"))
    r = CFG["reward"]
    problems = horizon_problems(schedules, _built(), max_steps=int(r["max_steps"]),
                                dt=float(CFG["sim"]["dt"]), margin=float(r["horizon_margin"]))
    assert not problems, problems[:10]
