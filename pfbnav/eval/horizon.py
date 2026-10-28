"""The episode horizon's derivation rule (ruled 2026-10-20, A23). Pure.

`reward.max_steps` is a dependent of v_max under A5; after A20 it was never re-derived, and at 1500
steps (75 s) even the privileged oracle tracker could finish only 10 of the 60 tune routes. Ruled:

  **the horizon covers the privileged oracle schedule of every traversable built route, with
  margin** -- `margin x schedule_time <= max_steps x dt` for every route whose schedule exists --

checked over every built set (tune, pilot, train; and the test sets once they are built), from the
schedules `scripts/route_schedules.py` computes offline. The margin covers what the schedule leaves
out (acceleration, the tracker's own lag): emulated completions ran 1.12-1.25 x their schedules on
eight tune routes (2026-10-20). A route an unmapped obstacle makes non-traversable has no schedule
and is not covered -- it is the encounter the methods are evaluated on, not a mission the oracle can
finish.

`horizon_problems` names every way a schedules file fails to establish the rule; empty means it holds.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any


def horizon_problems(schedules: Mapping[str, Any], sets: Mapping[str, Mapping[str, Any]],
                     *, max_steps: int, dt: float, margin: float) -> list[str]:
    """`schedules` is `scripts/route_schedules.py`'s output: `{"sets": {path: sha256}, "rows":
    [...]}`; `sets` the built sets now on disk, `{path: {"sha256": ..., "scenarios": [hashes]}}`.
    Problems: a built set missing from the schedules or changed since; a scenario without a row; a
    traversable route whose margin x schedule time exceeds the horizon."""
    if margin < 1.0 or max_steps <= 0 or dt <= 0.0:
        raise ValueError("margin must be >= 1, and max_steps and dt positive")
    horizon = max_steps * dt
    out: list[str] = []
    recorded = dict(schedules.get("sets", {}))
    rows = list(schedules.get("rows", []))
    by_set: dict[str, set[str]] = {}
    for r in rows:
        by_set.setdefault(r["set"], set()).add(r["scenario_sha256"])
    for path, built in sets.items():
        if path not in recorded:
            out.append(f"{path}: built but has no schedules (run scripts/route_schedules.py)")
            continue
        if recorded[path] != built["sha256"]:
            out.append(f"{path}: changed since its schedules were computed")
            continue
        missing = set(built["scenarios"]) - by_set.get(path, set())
        if missing:
            out.append(f"{path}: {len(missing)} scenario(s) without a schedule row")
    for r in rows:
        t = r.get("schedule_time_s")
        if r.get("traversable") and t is not None and margin * float(t) > horizon:
            out.append(f"{r['set']} seed {r['seed']}: {margin} x {float(t):.1f} s = "
                       f"{margin * float(t):.1f} s exceeds the {horizon:.0f} s horizon")
    return out


def summary(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Per set: routes, traversable, the largest schedule time, the allowances used."""
    out: dict[str, Any] = {}
    for r in rows:
        s = out.setdefault(r["set"], {"routes": 0, "traversable": 0, "max_schedule_s": 0.0,
                                      "allowances": {}})
        s["routes"] += 1
        if r.get("traversable"):
            s["traversable"] += 1
            s["max_schedule_s"] = max(s["max_schedule_s"], float(r["schedule_time_s"]))
            key = f"{float(r['yaw_allowance']):.2f}"
            s["allowances"][key] = s["allowances"].get(key, 0) + 1
    return out
