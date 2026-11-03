"""B7's timeouts, classified against the privileged oracle tracker (ruled 2026-11-03). Pure.

The supervisor's two classes, each decided by what the oracle did on the SAME scenario (it tracks
the mapped route, never replans, and knows every obstacle):
  * **(i) blocked route:** the oracle does not succeed AND the scenario has an unmapped obstacle --
    an obstacle across the mapped route that a non-replanning tracker cannot pass;
  * **(ii) filter deadlock:** the oracle succeeds -- the route is traversable, and B7 does not get
    there.
Anything else is not forced into either: **(iii) unclassified**, the oracle failing on a route with
no unmapped obstacle. Every row also carries what the episode logs say about HOW B7 timed out --
the distance it travelled against the planned length, its mean speed over the episode (distance / (steps x `dt`)), the
nearest unmapped obstacle's surface offset from the route, and the PFB's modified steps -- because
"never got going", "slow on a long route" and "stuck short of an obstacle" are different findings
that the two classes do not separate.
"""
from __future__ import annotations

from collections.abc import Sequence
from typing import Any

BLOCKED, DEADLOCK, UNCLASSIFIED = "blocked_route", "filter_deadlock", "unclassified"


def classify_timeouts(b7: Sequence[dict[str, Any]], oracle: Sequence[dict[str, Any]],
                      dt: float) -> list[dict[str, Any]]:
    """One row per B7 episode whose outcome is `timeout`, in scenario order. `oracle` must hold
    exactly one episode per scenario B7 ran (raises otherwise: the two did not run the same set)."""
    by_seed = {int(e["scenario_seed"]): e for e in oracle}
    if len(by_seed) != len(oracle) or set(by_seed) != {int(e["scenario_seed"]) for e in b7}:
        raise ValueError("the oracle's episodes are not one per scenario of B7's set")
    rows = []
    for e in sorted(b7, key=lambda x: int(x["scenario_seed"])):
        if e["outcome"] != "timeout":
            continue
        o = by_seed[int(e["scenario_seed"])]
        unmapped = int(e["unmapped_count"])
        if o["outcome"] == "success":
            cls = DEADLOCK
        elif unmapped > 0:
            cls = BLOCKED
        else:
            cls = UNCLASSIFIED
        offsets = [float(x) for x in e.get("unmapped_offsets_m") or []]
        plan = float(e["plan_length_m"])
        rows.append({"scenario_seed": int(e["scenario_seed"]), "stratum": e["stratum"],
                     "class": cls, "oracle_outcome": o["outcome"], "unmapped_count": unmapped,
                     "nearest_unmapped_offset_m": min(offsets) if offsets else None,
                     "plan_length_m": plan, "distance_m": float(e["distance_m"]),
                     "travelled_share": float(e["distance_m"]) / plan if plan > 0 else None,
                     "mean_speed_mps": float(e["distance_m"]) / (int(e["steps"]) * dt),
                     "pfb_modified_steps": int(e.get("pfb_modified", 0)),
                     "pfb_modified_share": int(e.get("pfb_modified", 0)) / max(1, int(e["steps"])),
                     "protective_stops": int(e.get("protective_stops", 0))})
    return rows


def class_counts(rows: Sequence[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    """Per stratum and overall: the count of each class and its scenario IDs."""
    out: dict[str, dict[str, Any]] = {}
    for key in sorted({r["stratum"] for r in rows}) + ["all"]:
        mine = [r for r in rows if key == "all" or r["stratum"] == key]
        out[key] = {c: {"count": sum(r["class"] == c for r in mine),
                        "ids": [r["scenario_seed"] for r in mine if r["class"] == c]}
                    for c in (BLOCKED, DEADLOCK, UNCLASSIFIED)}
    return out


def route_speeds(episodes: Sequence[dict[str, Any]], seeds: Sequence[int], dt: float
                 ) -> list[dict[str, Any]]:
    """One controller's episodes on the given scenarios (ruled 2026-11-03: B2 against B7 on B7's 13
    horizon timeouts): outcome, distance, the mean speed over the episode, and the mean speed over the
    steps NOT latched by the certified layer (`stopped_steps` excluded), which separates time spent
    stopped from how fast the controller drives. Raises if a scenario is missing or repeated."""
    by_seed: dict[int, dict[str, Any]] = {}
    for e in episodes:
        s = int(e["scenario_seed"])
        if s in seeds:
            if s in by_seed:
                raise ValueError(f"scenario {s} appears twice")
            by_seed[s] = e
    missing = [s for s in seeds if s not in by_seed]
    if missing:
        raise ValueError(f"scenarios without an episode: {missing}")
    rows = []
    for s in seeds:
        e = by_seed[s]
        steps, stopped = int(e["steps"]), int(e.get("stopped_steps", 0))
        moving = steps - stopped
        rows.append({"scenario_seed": s, "outcome": e["outcome"], "distance_m": float(e["distance_m"]),
                     "steps": steps, "stopped_steps": stopped,
                     "protective_stops": int(e.get("protective_stops", 0)),
                     "mean_speed_mps": float(e["distance_m"]) / (steps * dt),
                     "moving_speed_mps": float(e["distance_m"]) / (moving * dt) if moving else None})
    return rows
