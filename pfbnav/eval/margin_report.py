"""The planner-margin quantification's results: parsing, merging interrupted runs, summarising.
Pure functions.

`scripts/quantify_margins.py` writes one `routes.csv` row per built scenario and candidate
(lethal, soft) pair (ruled 2026-10-13, item 3). A run can stop part-way -- the first one did, at
1,938 of 2,271 scenarios, with its last scenario's rows cut short -- so a report is assembled from
several runs: only scenarios with a row for EVERY pair count, each scenario once, and a scenario
present complete in two runs must agree there (the planning is deterministic) or the merge refuses.
"""
from __future__ import annotations

from collections.abc import Iterable, Sequence
from typing import Any

INT_FIELDS = ("samples", "breached_samples", "offset_breached_samples", "breached_start",
              "breached_goal", "breached_bend", "breached_straight")
FLOAT_FIELDS = ("lethal_margin", "soft_margin", "path_length_m", "min_clearance_m", "worst_x",
                "worst_y", "worst_yaw", "offset_min_clearance_m", "min_clearance_outside_goal_m")
LOCATIONS = ("start", "goal", "bend", "straight")


def parse_row(raw: dict[str, str]) -> dict[str, Any]:
    """A `routes.csv` row with its numbers as numbers (empty cells -- an unplannable route's
    measurements -- stay None)."""
    row: dict[str, Any] = dict(raw)
    for k in INT_FIELDS:
        if k in row:
            row[k] = int(row[k]) if row[k] not in ("", None) else None
    for k in FLOAT_FIELDS:
        if k in row:
            row[k] = float(row[k]) if row[k] not in ("", None) else None
    return row


def key(row: dict[str, Any]) -> tuple[str, str]:
    return str(row["set"]), str(row["seed"])


def complete_scenarios(rows: Iterable[dict[str, Any]], pairs: set[tuple[float, float]]
                       ) -> tuple[dict[tuple[str, str], list[dict[str, Any]]], list[tuple[str, str]]]:
    """Group rows by scenario; keep those with exactly one row per pair. Returns (complete,
    incomplete keys)."""
    groups: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for r in rows:
        groups.setdefault(key(r), []).append(r)
    complete, incomplete = {}, []
    for k, rs in groups.items():
        got = [(float(r["lethal_margin"]), float(r["soft_margin"])) for r in rs]
        if sorted(got) == sorted(pairs) and len(got) == len(pairs):
            complete[k] = rs
        else:
            incomplete.append(k)
    return complete, incomplete


def merge_runs(runs: Sequence[Iterable[dict[str, Any]]], pairs: set[tuple[float, float]]
               ) -> dict[str, Any]:
    """Every complete scenario across `runs`, once. Returns {"rows", "scenarios", "incomplete",
    "duplicates"}; raises if a scenario complete in two runs differs between them."""
    merged: dict[tuple[str, str], list[dict[str, Any]]] = {}
    incomplete: set[tuple[str, str]] = set()
    duplicates = 0
    for rows in runs:
        complete, partial = complete_scenarios(rows, pairs)
        incomplete |= set(partial)
        for k, rs in complete.items():
            if k in merged:
                duplicates += 1
                order = sorted(rs, key=lambda r: (r["lethal_margin"], r["soft_margin"]))
                prior = sorted(merged[k], key=lambda r: (r["lethal_margin"], r["soft_margin"]))
                if order != prior:
                    raise ValueError(f"scenario {k} differs between runs")
            else:
                merged[k] = rs
    return {"rows": [r for rs in merged.values() for r in rs], "scenarios": sorted(merged),
            "incomplete": sorted(incomplete - set(merged)), "duplicates": duplicates}


def summarise(rows: list[dict[str, Any]], quant: list[dict[str, Any]],
              cfg: dict[str, Any]) -> list[dict[str, Any]]:
    pm = cfg["planner_margins"]
    out = []
    for lethal in pm["lethal_margins"]:
        for soft in pm["soft_margins"]:
            mine = [r for r in rows if r["lethal_margin"] == lethal and r["soft_margin"] == soft]
            planned = [r for r in mine if r["status"] == "ok"]
            per_stratum = {}
            for s in sorted({r["stratum"] for r in mine}):
                rs = [r for r in mine if r["stratum"] == s]
                ok = [r for r in rs if r["status"] == "ok"]
                per_stratum[s] = {"scenarios": len(rs), "plannable": len(ok),
                                  "breaching_routes": sum(r["breached_samples"] > 0 for r in ok),
                                  "offset_breaching_routes": sum(r["offset_breached_samples"] > 0
                                                                 for r in ok)}
            q = [x for x in quant if x["lethal_margin"] == lethal and x["soft_margin"] == soft]
            samples = sum(r["samples"] for r in planned)
            entry = {
                "lethal_margin": lethal, "soft_margin": soft,
                "lethal_radius_m": float(cfg["robot"]["half_width"]) + lethal,
                "scenarios": len(mine), "plannable": len(planned),
                "quantisation_failures": sum(len(x["failures"]) for x in q),
                "breaching_routes": sum(r["breached_samples"] > 0 for r in planned),
                "breached_sample_share": (sum(r["breached_samples"] for r in planned) / samples
                                          if samples else None),
                "worst_clearance_m": min((r["min_clearance_m"] for r in planned), default=None),
                "offset_breaching_routes": sum(r["offset_breached_samples"] > 0 for r in planned),
                "breaching_routes_by_location": {
                    c: sum(r[f"breached_{c}"] > 0 for r in planned)
                    for c in LOCATIONS},
                "breaching_routes_outside_goal_approach": sum(
                    (r["breached_start"] + r["breached_bend"] + r["breached_straight"]) > 0
                    for r in planned),
                "offset_breached_sample_share": (sum(r["offset_breached_samples"] for r in planned)
                                                 / samples if samples else None),
                "per_stratum": per_stratum,
            }
            entry["plannability_holds"] = (entry["plannable"] == entry["scenarios"]
                                           and entry["quantisation_failures"] == 0)
            entry["criterion_holds"] = entry["plannability_holds"] and entry["breaching_routes"] == 0
            out.append(entry)
    return out


def smallest_meeting(table: Sequence[dict[str, Any]]) -> tuple[float, float] | None:
    """The smallest (lethal, soft) pair meeting the criterion, lethal first; None if none does."""
    meeting = [e for e in table if e["criterion_holds"]]
    if not meeting:
        return None
    best = min(meeting, key=lambda e: (e["lethal_margin"], e["soft_margin"]))
    return best["lethal_margin"], best["soft_margin"]
