"""Spread of G1b results across separate invocations, and the verdict it supports.

Repetitions inside one process are bit-identical (docs/reproducibility.md), so they carry
no information about variation; separate invocations do. The supervisor's ruling on the
G1b re-run under the wheel budget (2026-09-19): report the spread across repetitions, and
where the repetitions straddle a limit, the verdict at that speed is FAIL with the margin
stated -- a single repetition 0.053 deg over a 5.0 deg limit does not establish a ceiling.

Pure functions; the criteria are passed in from `preregistration/gates.yaml`, never
restated here.
"""
from __future__ import annotations

from collections.abc import Sequence
from typing import Any

# Metric in each run record -> criterion key in G1b_closed_loop_tracking.
METRICS: dict[str, str] = {
    "max_abs_cross_track": "max_cross_track",
    "rms_cross_track": "rms_cross_track",
    "final_heading_error_deg": "max_final_heading_error_deg",
}


def _ceiling(speeds: Sequence[float], passed: dict[float, bool]) -> float | None:
    """Highest speed passing with every lower speed passing too (A6); None if the lowest fails."""
    ceiling = None
    for speed in sorted(speeds):
        if not passed[speed]:
            break
        ceiling = speed
    return ceiling


def cell_spread(values: Sequence[float], limit: float) -> dict[str, Any]:
    """Spread of one metric over every run of one profile at one speed, against its limit.

    `margin_worst` is limit minus the largest value: negative means at least one run is
    over. `status` is "pass" (every run within), "fail" (none within) or "straddle".
    """
    if not values:
        raise ValueError("no values")
    lo, hi = float(min(values)), float(max(values))
    within = sum(v <= limit for v in values)
    status = "pass" if within == len(values) else "fail" if within == 0 else "straddle"
    return {"min": lo, "max": hi, "spread": hi - lo, "limit": float(limit),
            "margin_worst": float(limit) - hi, "margin_best": float(limit) - lo,
            "runs_within": int(within), "runs": len(values), "status": status}


def within_invocation_identical(runs: Sequence[dict[str, Any]]) -> bool:
    """Whether every repetition of each (profile, speed) in one invocation is identical.

    The determinism claim (one process, bit-identical); a False here is itself a finding.
    """
    seen: dict[tuple[str, float], tuple[float, ...]] = {}
    for r in runs:
        key = (r["profile"], float(r["speed"]))
        values = tuple(float(r[m]) for m in METRICS)
        if key in seen and seen[key] != values:
            return False
        seen.setdefault(key, values)
    return True


def spread_verdict(invocations: Sequence[Sequence[dict[str, Any]]],
                   criteria: dict[str, Any]) -> dict[str, Any]:
    """Per-cell spread over all invocations and repetitions, and the ceilings it supports.

    A speed passes only if every profile passes in every run of every invocation. A cell
    that straddles a limit fails, and its margin is reported with it.
    """
    if not invocations or not all(invocations):
        raise ValueError("need at least one invocation, each with runs")
    all_runs = [r for inv in invocations for r in inv]
    cells: dict[str, dict[str, Any]] = {}
    for profile in sorted({r["profile"] for r in all_runs}):
        for speed in sorted({float(r["speed"]) for r in all_runs if r["profile"] == profile}):
            at = [r for r in all_runs if r["profile"] == profile and float(r["speed"]) == speed]
            metrics = {}
            for m, c in METRICS.items():
                limit = float(criteria[c])
                metrics[m] = cell_spread([float(r[m]) for r in at], limit)
                # Repetitions in one invocation are identical, so the evidence is counted
                # per invocation: an invocation is within if its worst repetition is.
                worst = [max(float(r[m]) for r in inv
                             if r["profile"] == profile and float(r["speed"]) == speed)
                         for inv in invocations
                         if any(r["profile"] == profile and float(r["speed"]) == speed
                                for r in inv)]
                metrics[m]["invocations_within"] = sum(w <= limit for w in worst)
                metrics[m]["invocations"] = len(worst)
            statuses = {s["status"] for s in metrics.values()}
            status = "pass" if statuses == {"pass"} else \
                     "straddle" if "straddle" in statuses else "fail"
            cells[f"{profile}@{speed:.2f}"] = {
                "profile": profile, "speed": speed, "runs": len(at),
                "invocations": sum(any(r["profile"] == profile and float(r["speed"]) == speed
                                       for r in inv) for inv in invocations),
                "status": status, "metrics": metrics}

    grid = sorted(float(v) for v in criteria["speeds"])
    profiles = set(criteria["profiles"])
    per_speed = {}
    for speed in grid:
        here = {c["profile"]: c for c in cells.values() if c["speed"] == speed}
        missing = sorted(profiles - set(here))
        failing = sorted(p for p, c in here.items() if c["status"] != "pass")
        per_speed[speed] = {"all_pass": not missing and not failing,
                            "failing": failing, "missing": missing,
                            "straddling": sorted(p for p, c in here.items()
                                                 if c["status"] == "straddle")}
    v_max = _ceiling(grid, {v: per_speed[v]["all_pass"] for v in grid})

    lateral = {c["speed"]: c["status"] == "pass" for c in cells.values()
               if c["profile"] == "lateral"}
    vy_max = _ceiling(sorted(lateral), lateral) if lateral else None

    return {
        "invocations": len(invocations),
        "within_invocation_identical": [within_invocation_identical(inv)
                                        for inv in invocations],
        "cells": cells, "per_speed": per_speed,
        "v_max_from_gate": v_max, "vy_max_from_gate": vy_max,
        "result": "PASS" if v_max is not None else "FAIL",
    }


def margin_statement(cell: dict[str, Any]) -> str:
    """One line naming each limit a cell fails or straddles, with its range and margin."""
    parts = []
    for name, s in cell["metrics"].items():
        if s["status"] == "pass":
            continue
        parts.append(f"{name} {s['min']:.4g}-{s['max']:.4g} vs {s['limit']:.4g} "
                     f"({s['invocations_within']}/{s['invocations']} invocations within, "
                     f"worst margin "
                     f"{s['margin_worst']:+.4g}, {s['status']})")
    return f"{cell['profile']} at {cell['speed']:.2f} m/s: " + "; ".join(parts) if parts else ""


def cell_verdict(cell: dict[str, Any]) -> str:
    """A cell's gate verdict: PASS only if every run is within every limit (straddle fails)."""
    return "PASS" if cell["status"] == "pass" else "FAIL"


def regression(reference: dict[str, dict[str, Any]], new: dict[str, dict[str, Any]],
               cells: Sequence[str]) -> dict[str, Any]:
    """Compare named cells' verdicts between a reference spread and a regression spread.

    The pre-declared rule (supervisor, 2026-09-21): if any verdict changes, G1b is re-run in
    full. A cell missing from either side is an error, never a silent pass.
    """
    missing = [c for c in cells if c not in reference or c not in new]
    if missing:
        raise ValueError(f"cells missing from the reference or the regression: {missing}")
    rows = {}
    for c in cells:
        before, after = cell_verdict(reference[c]), cell_verdict(new[c])
        rows[c] = {"reference": before, "regression": after,
                   "reference_status": reference[c]["status"],
                   "regression_status": new[c]["status"], "changed": before != after}
    changed = [c for c, r in rows.items() if r["changed"]]
    return {"cells": rows, "changed": changed, "full_rerun_required": bool(changed)}
