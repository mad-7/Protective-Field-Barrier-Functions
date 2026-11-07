"""HF-c and the other classical-vs-classical comparisons of the sealed classical test evaluation (A36,
2026-11-07). Pure.

Both arms run once per scenario, so the A14a interval has the scenario part only: the rate ratio of arm a
over arm b (each arm's total protective stops + 0.5, over its total PLANNED route length, A35), its log
with a t interval on n - 1 df (`pfbnav.eval.rate_ratio`: `components` with K = 1 on both sides gives no
seed part, and Welch-Satterthwaite over the single scenario part is n - 1). `confirmatory_interval`
refuses two classical arms by design (H1 always has a learned arm); this is H1prime's and HF-c's form, as
endpoints.yaml states it.
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np

from pfbnav.eval.rate_ratio import (
    WS,
    arm_terms,
    components,
    corrected_interval,
    log_rate_ratio_terms,
)

ARMS = ("B1", "B2", "B3", "B6", "B7")


def classical_comparison(stops_a: Sequence[float], stops_b: Sequence[float],
                         planned_m: Sequence[float], level: float) -> dict[str, Any]:
    """Arm a over arm b on the same n scenarios (per-scenario stop counts, planned lengths in m), at
    two-sided `level`. `favours_a`: the log interval lies wholly below 0 (a has fewer stops)."""
    sa, sb, d = (np.asarray(x, dtype=float) for x in (stops_a, stops_b, planned_m))
    if not (sa.shape == sb.shape == d.shape) or sa.ndim != 1 or sa.size < 2:
        raise ValueError("one stop count per scenario for each arm and one planned length each, n >= 2")
    if (sa < 0).any() or (sb < 0).any() or (d <= 0).any():
        raise ValueError("stop counts are non-negative and planned lengths positive")
    a, b = arm_terms(sa[None, None, :], d[None, :]), arm_terms(sb[None, None, :], d[None, :])
    est = float(log_rate_ratio_terms(a, b)[0])
    var, half = corrected_interval(components(a, b), level, WS)
    lo, hi = est - float(half[0]), est + float(half[0])
    total = float(d.sum())
    return {"n": int(sa.size), "df": int(sa.size - 1), "level": level,
            "stops_a": int(sa.sum()), "stops_b": int(sb.sum()), "planned_m": total,
            "rate_a_per_100m": 100.0 * sa.sum() / total, "rate_b_per_100m": 100.0 * sb.sum() / total,
            "log_rate_ratio": est, "log_lower": lo, "log_upper": hi, "variance": float(var[0]),
            "rate_ratio": math.exp(est), "ci": [math.exp(lo), math.exp(hi)],
            "favours_a": hi < 0.0, "favours_b": lo > 0.0,
            "verdict": "a fewer" if hi < 0.0 else "a more" if lo > 0.0 else "inconclusive"}


def paired_table(records_by_arm: Mapping[str, Sequence[Mapping[str, Any]]],
                 arms: Sequence[str] = ARMS) -> dict[str, Any]:
    """Per-scenario stop counts (scan-fault stops excluded, A25) and planned lengths, aligned on the
    scenario identity. Refuses unless every arm has exactly one episode on every scenario and the arms
    agree on each scenario's planned length (the pairing)."""
    keyed: dict[str, dict[str, Mapping[str, Any]]] = {}
    for arm in arms:
        recs = records_by_arm.get(arm)
        if not recs:
            raise ValueError(f"no episodes for {arm}")
        by = {}
        for r in recs:
            key = str(r["scenario_sha256"])
            if key in by:
                raise ValueError(f"{arm} has two episodes on scenario {key[:12]}")
            by[key] = r
        keyed[arm] = by
    keys = sorted(keyed[arms[0]])
    for arm in arms[1:]:
        if sorted(keyed[arm]) != keys:
            raise ValueError(f"{arm} and {arms[0]} were not run on the same scenarios")
    planned = []
    for k in keys:
        lengths = {float(keyed[a][k]["plan_length_m"]) for a in arms}
        if max(lengths) - min(lengths) > 1e-9:
            raise ValueError(f"the arms disagree on the planned length of scenario {k[:12]}")
        planned.append(lengths.pop())
    stops = {a: [int(keyed[a][k]["protective_stops"]) - int(keyed[a][k].get("scan_fault_stops", 0))
                 for k in keys] for a in arms}
    if any(s < 0 for v in stops.values() for s in v):
        raise ValueError("scan-fault stops exceed protective stops in some episode")
    return {"scenarios": keys, "planned_m": planned, "stops": stops,
            "stratum": [keyed[arms[0]][k].get("stratum") for k in keys]}
