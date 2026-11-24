"""Offline classification of the protective stops and the timeouts that remain with the filter
(2026-11-24, for the paper's section "Remaining stops and timeouts"). Pure functions over the
per-episode records of `episodes.jsonl`; nothing here touches the simulator or changes a run.

**What it reads, and only that.** Per episode: `outcome`, `steps`, `distance_m`, `plan_length_m`,
`protective_stops`, `stationary_triggers`, `at_rest_triggers`, `pfb_infeasible`, `pfb_modes`,
`recovery_attempts`, `unmapped_count`. It assigns each timeout to one PROGRESS class and each stop to
the conditions its episode shows; it states no cause. A class is a description of what the record shows
(how far the robot got, whether the filter fell back, whether an obstacle was unmapped), declared here
before any run is read, so the edges are not fitted to a result.

Progress classes of a timeout (`progress = distance_m / plan_length_m`, edges `PROGRESS_EDGES`):
    "under_10"   < 0.10   the robot barely left the start;
    "10_to_50"   [0.10, 0.50);
    "50_to_90"   [0.50, 0.90);
    "over_90"    >= 0.90  nearly there at the horizon.
`distance_m` is the distance driven, which can exceed the plan (detours), so "over_90" includes it.
"""
from __future__ import annotations

from collections import Counter
from collections.abc import Iterable, Mapping
from typing import Any

PROGRESS_EDGES: tuple[float, ...] = (0.10, 0.50, 0.90)    # fractions of the planned length
PROGRESS_CLASSES: tuple[str, ...] = ("under_10", "10_to_50", "50_to_90", "over_90")


def progress_class(distance_m: float, plan_length_m: float) -> str:
    """The progress class of an episode: driven distance over planned length against the edges."""
    if plan_length_m <= 0.0:
        raise ValueError(f"plan_length_m must be positive, got {plan_length_m}")
    frac = distance_m / plan_length_m
    for edge, name in zip(PROGRESS_EDGES, PROGRESS_CLASSES):
        if frac < edge:
            return name
    return PROGRESS_CLASSES[-1]


def _brake_share(ep: Mapping[str, Any]) -> float:
    modes = ep.get("pfb_modes") or {}
    steps = int(ep["steps"])
    return float(modes.get("brake", 0)) / steps if steps > 0 else 0.0


def classify_timeouts(episodes: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
    """Counts of the timeouts by progress class, split by whether the route has an unmapped obstacle,
    and the mean share of steps the filter commanded maximum braking (`pfb_modes["brake"]`) per class."""
    table: dict[str, Counter] = {c: Counter() for c in PROGRESS_CLASSES}
    brake: dict[str, list[float]] = {c: [] for c in PROGRESS_CLASSES}
    recovery = 0
    n = 0
    for ep in episodes:
        if ep["outcome"] != "timeout":
            continue
        n += 1
        cls = progress_class(float(ep["distance_m"]), float(ep["plan_length_m"]))
        table[cls]["unmapped" if int(ep.get("unmapped_count", 0)) > 0 else "mapped_only"] += 1
        brake[cls].append(_brake_share(ep))
        if int(ep.get("recovery_attempts", 0)) > 0:
            recovery += 1
    return {
        "timeouts": n,
        "by_progress": {c: {"mapped_only": table[c]["mapped_only"], "unmapped": table[c]["unmapped"],
                            "mean_brake_share": (sum(brake[c]) / len(brake[c])) if brake[c] else None}
                        for c in PROGRESS_CLASSES},
        "with_recovery_attempts": recovery,
    }


def classify_stops(episodes: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
    """The protective stops that remain, by the conditions of their episode: with or without a filter
    fallback (`pfb_infeasible` > 0), at an unmapped-obstacle route or not, and the episode's outcome.
    Stops are attributed to an episode, not to a step: an episode with stops and a fallback counts ALL
    its stops in `in_episodes_with_fallback`, which says where they happened, not why."""
    total = in_fb = 0
    by_condition: Counter = Counter()
    by_outcome: Counter = Counter()
    stationary = at_rest = episodes_with_stops = 0
    for ep in episodes:
        k = int(ep["protective_stops"])
        if k == 0:
            continue
        episodes_with_stops += 1
        total += k
        if int(ep.get("pfb_infeasible", 0)) > 0:
            in_fb += k
        by_condition["unmapped" if int(ep.get("unmapped_count", 0)) > 0 else "mapped_only"] += k
        by_outcome[str(ep["outcome"])] += k
        stationary += int(ep.get("stationary_triggers", 0))
        at_rest += int(ep.get("at_rest_triggers", 0))
    return {"stops": total, "episodes_with_stops": episodes_with_stops,
            "in_episodes_with_fallback": in_fb, "in_episodes_without_fallback": total - in_fb,
            "by_route_condition": dict(by_condition), "by_outcome": dict(by_outcome),
            "stationary_triggers": stationary, "at_rest_triggers": at_rest}
