"""Per-step trace analysis for the offline diagnostics ruled 2026-11-07 (items 3a, 3b, 3e). Pure.

A trace is the list of per-step rows `scripts/trace_episodes.py` writes: the layout-frame pose
(x, y, yaw), the route arc reached `s_m`, the cross-track distance, the goal distance, the requested
and applied commands, the certified layer's flags, the PFB's decision (or None) and, at a stop
trigger, the distance from the footprint to the nearest MAPPED and nearest UNMAPPED surface.

* **Wandering anatomy (3a, and item 4 of the additional rulings):** where along the route the excess
  driving accumulates (the driven distance in each third of the route's arc, by the arc reached at the
  step), the lateral offset from the planned path, goal overshoot (the goal distance rising again
  after a closest approach inside `near_goal_m`) and circling (accumulated heading turn while near the
  goal), and an oscillation measure (sign changes per second of the commanded lateral speed and yaw
  rate).
* **Stop attribution (3b):** each protective-stop trigger is labelled by the FIRST cause that holds,
  in this order: `pfb_fallback` (the PFB's QP was infeasible on the trigger step or the `lookback`
  steps before it), `unmapped_in_field` (the nearest surface to the footprint is an unmapped
  obstacle), `filter_active_no_fallback` (the PFB filtered or passed the command and still the field
  was breached -- prediction lag, the velocity estimate or the composition), `no_filter` (an arm
  without the PFB), else `other`.
"""
from __future__ import annotations

import itertools
import math
from collections.abc import Sequence
from typing import Any

import numpy as np

CAUSES = ("pfb_fallback", "unmapped_in_field", "filter_active_no_fallback", "no_filter", "other")


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wandering(rows: Sequence[dict[str, Any]], *, plan_length_m: float, dt: float,
              near_goal_m: float = 1.0, fast_mps: float = 0.45) -> dict[str, Any]:
    """The anatomy of one episode's driving (module docstring). Distances in m, rates per s."""
    if len(rows) < 2:
        raise ValueError("a trace needs at least two steps")
    xy = np.array([[r["pose"][0], r["pose"][1]] for r in rows], dtype=float)
    step_d = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.array([float(r["s_m"]) for r in rows])[1:]
    thirds = np.minimum((3.0 * s / plan_length_m).astype(int), 2) if plan_length_m > 0 else \
        np.zeros_like(s, dtype=int)
    by_third = [float(step_d[thirds == k].sum()) for k in range(3)]
    cross = np.abs(np.array([float(r["cross_track_m"]) for r in rows]))
    goal = np.array([float(r["goal_distance_m"]) for r in rows])
    yaw = np.array([float(r["pose"][2]) for r in rows])
    turn = np.abs([_wrap(b - a) for a, b in itertools.pairwise(yaw)])
    near = goal[1:] <= near_goal_m
    closest = int(np.argmin(goal))
    overshoot = bool(goal[closest] <= near_goal_m and goal[closest:].max() > goal[closest] + 0.05)
    req = np.array([r["requested"] for r in rows], dtype=float)

    def flips(x: np.ndarray, eps: float) -> float:
        sig = np.sign(np.where(np.abs(x) > eps, x, 0.0))
        sig = sig[sig != 0]
        return float(np.count_nonzero(np.diff(sig))) / (len(rows) * dt)

    return {"driven_m": float(step_d.sum()), "plan_length_m": float(plan_length_m),
            "driven_over_plan": float(step_d.sum()) / plan_length_m if plan_length_m > 0 else None,
            "driven_by_route_third_m": by_third,
            "cross_track_mean_m": float(cross.mean()), "cross_track_p95_m": float(
                np.percentile(cross, 95)), "cross_track_max_m": float(cross.max()),
            "closest_goal_m": float(goal[closest]), "goal_overshoot": overshoot,
            "steps_near_goal": int(near.sum()),
            "turn_near_goal_rad": float(turn[near].sum()),
            "circling_near_goal": bool(turn[near].sum() > 2.0 * math.pi),
            "lateral_flips_per_s": flips(req[:, 1], 0.01),
            "yaw_flips_per_s": flips(req[:, 2], 0.02),
            "mean_abs_command_change": float(np.abs(np.diff(req, axis=0)).sum(axis=1).mean()),
            # The PFB's decisions over the episode (None for an arm without the filter): the share
            # of steps it changed the command (filtered, brake or fallback), and the latched share.
            "pfb_intervention_share": _share(rows, lambda m: m in ("filtered", "brake", "fallback")),
            "pfb_fallback_share": _share(rows, lambda m: m == "fallback"),
            "latched_share": float(np.mean([bool(r["certified"].get("stop_active")) for r in rows])),
            # How fast the planner ASKS to go (before the filter): the median requested planar speed
            # and the share of steps at or above `fast_mps` (does it lean on the filter for speed?).
            "requested_speed_median_mps": float(np.median(np.hypot(req[:, 0], req[:, 1]))),
            "requested_fast_share": float(np.mean(np.hypot(req[:, 0], req[:, 1]) >= fast_mps))}


def _share(rows: Sequence[dict[str, Any]], pick) -> float | None:
    """The share of steps whose PFB mode satisfies `pick`; None if no step carries a PFB decision."""
    modes = [r["pfb"]["mode"] for r in rows if r.get("pfb")]
    return float(np.mean([bool(pick(m)) for m in modes])) if modes else None


def stop_causes(rows: Sequence[dict[str, Any]], *, lookback: int = 3) -> list[dict[str, Any]]:
    """One row per protective-stop trigger, labelled by its first holding cause (docstring)."""
    out = []
    for i, r in enumerate(rows):
        if not r["certified"].get("stop_triggered"):
            continue
        window = rows[max(0, i - lookback): i + 1]
        pfbs = [w.get("pfb") for w in window]
        near = r.get("nearest_surface") or {}
        mapped, unmapped = near.get("mapped_m"), near.get("unmapped_m")
        if any(p is not None and p.get("infeasible") for p in pfbs):
            cause = "pfb_fallback"
        elif unmapped is not None and (mapped is None or unmapped < mapped):
            cause = "unmapped_in_field"
        elif any(p is not None for p in pfbs):
            cause = "filter_active_no_fallback"
        elif all(p is None for p in pfbs):
            cause = "no_filter"
        else:
            cause = "other"
        out.append({"step": int(r["step"]), "cause": cause, "s_m": float(r["s_m"]),
                    "nearest_mapped_m": mapped, "nearest_unmapped_m": unmapped,
                    "pfb_modes": [None if p is None else p.get("mode") for p in pfbs]})
    return out


def latch_anatomy(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """For B1's failures (3e): stop triggers, releases, the latched share of steps, the longest
    latched run, the steps spent in recovery modes, and the last step's state."""
    active = np.array([bool(r["certified"].get("stop_active")) for r in rows])
    longest = cur = 0
    for a in active:
        cur = cur + 1 if a else 0
        longest = max(longest, cur)
    modes: dict[str, int] = {}
    for r in rows:
        m = (r.get("recovery") or {}).get("mode")
        if m is not None:
            modes[str(m)] = modes.get(str(m), 0) + 1
    releases = int(np.count_nonzero(active[:-1] & ~active[1:]))
    return {"steps": len(rows),
            "triggers": int(sum(bool(r["certified"].get("stop_triggered")) for r in rows)),
            "releases": releases, "latched_share": float(active.mean()),
            "longest_latch_steps": int(longest),
            "recovery_mode_steps": modes,
            "final": {k: rows[-1]["certified"].get(k) for k in
                      ("stop_active", "restart_blocked", "breached", "at_rest")},
            "outcome": rows[-1].get("outcome")}
