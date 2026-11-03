"""Diagnosing a training run that failed G4 (ruled 2026-10-30, before L1's pilot restarts). Pure.

The supervisor asked, of L1's seed 9000000: the entropy coefficient over training, the Q-values'
sign and scale, the per-episode reward components ("is progress ever positive?"), the action
statistics (saturated, or stuck at zero), and the observation ranges against their normalisation.

**What the run's logs hold, and what they do not.** `sb3/progress.csv` has the entropy coefficient
and the losses; `episodes.jsonl` has each episode's return, length, outcome, stops, distance and
latched steps. Neither holds a Q-value, an action or an observation, and the step reward's
progress and proximity terms are summed into the return. So:
  * from the logs: the entropy trajectory, and each episode's return split into the terms its
    record fixes (time, stops, goal, contact) and a residual = progress - proximity - intervention,
    which the log cannot separate. Progress is never negative (the tracker projects forward), so
    the residual is a LOWER bound on the progress term and the proximity penalty is at least
    -residual;
  * from a short replay of the run's checkpoints (`scripts/diag_training_run.py --replay`): Q, the
    actions, the observations and every reward term per step, measured directly.
"""
from __future__ import annotations

import itertools
import math
from collections.abc import Sequence
from typing import Any

import numpy as np

SATURATED = 0.95       # |a| at or above this, on an axis normalised to [-1, 1]
NEAR_ZERO = 0.05       # every axis at or below this: the command is ~nothing


def column_trajectory(rows: Sequence[dict[str, Any]], column: str,
                      fractions: Sequence[float] = (0.0, 0.1, 0.25, 0.5, 0.75, 1.0)
                      ) -> dict[str, Any]:
    """A progress.csv column over training: its value at each fraction of the logged rows (the
    nearest row with a finite value), with its min and max. Rows missing the column are skipped."""
    values = []
    for r in rows:
        v = r.get(column)
        if v in (None, ""):
            continue
        v = float(v)
        if math.isfinite(v):
            values.append((float(r.get("time/total_timesteps", len(values))), v))
    if not values:
        return {"column": column, "rows": 0}
    out: dict[str, Any] = {"column": column, "rows": len(values),
                           "min": min(v for _, v in values), "max": max(v for _, v in values),
                           "at": {}}
    for f in fractions:
        i = min(len(values) - 1, round(f * (len(values) - 1)))
        out["at"][f"{f:g}"] = {"timestep": values[i][0], "value": values[i][1]}
    return out


def reward_split(episode: dict[str, Any], weights: dict[str, float]) -> dict[str, float]:
    """One logged episode's return, split into the terms its record fixes and the residual.

    time = -time_cost x length; stop = -k_stop x protective stops; goal / contact from the outcome;
    residual = return - those = progress - proximity - intervention. `progress_lower_bound` =
    max(residual, 0) is a lower bound on the progress term (proximity and intervention are never
    rewards), and `proximity_at_least` = max(-residual, 0) bounds the penalties from below."""
    length = int(episode["episode_length"])
    stops = int(episode.get("protective_stops", 0))
    outcome = str(episode.get("outcome", ""))
    terms = {"time": -float(weights["time_cost"]) * length,
             "stop": -float(weights["k_stop"]) * stops,
             "goal": float(weights["goal_bonus"]) if outcome == "success" else 0.0,
             "contact": -float(weights["contact_penalty"]) if outcome == "collision" else 0.0}
    residual = float(episode["episode_return"]) - sum(terms.values())
    return {**terms, "residual": residual, "progress_lower_bound": max(residual, 0.0),
            "penalties_at_least": max(-residual, 0.0)}


def action_stats(actions: np.ndarray) -> dict[str, Any]:
    """Per-axis mean, std and saturated share (|a| >= SATURATED); the share of steps whose every
    axis is near zero (|a| <= NEAR_ZERO). `actions` is (steps, axes) in [-1, 1]."""
    a = np.asarray(actions, dtype=float)
    if a.ndim != 2 or a.shape[0] == 0:
        raise ValueError("actions must be a non-empty (steps, axes) array")
    return {"steps": int(a.shape[0]), "mean": a.mean(axis=0).tolist(),
            "std": a.std(axis=0).tolist(),
            "saturated_share": (np.abs(a) >= SATURATED).mean(axis=0).tolist(),
            "all_axes_saturated_share": float(np.all(np.abs(a) >= SATURATED, axis=1).mean()),
            "near_zero_share": float(np.all(np.abs(a) <= NEAR_ZERO, axis=1).mean())}


def value_stats(q: np.ndarray) -> dict[str, Any]:
    """Sign and scale of a set of Q-values: mean, percentiles, share negative."""
    v = np.asarray(q, dtype=float).ravel()
    if v.size == 0:
        raise ValueError("no values")
    if not np.all(np.isfinite(v)):
        return {"n": int(v.size), "non_finite": int((~np.isfinite(v)).sum())}
    p = np.percentile(v, [0, 5, 50, 95, 100])
    return {"n": int(v.size), "non_finite": 0, "mean": float(v.mean()),
            "min": float(p[0]), "p05": float(p[1]), "median": float(p[2]), "p95": float(p[3]),
            "max": float(p[4]), "negative_share": float((v < 0.0).mean())}


def observation_ranges(obs: np.ndarray, low: np.ndarray, high: np.ndarray,
                       blocks: Sequence[tuple[str, int]]) -> dict[str, Any]:
    """Per block: observed min and max against the declared bounds, values outside them, and
    channels that never moved. `blocks` is the layout's (name, width) in order."""
    o = np.asarray(obs, dtype=float)
    lo, hi = np.asarray(low, dtype=float), np.asarray(high, dtype=float)
    if o.ndim != 2 or o.shape[1] != lo.size or lo.size != hi.size:
        raise ValueError("obs must be (steps, dim) with bounds of the same dim")
    if sum(w for _, w in blocks) != lo.size:
        raise ValueError("the blocks do not cover the observation")
    out: dict[str, Any] = {}
    start = 0
    for name, width in blocks:
        sl = slice(start, start + width)
        x = o[:, sl]
        out[name] = {"observed_min": float(x.min()), "observed_max": float(x.max()),
                     "bound_min": float(lo[sl].min()), "bound_max": float(hi[sl].max()),
                     "outside_bounds": int(((x < lo[sl]) | (x > hi[sl])).sum()),
                     "constant_channels": int((x.max(axis=0) - x.min(axis=0) == 0.0).sum()),
                     "channels": width}
        start += width
    return out


# --- L1 seed 9000001's decline (ruled 2026-11-03, option b) ---------------------------------------
# The replay buffer is not saved with a run, but it is FIFO of the last `buffer_size` transitions and
# each logged episode occupies the steps (timestep - episode_length, timestep], so its composition at
# any step is reconstructed exactly from `episodes.jsonl` -- except the episode still running at that
# step, whose transitions are counted as `unfinished`.

def success_by_timestep(episodes: Sequence[dict[str, Any]], edges: Sequence[int]
                        ) -> list[dict[str, Any]]:
    """Episodes binned by the step they ENDED at, (edges[i], edges[i+1]]; per bin the episode
    count, the success share (None if empty) and the curriculum stages seen."""
    out = []
    for lo, hi in itertools.pairwise(edges):
        part = [e for e in episodes if lo < int(e["timestep"]) <= hi]
        out.append({"from": int(lo), "to": int(hi), "episodes": len(part),
                    "success": (sum(e["outcome"] == "success" for e in part) / len(part))
                    if part else None,
                    "stages": sorted({str(e.get("curriculum_stage")) for e in part})})
    return out


def buffer_composition(episodes: Sequence[dict[str, Any]], at: int, buffer_size: int
                       ) -> dict[str, Any]:
    """The replay buffer at step `at`: the transitions of steps (at - buffer_size, at], split by the
    outcome of the episode each belongs to (shares of the window), and the number of successful
    terminal transitions (an episode's last step is in the window when it ended inside it)."""
    lo = max(0, at - buffer_size)
    window = at - lo
    by: dict[str, int] = {}
    terminals = 0
    for e in episodes:
        end, start = int(e["timestep"]), int(e["timestep"]) - int(e["episode_length"])
        n = max(0, min(end, at) - max(start, lo))
        if n:
            by[e["outcome"]] = by.get(e["outcome"], 0) + n
        if e["outcome"] == "success" and lo < end <= at:
            terminals += 1
    counted = sum(by.values())
    return {"at": int(at), "window": int(window),
            "shares": {k: v / window for k, v in sorted(by.items())} if window else {},
            "unfinished": (window - counted) / window if window else 0.0,
            "success_terminals": terminals}
