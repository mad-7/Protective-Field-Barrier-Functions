"""Tuning a classical baseline on the tune set: candidates, their scores, and the selection. Pure.

Used by `scripts/tune_classical.py` (B2 first; ruled 2026-10-24: B2 is tuned under the
equal-tuning-budget rule, with the route cap and the classical recovery, on the tune set only).

**The budget** (ruled 2026-10-26): N = 24 candidates per classical baseline, candidate 0 included,
the same N for B1, B2, B3 and B6; B7 reuses B2's tuned parameters. It lives in
`configs/tuning/budget.yaml`, read by the runner, never passed on a command line.

  * **Candidates** (`sample_candidates`): candidate 0 is the configured initial values; the others
    are drawn by a SEEDED random search over a declared space (`configs/tuning/<baseline>.yaml`),
    each parameter uniform on its range or log-uniform, re-drawn while the controller's own
    validation refuses the combination (bounded, and the refusals counted).
  * **Scores** (`candidate_score`): the success rate, protective stops per 100 m pooled over the set
    (scan-fault stops excluded, as everywhere: `runner.outcome_summary`), and the mission time -- the
    mean duration of the successful episodes, s.
  * **Selection** (`select`, ruled 2026-10-26 for the baselines, as for alpha on 2026-10-24) --
    among the candidates whose success rate is within `success_window` (0.05, 5 percentage points)
    of the best, the fewest protective stops per 100 m; ties broken by the shorter mission time,
    then by the lower candidate index.
  * **Boundaries** (`on_boundary`, ruled 2026-10-26): a winner on a range boundary is REPORTED,
    never extended -- a parameter within `BOUNDARY_FRACTION` (1 %) of its range's width (in log for
    a log range) of either end.
"""
from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from pfbnav.eval.runner import outcome_summary

SUCCESS_WINDOW = 0.05          # 5 percentage points (the alpha rule, ruled 2026-10-24)
MAX_REDRAWS = 1000             # per candidate, before the space is declared unusable
BOUNDARY_FRACTION = 0.01       # of a range's width: "on the boundary" (reported, never extended)


@dataclass(frozen=True)
class ParamRange:
    low: float
    high: float
    log: bool = False
    integer: bool = False      # drawn as declared, then rounded to the nearest integer in range

    def __post_init__(self) -> None:
        if not self.low < self.high:
            raise ValueError("need low < high")
        if self.log and self.low <= 0.0:
            raise ValueError("a log-uniform range must be positive")
        if self.integer and not (float(self.low).is_integer() and float(self.high).is_integer()):
            raise ValueError("an integer range needs integer ends")

    def draw(self, rng: np.random.Generator) -> float:
        if self.log:
            x = float(math.exp(rng.uniform(math.log(self.low), math.log(self.high))))
        else:
            x = float(rng.uniform(self.low, self.high))
        if self.integer:
            x = float(min(max(round(x), self.low), self.high))
        return x


def on_boundary(params: dict[str, float], space: dict[str, ParamRange],
                fraction: float = BOUNDARY_FRACTION) -> dict[str, str]:
    """{name: "low" | "high"} for every tuned parameter within `fraction` of its range's width of
    an end (measured in log for a log range)."""
    out = {}
    for name, r in space.items():
        f = math.log if r.log else (lambda x: x)
        lo, hi, x = f(r.low), f(r.high), f(float(params[name]))
        tol = fraction * (hi - lo)
        if x <= lo + tol:
            out[name] = "low"
        elif x >= hi - tol:
            out[name] = "high"
    return out


def parse_space(spec: dict[str, Any]) -> dict[str, ParamRange]:
    """{name: {low, high, scale: linear | log, type: float | int}} -> {name: ParamRange}. A name
    may be dotted to reach a nested leaf (`weights.route_tracking`, `noise_std.0`; see
    `flatten_numeric`)."""
    out = {}
    for name, r in spec.items():
        scale = r.get("scale", "linear")
        if scale not in ("linear", "log"):
            raise ValueError(f"{name}: scale must be linear or log, got {scale!r}")
        kind = r.get("type", "float")
        if kind not in ("float", "int"):
            raise ValueError(f"{name}: type must be float or int, got {kind!r}")
        out[name] = ParamRange(float(r["low"]), float(r["high"]), scale == "log", kind == "int")
    return out


def flatten_numeric(block: Any, prefix: str = "") -> dict[str, float]:
    """The numeric leaves of a config block as {dotted name: float}: a nested mapping's keys and a
    list's indices are joined by dots (`weights.route_tracking`, `noise_std.0`). Strings, booleans
    and None are not tunable and are left out. A flat block of numbers maps to itself."""
    out: dict[str, float] = {}
    items = block.items() if isinstance(block, dict) else enumerate(block)
    for key, value in items:
        name = f"{prefix}{key}"
        if isinstance(value, (bool, str)) or value is None:
            continue
        if isinstance(value, (dict, list, tuple)):
            out.update(flatten_numeric(value, f"{name}."))
        else:
            out[name] = float(value)
    return out


def _child(node: Any, key: str, name: str) -> Any:
    if isinstance(node, dict) and key in node:
        return node[key]
    if isinstance(node, list) and key.isdigit() and int(key) < len(node):
        return node[int(key)]
    raise ValueError(f"no parameter {name!r} in the block")


def assign_flat(block: dict[str, Any], values: dict[str, float]) -> None:
    """Write {dotted name: value} into `block` IN PLACE, leaf by leaf. Every name must reach an
    existing numeric leaf; an integer leaf stays an integer and refuses a non-integral value."""
    for name, value in values.items():
        parts = name.split(".")
        node: Any = block
        for part in parts[:-1]:
            node = _child(node, part, name)
        old = _child(node, parts[-1], name)
        if isinstance(old, bool) or not isinstance(old, (int, float)):
            raise TypeError(f"{name!r} is not a numeric parameter")
        if isinstance(old, int):
            if not float(value).is_integer():
                raise ValueError(f"{name!r} is an integer, got {value!r}")
            new: float = int(value)
        else:
            new = float(value)
        if isinstance(node, list):
            node[int(parts[-1])] = new
        else:
            node[parts[-1]] = new


def sample_candidates(space: dict[str, ParamRange], defaults: dict[str, float], n: int,
                      rng: np.random.Generator, valid: Callable[[dict[str, float]], bool]
                      ) -> tuple[list[dict[str, float]], int]:
    """`n` candidates: the defaults first, then seeded random draws over `space` (parameters not in
    the space keep their default). Returns (candidates, refusals redrawn)."""
    if n < 1:
        raise ValueError("the budget is at least one candidate")
    if not valid(dict(defaults)):
        raise ValueError("the configured initial values are themselves invalid")
    unknown = set(space) - set(defaults)
    if unknown:
        raise ValueError(f"tuned parameters the controller does not have: {sorted(unknown)}")
    out, refused = [dict(defaults)], 0
    while len(out) < n:
        for _ in range(MAX_REDRAWS):
            cand = dict(defaults)
            cand.update({k: r.draw(rng) for k, r in space.items()})
            if valid(cand):
                out.append(cand)
                break
            refused += 1
        else:
            raise ValueError(f"no valid candidate in {MAX_REDRAWS} draws: the space is unusable")
    return out, refused


def candidate_score(records: Sequence[dict[str, Any]], dt: float) -> dict[str, Any]:
    """Success rate, pooled protective stops per 100 m (scan-fault stops excluded) and mission time
    (mean duration of the successful episodes, s; None without a success)."""
    s = outcome_summary(records)
    times = [float(r["episode_steps"]) * dt for r in records if r["outcome"] == "success"]
    return {"episodes": s["episodes"], "success_rate": s["success_rate"],
            "stops_per_100m": s["stops_per_100m"],
            "mission_time_s": float(np.mean(times)) if times else None,
            "contact_rate": s["contact_rate"], "counts": s["counts"]}


def select(scores: Sequence[dict[str, Any]], success_window: float = SUCCESS_WINDOW) -> int:
    """The index chosen by the rule (module docstring). A candidate with no distance driven (stop
    rate None) is ranked after every candidate with a rate."""
    if not scores:
        raise ValueError("no candidates to select from")
    best = max(float(s["success_rate"]) for s in scores)
    eligible = [i for i, s in enumerate(scores)
                if float(s["success_rate"]) >= best - success_window - 1e-12]

    def key(i: int) -> tuple[float, float, int]:
        s = scores[i]
        rate = s["stops_per_100m"]
        time = s["mission_time_s"]
        return (math.inf if rate is None else float(rate),
                math.inf if time is None else float(time), i)

    return min(eligible, key=key)


def monitoring_row(records: Sequence[dict[str, Any]], dt: float) -> dict[str, Any]:
    """One arm's row of the B2-B7 tune-set comparison (ruled 2026-10-26): `candidate_score`, plus
    the PFB's infeasible steps and fallback steps summed over the episodes (0 for an arm without the
    filter)."""
    row = candidate_score(records, dt)
    row["pfb_infeasible"] = int(sum(int(r.get("pfb_infeasible", 0)) for r in records))
    row["pfb_fallback_steps"] = int(sum(int((r.get("pfb_modes") or {}).get("fallback", 0))
                                        for r in records))
    return row


def stratum_comparison(arms: dict[str, Sequence[dict[str, Any]]], dt: float) -> dict[str, Any]:
    """{stratum (and "all"): {arm: monitoring_row}}. TUNE-SET EVIDENCE FOR MONITORING AND FRAMING
    ONLY (ruled 2026-10-26): never cited as a confirmatory result; it alters no preregistered
    analysis. Every arm must cover the same scenarios (by set and index)."""
    keys = {name: sorted((r["set"], r["index"]) for r in recs) for name, recs in arms.items()}
    if len({tuple(k) for k in keys.values()}) != 1:
        raise ValueError("the arms do not cover the same scenarios")
    strata = sorted({r["stratum"] for recs in arms.values() for r in recs})
    out: dict[str, Any] = {"all": {n: monitoring_row(recs, dt) for n, recs in arms.items()}}
    for s in strata:
        out[s] = {n: monitoring_row([r for r in recs if r["stratum"] == s], dt)
                  for n, recs in arms.items()}
    return out
