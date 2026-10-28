"""Gate G4 -- the learned-method pilot (amendment A15, ruled 2026-10-03). Pure functions.

Every threshold is read from `preregistration/gates.yaml` (`G4_learned_pilot`) by the caller and
passed in; nothing here restates one. Each criterion returns its numbers and a verdict, so a report
can show why it passed or failed, not only that it did.

**Controls come first** (ruled): the zero-action policy must FAIL the checkpoint criterion -- all
timeouts, no successes -- and holonomic Pure Pursuit must PASS it. If either misbehaves the gate is
broken and no pilot verdict is read; `g4_verdict` returns "GATE BROKEN" in that case whatever the
pilot did.
"""
from __future__ import annotations

import itertools
import math
from collections import Counter
from collections.abc import Sequence
from typing import Any


def learning_criterion(episodes: Sequence[dict[str, Any]], *, window_fraction: float,
                       min_last_success: float, min_improvement: float) -> dict[str, Any]:
    """Success over the first and last `window_fraction` of training episodes, in episode order."""
    n = len(episodes)
    k = max(1, math.floor(window_fraction * n)) if n else 0
    if n < 2 * k or k == 0:
        return {"episodes": n, "window": k, "first": None, "last": None, "improvement": None,
                "passed": False, "reason": "too few episodes for two disjoint windows"}
    ordered = sorted(episodes, key=lambda e: e["episode"])
    first = sum(e["outcome"] == "success" for e in ordered[:k]) / k
    last = sum(e["outcome"] == "success" for e in ordered[-k:]) / k
    passed = last >= min_last_success and (last - first) >= min_improvement
    return {"episodes": n, "window": k, "first": first, "last": last,
            "improvement": last - first, "passed": bool(passed)}


def checkpoint_criterion(summary: dict[str, Any], *, min_success: float,
                         max_contact_rate: float) -> dict[str, Any]:
    """On an `outcome_summary` of the deterministic policy over the tune set."""
    success, contact = summary["success_rate"], summary["contact_rate"]
    passed = (success is not None and contact is not None
              and success >= min_success and contact <= max_contact_rate)
    return {"success_rate": success, "contact_rate": contact, "episodes": summary["episodes"],
            "passed": bool(passed)}


def integrity_criterion(*, completed: bool, timesteps: int, total_timesteps: int,
                        final_model: bool, environment_sha256: str | None, frozen_sha256: str,
                        losses: Sequence[float], episodes: Sequence[dict[str, Any]],
                        error: str | None) -> dict[str, Any]:
    """The five integrity checks, each named, all required."""
    numbers = [e.get("timestep") for e in episodes]
    checks = {
        "reached_total_timesteps": bool(completed) and timesteps >= total_timesteps
                                   and bool(final_model),
        "frozen_environment": environment_sha256 is not None
                              and environment_sha256 == frozen_sha256,
        "finite_losses": all(math.isfinite(float(v)) for v in losses),
        "episode_log_complete": (
            [e.get("episode") for e in episodes] == list(range(1, len(episodes) + 1))
            and all(e.get("outcome") for e in episodes)
            and all(None not in (a, b) and b > a for a, b in itertools.pairwise(numbers))),
        "no_environment_exception": error is None,
    }
    return {"checks": checks, "passed": all(checks.values())}


def controls_verdict(zero: dict[str, Any], positive: dict[str, Any], *, min_success: float,
                     max_contact_rate: float) -> dict[str, Any]:
    """Zero action must FAIL the checkpoint criterion with all timeouts and no successes; the
    positive control must PASS it. Anything else means the gate is broken. The positive control is
    the privileged oracle tracker since 2026-10-20 (it was Pure Pursuit, B1, whose result is now
    recorded as a baseline finding and not judged)."""
    zero_c = checkpoint_criterion(zero, min_success=min_success, max_contact_rate=max_contact_rate)
    pos_c = checkpoint_criterion(positive, min_success=min_success,
                                 max_contact_rate=max_contact_rate)
    zero_ok = (not zero_c["passed"] and zero["counts"]["success"] == 0
               and zero["counts"]["timeout"] == zero["episodes"] and zero["episodes"] > 0)
    return {"zero_action": {**zero_c, "behaved": bool(zero_ok)},
            "positive_control": {**pos_c, "behaved": bool(pos_c["passed"])},
            "gate_sound": bool(zero_ok and pos_c["passed"])}


def g4_verdict(controls: dict[str, Any], integrity: dict[str, Any], learning: dict[str, Any],
               checkpoint: dict[str, Any]) -> str:
    """GATE BROKEN, INTEGRITY FAIL, PASS, or FAIL (learning or checkpoint, integrity intact)."""
    if not controls["gate_sound"]:
        return "GATE BROKEN"
    if not integrity["passed"]:
        return "INTEGRITY FAIL"
    return "PASS" if learning["passed"] and checkpoint["passed"] else "FAIL"


def recovery_statistics(episodes: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """The classical recovery's per-episode statistics, gathered (ruled 2026-10-16): attempts,
    the outcome of each, deadlocks, recovery distance and time. Episodes without recovery fields
    (a learned policy's) count as none. Per episode and in total."""
    rows, endings, deadlocks = [], Counter(), Counter()
    for e in episodes:
        log = list(e.get("recovery_attempt_log") or [])
        endings.update(a["ending"] for a in log if a.get("ending"))
        if e.get("recovery_deadlock"):
            deadlocks[e["recovery_deadlock"]] += 1
        rows.append({"scenario_seed": e.get("scenario_seed"), "stratum": e.get("stratum"),
                     "outcome": e.get("outcome"),
                     "attempts": int(e.get("recovery_attempts", 0)),
                     "endings": [a.get("ending") for a in log],
                     "attempt_times_s": [float(a.get("time_s", 0.0)) for a in log],
                     "deadlock": e.get("recovery_deadlock"),
                     "distance_m": float(e.get("recovery_distance_m", 0.0)),
                     "time_s": float(e.get("recovery_time_s", 0.0))})
    return {"episodes": len(rows),
            "episodes_with_recovery": sum(1 for r in rows if r["attempts"] > 0),
            "attempts": sum(r["attempts"] for r in rows),
            "endings": dict(sorted(endings.items())),
            "deadlocks": dict(sorted(deadlocks.items())),
            "distance_m": sum(r["distance_m"] for r in rows),
            "time_s": sum(r["time_s"] for r in rows),
            # Ruled 2026-10-19: attempt times are reported (an attempt to the full distance takes
            # ~17 s at 0.018 m/s, so the five-attempt cap is effectively lower in a 75 s episode).
            # An attempt with no ending was cut short by the episode's end.
            "attempt_times_s": [t for r in rows for t in r["attempt_times_s"]],
            "attempts_cut_short": sum(1 for e in episodes
                                      for a in e.get("recovery_attempt_log") or []
                                      if a.get("ending") is None),
            "suspected_defects": recovery_defects(episodes),
            "per_episode": rows}


def recovery_defects(episodes: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    """Every recovery attempt that ended `not_released` (ruled 2026-10-16: stop and report it as a
    suspected recovery defect). An attempt begins only once recovery has chosen a direction it
    judged permitted -- no permitted direction is its own ending, `no_permitted_direction` -- so
    each `not_released` is an attempt the emulator refused although a permitted direction
    existed in recovery's judgement: the two disagree. Reported in G4's controls and in every
    evaluation through WP3."""
    out = []
    for e in episodes:
        for a in e.get("recovery_attempt_log") or []:
            if a.get("ending") == "not_released":
                out.append({"scenario_seed": e.get("scenario_seed"), "index": e.get("index"),
                            "set": e.get("set"), "start_step": a.get("start_step"),
                            "heading_deg": a.get("heading_deg")})
    return out

