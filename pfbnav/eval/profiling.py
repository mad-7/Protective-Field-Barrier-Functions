"""Summaries of per-step timing samples, for the profiling pass. Pure functions.

A profiling pass is a measurement, not a gate: it reports costs and flags the expensive
components, and never issues PASS or FAIL (WP0 closure).
"""
from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np

# A component costing more than this per step, on average, is flagged (WP0 closure).
FLAG_MS = 2.0


def summarise_timings(samples_ms: Sequence[float], *, flag_ms: float = FLAG_MS) -> dict[str, Any]:
    """Mean, median, 95th percentile and maximum of per-step costs in milliseconds.

    `flagged` is set when the mean exceeds `flag_ms`; `p95_over` separately records a tail
    above it, since a component that is cheap on average but slow one step in twenty still
    matters for a real-time step budget.
    """
    x = np.asarray(samples_ms, dtype=float)
    if x.ndim != 1 or len(x) == 0:
        raise ValueError("need a non-empty 1-D sequence of samples")
    if np.any(x < 0.0) or not np.all(np.isfinite(x)):
        raise ValueError("timing samples must be finite and non-negative")
    p95 = float(np.percentile(x, 95))
    mean = float(x.mean())
    return {"n": len(x), "mean_ms": mean, "p50_ms": float(np.median(x)),
            "p95_ms": p95, "max_ms": float(x.max()),
            "flagged": mean > flag_ms, "p95_over": p95 > flag_ms}


# A single non-physics sample above this is listed as a spike, by step index (WP1 report,
# ruling 1: diagnose the contact-check spike with per-step samples).
SPIKE_MS = 50.0


def spikes(samples_ms: Sequence[float], *, threshold_ms: float = SPIKE_MS) -> list[int]:
    """Indices of the samples above `threshold_ms`, in order."""
    return [i for i, x in enumerate(samples_ms) if x > threshold_ms]


# Decision rule for the fixed-cost diagnosis, set in advance by the supervisor (2026-09-19):
# a per-step floor below about this -> proceed to the batched step and keep the 100 steps/s
# target; otherwise stop optimising and re-plan the training matrix at WP1 close.
STEP_FLOOR_THRESHOLD_MS = 15.0


def floor_decision(step_mean_ms: dict[str, float], representative: Sequence[str], *,
                   threshold_ms: float = STEP_FLOOR_THRESHOLD_MS) -> dict[str, Any]:
    """Apply the step-floor decision rule to measured mean ms per `sim.step()`.

    The floor is the cheapest of the `representative` configurations: those that keep the
    warehouse scene, the robot and the frozen physics, and differ only in simulator
    settings the environment could adopt. Stripped scenes (youBot alone, empty) and
    physics switched off locate the cost but are not configurations training could run,
    so they never set the floor.
    """
    if not representative:
        raise ValueError("no representative configuration")
    missing = [c for c in representative if c not in step_mean_ms]
    if missing:
        raise ValueError(f"no measurement for {missing}")
    floor_config = min(representative, key=lambda c: step_mean_ms[c])
    floor = float(step_mean_ms[floor_config])
    return {"floor_ms": floor, "floor_config": floor_config, "threshold_ms": threshold_ms,
            "decision": "batched_step" if floor < threshold_ms else "replan_matrix"}
