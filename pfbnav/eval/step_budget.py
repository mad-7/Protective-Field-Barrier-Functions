"""The environment's per-step wall-clock budget, and what a profiling pass says about it.

Pure functions.

**What binds is the aggregate**, not one instance: `>= 25 steps/s` summed over the running
instances, on the training host, derived in `docs/requirements.md` and measured there by G2.
A per-step budget follows from it and the instance count, and from nothing else:

    per-step budget = n_instances / aggregate_target      (s, p95 per instance)

With four instances and 25 steps/s that is 160 ms. The figure moves whenever the instance
count or the target moves, which is why it is derived here rather than written down. (The
earlier 40 ms came from dividing by the `>= 100 steps/s` target that was withdrawn as an
arithmetic error on 2026-09-20; it survived in `docs/requirements.md` until 2026-09-23.)

**A budget met on the development laptop says nothing about the training host** (WP0/WP1
standing note): the laptop shares four physical cores between the instances, renders unless
told not to, and never runs the learner. What transfers is the *shape* of the step -- which
components dominate, and whether a change moved them -- so `compare_components` reports
shares as well as milliseconds, and `headroom` is always reported with the host it was
measured on.
"""
from __future__ import annotations

from collections.abc import Mapping
from typing import Any


def per_step_budget_s(aggregate_target_steps_per_s: float, n_instances: int) -> float:
    """The p95 wall-clock budget for one instance's step, s.

    `aggregate_target_steps_per_s` is the requirement on the sum over instances.
    """
    if aggregate_target_steps_per_s <= 0.0:
        raise ValueError("the aggregate target must be positive")
    if n_instances < 1:
        raise ValueError("there must be at least one instance")
    return n_instances / float(aggregate_target_steps_per_s)


def headroom(measured_p95_s: float, budget_s: float) -> dict[str, Any]:
    """How a measured p95 stands against its budget: fraction used, margin, verdict."""
    if budget_s <= 0.0:
        raise ValueError("the budget must be positive")
    return {"measured_p95_s": float(measured_p95_s), "budget_s": float(budget_s),
            "fraction_used": float(measured_p95_s) / float(budget_s),
            "margin_s": float(budget_s) - float(measured_p95_s),
            "within": float(measured_p95_s) <= float(budget_s)}


def aggregate_from_components(component_p95_s: Mapping[str, float], n_instances: int,
                              parallel: bool = True) -> float:
    """Steps/s a step of these components would reach, if each instance were this fast.

    `parallel` says the instances step at the same time, which is what the environment
    does; it is the optimistic case, and a measured aggregate is the one to believe.
    """
    total = sum(float(v) for v in component_p95_s.values())
    if total <= 0.0:
        raise ValueError("the components must take some time")
    return (n_instances if parallel else 1) / total


def compare_components(before: Mapping[str, float], after: Mapping[str, float],
                       ) -> list[dict[str, Any]]:
    """Per-component change between two profiles, in ms and as a share of the whole step.

    Sorted by the size of the change, largest first: what moved, not what is biggest. A
    component missing from one side counts as zero there, so an added component (the scan
    packet) shows up as its full cost.
    """
    names = sorted(set(before) | set(after))
    whole_before = sum(float(v) for v in before.values()) or 1.0
    whole_after = sum(float(v) for v in after.values()) or 1.0
    rows = []
    for name in names:
        b, a = float(before.get(name, 0.0)), float(after.get(name, 0.0))
        rows.append({"component": name, "before_ms": b * 1e3, "after_ms": a * 1e3,
                     "change_ms": (a - b) * 1e3,
                     "before_share": b / whole_before, "after_share": a / whole_after})
    return sorted(rows, key=lambda r: abs(r["change_ms"]), reverse=True)
