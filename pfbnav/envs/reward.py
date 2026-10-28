"""The reward, term by term. Pure functions; no simulator.

Every weight comes from `configs/base.yaml` under `reward:`; nothing is written down twice.
Changing a term needs a decision-log entry in the same change (hard rule 7).

    term            sign  what it pays for
    ---------------------------------------------------------------------------------
    progress          +   arc length gained ALONG THE PLAN (the observation ruling of
                          2026-09-23 defines progress on the polyline projection, so a
                          robot rounding a shelf is paid for following the plan rather
                          than penalised for moving away from the goal)
    time              -   one step of wall time, so dawdling is not free
    stop              -   each protective stop TRIGGERED, which is the primary metric
    intervention      -   how much the PFB changed the requested command (0 without the filter)
    proximity         -   closeness to an obstacle (0 by default; the shaped variants only)
    goal bonus        +   terminal, on arrival
    contact penalty   -   terminal, on a collision (contact checks only -- hard rule 4)
    ---------------------------------------------------------------------------------

**A stop is charged once, on the trigger**, matching how the emulator counts it: charging
per stopped step would pay a policy to break a latched stop, which it cannot do, and would
make the penalty depend on the restart delay rather than on the stop.

**Intervention is the PFB's alone** (ruled 2026-10-24): ||(u_request - u_PFB) / ceilings||^2, and 0
for a method without the filter. Certified overrides, the wheel-budget projection and the limiter
are NOT in it, so it never acts as a hidden stop penalty (which would break L2's constraint-only
design and double-count stops for L1). Before the ruling it compared the request with the APPLIED
command, as a norm, for every method. Measured on the axes' own ceilings, as the action is: a lateral
difference of 0.1 m/s is not the same fraction of the envelope as a forward one.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import ArrayLike


@dataclass(frozen=True)
class RewardWeights:
    """The `reward:` block, read once."""

    k_progress: float
    time_cost: float
    k_stop: float
    k_intervention: float
    k_proximity: float
    goal_bonus: float
    contact_penalty: float
    goal_tolerance: float
    max_steps: int

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> RewardWeights:
        r = cfg["reward"]
        return cls(k_progress=float(r["k_progress"]), time_cost=float(r["time_cost"]),
                   k_stop=float(r["k_stop"]), k_intervention=float(r["k_intervention"]),
                   k_proximity=float(r["k_proximity"]), goal_bonus=float(r["goal_bonus"]),
                   contact_penalty=float(r["contact_penalty"]),
                   goal_tolerance=float(r["goal_tolerance"]), max_steps=int(r["max_steps"]))


def intervention_cost(requested: ArrayLike, filtered: ArrayLike | None,
                      ceilings: ArrayLike) -> float:
    """The PFB's intervention, ||(requested - filtered) / ceilings||^2 (ruled 2026-10-24).

    `filtered` is the PFB's output -- before the certified layer, the wheel-budget projection and
    the limiter -- or None for a method without the filter, whose intervention is exactly 0.
    0 when the PFB passes the request; 1 when it changes one axis by a full ceiling.
    """
    c = np.asarray(ceilings, dtype=float).ravel()
    if np.any(c <= 0.0):
        raise ValueError("every ceiling must be positive")
    if filtered is None:
        return 0.0
    q = np.asarray(requested, dtype=float).ravel()
    f = np.asarray(filtered, dtype=float).ravel()
    if not (q.size == f.size == c.size):
        raise ValueError("requested, filtered and ceilings must have the same length")
    return float(np.sum(((q - f) / c) ** 2))


def pfb_intervention(requested: ArrayLike, pfb_report: dict[str, Any] | None,
                     ceilings: ArrayLike) -> float:
    """The step's intervention term from the step loop's PFB report (`StepState.pfb`).

    0 without the filter (no report), and 0 on a step decided under the certified layer's latch
    (mode "latched"; ruled 2026-10-26): the latch is the certified layer's, and charging there would
    give the filtered methods a hidden stop signal that L1 and L2 lack. Otherwise
    `intervention_cost(requested, report["u"], ceilings)`.
    """
    if pfb_report is None or pfb_report.get("mode") == "latched":
        intervention_cost(requested, None, ceilings)          # validates the ceilings all the same
        return 0.0
    return intervention_cost(requested, pfb_report["u"], ceilings)


def proximity_cost(clearance: float, threshold: float) -> float:
    """1 at zero clearance, falling linearly to 0 at `threshold`; 0 beyond it."""
    if threshold <= 0.0:
        raise ValueError("the proximity threshold must be positive")
    return float(np.clip((threshold - float(clearance)) / threshold, 0.0, 1.0))


def step_reward(weights: RewardWeights, *, progress_m: float, stop_triggered: bool,
                intervention: float, proximity: float = 0.0, reached_goal: bool = False,
                in_contact: bool = False) -> tuple[float, dict[str, float]]:
    """One step's reward and its breakdown, so a run can report where the reward came from.

    The breakdown sums to the reward exactly; a term that is off carries 0, not a missing
    key, so a log has the same columns on every step.
    """
    terms = {
        "progress": weights.k_progress * float(progress_m),
        "time": -weights.time_cost,
        "stop": -weights.k_stop * (1.0 if stop_triggered else 0.0),
        "intervention": -weights.k_intervention * float(intervention),
        "proximity": -weights.k_proximity * float(proximity),
        "goal": weights.goal_bonus if reached_goal else 0.0,
        "contact": -weights.contact_penalty if in_contact else 0.0,
    }
    return float(sum(terms.values())), terms


def episode_outcome(*, reached_goal: bool, in_contact: bool, steps: int,
                    weights: RewardWeights, stop_active: bool = False,
                    restart_blocked: bool = False, breached: bool = False,
                    at_rest: bool = False) -> tuple[bool, bool, str]:
    """(terminated, truncated, outcome). One name per way an episode can end.

    Reaching the goal and touching something on the same step counts as a **collision**: a
    run that ends in contact is not a success, whatever else happened in that step.

    An episode that runs out of steps **while a protective stop is latched** is an
    `unrecoverable_stop`, not a `timeout` (supervisor 2026-09-27, ruling 4). The two are
    different failures and pooling them would hide the one the safety layers cause: a timeout
    is a controller that was too slow, an unrecoverable stop is a robot that could not legally
    move again. It is counted per stratum and reported if it occurs. The emulator's only release
    from inside the at-rest field is the receding release (A24, 2026-10-21): a creep away from
    every intrusion, after the restart delay; any other recovery belongs to the controller.

    **Stuck has two forms** (ruled 2026-10-16): the latched stop's restart is blocked, OR the robot
    is at rest with its own field breached -- an intrusion inside the minimum margin, which no
    direction can release. The emulator computes `restart_blocked` only while that field is clear,
    so the second form was labelled `timeout` until then (two of the 2026-10-13 traces). `at_rest`
    keeps a stop that has only just fired, with the robot still moving, from counting as stuck.
    """
    if in_contact:
        return True, False, "collision"
    if reached_goal:
        return True, False, "success"
    if steps >= weights.max_steps:
        stuck = bool(stop_active) and (bool(restart_blocked)
                                       or (bool(breached) and bool(at_rest)))
        return False, True, "unrecoverable_stop" if stuck else "timeout"
    return False, False, "running"
