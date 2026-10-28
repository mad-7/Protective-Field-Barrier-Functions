"""Re-apply the episode-outcome rule to recorded episodes (ruled 2026-10-16).

The 2026-10-16 labelling fix makes a robot at rest with its own field breached `unrecoverable_stop`,
as well as one whose restart is blocked. The supervisor asked to show that relabelling the 16
episodes traced on 2026-10-13 changes exactly the two labels found. This module does the
relabelling from each episode's final record, under the rule as it is now, and checks the recorded
label against the rule as it was, so a record the old rule does not reproduce is reported rather
than silently relabelled.
"""
from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from pfbnav.envs.reward import RewardWeights, episode_outcome


def _old_rule(*, reached: bool, in_contact: bool, at_limit: bool, stop_active: bool,
              restart_blocked: bool) -> str:
    """The rule before 2026-10-16, restated here so the check does not depend on the code it
    is checking: stuck meant a latched stop with the restart blocked, and nothing else."""
    if in_contact:
        return "collision"
    if reached:
        return "success"
    if at_limit:
        return "unrecoverable_stop" if (stop_active and restart_blocked) else "timeout"
    return "running"


def relabel(finals: Sequence[dict[str, Any]], weights: RewardWeights) -> list[dict[str, Any]]:
    """One row per episode, from its final record: {'name', 'recorded', 'old_rule', 'new_rule',
    'reproduced', 'changed'}.

    Each final record carries 'name', 'outcome' (the recorded label), 'step' (steps taken),
    'in_contact', 'reached' (optional, default False) and 'certified' with 'stop_active',
    'restart_blocked', 'breached' and 'at_rest'. `reproduced` is whether the old rule gives the
    recorded label; `changed` whether the current rule gives a different one.
    """
    rows = []
    for f in finals:
        c = f["certified"]
        steps = int(f["step"])
        reached = bool(f.get("reached", False))
        in_contact = bool(f.get("in_contact", False))
        old = _old_rule(reached=reached, in_contact=in_contact,
                        at_limit=steps >= weights.max_steps,
                        stop_active=bool(c["stop_active"]),
                        restart_blocked=bool(c["restart_blocked"]))
        new = episode_outcome(reached_goal=reached, in_contact=in_contact, steps=steps,
                              weights=weights, stop_active=bool(c["stop_active"]),
                              restart_blocked=bool(c["restart_blocked"]),
                              breached=bool(c["breached"]), at_rest=bool(c["at_rest"]))[2]
        rows.append({"name": f["name"], "recorded": f["outcome"], "old_rule": old,
                     "new_rule": new, "reproduced": old == f["outcome"],
                     "changed": new != f["outcome"]})
    return rows
