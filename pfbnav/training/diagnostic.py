"""Training diagnostics that are never methods (ruled 2026-10-30, A30 ruling 6).

A diagnostic run is labelled in its config (`diagnostic:`), trains with `scripts/train.py` like any
learned arm, and is never cited: it has no method ID (`configs/methods.yaml`), its seed is from the
pilot range but never a G4 pilot training seed (so `scripts/gate_g4.py` refuses it), and its
experiment name starts with `DIAG_`.

**Unicycle mode** (`diagnostic.unicycle: true`): the lateral command is zeroed before the
environment sees it (vy = 0), so the holonomic base drives as a unicycle. The policy keeps its three
outputs; the second is ignored. Everything else -- the observation, the reward, the certified layer,
the curriculum and the stall truncation -- is the arm's own.
"""
from __future__ import annotations

from typing import Any

import gymnasium as gym
import numpy as np

PREFIX = "DIAG_"


def diagnostic_problem(cfg: dict[str, Any], seed: int, gates: dict[str, Any]) -> str | None:
    """Why this config and seed may not train, as far as diagnostics go; None if they may. A config
    with no `diagnostic:` block may not use a diagnostic option."""
    diag = cfg.get("diagnostic")
    if not diag:
        return None
    if not str(cfg.get("experiment", "")).startswith(PREFIX):
        return f"a diagnostic's experiment name starts with {PREFIX}"
    if not diag.get("label"):
        return "a diagnostic carries a label (diagnostic.label)"
    lo, hi = gates["layout_sets"]["seed_ranges"]["pilot"]
    if not lo <= seed <= hi:
        return f"a diagnostic's seed is in the pilot range [{lo}, {hi}]"
    if seed in gates["G4_learned_pilot"]["pilot_training_seeds"]:
        return f"seed {seed} is a G4 pilot training seed; a diagnostic never uses one"
    return None


class UnicycleAction(gym.ActionWrapper):
    """Zeroes the lateral axis (index 1, a_vy) of every action."""

    def action(self, action):
        a = np.array(action, dtype=np.float32, copy=True).reshape(-1)
        if a.size != 3:
            raise ValueError(f"an action is (a_vx, a_vy, a_wz), got {a.size} values")
        a[1] = 0.0
        return a
