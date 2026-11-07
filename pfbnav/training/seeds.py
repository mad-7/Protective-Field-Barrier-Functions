"""Which training seeds a run may use (A35, ruled 2026-11-07). Pure.

Once `G4_learned_pilot.confirmatory_training_seeds` is preregistered (A35), a training run's seed must
be either one of the K = 10 confirmatory seeds (9100001-9100010) or in the pilot TRAINING range
(9000000-9000099); a DIAGNOSTIC never uses a confirmatory seed nor any seed of the carved-out block
9100000-9100999. Before A35 (no such key) nothing here applies, so earlier runs read as they did.
"""
from __future__ import annotations

from typing import Any

CONFIRMATORY_BLOCK = (9100000, 9100999)


def training_seed_problem(seed: int, gates: dict[str, Any], *, diagnostic: bool) -> str | None:
    """None if `seed` may train (a diagnostic when `diagnostic`), else why not."""
    g4 = gates["G4_learned_pilot"]
    confirmatory = g4.get("confirmatory_training_seeds")
    if confirmatory is None:
        return None
    lo, hi = CONFIRMATORY_BLOCK
    if diagnostic:
        if lo <= seed <= hi:
            return f"seed {seed} is in the confirmatory block {lo}-{hi}: a diagnostic never uses it"
        return None
    if seed in [int(s) for s in confirmatory]:
        return None
    plo, phi = (int(v) for v in g4["pilot_training_range"])
    if plo <= seed <= phi:
        return None
    return (f"seed {seed} is neither a confirmatory seed {confirmatory[0]}-{confirmatory[-1]} nor in "
            f"the pilot training range {plo}-{phi} (A35)")
