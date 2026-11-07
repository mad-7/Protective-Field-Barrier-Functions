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


PILOT_RANGE = (9000000, 9999999)   # layout_sets.seed_ranges.pilot (gates.yaml)


def reported_training_seed_problem(seed: int, gates: dict[str, Any]) -> str | None:
    """None if a TRAINING seed may appear in a reported result, else why not (A35 carve-out,
    confirmed 2026-11-07). Inside the frozen pilot range [9000000, 9999999] only two ranges are
    reportable: the pilot TRAINING range (G4 pilots, disclosed; never in H1/H2, rule 17) and the
    confirmatory block (9100000-9100999, holding the ten declared seeds). Seeds outside the pilot
    range are not training seeds of this study and are refused too. Layout seeds are another
    namespace (the narrow-rule pilot SET is built from layout seeds 9000000-9000149)."""
    g4 = gates["G4_learned_pilot"]
    plo, phi = (int(v) for v in g4["pilot_training_range"])
    clo, chi = CONFIRMATORY_BLOCK
    if plo <= seed <= phi or clo <= seed <= chi:
        return None
    lo, hi = PILOT_RANGE
    where = ("inside the pilot range but outside both declared ranges" if lo <= seed <= hi
             else "outside the pilot range")
    return f"training seed {seed} is {where}; it may not appear in a reported result (A35)"


def confirmatory_seed_problem(seed: int, gates: dict[str, Any]) -> str | None:
    """None if `seed` may enter H1/H2 (one of the declared confirmatory seeds), else why not
    (rule 17: pilot seeds never enter H1 or H2)."""
    seeds = [int(s) for s in gates["G4_learned_pilot"]["confirmatory_training_seeds"]]
    return None if seed in seeds else (
        f"seed {seed} is not one of the confirmatory seeds {seeds[0]}-{seeds[-1]}: it never enters "
        "H1 or H2")
