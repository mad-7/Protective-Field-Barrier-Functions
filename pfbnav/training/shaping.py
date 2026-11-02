"""The proximity-shaping weight of the shaped learned arms (L1, L4), derived (ruled 2026-11-01). Pure.

`k_proximity` = 2.0 was underived and made the shaped arms degenerate: in L1's first pilot the
proximity penalty outweighed the progress reward in 67 of 75 episodes (offline diagnosis,
2026-11-01). The supervisor ruled it a comparator-specification defect and fixed the rule:

    on the centreline of the narrowest aisle stratum at 0.3 m/s, the proximity penalty per step
    equals 25 % of the progress reward per step.

25 % is a DECLARED DESIGN CHOICE, not a measurement (`reward.shaping.proximity_share`).

**The two sides, from the reward's own definitions** (`pfbnav/envs/reward.py`, `NavEnv._proximity`):
  * progress per step at speed v along the route: k_progress x v x dt;
  * proximity per step: k_proximity x (threshold - r) / threshold, with r the nearest valid return
    from the BODY ORIGIN (the filter stream's ranges). Centred in an aisle of per-side clearance c
    (the strata's definition: aisle width = 2 (half_width + c)), the nearest return is the wall
    abeam, r = half_width + c. The narrowest aisle is the narrow stratum's floor, `min_clearance`.
So k_proximity = share x k_progress x v x dt x threshold / (threshold - half_width - c).

The real scan's nearest return there sits within ~1 mm of the geometric value (the 2026-09-27 scan
measurement); the derivation is the ruled definition, and the value carries its inputs.
"""
from __future__ import annotations

from typing import Any


def derive_k_proximity(*, k_progress: float, speed: float, dt: float, share: float,
                       half_width: float, clearance: float, threshold: float) -> float:
    """k_proximity such that, centred in an aisle of per-side `clearance` at `speed`, the proximity
    penalty per step is `share` of the progress reward per step. SI units."""
    if min(k_progress, speed, dt, share, half_width, threshold) <= 0.0 or clearance < 0.0:
        raise ValueError("every input is positive (clearance non-negative)")
    nearest = half_width + clearance
    if nearest >= threshold:
        raise ValueError(f"the wall abeam ({nearest:.4f} m) is beyond the proximity threshold "
                         f"({threshold} m): no penalty to scale")
    cost = (threshold - nearest) / threshold
    return share * k_progress * speed * dt / cost


def k_proximity_from_config(cfg: dict[str, Any]) -> float:
    """The ruled value, every input read from the configuration."""
    sh = cfg["reward"]["shaping"]
    strata = {s["name"]: s for s in cfg["world"]["aisle_strata"]}
    return derive_k_proximity(
        k_progress=float(cfg["reward"]["k_progress"]), speed=float(sh["reference_speed"]),
        dt=float(cfg["sim"]["dt"]), share=float(sh["proximity_share"]),
        half_width=float(cfg["robot"]["half_width"]),
        clearance=float(strata[sh["reference_stratum"]]["min_clearance"]),
        threshold=float(cfg["env"]["observation"]["proximity_threshold"]))


def shaping_problem(cfg: dict[str, Any], rel_tol: float = 1e-9) -> str | None:
    """Why this config's k_proximity may not train, or None: a shaped config (k_proximity > 0) must
    carry exactly the derived value."""
    k = float(cfg["reward"]["k_proximity"])
    if k == 0.0:
        return None
    derived = k_proximity_from_config(cfg)
    if abs(k - derived) > rel_tol * derived:
        return (f"reward.k_proximity {k} is not the derived value {derived!r} "
                "(pfbnav/training/shaping.py, ruled 2026-11-01)")
    return None
