"""Which seeds each scenario set draws from, and how engaged its unmapped obstacles are. Pure.

`scripts/build_scenario_sets.py` builds and writes the sets; the decisions it depends on live
here so they can be tested without generating anything.

**Seed blocks.** Every set draws from a contiguous block of its split's preregistered range, and
no two sets share a seed. Blocks are laid out in a fixed order from the start of each range, so
the same plan always gives the same seeds, and a set that fails a scenario takes the next seed in
its OWN block -- it can never reach into another set's.

**Engagement** (found 2026-09-29). Unmapped obstacles are placed uniformly over the hall's free
floor before any route exists, so most never come near the route: on pilot seeds only 8-15 % of
`unmapped_static` scenarios put any obstacle within 0.3 m of the plan. `engagement` reports that
share for every set built, so a set can never be described as testing unmapped obstacles without
saying how often the robot actually meets one.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

# The offset from the route within which an unmapped obstacle reaches the robot's protective field
# as it follows its plan: half_width 0.2086 plus the field's 0.05 abeam margin is ~0.26 m.
ENGAGED_OFFSET_M = 0.3


@dataclass(frozen=True)
class SetRequest:
    """One set to build: where it draws seeds from, what it must contain, and how many."""

    name: str
    split: str
    stratum: str | None
    episode_rate: float
    size: int
    lock: bool = False

    def __post_init__(self) -> None:
        if self.size <= 0:
            raise ValueError(f"{self.name}: a set needs a positive size, got {self.size}")
        if not 0.0 <= self.episode_rate <= 1.0:
            raise ValueError(f"{self.name}: episode rate {self.episode_rate} outside [0, 1]")


def allocate_blocks(ranges: dict[str, tuple[int, int]], requests: list[SetRequest],
                    headroom: int) -> dict[str, tuple[int, int]]:
    """A disjoint, inclusive seed block per request, `headroom` x its size, in request order.

    Raises if a split's range cannot hold its sets, or if two requests share a name: an ambiguous
    or overflowing plan is refused rather than silently shortened.
    """
    if headroom < 1:
        raise ValueError("headroom must be at least 1")
    names = [r.name for r in requests]
    if len(names) != len(set(names)):
        raise ValueError(f"duplicate set names in {names}")
    cursor = {split: int(lo) for split, (lo, _hi) in ranges.items()}
    blocks: dict[str, tuple[int, int]] = {}
    for r in requests:
        if r.split not in ranges:
            raise ValueError(f"{r.name}: no preregistered seed range for split {r.split!r}")
        lo = cursor[r.split]
        hi = lo + headroom * r.size - 1
        if hi > int(ranges[r.split][1]):
            raise ValueError(f"{r.name}: split {r.split!r} has no room for {headroom * r.size} "
                             f"more seeds after {lo - 1}")
        blocks[r.name] = (lo, hi)
        cursor[r.split] = hi + 1
    return blocks


def engagement(nearest_offsets_m: list[float | None],
               threshold_m: float = ENGAGED_OFFSET_M) -> dict[str, Any]:
    """How many scenarios put an unmapped obstacle where the robot will meet it.

    `nearest_offsets_m` holds, per scenario, the offset of its NEAREST unmapped obstacle from the
    plan, or None when it has none. Returns counts and the engaged share among scenarios that have
    any obstacle -- None when none do, because 0/0 is not "never engaged".
    """
    with_obstacle = [o for o in nearest_offsets_m if o is not None]
    engaged = sum(1 for o in with_obstacle if o <= threshold_m)
    return {"scenarios": len(nearest_offsets_m), "with_unmapped": len(with_obstacle),
            "engaged": engaged, "threshold_m": threshold_m,
            "engaged_share": (engaged / len(with_obstacle)) if with_obstacle else None}


def learned_seed_for(scenario_index: int, n_seeds: int = 5) -> int:
    """The training seed a LEARNED method runs pilot scenario `scenario_index` with (A13).

    Ruled 2026-09-29: the narrow pilot is 150 DISTINCT scenarios; each learned method runs
    scenario i with seed i mod 5 (30 scenarios per seed, 150 distinct per method), and each
    classical baseline runs all 150 once. Why not 30 scenarios x 5 seeds: a deterministic
    baseline would then give only 30 episodes, and a learned method's five episodes on one
    scenario are clustered, which the Clopper-Pearson bound -- a bound on independent Bernoulli
    trials -- does not allow. Distinct scenarios make the 150 trials per method independent.
    """
    if scenario_index < 0 or n_seeds < 1:
        raise ValueError("need scenario_index >= 0 and n_seeds >= 1")
    return scenario_index % n_seeds


def placement_mismatches(cfg: dict[str, Any], placement: dict[str, Any]) -> list[str]:
    """Where the config the generator READS disagrees with the placement A13 PREREGISTERS.

    `make_scenario` takes its numbers from `configs/base.yaml`; the preregistration is the
    authority. A set built from a config that has drifted from it would be a set nobody ruled,
    so the build script refuses while this list is non-empty.
    """
    u = cfg["world"]["layout"]["unmapped"]
    r = u["route_relative"]
    lo, hi = (float(v) for v in placement["offset_to_nearest_surface_m"])
    pairs = [
        ("offset range low", 0.0, lo),
        ("offset_max", float(r["offset_max"]), hi),
        ("first_offset_max", float(r["first_offset_max"]),
         float(placement["first_obstacle_offset_max_m"])),
        ("arc excluded from start", float(u["min_arc_from_start"]),
         float(placement["arc_excluded_from_start_m"])),
        ("arc excluded to goal", float(r["arc_to_goal"]),
         float(placement["arc_excluded_to_goal_m"])),
        ("attempts per obstacle", float(r["attempts_per_obstacle"]),
         float(placement["attempts_per_obstacle"])),
    ]
    # The bypass rule (revised 2026-10-02), once preregistered: W is checked; the clearance is
    # structural (half_width + the narrow floor, read in `BypassRule.from_config`).
    if "bypass" in placement:
        pairs.append(("bypass corridor half-width W",
                      float(r["bypass"]["corridor_half_width"]),
                      float(placement["bypass"]["corridor_half_width_m"])))
    return [f"{name}: config {got}, preregistered {want}"
            for name, got, want in pairs if abs(got - want) > 1e-12]


def rejection_summary(scenarios: list[dict[str, Any]], failures: list[dict[str, Any]],
                      no_placement_status: str) -> dict[str, Any]:
    """Placement draws, refusals by cause, and scenarios lost to placement, for one set."""
    attempts = sum(int(s.get("placement_attempts", 0)) for s in scenarios)
    by_cause: dict[str, int] = {}
    for s in scenarios:
        for cause, n in (s.get("placement_rejections") or {}).items():
            by_cause[cause] = by_cause.get(cause, 0) + int(n)
    refused = sum(by_cause.values())
    return {"attempts": attempts, "rejected": refused, "by_cause": dict(sorted(by_cause.items())),
            "rejection_rate": (refused / attempts) if attempts else None,
            "scenarios_lost_to_placement": sum(1 for f in failures
                                               if f.get("status") == no_placement_status)}


def position_by_index(scenarios: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    """Where along the route the capped obstacle (index 0) and the others came to rest, as a
    fraction of route length.

    They are DRAWN alike by construction (ruled 2026-09-30: "first" is an index, not a
    location), but kept only if the goal stays reachable, and a close obstacle is refused more
    often in a tight stretch than a far one. Any difference here comes from that selection; it
    is reported, not corrected.
    """
    groups: dict[str, list[float]] = {"capped": [], "others": []}
    for s in scenarios:
        length = float(s.get("path_length_m") or 0.0)
        if length <= 0.0:
            continue
        for k, arc in enumerate(s.get("unmapped_arcs_m") or []):
            groups["capped" if k == 0 else "others"].append(float(arc) / length)
    return {g: {"n": len(v), "mean_arc_fraction": (sum(v) / len(v)) if v else None}
            for g, v in groups.items()}


AISLE, AISLE_MOUTH, OPEN_FLOOR = "aisle", "aisle_mouth", "open_floor"


def location_of(centre_xy, aisles: list[tuple[float, float, float, float]],
                mouth_depth_m: float) -> str:
    """Where an obstacle's centre lies: inside an aisle BETWEEN two shelf rows, at an aisle's
    mouth (within `mouth_depth_m` beyond either end, in the aisle's own band), or on open floor.

    Ruled 2026-10-02: the share in each determines what "an unmapped obstacle in the narrow
    stratum" means in the results, so it is reported per stratum.
    """
    x, y = float(centre_xy[0]), float(centre_xy[1])
    for x0, y0, x1, y1 in aisles:
        if y0 <= y <= y1 and x0 <= x <= x1:
            return AISLE
    for x0, y0, x1, y1 in aisles:
        if y0 <= y <= y1 and (x0 - mouth_depth_m <= x < x0 or x1 < x <= x1 + mouth_depth_m):
            return AISLE_MOUTH
    return OPEN_FLOOR


def location_summary(locations: list[str]) -> dict[str, Any]:
    """Counts and shares of obstacle locations, for one set."""
    n = len(locations)
    counts = {k: sum(1 for v in locations if v == k) for k in (AISLE, AISLE_MOUTH, OPEN_FLOOR)}
    return {"obstacles": n, "counts": counts,
            "shares": {k: (c / n if n else None) for k, c in counts.items()}}
