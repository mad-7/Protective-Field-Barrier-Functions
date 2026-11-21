"""The validation set: scenarios no baseline was tuned on and no alpha was chosen on. Pure helpers; no simulator.

The classical baselines were tuned on `scenarios/tune/` and the PFB's alpha was chosen with B7 on the same
set, so a comparison on it favours the baselines' tuning and the chosen alpha. The validation set is built
from fresh seeds in the tune range, in the same six cells as the test set (three aisle strata, with and
without unmapped obstacles), so it is read the same way and is never a test set.

Seeds: one block of `BLOCK` seeds per cell from `FIRST_SEED`, checked disjoint from every other built set.
Each cell asks for `count + headroom` seeds and keeps the first `count` BUILT scenarios in seed order: a
rule that depends on the seeds only, never on how a controller does on them.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

FIRST_SEED = 2_500_000            # tune range 2,000,000-2,999,999; tune sets use 2,000,000-2,000,119
BLOCK = 1_000                     # seeds reserved per cell
TUNE_RANGE = (2_000_000, 2_999_999)

# (stratum, condition, unmapped-obstacle episode rate), the test set's cells.
CELLS: tuple[tuple[str, str, float], ...] = (
    ("narrow", "mapped_only", 0.0), ("medium", "mapped_only", 0.0), ("wide", "mapped_only", 0.0),
    ("narrow", "unmapped", 1.0), ("medium", "unmapped", 1.0), ("wide", "unmapped", 1.0),
)


def cell_name(stratum: str, condition: str) -> str:
    return f"validation_{condition}_{stratum}"


def cell_seeds(index: int, count: int, headroom: int, first_seed: int = FIRST_SEED) -> range:
    """The seeds a cell asks for: its own block, `count + headroom` long."""
    if count + headroom > BLOCK:
        raise ValueError(f"{count + headroom} seeds do not fit in a block of {BLOCK}")
    start = first_seed + index * BLOCK
    return range(start, start + count + headroom)


def used_seeds(scenarios_root: Path) -> dict[int, str]:
    """Every layout seed in every built set under `scenarios_root`, with the file that holds it,
    except the validation sets themselves (so a rebuild does not collide with its own output)."""
    out: dict[int, str] = {}
    for path in sorted(scenarios_root.rglob("*.json")):
        if path.parent.name == "validation":
            continue
        for s in json.loads(path.read_text(encoding="utf-8")).get("scenarios", []):
            out[int(s["seed"])] = path.relative_to(scenarios_root).as_posix()
    return out


def overlaps(seeds: range, used: dict[int, str]) -> dict[int, str]:
    """The seeds of `seeds` already used by another set."""
    return {s: used[s] for s in seeds if s in used}


def truncate(manifest: dict[str, Any], count: int) -> dict[str, Any]:
    """The manifest with its first `count` built scenarios (seed order). Raises if fewer were built."""
    built = list(manifest["scenarios"])
    if len(built) < count:
        raise ValueError(f"only {len(built)} scenarios were built, {count} were asked for")
    kept = sorted(built, key=lambda s: int(s["seed"]))[:count]
    out = dict(manifest)
    out["scenarios"] = kept
    out["built"] = out["requested"] = len(kept)
    with_unmapped = sum(1 for s in kept if s["unmapped_count"] > 0)
    out["episode_rate_achieved"] = with_unmapped / len(kept) if kept else None
    out["kind"] = "validation"
    return out
