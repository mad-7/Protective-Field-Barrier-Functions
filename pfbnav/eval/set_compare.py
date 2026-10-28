"""Compare two builds of a scenario set, scenario by scenario. Pure.

Written for a rebuild whose only purpose is to re-stamp a set after a change that moved the
scenario module's identity (`scenarios_sha256`) without touching what the generator produces --
the rotation-aware field of 2026-10-16 changed `certified.py`, which is inside that identity's
import closure (A18). Such a rebuild must reproduce every scenario exactly; this says whether it
did, field by field, ignoring only the fields named.
"""
from __future__ import annotations

from collections.abc import Iterable
from typing import Any


def compare_sets(old: dict[str, Any], new: dict[str, Any],
                 ignore: Iterable[str] = ("scenarios_sha256",)) -> dict[str, Any]:
    """{'identical', 'scenarios', 'differences'}: every difference as {'seed', 'field'} (or a
    seed present in one build only), comparing each scenario's record with `ignore` removed."""
    skip = set(ignore)
    o = {int(s["seed"]): s for s in old["scenarios"]}
    n = {int(s["seed"]): s for s in new["scenarios"]}
    diffs: list[dict[str, Any]] = []
    for seed in sorted(set(o) | set(n)):
        if seed not in o or seed not in n:
            diffs.append({"seed": seed, "field": "present in one build only"})
            continue
        keys = (set(o[seed]) | set(n[seed])) - skip
        diffs += [{"seed": seed, "field": k} for k in sorted(keys)
                  if o[seed].get(k) != n[seed].get(k)]
    return {"identical": not diffs, "scenarios": len(n), "differences": diffs}
