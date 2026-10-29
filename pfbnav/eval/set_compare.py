"""Compare two builds of a scenario set, scenario by scenario. Pure.

Written for a rebuild whose only purpose is to re-stamp a set after a change that moved the
scenario module's identity (`scenarios_sha256`) without touching what the generator produces --
the rotation-aware field of 2026-10-16 changed `certified.py`, which is inside that identity's
import closure (A18). Such a rebuild must reproduce every scenario; this says whether it did, field
by field, ignoring only the fields named.

**Ruled 2026-10-29** (after A29's re-stamp): identities and hashes -- every string, integer and
boolean, every key and every list length -- must match EXACTLY; recorded floats are compared with a
declared absolute tolerance (`world.set_verification.float_abs_tol_m`, 1e-9 m). The null test
(hard rule 19) is the measured cross-host difference: the same commit built on the laptop (AVX2)
and the training host (AVX-512), same NumPy wheel, differed in 158 float fields by at most
1.9e-15 m and nowhere else. Under that null the expected false-alarm rate is zero: the tolerance is
~5e5 times the largest rounding difference measured. `identical` still reports bit-for-bit
equality; `accepted` is the ruled criterion.
"""
from __future__ import annotations

import math
from collections.abc import Iterable
from typing import Any

#: A structural difference: the records differ in something the tolerance never covers.
STRUCTURAL = math.inf


def float_distance(a: Any, b: Any) -> float:
    """The largest |a - b| over the floats of two JSON values, 0.0 where they are equal, and
    `STRUCTURAL` (inf) where they differ in anything but float values: a string, an integer, a
    boolean, a key, a list length, or a type (an integer is never compared as a float)."""
    if isinstance(a, bool) or isinstance(b, bool) or isinstance(a, int) or isinstance(b, int):
        return 0.0 if type(a) is type(b) and a == b else STRUCTURAL
    if isinstance(a, float) and isinstance(b, float):
        if math.isnan(a) or math.isnan(b):
            return STRUCTURAL                 # a NaN is never "within tolerance" of anything
        return abs(a - b)
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return STRUCTURAL
        return max((float_distance(x, y) for x, y in zip(a, b)), default=0.0)
    if isinstance(a, dict) and isinstance(b, dict):
        if set(a) != set(b):
            return STRUCTURAL
        return max((float_distance(a[k], b[k]) for k in a), default=0.0)
    return 0.0 if type(a) is type(b) and a == b else STRUCTURAL


def compare_sets(old: dict[str, Any], new: dict[str, Any],
                 ignore: Iterable[str] = ("scenarios_sha256",),
                 float_tol: float = 0.0) -> dict[str, Any]:
    """{'identical', 'accepted', 'max_float_difference', 'scenarios', 'differences'}.

    Every difference is {'seed', 'field', 'max_abs'} (or a seed present in one build only, with
    `max_abs` inf). `identical`: no difference at all. `accepted`: every difference is in float
    values only, each within `float_tol` (m). `max_float_difference`: the largest finite one."""
    if float_tol < 0.0:
        raise ValueError(f"float_tol must be non-negative, got {float_tol}")
    skip = set(ignore)
    o = {int(s["seed"]): s for s in old["scenarios"]}
    n = {int(s["seed"]): s for s in new["scenarios"]}
    diffs: list[dict[str, Any]] = []
    for seed in sorted(set(o) | set(n)):
        if seed not in o or seed not in n:
            diffs.append({"seed": seed, "field": "present in one build only", "max_abs": STRUCTURAL})
            continue
        for k in sorted((set(o[seed]) | set(n[seed])) - skip):
            if k not in o[seed] or k not in n[seed]:
                diffs.append({"seed": seed, "field": k, "max_abs": STRUCTURAL})
            else:
                # Not `!=`: Python calls 3 and 3.0 equal, and a type change is not a float's.
                dist = float_distance(o[seed][k], n[seed][k])
                if dist != 0.0:
                    diffs.append({"seed": seed, "field": k, "max_abs": dist})
    finite = [d["max_abs"] for d in diffs if math.isfinite(d["max_abs"])]
    return {"identical": not diffs,
            "accepted": all(d["max_abs"] <= float_tol for d in diffs),
            "max_float_difference": max(finite, default=0.0),
            "scenarios": len(n), "differences": diffs}
