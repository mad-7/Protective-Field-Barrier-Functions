"""Oracle mission time, old against new (A20, ruled 2026-10-19). Pure.

Reads two free-orientation-oracle info tables (`routes.csv`, `scripts/quantify_oracle_free.py
--phase info`) -- the pre-A20 sets under the pre-A20 field and limits, and the A20 sets under A20 at
v_max 0.50 -- and compares, at one (lethal, soft) pair:
  * per stratum: routes, traversable share, median mission time, median time per planned metre;
  * matched by (set, seed) where both are traversable: the median ratio new / old. The wide
    layouts are unchanged by A20 (its stratum did not move), so the wide pairs are the same routes;
    narrow and medium seeds were rebuilt under new strata and are different worlds, so only their
    per-metre figures compare like with like.
"""
from __future__ import annotations

import csv
import statistics
from collections.abc import Iterable
from pathlib import Path
from typing import Any


def read_routes(path: Path, lethal: float, soft: float) -> list[dict[str, Any]]:
    out = []
    with Path(path).open(encoding="utf-8", newline="") as f:
        for r in csv.DictReader(f):
            if abs(float(r["lethal_margin"]) - lethal) > 1e-9 or \
                    abs(float(r["soft_margin"]) - soft) > 1e-9:
                continue
            t = r.get("oracle_time_s") or ""
            out.append({"set": r["set"], "seed": int(r["seed"]), "stratum": r["stratum"],
                        "status": r["status"],
                        "length_m": float(r["path_length_m"]) if r["path_length_m"] else None,
                        "traversable": r.get("traversable") == "1",
                        "time_s": float(t) if t not in ("", "inf", "None") else None})
    return out


def _median(xs: Iterable[float]) -> float | None:
    v = list(xs)
    return statistics.median(v) if v else None


def per_stratum(rows: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    out = {}
    for s in sorted({r["stratum"] for r in rows}):
        g = [r for r in rows if r["stratum"] == s]
        timed = [r for r in g if r["traversable"] and r["time_s"] is not None and r["length_m"]]
        out[s] = {"routes": len(g), "traversable": sum(r["traversable"] for r in g),
                  "median_time_s": _median(r["time_s"] for r in timed),
                  "median_s_per_m": _median(r["time_s"] / r["length_m"] for r in timed)}
    return out


def matched(old: list[dict[str, Any]], new: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    """Per stratum, over (set, seed) pairs timed in both: the median new / old time ratio."""
    key = {(r["set"], r["seed"]): r for r in old if r["time_s"] is not None}
    out: dict[str, dict[str, Any]] = {}
    for r in new:
        o = key.get((r["set"], r["seed"]))
        if o is None or r["time_s"] is None:
            continue
        entry = out.setdefault(r["stratum"], {"pairs": 0, "ratios": []})
        entry["pairs"] += 1
        entry["ratios"].append(r["time_s"] / o["time_s"])
    return {s: {"pairs": e["pairs"], "median_ratio": _median(e["ratios"]),
                "min_ratio": min(e["ratios"]), "max_ratio": max(e["ratios"])}
            for s, e in out.items()}
