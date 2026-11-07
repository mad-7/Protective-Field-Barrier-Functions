"""A14b: H1's power on a grid of comparator rates and rate ratios, at K = 10 (ruled 2026-11-07). The
RESEARCHER runs it (CPU, no simulator; hours):

    python scripts/power_a14b.py --config configs/analysis/power_a14b.yaml --seed 0

For every cell -- comparator rate x rate ratio x seed CV, every comparator at the rate (the decisive
`all_at_margin` case of the intersection-union) -- and every candidate size, the share of replications
in which ALL SEVEN H1 comparisons reject in L5's favour, with the corrected two-level interval at
K = 10 (`scripts/power_corrected.py`'s verdicts, Holm level alpha/3 from endpoints.yaml). Classical
arms run once, so their events scale with scenarios x route length, not seeds; the simulation carries
that (`pfbnav/eval/power_grid.py`). Reports per cell the smallest size with power >= 0.80 (None if no
size on the grid reaches it), and at ratio 1.0 the rejection rate (the test's size, not a power).
For each size, the test evaluation's wall-clock from the per-arm episode costs (config, estimates).

Writes `runs/power/A14B_<stamp>_seed<seed>/`: `grid.csv` (every cell and size), `summary.json` (the
smallest sufficient size per cell and the wall-clock per size), and the usual provenance.
"""
from __future__ import annotations

import argparse
import csv
import importlib.util
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pfbnav.config import load_config
from pfbnav.eval.power import smallest_sufficient
from pfbnav.eval.power_grid import h1_arms, h1_chunks
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]


def _corrected():
    spec = importlib.util.spec_from_file_location("power_corrected",
                                                  REPO_ROOT / "scripts" / "power_corrected.py")
    if spec is None or spec.loader is None:
        raise SystemExit("cannot load scripts/power_corrected.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def wall_clock_h(n_scenarios: int, p: dict[str, Any]) -> float:
    """Hours for the test evaluation of `n_scenarios`: classical arms once, learned arms K times,
    each episode at its declared cost, spread over `instances` simulator instances."""
    cost = p["episode_wall_s"]
    k = int(p["seeds_per_learned_arm"])
    per_scenario = (sum(float(cost[a]) for a in p["classical_arms_evaluated"])
                    + k * sum(float(cost[a]) for a in p["learned_arms_evaluated"]))
    return n_scenarios * per_scenario / int(p["instances"]) / 3600.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    args = parser.parse_args()
    full = load_config(args.config)
    seed_everything(args.seed)
    p = full["power_a14b"]
    pc = _corrected()
    endpoints = yaml.safe_load((REPO_ROOT / "preregistration" / "endpoints.yaml")
                               .read_text(encoding="utf-8"))
    if sorted(p["classical"] + p["learned"]) != sorted(endpoints["H1"]["comparators"]):
        raise SystemExit("the config's comparators are not H1's")
    family = endpoints["confirmatory_family"]
    level = float(endpoints["alpha"]) / (len(family["members"])
                                         + len(family.get("conditional_members", [])))
    k = int(p["seeds_per_learned_arm"])
    cand = f"C{k}"
    cfg_v = {"seed_counts": [k], "policy_scoped_seeds": k}
    strata = len(full["world"]["aisle_strata"])
    lengths = [float(s["path_length_m"]) for s in json.loads(
        (REPO_ROOT / p["route_lengths_from"]).read_text(encoding="utf-8"))["scenarios"]]
    h1p = {"classical": p["classical"], "learned": p["learned"], "secondary_other_ratio_to_l5": 1.0}

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "power", f"A14B_{stamp}", args.seed, full)
    rng = np.random.default_rng(args.seed)
    rows: list[dict[str, Any]] = []
    cells: dict[str, Any] = {}
    reps = int(p["reps"])
    for rate in p["comparator_rates_per_100m"]:
        for ratio in p["rate_ratios"]:
            for cv in p["difficulty_cv"]:
                for scv in p["seed_cv"]:
                    by_size: dict[int, float] = {}
                    for size in p["sizes_per_cell"]:
                        l5, comps = h1_arms("all_at_margin", float(rate),
                                            {**h1p, "l5_ratio": float(ratio)})
                        hits = 0
                        for d, counts in h1_chunks(rng, lengths, int(size) * strata, l5, comps,
                                                   float(cv), float(scv), k, reps):
                            per = pc.h1_verdicts(d, counts, [cand], cfg_v, comps, level,
                                                 str(p["classical_df"]))[cand]
                            ok = np.ones(d.shape[0], dtype=bool)
                            for low, _high in per.values():
                                ok &= low
                            hits += int(ok.sum())
                        by_size[int(size)] = hits / reps
                        rows.append({"rate": rate, "ratio": ratio, "difficulty_cv": cv,
                                     "seed_cv": scv, "size_per_stratum": size,
                                     "scenarios": int(size) * strata,
                                     "power_all_seven": by_size[int(size)]})
                        print(f"rate {rate}, ratio {ratio}, seed cv {scv}, {size}/stratum: "
                              f"{by_size[int(size)]:.3f}", flush=True)
                    key = f"rate={rate},ratio={ratio},difficulty_cv={cv},seed_cv={scv}"
                    need = smallest_sufficient(by_size, float(p["min_power"])) if ratio < 1.0 \
                        else None
                    cells[key] = {"smallest_size_per_stratum_for_80": need,
                                  "rejection_rate_under_null" if ratio == 1.0 else "power":
                                  by_size}
    with (run_dir / "grid.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    wall = {int(s): {"scenarios": int(s) * strata,
                     "wall_clock_h": wall_clock_h(int(s) * strata, p),
                     "wall_clock_days": wall_clock_h(int(s) * strata, p) / 24.0}
            for s in p["sizes_per_cell"]}
    (run_dir / "summary.json").write_text(json.dumps({
        "level": level, "seeds_per_learned_arm": k, "cells": cells, "wall_clock_per_size": wall,
        "episode_wall_s": p["episode_wall_s"], "instances": p["instances"]}, indent=2),
        encoding="utf-8")
    for s, w in wall.items():
        print(f"{s}/stratum = {w['scenarios']} scenarios: {w['wall_clock_days']:.1f} days")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
