"""HF-c's calibration gate (A36, 2026-11-07): B7 vs B2 at EQUAL rates must reject on each side no more
often than 0.025 + 3 MC SE before HF-c is tested. CPU, no simulator, about a minute:

    python scripts/calibrate_hfc.py --config configs/analysis/hfc_calibration.yaml --seed 0

Every (rate, difficulty CV) cell: `n_pairs` routes drawn from the pilot set's planned lengths, Poisson
stops with a shared gamma difficulty (`pfbnav/eval/power_grid.py` h1_chunks), the interval of
`pfbnav/eval/hfc.py`'s form (scenario part only, t on n - 1 df). Gated cells decide; reported cells are
printed and stored only. Writes `runs/power/HFC_CAL_<stamp>_seed<seed>/summary.json` with `gate_pass`;
`scripts/analyse_hfc.py` refuses to read the test without a passing one.
"""
from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from pfbnav.config import load_config
from pfbnav.eval.power import Arm
from pfbnav.eval.power_grid import h1_chunks
from pfbnav.eval.rate_ratio import WS, arm_terms, corrected_verdict
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]


def gate(reps: int, nominal: float, k_se: float) -> float:
    """The A14a gate: nominal + k MC SE of a rejection share at `reps` replications."""
    return nominal + k_se * math.sqrt(nominal * (1.0 - nominal) / reps)


def cell(rng: np.random.Generator, lengths: list[float], rate: float, cv: float, n: int, reps: int,
         level: float) -> tuple[float, float]:
    """(share rejecting 'a fewer', share rejecting 'a more') at equal rates."""
    a, b = Arm("B7", rate, False), Arm("B2", rate, False)
    lower = higher = 0
    for d, counts in h1_chunks(rng, lengths, n, a, [b], cv, 0.0, 1, reps):
        lo, hi = corrected_verdict(arm_terms(counts["B7"], d), arm_terms(counts["B2"], d), level,
                                   classical_df=WS)
        lower, higher = lower + int(lo.sum()), higher + int(hi.sum())
    return lower / reps, higher / reps


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    args = parser.parse_args()
    full = load_config(args.config)
    seed_everything(args.seed)
    p = full["hfc_calibration"]
    level, n, reps = float(p["level"]), int(p["n_pairs"]), int(p["reps"])
    bound = gate(reps, level / 2.0, float(p["tolerance_mc_se"]))
    lengths = [float(s["path_length_m"]) for s in json.loads(
        (REPO_ROOT / p["route_lengths_from"]).read_text(encoding="utf-8"))["scenarios"]]
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "power", f"HFC_CAL_{stamp}", args.seed, full)
    rng = np.random.default_rng(args.seed)
    rows, ok = [], True
    plan = ([(r, c, True) for r in p["gated_rates_per_100m"] for c in p["gated_difficulty_cv"]]
            + [(r, c, False) for r in p["reported_rates_per_100m"]
               for c in p["gated_difficulty_cv"] + p["reported_difficulty_cv"]]
            + [(r, c, False) for r in p["gated_rates_per_100m"] for c in p["reported_difficulty_cv"]])
    for rate, cv, gated in plan:
        lo, hi = cell(rng, lengths, float(rate), float(cv), n, reps, level)
        passed = max(lo, hi) <= bound
        ok &= passed or not gated
        rows.append({"rate_per_100m": rate, "difficulty_cv": cv, "gated": gated,
                     "reject_a_fewer": lo, "reject_a_more": hi, "pass": passed})
        print(f"rate {rate}, cv {cv}{'' if gated else ' (reported)'}: {lo:.4f} / {hi:.4f} "
              f"{'PASS' if passed else 'FAIL'}", flush=True)
    (run_dir / "summary.json").write_text(json.dumps({
        "gate_pass": bool(ok), "bound": bound, "level": level, "n_pairs": n, "reps": reps,
        "cells": rows}, indent=2), encoding="utf-8")
    print(f"GATE {'PASS' if ok else 'FAIL: stop and report; HF-c is not tested'} (bound {bound:.4f})")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
