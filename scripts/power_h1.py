"""H1 power analysis: a Monte Carlo of the frozen analysis, for amendment A14 (the test size).

Run by the RESEARCHER (hard rule 11):

    python scripts/power_h1.py --config configs/analysis/h1_power.yaml --seed 0

The model and the decision rule are the supervisor's (2026-10-02) and live in the config; alpha and
the comparators are read from `preregistration/endpoints.yaml`; the route lengths from the pilot
set. See `pfbnav/eval/power.py` for the model and for what is NOT modelled (the paired
detectability floor, undefined in the repository).

**Decision rule, declared before running:** the smallest size per cell in {50, 75, 100} whose power
is at least 0.80 at comparator rate 5 per 100 m, difficulty CV 1.0, all seven comparators at the 30 %
margin. It is evaluated at both Holm worst cases, alpha/2 and alpha/3. If 100 does not reach 0.80,
the script says so and goes no higher (supervisor: "stop and report to me").

Also recorded, at the decision point: each comparison's rejection rate in each direction when L5
and the comparators have EQUAL rates -- the test's behaviour under the null, which the
known-answer tests showed is not alpha/2 when the arms differ in kind or carry seed effects.

Writes `runs/power/H1_POWER_<timestamp>_seed<seed>/` with `grid.csv`, `summary.json` and the
provenance every run carries.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pfbnav.config import load_config
from pfbnav.eval.power import Arm, simulate_h1, smallest_sufficient
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ("all_at_margin", "one_at_margin_classical", "one_at_margin_learned")


def arms_for(scenario: str, rate: float, p: dict[str, Any]) -> tuple[Arm, list[Arm]]:
    """L5 and the seven comparators for one cell of the grid."""
    l5_rate = p["l5_ratio"] * rate
    other = p["secondary_other_ratio_to_l5"] * l5_rate
    names = [(n, False) for n in p["classical"]] + [(n, True) for n in p["learned"]]
    margin = {"all_at_margin": {n for n, _ in names},
              "one_at_margin_classical": {p["classical"][0]},
              "one_at_margin_learned": {p["learned"][0]},
              "null": set()}[scenario]
    if scenario == "null":
        return Arm("L5", rate, True), [Arm(n, rate, k) for n, k in names]
    return Arm("L5", l5_rate, True), [Arm(n, rate if n in margin else other, k)
                                      for n, k in names]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    p = cfg["h1_power"]
    endpoints = yaml.safe_load((REPO_ROOT / "preregistration" / "endpoints.yaml")
                               .read_text(encoding="utf-8"))
    comparators = list(endpoints["H1"]["comparators"])
    if sorted(p["classical"] + p["learned"]) != sorted(comparators):
        raise SystemExit(f"the config's comparators {p['classical'] + p['learned']} are not "
                         f"H1's {comparators}")
    alpha = float(endpoints["alpha"])
    holm = len(endpoints["confirmatory_family"]["members"])
    holm_h1n = holm + len(endpoints["confirmatory_family"].get("conditional_members", []))
    levels = {"alpha": alpha, f"alpha/{holm}": alpha / holm, f"alpha/{holm_h1n}": alpha / holm_h1n}
    strata = len(cfg["world"]["aisle_strata"])
    lengths_path = REPO_ROOT / p["route_lengths_from"]
    raw = lengths_path.read_bytes()
    lengths = [float(s["path_length_m"]) for s in json.loads(raw)["scenarios"]]

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "power", f"H1_POWER_{stamp}", args.seed, cfg)
    rng = np.random.default_rng(args.seed)
    rows: list[dict[str, Any]] = []
    cells = [(s, r, cv, size) for s in SCENARIOS for r in p["comparator_rates_per_100m"]
             for cv in p["difficulty_cv"] for size in p["sizes_per_cell"]]
    dec = p["decision"]
    null_cells = [("null", float(dec["comparator_rate_per_100m"]), float(dec["difficulty_cv"]), size)
                  for size in p["sizes_per_cell"]]
    for i, (scenario, rate, cv, size) in enumerate(cells + null_cells, 1):
        l5, comps = arms_for(scenario, float(rate), p)
        got = simulate_h1(rng, lengths, int(size) * strata, l5, comps,
                          difficulty_cv=float(cv), seed_cv=float(p["seed_cv"]),
                          n_seeds=int(p["n_seeds"]), levels=list(levels.values()),
                          reps=int(p["reps"]))
        for label, lv in levels.items():
            row = {"scenario": scenario, "comparator_rate_per_100m": rate, "difficulty_cv": cv,
                   "size_per_cell": size, "pairs": int(size) * strata, "level": label,
                   "power_all_seven": got["power"][lv]}
            row.update({f"reject_{n}": got["marginal"][lv][n] for n in comparators})
            row.update({f"opposite_{n}": got["opposite"][lv][n] for n in comparators})
            rows.append(row)
        print(f"[{i}/{len(cells) + len(null_cells)}] {scenario}, rate {rate}, cv {cv}, "
              f"{size}/cell: power " + ", ".join(f"{k} {got['power'][v]:.3f}"
                                                 for k, v in levels.items()))

    with (run_dir / "grid.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    decisions: dict[str, Any] = {}
    for label in levels:
        if label == "alpha":
            continue
        at = {int(r["size_per_cell"]): r["power_all_seven"] for r in rows
              if r["scenario"] == dec["scenario"] and r["level"] == label
              and float(r["comparator_rate_per_100m"]) == float(dec["comparator_rate_per_100m"])
              and float(r["difficulty_cv"]) == float(dec["difficulty_cv"])}
        decisions[label] = {"power_by_size": at,
                            "smallest_sufficient": smallest_sufficient(at, dec["min_power"])}
    chosen = {d["smallest_sufficient"] for d in decisions.values()}
    if None in chosen:
        verdict = ("STOP AND REPORT: 100 per cell does not reach the required power at the decision "
                   "point at one or both Holm worst cases. No size is chosen; nothing above 100 "
                   "was tried.")
    elif len(chosen) > 1:
        verdict = ("REPORT: the two Holm worst cases choose different sizes; the supervisor "
                   "decides which level applies.")
    else:
        verdict = f"Smallest sufficient size: {chosen.pop()} per cell at both Holm worst cases."
    summary = {"decision_rule": dec, "levels": levels, "strata_pooled": strata,
               "route_lengths": {"path": p["route_lengths_from"], "n": len(lengths),
                                 "sha256": hashlib.sha256(raw).hexdigest()},
               "decisions": decisions, "verdict": verdict, "reps": p["reps"],
               "not_modelled": "the paired detectability floor (undefined in the repository); "
                               "power is an upper bound on H1's success probability"}
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(verdict)
    for label, d in decisions.items():
        print(f"  {label}: " + ", ".join(f"{s}/cell {v:.3f}" for s, v in d["power_by_size"].items()))
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
