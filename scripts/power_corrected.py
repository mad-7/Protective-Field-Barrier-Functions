"""Corrected two-level calibration and power, H1 and H2 (rulings of 2026-10-03). Researcher runs:

    python scripts/power_corrected.py --config configs/analysis/power_corrected.yaml --seed 0

The correction: each learned arm's seed-level variance is scaled by K/(K-1), and the interval uses a
t quantile with Welch-Satterthwaite degrees of freedom over the scenario part (n - 1) and each
learned arm's seed part (K - 1) (`pfbnav/eval/rate_ratio.py`, `corrected_interval`). Candidates:
the corrected two-level test at 5, 10, 15 and 20 seeds per learned arm, and T1 (policy-scoped,
5 seeds), reported in every case. Level alpha/3 throughout, from endpoints.yaml.

Stages: H1 calibration at equal rates (every candidate, each side, per comparator kind, over the
whole grid and at seed CV 0); the H1 grid (power that all seven reject, and that the four
classical comparisons reject -- branch 3 of the tree); H2 calibration at equal success; the H2 grid
(power that the difference is >= +15 pp AND significant). A candidate exceeding nominal + 3 MC SE
anywhere on either side fails the ruled gate.

The decision tree is recorded in `docs/decision_log.md` (2026-10-03) and is applied AFTER the seed
CV is measured on the tune set, at its upper 80 % bound. This run reports what the tree would give
for H1 at each swept seed CV, for orientation; it decides nothing. H2 has NO decisive cell until its
seed SD is measured (ruled 2026-10-03), so for H2 only the grid is reported.

`--parts h1 h2` (default both) runs only the named parts: the H2 grid was extended by ruling after
a run had started, and its H1 part did not need repeating.

Writes `runs/power/CORRECTED_<timestamp>_seed<seed>/`: `h1_calibration.csv`, `h1_grid.csv`,
`h2_calibration.csv`, `h2_grid.csv`, `summary.json`, and the usual provenance.
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
from pfbnav.eval.power import smallest_sufficient
from pfbnav.eval.power_grid import SCENARIOS_H1, h1_arms, h1_chunks, success_outcomes
from pfbnav.eval.rate_ratio import (
    FLAT,
    POLICY,
    WS,
    arm_terms,
    corrected_verdict,
    mean_difference,
    mean_terms,
    verdict_terms,
)
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
T1 = "T1"


def candidates(cfg: dict[str, Any]) -> list[str]:
    return [T1] + [f"C{k}" for k in cfg["seed_counts"]]


def seeds_of(name: str, cfg: dict[str, Any]) -> int:
    return int(cfg["policy_scoped_seeds"]) if name == T1 else int(name[1:])


def h1_verdicts(d, counts, names, cfg, comps, level, classical_df=WS):
    """{candidate: {comparator: (L5 lower, L5 higher)}} for one chunk."""
    terms: dict[int, dict[str, Any]] = {}
    out: dict[str, dict[str, tuple[np.ndarray, np.ndarray]]] = {}
    for name in names:
        k = seeds_of(name, cfg)
        if k not in terms:
            terms[k] = {a: arm_terms(s[:, :k, :], d) for a, s in counts.items()}
        t = terms[k]
        out[name] = {c.name: (verdict_terms(t["L5"], t[c.name], POLICY, level) if name == T1
                              else corrected_verdict(t["L5"], t[c.name], level,
                                                     classical_df=classical_df))
                       for c in comps}
    return out


def h2_verdicts(l5, l4, names, cfg, level):
    """{candidate: (L5 higher and significant, L5 lower and significant, estimate)}."""
    out = {}
    for name in names:
        k = seeds_of(name, cfg)
        a, b = mean_terms(l5[:, :k, :]), mean_terms(l4[:, :k, :])
        est = mean_difference(a, b)
        lower, higher = (verdict_terms(a, b, POLICY, level, estimate=est) if name == T1
                         else corrected_verdict(a, b, level, estimate=est))
        out[name] = (higher, lower, est)
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--parts", nargs="+", choices=["h1", "h2"], default=["h1", "h2"])
    args = parser.parse_args()

    full = load_config(args.config)
    seed_everything(args.seed)
    cfg = full["power_corrected"]
    h1, h2 = cfg["h1"], cfg["h2"]
    classical_df = str(cfg["classical_df"])
    if classical_df not in (WS, FLAT):
        raise SystemExit(f"classical_df must be {WS} or {FLAT}")
    endpoints = yaml.safe_load((REPO_ROOT / "preregistration" / "endpoints.yaml")
                               .read_text(encoding="utf-8"))
    if sorted(h1["classical"] + h1["learned"]) != sorted(endpoints["H1"]["comparators"]):
        raise SystemExit("the config's comparators are not H1's")
    family = endpoints["confirmatory_family"]
    level = float(endpoints["alpha"]) / (len(family["members"])
                                         + len(family.get("conditional_members", [])))
    nominal = level / 2.0
    cal_reps = int(cfg["calibration"]["reps"])
    tol = nominal + cfg["calibration"]["tolerance_mc_se"] * np.sqrt(
        nominal * (1 - nominal) / cal_reps)
    names = candidates(cfg)
    max_k = max(int(k) for k in cfg["seed_counts"])
    strata = len(full["world"]["aisle_strata"])
    raw = (REPO_ROOT / h1["route_lengths_from"]).read_bytes()
    lengths = [float(s["path_length_m"]) for s in json.loads(raw)["scenarios"]]
    kinds = {n: "classical" for n in h1["classical"]} | {n: "learned" for n in h1["learned"]}

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "power", f"CORRECTED_{stamp}", args.seed, full)
    rng = np.random.default_rng(args.seed)
    failures: dict[str, dict[str, list]] = {"H1": {n: [] for n in names},
                                            "H2": {n: [] for n in names}}

    # ---- H1 calibration --------------------------------------------------------------------
    rows: list[dict[str, Any]] = []
    grid: list[dict[str, Any]] = []
    run_h1 = "h1" in args.parts
    seed_cvs = list(h1["seed_cv"]) + list(cfg["calibration"]["extra_seed_cv"])
    cells = [(r, cv, scv, size) for r in h1["comparator_rates_per_100m"]
             for cv in h1["difficulty_cv"] for scv in seed_cvs for size in h1["sizes_per_cell"]]
    for i, (rate, cv, scv, size) in enumerate(cells if run_h1 else [], 1):
        l5, comps = h1_arms("equal_rates", float(rate), h1)
        hits = {(n, c.name, s): 0 for n in names for c in comps for s in (0, 1)}
        for d, counts in h1_chunks(rng, lengths, int(size) * strata, l5, comps, float(cv),
                                   float(scv), max_k, cal_reps):
            for n, per in h1_verdicts(d, counts, names, cfg, comps, level,
                                      classical_df).items():
                for c, (low, high) in per.items():
                    hits[(n, c, 0)] += int(low.sum())
                    hits[(n, c, 1)] += int(high.sum())
        row = {"rate": rate, "difficulty_cv": cv, "seed_cv": scv, "size_per_cell": size}
        for kind in ("classical", "learned"):
            members = [c.name for c in comps if kinds[c.name] == kind]
            for n in names:
                for s, label in ((0, "favour"), (1, "against")):
                    r_hat = float(np.mean([hits[(n, c, s)] for c in members]) / cal_reps)
                    row[f"{n}_{kind}_{label}"] = r_hat
                    if r_hat > tol:
                        failures["H1"][n].append({"cell": [rate, cv, scv, size], "kind": kind,
                                                  "side": label, "rate": r_hat})
        rows.append(row)
        print(f"[H1 calibration {i}/{len(cells)}] rate {rate}, cv {cv}, seed cv {scv}, {size}/cell: "
              + "; ".join(f"{n} {max(v for k, v in row.items() if k.startswith(n + '_')):.4f}"
                          for n in names))
    if rows:
        write_csv(run_dir / "h1_calibration.csv", rows)

    # ---- H1 grid ---------------------------------------------------------------------------
    cells = [(s, r, cv, scv, size) for s in SCENARIOS_H1 for r in h1["comparator_rates_per_100m"]
             for cv in h1["difficulty_cv"] for scv in h1["seed_cv"] for size in h1["sizes_per_cell"]]
    for i, (scenario, rate, cv, scv, size) in enumerate(cells if run_h1 else [], 1):
        l5, comps = h1_arms(scenario, float(rate), h1)
        allseven = {n: 0 for n in names}
        classical = {n: 0 for n in names}
        for d, counts in h1_chunks(rng, lengths, int(size) * strata, l5, comps, float(cv),
                                   float(scv), max_k, int(h1["reps"])):
            for n, per in h1_verdicts(d, counts, names, cfg, comps, level,
                                      classical_df).items():
                ok_all = np.ones(d.shape[0], dtype=bool)
                ok_cl = np.ones(d.shape[0], dtype=bool)
                for c, (low, _high) in per.items():
                    ok_all &= low
                    if kinds[c] == "classical":
                        ok_cl &= low
                allseven[n] += int(ok_all.sum())
                classical[n] += int(ok_cl.sum())
        for n in names:
            grid.append({"scenario": scenario, "rate": rate, "difficulty_cv": cv, "seed_cv": scv,
                         "size_per_cell": size, "candidate": n,
                         "power_all_seven": allseven[n] / h1["reps"],
                         "power_classical_only": classical[n] / h1["reps"]})
        print(f"[H1 grid {i}/{len(cells)}] {scenario}, rate {rate}, cv {cv}, seed cv {scv}, "
              f"{size}/cell: " + ", ".join(f"{n} {allseven[n] / h1['reps']:.3f}" for n in names))
    if grid:
        write_csv(run_dir / "h1_grid.csv", grid)

    # ---- H2 calibration and grid -----------------------------------------------------------
    run_h2 = "h2" in args.parts
    baselines = [tuple(float(v) for v in b) for b in h2["l4_success_ranges"]]

    def h2_cell(effect, sd, size, reps, lo, hi):
        n_pairs = int(size) * int(h2["cells_pooled"])
        tallies = {n: {"success": 0, "higher": 0, "lower": 0} for n in names}
        done = 0
        while done < reps:
            r = min(500, reps - done)
            base = rng.uniform(lo, hi, size=(r, n_pairs))
            l4 = success_outcomes(rng, base, 0.0, float(sd), max_k)
            l5 = success_outcomes(rng, base, float(effect), float(sd), max_k)
            for n, (higher, lower, est) in h2_verdicts(l5, l4, names, cfg, level).items():
                tallies[n]["higher"] += int(higher.sum())
                tallies[n]["lower"] += int(lower.sum())
                tallies[n]["success"] += int((higher & (est >= h2["min_difference"])).sum())
            done += r
        return tallies

    h2_rows: list[dict[str, Any]] = []
    sds = list(h2["seed_logit_sd"]) + list(cfg["calibration"]["extra_seed_sd"])
    h2_cal_cells = ([(b, sd, size) for b in baselines for sd in sds
                     for size in h2["sizes_per_cell"]] if run_h2 else [])
    for (lo, hi), sd, size in h2_cal_cells:
        t = h2_cell(0.0, sd, size, cal_reps, lo, hi)
        row = {"baseline": f"{lo}-{hi}", "seed_logit_sd": sd, "size_per_cell": size}
        for n in names:
            for side in ("higher", "lower"):
                r_hat = t[n][side] / cal_reps
                row[f"{n}_{side}"] = r_hat
                if r_hat > tol:
                    failures["H2"][n].append({"cell": [f"{lo}-{hi}", sd, size],
                                              "side": side, "rate": r_hat})
        h2_rows.append(row)
        print(f"[H2 calibration] baseline {lo}-{hi}, seed sd {sd}, {size}/cell: " + "; ".join(
            f"{n} {row[f'{n}_higher']:.4f}/{row[f'{n}_lower']:.4f}" for n in names))
    if h2_rows:
        write_csv(run_dir / "h2_calibration.csv", h2_rows)
    h2_grid: list[dict[str, Any]] = []
    h2_cells = ([(b, e, sd, size) for b in baselines for e in h2["effects"]
                 for sd in h2["seed_logit_sd"] for size in h2["sizes_per_cell"]] if run_h2 else [])
    for (lo, hi), effect, sd, size in h2_cells:
        t = h2_cell(effect, sd, size, int(h2["reps"]), lo, hi)
        for n in names:
            h2_grid.append({"baseline": f"{lo}-{hi}", "effect": effect,
                            "seed_logit_sd": sd, "size_per_cell": size,
                            "candidate": n,
                            "power": t[n]["success"] / h2["reps"],
                            "significant": t[n]["higher"] / h2["reps"]})
        print(f"[H2 grid] baseline {lo}-{hi}, effect {effect}, seed sd {sd}, "
              f"{size}/cell: " + ", ".join(
            f"{n} {t[n]['success'] / h2['reps']:.3f}" for n in names))
    if h2_grid:
        write_csv(run_dir / "h2_grid.csv", h2_grid)

    # ---- the tree, for orientation ---------------------------------------------------------
    target = float(cfg["min_power"])
    # Only the parts that ran have a calibration verdict; an absent part is not a passed one.
    ran = {"H1": run_h1, "H2": run_h2}
    passed = {h: ({n: not v for n, v in fails.items()} if ran[h] else None)
              for h, fails in failures.items()}
    mp = h1["middle_point"]

    def h1_power(n, scv, key):
        return {int(r["size_per_cell"]): r[key] for r in grid
                if r["candidate"] == n and r["scenario"] == mp["scenario"]
                and float(r["rate"]) == float(mp["comparator_rate_per_100m"])
                and float(r["difficulty_cv"]) == float(mp["difficulty_cv"])
                and float(r["seed_cv"]) == float(scv)}

    def tree(power_of, ok, h):
        if ok.get("C10") and smallest_sufficient(power_of("C10"), target) is not None:
            return {"branch": 1, "seeds": 10, "size": smallest_sufficient(power_of("C10"), target)}
        for n in names:
            if n in (T1, "C5", "C10") or not ok.get(n):
                continue
            size = smallest_sufficient(power_of(n), target)
            if size is not None:
                return {"branch": 2, "seeds": seeds_of(n, cfg), "size": size,
                        "note": "subject to G2"}
        return {"branch": 3, "note": "method-level claim restricted / policy-scoped"
                if h == "H1" else "H2 becomes policy-scoped"}

    orientation: dict[str, Any] = {"H1": {}, "H2": {}}
    h1_ok = passed["H1"] or {}
    for scv in (h1["seed_cv"] if run_h1 else []):
        result = tree(lambda n, s=scv: h1_power(n, s, "power_all_seven"), h1_ok, "H1")
        if result["branch"] == 3:
            for n in names:
                if n != T1 and h1_ok.get(n):
                    size = smallest_sufficient(h1_power(n, scv, "power_classical_only"), target)
                    if size is not None:
                        result["classical_only"] = {"l5_seeds": seeds_of(n, cfg), "size": size}
                        break
        orientation["H1"][str(scv)] = result
    orientation["H2"] = "no decisive cell until the seed SD is measured on the tune set (ruled)"

    summary = {"level": level, "nominal_per_side": nominal, "calibration_tolerance": tol,
               "passes_calibration": passed,
               "calibration_failures": {h: {n: v[:20] for n, v in f.items()}
                                        for h, f in failures.items()},
               "parts": args.parts, "classical_df": classical_df, "h1_middle_point": mp,
               "h1_power_at_middle_point": {n: h1_power(n, mp["seed_cv"], "power_all_seven")
                                            for n in names} if run_h1 else None,
               "tree_for_orientation_only": orientation,
               "route_lengths": {"path": h1["route_lengths_from"], "n": len(lengths),
                                 "sha256": hashlib.sha256(raw).hexdigest()}}
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=float),
                                          encoding="utf-8")
    print("passes calibration:", json.dumps(passed))
    for n, pw in (summary["h1_power_at_middle_point"] or {}).items():
        print(f"  H1 {n} at the middle point: " + ", ".join(f"{s}/cell {v:.3f}"
                                                           for s, v in pw.items()))
    print("tree (orientation only):", json.dumps(orientation))
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
