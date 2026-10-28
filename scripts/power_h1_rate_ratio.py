"""H1 power under the rate-ratio estimand (A14 rulings, 2026-10-02). Researcher runs it:

    python scripts/power_h1_rate_ratio.py --config configs/analysis/h1_rate_ratio.yaml --seed 0

Three stages, in the ruled order:

1. **Calibration.** Every candidate (T1 policy-scoped, T2 two-level with 5 seeds, T3 two-level with
   10) at EQUAL rates over the whole grid, rejection rate in each direction against nominal
   (level / 2, level = alpha / 3). A candidate exceeding it on either side is excluded (tolerance
   declared in the config). The continuity correction's effect at equal rates is reported: the mean
   estimated log rate ratio per comparator kind, beside the bias `continuity_bias` predicts.
2. **The grid**, by the normal approximation on the log rate ratio: power that all seven comparisons
   reject in L5's favour, for every scenario x rate x difficulty CV x seed CV x size.
3. **The full bootstrap at selected cells**, against the normal approximation: rejection rates
   under both, and how often the two agree replicate by replicate.

Then the decision rule declared in the config. Data are simulated once per cell with 10 seeds per
learned arm; T1 and T2 use the first five, so the candidates are compared on common random numbers.

Writes `runs/power/H1_RR_<timestamp>_seed<seed>/`: `calibration.csv`, `grid.csv`,
`bootstrap_check.csv`, `summary.json`, and the provenance every run carries.
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
from pfbnav.eval.power import Arm, gamma_multiplier, smallest_sufficient
from pfbnav.eval.rate_ratio import (
    arm_terms,
    bootstrap_log_rr,
    continuity_bias,
    log_rate_ratio_terms,
    percentile_verdict,
    stop_counts,
    verdict_terms,
)
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ("all_at_margin", "one_at_margin_classical", "one_at_margin_learned")
CHUNK = 1000


def arms_for(scenario: str, rate: float, p: dict[str, Any]) -> tuple[Arm, list[Arm]]:
    names = [(n, False) for n in p["classical"]] + [(n, True) for n in p["learned"]]
    if scenario == "equal_rates":
        return Arm("L5", rate, True), [Arm(n, rate, k) for n, k in names]
    l5 = p["l5_ratio"] * rate
    other = p["secondary_other_ratio_to_l5"] * l5
    margin = {"all_at_margin": {n for n, _ in names},
              "one_at_margin_classical": {p["classical"][0]},
              "one_at_margin_learned": {p["learned"][0]}}[scenario]
    return Arm("L5", l5, True), [Arm(n, rate if n in margin else other, k) for n, k in names]


def simulate(rng, lengths, n_pairs, l5, comps, diff_cv, seed_cv, max_seeds, reps):
    """Yield chunks of (route lengths, {arm: counts with max_seeds seeds for learned arms})."""
    pool = np.asarray(lengths, dtype=float)
    done = 0
    while done < reps:
        r = min(CHUNK, reps - done)
        d = rng.choice(pool, size=(r, n_pairs), replace=True)
        diff = gamma_multiplier(rng, diff_cv, (r, n_pairs))
        counts = {a.name: stop_counts(rng, a, d, diff, max_seeds, seed_cv) for a in [l5, *comps]}
        yield d, counts
        done += r


def verdicts(d, counts, candidates, comps, level):
    """{candidate: {comparator: (favour, against)}} for one chunk."""
    out: dict[str, dict[str, tuple[np.ndarray, np.ndarray]]] = {}
    by_seeds: dict[int, dict[str, Any]] = {}
    for name, c in candidates.items():
        k = int(c["seeds"])
        if k not in by_seeds:
            by_seeds[k] = {a: arm_terms(s[:, :k, :], d) for a, s in counts.items()}
        t = by_seeds[k]
        out[name] = {a.name: verdict_terms(t["L5"], t[a.name], c["mode"], level) for a in comps}
    return out, by_seeds


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    p = cfg["h1_rate_ratio"]
    endpoints = yaml.safe_load((REPO_ROOT / "preregistration" / "endpoints.yaml")
                               .read_text(encoding="utf-8"))
    comparators = list(endpoints["H1"]["comparators"])
    if sorted(p["classical"] + p["learned"]) != sorted(comparators):
        raise SystemExit("the config's comparators are not H1's")
    family = endpoints["confirmatory_family"]
    k_family = len(family["members"]) + len(family.get("conditional_members", []))
    level = float(endpoints["alpha"]) / k_family
    nominal = level / 2.0
    candidates = p["candidates"]
    max_seeds = max(int(c["seeds"]) for c in candidates.values())
    strata = len(cfg["world"]["aisle_strata"])
    raw = (REPO_ROOT / p["route_lengths_from"]).read_bytes()
    lengths = [float(s["path_length_m"]) for s in json.loads(raw)["scenarios"]]
    kinds = {n: "classical" for n in p["classical"]} | {n: "learned" for n in p["learned"]}

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "power", f"H1_RR_{stamp}", args.seed, cfg)
    rng = np.random.default_rng(args.seed)

    # ---- 1. calibration -------------------------------------------------------------------
    cal_reps = int(p["calibration"]["reps"])
    tol = nominal + p["calibration"]["tolerance_mc_se"] * np.sqrt(nominal * (1 - nominal) / cal_reps)
    cal_rows, excluded = [], {name: [] for name in candidates}
    cal_seed_cvs = list(p["seed_cv"]) + list(p["calibration"].get("extra_seed_cv", []))
    cal_cells = [(r, cv, scv, size) for r in p["comparator_rates_per_100m"]
                 for cv in p["difficulty_cv"] for scv in cal_seed_cvs for size in p["sizes_per_cell"]]
    for i, (rate, cv, scv, size) in enumerate(cal_cells, 1):
        l5, comps = arms_for("equal_rates", float(rate), p)
        hits = {(c, a.name, side): 0 for c in candidates for a in comps for side in (0, 1)}
        est_sum = {"classical": 0.0, "learned": 0.0}
        stops_per_run = 0.0
        for d, counts in simulate(rng, lengths, int(size) * strata, l5, comps, float(cv),
                                  float(scv), max_seeds, cal_reps):
            got, terms = verdicts(d, counts, candidates, comps, level)
            for c, per in got.items():
                for a, (fav, against) in per.items():
                    hits[(c, a, 0)] += int(fav.sum())
                    hits[(c, a, 1)] += int(against.sum())
            t5 = terms[int(candidates["T2"]["seeds"])]
            for a in comps:
                est_sum[kinds[a.name]] += float(log_rate_ratio_terms(t5["L5"], t5[a.name]).sum())
            stops_per_run += float(counts["B1"].sum())
        mean_stops = stops_per_run / cal_reps
        k5 = int(candidates["T2"]["seeds"])
        row = {"rate": rate, "difficulty_cv": cv, "seed_cv": scv, "size_per_cell": size,
               "mean_stops_per_run": mean_stops,
               "predicted_continuity_bias_vs_classical": continuity_bias(mean_stops, k5, 1)
               if mean_stops > 0 else None}
        for kind in ("classical", "learned"):
            members = [a.name for a in comps if kinds[a.name] == kind]
            row[f"mean_log_rr_vs_{kind}"] = est_sum[kind] / (cal_reps * len(members))
            for c in candidates:
                for side, label in ((0, "favour"), (1, "against")):
                    rate_hat = np.mean([hits[(c, a, side)] for a in members]) / cal_reps
                    row[f"{c}_{kind}_{label}"] = rate_hat
                    if rate_hat > tol:
                        excluded[c].append({"cell": [rate, cv, scv, size], "kind": kind,
                                            "side": label, "rate": rate_hat})
        cal_rows.append(row)
        print(f"[calibration {i}/{len(cal_cells)}] rate {rate}, cv {cv}, seed cv {scv}, "
              f"{size}/cell: " + "; ".join(
                  f"{c} cl {row[f'{c}_classical_favour']:.4f}/{row[f'{c}_classical_against']:.4f}"
                  f" le {row[f'{c}_learned_favour']:.4f}/{row[f'{c}_learned_against']:.4f}"
                  for c in candidates))
    write_csv(run_dir / "calibration.csv", cal_rows)

    # ---- 2. the grid (normal approximation) ------------------------------------------------
    grid_rows = []
    cells = [(s, r, cv, scv, size) for s in SCENARIOS for r in p["comparator_rates_per_100m"]
             for cv in p["difficulty_cv"] for scv in p["seed_cv"] for size in p["sizes_per_cell"]]
    for i, (scenario, rate, cv, scv, size) in enumerate(cells, 1):
        l5, comps = arms_for(scenario, float(rate), p)
        allseven = {c: 0 for c in candidates}
        marg = {(c, a.name): 0 for c in candidates for a in comps}
        for d, counts in simulate(rng, lengths, int(size) * strata, l5, comps, float(cv),
                                  float(scv), max_seeds, int(p["reps"])):
            got, _ = verdicts(d, counts, candidates, comps, level)
            for c, per in got.items():
                ok = np.ones(d.shape[0], dtype=bool)
                for a, (fav, _against) in per.items():
                    marg[(c, a)] += int(fav.sum())
                    ok &= fav
                allseven[c] += int(ok.sum())
        for c in candidates:
            row = {"scenario": scenario, "rate": rate, "difficulty_cv": cv, "seed_cv": scv,
                   "size_per_cell": size, "pairs": int(size) * strata, "candidate": c,
                   "power_all_seven": allseven[c] / p["reps"]}
            row.update({f"reject_{a}": marg[(c, a)] / p["reps"] for a in comparators})
            grid_rows.append(row)
        print(f"[grid {i}/{len(cells)}] {scenario}, rate {rate}, cv {cv}, seed cv {scv}, "
              f"{size}/cell: " + ", ".join(f"{c} {allseven[c] / p['reps']:.3f}" for c in candidates))
    write_csv(run_dir / "grid.csv", grid_rows)

    # ---- 3. the full bootstrap at selected cells ------------------------------------------
    check_rows = []
    bc = p["bootstrap_check"]
    for scenario, rate, cv, scv, size in bc["cells"]:
        l5, comps = arms_for(scenario, float(rate), p)
        tally = {c: {"normal": 0, "bootstrap": 0, "agree": 0, "all_n": 0, "all_b": 0}
                 for c in candidates}
        for d, counts in simulate(rng, lengths, int(size) * strata, l5, comps, float(cv),
                                  float(scv), max_seeds, int(bc["reps"])):
            got, _ = verdicts(d, counts, candidates, comps, level)
            for r in range(d.shape[0]):
                for c, spec in candidates.items():
                    k = int(spec["seeds"])
                    boot = bootstrap_log_rr(rng, {a: s[r, :k, :] for a, s in counts.items()},
                                            d[r], spec["mode"], int(bc["draws"]))
                    all_n = all_b = True
                    for a in comps:
                        fav_b, _ = percentile_verdict(boot["L5"] - boot[a.name], level)
                        fav_n = bool(got[c][a.name][0][r])
                        tally[c]["normal"] += fav_n
                        tally[c]["bootstrap"] += fav_b
                        tally[c]["agree"] += fav_n == fav_b
                        all_n &= fav_n
                        all_b &= fav_b
                    tally[c]["all_n"] += all_n
                    tally[c]["all_b"] += all_b
        n_cmp = int(bc["reps"]) * len(comps)
        for c, t in tally.items():
            check_rows.append({"scenario": scenario, "rate": rate, "difficulty_cv": cv,
                               "seed_cv": scv, "size_per_cell": size, "candidate": c,
                               "reject_rate_normal": t["normal"] / n_cmp,
                               "reject_rate_bootstrap": t["bootstrap"] / n_cmp,
                               "agreement": t["agree"] / n_cmp,
                               "power_all_seven_normal": t["all_n"] / bc["reps"],
                               "power_all_seven_bootstrap": t["all_b"] / bc["reps"]})
            print(f"[bootstrap check] {scenario}, rate {rate}, cv {cv}, seed cv {scv}, {size}/cell,"
                  f" {c}: per-comparison normal {t['normal'] / n_cmp:.3f} bootstrap "
                  f"{t['bootstrap'] / n_cmp:.3f} agree {t['agree'] / n_cmp:.3f}; all seven "
                  f"{t['all_n'] / bc['reps']:.3f} vs {t['all_b'] / bc['reps']:.3f}")
    write_csv(run_dir / "bootstrap_check.csv", check_rows)

    # ---- decision ------------------------------------------------------------------------
    dec = p["decision"]

    def power_by_size(c):
        return {int(r["size_per_cell"]): r["power_all_seven"] for r in grid_rows
                if r["candidate"] == c and r["scenario"] == dec["scenario"]
                and float(r["rate"]) == float(dec["comparator_rate_per_100m"])
                and float(r["difficulty_cv"]) == float(dec["difficulty_cv"])
                and float(r["seed_cv"]) == float(dec["seed_cv"])}

    at_point = {c: power_by_size(c) for c in candidates}
    verdict, chosen = None, None
    for c in dec["order"]:
        if excluded[c]:
            continue
        size = smallest_sufficient(at_point[c], dec["min_power"])
        if size is not None:
            chosen = {"candidate": c, "size_per_cell": size}
            verdict = (f"{c} at {size} per cell" + ("" if c == dec["order"][0] else
                       " -- five more seeds per learned arm in every confirmatory comparison, "
                       "subject to G2"))
            break
    if chosen is None:
        verdict = "STOP AND REPORT: no admissible candidate reaches the power at <= 100 per cell."
    t1_policy_null = [{k: v for k, v in r.items() if k.startswith("T1_") or k in
                       ("rate", "difficulty_cv", "size_per_cell")}
                      for r in cal_rows if float(r["seed_cv"]) == 0.0]
    summary = {"level": level, "nominal_per_side": nominal, "calibration_tolerance": tol,
               "t1_at_its_own_null_seed_cv_0": t1_policy_null,
               "excluded": {c: bool(v) for c, v in excluded.items()},
               "exclusions": excluded, "decision_rule": dec, "power_at_decision_point": at_point,
               "chosen": chosen, "verdict": verdict, "strata_pooled": strata,
               "route_lengths": {"path": p["route_lengths_from"], "n": len(lengths),
                                 "sha256": hashlib.sha256(raw).hexdigest()}}
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=float),
                                          encoding="utf-8")
    print("excluded:", {c: bool(v) for c, v in excluded.items()})
    for c, pw in at_point.items():
        print(f"  {c} at the decision point: " + ", ".join(f"{s}/cell {v:.3f}"
                                                          for s, v in pw.items()))
    print(verdict)
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    raise SystemExit(main())
