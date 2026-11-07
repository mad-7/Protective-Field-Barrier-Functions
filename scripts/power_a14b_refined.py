"""A14b, refined (ruled 2026-11-07, item 2): power per hypothesis, the expected 95 % CI half-width of
L5 vs B6 and the evaluation wall-clock, per test size and seed count K, before K and the test size are
decided together (item 3). The RESEARCHER runs it (CPU, no simulator; about an hour):

    python scripts/power_a14b_refined.py --config configs/analysis/power_a14b_refined.yaml --seed 0

Against the first A14b grid (`scripts/power_a14b.py`, every comparator at one rate):
  * H1's comparators at their MEASURED tune-set rates per 100 m planned (B1, B2, B3), B6 on a BAND,
    the learned comparators L1-L3 at B6's rate (no tune-set rate exists for them; the conservative
    choice, declared), L5 at a rate ratio x B6 (0.5, 0.7, 1.0 = the null, 2.0 = L5 worse);
  * H1prime (B7 vs B1, B2, B6; every arm runs once: the scenario part only, t with n - 1 df), B7 at
    the same ratios x B6, at the pivot's level alpha; its ratio-1.0 row is the A14a calibration that
    must pass before H1prime is tested (endpoints.yaml);
  * H2 (L5 vs L4, narrow-stratum success, both conditions pooled; difference >= +15 pp AND
    significant), on the declared baseline, effect and seed-SD grids; effect 0.0 gives its size;
  * K in {10, 15, 20}: one simulation of 20 seeds per learned arm, the first K used (nested, as
    power_corrected.py), so the K comparison is paired;
  * the L5-vs-B6 comparison reported three ways (favours / against / inconclusive: ruling 3's
    decision-tree outcome), with the mean 95 % CI half-width of its log rate ratio;
  * sizes are scenarios PER CELL (stratum x condition). The test set holds 3 strata x 2 conditions
    (mapped_only, unmapped_static; gates.yaml), so a size s means 6 s scenarios evaluated, H1 sees the
    3 s unmapped_static scenarios, H2 the 2 s narrow ones. The first grid's wall-clock counted 3 s:
    it under-stated the evaluation by half (reported, decision log 2026-11-07).

Writes `runs/power/A14BR_<stamp>_seed<seed>/`: `h1.csv`, `h1prime.csv`, `h2.csv`, `summary.json`
(per K and size: the wall-clock, and per cell the smallest size reaching 80 % power) and provenance.
"""
from __future__ import annotations

import argparse
import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pfbnav.config import load_config
from pfbnav.eval.power import smallest_sufficient
from pfbnav.eval.power_grid import (
    h1_arms_measured,
    h1_chunks,
    h1prime_arms,
    success_outcomes,
    three_way,
)
from pfbnav.eval.rate_ratio import (
    FLAT,
    WS,
    arm_terms,
    components,
    corrected_interval,
    corrected_verdict,
    log_rate_ratio_terms,
    mean_difference,
    mean_terms,
)
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]


def wall_clock_h(n_scenarios: int, k: int, p: dict[str, Any]) -> float:
    """Hours for the test evaluation of `n_scenarios`: classical arms once, learned arms K times,
    each episode at its declared cost, over `instances` simulator instances."""
    cost = p["episode_wall_s"]
    per = (sum(float(cost[a]) for a in p["classical_arms_evaluated"])
           + k * sum(float(cost[a]) for a in p["learned_arms_evaluated"]))
    return n_scenarios * per / int(p["instances"]) / 3600.0


def _write(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    args = parser.parse_args()
    full = load_config(args.config)
    seed_everything(args.seed)
    p = full["power_a14b_refined"]
    endpoints = yaml.safe_load((REPO_ROOT / "preregistration" / "endpoints.yaml")
                               .read_text(encoding="utf-8"))
    if sorted(p["classical"] + p["learned"]) != sorted(endpoints["H1"]["comparators"]):
        raise SystemExit("the config's comparators are not H1's")
    if sorted(p["h1prime_comparators"]) != sorted(endpoints["H1prime"]["comparators"]):
        raise SystemExit("the config's H1prime comparators are not H1prime's")
    alpha = float(endpoints["alpha"])
    family = endpoints["confirmatory_family"]
    level = alpha / (len(family["members"]) + len(family.get("conditional_members", [])))
    ks = [int(k) for k in p["seed_counts"]]
    max_k = max(ks)
    strata, conditions = len(full["world"]["aisle_strata"]), int(p["conditions_evaluated"])
    sizes = [int(s) for s in p["sizes_per_cell"]]
    cap = int(p["max_scenarios"])
    if any(s * strata * conditions > cap for s in sizes):
        raise SystemExit(f"a size exceeds the ruled cap of {cap} scenarios (ruling 3)")
    lengths = [float(s["path_length_m"]) for s in json.loads(
        (REPO_ROOT / p["route_lengths_from"]).read_text(encoding="utf-8"))["scenarios"]]
    reps, cv, min_power = int(p["reps"]), float(p["difficulty_cv"]), float(p["min_power"])

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "power", f"A14BR_{stamp}", args.seed, full)
    rng = np.random.default_rng(args.seed)
    summary: dict[str, Any] = {"level_h1_h2": level, "level_h1prime": alpha, "seed_counts": ks,
                               "scenarios_per_size": strata * conditions, "h1": {}, "h1prime": {},
                               "h2": {}}

    # ---- H1 and the L5-vs-B6 interval -------------------------------------------------------
    h1_rows: list[dict[str, Any]] = []
    for b6 in p["b6_rates_per_100m"]:
        for ratio in p["rate_ratios"]:
            for scv in p["seed_cv"]:
                power: dict[int, dict[int, float]] = {k: {} for k in ks}
                for size in sizes:
                    l5, comps = h1_arms_measured(float(b6), float(ratio), p)
                    acc = {k: {"all": 0, "lo": [], "hi": [], "half": 0.0, "est": 0.0} for k in ks}
                    for d, counts in h1_chunks(rng, lengths, size * strata, l5, comps, cv,
                                               float(scv), max_k, reps):
                        for k in ks:
                            t = {a: arm_terms(s[:, :k, :], d) for a, s in counts.items()}
                            ok = np.ones(d.shape[0], dtype=bool)
                            for c in comps:
                                low, high = corrected_verdict(t["L5"], t[c.name], level,
                                                              classical_df=FLAT)
                                ok &= low
                                if c.name == "B6":
                                    acc[k]["lo"].append(low)
                                    acc[k]["hi"].append(high)
                            _, half = corrected_interval(components(t["L5"], t["B6"]), 0.05, FLAT)
                            acc[k]["all"] += int(ok.sum())
                            acc[k]["half"] += float(half.sum())
                            acc[k]["est"] += float(log_rate_ratio_terms(t["L5"], t["B6"]).sum())
                    for k in ks:
                        a = acc[k]
                        b6_way = three_way(np.concatenate(a["lo"]), np.concatenate(a["hi"]))
                        power[k][size] = a["all"] / reps
                        h1_rows.append({
                            "b6_rate": b6, "ratio": ratio, "seed_cv": scv, "K": k,
                            "size_per_cell": size, "scenarios": size * strata * conditions,
                            "h1_pairs": size * strata, "power_h1_all_seven": a["all"] / reps,
                            "l5_vs_b6_favours": b6_way["favours"],
                            "l5_vs_b6_against": b6_way["against"],
                            "l5_vs_b6_inconclusive": b6_way["inconclusive"],
                            "l5_vs_b6_ci95_half_width_log": a["half"] / reps,
                            "l5_vs_b6_mean_log_rr": a["est"] / reps})
                        print(f"H1 b6 {b6}, ratio {ratio}, seed cv {scv}, K {k}, {size}/cell: "
                              f"{a['all'] / reps:.3f}, B6 {b6_way}, half "
                              f"{a['half'] / reps:.3f}", flush=True)
                key = f"b6={b6},ratio={ratio},seed_cv={scv}"
                summary["h1"][key] = {
                    f"K{k}": {"power": power[k],
                              "smallest_size_per_cell_for_80":
                                  smallest_sufficient(power[k], min_power) if ratio < 1.0 else None}
                    for k in ks}
    _write(run_dir / "h1.csv", h1_rows)

    # ---- H1prime (classical only: no K, no seed CV) -----------------------------------------
    hp_rows: list[dict[str, Any]] = []
    for b6 in p["b6_rates_per_100m"]:
        for ratio in p["rate_ratios"]:
            by_size: dict[int, float] = {}
            for size in sizes:
                b7, comps = h1prime_arms(float(b6), float(ratio), p)
                hits, lo, hi = 0, [], []
                for d, counts in h1_chunks(rng, lengths, size * strata, b7, comps, cv, 0.0, 1,
                                           reps):
                    t = {a: arm_terms(s, d) for a, s in counts.items()}
                    ok = np.ones(d.shape[0], dtype=bool)
                    for c in comps:
                        low, high = corrected_verdict(t["B7"], t[c.name], alpha, classical_df=WS)
                        ok &= low
                        if c.name == "B6":
                            lo.append(low)
                            hi.append(high)
                    hits += int(ok.sum())
                by_size[size] = hits / reps
                way = three_way(np.concatenate(lo), np.concatenate(hi))
                hp_rows.append({"b6_rate": b6, "ratio": ratio, "size_per_cell": size,
                                "scenarios": size * strata * conditions,
                                "power_h1prime_all_three": hits / reps,
                                "b7_vs_b6_favours": way["favours"],
                                "b7_vs_b6_against": way["against"],
                                "b7_vs_b6_inconclusive": way["inconclusive"]})
                print(f"H1prime b6 {b6}, ratio {ratio}, {size}/cell: {hits / reps:.3f}, B6 {way}",
                      flush=True)
            summary["h1prime"][f"b6={b6},ratio={ratio}"] = {
                "power": by_size,
                "smallest_size_per_cell_for_80":
                    smallest_sufficient(by_size, min_power) if ratio < 1.0 else None}
    _write(run_dir / "h1prime.csv", hp_rows)

    # ---- H2 -------------------------------------------------------------------------------
    h2 = p["h2"]
    h2_rows: list[dict[str, Any]] = []
    for lo_b, hi_b in h2["l4_success_ranges"]:
        for effect in h2["effects"]:
            for sd in h2["seed_logit_sd"]:
                power = {k: {} for k in ks}
                for size in sizes:
                    n_pairs = size * int(h2["cells_pooled"])
                    tally = {k: 0 for k in ks}
                    done = 0
                    while done < int(h2["reps"]):
                        r = min(500, int(h2["reps"]) - done)
                        base = rng.uniform(float(lo_b), float(hi_b), size=(r, n_pairs))
                        l4 = success_outcomes(rng, base, 0.0, float(sd), max_k)
                        l5 = success_outcomes(rng, base, float(effect), float(sd), max_k)
                        for k in ks:
                            a, b = mean_terms(l5[:, :k, :]), mean_terms(l4[:, :k, :])
                            est = mean_difference(a, b)
                            _, higher = corrected_verdict(a, b, level, estimate=est)
                            tally[k] += int((higher if effect == 0.0
                                             else higher & (est >= float(h2["min_difference"])))
                                            .sum())
                        done += r
                    for k in ks:
                        power[k][size] = tally[k] / int(h2["reps"])
                        h2_rows.append({"l4_base": f"{lo_b}-{hi_b}", "effect": effect,
                                        "seed_logit_sd": sd, "K": k, "size_per_cell": size,
                                        "scenarios": size * strata * conditions,
                                        "h2_pairs": n_pairs, "under_null": effect == 0.0,
                                        # power, or the rejection rate (size) at effect 0.0
                                        "h2_success_share": power[k][size]})
                    print(f"H2 base {lo_b}-{hi_b}, effect {effect}, sd {sd}, {size}/cell: "
                          f"{[power[k][size] for k in ks]}", flush=True)
                summary["h2"][f"base={lo_b}-{hi_b},effect={effect},seed_logit_sd={sd}"] = {
                    f"K{k}": {"power": power[k],
                              "smallest_size_per_cell_for_80":
                                  smallest_sufficient(power[k], min_power) if effect > 0 else None}
                    for k in ks}
    _write(run_dir / "h2.csv", h2_rows)

    summary["wall_clock"] = {
        f"K{k}": {str(s): {"scenarios": s * strata * conditions,
                           "days": wall_clock_h(s * strata * conditions, k, p) / 24.0}
                  for s in sizes} for k in ks}
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    for k in ks:
        print(f"K {k}: " + ", ".join(f"{s * strata * conditions} scen. "
                                     f"{wall_clock_h(s * strata * conditions, k, p) / 24:.1f} d"
                                     for s in sizes))
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
