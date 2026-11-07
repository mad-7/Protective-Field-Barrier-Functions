"""HF-c's verdict (A36) and the exploratory H1prime from the sealed classical test evaluation. Read-only,
seconds, no simulator. The RESEARCHER runs it once every shard has finished:

    python scripts/analyse_hfc.py --config configs/base.yaml --seed 0 \
        --calibration runs/power/HFC_CAL_<stamp>_seed0/summary.json \
        --runs runs/test/CLASSICAL_shard0of4_<stamp>_seed0 runs/test/CLASSICAL_shard1of4_<stamp>_seed0 \
               runs/test/CLASSICAL_shard2of4_<stamp>_seed0 runs/test/CLASSICAL_shard3of4_<stamp>_seed0

**Checks first, verdict after:** the calibration gate passed (A36: otherwise HF-c is not tested); every
shard's provenance is clean, on one commit and one host, with the same sets, environment and alpha; the
shards are 0..n-1 of one n, each once (a rerun after a crash replaces its crashed attempt, which is
named); every arm has exactly one episode on each of the 150 unmapped_static scenarios, and the arms agree
on every planned length (`pfbnav/eval/hfc.py`). Any failure stops before a number is printed.

**Reported** (`runs/test/HFC_ANALYSIS_<stamp>_seed<seed>/report.json`):
  * **HF-c (confirmatory, alpha 0.05):** B7 over B2, the rate ratio per 100 m planned with its 95 %
    interval (scenario part, t on n - 1 df); verdict "B7 fewer" only if the interval lies below 1.
  * **H1prime (exploratory; no pivot occurred):** B7 over B1, B2 and B6 at 0.05, intersection-union.
  * **Beside it, descriptive (A34):** per arm and per stratum, success, contact rate, stops per 100 m
    planned / driven / on completed routes, latched share, moving speed, mean mission time of successes,
    scan-fault stops, and the PFB's correction measures for B7.
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from pfbnav.config import load_config
from pfbnav.eval.hfc import ARMS, classical_comparison, paired_table
from pfbnav.eval.runner import outcome_summary
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
H1PRIME = ("B1", "B2", "B6")


def read_runs(dirs: list[Path]) -> tuple[dict[str, list[dict[str, Any]]], dict[str, Any]]:
    """Every arm's episodes over all shards, after the integrity checks; and what was checked."""
    evals, provs = [], []
    for d in dirs:
        evals.append(json.loads((d / "evaluated.json").read_text(encoding="utf-8")))
        provs.append(json.loads((d / "provenance.json").read_text(encoding="utf-8")))
    if any(p.get("dirty") is not False for p in provs):
        raise SystemExit("a shard ran on a dirty tree")
    for key in ("commit",):
        if len({p.get(key) for p in provs}) != 1:
            raise SystemExit(f"the shards differ in {key}")
    if len({(p.get("host") or {}).get("hostname") for p in provs}) != 1:
        raise SystemExit("the shards ran on different hosts")
    for key in ("sets", "environment", "alpha", "b2_parameters", "arms", "shards"):
        if len({json.dumps(e[key], sort_keys=True) for e in evals}) != 1:
            raise SystemExit(f"the shards differ in {key}")
    n = int(evals[0]["shards"])
    if sorted(int(e["shard"]) for e in evals) != list(range(n)):
        raise SystemExit(f"the runs are not shards 0..{n - 1} of {n}, each once")
    positions = sorted(p for e in evals for p in e["positions"])
    if positions != list(range(len(positions))):
        raise SystemExit("the shards' scenario positions do not cover the sets exactly once")
    records: dict[str, list[dict[str, Any]]] = {a: [] for a in ARMS}
    for d in dirs:
        for a in ARMS:
            path = d / a / "episodes.jsonl"
            if not path.is_file():
                raise SystemExit(f"{path} is missing")
            records[a] += [json.loads(x) for x in path.read_text(encoding="utf-8").splitlines() if x]
    for a, recs in records.items():
        if len(recs) != len(positions):
            raise SystemExit(f"{a} has {len(recs)} episodes, not {len(positions)} (a truncated shard?)")
    checked = {"commit": provs[0]["commit"], "host": (provs[0].get("host") or {}).get("hostname"),
               "shards": n, "scenarios": len(positions),
               "reruns_after_crash": [e["rerun_after_crash"] for e in evals if e.get("rerun_after_crash")],
               "sets": evals[0]["sets"], "environment": evals[0]["environment"],
               "alpha": evals[0]["alpha"]}
    return records, checked


def describe(recs: list[dict[str, Any]], dt: float) -> dict[str, Any]:
    o = outcome_summary(recs, dt)
    times = [int(r.get("episode_steps", r.get("steps", 0))) * dt for r in recs
             if r["outcome"] == "success"]
    keep = ("episodes", "success_rate", "contact_rate", "counts", "protective_stops",
            "scan_fault_stops", "stops_per_100m_planned", "stops_per_100m", "stops_per_100m_completed",
            "latched_step_share", "moving_speed_mps", "pfb_infeasible", "pfb_measures")
    out = {k: o.get(k) for k in keep}
    out["mission_time_success_mean_s"] = sum(times) / len(times) if times else None
    return out


NAMES = {"B1": "B1 Pure Pursuit", "B2": "B2 RPP", "B3": "B3 DWA", "B6": "B6 MPPI (field-aware)",
         "B7": "B7 RPP + PFB"}


def _f(x, nd: int = 2) -> str:
    return "--" if x is None else f"{x:.{nd}f}"


def wording(v: dict[str, Any], a: str = "B7", b: str = "") -> str:
    """The sentence the paper may use for one comparison (supervisor, 2026-11-07: an inconclusive
    comparison is 'indistinguishable at x vs y events', never 'matches')."""
    if v["favours_a"]:
        return (f"{a} triggers fewer protective stops than {b} (rate ratio {v['rate_ratio']:.3f}, "
                f"95 % CI {v['ci'][0]:.3f}-{v['ci'][1]:.3f}; {v['stops_a']} vs {v['stops_b']} stops)")
    if v["favours_b"]:
        return (f"{a} triggers more protective stops than {b} (rate ratio {v['rate_ratio']:.3f}, "
                f"95 % CI {v['ci'][0]:.3f}-{v['ci'][1]:.3f})")
    return (f"{a} and {b} are indistinguishable at {v['stops_a']} vs {v['stops_b']} events "
            f"(rate ratio {v['rate_ratio']:.2f}, 95 % CI {v['ci'][0]:.2f}-{v['ci'][1]:.2f})")


def paper_numbers(report: dict[str, Any]) -> dict[str, Any]:
    """What the CoDIT draft cites: HELD-OUT test results, labelled as such."""
    d = report["descriptive"]
    return {
        "label": ("held-out test set (unmapped_static, 150 scenarios, 50 per stratum); HF-c "
                  "confirmatory (A36, alpha 0.05); H1prime exploratory"),
        "commit": report["checked"]["commit"],
        "HFc_sentence": wording(report["HFc"], "B7", "B2"),
        "H1prime_sentences": {c: wording(v, "B7", c)
                              for c, v in report["H1prime_exploratory"]["comparisons"].items()},
        "arms": {a: {k: d[a]["pooled"][k] for k in (
            "success_rate", "contact_rate", "protective_stops", "stops_per_100m_planned",
            "stops_per_100m", "stops_per_100m_completed", "latched_step_share", "moving_speed_mps",
            "mission_time_success_mean_s")} for a in d},
    }


def paper_table(report: dict[str, Any]) -> str:
    """A booktabs body for the CoDIT results table (held-out test)."""
    rows = [r"% held-out test set, unmapped_static, 150 scenarios; scripts/analyse_hfc.py",
            r"\begin{tabular}{lrrrrrr}", r"\toprule",
            r"Planner & Success & Stops & Stops/100\,m & Latched & Moving & Mission \\",
            r" & & & (planned) & (\%) & (m/s) & time (s) \\", r"\midrule"]
    for a, name in NAMES.items():
        p = report["descriptive"][a]["pooled"]
        latched = None if p["latched_step_share"] is None else 100 * p["latched_step_share"]
        rows.append(f"{name} & {_f(p['success_rate'])} & {p['protective_stops']} & "
                    f"{_f(p['stops_per_100m_planned'])} & {_f(latched, 1)} & "
                    f"{_f(p['moving_speed_mps'], 3)} & {_f(p['mission_time_success_mean_s'], 1)} \\\\")
    h = report["HFc"]
    rows += [r"\bottomrule", r"\end{tabular}",
             (f"% HF-c: B7/B2 rate ratio {h['rate_ratio']:.4f}, 95 % CI [{h['ci'][0]:.4f}, "
              f"{h['ci'][1]:.4f}], {h['result']}")]
    return "\n".join(rows) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--calibration", required=True, help="scripts/calibrate_hfc.py summary.json")
    parser.add_argument("--runs", nargs="+", required=True, help="every CLASSICAL_shard* run dir")
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    endpoints = yaml.safe_load((REPO_ROOT / "preregistration" / "endpoints.yaml")
                               .read_text(encoding="utf-8"))
    if "HFc" not in endpoints:
        raise SystemExit("HF-c is not preregistered (A36)")
    alpha = float(endpoints["filter_efficacy_family"]["alpha"])
    cal = json.loads(Path(args.calibration).read_text(encoding="utf-8"))
    if cal.get("gate_pass") is not True or float(cal.get("level", -1)) != alpha:
        raise SystemExit("the HF-c calibration gate did not pass at HF-c's level: stop and report; "
                         "HF-c is not tested (A36)")
    records, checked = read_runs([Path(d) for d in args.runs])
    table = paired_table(records)
    dt = float(cfg["sim"]["dt"])

    hfc = classical_comparison(table["stops"]["B7"], table["stops"]["B2"], table["planned_m"], alpha)
    h1p = {c: classical_comparison(table["stops"]["B7"], table["stops"][c], table["planned_m"], alpha)
           for c in H1PRIME}
    strata = sorted({s for s in table["stratum"] if s})
    report = {
        "checked": checked, "calibration": Path(args.calibration).as_posix(),
        "HFc": {"comparison": "B7 over B2", "alpha": alpha, **hfc,
                "result": "B7 fewer protective stops than B2" if hfc["favours_a"]
                else "B7 more than B2" if hfc["favours_b"] else "inconclusive"},
        "H1prime_exploratory": {"comparisons": h1p,
                                "all_reject": all(v["favours_a"] for v in h1p.values())},
        "descriptive": {a: {"pooled": describe(r, dt),
                            "per_stratum": {s: describe([x for x in r if x.get("stratum") == s], dt)
                                            for s in strata}}
                        for a, r in records.items()},
    }
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "test", f"HFC_ANALYSIS_{stamp}", args.seed, cfg)
    (run_dir / "report.json").write_text(json.dumps(report, indent=2, default=float),
                                         encoding="utf-8")
    (run_dir / "codit_table.tex").write_text(paper_table(report), encoding="utf-8")
    (run_dir / "codit_numbers.json").write_text(json.dumps(paper_numbers(report), indent=2,
                                                           default=float), encoding="utf-8")
    print(f"checked: {checked['scenarios']} scenarios x {len(ARMS)} arms, {checked['shards']} shards, "
          f"commit {checked['commit'][:7]}, reruns {checked['reruns_after_crash']}")
    print(f"HF-c (alpha {alpha}): B7 {hfc['stops_a']} stops ({hfc['rate_a_per_100m']:.3f}/100 m planned) "
          f"vs B2 {hfc['stops_b']} ({hfc['rate_b_per_100m']:.3f}); rate ratio {hfc['rate_ratio']:.4f}, "
          f"95 % CI [{hfc['ci'][0]:.4f}, {hfc['ci'][1]:.4f}] -> {report['HFc']['result']}")
    for c, v in h1p.items():
        print(f"H1prime (exploratory) B7 vs {c}: ratio {v['rate_ratio']:.4f} "
              f"[{v['ci'][0]:.4f}, {v['ci'][1]:.4f}] {v['verdict']}")
    for a in ARMS:
        d = report["descriptive"][a]["pooled"]
        print(f"  {a}: success {d['success_rate']:.3f}, contact {d['contact_rate']:.3f}, "
              f"stops/100 m planned {d['stops_per_100m_planned']}, latched {d['latched_step_share']}")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
