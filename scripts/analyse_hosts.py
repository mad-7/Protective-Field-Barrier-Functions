"""HF_hosts (A37): the PFB on DWA and on field-aware MPPI, from `scripts/evaluate_test.py --arm-set hosts`. Read-only, seconds,
no simulator. The RESEARCHER runs it once per half, when that half's shards have finished:

    python scripts/analyse_hosts.py --config configs/base.yaml --seed 0 --condition unmapped_static \
        --runs runs/test/HOSTS_unmapped_static_shard0of4_<stamp>_seed0 ... \
        --sealed runs/test/CLASSICAL_shard0of4_<stamp>_seed0 ...

Everything here is EXPLORATORY: these arms were proposed after HF-c was read (A37). Reports B9 vs B3 and B8 vs B6 per 100 m
planned (`pfbnav/eval/hfc.py`, scenario part only, t on n - 1 df), flags a low-count comparison (reported as counts, never as a
verdict), checks each host's rerun against its sealed result (flagged above 0.05 in success; the sealed result stays the cited
one), and gives each arm's descriptive row with its measured compute against the 50 ms criterion. The integrity checks are those
of `scripts/analyse_hfc.py`. Writes `runs/test/HOSTS_ANALYSIS_<condition>_<stamp>_seed<seed>/report.json`.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from pfbnav.config import load_config
from pfbnav.eval.hfc import classical_comparison, consistency, low_count, paired_table
from pfbnav.eval.runner import outcome_summary
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
ARMS = ("B3", "B9", "B6", "B8")
PAIRS = {"B9": "B3", "B8": "B6"}       # filtered -> its host
LEVEL = 0.05


def _analyse_hfc():
    spec = importlib.util.spec_from_file_location("analyse_hfc", REPO_ROOT / "scripts" / "analyse_hfc.py")
    if spec is None or spec.loader is None:
        raise SystemExit("cannot load scripts/analyse_hfc.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def comparisons(records: dict[str, list[dict[str, Any]]]) -> dict[str, dict[str, Any]]:
    """Each filtered arm over its host, with the declared low-count flag."""
    table = paired_table(records, ARMS)
    out = {}
    for arm, host in PAIRS.items():
        c = classical_comparison(table["stops"][arm], table["stops"][host], table["planned_m"], LEVEL)
        flag = low_count(c["stops_a"], c["stops_b"])
        out[f"{arm}_vs_{host}"] = {**c, "low_count": flag,
                                   "reading": f"counts only: {c['stops_a']} vs {c['stops_b']} events" if flag
                                   else c["verdict"]}
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--condition", choices=("unmapped_static", "mapped_only"), required=True)
    parser.add_argument("--runs", nargs="+", required=True, help="every HOSTS_<condition>_shard* run dir")
    parser.add_argument("--sealed", nargs="+", required=True, help="the same half's sealed CLASSICAL shards")
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    if "## A37 - " not in (REPO_ROOT / "preregistration" / "AMENDMENTS.md").read_text(encoding="utf-8"):
        raise SystemExit("A37 is not applied")
    hfc = _analyse_hfc()
    records, checked = hfc.read_runs([Path(d) for d in args.runs], ARMS)
    sealed, sealed_checked = hfc.read_runs([Path(d) for d in args.sealed], ("B3", "B6"))
    for name, c in (("runs", checked), ("sealed", sealed_checked)):
        if c["condition"] != args.condition:
            raise SystemExit(f"--{name} are {c['condition']}, not {args.condition}")
    if checked["sets"] != sealed_checked["sets"]:
        raise SystemExit("the reruns and the sealed runs did not use the same test sets")
    dt = float(cfg["sim"]["dt"])
    summaries = {a: outcome_summary(r, dt) for a, r in records.items()}
    compared = comparisons(records)
    report = {"status": "exploratory (A37): proposed after HF-c was read", "checked": checked,
              "sealed_commit": sealed_checked["commit"], "comparisons": compared,
              "consistency_with_sealed": {h: consistency(summaries[h], outcome_summary(sealed[h], dt))
                                          for h in ("B3", "B6")},
              "descriptive": {a: {"pooled": hfc.describe(r, dt), "compute": summaries[a]["compute"]}
                              for a, r in records.items()}}
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "test", f"HOSTS_ANALYSIS_{args.condition}_{stamp}", args.seed, cfg)
    (run_dir / "report.json").write_text(json.dumps(report, indent=2, default=float), encoding="utf-8")
    print(f"checked: {checked['scenarios']} scenarios x {len(ARMS)} arms, {checked['shards']} shards, "
          f"commit {checked['commit'][:7]}, reruns {checked['reruns_after_crash']}; EXPLORATORY (A37)")
    for name, c in compared.items():
        print(f"{name}: {c['stops_a']} vs {c['stops_b']} stops ({c['rate_a_per_100m']:.3f} vs {c['rate_b_per_100m']:.3f} "
              f"per 100 m planned); ratio {c['rate_ratio']:.4f} [{c['ci'][0]:.4f}, {c['ci'][1]:.4f}] -> {c['reading']}")
    for h, c in report["consistency_with_sealed"].items():
        print(f"{h} rerun vs sealed: success {c['success_rerun']:.3f} vs {c['success_sealed']:.3f}, stops "
              f"{c['stops_rerun']} vs {c['stops_sealed']}{'  FLAGGED: difference above 0.05' if c['flagged'] else ''}")
    for a in ARMS:
        d = report["descriptive"][a]
        p = d["pooled"]
        decision = d["compute"].get("decision", {}).get("p95_ms")
        print(f"  {a}: {p['counts']} success {p['success_rate']:.3f}, contact {p['contact_rate']:.3f}, stops/100 m planned "
              f"{p['stops_per_100m_planned']}, driven {p['stops_per_100m']}, completed {p['stops_per_100m_completed']}, "
              f"latched {p['latched_step_share']}, moving {p['moving_speed_mps']}, time {p['mission_time_success_mean_s']}, "
              f"infeasible {p['pfb_infeasible']}, decision p95 {decision} ms")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
