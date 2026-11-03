"""B7's timeouts at the frozen alpha, classified against the oracle (ruled 2026-11-03). Read-only.

    python scripts/classify_b7_timeouts.py --config configs/base.yaml --seed 0 --tuning-run runs/tuning/B7_alpha_20261102T195759Z_seed0 --oracle-run runs/eval/oracle_tracker_privileged_<stamp>_seed0

Reads the alpha tuning run's `summary.json` (the selected alpha) and that candidate's
`episodes.jsonl`, and the privileged oracle tracker's evaluation of the same tune scenarios (G4's
positive control, `episodes.jsonl`). Classes and their rule: `pfbnav/eval/timeout_classes.py`.
Prints counts per stratum with route IDs and one line per timeout, and writes
`runs/diag/B7TIMEOUTS_<timestamp>_seed<seed>/summary.json`. Changes nothing in the recovery or the
filter: the supervisor decides after the classification.
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.eval.timeout_classes import class_counts, classify_timeouts
from pfbnav.utils.runio import create_run_dir

REPO_ROOT = Path(__file__).resolve().parents[1]


def _episodes(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--tuning-run", required=True)
    parser.add_argument("--oracle-run", required=True)
    args = parser.parse_args()
    cfg = load_config(args.config)
    tuning = REPO_ROOT / args.tuning_run
    summary = json.loads((tuning / "summary.json").read_text(encoding="utf-8"))
    index = [float(a) for a in summary["grid_per_s"]].index(float(summary["selected_alpha"]))
    b7 = _episodes(tuning / f"candidate_{index}" / "episodes.jsonl")
    oracle_dir = REPO_ROOT / args.oracle_run
    evaluated = json.loads((oracle_dir / "evaluated.json").read_text(encoding="utf-8"))
    oracle = _episodes(oracle_dir / "episodes.jsonl")
    rows = classify_timeouts(b7, oracle, dt=float(cfg["sim"]["dt"]))
    counts = class_counts(rows)
    print(f"B7 at alpha {summary['selected_alpha']} (candidate_{index}): {len(b7)} episodes, "
          f"{len(rows)} timeouts; oracle: {args.oracle_run}")
    for stratum, c in counts.items():
        print(f"  {stratum}: " + "; ".join(f"{k} {v['count']} {v['ids']}" for k, v in c.items()))
    print("per timeout: seed stratum class | oracle | unmapped (nearest offset m) | travelled/plan m "
          "| mean speed m/s | PFB-modified share | stops")
    for r in rows:
        off = "-" if r["nearest_unmapped_offset_m"] is None else f"{r['nearest_unmapped_offset_m']:.3f}"
        print(f"  {r['scenario_seed']} {r['stratum']:<6} {r['class']:<15} | {r['oracle_outcome']:<18} "
              f"| {r['unmapped_count']} ({off}) | {r['distance_m']:.1f}/{r['plan_length_m']:.1f} "
              f"| {r['mean_speed_mps']:.3f} | {r['pfb_modified_share']:.2f} | {r['protective_stops']}")
    out_dir = create_run_dir(REPO_ROOT / "runs" / "diag",
                             f"B7TIMEOUTS_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}", args.seed, cfg)
    (out_dir / "summary.json").write_text(json.dumps({
        "tuning_run": args.tuning_run, "selected_alpha": summary["selected_alpha"],
        "oracle_run": args.oracle_run, "oracle_evaluated": evaluated,
        "counts": counts, "timeouts": rows}, indent=2), encoding="utf-8")
    print(f"-> {out_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
