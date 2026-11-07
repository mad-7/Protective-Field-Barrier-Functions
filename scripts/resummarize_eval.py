"""Re-read a finished evaluation with the current summary (read-only, seconds): the metrics added after it
ran -- moving speed, latched share, the stop rate on completed routes (ruled 2026-11-06, A34) -- beside
the primary and the per-planned-length rate, overall and per stratum.

    python scripts/resummarize_eval.py --config configs/base.yaml --seed 0 --run runs/eval/<name>

Reads the run's `episodes.jsonl` only, through `pfbnav.eval.runner.outcome_summary` (tested). Writes
nothing; the run's own `summary.json` is never modified (hard rule 1).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.eval.runner import outcome_summary

REPO_ROOT = Path(__file__).resolve().parents[1]
KEYS = ("episodes", "success_rate", "stops_per_100m", "stops_per_100m_planned",
        "stops_per_100m_completed", "stops_per_100m_excluding_recovery", "distance_m",
        "planned_length_m", "completed_distance_m", "moving_speed_mps", "latched_step_share")


def _line(name: str, s: dict) -> str:
    return f"  {name}: " + ", ".join(
        f"{k} {'-' if s[k] is None else (f'{s[k]:.4g}' if isinstance(s[k], float) else s[k])}"
        for k in KEYS)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--run", required=True, help="a runs/eval/ directory")
    args = parser.parse_args()
    dt = float(load_config(args.config)["sim"]["dt"])
    run = REPO_ROOT / args.run
    records = [json.loads(line) for line in
               (run / "episodes.jsonl").read_text(encoding="utf-8").splitlines() if line]
    print(f"{run.name}: {len(records)} episodes")
    print(_line("overall", outcome_summary(records, dt)))
    for stratum in sorted({r.get("stratum") for r in records} - {None}):
        print(_line(stratum, outcome_summary([r for r in records if r.get("stratum") == stratum], dt)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
