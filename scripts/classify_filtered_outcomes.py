"""Classify the stops and the timeouts that remain with the filter, from finished run directories
(offline: reads `episodes.jsonl` only; nothing is run and nothing is written under `runs/`).

    python scripts/classify_filtered_outcomes.py <run dir> [<run dir> ...] [--out classes.json]

Prints, per run, the timeouts by progress class (`pfbnav/eval/outcome_classes.py`) and the stops by the
conditions of their episodes. With `--out`, also writes the same as JSON. The classes describe what the
record shows and state no cause.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from pfbnav.eval.outcome_classes import PROGRESS_CLASSES, classify_stops, classify_timeouts


def main() -> int:
    ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    ap.add_argument("runs", nargs="+", type=Path, help="run directories holding episodes.jsonl")
    ap.add_argument("--out", type=Path, default=None, help="write the classes as JSON")
    args = ap.parse_args()
    report: dict[str, dict] = {}
    for run in args.runs:
        path = run / "episodes.jsonl"
        if not path.is_file():
            raise SystemExit(f"{path} does not exist")
        episodes = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]
        report[run.name] = {"episodes": len(episodes), "timeouts": classify_timeouts(episodes),
                            "stops": classify_stops(episodes)}
        t, s = report[run.name]["timeouts"], report[run.name]["stops"]
        print(f"{run.name}: {len(episodes)} episodes, {t['timeouts']} timeouts, {s['stops']} stops")
        for c in PROGRESS_CLASSES:
            row = t["by_progress"][c]
            print(f"  timeouts {c:9s} mapped-only {row['mapped_only']:3d}  unmapped {row['unmapped']:3d}"
                  f"  mean brake share {row['mean_brake_share']}")
        print(f"  stops: {s['stops']} in {s['episodes_with_stops']} episodes; "
              f"with fallback {s['in_episodes_with_fallback']}; {s['by_route_condition']}")
    if args.out:
        args.out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
