"""A read-only look at a tuning run while it runs: each candidate's score so far. Seconds.

    python scripts/peek_tuning.py --config configs/base.yaml --seed 0
    python scripts/peek_tuning.py --config configs/base.yaml --seed 0 --run runs/tuning/<name>

Reads `candidates.json` and every `candidate_<i>/episodes.jsonl` of the run (the latest under
`runs/tuning/` by default) and prints, per candidate, the tuning's own score
(`pfbnav.eval.tuning.candidate_score`: success rate, pooled stops per 100 m, mission time) with its
parameters. A candidate still running is marked partial. This is a monitor, not the selection: the
selection is the run's own `summary.json`, under the declared rule, over complete candidates only.
Writes nothing.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.eval.tuning import candidate_score

REPO_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--run", default=None, help="a runs/tuning/ directory (default: latest)")
    args = parser.parse_args()
    cfg = load_config(args.config)
    dt = float(cfg["sim"]["dt"])
    if args.run:
        run = Path(args.run)
    else:
        runs = sorted((p for p in (REPO_ROOT / "runs" / "tuning").iterdir() if p.is_dir()),
                      key=lambda p: p.stat().st_mtime)
        if not runs:
            raise SystemExit("no tuning run under runs/tuning/")
        run = runs[-1]
    doc = json.loads((run / "candidates.json").read_text(encoding="utf-8"))
    candidates = doc["candidates"]
    print(f"{run.name}: {doc['controller']}, {len(candidates)} candidates")
    logs = {i: run / f"candidate_{i}" / "episodes.jsonl" for i in range(len(candidates))}
    found = {i: [json.loads(line) for line in p.read_text(encoding="utf-8").splitlines() if line]
             for i, p in logs.items() if p.is_file()}
    full = max((len(r) for r in found.values()), default=0)    # a complete candidate's episodes
    for i, records in found.items():
        params = candidates[i]
        if not records:
            continue
        s = candidate_score(records, dt)
        stops = "-" if s["stops_per_100m"] is None else f"{s['stops_per_100m']:7.1f}"
        mission = "-" if s["mission_time_s"] is None else f"{s['mission_time_s']:6.1f}"
        partial = "" if len(records) == full else f"  (partial: {len(records)}/{full})"
        print(f"{i:2d}  success {s['success_rate']:.3f} ({s['counts'].get('success', 0):2d}/"
              f"{len(records)})  stops/100m {stops}  mission {mission} s  "
              f"contact {s['contact_rate']:.3f}{partial}")
        print("    " + ", ".join(f"{k}={v:.3g}" for k, v in params.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
