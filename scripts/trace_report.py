"""Re-analyse finished traces (`scripts/trace_episodes.py` output) with the current analysis. Read-only,
seconds, no simulator.

    python scripts/trace_report.py --config configs/base.yaml --seed 0 --run runs/diag/TRACE2_<stamp>_seed0

For each `trace_<index>.jsonl`: driven distance against the planned length and by route third, the
lateral offset, goal overshoot and circling, the oscillation measures (sign flips per second of the
commanded lateral speed and yaw rate), the PFB's intervention and fallback shares, the latched share,
and the stop causes (`pfbnav.eval.trace_analysis`). The planned length is read from the trace folder's
`summary.json`. Writes nothing.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.eval.trace_analysis import stop_causes, wandering

REPO_ROOT = Path(__file__).resolve().parents[1]


def _f(x, nd=2) -> str:
    return "-" if x is None else f"{x:.{nd}f}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--run", required=True, help="a runs/diag/TRACE2_* directory")
    args = parser.parse_args()
    dt = float(load_config(args.config)["sim"]["dt"])
    run = REPO_ROOT / args.run
    plans = {int(e["index"]): float(e["wandering"]["plan_length_m"]) for e in
             json.loads((run / "summary.json").read_text(encoding="utf-8"))}
    traced = json.loads((run / "traced.json").read_text(encoding="utf-8"))
    print(f"{run.name}: {traced['controller']}, action {traced.get('action')}, filter {traced['filter']}")
    print("index | driven/plan | thirds (m) | cross-track mean/p95 | flips/s lat, yaw | PFB interv., "
          "fallback | latched | stops")
    for index, plan in sorted(plans.items()):
        rows = [json.loads(line) for line in (run / f"trace_{index}.jsonl")
                .read_text(encoding="utf-8").splitlines() if line]
        w = wandering(rows, plan_length_m=plan, dt=dt)
        causes = [c["cause"] for c in stop_causes(rows)]
        print(f"{index:5d} | {w['driven_m']:.1f}/{plan:.1f} = {_f(w['driven_over_plan'])} | "
              f"{[round(v, 1) for v in w['driven_by_route_third_m']]} | "
              f"{w['cross_track_mean_m']:.2f}/{w['cross_track_p95_m']:.2f} | "
              f"{w['lateral_flips_per_s']:.2f}, {w['yaw_flips_per_s']:.2f} | "
              f"{_f(w['pfb_intervention_share'])}, {_f(w['pfb_fallback_share'], 3)} | "
              f"{w['latched_share']:.3f} | {causes}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
