"""Score a profiling pass against the derived per-step budget. Offline; no simulator.

The budget is not a number anyone wrote down: it follows from the aggregate requirement on
the training host and the instance count (`pfbnav/eval/step_budget.py`), both read from the
config. This reads a `docs/profiling/step_cost_<stamp>/summary.json` and reports, per
instance count, the measured p95 step against the budget for that count, the measured
aggregate steps/s against the requirement, and which components carry the step.

**A pass on the development laptop is not a pass on the training host** and is never
reported as one: the laptop shares four physical cores between the instances, rendered
unless told otherwise, and never runs the learner. G2 measures the requirement where it
binds. What this catches is a regression in the *shape* of the step, and a step whose cost
has grown past the point where the aggregate could be met at all.

Usage:
    python scripts/check_step_budget.py --config configs/base.yaml \
        --profile docs/profiling/step_cost_20260918T194545Z
    python scripts/check_step_budget.py --config configs/base.yaml \
        --profile docs/profiling/step_cost_20260918T194545Z \
        --against docs/profiling/step_cost_20260918T182131Z
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from pfbnav.config import load_config
from pfbnav.eval.step_budget import compare_components, headroom, per_step_budget_s

REPO_ROOT = Path(__file__).resolve().parents[1]


def components_s(run: dict[str, Any]) -> dict[str, float]:
    """A run's per-component p95, in seconds."""
    return {name: float(c["p95_ms"]) / 1e3 for name, c in run["components"].items()}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--profile", required=True, help="a docs/profiling/step_cost_* run")
    parser.add_argument("--against", help="an earlier profiling run, to report what moved")
    args = parser.parse_args()

    cfg = load_config(args.config)
    budget_cfg = cfg["env"]["step_budget"]
    target = float(budget_cfg["aggregate_steps_per_s"])
    planned = int(budget_cfg["instances"])

    profile = Path(args.profile)
    summary = json.loads((profile / "summary.json").read_text())
    provenance = json.loads((profile / "provenance.json").read_text())
    machine = summary.get("machine", {})

    print(f"profile {profile.name}  commit {str(provenance.get('commit'))[:8]}  "
          f"dirty {provenance.get('dirty')}")
    print(f"machine: {machine.get('physical_cores')} physical / "
          f"{machine.get('logical_cores')} logical cores, {machine.get('platform')}")
    print(f"requirement: >= {target:g} steps/s aggregate ON THE TRAINING HOST; this is not "
          f"that host, so what follows is a shape check, not a verdict on the requirement.")
    print(f"planned parallelism: {planned} instances -> per-step p95 budget "
          f"{per_step_budget_s(target, planned) * 1e3:.0f} ms\n")

    print(f"{'instances':>9} {'p95 step':>9} {'budget':>8} {'used':>6} {'aggregate':>10} "
          f"{'vs target':>10}")
    rows = []
    for key in sorted(summary["runs"], key=int):
        run = summary["runs"][key]
        n = int(key)
        h = headroom(float(run["whole_step"]["p95_ms"]) / 1e3, per_step_budget_s(target, n))
        passes = run["aggregate_steps_per_s_by_pass"]
        aggregate = min(float(v) for v in passes)      # the worst pass, not the best
        rows.append({"instances": n, **h, "aggregate_steps_per_s_worst_pass": aggregate,
                     "meets_target_here": aggregate >= target,
                     "components_s": components_s(run)})
        print(f"{n:>9} {h['measured_p95_s'] * 1e3:>8.1f}ms {h['budget_s'] * 1e3:>7.0f}ms "
              f"{h['fraction_used']:>5.0%} {aggregate:>9.1f}/s "
              f"{'ok' if aggregate >= target else 'BELOW':>10}")

    planned_row = next((r for r in rows if r["instances"] == planned), None)
    if planned_row is not None:
        print(f"\nat the planned {planned} instances: step p95 uses "
              f"{planned_row['fraction_used']:.0%} of its budget, "
              f"{planned_row['margin_s'] * 1e3:.0f} ms spare; worst-pass aggregate "
              f"{planned_row['aggregate_steps_per_s_worst_pass']:.1f} steps/s")
        biggest = sorted(planned_row["components_s"].items(), key=lambda kv: -kv[1])[:4]
        print("  carried by: " + ", ".join(f"{k} {v * 1e3:.1f} ms" for k, v in biggest))

    if args.against:
        earlier = json.loads((Path(args.against) / "summary.json").read_text())
        key = str(planned) if str(planned) in earlier["runs"] else max(earlier["runs"])
        print(f"\nagainst {Path(args.against).name} at {key} instance(s), by what moved:")
        for row in compare_components(components_s(earlier["runs"][key]),
                                      components_s(summary["runs"][str(planned)])):
            print(f"  {row['component']:<12} {row['before_ms']:>7.2f} -> "
                  f"{row['after_ms']:>7.2f} ms  ({row['change_ms']:+7.2f}; share "
                  f"{row['before_share']:.0%} -> {row['after_share']:.0%})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
