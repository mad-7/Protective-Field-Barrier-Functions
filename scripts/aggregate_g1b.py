"""Aggregate separate G1b invocations: spread across them, and the verdict it supports.

Reads each invocation's `summary.json` (never re-runs anything) and applies
`pfbnav.eval.tracking_spread.spread_verdict` with the committed criteria. Refuses to
aggregate invocations that differ in commit, dirty state, scene, criteria or physics, since
their spread would then mix a code or configuration change into what it reports as run-to-
run variation.

Usage (after three G1b invocations have finished):
    python scripts/aggregate_g1b.py --config configs/gates/g1b_rerun.yaml --seed 0 \
        --runs runs/gates/G1b_<stamp>_seed0 runs/gates/G1b_<stamp>_seed1 \
               runs/gates/G1b_<stamp>_seed2
"""
from __future__ import annotations

import argparse
import csv
import json
from datetime import datetime, timezone
from pathlib import Path

import yaml

from pfbnav.config import load_config
from pfbnav.eval.tracking_spread import margin_statement, spread_verdict
from pfbnav.utils.runio import create_run_dir

REPO_ROOT = Path(__file__).resolve().parents[1]
PREREG = REPO_ROOT / "preregistration" / "gates.yaml"

# Fields that must agree across invocations for their spread to mean run-to-run variation.
PROVENANCE_KEYS = ("commit", "dirty", "scene_sha256")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--runs", nargs="+", required=True)
    args = parser.parse_args()

    criteria = yaml.safe_load(PREREG.read_text())["G1b_closed_loop_tracking"]
    runs = [Path(r) for r in args.runs]
    summaries = [json.loads((r / "summary.json").read_text()) for r in runs]
    provenance = [json.loads((r / "provenance.json").read_text()) for r in runs]

    for key in PROVENANCE_KEYS:
        values = {str(p.get(key)) for p in provenance}
        if len(values) != 1:
            raise SystemExit(f"invocations differ in {key}: {sorted(values)}")
    if provenance[0]["dirty"]:
        raise SystemExit("the invocations ran on a dirty tree; not aggregating")
    for s in summaries:
        if s.get("result") == "PARTIAL":
            raise SystemExit("a narrowed (PARTIAL) invocation cannot enter the gate")
        if s["criteria"] != criteria:
            raise SystemExit("an invocation's criteria differ from preregistration/gates.yaml")
        if s["physics"] != summaries[0]["physics"]:
            raise SystemExit(f"invocations differ in physics: {s['physics']}")

    report = spread_verdict([s["runs"] for s in summaries], criteria)
    report.update(runs=[r.name for r in runs], criteria=criteria,
                  physics=summaries[0]["physics"], commit=provenance[0]["commit"],
                  per_invocation={r.name: {"v_max_from_gate": s["v_max_from_gate"],
                                           "vy_max_from_gate": s["vy_max_from_gate"],
                                           "result": s["result"]}
                                  for r, s in zip(runs, summaries, strict=True)})
    report["margins"] = [m for c in report["cells"].values() if (m := margin_statement(c))]

    cfg = load_config(args.config)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = create_run_dir(REPO_ROOT / "runs" / "gates", f"G1b-SPREAD_{stamp}", args.seed, cfg)
    (out / "summary.json").write_text(json.dumps(report, indent=2, default=float))
    with (out / "spread.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["profile", "speed", "metric", "min", "max", "spread", "limit",
                     "margin_worst", "invocations_within", "invocations", "status"])
        for c in report["cells"].values():
            for name, m in c["metrics"].items():
                w.writerow([c["profile"], c["speed"], name, m["min"], m["max"], m["spread"],
                            m["limit"], m["margin_worst"], m["invocations_within"],
                            m["invocations"], m["status"]])

    print(f"G1b across {report['invocations']} invocations: {report['result']}, "
          f"v_max {report['v_max_from_gate']}, vy_max {report['vy_max_from_gate']}")
    print(f"repetitions identical within each invocation: "
          f"{report['within_invocation_identical']}")
    for speed, p in report["per_speed"].items():
        print(f"  {speed:.2f}  {'pass' if p['all_pass'] else 'FAIL'}  failing {p['failing']}  "
              f"straddling {p['straddling']}")
    for line in report["margins"]:
        print("  " + line)
    print(f"-> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
