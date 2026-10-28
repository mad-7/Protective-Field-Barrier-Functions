"""G1b physics regression after the scanner-layout change: binding cells only. (WP1)

Pre-declared (supervisor, 2026-09-21): the S300 layout changes the robot's mass and
inertia, so G1b's binding cells are re-run -- translate_while_rotating at 0.40 and 0.50,
lateral at 0.50 -- in three separate invocations. **If any cell's verdict changes against
the closed G1b spread, G1b is re-run in full.** Verdict is PASS only if every run of every
invocation is within every limit; a straddle is FAIL, as in the gate itself.

This reads finished invocations only (never re-runs anything). The invocations are
narrowed on purpose, so a PARTIAL result is expected here -- unlike `aggregate_g1b.py`,
which refuses it. They must agree with each other in commit, dirty state and scene, and
their scene must differ from the reference's (otherwise this is not a regression of the
layout change).

Usage (after three narrowed G1b invocations):
    python scripts/g1b_regression.py --config configs/gates/g1b_rerun.yaml --seed 0 \
        --reference runs/gates/G1b-SPREAD_20260920T215108Z_seed0 \
        --runs runs/gates/G1b_<stamp>_seed0 runs/gates/G1b_<stamp>_seed1 \
               runs/gates/G1b_<stamp>_seed2
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import yaml

from pfbnav.config import load_config
from pfbnav.eval.tracking_spread import margin_statement, regression, spread_verdict
from pfbnav.utils.runio import create_run_dir

REPO_ROOT = Path(__file__).resolve().parents[1]
PREREG = REPO_ROOT / "preregistration" / "gates.yaml"
BINDING_CELLS = ("translate_while_rotating@0.40", "translate_while_rotating@0.50",
                 "lateral@0.50")
PROVENANCE_KEYS = ("commit", "dirty", "scene_sha256")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--reference", required=True, help="the closed G1b-SPREAD directory")
    parser.add_argument("--runs", nargs="+", required=True)
    args = parser.parse_args()

    if len(args.runs) < 3:
        raise SystemExit("the regression is pre-declared at three invocations")
    criteria = yaml.safe_load(PREREG.read_text())["G1b_closed_loop_tracking"]
    reference_dir = Path(args.reference)
    reference = json.loads((reference_dir / "summary.json").read_text())
    reference_prov = json.loads((reference_dir / "provenance.json").read_text())
    runs = [Path(r) for r in args.runs]
    summaries = [json.loads((r / "summary.json").read_text()) for r in runs]
    provenance = [json.loads((r / "provenance.json").read_text()) for r in runs]

    for key in PROVENANCE_KEYS:
        values = {str(p.get(key)) for p in provenance}
        if len(values) != 1:
            raise SystemExit(f"invocations differ in {key}: {sorted(values)}")
    if provenance[0]["dirty"]:
        raise SystemExit("the invocations ran on a dirty tree; not comparing")
    if reference["criteria"] != criteria:
        raise SystemExit("the reference's criteria differ from preregistration/gates.yaml")
    for s in summaries:
        if s["criteria"] != criteria:
            raise SystemExit("an invocation's criteria differ from preregistration/gates.yaml")
        if s["physics"] != reference["physics"]:
            raise SystemExit(f"physics differ from the reference: {s['physics']}")
    new_scene = provenance[0]["scene_sha256"]
    if new_scene == reference_prov.get("scene_sha256"):
        raise SystemExit("the invocations ran on the reference's scene; nothing to regress")

    spread = spread_verdict([s["runs"] for s in summaries], criteria)
    report = regression(reference["cells"], spread["cells"], BINDING_CELLS)
    report.update(reference=reference_dir.name, runs=[r.name for r in runs],
                  reference_scene_sha256=reference_prov.get("scene_sha256"),
                  scene_sha256=new_scene, commit=provenance[0]["commit"],
                  within_invocation_identical=spread["within_invocation_identical"],
                  spread={c: spread["cells"][c] for c in BINDING_CELLS},
                  margins=[m for c in BINDING_CELLS
                           if (m := margin_statement(spread["cells"][c]))])

    cfg = load_config(args.config)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = create_run_dir(REPO_ROOT / "runs" / "gates", f"G1b-REGRESSION_{stamp}", args.seed,
                         cfg)
    (out / "summary.json").write_text(json.dumps(report, indent=2, default=float))

    print(f"G1b regression, {len(runs)} invocations against {reference_dir.name}")
    for c, r in report["cells"].items():
        print(f"  {c:<32} {r['reference']} ({r['reference_status']}) -> {r['regression']} "
              f"({r['regression_status']}){'   CHANGED' if r['changed'] else ''}")
    for line in report["margins"]:
        print("  " + line)
    print(f"repetitions identical within each invocation: "
          f"{report['within_invocation_identical']}")
    print("FULL G1b RE-RUN REQUIRED" if report["full_rerun_required"]
          else "no verdict changed: G1b stands")
    print(f"-> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
