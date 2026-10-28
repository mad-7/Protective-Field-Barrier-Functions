"""The route-level traversability report: the planner-margin quantification, merged and tabled.
Offline and quick (it reads CSVs); for the supervisor's ruling on margin values (2026-10-13,
item 3: "report before I rule values").

    python scripts/report_margins.py --config configs/analysis/planner_margins.yaml --seed 0 \
        --runs runs/analysis/MARGINS_20261013T233627Z_seed0 runs/analysis/MARGINS_<...>_seed0

Merges the `routes.csv` of every run given (`pfbnav.eval.margin_report.merge_runs`: complete
scenarios only, each once, duplicates required to agree), checks that every scenario of every
configured set is covered, and writes `runs/analysis/MARGINS_REPORT_<timestamp>_seed<seed>/`:
report.md (the tables), report.json (everything in them). A report missing scenarios says
PARTIAL at its head and names what is missing; it is never presented as the criterion's verdict.
"""
from __future__ import annotations

import argparse
import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from pfbnav.config import load_config
from pfbnav.eval.margin_report import LOCATIONS, merge_runs, parse_row, smallest_meeting, summarise
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.scenarios import load_set

REPO_ROOT = Path(__file__).resolve().parents[1]


def _pct(n: int, d: int) -> str:
    return f"{n} ({100.0 * n / d:.1f} %)" if d else "-"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--runs", nargs="+", required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    pm = cfg["planner_margins"]
    pairs = {(float(lm), float(sm)) for lm in pm["lethal_margins"] for sm in pm["soft_margins"]}
    run_dirs = [REPO_ROOT / r for r in args.runs]
    inputs, row_lists, quant = [], [], None
    for d in run_dirs:
        prov = json.loads((d / "provenance.json").read_text(encoding="utf-8"))
        with (d / "routes.csv").open(encoding="utf-8") as f:
            row_lists.append([parse_row(r) for r in csv.DictReader(f)])
        q = json.loads((d / "quantisation.json").read_text(encoding="utf-8"))
        if quant is None:
            quant = q
        elif q != quant:
            raise SystemExit(f"{d.name}'s quantisation check differs from the first run's")
        inputs.append({"run": d.relative_to(REPO_ROOT).as_posix(), "commit": prov.get("commit"),
                       "dirty": prov.get("dirty"), "rows": len(row_lists[-1])})
    merged = merge_runs(row_lists, pairs)

    expected = {Path(s).stem: len(load_set(REPO_ROOT / s)["scenarios"]) for s in pm["sets"]}
    covered: dict[str, int] = {}
    for s, _seed in merged["scenarios"]:
        covered[s] = covered.get(s, 0) + 1
    coverage = {s: {"built": n, "covered": covered.get(s, 0)} for s, n in expected.items()}
    complete = all(c["covered"] == c["built"] for c in coverage.values())
    table = summarise(merged["rows"], quant or [], cfg)
    meeting = smallest_meeting(table)
    straights_clear = [e for e in table if e["plannability_holds"]
                       and e["breaching_routes_by_location"]["straight"] == 0]
    smallest_straight = (min(straights_clear, key=lambda e: (e["lethal_margin"],
                                                              e["soft_margin"]))
                         if straights_clear else None)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = create_run_dir(REPO_ROOT / "runs" / "analysis", f"MARGINS_REPORT_{stamp}", args.seed,
                         cfg)
    report: dict[str, Any] = {
        "status": "COMPLETE" if complete else "PARTIAL", "inputs": inputs,
        "coverage": coverage, "duplicates": merged["duplicates"],
        "incomplete_scenarios": merged["incomplete"], "pairs": table,
        "smallest_meeting_criterion": meeting,
        "smallest_with_straights_clear": (None if smallest_straight is None else
                                          (smallest_straight["lethal_margin"],
                                           smallest_straight["soft_margin"])),
    }
    (out / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    n = len(merged["scenarios"])
    lines = [f"# Planner margins by criterion: route-level traversability ({report['status']})", "",
             f"{n} routes x {len(pairs)} (lethal, soft) pairs; inputs: "
             + ", ".join(f"`{i['run']}` ({str(i['commit'])[:8]}, dirty {i['dirty']})"
                         for i in inputs), ""]
    if not complete:
        lines += ["**PARTIAL -- not the criterion's verdict.** Missing: "
                  + ", ".join(f"{s} {c['covered']}/{c['built']}" for s, c in coverage.items()
                              if c["covered"] != c["built"]), ""]
    lines += [("**Criterion** (ruled 2026-10-13): zero field breaches for an ideal tracker on "
               "every route at its stratum's traversal speed, field along the tangent; every "
               "stratum plannable, grid quantisation included."), "",
              (f"**Smallest pair meeting it:** {meeting if meeting else 'none'}. "
               "**Smallest pair with every straight segment clear and plannability kept** "
               f"(information): {report['smallest_with_straights_clear']}."), "",
              "| lethal | soft | plannable | quant. fails | breaching routes | "
              + " | ".join(LOCATIONS) + " | outside goal approach | worst clearance (mm) | "
              "50 mm inside (info) |",
              "|---|---|---|---|---|" + "---|" * len(LOCATIONS) + "---|---|---|"]
    for e in table:
        loc = e["breaching_routes_by_location"]
        worst = e["worst_clearance_m"]
        lines.append(
            f"| {e['lethal_margin']:.2f} | {e['soft_margin']:.2f} | {e['plannable']}/"
            f"{e['scenarios']} | {e['quantisation_failures']} | "
            f"{_pct(e['breaching_routes'], e['plannable'])} | "
            + " | ".join(str(loc[c]) for c in LOCATIONS)
            + f" | {e['breaching_routes_outside_goal_approach']} | "
            f"{'-' if worst is None else f'{worst * 1000:.1f}'} | "
            f"{_pct(e['offset_breaching_routes'], e['plannable'])} |")
    lines += ["", "Breaching routes by stratum (planned routes in brackets):", "",
              "| lethal | soft | " + " | ".join(sorted({s for e in table
                                                        for s in e["per_stratum"]})) + " |",
              "|---|---|" + "---|" * len({s for e in table for s in e["per_stratum"]})]
    for e in table:
        ps = e["per_stratum"]
        lines.append(f"| {e['lethal_margin']:.2f} | {e['soft_margin']:.2f} | "
                     + " | ".join(f"{ps[s]['breaching_routes']} ({ps[s]['plannable']})"
                                  for s in sorted(ps)) + " |")
    (out / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    print(f"\n-> {out.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
