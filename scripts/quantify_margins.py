"""The planner's lethal and soft margins, quantified by criterion (ruled 2026-10-13). Offline; the
RESEARCHER runs it (hours of CPU: hard rule 11).

    python scripts/quantify_margins.py --config configs/analysis/planner_margins.yaml --seed 0
    python scripts/quantify_margins.py --config configs/analysis/planner_margins.yaml --seed 0 \
        --workers 6

**The defect.** The planner admits a route within its lethal radius, `half_width +
lethal_margin` = 0.2386 m, of a surface, which is inside the at-rest field
(`half_width + lateral_margin` = 0.2586 m) and further inside every moving field. An environment
defect from the D1 ruling (supervisor, 2026-10-13).

**The criterion, for every candidate (lethal, soft) pair in `planner_margins`:**
  1. **Zero field breaches**: every built route, re-planned with the candidate margins and
     followed exactly by an ideal tracker at its stratum's traversal speed with the field along
     the path tangent, never has a MAPPED surface inside its field
     (`pfbnav/world/route_check.py`; unmapped obstacles are placed on the route by design).
  2. **No stratum loses plannability**: every built scenario still plans, AND an aisle at each
     stratum's minimum clearance plans at every sub-cell position of the grid
     (`quantisation_offsets` per axis, both axes) -- the grid-quantisation case the built sets
     may not happen to contain.
The pairs meeting both are listed, with the smallest; the supervisor rules the values. **For
information only**, each route is also checked displaced `inside_offset_m` toward the inside of
its bends.

The traversal speed is derived per stratum exactly as the traversability probe derives it
(`max_traversal_speed` at G1b's committed cross-track, then `usable_band_top`); the field is that
band's moving field. Today every stratum derives 0.40 m/s, the largest field.

Sets: tune, tune-unmapped, pilot and training -- never `scenarios/test` (hard rule 2: choosing
margins is tuning). The built scenarios keep their worlds (`world_of`); only the plan is redone.

Writes `runs/analysis/MARGINS_<timestamp>_seed<seed>/`: routes.csv (one row per scenario and
pair), quantisation.json, summary.json (per pair: plannability per stratum, breaching routes and
samples, the worst clearance, the offset information, whether the criterion holds) and the table
it prints.
"""
from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import os
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pfbnav.config import load_config
from pfbnav.eval.margin_report import summarise
from pfbnav.planning.astar import plan_from_config
from pfbnav.planning.plannability import aisle_plannability
from pfbnav.safety.certified import FieldGeometry
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.baking import bake, planner_radii
from pfbnav.world.route_check import (
    FieldShape,
    clearance_to_field,
    location_classes,
    sample_route,
)
from pfbnav.world.scenarios import Scenario, load_set, world_of
from pfbnav.world.strata import (
    certified_params_from_config,
    max_traversal_speed,
    strata_from_config,
    usable_band_top,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
PREREG = REPO_ROOT / "preregistration" / "gates.yaml"
TEST_DIR = (REPO_ROOT / "scenarios" / "test").resolve()
ROUTE_FIELDS = ("set", "seed", "stratum", "lethal_margin", "soft_margin", "status",
                "path_length_m", "samples", "breached_samples", "min_clearance_m", "worst_x",
                "worst_y", "worst_yaw", "worst_location", "breached_start", "breached_goal",
                "breached_bend", "breached_straight", "min_clearance_outside_goal_m",
                "offset_breached_samples", "offset_min_clearance_m")

_W: dict[str, Any] = {}       # per-worker state, built once by _init


def with_margins(cfg: dict[str, Any], lethal: float, soft: float) -> dict[str, Any]:
    out = copy.deepcopy(cfg)
    out["planner"]["lethal_margin"] = float(lethal)
    out["planner"]["soft_margin"] = float(soft)
    return out


def stratum_bands(cfg: dict[str, Any], fields: FieldGeometry) -> dict[str, float]:
    """Each stratum's traversal band top, m/s, derived as the traversability probe does."""
    params = certified_params_from_config(cfg)
    offset = float(yaml.safe_load(PREREG.read_text(encoding="utf-8"))
                   ["G1b_closed_loop_tracking"]["max_cross_track"])
    out = {}
    for s in strata_from_config(cfg):
        band = usable_band_top(max_traversal_speed(fields, float(cfg["robot"]["half_width"]),
                                                   s.min_clearance, offset=offset), params)
        if band is None:
            raise SystemExit(f"stratum {s.name} admits no band")
        out[s.name] = float(band)
    return out


def _init(cfg: dict[str, Any]) -> None:
    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    fields = FieldGeometry(table.extent, certified_params_from_config(cfg))
    bands = stratum_bands(cfg, fields)
    _W.update(cfg=cfg, circumscribed=float(table.circumscribed), bands=bands,
              shapes={b: FieldShape.moving(fields, b) for b in set(bands.values())})


def evaluate(job: tuple[str, dict[str, Any]]) -> list[dict[str, Any]]:
    """Every candidate pair on one built scenario."""
    set_name, record = job
    cfg = _W["cfg"]
    pm = cfg["planner_margins"]
    scenario = Scenario(**record)
    layout = world_of(scenario, cfg)
    mapped = [o.polygon for o in layout.mapped]
    field = _W["shapes"][_W["bands"][scenario.stratum]]
    rows = []
    for lethal in pm["lethal_margins"]:
        for soft in pm["soft_margins"]:
            c = with_margins(cfg, lethal, soft)
            baked = bake(layout.obstacles, layout.bounds, float(c["planner"]["resolution"]),
                         planner_radii(c, circumscribed=_W["circumscribed"]))
            plan = plan_from_config(baked, scenario.start[:2], scenario.goal, c)
            row: dict[str, Any] = {"set": set_name, "seed": scenario.seed,
                                   "stratum": scenario.stratum, "lethal_margin": lethal,
                                   "soft_margin": soft, "status": plan.status}
            if plan.ok:
                samples = sample_route(plan.path_world, float(pm["sample_spacing_m"]))
                gap = clearance_to_field(samples, mapped, field)
                off = clearance_to_field(samples.offset(float(pm["inside_offset_m"])), mapped,
                                         field)
                k = int(np.argmin(gap))
                where = location_classes(samples, start_window=float(pm["start_window_m"]),
                                         goal_window=float(pm["goal_window_m"]),
                                         bend_window=float(pm["bend_window_m"]))
                outside_goal = gap[where != "goal"]
                row.update({f"breached_{c}": int(np.sum((gap < 0) & (where == c)))
                            for c in ("start", "goal", "bend", "straight")})
                row.update(worst_location=str(where[k]),
                           min_clearance_outside_goal_m=(float(outside_goal.min())
                                                         if outside_goal.size else None))
                path = np.asarray(plan.path_world)
                row.update(path_length_m=float(np.sum(np.linalg.norm(np.diff(path, axis=0),
                                                                     axis=1))),
                           samples=int(gap.size), breached_samples=int(np.sum(gap < 0)),
                           min_clearance_m=float(gap.min()), worst_x=float(samples.x[k]),
                           worst_y=float(samples.y[k]), worst_yaw=float(samples.yaw[k]),
                           offset_breached_samples=int(np.sum(off < 0)),
                           offset_min_clearance_m=float(off.min()))
            rows.append(row)
    return rows


def quantisation(cfg: dict[str, Any], circumscribed: float) -> list[dict[str, Any]]:
    """Does an aisle at each stratum's minimum clearance plan at every sub-cell position?
    (`pfbnav/planning/plannability.py`, at every configured margin pair.)"""
    pm = cfg["planner_margins"]
    out = []
    for lethal in pm["lethal_margins"]:
        for soft in pm["soft_margins"]:
            for entry in aisle_plannability(
                    with_margins(cfg, lethal, soft), circumscribed,
                    offsets=int(pm["quantisation_offsets"]),
                    aisle_length=float(pm["quantisation_aisle_length_m"])):
                out.append({"lethal_margin": lethal, "soft_margin": soft, **entry})
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    parser.add_argument("--sets", nargs="+", default=None,
                        help="only these of the configured sets (to complete an interrupted run; "
                             "the summary is then of these sets alone)")
    parser.add_argument("--limit", type=int, default=None,
                        help="only the first N scenarios of each set (a smoke check, PARTIAL)")
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    pm = cfg["planner_margins"]
    chosen = pm["sets"]
    if args.sets:
        unknown = [s for s in args.sets if s not in pm["sets"]]
        if unknown:
            raise SystemExit(f"not configured sets: {unknown}")
        chosen = args.sets
    paths = [(REPO_ROOT / s).resolve() for s in chosen]
    if any(TEST_DIR in p.parents for p in paths):
        raise SystemExit("test layouts are never used to choose margins (hard rule 2)")
    jobs = []
    for path in paths:
        records = load_set(path)["scenarios"]
        jobs += [(path.stem, r) for r in (records[:args.limit] if args.limit else records)]

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "analysis", f"MARGINS_{stamp}", args.seed, cfg)
    _init(cfg)
    print(f"{len(jobs)} scenarios x {len(pm['lethal_margins']) * len(pm['soft_margins'])} pairs, "
          f"{args.workers} workers; traversal bands {_W['bands']}")
    quant = quantisation(cfg, _W["circumscribed"])
    (run_dir / "quantisation.json").write_text(json.dumps(quant, indent=2), encoding="utf-8")

    rows: list[dict[str, Any]] = []
    with (run_dir / "routes.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(ROUTE_FIELDS))
        writer.writeheader()
        with ProcessPoolExecutor(max_workers=args.workers, initializer=_init,
                                 initargs=(cfg,)) as pool:
            for i, result in enumerate(pool.map(evaluate, jobs, chunksize=4), start=1):
                rows += result
                writer.writerows(result)
                if i % 50 == 0 or i == len(jobs):
                    print(f"  {i}/{len(jobs)} scenarios", flush=True)

    table = summarise(rows, quant, cfg)
    meeting = [e for e in table if e["criterion_holds"]]
    smallest = (min(meeting, key=lambda e: (e["lethal_margin"], e["soft_margin"]))
                if meeting else None)
    summary = {"partial": bool(args.limit), "sets": chosen, "bands": _W["bands"], "pairs": table,
               "meeting_criterion": [(e["lethal_margin"], e["soft_margin"]) for e in meeting],
               "smallest_meeting": (None if smallest is None else
                                    (smallest["lethal_margin"], smallest["soft_margin"]))}
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(f"\n{'lethal':>7} {'soft':>5} | {'plannable':>11} {'quant':>5} | {'breaching':>9} "
          f"{'share':>8} {'worst mm':>8} | {'off-50mm':>8} | criterion")
    for e in table:
        worst = e["worst_clearance_m"]
        print(f"{e['lethal_margin']:7.2f} {e['soft_margin']:5.2f} | "
              f"{e['plannable']:5d}/{e['scenarios']:<5d} {e['quantisation_failures']:5d} | "
              f"{e['breaching_routes']:9d} {e['breached_sample_share'] or 0:8.4f} "
              f"{(worst * 1000 if worst is not None else math.nan):8.1f} | "
              f"{e['offset_breaching_routes']:8d} | {'HOLDS' if e['criterion_holds'] else '-'}  "
              f"by location {e['breaching_routes_by_location']}")
    print(f"\nsmallest pair meeting the criterion: {summary['smallest_meeting']}"
          + ("  (PARTIAL: --limit)" if args.limit else ""))
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
