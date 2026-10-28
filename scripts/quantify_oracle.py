"""The soft margin under the ORACLE criterion, lethal margin 0.07 (ruled 2026-10-15). Offline; the
RESEARCHER runs it (an hour or more of CPU: hard rule 11).

    python scripts/quantify_oracle.py --config configs/analysis/planner_oracle.yaml --seed 0 \
        --workers 6
    # if a run stops part-way: complete the missing sets, then merge
    python scripts/quantify_oracle.py --config configs/analysis/planner_oracle.yaml --seed 0 \
        --workers 6 --sets scenarios/train/train_wide.json
    python scripts/quantify_oracle.py --config configs/analysis/planner_oracle.yaml --seed 0 \
        --report runs/analysis/ORACLE_<a>_seed0 runs/analysis/ORACLE_<b>_seed0

**The criterion (amended 2026-10-15).** The first quantification held an ideal tracker at the
stratum's traversal speed through every corner and into every goal -- wrong, as the supervisor
ruled. Now an **oracle tracker** follows each route exactly, at every point in the fastest band
whose field (along the tangent) is clear, under the deceleration limit
(`pfbnav/world/route_check.py`: `band_caps`, `oracle_speeds`). A route is traversable without a stop
iff the band-0 field is clear everywhere along it, start and goal included; **the criterion is zero
routes with a band-0 breach.**

Per built route and candidate soft margin (lethal fixed at 0.07), re-planned on the built world:
  * the band-0 breach, where it happens (start, goal approach, bend, straight), and the number of
    separate breached stretches -- the route's **unavoidable-stop floor** under the oracle;
  * **start validity** (band-0 field along the initial direction clear at the start) and **goal
    validity** (along the final approach, on arrival): the generator's rules from the rebuild on,
    which re-sample the goal on failure, never the layout;
  * the oracle's **mission time** against constant top speed (traversable routes);
  * **information:** band-0 breaches with the route displaced 50 mm toward the inside of bends.
Plannability is checked as before: every route plans, and the minimum-clearance aisle plans at
every sub-cell position (`quantify_margins.quantisation`).

The decisions are declared in `pfbnav/eval/oracle_report.py`: the soft margin with the fewest
band-0 breaches at bends (among those keeping every route plannable), and the residual rule for
it -- residual <= 2 % of routes in every stratum -> re-sample; any stratum above -> stop and
report.

Writes `runs/analysis/ORACLE_<timestamp>_seed<seed>/`: routes.csv, quantisation.json,
summary.json, report.md. `--report` writes `ORACLE_REPORT_<timestamp>_seed<seed>/` from several
runs (complete scenarios only, each once, duplicates required to agree; PARTIAL if any is missing).
"""
from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.eval.margin_report import merge_runs
from pfbnav.eval.oracle_report import RESIDUAL_LIMIT, parse_row, summarise
from pfbnav.planning.astar import plan_from_config
from pfbnav.safety.certified import FieldGeometry
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.baking import bake, planner_radii
from pfbnav.world.route_check import (
    FieldShape,
    arc_length,
    band_caps,
    clearance_to_field,
    location_classes,
    oracle_speeds,
    sample_route,
    stop_runs,
    traverse_time,
)
from pfbnav.world.scenarios import Scenario, load_set, world_of
from pfbnav.world.strata import certified_params_from_config

REPO_ROOT = Path(__file__).resolve().parents[1]
TEST_DIR = (REPO_ROOT / "scenarios" / "test").resolve()
ROUTE_FIELDS = ("set", "seed", "stratum", "lethal_margin", "soft_margin", "status",
                "path_length_m", "samples", "band0_breached_samples", "unavoidable_stops",
                "start_valid", "goal_valid", "band0_breach_start", "band0_breach_goal",
                "band0_breach_bend", "band0_breach_straight", "band0_min_clearance_m",
                "oracle_time_s", "constant_time_s", "offset_band0_breached_samples")

_W: dict[str, Any] = {}


def _margins_module():
    spec = importlib.util.spec_from_file_location(
        "quantify_margins", REPO_ROOT / "scripts" / "quantify_margins.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["quantify_margins"] = module
    spec.loader.exec_module(module)
    return module


def _init(cfg: dict[str, Any]) -> None:
    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    params = certified_params_from_config(cfg)
    fields = FieldGeometry(table.extent, params)
    tops = [float(t) for t in params.band_top_speeds]
    _W.update(cfg=cfg, circumscribed=float(table.circumscribed), tops=tops,
              shapes=[FieldShape.moving(fields, t) for t in tops],
              decel=float(cfg["actuation"]["a_decel_max"]))


def evaluate(job: tuple[str, dict[str, Any]]) -> list[dict[str, Any]]:
    """Every candidate soft margin on one built scenario, under the oracle."""
    set_name, record = job
    cfg = _W["cfg"]
    pm = cfg["planner_margins"]
    qm = sys.modules.get("quantify_margins") or _margins_module()
    scenario = Scenario(**record)
    layout = world_of(scenario, cfg)
    mapped = [o.polygon for o in layout.mapped]
    rows = []
    for lethal in pm["lethal_margins"]:
        for soft in pm["soft_margins"]:
            c = qm.with_margins(cfg, lethal, soft)
            baked = bake(layout.obstacles, layout.bounds, float(c["planner"]["resolution"]),
                         planner_radii(c, circumscribed=_W["circumscribed"]))
            plan = plan_from_config(baked, scenario.start[:2], scenario.goal, c)
            row: dict[str, Any] = {"set": set_name, "seed": scenario.seed,
                                   "stratum": scenario.stratum, "lethal_margin": lethal,
                                   "soft_margin": soft, "status": plan.status}
            if plan.ok:
                samples = sample_route(plan.path_world, float(pm["sample_spacing_m"]))
                gaps = [clearance_to_field(samples, mapped, shape) for shape in _W["shapes"]]
                clear = np.array([g >= 0 for g in gaps])
                band0 = ~clear[0]
                where = location_classes(samples, start_window=float(pm["start_window_m"]),
                                         goal_window=float(pm["goal_window_m"]),
                                         bend_window=float(pm["bend_window_m"]))
                arc = arc_length(samples)
                oracle = (traverse_time(arc, oracle_speeds(arc, band_caps(clear, _W["tops"]),
                                                           _W["decel"]))
                          if not band0.any() else None)
                off = clearance_to_field(samples.offset(float(pm["inside_offset_m"])), mapped,
                                         _W["shapes"][0])
                row.update(path_length_m=float(arc[-1]), samples=int(arc.size),
                           band0_breached_samples=int(band0.sum()),
                           unavoidable_stops=stop_runs(band0),
                           start_valid=int(not band0[0]), goal_valid=int(not band0[-1]),
                           band0_min_clearance_m=float(gaps[0].min()),
                           oracle_time_s=oracle, constant_time_s=float(arc[-1]) / _W["tops"][-1],
                           offset_band0_breached_samples=int(np.sum(off < 0)),
                           **{f"band0_breach_{k}": int(np.sum(band0 & (where == k)))
                              for k in ("start", "goal", "bend", "straight")})
            rows.append(row)
    return rows


def write_report(out: Path, rows: list[dict[str, Any]], quant: list[dict[str, Any]],
                 cfg: dict[str, Any], header: list[str]) -> dict[str, Any]:
    pm = cfg["planner_margins"]
    result = summarise(rows, pm["soft_margins"])
    quant_failures = {float(q["soft_margin"]): len(q["failures"]) for q in quant}
    for soft, entry in result["by_soft"].items():
        entry["quantisation_failures"] = quant_failures.get(soft, 0)
    (out / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")

    lines = [*header, "",
             ("Lethal margin 0.07 (ruled). Oracle tracker: fastest clear band at every point, "
             "deceleration-limited; criterion: zero routes with a band-0 breach."), "",
             ("| soft | plannable | quant. fails | start-invalid | goal-invalid | residual | "
             "bend-breach routes | unavoidable stops /100 m | mission time / constant (median) | "
             "50 mm inside (info) | residual rule |"),
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for soft, e in result["by_soft"].items():
        a = e["all"]
        ratio = a["mission_time_ratio_median"]
        lines.append(
            f"| {soft:.2f} | {a['plannable']}/{a['routes']} | {e['quantisation_failures']} | "
            f"{a['start_invalid']} | {a['goal_invalid']} | {a['residual']} | "
            f"{a['bend_breach_routes']} | {a['unavoidable_stops_per_100m']:.3f} | "
            f"{'-' if ratio is None else f'{ratio:.3f}'} | {a['offset_band0_breach_routes']} | "
            f"{e['residual_rule']} |")
    chosen = result["chosen_soft"]
    lines += ["", (f"**Chosen soft margin (fewest bend breaches, plannability kept):** {chosen}. "
              f"**Residual rule** (<= {RESIDUAL_LIMIT:.0%} in every stratum): "
              f"{result['residual_rule']}."), ""]
    if chosen is not None:
        lines += [f"Per stratum at soft {chosen}:", "",
                  ("| stratum | routes | start-invalid | goal-invalid | residual (share) | "
                  "bend-breach routes | stops /100 m | time ratio (median) | 50 mm inside |"),
                  "|---|---|---|---|---|---|---|---|---|"]
        for s, b in result["by_soft"][chosen]["per_stratum"].items():
            ratio = b["mission_time_ratio_median"]
            lines.append(
                f"| {s} | {b['routes']} | {b['start_invalid']} | {b['goal_invalid']} | "
                f"{b['residual']} ({b['residual_share']:.1%}) | {b['bend_breach_routes']} | "
                f"{b['unavoidable_stops_per_100m']:.3f} | "
                f"{'-' if ratio is None else f'{ratio:.3f}'} | "
                f"{b['offset_band0_breach_routes']} |")
    (out / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    parser.add_argument("--sets", nargs="+", default=None,
                        help="only these of the configured sets (to complete an interrupted run)")
    parser.add_argument("--report", nargs="+", default=None,
                        help="merge these ORACLE run directories into one report; runs nothing")
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    pm = cfg["planner_margins"]
    pairs = {(float(lm), float(sm)) for lm in pm["lethal_margins"] for sm in pm["soft_margins"]}
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")

    if args.report:
        dirs = [REPO_ROOT / r for r in args.report]
        lists, quant, inputs = [], None, []
        for d in dirs:
            with (d / "routes.csv").open(encoding="utf-8") as f:
                lists.append([parse_row(r) for r in csv.DictReader(f)])
            q = json.loads((d / "quantisation.json").read_text(encoding="utf-8"))
            if quant is not None and q != quant:
                raise SystemExit(f"{d.name}'s quantisation check differs from the first run's")
            quant = q
            prov = json.loads((d / "provenance.json").read_text(encoding="utf-8"))
            inputs.append(f"`{d.relative_to(REPO_ROOT).as_posix()}` "
                          f"({str(prov.get('commit'))[:8]}, dirty {prov.get('dirty')})")
        merged = merge_runs(lists, pairs)
        expected = {Path(s).stem: len(load_set(REPO_ROOT / s)["scenarios"]) for s in pm["sets"]}
        covered: dict[str, int] = {}
        for s, _seed in merged["scenarios"]:
            covered[s] = covered.get(s, 0) + 1
        missing = {s: f"{covered.get(s, 0)}/{n}" for s, n in expected.items()
                   if covered.get(s, 0) != n}
        out = create_run_dir(REPO_ROOT / "runs" / "analysis", f"ORACLE_REPORT_{stamp}", args.seed,
                             cfg)
        status = "PARTIAL" if missing else "COMPLETE"
        header = [f"# Soft margin under the oracle criterion ({status})", "",
                  (f"{len(merged['scenarios'])} routes; inputs: {', '.join(inputs)}; "
                  f"{merged['duplicates']} routes in more than one run, all agreeing.")]
        if missing:
            header += ["", f"**PARTIAL -- not the verdict.** Missing: {missing}"]
        write_report(out, merged["rows"], quant or [], cfg, header)
        print(f"\n-> {out.relative_to(REPO_ROOT).as_posix()}")
        return 0

    chosen = pm["sets"]
    if args.sets:
        unknown = [s for s in args.sets if s not in pm["sets"]]
        if unknown:
            raise SystemExit(f"not configured sets: {unknown}")
        chosen = args.sets
    paths = [(REPO_ROOT / s).resolve() for s in chosen]
    if any(TEST_DIR in p.parents for p in paths):
        raise SystemExit("test layouts are never used to choose margins (hard rule 2)")
    jobs = [(p.stem, r) for p in paths for r in load_set(p)["scenarios"]]

    run_dir = create_run_dir(REPO_ROOT / "runs" / "analysis", f"ORACLE_{stamp}", args.seed, cfg)
    qm = _margins_module()
    _init(cfg)
    print(f"{len(jobs)} scenarios x {len(pairs)} soft margins (lethal {pm['lethal_margins']}), "
          f"{args.workers} workers; bands {_W['tops']}, deceleration {_W['decel']} m/s^2")
    quant = qm.quantisation(cfg, _W["circumscribed"])
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
                f.flush()
                if i % 50 == 0 or i == len(jobs):
                    print(f"  {i}/{len(jobs)} scenarios", flush=True)

    header = [f"# Soft margin under the oracle criterion ({'PARTIAL: --sets' if args.sets else 'one run'})",
              "", f"{len(jobs)} routes from {', '.join(Path(s).stem for s in chosen)}."]
    write_report(run_dir, [parse_row({k: "" if v is None else str(v) for k, v in r.items()})
                           for r in rows], quant, cfg, header)
    print(f"\n-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
