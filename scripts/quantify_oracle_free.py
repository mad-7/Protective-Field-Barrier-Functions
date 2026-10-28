"""The soft margin under the FREE-ORIENTATION oracle at the ruled lethal margin (0.07 ruled
2026-10-15, second ruling: `planner_oracle_free.yaml`; 0.10 ruled 2026-10-16, before A18:
`planner_oracle_free_l010.yaml`).
Offline; the RESEARCHER runs it (hours of CPU: hard rule 11).

    python scripts/quantify_oracle_free.py --config configs/analysis/planner_oracle_free.yaml \
        --seed 0 --workers 6 --phase decide
    python scripts/quantify_oracle_free.py --config configs/analysis/planner_oracle_free.yaml \
        --seed 0 --workers 6 --phase info --soft <the chosen soft margin>
    # an interrupted run: complete the missing sets with --sets, then merge with --report

**Why a second oracle.** The first turned the robot's body to face the route's tangent; the
criterion never said so, and a mecanum base need not. Under that assumption narrow routes left a
16.2 % residual; with one body offset held through each breached stretch, 142 of the 143 cleared.
Ruled: formalise the oracle with free orientation (`pfbnav/world/oracle.py`) -- yaw continuity,
yaw rate <= w_max, yaw acceleration <= alpha_max (VERIFY), the wheel budget at the nominal share
(rotation consumes it; when it binds the oracle drops to a slower band), the field along the velocity
direction and the footprint rotating with the body -- and rerun; the residual rule decides unchanged.

**Phase `decide`** -- every candidate soft margin at the config's ONE lethal margin. Per route: whether a continuous yaw
path keeps band 0 clear from the start yaw to the goal (`reach`); the free floor (separate stretches
where none survives) and where each begins; start and goal validity; and the TANGENT-LOCKED floor,
A1's own (ruled: reported beside it). The decisions are declared in `pfbnav/eval/oracle_report.py`
(`summarise_free`): the soft margin with the fewest routes stopping at a bend, then the residual rule
for it. Feasibility needs no speed: every limit scales down with speed and band 0 admits any speed
up to its top, so the limits shape the speed profile, not whether a route can be driven.

**Phase `info`** -- at the chosen soft margin, information only: the oracle's mission time against
constant top speed (bands 1 and 2 on the reachable states, then `fastest_path_time`), and the 50 mm
inside-bend breach rate at lethal 0.07 and 0.10 with narrow plannability at each (ruled).

The floor is platform-specific -- a holonomic base with full route knowledge -- and every report
says so (ruled 2026-10-15).

Writes `runs/analysis/ORACLE_FREE_<phase>_<timestamp>_seed<seed>/`: routes.csv, quantisation.json,
summary.json, report.md.
"""
from __future__ import annotations

import argparse
import copy
import csv
import importlib.util
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.eval.margin_report import merge_runs
from pfbnav.eval.oracle_report import (
    FLOOR_NOTE,
    RESIDUAL_LIMIT,
    parse_free_row,
    summarise_free,
    summarise_free_info,
)
from pfbnav.planning.astar import plan_from_config
from pfbnav.safety.certified import FieldGeometry, band_speed_limit
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.baking import bake, planner_radii
from pfbnav.world.oracle import (
    Limits,
    fastest_path_time,
    rate_levels,
    reach,
    state_tables,
    tracking_yaw_rates,
    yaw_set,
)
from pfbnav.world.route_check import (
    FieldShape,
    RouteSamples,
    arc_length,
    clearance_to_field,
    location_classes,
    sample_route,
    stop_runs,
)
from pfbnav.world.scenarios import Scenario, load_set, world_of
from pfbnav.world.strata import certified_params_from_config

REPO_ROOT = Path(__file__).resolve().parents[1]
TEST_DIR = (REPO_ROOT / "scenarios" / "test").resolve()
DECIDE_FIELDS = ("set", "seed", "stratum", "lethal_margin", "soft_margin", "status",
                 "path_length_m", "samples", "start_valid", "goal_valid", "traversable",
                 "unavoidable_stops", "stops_start", "stops_goal", "stops_bend", "stops_straight",
                 "tangent_stops")
INFO_FIELDS = ("set", "seed", "stratum", "lethal_margin", "soft_margin", "status",
               "path_length_m", "samples", "traversable", "oracle_time_s", "constant_time_s",
               "offset_traversable")

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
    a = cfg["actuation"]
    b = a["wheel_budget"]
    _W.update(cfg=cfg, circumscribed=float(table.circumscribed),
              fields=FieldGeometry(table.extent, params),
              tops=[float(t) for t in params.band_top_speeds], cache={},
              # A20: the speed a tracker may drive at in each band, top - a_max T (the top band:
              # the speed cap), so its reachable speed stays in the band.
              speeds=[band_speed_limit(i, params) for i in range(len(params.band_top_speeds))],
              qm=sys.modules.get("quantify_margins") or _margins_module(),
              lim=Limits(v_max=float(a["v_max"]), vy_max=float(a["vy_max"]),
                         w_max=float(a["w_max"]), alpha_max=float(a["alpha_max"]),
                         budget=float(b["nominal_fraction"]) * float(b["v_rim"]),
                         lever_arm=float(cfg["robot"]["half_wheelbase"])
                         + float(cfg["robot"]["half_track"]),
                         decel=float(a["a_decel_max"])))


def _shape_for(band: float):
    cache = _W["cache"]

    def get(heading: float, yaw_rate: float = 0.0) -> FieldShape:
        key = (band, round(heading, 9), round(yaw_rate, 9))
        if key not in cache:
            cache[key] = FieldShape.moving(_W["fields"], band, heading=heading,
                                           yaw_rate=yaw_rate)
        return cache[key]
    return get


def _rates(samples):
    """The rotation sweep's yaw rates: tangent tracking at band 0's speed over one lookahead
    (ruled 2026-10-16; band 0's speed under A20), as the generator's validity check uses them."""
    return tracking_yaw_rates(samples, float(_W["cfg"]["planner"]["lookahead"]), _W["speeds"][0])


def _rate_step() -> float:
    return float(_W["cfg"]["world"]["layout"]["validity"]["yaw_rate_step"])


def _route(scenario: Scenario, layout, lethal: float, soft: float):
    c = _W["qm"].with_margins(_W["cfg"], lethal, soft)
    baked = bake(layout.obstacles, layout.bounds, float(c["planner"]["resolution"]),
                 planner_radii(c, circumscribed=_W["circumscribed"]))
    return plan_from_config(baked, scenario.start[:2], scenario.goal, c)


def _orientations(scenario: Scenario, samples):
    """The start-anchored grid plus the route's own segment directions (`yaw_set`), and the start
    yaw's index: the orientation along the route must be available (found 2026-10-15)."""
    step = math.radians(float(_W["cfg"]["oracle_free"]["yaw_step_deg"]))
    return yaw_set(float(scenario.start[2]), step, extra=np.unique(np.round(samples.yaw, 12)))


def _band0(samples, mapped, yaws, start):
    of = _W["cfg"]["oracle_free"]
    gaps = state_tables(samples, mapped, yaws, _shape_for(_W["tops"][0]),
                        heading_round=math.radians(float(of["heading_round_deg"])),
                        coarse_stride=int(of["coarse_stride"]), yaw_rates=_rates(samples),
                        rate_step=_rate_step())
    return reach(gaps >= 0, start_index=start)


def decide(job: tuple[str, dict[str, Any]]) -> list[dict[str, Any]]:
    set_name, record = job
    cfg = _W["cfg"]
    pm, of = cfg["planner_margins"], cfg["oracle_free"]
    scenario = Scenario(**record)
    layout = world_of(scenario, cfg)
    mapped = [o.polygon for o in layout.mapped]
    rows = []
    for lethal in pm["lethal_margins"]:
        for soft in pm["soft_margins"]:
            plan = _route(scenario, layout, lethal, soft)
            row: dict[str, Any] = {"set": set_name, "seed": scenario.seed,
                                   "stratum": scenario.stratum, "lethal_margin": lethal,
                                   "soft_margin": soft, "status": plan.status}
            if plan.ok:
                s = sample_route(plan.path_world, float(pm["sample_spacing_m"]))
                yaws, start = _orientations(scenario, s)
                r = _band0(s, mapped, yaws, start)
                where = location_classes(s, start_window=float(pm["start_window_m"]),
                                         goal_window=float(pm["goal_window_m"]),
                                         bend_window=float(pm["bend_window_m"]))
                at = [str(where[i]) for i in r.deaths]
                # A1's tangent-locked floor: the body along the tangent, so heading 0, with the
                # same rotation sweep (one shape per rate level).
                levels = rate_levels(_rates(s), _rate_step())
                tangent = np.empty(s.x.size)
                for level in np.unique(levels):
                    idx = np.flatnonzero(levels == level)
                    sub = RouteSamples(s.x[idx], s.y[idx], s.yaw[idx], s.inside[idx])
                    tangent[idx] = clearance_to_field(
                        sub, mapped, _shape_for(_W["tops"][0])(0.0, float(level)),
                        coarse_stride=int(of["coarse_stride"]))
                row.update(path_length_m=float(arc_length(s)[-1]), samples=int(s.x.size),
                           start_valid=int(r.start_valid), goal_valid=int(r.goal_valid),
                           traversable=int(r.traversable), unavoidable_stops=r.stops,
                           tangent_stops=stop_runs(tangent < 0),
                           **{f"stops_{k}": at.count(k)
                              for k in ("start", "goal", "bend", "straight")})
            rows.append(row)
    return rows


def info(job: tuple[str, dict[str, Any]]) -> list[dict[str, Any]]:
    set_name, record = job
    cfg = _W["cfg"]
    pm, of = cfg["planner_margins"], cfg["oracle_free"]
    soft = float(_W["soft"])
    scenario = Scenario(**record)
    layout = world_of(scenario, cfg)
    mapped = [o.polygon for o in layout.mapped]
    rows = []
    for lethal in of["info_lethal_margins"]:
        plan = _route(scenario, layout, float(lethal), soft)
        row: dict[str, Any] = {"set": set_name, "seed": scenario.seed, "stratum": scenario.stratum,
                               "lethal_margin": float(lethal), "soft_margin": soft,
                               "status": plan.status}
        if plan.ok:
            s = sample_route(plan.path_world, float(pm["sample_spacing_m"]))
            yaws, start = _orientations(scenario, s)
            r = _band0(s, mapped, yaws, start)
            arc = arc_length(s)
            oracle_time = None
            if r.traversable:
                hr = math.radians(float(of["heading_round_deg"]))
                stride = int(of["coarse_stride"])
                g1 = state_tables(s, mapped, yaws, _shape_for(_W["tops"][1]), mask=r.reachable,
                                  heading_round=hr, coarse_stride=stride)
                clear1 = r.reachable & (g1 >= 0)
                g2 = state_tables(s, mapped, yaws, _shape_for(_W["tops"][2]), mask=clear1,
                                  heading_round=hr, coarse_stride=stride)
                # A20: each band's speed limit, not its top (a tracker at the top would reach
                # the band above within T).
                caps = np.where(r.reachable, _W["speeds"][0], 0.0)
                caps = np.where(clear1, _W["speeds"][1], caps)
                caps = np.where(clear1 & (g2 >= 0), _W["speeds"][2], caps)
                oracle_time = fastest_path_time(arc, s.yaw, yaws, r.reachable, caps, _W["lim"],
                                                start_index=start)
            offset = _band0(s.offset(float(pm["inside_offset_m"])), mapped, yaws, start)
            row.update(path_length_m=float(arc[-1]), samples=int(s.x.size),
                       traversable=int(r.traversable), oracle_time_s=oracle_time,
                       constant_time_s=float(arc[-1]) / min(_W["speeds"][-1], _W["lim"].v_max),
                       offset_traversable=int(offset.traversable))
        rows.append(row)
    return rows


def write_decide_report(out: Path, rows, quant, cfg, header) -> dict[str, Any]:
    pm = cfg["planner_margins"]
    result = summarise_free(rows, pm["soft_margins"])
    qf = {float(q["soft_margin"]): len(q["failures"]) for q in quant}
    for soft, entry in result["by_soft"].items():
        entry["quantisation_failures"] = qf.get(soft, 0)
    (out / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    lines = [*header, "",
             (f"Lethal margin {float(pm['lethal_margins'][0]):.2f} (ruled). Free-orientation "
              "oracle: a continuous yaw path within the "
              "yaw-rate, yaw-acceleration and wheel-budget limits, field along the velocity, "
              "footprint rotating with the body; criterion: zero routes with a band-0 breach."),
             "", f"*{FLOOR_NOTE}*", "",
             ("| soft | plannable | quant. fails | start-invalid | goal-invalid | residual | "
              "bend-stop routes | free floor /100 m | A1 (tangent) floor /100 m | residual rule |"),
             "|---|---|---|---|---|---|---|---|---|---|"]
    for soft, e in result["by_soft"].items():
        a = e["all"]
        lines.append(
            f"| {soft:.2f} | {a['plannable']}/{a['routes']} | {e['quantisation_failures']} | "
            f"{a['start_invalid']} | {a['goal_invalid']} | {a['residual']} | "
            f"{a['bend_stop_routes']} | {a['unavoidable_stops_per_100m']:.3f} | "
            f"{a['a1_tangent_stops_per_100m']:.3f} | {e['residual_rule']} |")
    chosen = result["chosen_soft"]
    lines += ["", (f"**Chosen soft margin (fewest bend stops, plannability kept):** {chosen}. "
                   f"**Residual rule** (<= {RESIDUAL_LIMIT:.0%} in every stratum): "
                   f"{result['residual_rule']}."), ""]
    if chosen is not None:
        lines += [f"Per stratum at soft {chosen}:", "",
                  ("| stratum | routes | start-invalid | goal-invalid | residual (share) | "
                   "bend-stop routes | free floor /100 m | A1 floor /100 m |"),
                  "|---|---|---|---|---|---|---|---|"]
        for s, b in result["by_soft"][chosen]["per_stratum"].items():
            lines.append(
                f"| {s} | {b['routes']} | {b['start_invalid']} | {b['goal_invalid']} | "
                f"{b['residual']} ({b['residual_share']:.1%}) | {b['bend_stop_routes']} | "
                f"{b['unavoidable_stops_per_100m']:.3f} | {b['a1_tangent_stops_per_100m']:.3f} |")
    (out / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return result


def write_info_report(out: Path, rows, quant, header) -> dict[str, Any]:
    result = {"by_lethal": summarise_free_info(rows), "floor_note": FLOOR_NOTE,
              "quantisation": quant}
    (out / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    lines = [*header, "", f"*{FLOOR_NOTE}*", "",
             ("| lethal | stratum | plannable | quant. fails (stratum) | 50 mm inside: routes "
              "breaching band 0 (share) | mission time / constant (median) |"),
             "|---|---|---|---|---|---|"]
    for lethal, per in result["by_lethal"].items():
        for s, b in per.items():
            qf = sum(len(q["failures"]) for q in quant
                     if float(q["lethal_margin"]) == float(lethal) and q["stratum"] == s)
            ratio = b["mission_time_ratio_median"]
            share = b["offset_breach_share"]
            lines.append(
                f"| {float(lethal):.2f} | {s} | {b['plannable']}/{b['routes']} | {qf} | "
                f"{b['offset_breach_routes']} ({'-' if share is None else f'{share:.1%}'}) | "
                f"{'-' if ratio is None else f'{ratio:.3f}'} |")
    (out / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--phase", choices=["decide", "info"], required=True)
    parser.add_argument("--soft", type=float, default=None,
                        help="phase info: the soft margin chosen by phase decide")
    parser.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    parser.add_argument("--sets", nargs="+", default=None,
                        help="only these of the configured sets (to complete an interrupted run)")
    parser.add_argument("--report", nargs="+", default=None,
                        help="merge these run directories of the same phase; runs nothing")
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    pm, of = cfg["planner_margins"], cfg["oracle_free"]
    if args.phase == "info":
        if args.soft is None:
            raise SystemExit("--phase info needs --soft, the margin phase decide chose")
        if args.soft not in [float(s) for s in pm["soft_margins"]]:
            raise SystemExit(f"--soft {args.soft} is not a configured candidate")
        pairs = {(float(lm), float(args.soft)) for lm in of["info_lethal_margins"]}
    else:
        if len(pm["lethal_margins"]) != 1:
            raise SystemExit("--phase decide chooses a soft margin at ONE ruled lethal margin; the "
                             f"config lists {pm['lethal_margins']}")
        pairs = {(float(lm), float(sm)) for lm in pm["lethal_margins"] for sm in pm["soft_margins"]}
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    tag = f"ORACLE_FREE_{args.phase}"

    if args.report:
        dirs = [REPO_ROOT / r for r in args.report]
        lists, quant, inputs = [], None, []
        for d in dirs:
            with (d / "routes.csv").open(encoding="utf-8") as f:
                lists.append([parse_free_row(r) for r in csv.DictReader(f)])
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
        out = create_run_dir(REPO_ROOT / "runs" / "analysis", f"{tag}_REPORT_{stamp}", args.seed,
                             cfg)
        header = [(f"# Free-orientation oracle, phase {args.phase} "
                  f"({'PARTIAL' if missing else 'COMPLETE'})"), "",
                  (f"{len(merged['scenarios'])} routes; inputs: {', '.join(inputs)}; "
                  f"{merged['duplicates']} routes in more than one run, all agreeing.")]
        if missing:
            header += ["", f"**PARTIAL -- not the verdict.** Missing: {missing}"]
        if args.phase == "decide":
            write_decide_report(out, merged["rows"], quant or [], cfg, header)
        else:
            write_info_report(out, merged["rows"], quant or [], header)
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

    run_dir = create_run_dir(REPO_ROOT / "runs" / "analysis", f"{tag}_{stamp}", args.seed, cfg)
    qm = _margins_module()
    _init(cfg)
    qcfg = copy.deepcopy(cfg)
    if args.phase == "info":
        qcfg["planner_margins"]["lethal_margins"] = list(of["info_lethal_margins"])
        qcfg["planner_margins"]["soft_margins"] = [float(args.soft)]
    quant = qm.quantisation(qcfg, _W["circumscribed"])
    (run_dir / "quantisation.json").write_text(json.dumps(quant, indent=2), encoding="utf-8")
    fields = DECIDE_FIELDS if args.phase == "decide" else INFO_FIELDS
    worker = decide if args.phase == "decide" else info
    print(f"phase {args.phase}: {len(jobs)} scenarios x {len(pairs)} (lethal, soft) pairs, "
          f"{args.workers} workers; yaw grid {of['yaw_step_deg']} deg")

    rows: list[dict[str, Any]] = []
    with (run_dir / "routes.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(fields))
        writer.writeheader()
        with ProcessPoolExecutor(max_workers=args.workers, initializer=_worker_init,
                                 initargs=(cfg, args.soft)) as pool:
            for i, result in enumerate(pool.map(worker, jobs, chunksize=2), start=1):
                rows += result
                writer.writerows(result)
                f.flush()
                if i % 50 == 0 or i == len(jobs):
                    print(f"  {i}/{len(jobs)} scenarios", flush=True)

    parsed = [parse_free_row({k: "" if v is None else str(v) for k, v in r.items()}) for r in rows]
    header = [(f"# Free-orientation oracle, phase {args.phase} "
              f"({'PARTIAL: --sets' if args.sets else 'one run'})"), "",
              f"{len(jobs)} routes from {', '.join(Path(s).stem for s in chosen)}."]
    if args.phase == "decide":
        write_decide_report(run_dir, parsed, quant, cfg, header)
    else:
        write_info_report(run_dir, parsed, quant, header)
    print(f"\n-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


def _worker_init(cfg: dict[str, Any], soft: float | None) -> None:
    _init(cfg)
    _W["soft"] = soft


if __name__ == "__main__":
    raise SystemExit(main())
