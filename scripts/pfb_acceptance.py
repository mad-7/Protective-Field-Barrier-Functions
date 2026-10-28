"""The PFB acceptance step (WP2, ruled 2026-10-23): an ideal tracker plus the PFB on every tune route,
in the emulator, with the certified layer beneath. Offline, no simulator; the RESEARCHER runs it
(an evaluation over a scenario set: hard rule 11).

    python scripts/pfb_acceptance.py --config configs/base.yaml --seed 0 --workers 4

**Declared before running** (ruled 2026-10-23; `pfbnav/eval/pfb_acceptance.py`):
  * the 60 tune routes (tune_narrow, tune_medium, tune_wide), each planned exactly as `NavEnv` plans
    it, driven by the ideal tracker (`baselines.ideal_tracker`) through the PFB (`filter:`, alpha
    provisional) and the certified layer, at the emulator's declared slip 0.95, until the goal
    (`reward.goal_tolerance`), `reward.max_steps`, or a STALL: route progress under 0.05 m in the
    last 30 s (declared 2026-10-23: the ideal tracker is blind and never replans, so on the 19 tune
    routes with an unmapped obstacle near the route it can stall indefinitely, at about 1 s of compute
    per step. Thirty seconds of stall exercises the at-rest regime, the creep and the restart many
    times over; the stall is the nominal's, recorded and counted, not a verdict criterion);
  * the scan is the simulator's FILTER stream -- per 3 deg sector the nearest of 13 rays
    (`scanners.rays_per_degree` 4.27 x 3 deg, the measured pixel density) at its exact bearing --
    not one ray per bin centre (declared 2026-10-24 after the first run's one unattributed stop:
    docs/decision_log.md);
  * every certified stop on static geometry attributed: unobserved, infeasible, creep, or
    prediction (regime / heading / yaw / motion). **Unattributed stops must be zero; otherwise stop
    and report;**
  * from every fallback state, pure maximum braking rolled forward must never touch anything (Q3);
  * the footprint must never touch anything along any run.

Writes `runs/analysis/PFB_ACCEPTANCE_<stamp>_seed<seed>/`: routes.json (per route: reached, steps,
every stop with its attribution, fallback steps, rollout contacts, contact steps) and summary.json
(counts, verdict).
"""
from __future__ import annotations

import argparse
import json
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.control.ideal_tracker import IdealTracker
from pfbnav.eval.pfb_acceptance import (
    attribute_stops,
    braking_contact,
    footprint_polygon,
    in_contact,
    summarise,
    yaw_profile,
    yaw_summary,
)
from pfbnav.eval.recovery_control import emulate_layout
from pfbnav.planning.astar import plan_from_config
from pfbnav.safety import pfb_field as pf
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.safety.pfb import PFB, PFBConfig
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.baking import bake, planner_radii
from pfbnav.world.scenarios import Scenario, load_set, world_of

REPO_ROOT = Path(__file__).resolve().parents[1]
SETS = ("scenarios/tune/tune_narrow.json", "scenarios/tune/tune_medium.json",
        "scenarios/tune/tune_wide.json")
SLIP = 0.95            # the emulator's declared slip (its positive control, 2026-10-16)
STALL_S, STALL_M = 30.0, 0.05   # s, m: declared stall cut-off (module docstring)


def sector_rays(cfg: dict[str, Any]) -> int:
    """Rays per filter sector at the scanners' measured density (13 for 3 deg at 4.27 / deg)."""
    s = cfg["scanners"]
    return max(1, round(float(s["rays_per_degree"]) * 360.0 / int(s["filter_bins"])))


def run_route(job: tuple[dict[str, Any], dict[str, Any], int | None]) -> dict[str, Any]:
    record, cfg, max_steps = job
    from shapely.geometry import Polygon

    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    field = pf.PFBField(table.extent, pf.field_params_from_config(cfg))
    pfb = PFB(field, PFBConfig.from_config(cfg, field))
    scenario = Scenario(**record)
    layout = world_of(scenario, cfg)
    baked = bake(layout.obstacles, layout.bounds, float(cfg["planner"]["resolution"]),
                 planner_radii(cfg, circumscribed=float(table.circumscribed)))
    plan = plan_from_config(baked, scenario.start[:2], scenario.goal, cfg)
    polygons = [o.polygon for o in layout.obstacles]
    steps = int(max_steps or cfg["reward"]["max_steps"])
    dt = float(cfg["sim"]["dt"])
    records = emulate_layout(cfg, polygons, np.asarray(plan.path_world), scenario.start,
                             IdealTracker.from_config(cfg), SLIP, steps, pfb=pfb,
                             goal=tuple(scenario.goal),
                             goal_tolerance=float(cfg["reward"]["goal_tolerance"]),
                             stall=(round(STALL_S / dt), STALL_M),
                             sector_rays=sector_rays(cfg))
    footprint = footprint_polygon(table.extent)
    obstacles = [Polygon(np.asarray(p, dtype=float)[:, :2]) for p in polygons]
    contact_steps = sum(int(in_contact(r["pose"], footprint, obstacles)) for r in records)
    fallbacks = [r for r in records if r["pfb"] and r["pfb"]["infeasible"]]
    rollout = sum(int(braking_contact(r["decision_pose"], r["pfb"]["applied_before"], pfb,
                                      footprint, obstacles, dt=dt, slip=SLIP))
                  for r in fallbacks)
    reasons: dict[str, int] = {}
    for r in fallbacks:
        reasons[r["pfb"]["reason"]] = reasons.get(r["pfb"]["reason"], 0) + 1
    return {"seed": scenario.seed, "stratum": scenario.stratum,
            "plan_length_m": float(plan.length_m), "steps": len(records),
            "reached_goal": bool(records and records[-1].get("reached_goal")),
            "stalled": bool(records and records[-1].get("stalled")),
            "final_s_m": float(records[-1]["s"]) if records else 0.0,
            "stops": attribute_stops(records, dt), "fallback_steps": len(fallbacks),
            "_yaw": yaw_profile(records, np.asarray(plan.path_world)),
            "fallback_reasons": reasons, "fallback_rollout_contacts": rollout,
            "contact_steps": contact_steps,
            "modes": {m: sum(1 for r in records if r["pfb"] and r["pfb"]["mode"] == m)
                      for m in ("nominal", "filtered", "brake", "fallback", "latched")}}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--max-steps", type=int, default=None,
                        help="a SMOKE run's cap; the acceptance step runs to reward.max_steps")
    parser.add_argument("--limit", type=int, default=None, help="a SMOKE run: the first N routes")
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    records = [r for s in SETS for r in load_set(REPO_ROOT / s)["scenarios"]]
    if args.limit:
        records = records[:args.limit]
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    smoke = bool(args.limit or args.max_steps)
    run_dir = create_run_dir(REPO_ROOT / "runs" / "analysis",
                             f"PFB_ACCEPTANCE{'_SMOKE' if smoke else ''}_{stamp}", args.seed, cfg)
    jobs = [(r, cfg, args.max_steps) for r in records]
    routes: list[dict[str, Any]] = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for i, res in enumerate(pool.map(run_route, jobs), start=1):
            routes.append(res)
            cats = [st["category"] for st in res["stops"]]
            print(f"[{i}/{len(jobs)}] {res['stratum']} {res['seed']}: "
                  f"{'goal' if res['reached_goal'] else 'stalled' if res['stalled'] else 'no goal'}"
                  f" in {res['steps']} steps, "
                  f"stops {cats}, fallbacks {res['fallback_steps']}, rollout contacts "
                  f"{res['fallback_rollout_contacts']}, contact steps {res['contact_steps']}",
                  flush=True)
    # The yaw rates used, by field in force and at bends (ruled 2026-10-26, the realism audit's
    # rotation row); the per-step profiles are pooled here and not kept per route.
    summary = summarise(routes)
    summary["yaw_by_field"] = yaw_summary([r.pop("_yaw") for r in routes])
    summary["smoke"] = smoke
    (run_dir / "routes.json").write_text(json.dumps(routes, indent=1, default=float),
                                         encoding="utf-8")
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0 if summary["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
