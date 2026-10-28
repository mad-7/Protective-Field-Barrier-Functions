"""Re-emulate the six B1 episodes traced on 2026-10-18 under the A20 field and limits (A20 step 4).
Offline, a few minutes; no simulator.

    python scripts/reemulate_b1_traces.py --config configs/base.yaml --seed 0

The traces (`runs/diag/TRACE_20261018T173222Z_seed0`, tune_narrow 3, 6, 10, 12;
`runs/diag/TRACE_20261018T175027Z_seed0`, tune_wide 1, 4) deadlocked inside the minimum margin
under the pre-A20 field. This replays the SAME scenarios -- read from git at f839db3, before the A20
rebuild -- through the emulator (`pfbnav/eval/recovery_control.py`, whose positive control against
the simulator under A20 is `tests/test_recovery_control.py`), with B1 under the declared recovery as
in evaluation, 1,500 steps at the control's slip 0.95.
  * The narrow layouts were generated under the pre-A20 strata (narrow [0.13, 0.20]); they are
    rebuilt with those strata so the worlds match their recorded hashes. Their aisles now lie BELOW
    the A20 narrow floor (0.1735), which the new sets no longer contain -- read the narrow results
    with that in mind. The wide layouts are unchanged by A20.
  * Everything else is the configuration as it stands: the A20 field, limits and bands.

**Per stop, the two ruled outcomes:**
  * containment: how far the footprint swept from the trigger to rest reaches beyond the field in
    force at the trigger (`pfbnav/eval/containment.py`), m; <= 0 is contained. The emulator's
    sampling costs about a millimetre (360 footprint points, the 1,440-normal support grid);
  * the minimum-margin outcome: the smallest (range - at-rest field) at rest, m; < 0 means the stop
    ended inside the minimum margin.
Plus each episode's stops, recovery attempts and endings, deadlock, and progress.
"""
from __future__ import annotations

import argparse
import copy
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from pfbnav.config import load_config
from pfbnav.control.pure_pursuit import PurePursuit
from pfbnav.control.recovery import Recovery
from pfbnav.eval.containment import containment_excess, footprint_boundary, rest_margin
from pfbnav.eval.recovery_control import _layer, emulate_layout
from pfbnav.planning.astar import plan_from_config
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.baking import bake, planner_radii
from pfbnav.world.scenarios import _circumscribed, world_of

REPO_ROOT = Path(__file__).resolve().parents[1]
SOURCE_COMMIT = "f839db3"           # the sets as traced, before the A20 rebuild
TRACED = {"scenarios/tune/tune_narrow.json": (3, 6, 10, 12),
          "scenarios/tune/tune_wide.json": (1, 4)}
PRE_A20_STRATA = [{"name": "narrow", "min_clearance": 0.13, "max_clearance": 0.20},
                  {"name": "medium", "min_clearance": 0.20, "max_clearance": 0.40},
                  {"name": "wide", "min_clearance": 0.40, "max_clearance": 0.80}]
STEPS = 1500
SLIP = 0.95                         # the emulator's positive control's slip
SETTLE_LIMIT = 80                   # steps after a trigger within which the body must be at rest
GRID = np.linspace(-np.pi, np.pi, 3600, endpoint=False)   # rad, the field's evaluation grid


def _old_set(path: str) -> dict:
    text = subprocess.run(["git", "show", f"{SOURCE_COMMIT}:{path}"], cwd=REPO_ROOT, check=True,
                          capture_output=True, text=True, encoding="utf-8").stdout
    return json.loads(text)


def _strata_key(cfg: dict) -> list:
    """Where the strata live in the configuration (found, not assumed)."""
    for path in (("world", "layout", "aisle_strata"), ("world", "aisle_strata")):
        node = cfg
        for k in path:
            node = node.get(k) if isinstance(node, dict) else None
        if node is not None:
            return list(path)
    raise KeyError("aisle_strata not found in the configuration")


def stop_outcomes(records: list[dict], fields, boundary: np.ndarray) -> list[dict]:
    """Per stop trigger: containment excess and the at-rest margin once still."""
    out = []
    for i, r in enumerate(records):
        if not r["cert"]["stop_triggered"]:
            continue
        c = r["cert"]
        frame = r["decision_pose"]
        poses, j = [frame], i
        while j < len(records) and j - i <= SETTLE_LIMIT:
            poses.append(records[j]["pose"])
            nxt = records[j + 1] if j + 1 < len(records) else None
            still = nxt is not None and np.allclose(nxt["pose"], records[j]["pose"], atol=1e-7)
            if records[j]["cert"]["at_rest"] and still:
                break
            j += 1
        settled = j < len(records) and j - i <= SETTLE_LIMIT

        # The field on a 0.1 deg grid, interpolated at each swept point's bearing: a chord of a
        # convex boundary lies inside it, so interpolation can only understate the field -- the
        # excess errs toward flagging (and evaluating ~29,000 bearings directly costs ~0.3 GB).
        reach = fields.select(GRID, c["field_speed"], r["field_heading"], c["at_rest"],
                              yaw_rate=c["field_yaw_rate"])

        def field(b, reach=reach):
            return np.interp(b, GRID, reach, period=2.0 * np.pi)
        excess = containment_excess(boundary, poses, frame, field)
        rest = records[min(j + 1, len(records) - 1)]
        scan = rest["scan"]
        margin = rest_margin(scan.ranges, scan.valid,
                             fields.select(scan.bearings, 0.0, 0.0, True))
        out.append({"step": r["step"], "s_m": r["s"], "field_speed": c["field_speed"],
                    "field_yaw_rate": c["field_yaw_rate"], "settled": bool(settled),
                    "rest_step": records[min(j, len(records) - 1)]["step"],
                    "containment_excess_m": excess, "rest_margin_m": margin})
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--no-record", action="store_true", help="print only")
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    old_cfg = copy.deepcopy(cfg)
    node = old_cfg
    key = _strata_key(cfg)
    for k in key[:-1]:
        node = node[k]
    node[key[-1]] = copy.deepcopy(PRE_A20_STRATA)
    layer = _layer(cfg)
    boundary = footprint_boundary(
        FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"]).extent, 360)
    episodes = []
    for set_path, indices in TRACED.items():
        manifest = _old_set(set_path)
        for idx in indices:
            sc = manifest["scenarios"][idx]
            world = world_of(sc, old_cfg if sc["stratum"] != "wide" else cfg)
            baked = bake(world.mapped, world.bounds, float(cfg["planner"]["resolution"]),
                         planner_radii(cfg, circumscribed=_circumscribed(cfg)))
            plan = plan_from_config(baked, tuple(sc["start"][:2]), tuple(sc["goal"]), cfg)
            if not plan.ok:
                episodes.append({"set": set_path, "index": idx, "plan": plan.status})
                print(f"{Path(set_path).stem} {idx}: no plan ({plan.status})")
                continue
            controller = Recovery.from_config(PurePursuit.from_config(cfg), cfg)
            records = emulate_layout(cfg, [o.polygon for o in world.obstacles],
                                     np.asarray(plan.path_world), sc["start"], controller, SLIP,
                                     STEPS)
            stops = stop_outcomes(records, layer.fields, boundary)
            report = controller.episode_report()
            ep = {"set": set_path, "index": idx, "seed": sc["seed"], "stratum": sc["stratum"],
                  "plan_length_m": float(plan.length_m), "final_s_m": records[-1]["s"],
                  "min_centre_clearance_m": min(r["centre_clearance"] for r in records),
                  "stops": stops, "recovery": {k: report[k] for k in (
                      "recovery_attempts", "recovery_endings", "recovery_deadlock",
                      "recovery_gave_up")}}
            episodes.append(ep)
            worst_c = max((s["containment_excess_m"] for s in stops), default=None)
            worst_m = min((s["rest_margin_m"] for s in stops), default=None)
            print(f"{Path(set_path).stem} {idx}: {len(stops)} stops, progress "
                  f"{ep['final_s_m']:.2f}/{ep['plan_length_m']:.2f} m, recovery "
                  f"{report['recovery_attempts']} {report['recovery_endings']}, deadlock "
                  f"{report['recovery_deadlock']}; worst containment excess "
                  + ("-" if worst_c is None else f"{worst_c * 1e3:+.1f} mm")
                  + "; smallest rest margin "
                  + ("-" if worst_m is None else f"{worst_m * 1e3:+.1f} mm"))
    all_stops = [s for e in episodes for s in e.get("stops", [])]
    summary = {"source_commit": SOURCE_COMMIT, "slip": SLIP, "steps": STEPS,
               "stops": len(all_stops),
               "max_containment_excess_m": max((s["containment_excess_m"] for s in all_stops),
                                               default=None),
               "stops_ending_inside_min_margin": sum(s["rest_margin_m"] < 0 for s in all_stops),
               "unsettled_stops": sum(not s["settled"] for s in all_stops),
               "episodes": episodes}
    print(f"all: {summary['stops']} stops; max containment excess "
          f"{(summary['max_containment_excess_m'] or 0.0) * 1e3:+.1f} mm; stops ending inside "
          f"the minimum margin: {summary['stops_ending_inside_min_margin']}; unsettled "
          f"{summary['unsettled_stops']}")
    if args.no_record:
        return 0
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "analysis", f"REEMULATE_B1_{stamp}", args.seed,
                             cfg)
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=float),
                                          encoding="utf-8")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
