"""Re-tune the synthetic recovery scene on its PREMISE only (ruling (c), 2026-10-16). No simulator.

    python scripts/search_recovery_scene.py --config configs/base.yaml --seed 0

Every candidate geometry declared in docs/decision_log.md (2026-10-16, "how the synthetic scene is
re-tuned") is emulated with bare B1 at every declared slip, up to and including the rest state
after the first stop -- never beyond, so release, progress and contact are not computed and cannot
steer the choice. The selection rule is the declared one: among candidates whose premise holds at
every slip, the largest worst-case block depth, then the largest worst-case at-rest margin, then the
smallest (cy, gx, gy). Writes `runs/diag/SEARCH_RECOVERY_SCENE_<timestamp>_seed<seed>/summary.json`.
"""
from __future__ import annotations

import argparse
import itertools
import json
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.control.pure_pursuit import PurePursuit
from pfbnav.eval.recovery_control import (
    ControlSpec,
    _layer,
    emulate,
    plan_route,
    premise,
    premise_holds,
    route_clearance,
)
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
CYS = (0.40, 0.45, 0.50, 0.55, 0.60)
GXS = (0.6, 0.8, 1.0, 1.2)
GYS = (0.4, 0.6, 0.8, 1.2)
STEPS = 240              # the first stop must come within 200; the rest state follows within ~30


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    base = ControlSpec.from_file(REPO_ROOT / "configs" / "controls" / "recovery_synthetic.yaml")
    layer = _layer(cfg)
    rows = []
    for cy, gx, gy in itertools.product(CYS, GXS, GYS):
        spec = replace(base, corner=(base.corner[0], cy), goal_from_corner=(gx, gy))
        try:
            route = plan_route(cfg, spec)
        except ValueError as e:
            rows.append({"cy": cy, "gx": gx, "gy": gy, "error": str(e)})
            continue
        clearance = route_clearance(route, spec)
        by_slip = {}
        for slip in spec.slips:
            recs = emulate(cfg, spec, route, PurePursuit.from_config(cfg), slip, STEPS)
            by_slip[str(slip)] = premise(recs, layer, spec.stop_within_steps)
        holds = clearance >= spec.min_route_clearance and all(
            premise_holds(p, spec) for p in by_slip.values())
        row = {"cy": cy, "gx": gx, "gy": gy, "route_clearance": clearance, "holds": holds,
               "by_slip": by_slip}
        if holds:
            row["worst_block_depth"] = min(p["block_depth"] for p in by_slip.values())
            row["worst_at_rest_margin"] = min(p["at_rest_margin"] for p in by_slip.values())
        rows.append(row)
        print(f"cy {cy} g ({gx},{gy}) holds {holds}"
              + (f" depth {1e3 * row['worst_block_depth']:.1f} mm margin "
                 f"{1e3 * row['worst_at_rest_margin']:.1f} mm" if holds else ""), flush=True)
    ok = [r for r in rows if r.get("holds")]
    chosen = (max(ok, key=lambda r: (r["worst_block_depth"], r["worst_at_rest_margin"],
                                     -r["cy"], -r["gx"], -r["gy"])) if ok else None)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"SEARCH_RECOVERY_SCENE_{stamp}",
                             args.seed, cfg)
    (run_dir / "summary.json").write_text(json.dumps({"chosen": chosen, "candidates": rows},
                                                     indent=2, default=float), encoding="utf-8")
    print(f"chosen: {None if chosen is None else (chosen['cy'], chosen['gx'], chosen['gy'])}"
          f" -> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
