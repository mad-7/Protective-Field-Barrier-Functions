"""B2 against B7 on B7's horizon timeouts (ruled 2026-11-03): is B7's slowness RPP's own regulation or
the filter? Read-only, seconds.

    python scripts/compare_route_speeds.py --config configs/base.yaml --seed 0 --classification runs/diag/B7TIMEOUTS_20261103T174622Z_seed0 --b2-tuning runs/tuning/B2_regulated_pure_pursuit_20261030T111716Z_seed0

Reads the classification's `summary.json` (the routes of class `filter_deadlock`, ruled to be horizon
timeouts of a slow controller, and the B7 run it read), B7's episodes at the frozen alpha, and B2's
episodes for its selected candidate (`summary.json` `selected`). Per route and pooled: outcome,
distance, mean speed over the episode, and mean speed over the steps not latched by the certified
layer (`pfbnav.eval.timeout_classes.route_speeds`). B2's failures are stop-recovery-restop cycles, so
the not-latched speed is the comparison of driving speed. Writes
`runs/diag/ROUTESPEED_<timestamp>_seed<seed>/summary.json`.
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.eval.timeout_classes import DEADLOCK, route_speeds
from pfbnav.utils.runio import create_run_dir

REPO_ROOT = Path(__file__).resolve().parents[1]


def _episodes(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def _pooled(rows: list[dict], dt: float) -> dict:
    d = sum(r["distance_m"] for r in rows)
    steps = sum(r["steps"] for r in rows)
    moving = steps - sum(r["stopped_steps"] for r in rows)
    return {"distance_m": d, "mean_speed_mps": d / (steps * dt),
            "moving_speed_mps": d / (moving * dt) if moving else None,
            "latched_share": 1 - moving / steps,
            "protective_stops": sum(r["protective_stops"] for r in rows),
            "outcomes": {o: sum(r["outcome"] == o for r in rows) for o in sorted({r["outcome"] for r in rows})}}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--classification", required=True, help="a runs/diag/B7TIMEOUTS_* directory")
    parser.add_argument("--b2-tuning", required=True, help="B2's tuning run directory")
    args = parser.parse_args()
    cfg = load_config(args.config)
    dt = float(cfg["sim"]["dt"])
    cls = json.loads((REPO_ROOT / args.classification / "summary.json").read_text(encoding="utf-8"))
    seeds = list(cls["counts"]["all"][DEADLOCK]["ids"])
    b7_run = REPO_ROOT / cls["tuning_run"]
    b7_summary = json.loads((b7_run / "summary.json").read_text(encoding="utf-8"))
    b7_index = [float(a) for a in b7_summary["grid_per_s"]].index(float(cls["selected_alpha"]))
    b2_run = REPO_ROOT / args.b2_tuning
    b2_selected = int(json.loads((b2_run / "summary.json").read_text(encoding="utf-8"))["selected"])
    b7 = route_speeds(_episodes(b7_run / f"candidate_{b7_index}" / "episodes.jsonl"), seeds, dt)
    b2 = route_speeds(_episodes(b2_run / f"candidate_{b2_selected}" / "episodes.jsonl"), seeds, dt)
    print(f"{len(seeds)} routes; B7 alpha {cls['selected_alpha']} (candidate_{b7_index}), "
          f"B2 candidate_{b2_selected}")
    print("route | B7 outcome, m, mean / moving m/s | B2 outcome, m, mean / moving m/s, latched share")
    for a, b in zip(b7, b2, strict=True):
        mv = "-" if b["moving_speed_mps"] is None else f"{b['moving_speed_mps']:.3f}"
        print(f"  {a['scenario_seed']} | {a['outcome']:<8} {a['distance_m']:5.1f} "
              f"{a['mean_speed_mps']:.3f} / {a['moving_speed_mps']:.3f} | {b['outcome']:<18} "
              f"{b['distance_m']:5.1f} {b['mean_speed_mps']:.3f} / {mv}, "
              f"{b['stopped_steps'] / b['steps']:.2f}")
    pooled = {"B7": _pooled(b7, dt), "B2": _pooled(b2, dt)}
    for name, p in pooled.items():
        print(f"  pooled {name}: mean {p['mean_speed_mps']:.3f} m/s, moving "
              f"{p['moving_speed_mps']:.3f} m/s, latched {p['latched_share']:.2f}, "
              f"stops {p['protective_stops']}, {p['outcomes']}")
    out_dir = create_run_dir(REPO_ROOT / "runs" / "diag",
                             f"ROUTESPEED_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}", args.seed, cfg)
    (out_dir / "summary.json").write_text(json.dumps({
        "classification": args.classification, "b2_tuning": args.b2_tuning, "routes": seeds,
        "b7": b7, "b2": b2, "pooled": pooled}, indent=2), encoding="utf-8")
    print(f"-> {out_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
