"""The privileged oracle schedule of every route in every built set: the horizon's evidence (ruled
2026-10-20, A23; `pfbnav/eval/horizon.py`). Offline, no simulator; the RESEARCHER runs it (about
an hour of CPU over the 2,271 built routes at 4 workers: hard rule 11).

    python scripts/route_schedules.py --config configs/base.yaml --seed 0 --workers 4

Every set under `scenarios/tune`, `scenarios/pilot` and `scenarios/train`, and -- as ruled -- under
`scenarios/test` once built (only to check the horizon; nothing is tuned on them). Per route: whether
the privileged schedule exists (an unmapped obstacle across the mapped route makes it not), its
yaw-rate allowance and its mission time. Writes `runs/analysis/ROUTE_SCHEDULES_<stamp>_seed<seed>/`
(schedules.json, summary.json) and `tests/data/route_schedules.json`, which `tests/test_horizon.py`
checks against the sets on disk and `reward.max_steps`; commit that file by name.
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from pfbnav.config import load_config
from pfbnav.eval.horizon import horizon_problems, summary
from pfbnav.eval.route_profiles import build_profiles
from pfbnav.utils.hashing import text_sha256
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.scenarios import load_set

REPO_ROOT = Path(__file__).resolve().parents[1]
SPLITS = ("tune", "pilot", "train", "test")
OUT = REPO_ROOT / "tests" / "data" / "route_schedules.json"


def built_sets() -> list[Path]:
    """Every built set file, in a fixed order."""
    return [p for split in SPLITS for p in sorted((REPO_ROOT / "scenarios" / split).glob("*.json"))]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    paths = built_sets()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "analysis", f"ROUTE_SCHEDULES_{stamp}",
                             args.seed, cfg)
    sets: dict[str, str] = {}
    rows: list[dict[str, Any]] = []
    for path in paths:
        rel = path.relative_to(REPO_ROOT).as_posix()
        records = load_set(path)["scenarios"]
        print(f"{rel}: {len(records)} routes", flush=True)
        entries = build_profiles(records, cfg, "schedule", workers=args.workers,
                                 on_done=lambda i, n: print(f"  {i}/{n}", flush=True)
                                 if i % 100 == 0 or i == n else None)
        sets[rel] = text_sha256(path)
        for rec, e in zip(records, entries, strict=True):
            p = e["profile"]
            rows.append({"set": rel, "seed": rec["seed"], "stratum": rec["stratum"],
                         "scenario_sha256": rec["scenario_sha256"],
                         "unmapped": len(rec.get("unmapped_polygons") or []),
                         "plan_length_m": e["plan_length_m"],
                         "traversable": bool(p["traversable"]),
                         "yaw_allowance": p.get("yaw_allowance"),
                         "schedule_time_s": p.get("schedule_time")})
    data = {"sets": sets, "rows": rows}
    (run_dir / "schedules.json").write_text(json.dumps(data), encoding="utf-8")
    OUT.write_text(json.dumps(data), encoding="utf-8")
    r = cfg["reward"]
    built = {k: {"sha256": v, "scenarios": [row["scenario_sha256"] for row in rows
                                             if row["set"] == k]} for k, v in sets.items()}
    problems = horizon_problems(data, built, max_steps=int(r["max_steps"]),
                                dt=float(cfg["sim"]["dt"]), margin=float(r["horizon_margin"]))
    result = {"per_set": summary(rows), "problems": problems,
              "max_steps": int(r["max_steps"]), "margin": float(r["horizon_margin"])}
    (run_dir / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    for name, s in result["per_set"].items():
        print(f"{name}: {s['traversable']}/{s['routes']} traversable, longest schedule "
              f"{s['max_schedule_s']:.0f} s, allowances {s['allowances']}")
    print("HORIZON HOLDS" if not problems else f"HORIZON FAILS: {problems[:5]}")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()} and "
          f"{OUT.relative_to(REPO_ROOT).as_posix()} (commit it by name)")
    return 0 if not problems else 1


if __name__ == "__main__":
    raise SystemExit(main())
