"""Check classical recovery's speed against the velocity estimate's overshoot (ruled 2026-10-16).

Replays the environment's velocity estimator over the ground-truth poses recorded by the G1b runs
on the S300 scene (carried over to 537f1bbb) and reports, per nominal speed, the largest ratio of
estimated speed to the largest command issued so far. Recovery's speed is safe while
`speed x ratio < band 0's top speed`. The positive control (hard rule 15): the replayed steady-state
speed must reproduce each run's recorded realised speed (`realised_speed_map.csv`). Seconds; no
simulator.

    python scripts/check_recovery_speed.py --config configs/base.yaml --seed 0

Writes `runs/diag/RECOVERY_SPEED_<timestamp>_seed<seed>/summary.json`.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.eval.overshoot import estimated_speeds, overshoot_ratio
from pfbnav.sim.velocity_estimate import VelocityEstimateConfig
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.strata import certified_params_from_config

REPO_ROOT = Path(__file__).resolve().parents[1]
G1B_RUNS = ("G1b_20260926T180549Z_seed0", "G1b_20260926T180547Z_seed1",
            "G1b_20260926T180544Z_seed2")
CONTROL_TOLERANCE = 0.002      # m/s, replayed vs recorded realised speed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    vc = VelocityEstimateConfig.from_config(cfg)
    speed = float(cfg["baselines"]["recovery"]["speed"])
    band0 = float(certified_params_from_config(cfg).band_top_speeds[0])

    per_speed: dict[float, list[dict]] = {}
    control_worst = 0.0
    for run in G1B_RUNS:
        d = REPO_ROOT / "runs" / "gates" / run
        with open(d / "realised_speed_map.csv", encoding="utf-8") as fh:
            realised = {(r["profile"], float(r["commanded_m_s"])): float(r["realised_mean_m_s"])
                        for r in csv.DictReader(fh)}
        for path in sorted(d.glob("*_rep*.csv")):
            profile, nominal, rep = path.stem.rsplit("_", 2)
            with open(path, encoding="utf-8") as fh:
                rows = list(csv.DictReader(fh))
            poses = [(float(r["x"]), float(r["y"]), float(r["yaw"])) for r in rows]
            cmds = [math.hypot(float(r["cmd_vx"]), float(r["cmd_vy"])) for r in rows]
            out = overshoot_ratio(poses, cmds, vc)
            per_speed.setdefault(float(nominal), []).append({"run": run, "file": path.name, **out})
            if rep == "rep0":
                e = estimated_speeds(poses, vc)
                control_worst = max(control_worst, abs(float(e[len(e) // 2:].mean())
                                                       - realised[(profile, float(nominal))]))
    worst = max((r for rs in per_speed.values() for r in rs), key=lambda r: r["ratio"])
    summary = {
        "recovery_speed_m_s": speed, "band0_top_m_s": band0, "runs": sum(map(len, per_speed.values())),
        "max_ratio": worst["ratio"], "worst": worst,
        "max_estimate_at_recovery_speed_m_s": speed * worst["ratio"],
        "margin_to_band0_m_s": band0 - speed * worst["ratio"],
        "verdict": "ok" if speed * worst["ratio"] < band0 else "LOWER THE RECOVERY SPEED",
        "positive_control_max_abs_difference_m_s": control_worst,
        "positive_control": "ok" if control_worst <= CONTROL_TOLERANCE else "FAILED",
        "by_nominal_speed": {f"{k:.2f}": max(r["ratio"] for r in v) for k, v in sorted(per_speed.items())},
    }
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"RECOVERY_SPEED_{stamp}", args.seed, cfg)
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in summary.items() if k != "worst"}, indent=2))
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0 if summary["verdict"] == "ok" and summary["positive_control"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
