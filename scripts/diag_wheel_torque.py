"""Is the ~12.34 rad/s wheel ceiling a torque limit? Log wheel torque while saturated.

Ruling 2 on the WP0 results report: investigate whether 12.34 rad/s is a joint velocity
limit or a torque limit; do not raise it. The joints are velocity-controlled with a
2.5 N m maximum torque, and their `maxvel` setting (6.28 rad/s) is plainly not in force,
since the wheels run at 10-12.3 rad/s (tests/test_wheel_joint_limits.py). If the ceiling
is the torque cap, the driven wheels sit at |torque| = 2.5 N m on the plateau; if it were
a velocity limit, torque would be below the cap while speed stayed pinned.

Drives G1's diagonal command (v_max, vy_max) -- 16 rad/s on two wheels -- open loop for
a fixed time, and logs every wheel's velocity and torque. A diagnostic, not a gate.

Usage:
    python scripts/diag_wheel_torque.py --config configs/base.yaml --seed 0
"""
from __future__ import annotations

import argparse
import csv
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from pfbnav.config import load_config
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.robot import ActuationLimits, YouBotBase, wheels_from_body
from pfbnav.utils.runio import create_run_dir, stamp_display
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
DURATION_S = 4.0
# Steps skipped before the plateau statistics: the ramp and the approach to saturation.
PLATEAU_FROM_S = 2.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    limits = ActuationLimits.from_config(cfg)
    dt = float(cfg["sim"]["dt"])
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"WHEEL_TORQUE_{stamp}",
                             args.seed, cfg)

    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))
    command = np.array([limits.v_max, limits.vy_max, 0.0])
    robot = None
    rows = []
    try:
        client.start(settle_steps=20)
        physics = client.physics_settings()
        robot = YouBotBase.from_config(client, cfg)
        robot.bind()
        robot.reset_command_state()
        target = wheels_from_body(command, robot.geometry)
        names = ["fl", "fr", "rl", "rr"]
        max_torque = [float(abs(client.sim.getJointTargetForce(h)))
                      for h in robot.wheel_handles]
        for step in range(int(DURATION_S / dt)):
            robot.command_body_velocity(command)
            client.step()
            row = {"step": step, "t": round(step * dt, 4)}
            for name, h in zip(names, robot.wheel_handles):
                row[f"vel_{name}"] = float(client.sim.getJointVelocity(h))
                row[f"torque_{name}"] = float(client.sim.getJointForce(h))
            rows.append(row)
    finally:
        if robot is not None:
            robot.halt()
        client.stop()

    with (run_dir / "wheels.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    plateau = [r for r in rows if r["t"] >= PLATEAU_FROM_S]
    report = {"physics": physics, "command_body": command.tolist(),
              "target_wheel_rad_s": [float(v) for v in target],
              "max_torque_setting_nm": max_torque, "wheels": {}}
    print(f"physics in force: {physics}")
    print(f"{'wheel':>5} {'target':>8} {'plateau vel':>12} {'|torque| mean':>14} "
          f"{'cap':>5}")
    for i, name in enumerate(names):
        vel = np.array([r[f"vel_{name}"] for r in plateau])
        tq = np.abs(np.array([r[f"torque_{name}"] for r in plateau]))
        report["wheels"][name] = {
            "target_rad_s": float(target[i]),
            "plateau_velocity_mean": float(np.mean(np.abs(vel))),
            "plateau_velocity_spread": float(np.ptp(vel)),
            "plateau_abs_torque_mean": float(np.mean(tq)),
            "plateau_abs_torque_max": float(np.max(tq)),
        }
        w = report["wheels"][name]
        print(f"{name:>5} {w['target_rad_s']:8.2f} {w['plateau_velocity_mean']:12.3f} "
              f"{w['plateau_abs_torque_mean']:14.3f} {max_torque[i]:5.2f}")
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2))
    stamp_display(run_dir, client.display_status())
    print(f"\n-> {run_dir}")
    print("Torque at the cap on the saturated wheels means a torque limit; torque below the "
          "cap with speed pinned would mean a velocity limit.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
