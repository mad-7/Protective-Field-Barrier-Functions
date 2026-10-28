"""Diagnostic: how hard can this platform actually brake?

`filter.a_brake` and `certified.a_brake` are both 1.0 m/s^2, and the PFB's stopping
distance is `v * t_react + v^2 / (2 * a_brake) + margin`. That expression is the safety
argument, and nothing has ever measured the number underneath it. Gate G1 showed the
robot loses 36 per cent of its commanded speed to wheel slip at 0.5 m/s; a contact that
cannot deliver traction to reach a speed may not deliver it to shed one either.

This is a **diagnostic, not a gate**. `preregistration/gates.yaml` contains no braking
criterion, and hard rule 1 forbids adding one here, so the script measures and reports
the ratio to the configured `a_brake` without issuing a verdict. If a criterion is wanted
it must be committed to the preregistration first.

Method: accelerate to the target speed under the normal acceleration limiter, hold until
steady, then command zero **with the rate limiter bypassed** so the wheels are asked to
stop immediately. Anything else would measure the limiter's ramp rather than the plant.
Braking distance is the travelled path from the moment of that command, not an integral
of the speed samples: integrating speed rectangularly reports a deceleration about 9 per
cent low, which `tests/test_gate_metrics.py` pins so it cannot creep back in.

Usage:
    python scripts/diag_braking.py --config configs/base.yaml --seed 0
    python scripts/diag_braking.py --config configs/base.yaml --seed 0 --speeds 0.1 0.2
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.eval.gates import (
    braking_distance,
    effective_deceleration,
    mean_deceleration,
    stop_index,
    unwrap_angles,
)
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.robot import ActuationLimits, YouBotBase
from pfbnav.utils.runio import create_run_dir, stamp_display
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]

# Steps to hold at the target speed before braking, so the measurement starts from a
# settled state rather than from the tail of the acceleration ramp.
HOLD_STEPS = 40
# The robot counts as stopped below this speed, or below 2 per cent of the entry speed,
# whichever is larger. A floor is needed because the simulator leaves a little numerical
# drift that never reaches exactly zero.
STOP_FLOOR_MPS = 0.005
STOP_FRACTION = 0.02
# Give up rather than log forever if the robot will not come to rest.
MAX_BRAKE_STEPS = 400


def run_one(client: SimClient, cfg: dict[str, Any], speed: float, axis: str,
            run_dir: Path) -> dict[str, Any]:
    """Accelerate to `speed`, hold, then brake flat out, logging every step."""
    limits = ActuationLimits.from_config(cfg)
    dt = float(cfg["sim"]["dt"])
    command = np.array([speed, 0.0, 0.0]) if axis == "forward" else np.array([0.0, speed, 0.0])

    client.start(settle_steps=20)
    robot = YouBotBase.from_config(client, cfg)
    robot.bind()
    robot.reset_command_state()

    ramp_steps = math.ceil(speed / (limits.a_max * dt))
    rows: list[dict[str, Any]] = []
    positions: list[tuple[float, float]] = []

    def log(step: int, phase: str, applied) -> None:
        pose = robot.pose()
        v = robot.body_velocity()
        w = robot.body_velocity_from_wheels()
        positions.append((pose[0], pose[1]))
        rows.append({
            "step": step, "t": round(step * dt, 4), "phase": phase,
            "x": pose[0], "y": pose[1], "yaw": pose[2],
            "cmd_vx": applied[0], "cmd_vy": applied[1],
            "pose_vx": v[0], "pose_vy": v[1], "pose_wz": v[2],
            "wheel_vx": w[0], "wheel_vy": w[1],
            "speed": float(math.hypot(v[0], v[1])),
        })

    step = 0
    try:
        for _ in range(ramp_steps + HOLD_STEPS):
            applied = robot.command_body_velocity(command)
            client.step()
            log(step, "accelerate" if step < ramp_steps else "hold", applied)
            step += 1

        onset = len(rows)
        entry_speed = float(np.mean([r["speed"] for r in rows[-10:]]))
        stop_threshold = max(STOP_FLOOR_MPS, STOP_FRACTION * entry_speed)

        for _ in range(MAX_BRAKE_STEPS):
            # Rate limiter bypassed: this must be a step change, or the ramp being
            # measured would be the limiter's rather than the platform's.
            applied = robot.command_body_velocity(np.zeros(3), rate_limit=False)
            client.step()
            log(step, "brake", applied)
            step += 1
            if rows[-1]["speed"] <= stop_threshold:
                break
    finally:
        robot.halt()
        client.stop()

    with (run_dir / f"{axis}_{speed:.2f}.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    return measure(rows, positions, onset, entry_speed, stop_threshold, dt, cfg, axis, speed)


def measure(rows, positions, onset: int, entry_speed: float, stop_threshold: float,
            dt: float, cfg: dict[str, Any], axis: str, speed: float) -> dict[str, Any]:
    """Reduce one braking run to the numbers the PFB's stopping distance depends on."""
    brake_speeds = [r["speed"] for r in rows[onset:]]
    result: dict[str, Any] = {
        "axis": axis,
        "commanded_speed": speed,
        "entry_speed": entry_speed,
        "stop_threshold": stop_threshold,
        "brake_steps_logged": len(brake_speeds),
    }
    try:
        stopped_at = stop_index(brake_speeds, threshold=stop_threshold)
    except ValueError as exc:
        result.update(stopped=False, reason=str(exc))
        return result

    # Travelled path, not an integral of the speed samples: see the module docstring.
    # `braking_distance` starts one sample before the onset, because the first braking
    # step's travel is logged between those two samples.
    distance = braking_distance(positions, onset, stopped_at)
    yaw = unwrap_angles([r["yaw"] for r in rows[onset:onset + stopped_at + 1]])

    a_effective = effective_deceleration(entry_speed, distance) if distance > 0 else None
    configured = float(cfg["certified"]["a_brake"])      # the PFB reads this key (WP2, 2026-10-23)
    result.update(
        stopped=True,
        stop_steps=stopped_at,
        stop_time_s=stopped_at * dt,
        braking_distance_m=distance,
        effective_deceleration=a_effective,
        mean_deceleration=mean_deceleration(brake_speeds[: stopped_at + 1], dt),
        yaw_during_braking_deg=abs(math.degrees(float(yaw[-1] - yaw[0]))),
        configured_a_brake=configured,
        ratio_to_configured=(None if a_effective is None else a_effective / configured),
        distance_at_configured_a_brake_m=entry_speed ** 2 / (2 * configured),
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--speeds", type=float, nargs="*", default=None,
                        help="entry speeds in m/s; default is a sweep up to v_max")
    parser.add_argument("--axis", choices=("forward", "lateral"), default="forward")
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    limits = ActuationLimits.from_config(cfg)
    cap = limits.v_max if args.axis == "forward" else limits.vy_max
    speeds = args.speeds or [round(cap * f, 3) for f in (0.2, 0.4, 0.6, 1.0)]

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "gates", f"BRAKE_{stamp}", args.seed, cfg)

    scene = REPO_ROOT / cfg["sim"]["scene"]
    if not scene.is_file():
        raise SystemExit(f"scene {scene} does not exist")

    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(scene))

    results = []
    try:
        for speed in speeds:
            print(f"-- {args.axis} brake from {speed:.3f} m/s")
            results.append(run_one(client, cfg, speed, args.axis, run_dir))
            print(f"   {results[-1]}")
    finally:
        client.stop()

    report = {
        "diagnostic": "braking",
        "note": ("preregistration/gates.yaml contains no braking criterion, so this "
                 "reports measurements and their ratio to the configured a_brake and "
                 "issues no verdict"),
        "axis": args.axis,
        "configured_a_brake": float(cfg["certified"]["a_brake"]),
        "configured_certified_a_brake": float(cfg["certified"]["a_brake"]),
        "runs": results,
    }
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float))
    stamp_display(run_dir, client.display_status())

    print(f"\nbraking diagnostic -> {run_dir}")
    print(f"{'entry m/s':>10} {'stop m':>8} {'stop s':>7} {'a_eff':>7} {'a_mean':>7} "
          f"{'ratio':>6} {'yaw deg':>8}")
    for r in results:
        if not r.get("stopped"):
            print(f"{r['entry_speed']:>10.3f}   did not stop: {r.get('reason', '')}")
            continue
        print(f"{r['entry_speed']:>10.3f} {r['braking_distance_m']:>8.4f} "
              f"{r['stop_time_s']:>7.2f} {r['effective_deceleration']:>7.3f} "
              f"{r['mean_deceleration']:>7.3f} {r['ratio_to_configured']:>6.2f} "
              f"{r['yaw_during_braking_deg']:>8.2f}")
    print(f"\nconfigured a_brake = {report['configured_filter_a_brake']} m/s^2; a ratio "
          "below 1.0 means the PFB's stopping distance is optimistic")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
