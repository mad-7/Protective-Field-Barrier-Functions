"""Gate G1: holonomic motion fidelity in CoppeliaSim.

Criteria come from `preregistration/gates.yaml` and are never hard-coded here. Each
profile commands a constant body velocity until the robot has travelled the committed
distance, logging commanded against measured body velocity every step, and reports the
mean relative speed error over steady state and the heading drift.

Two velocity readouts are logged, and the difference between them is the point:

  * `body_velocity()` comes from the pose of the reference frame,
  * `body_velocity_from_wheels()` is reconstructed from the wheel joints alone.

They share no code path, and `diagnose_direction` reads them together. Note what they can
and cannot separate: because `body_from_wheels` divides by the *configured* sign vector, an
inverted `wheel_sign` and a `yaw_offset` that is pi out produce the *same* disagreement.
The wheel layout test is what separates them, by measuring which quadrant each named wheel
sits in. The script says so rather than picking one.

Per the decision log, holonomic actuation is primary only if G1 passes; the committed
fallback is unicycle primary with holonomic as a caveated ablation.

Usage:
    python scripts/gate_g1_fidelity.py --config configs/base.yaml --seed 0
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pfbnav.config import load_config
from pfbnav.eval.gates import (
    diagnose_direction,
    heading_drift_deg,
    path_length,
    relative_speed_error,
    signed_heading_drift_deg,
    steady_state_start,
)
from pfbnav.sim.client import ENGINE_NAMES, SimClient, SimConnection
from pfbnav.sim.robot import ActuationLimits, YouBotBase
from pfbnav.utils.runio import create_run_dir, stamp_display
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
PREREG = REPO_ROOT / "preregistration" / "gates.yaml"

# How close the applied command must be to the target before steady state begins, and how
# many further steps to let the physics settle. The limiter reaches the target in a known
# number of steps; the base takes a little longer to catch up with it.
STEADY_TOL = 0.02
SETTLE_STEPS = 10
# Stop a profile that is going nowhere rather than looping to the step cap.
STALL_SPEED_FRACTION = 0.05


@dataclass(frozen=True)
class Profile:
    """One commanded body velocity, as a fraction of the configured limits."""

    name: str
    vx: float
    vy: float
    wz: float

    def command(self, limits: ActuationLimits) -> np.ndarray:
        return np.array([self.vx * limits.v_max, self.vy * limits.vy_max,
                         self.wz * limits.w_max])


PROFILES = {
    "forward": Profile("forward", 1.0, 0.0, 0.0),
    "lateral": Profile("lateral", 0.0, 1.0, 0.0),
    # Equal fractions of each axis limit, so neither axis saturates alone.
    "diagonal_45": Profile("diagonal_45", 1.0, 1.0, 0.0),
    "translate_while_rotating": Profile("translate_while_rotating", 1.0, 0.0, 0.2),
}


def run_profile(client: SimClient, cfg: dict[str, Any], profile: Profile,
                distance_m: float, run_dir: Path, speed_scale: float = 1.0,
                log_joints: tuple[str, ...] = ()) -> dict[str, Any]:
    """Drive one profile for `distance_m` of path length and return its measurements.

    `log_joints` names extra scene joints whose position is logged every step. The youBot's
    front wheels hang off a live revolute rocker while the rear pair is rigid, so watching
    that joint separates "the chassis is deforming under load" from "the wheel model is
    wrong" -- two explanations for the lateral yaw that the velocity logs cannot tell apart.
    """
    limits = ActuationLimits.from_config(cfg)
    dt = float(cfg["sim"]["dt"])
    command = profile.command(limits) * speed_scale
    target_speed = float(np.linalg.norm(command[:2]))

    client.start(settle_steps=20)
    robot = YouBotBase.from_config(client, cfg)
    robot.bind()
    robot.reset_command_state()
    joint_handles = {path.rsplit("/", 1)[-1]: client.object(path) for path in log_joints}

    ramp_steps = math.ceil(target_speed / (limits.a_max * dt)) if target_speed > 0 else 0
    max_steps = int(distance_m / max(target_speed, 1e-6) / dt) + ramp_steps + SETTLE_STEPS + 200

    rows: list[dict[str, Any]] = []
    positions: list[tuple[float, float]] = []
    stalled = False
    try:
        for step in range(max_steps):
            applied = robot.command_body_velocity(command)
            client.step()
            pose = robot.pose()
            from_pose = robot.body_velocity()
            from_wheels = robot.body_velocity_from_wheels()
            positions.append((pose[0], pose[1]))
            row = {
                "step": step, "t": round(step * dt, 4),
                "x": pose[0], "y": pose[1], "yaw": pose[2],
                "cmd_vx": applied[0], "cmd_vy": applied[1], "cmd_wz": applied[2],
                "pose_vx": from_pose[0], "pose_vy": from_pose[1], "pose_wz": from_pose[2],
                "wheel_vx": from_wheels[0], "wheel_vy": from_wheels[1],
                "wheel_wz": from_wheels[2],
            }
            for name, handle in joint_handles.items():
                row[f"joint_{name}"] = float(client.sim.getJointPosition(handle))
            rows.append(row)
            if path_length(positions) >= distance_m:
                break
            if step > ramp_steps + SETTLE_STEPS + 40 and path_length(positions) < 0.02:
                stalled = True
                break
    finally:
        robot.halt()
        client.stop()

    with (run_dir / f"{profile.name}.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    return measure_profile(profile, rows, positions, command, target_speed, dt, stalled)


def measure_profile(profile: Profile, rows: list[dict[str, Any]],
                    positions: list[tuple[float, float]], command: np.ndarray,
                    target_speed: float, dt: float, stalled: bool) -> dict[str, Any]:
    """Reduce one profile's log to the quantities the criteria are stated in."""
    travelled = path_length(positions)
    applied_speed = [math.hypot(r["cmd_vx"], r["cmd_vy"]) for r in rows]
    pose_v = np.array([[r["pose_vx"], r["pose_vy"]] for r in rows])
    wheel_v = np.array([[r["wheel_vx"], r["wheel_vy"]] for r in rows])
    yaw = [r["yaw"] for r in rows]

    result: dict[str, Any] = {
        "profile": profile.name,
        "commanded": {"vx": command[0], "vy": command[1], "wz": command[2]},
        "target_speed": target_speed,
        "steps": len(rows),
        "distance_travelled_m": travelled,
        "stalled": stalled,
    }

    # The direction check is what tells a sign-vector error from a physics one, and it
    # does not need steady state, so it is computed before anything that can raise.
    mean_pose = pose_v.mean(axis=0)
    mean_wheel = wheel_v.mean(axis=0)
    intended = command[:2] / max(float(np.linalg.norm(command[:2])), 1e-9)
    result["direction"] = {
        "mean_pose_velocity": mean_pose.tolist(),
        "mean_wheel_velocity": mean_wheel.tolist(),
        "pose_along_command": float(np.dot(mean_pose, intended)),
        "wheel_along_command": float(np.dot(mean_wheel, intended)),
    }
    result["diagnosis"] = diagnose_direction(
        result["direction"]["pose_along_command"],
        result["direction"]["wheel_along_command"],
        target_speed,
        stall_fraction=STALL_SPEED_FRACTION,
    )

    try:
        start = steady_state_start(applied_speed, target_speed, tol=STEADY_TOL,
                                   settle_steps=SETTLE_STEPS)
    except ValueError as exc:
        result.update(measurable=False, reason=str(exc))
        return result

    commanded_series = np.tile(command[:2], (len(rows) - start, 1))
    result.update(
        measurable=True,
        steady_state_from_step=start,
        mean_relative_speed_error=relative_speed_error(pose_v[start:], commanded_series),
        mean_relative_speed_error_from_wheels=relative_speed_error(
            wheel_v[start:], commanded_series),
        # Wheels turning faster than the body moves is slip at the contact, which is a
        # different fault from the controller failing to reach its command.
        mean_slip_fraction=float(1.0 - (
            np.linalg.norm(pose_v[start:], axis=1).mean()
            / max(float(np.linalg.norm(wheel_v[start:], axis=1).mean()), 1e-9))),
        heading_drift_deg=heading_drift_deg(yaw, command[2], dt),
        # Signed, for comparing one run's drift with another's (A10 review).
        signed_heading_drift_deg=signed_heading_drift_deg(yaw, command[2], dt),
    )
    return result


def verdict(measurements: list[dict[str, Any]], criteria: dict[str, Any]) -> dict[str, Any]:
    """Apply the committed criteria. Thresholds are read, never restated."""
    max_error = float(criteria["max_speed_error"])
    max_drift = float(criteria["max_heading_drift_deg"])
    for item in measurements:
        if not item.get("measurable", False):
            item["passed"] = False
            continue
        speed_ok = item["mean_relative_speed_error"] <= max_error
        drift_ok = item["heading_drift_deg"] <= max_drift
        item["passed"] = speed_ok and drift_ok
        item["reason"] = "; ".join(
            part for part, ok in (
                (f"speed error {item['mean_relative_speed_error']:.4f} > {max_error}",
                 speed_ok),
                (f"heading drift {item['heading_drift_deg']:.2f} deg > {max_drift}",
                 drift_ok),
            ) if not ok
        )
    passed = all(item["passed"] for item in measurements)
    return {
        "profiles": measurements,
        "result": "PASS" if passed else "FAIL",
        "fallback": "" if passed else criteria.get("fallback", ""),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--profiles", nargs="*", default=None,
                        help="subset of the committed profiles; default is all of them")
    parser.add_argument("--engine", type=int, default=None,
                        help="dynamics engine to set before running (D7 probe): "
                             "0 Bullet, 1 ODE, 2 Vortex, 3 Newton, 4 MuJoCo")
    parser.add_argument("--dynamic-step", type=float, default=None,
                        help="dynamics substep size in seconds, e.g. 0.002 (D7 probe)")
    parser.add_argument("--log-joints", nargs="*", default=[],
                        help="scene paths of extra joints to log each step, e.g. "
                             "/youBot/Joint (the front-axle rocker)")
    parser.add_argument("--speed-scale", type=float, default=1.0,
                        help="scale every commanded velocity, for diagnosing how slip "
                             "varies with speed. Any value other than 1.0 makes the run "
                             "a diagnostic: the result is reported as DIAGNOSTIC, never "
                             "as a gate verdict, because the criteria are stated at the "
                             "configured limits.")
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    criteria = yaml.safe_load(PREREG.read_text())["G1_holonomic_fidelity"]
    names = args.profiles or list(criteria["profiles"])
    unknown = [n for n in names if n not in PROFILES]
    if unknown:
        raise SystemExit(f"unknown profiles {unknown}; known: {sorted(PROFILES)}")
    distance_m = float(criteria["distance"])

    scene = REPO_ROOT / cfg["sim"]["scene"]
    if not scene.is_file():
        raise SystemExit(f"scene {scene} does not exist")

    connection = SimConnection.from_config(cfg)
    frozen = (connection.engine, connection.dynamic_step)
    if args.engine is not None:
        connection = dataclasses.replace(connection, engine=args.engine)
    if args.dynamic_step is not None:
        connection = dataclasses.replace(connection, dynamic_step=args.dynamic_step)
    physics_overridden = (connection.engine, connection.dynamic_step) != frozen

    # The engine is in the run name, so a G1 result is never quoted without it: the
    # recorded Bullet FAIL and the ODE re-run (G1-ODE) sit side by side (rulings 5, 6).
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    engine_name = ENGINE_NAMES.get(connection.engine, f"engine{connection.engine}")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "gates", f"G1-{engine_name}_{stamp}",
                             args.seed, cfg)

    client = SimClient(connection)
    client.connect()
    client.load_scene(str(scene))

    # Captured once, before any profile runs, so the summary records the physics that was
    # actually in force rather than what the configuration hoped for (D7).
    client.start(settle_steps=1)
    physics = client.physics_settings()
    client.stop()

    measurements = []
    try:
        for name in names:
            print(f"-- profile {name}: {distance_m} m")
            measurements.append(run_profile(client, cfg, PROFILES[name], distance_m,
                                            run_dir, args.speed_scale,
                                            tuple(args.log_joints)))
            print(f"   {measurements[-1]['diagnosis']}")
    finally:
        client.stop()

    report = verdict(measurements, criteria)
    report["criteria"] = criteria
    report["profiles_run"] = names
    report["physics"] = physics
    report["speed_scale"] = args.speed_scale
    report["physics_overridden"] = physics_overridden
    band = float(cfg["sim"]["physics"]["solver_sensitivity_drift_deg"])
    report["solver_sensitivity_drift_deg"] = band
    if args.speed_scale != 1.0 or physics_overridden:
        # The criteria are stated at the configured limits and the frozen physics, so a
        # scaled or re-engined run is a measurement; it does not pass or fail the gate.
        report["result"] = "DIAGNOSTIC"
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float))
    stamp_display(run_dir, client.display_status())

    print(f"\nG1 {report['result']} on {physics['engine_name']}  ->  {run_dir}")
    print(f"   drift figures carry a +/-{band} deg solver-sensitivity band (numerical noise)")
    for item in report["profiles"]:
        mark = "ok  " if item["passed"] else "FAIL"
        error = item.get("mean_relative_speed_error")
        drift = item.get("heading_drift_deg")
        print(f"   {mark} {item['profile']:<26} "
              f"speed_err={error if error is None else round(error, 4)} "
              f"drift_deg={drift if drift is None else round(drift, 2)} "
              f"travelled={item['distance_travelled_m']:.2f} m")
        print(f"        {item['diagnosis']}")
    if report["fallback"]:
        print(f"   fallback on failure: {report['fallback']}")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
