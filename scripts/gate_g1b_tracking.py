"""Gate G1b: closed-loop path tracking.

Criteria come from `preregistration/gates.yaml` and are never hard-coded here. G1 measured
open-loop fidelity and failed; G1b measures what the system actually does, which is to
re-command every physics step. Both results are reported (amendment A1). The unicycle
fallback attaches to this gate, not to G1.

The reference is a **straight line** for every profile, fixed at the start pose from the
profile's commanded direction. `translate_while_rotating` follows that same straight line
while its heading rotates at the commanded rate, which is a holonomic capability rather
than a circular path: a robot that can hold a line while spinning is doing something a
unicycle cannot.

The controller is the proportional heading and cross-track law the criteria name, with
gains from `configs/base.yaml`:

    desired world velocity = speed * along + (-k_cross * cross_track) * normal
    commanded yaw rate     = k_heading * (reference heading - actual heading)

The desired velocity is rotated into the body frame each step, so heading and translation
are corrected independently, as a holonomic base allows.

`v_max` is an output of this gate, not an input (D8): the highest speed at which every
profile passes in every repetition *and every lower speed does too* (A6). `vy_max` comes
from the lateral profile the same way, over the committed grid plus the lateral-only
speeds in `lateral_extra_speeds` (A6). The script never reads `actuation.v_max` to decide
what to run -- the speeds come from the committed criteria -- but the configured box does
clamp the commands, so the re-run uses `configs/gates/g1b_rerun.yaml`, whose box sits at
or above every speed tested. Clamp bindings are counted per run and reported.

Each run also records its realised steady-state speed. G1b has no speed criterion; the
realised-speed map (`realised_speed_map.csv`) is published alongside the verdict because
`actuation.v_max` is a commanded ceiling, not a speed the robot reaches.

Usage:
    python scripts/gate_g1b_tracking.py --config configs/gates/g1b_rerun.yaml --seed 0

Repetitions inside one invocation are bit-identical, so run-to-run spread comes from
separate invocations (`--seed 0/1/2`, `--instance 0/1/2` to run them in parallel),
aggregated by `scripts/aggregate_g1b.py`.
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
import yaml

from pfbnav.config import load_config
from pfbnav.eval.gates import cross_track_error, path_length, rms, steady_speed
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.robot import ActuationLimits, YouBotBase, world_to_body, wrap_angle
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
PREREG = REPO_ROOT / "preregistration" / "gates.yaml"

# Direction of travel in the body frame at the start pose, and the commanded yaw rate as a
# fraction of w_max, for each committed profile name.
PROFILE_DIRECTION: dict[str, tuple[float, float, float]] = {
    "forward": (1.0, 0.0, 0.0),
    "lateral": (0.0, 1.0, 0.0),
    "diagonal_45": (math.sqrt(0.5), math.sqrt(0.5), 0.0),
    "translate_while_rotating": (1.0, 0.0, 0.2),
}

# Settling allowance before cross-track error is scored, in metres of travel. The robot
# starts exactly on the line, so this only excludes the acceleration ramp.
SCORE_FROM_M = 0.25

# Travel after which realised speed is measured, in metres; excludes the ramp at every
# speed tested (0.50 m/s at a_max 1.0 m/s^2 reaches speed within 0.125 m).
REALISED_SPEED_FROM_M = 1.0


def run_once(client: SimClient, cfg: dict[str, Any], profile: str, speed: float,
             distance_m: float, run_dir: Path, rep: int) -> dict[str, Any]:
    """One profile at one speed, tracked closed-loop for `distance_m`."""
    limits = ActuationLimits.from_config(cfg)
    gains = cfg["tracking"]
    dt = float(cfg["sim"]["dt"])
    bx, by, wz_fraction = PROFILE_DIRECTION[profile]
    wz_ref = wz_fraction * limits.w_max

    client.start(settle_steps=20)
    robot = YouBotBase.from_config(client, cfg)
    robot.bind()
    robot.reset_command_state()

    x0, y0, yaw0 = robot.pose()
    # The reference line, fixed in the world at the start pose.
    along = np.array([math.cos(yaw0) * bx - math.sin(yaw0) * by,
                      math.sin(yaw0) * bx + math.cos(yaw0) * by])
    along /= float(np.linalg.norm(along))
    origin = np.array([x0, y0])

    ramp_steps = math.ceil(speed / (limits.a_max * dt))
    max_steps = int(distance_m / speed / dt) + ramp_steps + 300

    rows: list[dict[str, Any]] = []
    positions: list[tuple[float, float]] = []
    try:
        for step in range(max_steps):
            x, y, yaw = robot.pose()
            error = cross_track_error([x, y], origin, along)
            yaw_ref = wrap_angle(yaw0 + wz_ref * step * dt)
            heading_error = wrap_angle(yaw_ref - yaw)

            normal = np.array([-along[1], along[0]])
            desired_world = speed * along - float(gains["k_cross_track"]) * error * normal
            vx, vy = world_to_body(desired_world, yaw)
            command = np.array([vx, vy, wz_ref + float(gains["k_heading"]) * heading_error])

            applied = robot.command_body_velocity(command)
            clamp = robot.last_clamp
            if clamp is None:
                raise RuntimeError("command_body_velocity did not record its clamp status")
            client.step()
            positions.append((x, y))
            rows.append({
                "step": step, "t": round(step * dt, 4), "x": x, "y": y, "yaw": yaw,
                "cross_track": error, "heading_error": heading_error, "yaw_ref": yaw_ref,
                "cmd_vx": applied[0], "cmd_vy": applied[1], "cmd_wz": applied[2],
                "clamp_scale": clamp.translation_scale,
                "omega_clipped": int(clamp.omega_clipped),
                "budget_scale": (robot.last_budget.scale if robot.last_budget is not None
                                 else 1.0),
                "travelled": path_length(positions) if len(positions) > 1 else 0.0,
            })
            if rows[-1]["travelled"] >= distance_m:
                break
    finally:
        robot.halt()
        client.stop()

    name = f"{profile}_{speed:.2f}_rep{rep}"
    with (run_dir / f"{name}.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    scored = [r for r in rows if r["travelled"] >= SCORE_FROM_M] or rows
    errors = [r["cross_track"] for r in scored]
    return {
        "profile": profile, "speed": speed, "repetition": rep,
        "steps": len(rows),
        "distance_travelled_m": rows[-1]["travelled"],
        "max_abs_cross_track": float(max(abs(e) for e in errors)),
        "rms_cross_track": rms(errors),
        "final_heading_error_deg": abs(math.degrees(rows[-1]["heading_error"])),
        "scored_from_step": rows.index(scored[0]),
        "realised_speed": steady_speed([r["travelled"] for r in rows],
                                       [r["t"] for r in rows],
                                       from_m=REALISED_SPEED_FROM_M),
        "clamp_translation_steps": robot.clamp_counts["translation"],
        "clamp_omega_steps": robot.clamp_counts["omega"],
        "budget_steps": robot.clamp_counts["budget"],
    }


def _lowest_first_ceiling(speeds: list[float], passed: dict[float, bool]) -> float | None:
    """Highest speed that passes with every lower speed passing too; None if the lowest fails.

    A6: a pass above a failure is not a usable ceiling -- a robot that holds its line at
    0.40 m/s but not at 0.30 cannot be run anywhere up to 0.40.
    """
    ceiling = None
    for speed in sorted(speeds):
        if not passed[speed]:
            break
        ceiling = speed
    return ceiling


def verdict(results: list[dict[str, Any]], criteria: dict[str, Any]) -> dict[str, Any]:
    """Apply the committed criteria and derive v_max and vy_max.

    Thresholds are read, never restated. `v_max` comes from the committed grid, where
    every profile runs; `vy_max` from the lateral profile over the grid and the
    lateral-only extension speeds.
    """
    max_ct = float(criteria["max_cross_track"])
    max_rms = float(criteria["rms_cross_track"])
    max_heading = float(criteria["max_final_heading_error_deg"])
    for r in results:
        reasons = []
        if r["max_abs_cross_track"] > max_ct:
            reasons.append(f"peak cross-track {r['max_abs_cross_track']:.4f} > {max_ct}")
        if r["rms_cross_track"] > max_rms:
            reasons.append(f"RMS cross-track {r['rms_cross_track']:.4f} > {max_rms}")
        if r["final_heading_error_deg"] > max_heading:
            reasons.append(
                f"final heading error {r['final_heading_error_deg']:.2f} > {max_heading}")
        r["passed"] = not reasons
        r["reason"] = "; ".join(reasons)

    grid = sorted(float(v) for v in criteria["speeds"])
    profiles = set(criteria["profiles"])
    per_speed = {}
    for speed in grid:
        at_speed = [r for r in results if r["speed"] == speed]
        covered = {r["profile"] for r in at_speed} >= profiles
        per_speed[speed] = {
            "all_pass": covered and all(r["passed"] for r in at_speed),
            "repetitions": len(at_speed) // max(len(profiles), 1),
            "failing": sorted({r["profile"] for r in at_speed if not r["passed"]}),
            "missing": sorted(profiles - {r["profile"] for r in at_speed}),
        }
    v_max = _lowest_first_ceiling(grid, {v: per_speed[v]["all_pass"] for v in grid})

    lateral = [r for r in results if r["profile"] == "lateral"]
    lateral_speeds = sorted({r["speed"] for r in lateral})
    lateral_pass = {v: all(r["passed"] for r in lateral if r["speed"] == v)
                    for v in lateral_speeds}
    vy_max = _lowest_first_ceiling(lateral_speeds, lateral_pass) if lateral else None

    return {
        "runs": results,
        "per_speed": per_speed,
        "lateral_per_speed": lateral_pass,
        "v_max_from_gate": v_max,
        "vy_max_from_gate": vy_max,
        "result": "PASS" if v_max is not None else "FAIL",
        "fallback": "" if v_max is not None else
                    "unicycle primary; holonomic becomes caveated ablation (amendment A1)",
    }


def realised_speed_map(results: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Commanded against realised speed, per profile and speed, over the repetitions."""
    table = []
    for profile in sorted({r["profile"] for r in results}):
        for speed in sorted({r["speed"] for r in results if r["profile"] == profile}):
            realised = [r["realised_speed"] for r in results
                        if r["profile"] == profile and r["speed"] == speed]
            mean = float(np.mean(realised))
            table.append({
                "profile": profile, "commanded_m_s": speed,
                "realised_mean_m_s": round(mean, 4),
                "realised_min_m_s": round(float(min(realised)), 4),
                "realised_max_m_s": round(float(max(realised)), 4),
                "shortfall": round(1.0 - mean / speed, 4),
                "repetitions": len(realised),
            })
    return table


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    add_display_flag(parser)
    parser.add_argument("--profiles", nargs="*", default=None)
    parser.add_argument("--speeds", type=float, nargs="*", default=None)
    parser.add_argument("--repetitions", type=int, default=None)
    parser.add_argument("--instance", type=int, default=0,
                        help="instance to run on (port sim.port + sim.port_stride * i), so "
                             "separate invocations can run in parallel")
    args = parser.parse_args()

    cfg = load_config(args.config)
    cfg = apply_display(cfg, args.display)
    seed_everything(args.seed)
    criteria = yaml.safe_load(PREREG.read_text())["G1b_closed_loop_tracking"]
    profiles = args.profiles or list(criteria["profiles"])
    speeds = args.speeds or [float(s) for s in criteria["speeds"]]
    repetitions = args.repetitions or int(criteria["repetitions"])
    distance_m = float(criteria["distance"])
    if "lateral_extra_speeds" not in criteria:
        raise SystemExit(
            "preregistration/gates.yaml has no G1b_closed_loop_tracking.lateral_extra_speeds; "
            "the vy_max extension (A6) must be committed there before this gate runs")
    # --speeds narrows a run to a smoke test, and then the extension is not run either.
    extra_lateral = [] if args.speeds or "lateral" not in profiles else \
        [float(v) for v in criteria["lateral_extra_speeds"]]

    # The box must not decide the result: every speed tested has to sit inside it, or the
    # clamp shapes the command and the gate measures the clamp (A6).
    limits = ActuationLimits.from_config(cfg)
    top_lateral = max(speeds + extra_lateral) if "lateral" in profiles else 0.0
    if limits.v_max < max(speeds) or limits.vy_max < top_lateral:
        raise SystemExit(
            f"the configured box (v_max {limits.v_max}, vy_max {limits.vy_max} m/s) is below "
            f"the speeds tested (up to {max(speeds)}, lateral up to {top_lateral} m/s); "
            "use configs/gates/g1b_rerun.yaml (A6)")

    unknown = [p for p in profiles if p not in PROFILE_DIRECTION]
    if unknown:
        raise SystemExit(f"unknown profiles {unknown}; known: {sorted(PROFILE_DIRECTION)}")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "gates", f"G1b_{stamp}", args.seed, cfg)

    scene = REPO_ROOT / cfg["sim"]["scene"]
    if not scene.is_file():
        raise SystemExit(f"scene {scene} does not exist")

    client = SimClient(SimConnection.from_config(cfg, instance_id=args.instance))
    client.connect()
    client.load_scene(str(scene))

    client.start(settle_steps=1)
    physics = client.physics_settings()
    client.stop()
    print(f"physics in force: {physics}")

    plan = [(speed, profile) for speed in speeds for profile in profiles]
    plan += [(speed, "lateral") for speed in extra_lateral]
    results = []
    total = len(plan) * repetitions
    try:
        for rep in range(repetitions):
            for speed, profile in plan:
                print(f"-- [{len(results) + 1}/{total}] {profile} at {speed:.2f} m/s "
                      f"(rep {rep}, {physics['engine_name']})")
                refusals_before = client.display_refusals
                results.append(
                    run_once(client, cfg, profile, speed, distance_m, run_dir, rep))
                r = results[-1]
                # Which episodes rendered, not merely that some did. The run-level
                # `consistent` flag said an instance changed its mind part-way (seen
                # 2026-09-20 and again 2026-09-26) but not where, so review could not tell
                # whether a failing cell was one of them.
                r["display_in_force"] = client.display_in_force
                r["display_refused"] = client.display_refusals > refusals_before
                print(f"   peak {r['max_abs_cross_track']:.4f} m, "
                      f"RMS {r['rms_cross_track']:.4f} m, "
                      f"heading {r['final_heading_error_deg']:.2f} deg, "
                      f"realised {r['realised_speed']:.3f} m/s, "
                      f"clamp bound {r['clamp_translation_steps']} translation / "
                      f"{r['clamp_omega_steps']} omega steps")
    finally:
        client.stop()

    report = verdict(results, criteria)
    if args.profiles or args.speeds or args.repetitions:
        # A narrowed run (a smoke test) cannot pass or fail the gate: the committed grid
        # was not run, and printing FAIL with the unicycle fallback would invite misquoting.
        report["result"] = "PARTIAL"
        report["fallback"] = ""
    speed_map = realised_speed_map(results)
    report.update(criteria=criteria, physics=physics, gains=cfg["tracking"],
                  # The wheel budget shapes the controller's commands, so the verdict is
                  # a verdict on the controller with this budget (WP0 results, ruling 2).
                  wheel_budget=cfg["actuation"]["wheel_budget"], instance=args.instance,
                  # Rendering changes the step's cost, not its result, but a run that
                  # rendered when it was asked not to says so (2026-09-20).
                  # `display_status` also carries `starts`, which exists so a refusal
                  # count can be read against the number of chances to refuse; building
                  # this dict by hand had dropped it.
                  display=client.display_status(),
                  repetitions=repetitions, lateral_extra_speeds=extra_lateral,
                  box={"v_max": limits.v_max, "vy_max": limits.vy_max,
                       "w_max": limits.w_max},
                  realised_speed_map=speed_map)
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float))
    with (run_dir / "realised_speed_map.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(speed_map[0].keys()))
        writer.writeheader()
        writer.writerows(speed_map)

    print(f"\nG1b {report['result']} on {physics['engine_name']}  ->  {run_dir}")
    print(f"{'speed':>7} {'all pass':>9}  failing profiles")
    for speed, info in report["per_speed"].items():
        print(f"{speed:>7.2f} {info['all_pass']!s:>9}  {', '.join(info['failing']) or '-'}")
    lateral = ", ".join(f"{v:.2f} {'pass' if ok else 'FAIL'}"
                        for v, ok in report["lateral_per_speed"].items())
    print(f"\nlateral: {lateral}")
    print(f"\nv_max from this gate:  {report['v_max_from_gate']} m/s")
    print(f"vy_max from this gate: {report['vy_max_from_gate']} m/s")
    print("Neither is applied here; D8 sets both, and every dependent, in one commit.")
    if report["result"] == "PARTIAL":
        print("PARTIAL run: the committed grid was not run, so there is no gate verdict.")
    elif report["v_max_from_gate"] is None:
        print(f"no speed passed every profile in every repetition; {report['fallback']}")
    return 0 if report["result"] in ("PASS", "PARTIAL") else 1


if __name__ == "__main__":
    raise SystemExit(main())
