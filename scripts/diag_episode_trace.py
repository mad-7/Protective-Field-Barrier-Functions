"""Diagnostic: a per-step trace of a few scenarios under one controller. Researcher runs it.

    python scripts/diag_episode_trace.py --config configs/base.yaml --seed 0 \
        --controller pure_pursuit --set scenarios/tune/tune_narrow.json --indices 0 6 --display on

Written for G4's broken Pure Pursuit control (2026-10-09): 50 of 60 tune episodes ended in an
unrecoverable stop and 7 of 8 collisions came on step 1, while zero action had none. The episode
summaries cannot say where or how a robot stopped; this records, every step, what each layer saw
and did, so the cause can be read rather than guessed.

Reads the environment, never changes it: the step loop's own `StepState` is captured by wrapping
`env.loop.step` in this process only, so the frozen environment's code and identity are untouched
(the identity is checked like any frozen-environment run).

Writes `runs/diag/TRACE_<timestamp>_seed<seed>/trace_<index>.jsonl`, one line per step: the step,
the pose (world x, y, yaw), the estimated velocity, the requested and applied commands, the
certified layer's full report, contact and its names, the lookahead and goal channels, cross-track.

**Step 0** (asked for by the supervisor, 2026-10-10) is its own line, `{"step0": ...}`: the
measured pose against the scenario's start (position and signed yaw error, in the layout frame,
and the error's nearest quarter turn), the route's first segment bearing in the body frame, the
lookahead bearing the policy sees, the first commanded and applied twist, and the shape struck if
the episode ends in contact. `trace_summary.json` gathers one such row per index, with where and
when the robot first stopped, so whether the early stops share a cause can be read in one place.
The indices run in sequence in one process, each reset following an episode that moved -- the
condition under which the stale-pose bug of 2026-10-10 appeared.
"""
from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from pfbnav.config import load_config
from pfbnav.control.pure_pursuit import PurePursuit, ZeroAction
from pfbnav.control.recovery import Recovery, RecoveryGeometry
from pfbnav.envs.freeze import freeze_problem
from pfbnav.envs.nav_env import NavEnv
from pfbnav.envs.observation import BLOCKS, LOOKAHEAD
from pfbnav.eval.trace import (
    bins_inside,
    first_segment_bearing_body_deg,
    nearest_to_footprint,
    quarter_turn_class,
    signed_angle_deg,
)
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.scenarios import load_set

REPO_ROOT = Path(__file__).resolve().parents[1]


# The observation's lookahead channels, for the step-0 record (the diagnostic reads the
# environment's side; the controller itself gets only the typed input).
LOOKAHEAD_AT = sum(w for name, w in BLOCKS[:[n for n, _ in BLOCKS].index(LOOKAHEAD)])


def step0(env: NavEnv, state, obs, index: int) -> dict:
    """The step-0 comparison: where the robot is against where the scenario says it starts."""
    start = env.scenario.start
    x, y, yaw = env._pose_in_layout(state.pose)
    yaw_error = signed_angle_deg(yaw, float(start[2]))
    look = obs[LOOKAHEAD_AT:LOOKAHEAD_AT + 3]
    bearing = first_segment_bearing_body_deg(np.asarray(env.plan.path_world), yaw)
    return {
        "index": index, "scenario_seed": env.scenario.seed,
        "world_pose": state.pose.tolist(), "layout_pose": [x, y, yaw],
        "scenario_start": list(start),
        "start_position_error_m": math.hypot(x - float(start[0]), y - float(start[1])),
        "start_yaw_error_deg": yaw_error, "yaw_error_class": quarter_turn_class(yaw_error),
        "first_segment_bearing_body_deg": bearing,
        "lookahead_bearing_body_deg": math.degrees(math.atan2(look[1], look[2])),
        "in_contact_at_reset": bool(state.in_contact),
        "transform": {"x": env.transform.x, "y": env.transform.y, "yaw": env.transform.yaw},
        "struck": None, "first_stop": None,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--controller", choices=["pure_pursuit", "zero"], required=True)
    parser.add_argument("--set", required=True)
    parser.add_argument("--indices", type=int, nargs="+", required=True)
    add_display_flag(parser)
    args = parser.parse_args()

    cfg = apply_display(load_config(args.config), args.display)
    seed_everything(args.seed)
    problem = freeze_problem(cfg)
    if problem:
        raise SystemExit(problem)
    if "scenarios/test" in Path(args.set).as_posix():
        raise SystemExit("test layouts are never used during development (hard rule 2)")
    scenarios = load_set(REPO_ROOT / args.set)["scenarios"]
    base = PurePursuit.from_config(cfg) if args.controller == "pure_pursuit"         else ZeroAction.from_config(cfg)
    controller = Recovery.from_config(base, cfg)      # the declared recovery (ruled 2026-10-16)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"TRACE_{stamp}", args.seed, cfg)
    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))
    env = NavEnv(client, cfg, scenarios, training=False, seed=args.seed)
    # The layer's own geometry, for the clearance and field records (2026-10-18: G4's B1 control
    # ended 45 of 51 episodes deadlocked inside the minimum margin after a cleared recovery).
    geom = RecoveryGeometry.from_config(cfg)
    range_min = float(cfg["scanners"]["range_min"])

    captured: dict = {}
    original_step, original_reset = env.loop.step, env.loop.reset

    def capturing_step(command):
        state = original_step(command)
        captured["state"] = state
        return state

    def capturing_reset(**kwargs):
        state = original_reset(**kwargs)
        captured["reset"] = state
        return state

    env.loop.step = capturing_step
    env.loop.reset = capturing_reset
    summary: list[dict] = []
    try:
        for index in args.indices:
            controller.reset()
            obs, info = env.reset(options={"index": index})
            path = run_dir / f"trace_{index}.jsonl"
            row = step0(env, captured["reset"], obs, index)
            events: list[dict] = []
            with path.open("w", encoding="utf-8") as f:
                # `controller_input` (the typed ClassicalInput, in `info` since 2026-10-16) is an
                # object, not data: left out here; the per-step rows record what it carries.
                f.write(json.dumps({"reset": {k: v for k, v in info.items()
                                              if k != "controller_input"}},
                                   default=float) + "\n")
                done, step = False, 0
                was_stopped = False
                while not done:
                    inp = info["controller_input"]
                    action = controller.act(inp)
                    obs, reward, terminated, truncated, info = env.step(action)
                    s = captured["state"]
                    step += 1
                    if step == 1:
                        row["first_action"] = np.asarray(action, dtype=float).tolist()
                        row["first_requested"] = s.requested.tolist()
                        row["first_applied"] = s.applied.tolist()
                        f.write(json.dumps({"step0": row}, default=float) + "\n")
                    if s.in_contact and row.get("struck") is None:
                        row["struck"] = {"step": step, "names": list(s.contact_names)}
                    if s.certified.get("stop_triggered") and row.get("first_stop") is None:
                        row["first_stop"] = {"step": step, "pose": s.pose.tolist(),
                                             "layout_pose": list(env._pose_in_layout(s.pose))}
                    # The scan the next decision sees: how near the footprint the nearest point
                    # is (signed, m), every step; and at each stop trigger, release and the last
                    # step, every bin inside the at-rest field and inside the field in force.
                    scan = info["controller_input"].scan
                    near = nearest_to_footprint(geom.footprint_distance, scan.ranges,
                                                scan.bearings, scan.valid, range_min)
                    c = s.certified
                    released = was_stopped and not c["stop_active"]
                    was_stopped = bool(c["stop_active"])
                    done = bool(terminated or truncated)
                    if c.get("stop_triggered") or released or done:
                        src = s.velocity if c["speed"] >= float(np.hypot(*s.applied[:2])) \
                            else s.applied
                        in_force = geom.fields.select(
                            scan.bearings, c["field_speed"], float(math.atan2(src[1], src[0])),
                            at_rest=bool(c["at_rest"]), yaw_rate=c["field_yaw_rate"])
                        at_rest = geom.fields.select(scan.bearings, 0.0, 0.0, at_rest=True)
                        events.append({
                            "step": step, "kind": ("trigger" if c.get("stop_triggered")
                                                   else "release" if released else "final"),
                            "layout_pose": list(env._pose_in_layout(s.pose)),
                            "velocity": s.velocity.tolist(), "applied": s.applied.tolist(),
                            "certified": c, "nearest": near,
                            "recovery": {"mode": controller.mode,
                                         "attempts": controller.attempts},
                            "inside_at_rest_field": bins_inside(
                                scan.ranges, scan.bearings, scan.valid, at_rest),
                            "inside_field_in_force": bins_inside(
                                scan.ranges, scan.bearings, scan.valid, in_force)})
                    f.write(json.dumps({
                        "step": step, "pose": s.pose.tolist(), "velocity": s.velocity.tolist(),
                        "action": np.asarray(action, dtype=float).tolist(),
                        "requested": s.requested.tolist(), "applied": s.applied.tolist(),
                        "certified": s.certified, "in_contact": s.in_contact,
                        "contact_names": list(s.contact_names),
                        "lookahead_bearing_deg": math.degrees(math.atan2(inp.route.lookahead[1],
                                                                         inp.route.lookahead[0])),
                        "goal_distance_m": float(np.hypot(*inp.route.goal)),
                        "recovery": {"mode": controller.mode, "attempts": controller.attempts,
                                     "gave_up": controller.gave_up},
                        "cross_track_m": info.get("cross_track_m"),
                        "nearest": near,
                        "reward": float(reward), "outcome": info.get("outcome"),
                    }, default=float) + "\n")
            (run_dir / f"events_{index}.json").write_text(
                json.dumps(events, indent=1, default=float), encoding="utf-8")
            row["events"] = [{k: e[k] for k in ("step", "kind", "nearest")}
                             | {"n_inside_at_rest": len(e["inside_at_rest_field"]),
                                "n_inside_in_force": len(e["inside_field_in_force"])}
                             for e in events]
            row.update(outcome=info.get("outcome"), steps=step,
                       distance_m=info.get("episode", {}).get("distance_m"),
                       **controller.episode_report())
            summary.append(row)
            print(f"index {index}: {info.get('outcome')} after {step} steps; step 0 off by "
                  f"{row['start_position_error_m'] * 1e3:.1f} mm, {row['start_yaw_error_deg']:+.2f} "
                  f"deg ({row['yaw_error_class']}); route {row['first_segment_bearing_body_deg']} "
                  f"deg; struck {row.get('struck')} -> {path.name}")
    finally:
        env.close()
        client.stop()
        (run_dir / "trace_summary.json").write_text(json.dumps(summary, indent=2, default=float),
                                                     encoding="utf-8")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
