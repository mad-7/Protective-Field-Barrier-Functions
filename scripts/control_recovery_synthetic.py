"""The synthetic recovery control on current geometry, on the simulator (ruled 2026-10-16).

    python scripts/control_recovery_synthetic.py --config configs/base.yaml --seed 0

Researcher runs it (~3 min). The scene and criteria are declared in
`configs/controls/recovery_synthetic.yaml` before any run: one shelf, corner ahead-left, the goal
round it, the route planned at today's margins (lethal 0.10). Constructed at run time like G0's
cases, in the frame of the robot's start pose, with the environment's own layout machinery
(`transform_placing`, `LayoutShapes`). Both arms run in ONE scene load (determinism is per load):

  * **bare B1** -- the control arm: it must stop at the bend and stay blocked;
  * **B1 with the declared recovery** -- it must reach the intended stop state (at rest, at-rest
    field clear, restart blocked), recovery must release it, and B1 must then make 0.5 m of route
    progress (ruled).

The known-answer check (`pfbnav/eval/recovery_control.py`, `tests/test_recovery_control.py`) is
re-run here and written beside the result as `prediction.json`, so the simulator's verdict can be
compared with what the emulation said before the run.

Writes `runs/diag/CONTROL_RECOVERY_SYNTHETIC_<timestamp>_seed<seed>/`: `trace_bare.jsonl`,
`trace_recovery.jsonl` (one line per step), `prediction.json`, `summary.json` (each criterion,
PASS/FAIL, the analyses, the recovery's per-attempt statistics, any contact).
"""
from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from pfbnav.config import load_config
from pfbnav.control.pure_pursuit import PurePursuit
from pfbnav.control.recovery import Recovery
from pfbnav.envs.controller_input import ScanStream
from pfbnav.envs.freeze import freeze_problem
from pfbnav.envs.step_loop import StepLoop
from pfbnav.eval.recovery_control import (
    ControlSpec,
    _layer,
    analyse,
    classical_input,
    judge,
    plan_route,
    predict,
    route_clearance,
    shelf_polygon,
)
from pfbnav.planning.lookahead import PathTracker
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.layout_shapes import LayoutShapes, transform_placing, wrap
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.baking import Obstacle, polygon_distance

REPO_ROOT = Path(__file__).resolve().parents[1]
SPEC_PATH = REPO_ROOT / "configs" / "controls" / "recovery_synthetic.yaml"
SETTLE = 20


def _robot_world_pose(client, cfg) -> tuple[float, float, float]:
    """The robot's pose in the BODY convention, exactly as `NavEnv._robot_world_pose` reads it."""
    sim = client.sim
    ref = client.object(cfg["sim"]["objects"]["base_ref"])
    position = [float(v) for v in sim.getObjectPosition(ref, sim.handle_world)]
    yaw = float(sim.getObjectOrientation(ref, sim.handle_world)[2])
    return position[0], position[1], wrap(yaw + float(cfg["robot"]["yaw_offset"]))


def run_arm(client, loop, shapes, cfg, spec, route, controller, label, run_dir) -> dict:
    shelf = shelf_polygon(spec)
    obstacle = Obstacle(shelf, mapped=True, name="shelf_control")
    placed: dict = {}

    def build() -> None:
        placed["transform"] = transform_placing((0.0, 0.0, 0.0), _robot_world_pose(client, cfg))
        shapes.instantiate([obstacle], placed["transform"])

    shapes.remove()
    state = loop.reset(settle_steps=SETTLE, after_start=build)
    problems = shapes.verify([obstacle])
    if problems:
        raise RuntimeError(f"the constructed shelf does not match its declaration: {problems}")
    inverse = placed["transform"].invert()
    tracker = PathTracker(route, float(cfg["planner"]["lookahead"]),
                          window=float(cfg["planner"]["track_window"]))
    ceilings = np.array([cfg["actuation"]["v_max"], cfg["actuation"]["vy_max"],
                         cfg["actuation"]["w_max"]], dtype=float)
    dt = float(cfg["sim"]["dt"])
    controller.reset()
    records, contact = [], False
    with (run_dir / f"trace_{label}.jsonl").open("w", encoding="utf-8") as f:
        for step in range(1, spec.steps + 1):
            xy = inverse.apply([[float(state.pose[0]), float(state.pose[1])]])[0]
            pose = (float(xy[0]), float(xy[1]), inverse.apply_yaw(float(state.pose[2])))
            inp, ts = classical_input(tracker, pose, state.scan.filter_ranges,
                                      state.scan.filter_bearings,
                                      bool(state.certified["stop_active"]), state.velocity, dt,
                                      valid=state.scan.filter_valid)
            base_request = np.asarray(getattr(controller, "base", controller).command(inp))
            # As NavEnv does: the normalised action, scaled back by each axis's own ceiling.
            command = np.clip(controller.act(inp), -1.0, 1.0) * ceilings
            state = loop.step(command)
            contact = contact or bool(state.in_contact)
            cert = dict(state.certified)
            clearance = float(np.min(polygon_distance(np.array([pose[:2]]), shelf)))
            records.append({"step": step, "s": float(ts.s), "cert": cert,
                            "base_request": base_request.tolist(),
                            "scan": ScanStream.of(state.scan.filter_ranges,
                                                  state.scan.filter_bearings,
                                                  state.scan.filter_valid),
                            "centre_clearance": clearance})
            f.write(json.dumps({"step": step, "s": float(ts.s), "pose_start_frame": list(pose),
                                "velocity": state.velocity.tolist(),
                                "requested": state.requested.tolist(),
                                "applied": state.applied.tolist(), "certified": cert,
                                "in_contact": bool(state.in_contact),
                                "centre_clearance_m": clearance,
                                "mode": getattr(controller, "mode", None)}, default=float) + "\n")
    # analyse() reads the state AFTER each step; the records hold that state's scan and flags.
    return {"analysis": analyse(records, _layer(cfg)), "contact": contact,
            "report": controller.episode_report() if hasattr(controller, "episode_report")
            else None}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    problem = freeze_problem(cfg)
    if problem:
        raise SystemExit(problem)
    spec = ControlSpec.from_file(SPEC_PATH)
    route = plan_route(cfg, spec)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"CONTROL_RECOVERY_SYNTHETIC_{stamp}",
                             args.seed, cfg)
    prediction = predict(cfg, spec, lambda: PurePursuit.from_config(cfg),
                         lambda b: Recovery.from_config(b, cfg))
    (run_dir / "prediction.json").write_text(json.dumps(
        {k: v for k, v in prediction.items() if k != "route"}, indent=2, default=lambda o: None),
        encoding="utf-8")

    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))
    loop = StepLoop(client, cfg)
    shapes = LayoutShapes(client, cfg)
    client.stop()
    loop.install()
    try:
        bare = run_arm(client, loop, shapes, cfg, spec, route, PurePursuit.from_config(cfg),
                       "bare", run_dir)
        rec_ctl = Recovery.from_config(PurePursuit.from_config(cfg), cfg)
        rec = run_arm(client, loop, shapes, cfg, spec, route, rec_ctl, "recovery", run_dir)
    finally:
        shapes.remove()
        loop.remove()
        client.stop()

    criteria = judge(spec, rec["analysis"], bare["analysis"], rec["report"])
    criteria["route_clearance"] = route_clearance(route, spec) >= spec.min_route_clearance
    criteria["no_contact"] = not (bare["contact"] or rec["contact"])
    summary = {"criteria": criteria, "all_pass": all(criteria.values()),
               "route_clearance_m": route_clearance(route, spec),
               "recovery": rec, "bare": bare,
               "prediction_all_pass": prediction["all_pass"],
               "predicted_criteria_by_slip": {str(k): v["criteria"]
                                              for k, v in prediction["by_slip"].items()}}
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=float),
                                          encoding="utf-8")
    for name, ok in criteria.items():
        print(f"  {name:26s} {'PASS' if ok else 'FAIL'}")
    a = rec["analysis"]
    st = a.get("stop_state") or {}
    print(f"recovery arm: first stop step {a.get('first_stop_step')}, at-rest margin "
          f"{1e3 * st.get('at_rest_margin', math.nan):.1f} mm, blocked "
          f"{st.get('restart_permitted') is False}, released step {a.get('released_step')}, "
          f"progress after release {a.get('progress_after_release', 0.0):.3f} m; "
          f"{rec['report']['recovery_endings']}, deadlock {rec['report']['recovery_deadlock']}")
    print(f"bare arm: progress after its stop {bare['analysis'].get('progress_after_stop', 0):.3f} m")
    print(f"emulation predicted all_pass={prediction['all_pass']}; "
          f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
