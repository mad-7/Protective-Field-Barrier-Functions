"""Diagnostic: per-step traces of the recovery positive control. Researcher runs it (~5 min).

    python scripts/diag_recovery_corner.py --config configs/base.yaml --seed 0

The positive control (`tests/test_recovery_sim.py`) replays the 2026-10-13 corner stop (tune_wide
2000057, planned at that day's lethal margin 0.03; `tests/data/corner_stop_20261013.json`). On
2026-10-16, bare B1 reproduced the deadlock, and B1 with recovery cleared it. But the wrapped robot
stopped again 0.15 m further along, at rest with its own field breached (`inside_min_margin`). This
records both arms, in one scene load, so that second stop can be read rather than guessed: what
breached the at-rest field, at what bearing and range, and how the body was turned relative to the
route.

Reads the environment, never changes it: the step loop's `StepState` is captured by wrapping
`env.loop.step` in this process only (as `diag_episode_trace.py` does).

Writes `runs/diag/RECOVERY_CORNER_<timestamp>_seed<seed>/`:
  * `trace_bare.jsonl` and `trace_recovery.jsonl`, one line per step: step, s, cross-track, layout
    pose, the body's yaw relative to the route tangent, the estimated velocity, the requested and
    applied commands, the certified layer's report, and the recovery's mode and counts;
  * `stops.json`: for every stop trigger, and for the final step, the bins inside the AT-REST
    field and inside the field in force (bearing deg, range, field extent, by how much);
  * `summary.json`.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from pfbnav.config import load_config
from pfbnav.control.pure_pursuit import PurePursuit
from pfbnav.control.recovery import Recovery, RecoveryGeometry
from pfbnav.envs.freeze import freeze_problem
from pfbnav.envs.nav_env import NavEnv
from pfbnav.planning.lookahead import point_at
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURE = REPO_ROOT / "tests" / "data" / "corner_stop_20261013.json"
STEPS = 500
SETTLE = 10


def _tangent_yaw(path: np.ndarray, s: float) -> float:
    a, b = point_at(path, max(s - 0.01, 0.0)), point_at(path, s + 0.01)
    return float(math.atan2(b[1] - a[1], b[0] - a[0]))


def _inside(scan, field) -> list[dict]:
    out = []
    for b, r, v, f in zip(scan.bearings, scan.ranges, scan.valid, field, strict=True):
        if not v or r < f:
            out.append({"bearing_deg": math.degrees(float(b)), "range_m": float(r),
                        "field_m": float(f), "by_m": float(f - r) if v else None,
                        "valid": bool(v)})
    return out


def _run(env, controller, geom, label, run_dir) -> dict:
    captured: dict = {}
    original = env.loop.step

    def capture(command):
        state = original(command)
        captured["state"] = state
        return state

    env.loop.step = capture
    stops = []
    step = 0
    try:
        controller.reset()
        _obs, info = env.reset(options={"index": 0})
        path = np.asarray(env.plan.path_world)
        with (run_dir / f"trace_{label}.jsonl").open("w", encoding="utf-8") as f:
            for step in range(1, STEPS + 1):
                inp = info["controller_input"]
                _obs, _r, term, trunc, info = env.step(controller.act(inp))
                st = captured["state"]
                pose = env._pose_in_layout(st.pose)
                yaw_rel = math.degrees(math.remainder(
                    pose[2] - _tangent_yaw(path, float(info["s_m"])), 2 * math.pi))
                cert = dict(st.certified)
                row = {"step": step, "s_m": float(info["s_m"]),
                       "cross_track_m": float(info["cross_track_m"]),
                       "layout_pose": [float(v) for v in pose], "yaw_rel_route_deg": yaw_rel,
                       "velocity": st.velocity.tolist(), "requested": st.requested.tolist(),
                       "applied": st.applied.tolist(), "certified": cert}
                if isinstance(controller, Recovery):
                    row["recovery"] = {"mode": controller.mode, "attempts": controller.attempts,
                                       "gave_up": controller.gave_up}
                f.write(json.dumps(row, default=float) + "\n")
                last = step == STEPS or term or trunc
                if cert["stop_triggered"] or last:
                    nxt = info["controller_input"]
                    speed = float(np.hypot(*nxt.velocity[:2]))
                    heading = float(math.atan2(nxt.velocity[1], nxt.velocity[0]))
                    stops.append({
                        "step": step, "final": bool(last), "s_m": row["s_m"],
                        "layout_pose": row["layout_pose"], "yaw_rel_route_deg": yaw_rel,
                        "speed": speed,
                        "inside_at_rest_field": _inside(nxt.scan, geom.fields.select(
                            nxt.scan.bearings, 0.0, 0.0, at_rest=True)),
                        "inside_field_in_force": _inside(nxt.scan, geom.fields.select(
                            nxt.scan.bearings, speed, heading, at_rest=bool(cert["at_rest"]))),
                        "certified": cert,
                        "recovery": row.get("recovery")})
                if term or trunc:
                    break
    finally:
        env.loop.step = original
    summary = {"label": label, "steps": step, "final_s_m": float(info["s_m"]),
               "stops": [{k: v for k, v in s.items() if not k.startswith("inside")} | {
                   "n_inside_at_rest": len(s["inside_at_rest_field"])} for s in stops]}
    if isinstance(controller, Recovery):
        summary["recovery"] = controller.episode_report()
    return {"summary": summary, "stops": stops}


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
    fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
    run_cfg = copy.deepcopy(cfg)
    run_cfg["planner"]["lethal_margin"] = float(fixture["planner_lethal_margin"])
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"RECOVERY_CORNER_{stamp}", args.seed,
                             run_cfg)
    geom = RecoveryGeometry.from_config(run_cfg)
    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))
    env = NavEnv(client, run_cfg, [fixture["scenario"]], settle_steps=SETTLE, training=False,
                 seed=args.seed)
    try:
        bare = _run(env, PurePursuit.from_config(run_cfg), geom, "bare", run_dir)
        rec = _run(env, Recovery.from_config(PurePursuit.from_config(run_cfg), run_cfg), geom,
                   "recovery", run_dir)
    finally:
        env.close()
        client.stop()
    (run_dir / "stops.json").write_text(json.dumps({"bare": bare["stops"],
                                                    "recovery": rec["stops"]}, indent=2,
                                                   default=float), encoding="utf-8")
    summary = {"bare": bare["summary"], "recovery": rec["summary"]}
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=float),
                                          encoding="utf-8")
    for arm in summary.values():
        print(f"{arm['label']}: final s {arm['final_s_m']:.3f} m after {arm['steps']} steps; "
              f"stops at {[(s['step'], round(s['s_m'], 3)) for s in arm['stops'] if not s['final']]}"
              f"{'; ' + str(arm['recovery']) if 'recovery' in arm else ''}")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
