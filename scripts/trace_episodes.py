"""Offline diagnostics on EXISTING checkpoints and tuned controllers (ruled 2026-11-07, items 3a, 3b,
3e; additional item 4): a per-step trace of chosen episodes, then their analysis. No training. The
RESEARCHER runs it (a few episodes on one simulator instance).

    python scripts/trace_episodes.py --config configs/experiments/L5_sac_pfb_unshaped.yaml --seed 0 --controller sac --checkpoint runs/train/L5_sac_pfb_unshaped_seed9000002/final_model.zip --sets scenarios/tune/tune_narrow.json scenarios/tune/tune_medium.json scenarios/tune/tune_wide.json --from-run runs/eval/L5_sac_pfb_unshaped_20261106T130110Z_seed0 --pick wander

**Which episodes.** `--indices` names them (indices into the concatenated `--sets`), or `--from-run`
with `--pick` takes them from a finished run's `episodes.jsonl`: `wander` (driven distance over the
planned length above `--ratio`), `stops` (any protective stop) or `failed` (not a success), at most
`--max`. Each picked index is checked against the run's recorded `scenario_seed`, so `--sets` must be
the run's sets in its order; a mismatch is refused.

**Controllers:** as `scripts/evaluate.py` builds them (the classical ones under the declared route
cap and the recovery). `--filter on` turns the PFB on at the frozen alpha (B7 is
`regulated_pure_pursuit --filter on`). For `sac` the action is the DETERMINISTIC mean action, as in
every evaluation; `--stochastic` samples it as in training (a diagnostic contrast only).

**Not a replay.** A trace runs in its own scene load, and determinism holds within one load only
(ruled 2026-10-14), so a traced episode need not repeat the recorded one; the summary prints both.

Writes `runs/diag/TRACE2_<stamp>_seed<seed>/`: `trace_<index>.jsonl` (one row per step: layout-frame
pose, route arc, cross-track, goal distance, requested/applied command, velocity estimate, certified
flags, the PFB's decision, the recovery's mode, and at each stop trigger the distance from the robot's
CENTRE to the nearest mapped and nearest unmapped surface) and `summary.json`
(`pfbnav.eval.trace_analysis`: wandering anatomy, stop causes, latch anatomy, per episode).
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.control.base import input_kind
from pfbnav.envs.nav_env import NavEnv
from pfbnav.eval.route_profiles import build_profiles
from pfbnav.eval.runner import EPISODE_IDENTITY, _given
from pfbnav.eval.trace_analysis import latch_anatomy, stop_causes, wandering
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.training.record import _jsonable
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.baking import polygon_distance
from pfbnav.world.scenarios import load_set, verify

REPO_ROOT = Path(__file__).resolve().parents[1]


def _evaluate():
    spec = importlib.util.spec_from_file_location("evaluate", REPO_ROOT / "scripts" / "evaluate.py")
    if spec is None or spec.loader is None:
        raise SystemExit("cannot load scripts/evaluate.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pick(records: list[dict[str, Any]], how: str, ratio: float, most: int) -> list[int]:
    """Indices of a finished run's episodes chosen by `how` (module docstring), in index order."""
    def chosen(r: dict[str, Any]) -> bool:
        if how == "wander":
            plan = float(r.get("plan_length_m") or 0.0)
            return plan > 0 and float(r.get("distance_m", 0.0)) / plan > ratio
        if how == "stops":
            return int(r.get("protective_stops", 0)) > 0
        if how == "failed":
            return r["outcome"] != "success"
        raise ValueError(f"unknown pick {how!r}")
    return sorted(int(r["index"]) for r in records if chosen(r))[:most]


def _nearest(xy: np.ndarray, layout) -> dict[str, float | None]:
    def near(obstacles) -> float | None:
        return min((float(polygon_distance(xy, o.polygon)[0]) for o in obstacles), default=None)
    return {"mapped_m": near(layout.mapped), "unmapped_m": near(layout.unmapped)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--controller", required=True)
    parser.add_argument("--checkpoint", default=None)
    parser.add_argument("--stochastic", action="store_true", help="sac only: sample the action")
    parser.add_argument("--filter", choices=["on", "off"], default=None)
    parser.add_argument("--sets", nargs="+", required=True)
    parser.add_argument("--indices", type=int, nargs="*", default=None)
    parser.add_argument("--from-run", default=None)
    parser.add_argument("--pick", choices=["wander", "stops", "failed"], default=None)
    parser.add_argument("--ratio", type=float, default=1.5)
    parser.add_argument("--max", type=int, default=6)
    parser.add_argument("--workers", type=int, default=4)
    add_display_flag(parser)
    args = parser.parse_args()

    cfg = apply_display(load_config(args.config), args.display)
    if args.filter is not None:
        cfg["filter"]["enabled"] = args.filter == "on"
    seed_everything(args.seed)
    ev = _evaluate()
    paths = ev.preflight(cfg, args.sets, args.checkpoint, args.controller)   # every refusal
    scenarios = []
    for path in paths:
        manifest = load_set(path)
        if not verify(manifest, cfg)["ok"]:
            raise SystemExit(f"{path.name} fails verification")
        scenarios += manifest["scenarios"]
    recorded: dict[int, dict[str, Any]] = {}
    if args.from_run:
        recs = [json.loads(line) for line in (REPO_ROOT / args.from_run / "episodes.jsonl")
                .read_text(encoding="utf-8").splitlines() if line]
        recorded = {int(r["index"]): r for r in recs}
        indices = pick(recs, args.pick or "failed", args.ratio, args.max)
    else:
        indices = list(args.indices or [])
    if not indices:
        raise SystemExit("no episodes to trace (give --indices, or --from-run with --pick)")
    for i in indices:
        if i in recorded and int(recorded[i]["scenario_seed"]) != int(scenarios[i]["seed"]):
            raise SystemExit(f"index {i}: the run recorded scenario {recorded[i]['scenario_seed']}, "
                             f"--sets give {scenarios[i]['seed']}: pass the run's sets in its order")
    kind = ev.PROFILE_KIND.get(args.controller)
    profiles = (build_profiles([scenarios[i] for i in range(len(scenarios))], cfg, kind,
                               workers=args.workers) if kind else None)
    controller = ev.make_controller(args.controller, cfg, args.checkpoint, profiles)
    if args.controller == "sac":
        controller.deterministic = not args.stochastic
    dt = float(cfg["sim"]["dt"])

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"TRACE2_{stamp}", args.seed, cfg)
    (run_dir / "traced.json").write_text(json.dumps({
        "controller": args.controller, "checkpoint": args.checkpoint,
        "action": ("stochastic" if args.stochastic else "deterministic") if args.controller == "sac"
        else None, "filter": bool(cfg["filter"]["enabled"]), "from_run": args.from_run,
        "pick": args.pick, "indices": indices}, indent=2), encoding="utf-8")
    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))
    env = NavEnv(client, cfg, scenarios, training=False, seed=args.seed)
    captured: dict[str, Any] = {}
    original = env.loop.step

    def capturing(command):
        captured["state"] = original(command)
        return captured["state"]

    env.loop.step = capturing
    kind_in = input_kind(controller)
    summary = []
    try:
        for index in indices:
            controller.reset()
            obs, info = env.reset(options={"index": int(index)})
            begin = getattr(controller, "begin_episode", None)
            if begin is not None:
                begin({k: info[k] for k in EPISODE_IDENTITY})
            rows, done, step = [], False, 0
            while not done:
                action = controller.act(_given(kind_in, obs, info))
                obs, _reward, terminated, truncated, info = env.step(action)
                s = captured["state"]
                step += 1
                done = bool(terminated or truncated)
                x, y, yaw = (float(v) for v in env._pose_in_layout(s.pose))
                c = dict(info["certified"])
                row = {"step": step, "pose": [x, y, yaw], "s_m": info["s_m"],
                       "cross_track_m": info["cross_track_m"],
                       "goal_distance_m": info["goal_distance_m"],
                       "requested": info["requested"], "applied": info["applied"],
                       "velocity": np.asarray(info["controller_input"].velocity).tolist(),
                       "certified": c, "pfb": _jsonable(info.get("pfb")),
                       "recovery": ({"mode": getattr(controller, "mode", None)}
                                    if hasattr(controller, "mode") else None),
                       "nearest_surface": (_nearest(np.array([[x, y]]), env.layout)
                                           if c.get("stop_triggered") else None),
                       "outcome": info.get("outcome")}
                rows.append(row)
            with (run_dir / f"trace_{index}.jsonl").open("w", encoding="utf-8") as f:
                for row in rows:
                    f.write(json.dumps(row, default=float) + "\n")
            plan = float(env.plan.length_m)
            rec = recorded.get(index, {})
            entry = {"index": index, "scenario_seed": int(scenarios[index]["seed"]),
                     "stratum": scenarios[index]["stratum"],
                     "unmapped_count": int(scenarios[index]["unmapped_count"]),
                     "traced": {"outcome": info.get("outcome"), "steps": step},
                     "recorded": {k: rec.get(k) for k in ("outcome", "distance_m", "steps",
                                                          "protective_stops")} if rec else None,
                     "wandering": wandering(rows, plan_length_m=plan, dt=dt),
                     "stop_causes": stop_causes(rows), "latch": latch_anatomy(rows)}
            summary.append(entry)
            w = entry["wandering"]
            print(f"index {index} ({entry['stratum']}): {info.get('outcome')} in {step} steps; "
                  f"driven {w['driven_m']:.1f} / plan {plan:.1f} m by thirds "
                  f"{[round(v, 1) for v in w['driven_by_route_third_m']]}; cross-track p95 "
                  f"{w['cross_track_p95_m']:.2f} m; overshoot {w['goal_overshoot']}, circling "
                  f"{w['circling_near_goal']}; stops {[c['cause'] for c in entry['stop_causes']]}"
                  + (f"; recorded {rec.get('outcome')} {rec.get('distance_m', 0):.1f} m" if rec
                     else ""))
    finally:
        env.close()
        client.stop()
        (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=float),
                                              encoding="utf-8")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
