"""The PFB acceptance step IN THE SIMULATOR (WP2 step 4, 2026-10-24): the emulator's acceptance step
(`scripts/pfb_acceptance.py`, ruled 2026-10-23) repeated through the real step loop, so the PFB is
judged on the real filter stream, the real slip and the real certified layer. The RESEARCHER runs it
(it drives the simulator over a scenario set: hard rule 11).

    python scripts/pfb_sim_acceptance.py --config configs/analysis/pfb_acceptance_sim.yaml --seed 0

**Declared before running** (the emulator step's criteria, unchanged):
  * the tune routes (tune_narrow, tune_medium, tune_wide; `--limit N` for a smoke run of the first
    N), each run once through `NavEnv` with the PFB on (`filter.enabled`, alpha provisional) and the
    certified layer beneath, driven by the ideal tracker (`baselines.ideal_tracker`), until the
    environment ends the episode or the declared STALL cut-off (route progress under 0.05 m in the
    last 30 s) ends it;
  * every certified stop attributed exactly as in the emulator (`attribute_stops`): unobserved,
    infeasible, creep, or prediction (regime / heading / yaw / motion). **Unattributed stops must be
    zero; otherwise stop and report.** Stops decided on a scan-faulted packet are counted apart (A25)
    and not attributed;
  * contact is the simulator's contact check (hard rule 4): **zero contact steps**;
  * from every fallback state, pure maximum braking rolled forward in the layout must never touch
    anything (Q3); rolled at slip 1.0, the longest travel, so the check is conservative.
Also reported: the PFB's own time per step (mean, p95, max) and steps over the budget.

The field heading the certified layer used is read from the layer after each step, and the pose
from the loop's state: an analysis read, never given to the controller (ruled 2026-10-16).

Writes `runs/analysis/PFB_SIM_ACCEPTANCE[_SMOKE]_<stamp>_seed<seed>/`: routes.json and summary.json.
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.control.ideal_tracker import IdealTracker
from pfbnav.envs.freeze import freeze_problem
from pfbnav.envs.nav_env import NavEnv
from pfbnav.eval.pfb_acceptance import (
    attribute_stops,
    braking_contact,
    footprint_polygon,
    summarise,
    yaw_profile,
    yaw_summary,
)
from pfbnav.eval.ring import StepRing
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.scenarios import Scenario, load_set, verify, world_of

REPO_ROOT = Path(__file__).resolve().parents[1]
SETS = ("scenarios/tune/tune_narrow.json", "scenarios/tune/tune_medium.json",
        "scenarios/tune/tune_wide.json")
STALL_S, STALL_M = 30.0, 0.05   # s, m: the emulator step's declared stall cut-off
ROLLOUT_SLIP = 1.0              # the fallback rollout at full travel (conservative)


TRACE_BEFORE, TRACE_AFTER = 20, 5   # steps of full record kept around every stop (traces/)


def route_record(state, prev, heading: float, s_m: float, pose_layout,
                 pfb_state=None) -> dict[str, Any]:
    """One step as `attribute_stops` reads it: the certified decision and the PFB's, the decision
    taken on the packet in hand at the pose before this step's physics. Also everything needed to
    re-solve the PFB's decision offline (2026-10-26, for the traces): the scan it filtered on, the
    velocity estimate and applied command it was handed, and its own state (`_heading`,
    `_at_rest`) before the call -- analysis reads, never given to the controller."""
    return {"step": state.step, "s": float(s_m), "cert": dict(state.certified), "pfb": state.pfb,
            "field_heading": float(heading),
            "scan": SimpleNamespace(valid=np.asarray(prev.scan.filter_valid, dtype=bool)),
            "scan_ranges": np.asarray(prev.scan.filter_ranges, dtype=float).tolist(),
            "scan_bearings": np.asarray(prev.scan.filter_bearings, dtype=float).tolist(),
            "velocity_before": np.asarray(prev.velocity, dtype=float).tolist(),
            "pfb_state_before": None if pfb_state is None else [float(pfb_state[0]),
                                                                bool(pfb_state[1])],
            "decision_pose": np.asarray(prev.pose, dtype=float).tolist(),
            "decision_pose_layout": [float(x) for x in pose_layout],
            "applied_before": np.asarray(prev.applied, dtype=float).tolist(),
            "decided_on_fault": bool(state.decided_on_fault),
            "pfb_ms": 1e3 * float(state.timings_s.get("pfb", 0.0)),
            "over_budget": bool(state.over_budget), "in_contact": bool(state.in_contact),
            "applied": np.asarray(state.applied, dtype=float).tolist()}


def write_traces(records: list[dict[str, Any]], stops: list[dict[str, Any]], seed: int,
                 trace_dir: Path) -> list[str]:
    """For every stop, the full records from TRACE_BEFORE steps before to TRACE_AFTER after."""
    trace_dir.mkdir(parents=True, exist_ok=True)
    names = []
    by_step = {r["step"]: i for i, r in enumerate(records)}
    for st in stops:
        i = by_step[st["step"]]
        window = [{k: v for k, v in r.items() if k != "scan"} | {
            "scan_valid": np.asarray(r["scan"].valid, dtype=bool).tolist()}
            for r in records[max(0, i - TRACE_BEFORE):i + TRACE_AFTER + 1]]
        name = f"{seed}_{st['step']}_{st['category']}.json"
        (trace_dir / name).write_text(json.dumps({"stop": st, "records": window}, default=float),
                                      encoding="utf-8")
        names.append(name)
    return names


def run_route(env: NavEnv, index: int, scenario: dict[str, Any], tracker: IdealTracker,
              cfg: dict[str, Any], footprint, captured: list,
              trace_dir: Path | None = None, *, ring: StepRing) -> dict[str, Any]:
    from shapely.geometry import Polygon

    dt = float(cfg["sim"]["dt"])
    stall_steps = round(STALL_S / dt)
    tracker.reset()
    captured.clear()
    _, info = env.reset(options={"index": index})
    layout = world_of(Scenario(**scenario), cfg)
    obstacles = [Polygon(np.asarray(o.polygon, dtype=float)[:, :2]) for o in layout.obstacles]
    records: list[dict[str, Any]] = []
    stalled = done = False
    ring.begin(index)
    while not done:
        _, _, terminated, truncated, info = env.step(tracker.act(info["controller_input"]))
        done = bool(terminated or truncated)
        ring.record(len(records) + 1, info)
        (prev, _, pfb_before), (state, heading, _) = captured[-2], captured[-1]
        records.append(route_record(state, prev, heading, info["s_m"],
                                    env._pose_in_layout(prev.pose), pfb_before))
        if not done and len(records) > stall_steps and \
                records[-1]["s"] - records[-1 - stall_steps]["s"] < STALL_M:
            stalled = done = True
    faulted = sum(1 for r in records if r["decided_on_fault"] and r["cert"]["stop_triggered"])
    stops = attribute_stops([r for r in records if not r["decided_on_fault"]], dt)
    fallbacks = [r for r in records if r["pfb"] and r["pfb"]["infeasible"]]
    rollout = sum(int(braking_contact(r["decision_pose_layout"], r["applied_before"], env.loop.pfb,
                                      footprint, obstacles, dt=dt, slip=ROLLOUT_SLIP))
                  for r in fallbacks)
    reasons: dict[str, int] = {}
    for r in fallbacks:
        reasons[r["pfb"]["reason"]] = reasons.get(r["pfb"]["reason"], 0) + 1
    ms = np.array([r["pfb_ms"] for r in records]) if records else np.zeros(1)
    outcome = (info.get("episode") or {}).get("outcome", "stalled" if stalled else None)
    traces = write_traces(records, stops, scenario["seed"], trace_dir) if trace_dir else []
    ring_dumps = ring.end(outcome or "running")
    # The yaw rates used, by field in force and at bends, along the route the environment tracks
    # (ruled 2026-10-26; an analysis read of `env.tracker.path`, the layout-frame route).
    yaw = yaw_profile(records, np.asarray(env.tracker.path, dtype=float))
    return {"seed": scenario["seed"], "stratum": scenario["stratum"], "outcome": outcome,
            "traces": traces, "ring_dumps": ring_dumps, "_yaw": yaw,
            "steps": len(records), "reached_goal": outcome == "success", "stalled": stalled,
            "final_s_m": records[-1]["s"] if records else 0.0,
            "stops": stops, "scan_fault_stops": faulted,
            "fallback_steps": len(fallbacks), "fallback_reasons": reasons,
            "fallback_rollout_contacts": rollout,
            "contact_steps": sum(int(r["in_contact"]) for r in records),
            "modes": {m: sum(1 for r in records if r["pfb"] and r["pfb"]["mode"] == m)
                      for m in ("nominal", "filtered", "brake", "fallback", "latched")},
            "pfb_ms": {"mean": float(ms.mean()), "p95": float(np.percentile(ms, 95)),
                       "max": float(ms.max())},
            "over_budget_steps": sum(int(r["over_budget"]) for r in records)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--limit", type=int, default=None, help="a SMOKE run: the first N routes")
    parser.add_argument("--seeds", type=int, nargs="+", default=None,
                        help="a TRACE run: only these scenario seeds (a diagnosis, not the step)")
    add_display_flag(parser)
    args = parser.parse_args()
    cfg = apply_display(load_config(args.config), args.display)
    if not cfg["filter"]["enabled"]:
        raise SystemExit("the acceptance step needs filter.enabled: true "
                         "(configs/analysis/pfb_acceptance_sim.yaml)")
    problem = freeze_problem(cfg)
    if problem:
        raise SystemExit(problem)
    seed_everything(args.seed)
    scenarios: list[dict[str, Any]] = []
    for s in SETS:
        manifest = load_set(REPO_ROOT / s)
        checked = verify(manifest, cfg)
        if not checked["ok"]:
            raise SystemExit(f"{s} fails verification: {checked['problems'][:3]}")
        scenarios += manifest["scenarios"]
    if args.seeds:
        missing = sorted(set(args.seeds) - {int(s["seed"]) for s in scenarios})
        if missing:
            raise SystemExit(f"no tune scenario with seed(s) {missing}")
        scenarios = [s for s in scenarios if int(s["seed"]) in set(args.seeds)]
    if args.limit:
        scenarios = scenarios[:args.limit]
    smoke = bool(args.limit or args.seeds)
    label = "_TRACE" if args.seeds else "_SMOKE" if args.limit else ""
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "analysis",
                             f"PFB_SIM_ACCEPTANCE{label}_{stamp}", args.seed, cfg)
    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    footprint = footprint_polygon(table.extent)
    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))
    env = NavEnv(client, cfg, scenarios, training=False, seed=args.seed)
    # Capture every StepState, the layer's field heading after it, and the PFB's own state after it
    # -- which is its state before the next step's call (analysis reads).
    captured: list = []
    loop_reset, loop_step = env.loop.reset, env.loop.step

    def pfb_state():
        return (env.loop.pfb._heading, env.loop.pfb._at_rest)

    def reset(*a, **k):
        state = loop_reset(*a, **k)
        captured.append((state, None, pfb_state()))
        return state

    def step(command):
        state = loop_step(command)
        captured.append((state, float(env.loop.certified._heading), pfb_state()))
        return state

    env.loop.reset, env.loop.step = reset, step
    tracker = IdealTracker.from_config(cfg)
    # The per-step ring buffer (ruled 2026-10-23; required for this step's rerun, 2026-10-26): dumps
    # under ring/ on stationary triggers, scan faults, contacts and unrecoverable stops. Every
    # protective stop also gets its re-solvable trace under traces/.
    ring = StepRing(int(cfg["evaluation"]["ring_steps"]), run_dir / "ring")
    routes: list[dict[str, Any]] = []
    try:
        for i, scenario in enumerate(scenarios):
            res = run_route(env, i, scenario, tracker, cfg, footprint, captured,
                            trace_dir=run_dir / "traces", ring=ring)
            routes.append(res)
            cats = [st["category"] for st in res["stops"]]
            print(f"[{i + 1}/{len(scenarios)}] {res['stratum']} {res['seed']}: {res['outcome']} in "
                  f"{res['steps']} steps, stops {cats}, scan-fault stops "
                  f"{res['scan_fault_stops']}, fallbacks {res['fallback_steps']}, rollout contacts "
                  f"{res['fallback_rollout_contacts']}, contact steps {res['contact_steps']}, "
                  f"PFB {res['pfb_ms']['mean']:.0f}/{res['pfb_ms']['p95']:.0f} ms", flush=True)
    finally:
        env.close()
    summary = summarise(routes)
    summary["yaw_by_field"] = yaw_summary([r.pop("_yaw") for r in routes])
    summary.update(smoke=smoke, steps=sum(r["steps"] for r in routes),
                   scan_fault_stops=sum(r["scan_fault_stops"] for r in routes),
                   over_budget_steps=sum(r["over_budget_steps"] for r in routes))
    (run_dir / "routes.json").write_text(json.dumps(routes, indent=1, default=float),
                                         encoding="utf-8")
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0 if summary["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
