"""Tune the PFB's gain alpha with B7, then compare B2 and B7 on the tune set. The RESEARCHER runs
this (hard rule 11), after B2's tuning:

    python scripts/tune_alpha.py --config configs/base.yaml --seed 0 \
        --b2-tuning runs/tuning/B2_regulated_pure_pursuit_<stamp>_seed0/summary.json \
        --g2 runs/gates/G2_<stamp>_seed9000000/summary.json --instances <k>

**Declared** (A14a, `preregistration/endpoints.yaml`, `tuning.pfb_alpha`; ruled 2026-10-24):
  * the grid: `grid_per_s`, read from the preregistration, never restated;
  * B7 = B2's TUNED parameters plus the PFB, nothing else: `baselines.regulated_pure_pursuit` in the
    config must equal the B2 tuning run's selected parameters (they are written into
    configs/base.yaml by hand after B2's review, with a decision-log entry), and the controller is
    built exactly as B2's is (`scripts/evaluate.py`: route cap and classical recovery);
  * the set: the three tune sets at the training unmapped rate, each scenario once per grid value,
    every grid value in ONE scene load (the PFB is rebuilt with each alpha between candidates; the
    environment is otherwise the same object);
  * the selection: among the settings within 5 percentage points of the best success rate, the
    fewest protective stops per 100 m; ties by mission time. A selection at either end of the grid
    is reported, never extended.

**Then the B2-versus-B7 comparison** (ruled 2026-10-26), per stratum and pooled: protective stops per
100 m, success rate, mission time, pfb_infeasible and fallback steps, for B2's selected candidate (from
its tuning run) and B7 at the selected alpha. **TUNE-SET EVIDENCE FOR MONITORING AND FRAMING ONLY:
never cited as a confirmatory result; it alters no preregistered analysis.** B2's episodes come from
its own tuning run, another scene load, so the comparison is not paired within one load.

Refuses: the conditions of `scripts/tune_classical.py` (test sets, a non-frozen environment, an
instance count G2 has not approved on this host, an uncommitted budget file), and a config whose
B2 parameters are not the tuned ones. Nothing is written back: alpha is frozen for L3, L4, L5 and B7
by hand, after review, with a decision-log entry.

Writes `runs/tuning/B7_alpha_<stamp>_seed<seed>/`: config.yaml, provenance.json, candidates.json,
candidate_<i>/episodes.jsonl and ring/, route_profiles.json, summary.json, and
b2_vs_b7_tune_monitoring.json.
"""
from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import math
import platform
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from pfbnav.config import load_config
from pfbnav.envs.freeze import environment_identity, freeze_problem
from pfbnav.envs.nav_env import NavEnv
from pfbnav.eval.g4 import recovery_statistics
from pfbnav.eval.ring import StepRing
from pfbnav.eval.route_profiles import build_profiles, write_profiles
from pfbnav.eval.runner import run_episodes
from pfbnav.eval.tuning import monitoring_row, select, stratum_comparison
from pfbnav.safety.pfb import PFB, PFBConfig
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.hashing import text_sha256
from pfbnav.utils.memory import MemoryRecord
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.scenarios import load_set, verify

REPO_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = "regulated_pure_pursuit"
BLOCK = "baselines.regulated_pure_pursuit"
LABEL = ("TUNE-SET EVIDENCE FOR MONITORING AND FRAMING ONLY (ruled 2026-10-26): never cited as a "
         "confirmatory result; alters no preregistered analysis. B2 from its tuning run (another "
         "scene load), B7 at the selected alpha.")


def _module(name: str):
    spec = importlib.util.spec_from_file_location(name, REPO_ROOT / "scripts" / f"{name}.py")
    if spec is None or spec.loader is None:
        raise SystemExit(f"cannot load scripts/{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def b2_problem(cfg: dict[str, Any], b2: dict[str, Any]) -> str | None:
    """Why the config's B2 parameters are not the B2 tuning run's selection, or None."""
    if b2.get("controller") != CONTROLLER:
        return f"the B2 summary is for {b2.get('controller')!r}"
    chosen = b2["selected_parameters"]
    have = cfg["baselines"]["regulated_pure_pursuit"]
    if set(chosen) != set(have):
        return f"parameter sets differ: {sorted(set(chosen) ^ set(have))}"
    off = {k: (float(have[k]), float(v)) for k, v in chosen.items()
           if not math.isclose(float(have[k]), float(v), rel_tol=1e-9, abs_tol=0.0)}
    if off:
        return (f"configs' B2 parameters are not the tuned ones (config, tuned): {off}; copy the "
                "selected values exactly")
    return None


def with_alpha(cfg: dict[str, Any], alpha: float) -> dict[str, Any]:
    out = copy.deepcopy(cfg)
    out["filter"]["enabled"] = True
    out["filter"]["alpha"] = float(alpha)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--b2-tuning", required=True, help="B2's tuning summary.json")
    parser.add_argument("--g2", required=True, help="the G2 summary.json that approves --instances")
    parser.add_argument("--instances", type=int, required=True)
    parser.add_argument("--workers", type=int, default=4, help="processes for the route profiles")
    add_display_flag(parser)
    args = parser.parse_args()
    tc = _module("tune_classical")
    cfg = apply_display(load_config(args.config), args.display)

    problem = (tc.committed_problem(tc.BUDGET)
               or tc.g2_problem(Path(args.g2), args.instances, platform.node()))
    if problem:
        raise SystemExit(problem)
    b2_path = Path(args.b2_tuning)
    b2 = json.loads(b2_path.read_text(encoding="utf-8"))
    problem = b2_problem(cfg, b2)
    if problem:
        raise SystemExit(problem)
    if cfg["filter"]["enabled"]:
        raise SystemExit("filter.enabled is already true in the config; this runner enables it")
    space_doc = {"controller": CONTROLLER, "config_block": BLOCK}
    paths = tc.preflight(cfg, space_doc, list(tc.SETS), 1)      # test sets, freeze, the filter off
    endpoints = yaml.safe_load((REPO_ROOT / "preregistration" / "endpoints.yaml")
                               .read_text(encoding="utf-8"))
    grid = [float(a) for a in endpoints["tuning"]["pfb_alpha"]["grid_per_s"]]
    window = float(yaml.safe_load(tc.BUDGET.read_text(encoding="utf-8"))["selection"]
                   ["success_window"])
    on = with_alpha(cfg, grid[0])
    problem = freeze_problem(on)
    if problem:
        raise SystemExit(problem)
    seed_everything(args.seed)

    scenarios: list[dict[str, Any]] = []
    set_of: list[str] = []
    for path in paths:
        manifest = load_set(path)
        checked = verify(manifest, cfg)
        if not checked["ok"]:
            raise SystemExit(f"{path.name} fails verification: {checked['problems'][:3]}")
        scenarios += manifest["scenarios"]
        set_of += [path.stem] * len(manifest["scenarios"])
    print(f"route profiles (cap): {len(scenarios)} scenarios, {args.workers} workers", flush=True)
    profiles = build_profiles(scenarios, cfg, "cap", workers=args.workers)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "tuning", f"B7_alpha_{stamp}", args.seed, on)
    (run_dir / "candidates.json").write_text(json.dumps({
        "grid_per_s": grid, "b2_tuning": b2_path.as_posix(),
        "b2_parameters": b2["selected_parameters"],
        "environment": environment_identity(on)["sha256"],
        "sets": {p.relative_to(REPO_ROOT).as_posix(): text_sha256(p) for p in paths},
        "g2": {"summary": Path(args.g2).as_posix(), "instances": args.instances}},
        indent=1), encoding="utf-8")
    write_profiles(run_dir / "route_profiles.json", profiles)

    evaluate = _module("evaluate")
    dt = float(cfg["sim"]["dt"])
    client = SimClient(SimConnection.from_config(on))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))       # ONE load for every alpha
    env = NavEnv(client, on, scenarios, training=False, seed=args.seed)
    memory = MemoryRecord(client.connection.port)
    scores: list[dict[str, Any]] = []
    recovery: list[dict[str, Any]] = []
    records_of: list[list[dict[str, Any]]] = []
    try:
        for i, alpha in enumerate(grid):
            cfg_a = with_alpha(cfg, alpha)
            if env.loop.pfb is None:
                raise RuntimeError("the step loop has no PFB although the filter is enabled")
            field = env.loop.pfb.field
            env.loop.pfb = PFB(field, PFBConfig.from_config(cfg_a, field))
            if env.loop.pfb.c.alpha != alpha:                       # the swap took effect
                raise RuntimeError(f"the PFB runs alpha {env.loop.pfb.c.alpha}, not {alpha}")
            out = run_dir / f"candidate_{i}"
            out.mkdir()
            log = out / "episodes.jsonl"

            def write(record: dict[str, Any], log=log, alpha=alpha) -> None:
                record["set"] = set_of[record["index"]]
                record["alpha"] = alpha
                with log.open("a", encoding="utf-8") as f:
                    f.write(json.dumps(record, default=float) + "\n")

            ctrl = evaluate.make_controller(CONTROLLER, cfg_a, None, profiles)
            records = run_episodes(env, ctrl, range(len(scenarios)), on_episode=write,
                                   ring=StepRing(int(cfg["evaluation"]["ring_steps"]), out / "ring"))
            records_of.append(records)
            score = monitoring_row(records, dt)
            scores.append(score)
            recovery.append(recovery_statistics(records))
            print(f"[alpha {alpha:g}] success {score['success_rate']:.3f}, stops/100 m "
                  f"{score['stops_per_100m']}, mission time {score['mission_time_s']}, "
                  f"infeasible {score['pfb_infeasible']}, fallback {score['pfb_fallback_steps']}",
                  flush=True)
    finally:
        env.close()
        client.stop()
    chosen = select(scores, success_window=window)
    edge = "low" if chosen == 0 else "high" if chosen == len(grid) - 1 else None
    defects = [grid[i] for i, r in enumerate(recovery) if r["suspected_defects"]]
    summary = {"grid_per_s": grid, "scores": scores, "recovery": recovery,
               "suspected_defects_at_alpha": defects, "selected_alpha": grid[chosen],
               "selected_on_grid_edge": edge,
               "rule": "among settings within 5 pp of the best success, the fewest protective stops "
                       "per 100 m; ties by mission time (A14a); an edge is reported, never extended",
               "frozen_for": endpoints["tuning"]["pfb_alpha"]["frozen_for"],
               "memory": memory.report(sum(len(r) for r in records_of))}
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=float),
                                          encoding="utf-8")

    b2_eps = b2_path.parent / f"candidate_{int(b2['selected'])}" / "episodes.jsonl"
    b2_records = [json.loads(x) for x in b2_eps.read_text(encoding="utf-8").splitlines() if x]
    comparison = {"label": LABEL, "b2_episodes": b2_eps.as_posix(), "b7_alpha": grid[chosen],
                  "by_stratum": stratum_comparison({"B2": b2_records, "B7": records_of[chosen]},
                                                   dt)}
    (run_dir / "b2_vs_b7_tune_monitoring.json").write_text(
        json.dumps(comparison, indent=2, default=float), encoding="utf-8")

    print(f"selected alpha {grid[chosen]:g}" + (f" (ON THE GRID'S {edge.upper()} EDGE: reported, "
                                                 "never extended)" if edge else ""))
    print(LABEL)
    for stratum, arms in comparison["by_stratum"].items():
        print(f"  {stratum}: " + "; ".join(
            f"{n} stops/100 m {a['stops_per_100m']}, success {a['success_rate']:.3f}, "
            f"time {a['mission_time_s']}, infeasible {a['pfb_infeasible']}, "
            f"fallback {a['pfb_fallback_steps']}" for n, a in arms.items()))
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    if defects:
        print(f"STOP: suspected recovery defect(s) at alpha {defects}; report before the selection "
              "is read")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
