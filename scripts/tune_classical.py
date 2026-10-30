"""Tune a classical baseline on the tune set: a seeded random search of declared size. The RESEARCHER
runs this (hard rule 11: a tuning sweep is long-duration).

    python scripts/tune_classical.py --config configs/base.yaml --seed 0 \
        --space configs/tuning/b2_regulated_pure_pursuit.yaml \
        --g2 runs/gates/G2_<stamp>_seed9000000/summary.json --instances <k> [--instance <i>]

First use: B2, holonomic Regulated Pure Pursuit (ruled 2026-10-24: tuned under the equal-tuning-budget
rule, with the route cap and the classical recovery; B7 = B2's tuned parameters plus the PFB).

**What is declared before running** (`pfbnav/eval/tuning.py`):
  * the space: `--space`, the parameter ranges, linear or log; candidate 0 is the configured initial
    values; the others are seeded random draws (`--seed`), re-drawn where the controller refuses the
    combination;
  * the budget: `configs/tuning/budget.yaml` (ruled 2026-10-26: 24 candidates, candidate 0
    included, the same for every classical baseline), never a command-line value;
  * the evaluation: each candidate exactly as `scripts/evaluate.py` evaluates the baseline -- under
    the declared route cap (computed once, from the MAPPED world; it does not depend on the
    candidate) and the classical recovery -- on every scenario of the tune sets (`--sets`, default
    the three, at the training unmapped rate), each once, every candidate in ONE scene load so the
    candidates are compared on the same simulator state (determinism is per scene load);
  * the selection (ruled 2026-10-26): among the candidates within 5 percentage points of the best
    success rate, the fewest protective stops per 100 m, ties by mission time. A winner on a range
    boundary is reported (`summary.json["selected_on_boundary"]`), never extended.

**What it refuses:** test layouts (hard rule 2), an environment that is not the frozen one, the
filter enabled (B2 is a baseline without the PFB; B7 is tuned only in alpha), an unknown controller;
a space or budget file that is not committed and unmodified, or that still says VERIFY (ruled: the
ranges are committed before the first candidate runs); and an instance count G2 has not approved on
this host (ruled 2026-10-26: `--g2`, a G2 summary that PASSED on this hostname, with
`approved_instances` >= `--instances`, the simulator instances running on this host meanwhile).

Writes `runs/tuning/<name>_<stamp>_seed<seed>/`:

    config.yaml, provenance.json   as every run
    space.yaml                     the declared space, as read
    candidates.json                every candidate's parameters
    candidate_<i>/episodes.jsonl   one line per episode, written as it finishes
    route_profiles.json            the route cap per scenario
    summary.json                   every candidate's score, the selection and the selected
                                   parameters (to be written into configs/base.yaml BY HAND, with a
                                   decision-log entry, after review)
"""
from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import platform
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pfbnav.config import load_config
from pfbnav.control.dwa import DWAParams, HolonomicDWA
from pfbnav.control.mppi import FieldAwareMPPI, MPPIParams
from pfbnav.control.regulated_pure_pursuit import RegulatedPurePursuit, RPPParams
from pfbnav.envs.freeze import environment_identity, freeze_problem
from pfbnav.envs.nav_env import NavEnv
from pfbnav.eval.g4 import recovery_statistics
from pfbnav.eval.ring import StepRing
from pfbnav.eval.route_profiles import build_profiles, write_profiles
from pfbnav.eval.runner import run_episodes
from pfbnav.eval.tuning import (
    assign_flat,
    candidate_score,
    flatten_numeric,
    on_boundary,
    parse_space,
    sample_candidates,
    select,
)
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.hashing import text_sha256
from pfbnav.utils.memory import MemoryRecord
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.scenarios import load_set, verify

REPO_ROOT = Path(__file__).resolve().parents[1]
TEST_DIR = (REPO_ROOT / "scenarios" / "test").resolve()
BUDGET = REPO_ROOT / "configs" / "tuning" / "budget.yaml"
SETS = ("scenarios/tune/tune_narrow.json", "scenarios/tune/tune_medium.json",
        "scenarios/tune/tune_wide.json")
# The controllers this runner can tune: the evaluation script's name -> (the config block, the
# controller's run name, a constructor of its parameters from the block's numeric leaves by their
# dotted names (`pfbnav.eval.tuning.flatten_numeric`), whose own validation refuses a combination).
TUNABLE = {
    "regulated_pure_pursuit": ("baselines.regulated_pure_pursuit", RegulatedPurePursuit.name,
                               RPPParams),
    # B3: a flat block whose sample counts every candidate carries as floats; read as the
    # controller reads it (`DWAParams.from_block`), so a count is checked whole, never truncated.
    "dwa": ("baselines.dwa", HolonomicDWA.name, lambda **p: DWAParams.from_block(p)),
    # B6 (A16): a nested block (weights, noise_std) with integers (horizon_steps, samples,
    # iterations) and declared strings (model, scan, hard_rejection), which are not tunable.
    "field_aware_mppi": ("baselines.mppi", FieldAwareMPPI.name, MPPIParams.from_flat),
}


def _valid(params_cls, p: dict[str, float]) -> bool:
    try:
        params_cls(**p)
    except (ValueError, TypeError):  # the controller's own refusal; counted by the sampler
        return False
    return True


def _evaluate_module():
    spec = importlib.util.spec_from_file_location("evaluate", REPO_ROOT / "scripts" / "evaluate.py")
    if spec is None or spec.loader is None:
        raise SystemExit("cannot load scripts/evaluate.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def with_block(cfg: dict[str, Any], dotted: str, values: dict[str, float]) -> dict[str, Any]:
    """A copy of `cfg` with the dotted block's numeric leaves replaced by `values`, named as
    `flatten_numeric` names them (a nested leaf by a dotted name; an integer stays an integer)."""
    out = copy.deepcopy(cfg)
    node = out
    for key in dotted.split("."):
        node = node[key]
    unknown = set(values) - set(flatten_numeric(node))
    if unknown:
        raise ValueError(f"{dotted} has no {sorted(unknown)}")
    assign_flat(node, values)
    return out


def committed_problem(path: Path) -> str | None:
    """Why `path` is not a committed, unmodified file free of VERIFY, or None."""
    rel = path.resolve().relative_to(REPO_ROOT).as_posix()
    if "VERIFY" in path.read_text(encoding="utf-8"):
        return f"{rel} still says VERIFY"
    tracked = subprocess.run(["git", "ls-files", "--error-unmatch", rel], cwd=REPO_ROOT,
                             capture_output=True, text=True, check=False)
    if tracked.returncode != 0:
        return f"{rel} is not committed"
    status = subprocess.run(["git", "status", "--porcelain", "--", rel], cwd=REPO_ROOT,
                            capture_output=True, text=True, check=True)
    if status.stdout.strip():
        return f"{rel} has uncommitted changes"
    return None


def contingency_problem(endpoints: dict[str, Any]) -> str | None:
    """Ruled 2026-10-26: the contingency hypothesis H1prime is preregistered (A27) before B2's
    tuning starts. Why tuning may not start yet, or None."""
    h = endpoints.get("H1prime")
    if not h or "pivot" not in endpoints.get("confirmatory_family", {}):
        return ("preregistration/endpoints.yaml has no H1prime and pivot (A27): the contingency "
                "hypothesis is preregistered before B2's tuning starts (ruled 2026-10-26)")
    return None


def realtime_problem(gates: dict[str, Any]) -> str | None:
    """Ruled 2026-10-27: the real-time criterion (p95 compute per step <= the control period, on the
    training host) is preregistered (A28) and relocked before any tuning. Why tuning may not start
    yet, or None."""
    if "realtime_compute" not in gates:
        return ("preregistration/gates.yaml has no realtime_compute (A28): the rulings of 2026-10-27 "
                "are preregistered and relocked before any tuning")
    return None


def measurement_problem(controller: str, cfg: dict[str, Any]) -> str | None:
    """B6's samples and horizon are not tuned: they are fixed by the declared real-time measurement
    on the training host (ruled 2026-10-27), named in `baselines.mppi.compute_measurement`. Why B6
    may not be tuned yet, or None; None for every other controller."""
    if controller != "field_aware_mppi":
        return None
    run = cfg["baselines"]["mppi"].get("compute_measurement")
    if not run:
        return ("baselines.mppi.compute_measurement is not set: B6's samples and horizon are fixed "
                "by scripts/measure_realtime.py on the training host before B6 is tuned (ruled "
                "2026-10-27)")
    if not (REPO_ROOT / str(run) / "summary.json").is_file():
        return f"baselines.mppi.compute_measurement names {run}, which has no summary.json"
    return None


def g2_problem(summary_path: Path, instances: int, hostname: str) -> str | None:
    """Why `instances` simulator instances on `hostname` are not approved by this G2 summary."""
    if not summary_path.is_file():
        return f"no G2 summary at {summary_path}"
    s = json.loads(summary_path.read_text(encoding="utf-8"))
    if s.get("verdict") != "PASS":
        return f"G2 did not pass ({s.get('verdict')})"
    if (s.get("host") or {}).get("hostname") != hostname:
        return (f"G2 ran on {(s.get('host') or {}).get('hostname')!r}, this is {hostname!r}: an "
                "instance count is approved per host")
    approved = s.get("approved_instances")
    if approved is None or instances > int(approved):
        return f"{instances} instance(s) requested, G2 approved {approved}"
    if instances < 1:
        return "--instances must be at least 1"
    return None


def preflight(cfg: dict[str, Any], space_doc: dict[str, Any], sets: list[str], budget: int
              ) -> list[Path]:
    controller = space_doc.get("controller")
    if controller not in TUNABLE:
        raise SystemExit(f"cannot tune {controller!r}; tunable: {sorted(TUNABLE)}")
    if space_doc.get("config_block") != TUNABLE[controller][0]:
        raise SystemExit(f"the space names block {space_doc.get('config_block')!r}, the "
                         f"controller's is {TUNABLE[controller][0]!r}")
    if budget < 1:
        raise SystemExit("--budget must be at least 1")
    if cfg["filter"]["enabled"]:
        raise SystemExit("filter.enabled is true: a classical baseline is tuned without the PFB")
    problem = freeze_problem(cfg) or measurement_problem(controller, cfg)
    if problem:
        raise SystemExit(problem)
    paths = [(REPO_ROOT / s).resolve() for s in sets]
    if any(TEST_DIR in p.parents for p in paths):
        raise SystemExit("test layouts are never used for tuning (hard rule 2)")
    missing = [s for s, p in zip(sets, paths, strict=True) if not p.is_file()]
    if missing:
        raise SystemExit(f"scenario sets not found: {missing}")
    return paths


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--space", required=True)
    parser.add_argument("--g2", required=True, help="the G2 summary.json that approves --instances")
    parser.add_argument("--instances", type=int, required=True,
                        help="simulator instances running on this host while this runs")
    parser.add_argument("--instance", type=int, default=0,
                        help="the simulator instance this run uses: port sim.port + "
                             "sim.port_stride * i, i < --instances")
    parser.add_argument("--sets", nargs="+", default=list(SETS))
    parser.add_argument("--workers", type=int, default=4, help="processes for the route profiles")
    add_display_flag(parser)
    args = parser.parse_args()
    cfg = apply_display(load_config(args.config), args.display)
    space_path = Path(args.space)
    for path in (space_path, BUDGET):
        problem = committed_problem(path)
        if problem:
            raise SystemExit(problem)
    prereg = REPO_ROOT / "preregistration"
    problem = (g2_problem(Path(args.g2), args.instances, platform.node())
               or contingency_problem(yaml.safe_load(
                   (prereg / "endpoints.yaml").read_text(encoding="utf-8")))
               or realtime_problem(yaml.safe_load(
                   (prereg / "gates.yaml").read_text(encoding="utf-8"))))

    if not problem and not 0 <= args.instance < args.instances:
        problem = f"--instance {args.instance} is not one of the {args.instances} declared instances"
    if problem:
        raise SystemExit(problem)
    space_doc = yaml.safe_load(space_path.read_text(encoding="utf-8"))
    budget_doc = yaml.safe_load(BUDGET.read_text(encoding="utf-8"))
    budget = int(budget_doc["candidates"])
    window = float(budget_doc["selection"]["success_window"])
    paths = preflight(cfg, space_doc, args.sets, budget)
    seed_everything(args.seed)
    controller = space_doc["controller"]
    block, label, params_cls = TUNABLE[controller]
    node = cfg
    for key in block.split("."):
        node = node[key]
    defaults = flatten_numeric(node)
    space = parse_space(space_doc["space"])
    candidates, refused = sample_candidates(space, defaults, budget,
                                            np.random.default_rng(args.seed),
                                            lambda p: _valid(params_cls, p))

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
    profiles = build_profiles(
        scenarios, cfg, "cap", workers=args.workers,
        on_done=lambda i, n: print(f"  {i}/{n}", flush=True) if i % 10 == 0 or i == n else None)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "tuning", f"{label}_{stamp}", args.seed, cfg)
    (run_dir / "space.yaml").write_text(yaml.safe_dump(space_doc, sort_keys=False),
                                        encoding="utf-8")
    (run_dir / "candidates.json").write_text(json.dumps({
        "controller": controller, "budget": budget, "refused_redraws": refused,
        "g2": {"summary": Path(args.g2).as_posix(), "instances": args.instances,
               "instance": args.instance},
        "environment": environment_identity(cfg)["sha256"],
        "sets": {p.relative_to(REPO_ROOT).as_posix(): text_sha256(p) for p in paths},
        "candidates": candidates}, indent=1), encoding="utf-8")
    write_profiles(run_dir / "route_profiles.json", profiles)

    evaluate = _evaluate_module()
    dt = float(cfg["sim"]["dt"])
    client = SimClient(SimConnection.from_config(cfg, instance_id=args.instance))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))       # ONE load for every candidate
    env = NavEnv(client, cfg, scenarios, training=False, seed=args.seed)
    memory = MemoryRecord(client.connection.port)
    scores: list[dict[str, Any]] = []
    recovery: list[dict[str, Any]] = []
    episodes = 0
    try:
        for i, params in enumerate(candidates):
            out = run_dir / f"candidate_{i}"
            out.mkdir()
            log = out / "episodes.jsonl"

            def write(record: dict[str, Any], log=log) -> None:
                record["set"] = set_of[record["index"]]
                with log.open("a", encoding="utf-8") as f:
                    f.write(json.dumps(record, default=float) + "\n")

            ctrl = evaluate.make_controller(controller, with_block(cfg, block, params), None,
                                            profiles)
            ring = StepRing(int(cfg["evaluation"]["ring_steps"]), out / "ring")
            records = run_episodes(env, ctrl, range(len(scenarios)), on_episode=write, ring=ring)
            episodes += len(records)
            score = candidate_score(records, dt)
            scores.append(score)
            recovery.append(recovery_statistics(records))
            print(f"[{i + 1}/{len(candidates)}] success {score['success_rate']:.3f}, stops/100 m "
                  f"{score['stops_per_100m']}, mission time {score['mission_time_s']}, contact "
                  f"{score['contact_rate']:.3f}", flush=True)
            if recovery[-1]["suspected_defects"]:
                print(f"  SUSPECTED RECOVERY DEFECT(S) in candidate {i}: report before selecting")
    finally:
        env.close()
        client.stop()
    chosen = select(scores, success_window=window)
    boundary = on_boundary(candidates[chosen], space)
    defects = [i for i, r in enumerate(recovery) if r["suspected_defects"]]
    summary = {"controller": controller, "budget": budget, "refused_redraws": refused,
               "scores": scores, "recovery": recovery, "suspected_defects_in": defects,
               "selected": chosen, "selected_parameters": candidates[chosen],
               "selected_on_boundary": boundary,
               "rule": "among candidates within 5 pp of the best success, the fewest protective "
                       "stops per 100 m; ties by mission time (ruled 2026-10-26); a winner on a "
                       "range boundary is reported, never extended",
               "memory": memory.report(episodes)}
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=float),
                                          encoding="utf-8")
    print(json.dumps({"selected": chosen, "score": scores[chosen],
                      "parameters": candidates[chosen]}, indent=2, default=float))
    if boundary:
        print(f"ON A RANGE BOUNDARY (reported, never extended): {boundary}")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    if defects:
        print(f"STOP: suspected recovery defect(s) in candidates {defects}; report before the "
              "selection is read")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
