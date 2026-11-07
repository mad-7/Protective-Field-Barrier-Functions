"""The SEALED classical test evaluation (A36, 2026-11-07): B1, B2, B3, B6 and B7 on the unmapped_static
test sets, every arm on each scenario within ONE scene load (the pairing), sharded over the G2-approved
simulator instances. The RESEARCHER runs it (hard rule 11), one process per instance:

    python scripts/evaluate_test.py --config configs/base.yaml --seed 0 --shard 0 --shards 4 \
        --instance 0 --instances 4 --g2 runs/gates/G2_20261029T171718Z_seed9000000/summary.json

Shard i takes the scenarios whose position in (narrow, medium, wide) order is i modulo `--shards`, so
every shard holds every stratum. On each of its scenarios it runs the five arms in the fixed order B1,
B2, B3, B6, B7: the same controllers as `scripts/evaluate.py` (route cap and classical recovery), B7 being
B2's tuned parameters with the PFB at the frozen alpha switched on in the step loop for that arm alone
(the environment is built with the filter on and the PFB is detached for the other four, which is the
filter-off code path itself: `pfb_stage` returns the request untouched).

**Refuses:** an unapplied A36 or test size; a dirty tree; a config with the filter on, or a non-frozen
environment or alpha; an instance count G2 has not approved on this host; a test set that is missing,
unlocked, of the wrong size or failing `verify()`; and a SECOND attempt at a shard -- a shard is evaluated
once. After a crash, `--rerun-after-crash <the crashed run dir>` runs it again and records the crashed run
in `evaluated.json`; the rerun goes into the decision log.

**`--condition`** (2026-11-08): `unmapped_static` (the default, HF-c's population) or `mapped_only`,
the other half of the locked test set, run the same way (descriptive and per-condition reporting; HF-c
and H1 do not use it). The two conditions are separate sealed evaluations, each shard once.

Writes `runs/test/CLASSICAL_shard<i>of<n>_<stamp>_seed<seed>/` (unmapped_static; the name kept from the
first run) or `runs/test/CLASSICAL_mapped_only_shard<i>of<n>_<stamp>_seed<seed>/`: config.yaml, provenance.json,
evaluated.json, route_profiles.json, <arm>/episodes.jsonl (one line per episode, written as it ends),
<arm>/ring/, and summary.json (per arm: outcome_summary and the recovery statistics). Nothing here
analyses HF-c: `scripts/analyse_hfc.py` does, from all shards together.
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

import yaml

from pfbnav.config import load_config
from pfbnav.envs.freeze import environment_identity, freeze_problem
from pfbnav.envs.nav_env import NavEnv
from pfbnav.eval.g4 import recovery_statistics
from pfbnav.eval.ring import StepRing
from pfbnav.eval.route_profiles import build_profiles, write_profiles
from pfbnav.eval.runner import outcome_summary, run_episodes
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.filter_ruling import filter_problem
from pfbnav.utils.hashing import text_sha256
from pfbnav.utils.memory import MemoryRecord
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.scenarios import load_set, verify

REPO_ROOT = Path(__file__).resolve().parents[1]
PREREG = REPO_ROOT / "preregistration"
TEST_DIR = REPO_ROOT / "scenarios" / "test"
STRATA = ("narrow", "medium", "wide")
CONDITIONS = ("unmapped_static", "mapped_only")
# arm -> (controller, filter on)
ARMS = {"B1": ("pure_pursuit", False), "B2": ("regulated_pure_pursuit", False),
        "B3": ("dwa", False), "B6": ("field_aware_mppi", False),
        "B7": ("regulated_pure_pursuit", True)}


def _module(name: str):
    spec = importlib.util.spec_from_file_location(name, REPO_ROOT / "scripts" / f"{name}.py")
    if spec is None or spec.loader is None:
        raise SystemExit(f"cannot load scripts/{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sets(condition: str = "unmapped_static") -> list[Path]:
    if condition not in CONDITIONS:
        raise ValueError(f"unknown condition {condition!r}")
    return [TEST_DIR / f"test_{condition}_{s}.json" for s in STRATA]


def run_prefix(condition: str) -> str:
    """The run-name prefix: unmapped_static keeps the first run's `CLASSICAL_` (2026-11-07)."""
    return "CLASSICAL_" if condition == "unmapped_static" else f"CLASSICAL_{condition}_"


def shard_indices(n: int, shard: int, shards: int) -> list[int]:
    """Positions i in [0, n) with i % shards == shard."""
    if not 0 <= shard < shards:
        raise ValueError(f"shard {shard} is not one of {shards}")
    return [i for i in range(n) if i % shards == shard]


def set_filter(loop, pfb) -> None:
    """Attach `pfb` to the step loop (the filter on), or detach it with None (the filter-off path)."""
    loop.pfb = pfb
    loop.filter_enabled = pfb is not None


def preregistered_size() -> int:
    amendments = (PREREG / "AMENDMENTS.md").read_text(encoding="utf-8")
    if "## A36 - " not in amendments or "## A14b - " not in amendments:
        raise SystemExit("A14b/A36 are not applied (docs/pending/apply_A36.py, then relock)")
    test = yaml.safe_load((PREREG / "scenario_sets.yaml").read_text(encoding="utf-8")
                          )["scenario_sets"]["test"]
    if not isinstance(test, dict) or "per_cell" not in test:
        raise SystemExit("the test size is not preregistered (scenario_sets.yaml test.per_cell)")
    return int(test["per_cell"])


def prior_attempts(shard: int, shards: int, condition: str = "unmapped_static",
                   root: Path | None = None) -> list[Path]:
    root = REPO_ROOT / "runs" / "test" if root is None else root
    pattern = f"{run_prefix(condition)}shard{shard}of{shards}_*"
    return sorted(root.glob(pattern)) if root.is_dir() else []


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--shard", type=int, required=True)
    parser.add_argument("--shards", type=int, required=True)
    parser.add_argument("--instance", type=int, required=True)
    parser.add_argument("--instances", type=int, required=True)
    parser.add_argument("--g2", required=True, help="the G2 summary.json that approves --instances")
    parser.add_argument("--workers", type=int, default=4, help="processes for the route profiles")
    parser.add_argument("--condition", choices=CONDITIONS, default="unmapped_static",
                        help="which half of the locked test set (2026-11-08)")
    parser.add_argument("--rerun-after-crash", default=None,
                        help="the crashed run directory of this shard (recorded; decision log)")
    add_display_flag(parser, default="on")
    args = parser.parse_args()

    cfg = apply_display(load_config(args.config), args.display)
    if cfg["filter"]["enabled"]:
        raise SystemExit("filter.enabled is true in the config; this runner enables it for B7 alone")
    on = copy.deepcopy(cfg)
    on["filter"]["enabled"] = True
    problem = filter_problem(on) or freeze_problem(on)
    tc = _module("tune_classical")
    problem = problem or tc.g2_problem(Path(args.g2), args.instances, platform.node())
    if not problem and not 0 <= args.instance < args.instances:
        problem = f"--instance {args.instance} is not one of the {args.instances} approved instances"
    if not problem and args.shards > args.instances:
        problem = f"{args.shards} shards on {args.instances} instances"
    if problem:
        raise SystemExit(problem)
    dirty = subprocess.run(["git", "status", "--porcelain"], cwd=REPO_ROOT,
                           capture_output=True, text=True, check=True).stdout.strip()
    if dirty:      # the same test as provenance.json's `dirty`: a sealed run must record False
        raise SystemExit("the working tree is not clean (untracked files count); commit first")
    per_cell = preregistered_size()
    prior = prior_attempts(args.shard, args.shards, args.condition)
    if prior:
        named = Path(args.rerun_after_crash).resolve() if args.rerun_after_crash else None
        if named is None or named not in [p.resolve() for p in prior]:
            raise SystemExit(f"shard {args.shard} was already attempted ({[p.name for p in prior]}); "
                             "a shard is evaluated once. After a crash pass --rerun-after-crash <dir>.")
    elif args.rerun_after_crash:
        raise SystemExit("--rerun-after-crash names no earlier attempt of this shard")
    seed_everything(args.seed)

    scenarios: list[dict[str, Any]] = []
    set_of: list[str] = []
    for path in test_sets(args.condition):
        if not path.is_file():
            raise SystemExit(f"test set not found: {path.relative_to(REPO_ROOT).as_posix()}")
        manifest = load_set(path)
        if not manifest.get("locked"):
            raise SystemExit(f"{path.name} is not locked")
        if len(manifest["scenarios"]) != per_cell:
            raise SystemExit(f"{path.name} holds {len(manifest['scenarios'])}, not {per_cell}")
        checked = verify(manifest, cfg)
        if not checked["ok"]:
            raise SystemExit(f"{path.name} fails verification: {checked['problems'][:3]}")
        scenarios += manifest["scenarios"]
        set_of += [path.stem] * len(manifest["scenarios"])
    picked = shard_indices(len(scenarios), args.shard, args.shards)
    sub = [scenarios[i] for i in picked]
    sub_set = [set_of[i] for i in picked]
    print(f"shard {args.shard}/{args.shards}: {len(sub)} of {len(scenarios)} scenarios; "
          f"route profiles (cap), {args.workers} workers", flush=True)
    profiles = build_profiles(sub, cfg, "cap", workers=args.workers)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "test",
                             f"{run_prefix(args.condition)}shard{args.shard}of{args.shards}_{stamp}", args.seed, cfg)
    (run_dir / "evaluated.json").write_text(json.dumps({
        "amendments": ["A14b", "A36"], "condition": args.condition, "arms": {a: {"controller": c, "filter": f}
                                                for a, (c, f) in ARMS.items()},
        "order_within_scenario": list(ARMS), "shard": args.shard, "shards": args.shards,
        "instance": args.instance, "instances": args.instances, "positions": picked,
        "scenario_sha256": [s.get("scenario_sha256") for s in sub],
        "environment": environment_identity(on)["sha256"], "alpha": float(on["filter"]["alpha"]),
        "b2_parameters": cfg["baselines"]["regulated_pure_pursuit"],
        "sets": {p.relative_to(REPO_ROOT).as_posix(): text_sha256(p) for p in test_sets(args.condition)},
        "g2": Path(args.g2).as_posix(),
        "rerun_after_crash": Path(args.rerun_after_crash).as_posix() if args.rerun_after_crash
        else None}, indent=1, default=float), encoding="utf-8")
    write_profiles(run_dir / "route_profiles.json", profiles)

    evaluate = _module("evaluate")
    controllers = {a: evaluate.make_controller(c, on if f else cfg, None, profiles)
                   for a, (c, f) in ARMS.items()}
    rings = {}
    for a in ARMS:
        (run_dir / a).mkdir()
        rings[a] = StepRing(int(cfg["evaluation"]["ring_steps"]), run_dir / a / "ring")
    client = SimClient(SimConnection.from_config(on, instance_id=args.instance))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))         # ONE load: the pairing
    env = NavEnv(client, on, sub, training=False, seed=args.seed)
    memory = MemoryRecord(client.connection.port)
    pfb = env.loop.pfb
    if pfb is None or float(pfb.c.alpha) != float(on["filter"]["alpha"]):
        raise RuntimeError("the step loop's PFB is missing or not at the frozen alpha")
    records: dict[str, list[dict[str, Any]]] = {a: [] for a in ARMS}
    try:
        for j in range(len(sub)):
            for a, (_, filtered) in ARMS.items():
                set_filter(env.loop, pfb if filtered else None)

                def write(record: dict[str, Any], a=a, j=j) -> None:
                    record["arm"], record["set"], record["position"] = a, sub_set[j], picked[j]
                    with (run_dir / a / "episodes.jsonl").open("a", encoding="utf-8") as f:
                        f.write(json.dumps(record, default=float) + "\n")
                    print(f"[{j + 1}/{len(sub)}] {a} {sub_set[j]}: {record['outcome']}, "
                          f"{record.get('protective_stops', 0)} stops, "
                          f"{record.get('distance_m', 0.0):.1f} m", flush=True)

                records[a] += run_episodes(env, controllers[a], [j], on_episode=write, ring=rings[a])
                if (records[a][-1].get("pfb_modes") is not None) != filtered:
                    raise RuntimeError(f"{a}: the filter state of the episode is not the arm's")
    finally:
        set_filter(env.loop, pfb)
        env.close()
        client.stop()

    dt = float(cfg["sim"]["dt"])
    summary = {a: {"overall": outcome_summary(r, dt), "recovery": recovery_statistics(r)}
               for a, r in records.items()}
    summary["memory"] = memory.report(sum(len(r) for r in records.values()))
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=float),
                                          encoding="utf-8")
    defects = [a for a in ARMS if summary[a]["recovery"]["suspected_defects"]]
    for a in ARMS:
        o = summary[a]["overall"]
        print(f"{a}: success {o['success_rate']:.2f}, contact {o['contact_rate']:.2f}, "
              f"stops {o['protective_stops']}, per 100 m planned {o['stops_per_100m_planned']}")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    if defects:
        print(f"STOP: suspected recovery defect(s) in {defects}; report before any verdict is read")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
