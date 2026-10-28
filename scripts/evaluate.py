"""Evaluate one controller over scenario sets: each scenario once, one process. The RESEARCHER runs
this (hard rule 11: evaluation over a scenario set is long-duration).

    python scripts/evaluate.py --config configs/experiments/L1_sac_shaped.yaml --seed 0 \
        --controller sac --checkpoint runs/train/L1_sac_shaped_seed9000000/final_model.zip \
        --sets scenarios/tune/tune_narrow.json scenarios/tune/tune_medium.json \
               scenarios/tune/tune_wide.json
    python scripts/evaluate.py --config configs/base.yaml --seed 0 --controller pure_pursuit \
        --sets ...

Controllers: `sac` (a checkpoint's deterministic policy; needs `--checkpoint` and an experiment
config), `pure_pursuit` (B1, under the declared route cap and the recovery),
`regulated_pure_pursuit` (B2, likewise; ruled 2026-10-24), `dwa` (B3, holonomic DWA, likewise;
ruled 2026-10-26), `field_aware_mppi` (B6, likewise; A16, implemented 2026-10-27), `zero` (G4's negative
control) and `oracle_tracker` (G4's privileged positive control, ruled 2026-10-20). First use: G4's
controls and L1's pilot checkpoint (A15).

**Route profiles (ruled 2026-10-20)** are computed OFFLINE, before the simulator is touched, for the
controllers that need them -- the classical baselines the route cap from the MAPPED world, `oracle_tracker`
the privileged schedule from every obstacle -- over `--workers` processes, and written to
`route_profiles.json` in the run directory, so the run records the caps it was held to. Writes `runs/eval/<label>_<timestamp>_seed<seed>/`, the label being the
experiment for `sac` and the controller's name otherwise:

    config.yaml, provenance.json   as every run
    evaluated.json                 the controller, the checkpoint's SHA-256 if any, the
                                   environment identity, the sets
    episodes.jsonl                 one line per episode, written as it finishes
    summary.json                   outcome counts and shares, contact rate, the pooled stop rate --
                                   overall, per set and per stratum
    route_profiles.json            the route cap or the schedule per scenario (when used)
    ring/                          per-step dumps on stationary triggers, contacts and
                                   unrecoverable stops (ruled 2026-10-23; `pfbnav/eval/ring.py`)

**What it refuses:**
  * **Test layouts** (`scenarios/test/`): never used during development (hard rule 2); the
    confirmatory evaluation is designed with A14, not here.
  * **An environment that is not the frozen one** (`pfbnav/envs/freeze.py`).
  * **`filter.enabled`** until WP2 step 5 declares the alpha grid and its runner (Q4); the PFB is
    in the step loop from step 4.
  * **A set that fails `verify()`.**
"""
from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from pfbnav.config import load_config
from pfbnav.control.dwa import HolonomicDWA
from pfbnav.control.mppi import FieldAwareMPPI
from pfbnav.control.pure_pursuit import PurePursuit, ZeroAction
from pfbnav.control.recovery import Recovery
from pfbnav.control.regulated_pure_pursuit import RegulatedPurePursuit
from pfbnav.control.route_cap import OracleTracker, RouteCapped
from pfbnav.envs.freeze import environment_identity, freeze_problem
from pfbnav.envs.nav_env import NavEnv
from pfbnav.eval.g4 import recovery_statistics
from pfbnav.eval.ring import StepRing
from pfbnav.eval.route_profiles import build_profiles, lookup, write_profiles
from pfbnav.eval.runner import outcome_summary, run_episodes
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.hashing import text_sha256
from pfbnav.utils.memory import MemoryRecord
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.scenarios import load_set, verify

REPO_ROOT = Path(__file__).resolve().parents[1]
TEST_DIR = (REPO_ROOT / "scenarios" / "test").resolve()


CONTROLLERS = ("sac", "pure_pursuit", "regulated_pure_pursuit", "dwa", "field_aware_mppi", "zero",
               "oracle_tracker")
# The route profile each controller needs (ruled 2026-10-20): the declared cap for every classical
# baseline, the privileged schedule for the oracle tracker; none for learned arms or zero-action.
PROFILE_KIND = {"pure_pursuit": "cap", "regulated_pure_pursuit": "cap", "dwa": "cap",
                "field_aware_mppi": "cap",
                "oracle_tracker": "schedule"}


def preflight(cfg: dict[str, Any], sets: list[str], checkpoint: str | None,
              controller: str = "sac") -> list[Path]:
    """Every refusal that needs no simulator. Returns the sets' paths."""
    if controller not in CONTROLLERS:
        raise SystemExit(f"unknown controller {controller!r}; one of {CONTROLLERS}")
    if controller == "sac" and not cfg.get("experiment"):
        raise SystemExit("the config names no `experiment`; evaluate SAC from configs/experiments/")
    if cfg["filter"]["enabled"]:
        raise SystemExit("filter.enabled is true; filtered evaluation starts with WP2 step 5 (the "
                         "alpha grid with B7, Q4)")
    problem = freeze_problem(cfg)
    if problem:
        raise SystemExit(problem)
    paths = [(REPO_ROOT / s).resolve() for s in sets]
    if any(TEST_DIR in p.parents for p in paths):
        raise SystemExit("test layouts are never evaluated during development (hard rule 2)")
    missing = [s for s, p in zip(sets, paths, strict=True) if not p.is_file()]
    if missing:
        raise SystemExit(f"scenario sets not found: {missing}")
    if controller == "sac" and (not checkpoint or not Path(checkpoint).is_file()):
        raise SystemExit(f"no checkpoint at {checkpoint}")
    if controller != "sac" and checkpoint:
        raise SystemExit(f"--checkpoint is for the sac controller, not {controller}")
    return paths


def make_controller(name: str, cfg: dict[str, Any], checkpoint: str | None,
                    profiles: list[dict[str, Any]] | None = None):
    if name == "sac":
        # Imported here, not at the top: torch costs about 200 MB of committed memory, and a
        # classical evaluation never needs it (the B1 run of 2026-10-17 stalled in a host memory
        # exhaustion with its process at 811 MB).
        from pfbnav.control.sac import SACController
        return SACController(str(checkpoint), name=str(cfg["experiment"]))
    # Every classical controller carries the declared recovery, the zero-action control too
    # (ruled 2026-10-16): requesting nothing, it must never trigger it.
    if name == "oracle_tracker":
        # A privileged positive control, not a baseline: no recovery, no cap but its schedule.
        return OracleTracker.from_config(cfg, lookup(profiles or [], "schedule"))
    classical = {"pure_pursuit": PurePursuit, "regulated_pure_pursuit": RegulatedPurePursuit,
                 "dwa": HolonomicDWA, "field_aware_mppi": FieldAwareMPPI}
    if name in classical:
        # The declared route cap (ruled 2026-10-20) under the recovery, so the recovery judges the
        # capped direction.
        return Recovery.from_config(
            RouteCapped(classical[name].from_config(cfg), lookup(profiles or [], "cap")), cfg)
    return Recovery.from_config(ZeroAction.from_config(cfg), cfg)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--controller", choices=CONTROLLERS, default="sac")
    parser.add_argument("--checkpoint", default=None)
    parser.add_argument("--sets", nargs="+", required=True)
    parser.add_argument("--workers", type=int, default=4,
                        help="processes for the offline route profiles (2026-10-20)")
    add_display_flag(parser)
    args = parser.parse_args()

    # Rendering changes no physics (scripts/regress_display_off.py: bit-identical trajectories)
    # and is outside the frozen environment's identity; the run records what was in force.
    cfg = apply_display(load_config(args.config), args.display)
    seed_everything(args.seed)
    paths = preflight(cfg, args.sets, args.checkpoint, args.controller)
    scenarios, set_of = [], []
    for path in paths:
        manifest = load_set(path)
        checked = verify(manifest, cfg)
        if not checked["ok"]:
            raise SystemExit(f"{path.name} fails verification: {checked['problems'][:3]}")
        scenarios += manifest["scenarios"]
        set_of += [path.stem] * len(manifest["scenarios"])
    profiles = None
    kind = PROFILE_KIND.get(args.controller)
    if kind is not None:
        print(f"route profiles ({kind}): {len(scenarios)} scenarios, {args.workers} workers")
        profiles = build_profiles(
            scenarios, cfg, kind, workers=args.workers,
            on_done=lambda i, n: print(f"  {i}/{n}", flush=True) if i % 10 == 0 or i == n
            else None)
    controller = make_controller(args.controller, cfg, args.checkpoint, profiles)
    label = cfg["experiment"] if args.controller == "sac" else controller.name

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "eval", f"{label}_{stamp}", args.seed, cfg)
    ckpt = Path(args.checkpoint) if args.checkpoint else None
    (run_dir / "evaluated.json").write_text(json.dumps({
        "controller": controller.name,
        "checkpoint": ckpt.as_posix() if ckpt else None,
        "checkpoint_sha256": hashlib.sha256(ckpt.read_bytes()).hexdigest() if ckpt else None,
        "environment": environment_identity(cfg)["sha256"],
        "sets": {p.relative_to(REPO_ROOT).as_posix(): text_sha256(p) for p in paths},
    }, indent=2), encoding="utf-8")
    log = run_dir / "episodes.jsonl"
    if profiles is not None:
        write_profiles(run_dir / "route_profiles.json", profiles)

    def write(record: dict[str, Any]) -> None:
        record["set"] = set_of[record["index"]]
        with log.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, default=float) + "\n")
        endings = [a.get("ending") for a in record.get("recovery_attempt_log") or []]
        print(f"[{record['index'] + 1}/{len(scenarios)}] {record['set']}: {record['outcome']}, "
              f"{record.get('protective_stops', 0)} stops, {record.get('distance_m', 0.0):.1f} m"
              + (f", recovery {endings}" if endings else "")
              + (f", deadlock {record['recovery_deadlock']}"
                 if record.get("recovery_deadlock") else ""))
        if "not_released" in endings:
            print("  SUSPECTED RECOVERY DEFECT: an attempt was not released although recovery "
                  "judged its direction permitted (ruled 2026-10-16: stop and report)")

    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))
    env = NavEnv(client, cfg, scenarios, training=False, seed=args.seed)
    memory = MemoryRecord(client.connection.port)     # ruled 2026-10-19: in every run record
    try:
        # The per-step ring buffer (ruled 2026-10-23): dumps under ring/ on stationary triggers,
        # contacts and unrecoverable stops.
        ring = StepRing(int(cfg["evaluation"]["ring_steps"]), run_dir / "ring")
        records = run_episodes(env, controller, range(len(scenarios)), on_episode=write,
                               ring=ring)
    finally:
        env.close()
        client.stop()

    summary = {"overall": outcome_summary(records),
               "per_set": {s: outcome_summary([r for r in records if r["set"] == s])
                           for s in dict.fromkeys(set_of)},
               "per_stratum": {s: outcome_summary([r for r in records if r.get("stratum") == s])
                               for s in sorted({r.get("stratum") for r in records} - {None})},
               # The classical recovery's statistics and any suspected defect (ruled 2026-10-16;
               # reported in every evaluation through WP3).
               "recovery": recovery_statistics(records),
               "memory": memory.report(len(records))}
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=float),
                                          encoding="utf-8")
    o = summary["overall"]
    print(f"success {o['success_rate']:.2f}, contact {o['contact_rate']:.2f}, largest outcome "
          f"{o['largest_outcome_share']:.2f}, stops/100 m {o['stops_per_100m']}, at-rest triggers "
          f"{o['at_rest_triggers']} (stationary {o['stationary_triggers']})")
    if o["stationary_triggers"]:
        # Ruled 2026-10-19: in a static world a stationary robot with a clear field cannot be
        # triggered, so a stationary trigger is a construction gap, reported, never absorbed.
        print(f"CONSTRUCTION GAP: {o['stationary_triggers']} stop(s) triggered with the robot "
              "stationary in a static world -- report them (summary.json, episodes.jsonl)")
    if o["creep_stops_ending_unrecoverable"]:
        # Ruled 2026-10-19: a creep stop is an at-rest stop, not engineered for; but one ending as
        # an unrecoverable stop is to be stopped on and reported.
        print(f"STOP AND REPORT: {o['creep_stops_ending_unrecoverable']} episode(s) with a creep "
              "stop ended unrecoverable_stop")
    rs = summary["recovery"]
    if rs["attempts"]:
        print(f"recovery: {rs['episodes_with_recovery']}/{rs['episodes']} episodes, "
              f"{rs['attempts']} attempts {rs['endings']}, deadlocks {rs['deadlocks']}")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    if rs["suspected_defects"]:
        print(f"STOP: {len(rs['suspected_defects'])} SUSPECTED RECOVERY DEFECT(S) -- see "
              "summary.json['recovery']['suspected_defects']; report before reading any verdict")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
