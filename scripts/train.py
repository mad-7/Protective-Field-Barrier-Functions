"""Train one SAC policy: one experiment config, one seed. The RESEARCHER runs this (hard rule 11).

    python scripts/train.py --config configs/experiments/L1_sac_shaped.yaml --seed 0

Writes `runs/train/<experiment>_seed<seed>/` -- and refuses if it exists, so a run is never
retrained over (hard rule 10):

    config.yaml, provenance.json     as every run (`create_run_dir`)
    environment.json                 the frozen environment's identity, the seed's scenario
                                     order and the training set's file hashes
    episodes.jsonl                   one line per finished episode, written as it finishes
    sb3/progress.csv, sb3/events.*   the learner's losses, as CSV (read by G4's integrity check
                                     for non-finite values) and for TensorBoard
    checkpoints/                     every `training.checkpoint_every` steps
    final_model.zip                  at `training.total_timesteps`
    summary.json                     steps, episodes, outcomes, wall time, steps per second;
                                     for L2 also the Lagrangian's settings and, per finished
                                     episode, its stop count and the multiplier then (it moves
                                     per gradient step, ruled 2026-10-27)

**The learner.** SAC from `training:` (`make_sac`); SAC-Lagrangian when the config carries a
`lagrangian:` block (L2, `pfbnav/control/sac_lagrangian.py`), with the same `training:` block.

**What it refuses, before touching the simulator:**
  * **`filter.enabled: true`**, until WP2 rules alpha. The PFB is in the step loop (WP2 step 4,
    2026-10-24), but its alpha is provisional until the tuning with B7 (step 5, ruled Q4); L5's
    pilot follows that (step 6).
  * **An environment that is not frozen, or not the frozen one** (ruled 2026-10-03): L1 trains
    while WP2 builds the PFB only on an environment whose code identity (`pfbnav/envs/freeze.py`)
    equals `env.frozen_code_sha256`.
  * **No training set.** `training.scenarios` names the built sets (equal thirds by stratum,
    ruled 2026-10-03); `OPEN`, or a set not yet built, and nothing trains.
  * **A training set that fails `verify()`**, as every set is checked before it is used.
  * **An L2 config that is not ruled:** a cost limit other than 0.1 (L2) or 0.05 / 0.2 (the
    exploratory sensitivity, ruled 2026-10-27), multiplier settings other than the declared ones, a
    stop or proximity term in the reward, or the filter on (`LagrangianSettings.from_config`).

**Order.** Training seed k visits the combined set in its own permutation, the same for every
method's seed k (`pfbnav/training/order.py`; ruled 2026-10-03). No curriculum.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
from stable_baselines3.common.logger import configure

from pfbnav.config import load_config
from pfbnav.control.sac import make_sac
from pfbnav.control.sac_lagrangian import LagrangianSettings, make_sac_lagrangian
from pfbnav.envs.freeze import environment_identity, freeze_problem
from pfbnav.envs.nav_env import NavEnv
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.training.order import order_sha256, scenario_order
from pfbnav.training.record import EpisodeLog, EpisodeRecorder
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.hashing import text_sha256
from pfbnav.utils.memory import MemoryRecord
from pfbnav.utils.runio import create_run_dir, stamp_learner_device
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.scenarios import load_set, verify

REPO_ROOT = Path(__file__).resolve().parents[1]


def preflight(cfg: dict[str, Any]) -> list[Path]:
    """Every refusal that needs no simulator. Returns the training sets' paths."""
    if not cfg.get("experiment"):
        raise SystemExit("the config names no `experiment`; train from configs/experiments/")
    if cfg["filter"]["enabled"]:
        raise SystemExit(
            "filter.enabled is true, but the PFB's alpha is not ruled yet (WP2 step 5, tuned with "
            "B7, Q4); L5's pilot is step 6.")
    problem = freeze_problem(cfg)
    if problem:
        raise SystemExit(problem)
    if cfg.get("lagrangian"):
        try:
            LagrangianSettings.from_config(cfg)
        except ValueError as exc:
            raise SystemExit(f"L2: {exc}") from exc
    scenarios = cfg["training"].get("scenarios")
    if not scenarios or scenarios == "OPEN":
        raise SystemExit(
            "training.scenarios is not set: the training set's size and stratum mix await a "
            "ruling, and no policy trains on an unruled distribution.")
    paths = [REPO_ROOT / s for s in ([scenarios] if isinstance(scenarios, str) else scenarios)]
    missing = [p.relative_to(REPO_ROOT).as_posix() for p in paths if not p.is_file()]
    if missing:
        raise SystemExit(f"the training set has not been built: {missing}")
    return paths


def make_learner(env, cfg: dict[str, Any], seed: int):
    """Plain SAC, or SAC-Lagrangian when the config carries a `lagrangian:` block (L2)."""
    if cfg.get("lagrangian"):
        return make_sac_lagrangian(env, cfg, seed)
    return make_sac(env, cfg, seed)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    add_display_flag(parser)
    args = parser.parse_args()

    # Rendering changes no physics (scripts/regress_display_off.py: bit-identical trajectories)
    # and is outside the frozen environment's identity; it costs ~7 ms a step, and the run
    # records what was in force.
    cfg = apply_display(load_config(args.config), args.display)
    seed_everything(args.seed)
    set_paths = preflight(cfg)
    scenarios: list[dict[str, Any]] = []
    for path in set_paths:
        manifest = load_set(path)
        checked = verify(manifest, cfg)
        if not checked["ok"]:
            raise SystemExit(f"{path.name} fails verification: {checked['problems'][:3]}")
        scenarios += manifest["scenarios"]
    order = scenario_order(len(scenarios), args.seed)
    ordered = [scenarios[i] for i in order]

    t = cfg["training"]
    run_dir = create_run_dir(REPO_ROOT / "runs" / "train", str(cfg["experiment"]), args.seed, cfg)
    identity = environment_identity(cfg)
    (run_dir / "environment.json").write_text(json.dumps({
        "environment": identity, "scenario_order_sha256": order_sha256(order),
        "scenarios": len(ordered),
        "training_sets": {p.relative_to(REPO_ROOT).as_posix(): text_sha256(p) for p in set_paths},
    }, indent=2), encoding="utf-8")
    scene = REPO_ROOT / cfg["sim"]["scene"]
    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(scene))
    env = NavEnv(client, cfg, ordered, training=True, seed=args.seed)
    memory = MemoryRecord(client.connection.port)     # ruled 2026-10-19: in every run record
    log = EpisodeLog(run_dir / "episodes.jsonl")
    checkpoints = CheckpointCallback(save_freq=int(t["checkpoint_every"]),
                                     save_path=str(run_dir / "checkpoints"), name_prefix="sac")
    started = time.perf_counter()
    error: str | None = None
    model = None
    try:
        model = make_learner(EpisodeRecorder(env), cfg, args.seed)
        stamp_learner_device(run_dir, str(model.device))
        model.set_logger(configure(str(run_dir / "sb3"), ["csv", "tensorboard"]))
        model.learn(total_timesteps=int(t["total_timesteps"]),
                    callback=CallbackList([log, checkpoints]))
        model.save(run_dir / "final_model.zip")
    except Exception as exc:
        # Recorded for G4's integrity check, then re-raised: a run that died says so loudly.
        error = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        wall = time.perf_counter() - started
        env.close()
        client.stop()
        summary = {"experiment": cfg["experiment"], "seed": args.seed,
                   "timesteps": int(getattr(log, "num_timesteps", 0)),
                   "episodes": log.episodes, "outcomes": log.outcomes,
                   # Ruled 2026-10-19: expected zero in a static world; any is a construction gap.
                   "at_rest_triggers": int(log.at_rest_triggers),
                   "stationary_triggers": int(log.stationary_triggers),
                   "wall_seconds": wall,
                   "steps_per_second": (log.num_timesteps / wall) if wall > 0 else None,
                   "environment_sha256": identity["sha256"],
                   "scenario_order_sha256": order_sha256(order),
                   "total_timesteps": int(t["total_timesteps"]),
                   "error": error,
                   "completed": (run_dir / "final_model.zip").is_file(),
                   "memory": memory.report(int(log.episodes))}
        if cfg.get("lagrangian") and model is not None:
            summary["lagrangian"] = {
                "cost_limit": model.cost_limit, "pid_gains": list(model.pid_gains),
                "pid_candidate": cfg["lagrangian"]["pid_candidate"],
                "multiplier_max": model.multiplier_max, "cost_window": model.cost_window,
                "final_multiplier": model.lagrange_multiplier,
                "final_integral": model.pid_integral,
                "counted_episodes": sum(e["counted"] for e in model.dual_history),
                "episodes": model.dual_history}
        (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
