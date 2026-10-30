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
    stopped_model.zip                if a file named STOP was created in the run directory: training
                                     ends at the next 100-step check and the model is saved
    interrupted_model.zip            on Ctrl+C, the model as it was (the run still exits as
                                     interrupted). Neither is `final_model.zip`: a run stopped
                                     early has not reached `total_timesteps`, so it is not complete
                                     and fails G4's integrity check -- a stopped pilot uses up its
                                     seed (G4 on_failure, integrity)
    memory_trend.jsonl               private memory of this process and the simulator every
                                     `training.memory_sample_every` steps (ruled 2026-10-30)
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

**Stall truncation (A30, ruled 2026-10-30).** The environment is wrapped in
`StallTruncation` (`pfbnav/training/stall.py`): an episode with less than 0.05 m of route progress
over 30 s is truncated as a time limit, outcome `stall_truncated`. Training only.

**Curriculum (A30, ruled 2026-10-30).** `Curriculum` (`pfbnav/training/curriculum.py`) picks each
episode's scenario by stage: 0-20 % wide and medium, no unmapped obstacle; 20-40 % all strata, no
unmapped obstacle; 40-100 % the full set. Each episode's record carries `curriculum_stage`.

**Diagnostics.** A config with a `diagnostic:` block (`configs/experiments/diag/`) trains the same
way, labelled, never cited, at a pilot-range seed that is not a G4 pilot seed; `unicycle: true`
zeroes the lateral command (`pfbnav/training/diagnostic.py`).

**Memory.** `memory_trend.jsonl` samples this process, the simulator and the GPU every
`training.memory_sample_every` steps; growth beyond `training.memory_alarm_mb` past the settled
baseline prints a MEMORY ALARM and is recorded in the summary (ruled 2026-10-30).

**Parallel runs.** `--instance i` connects to `sim.port + sim.port_stride * i`, so a training run
can share the host with a tuning run on another instance (each declares the host's total to G2's
approval where its script asks; `scripts/tune_classical.py --instances`).

**Order.** Training seed k visits the combined set in its own permutation, the same for every
method's seed k (`pfbnav/training/order.py`; ruled 2026-10-03). No curriculum.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import yaml
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
from stable_baselines3.common.logger import configure

from pfbnav.config import load_config
from pfbnav.control.sac import make_sac
from pfbnav.control.sac_lagrangian import LagrangianSettings, make_sac_lagrangian
from pfbnav.envs.freeze import environment_identity, freeze_problem
from pfbnav.envs.nav_env import NavEnv
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.training.curriculum import Curriculum, stages_from_config
from pfbnav.training.diagnostic import UnicycleAction, diagnostic_problem
from pfbnav.training.order import order_sha256, scenario_order
from pfbnav.training.record import STOP_FILE, EpisodeLog, EpisodeRecorder, MemoryTrend, StopFile
from pfbnav.training.stall import StallRule, StallTruncation
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


def load_training_sets(cfg: dict[str, Any], seed: int) -> tuple[list[Path], list[dict[str, Any]],
                                                                list[int]]:
    """The training sets' paths, every scenario in the seed's order, and the order."""
    set_paths = preflight(cfg)
    gates = yaml.safe_load((REPO_ROOT / "preregistration" / "gates.yaml").read_text(encoding="utf-8"))
    problem = diagnostic_problem(cfg, seed, gates)
    if problem:
        raise SystemExit(problem)
    if not cfg.get("diagnostic") and str(cfg.get("experiment", "")).startswith("DIAG_"):
        raise SystemExit("a DIAG_ experiment carries a `diagnostic:` block")
    scenarios: list[dict[str, Any]] = []
    for path in set_paths:
        manifest = load_set(path)
        checked = verify(manifest, cfg)
        if not checked["ok"]:
            raise SystemExit(f"{path.name} fails verification: {checked['problems'][:3]}")
        scenarios += manifest["scenarios"]
    lag = cfg.get("lagrangian") or {}
    if lag.get("cost_unit") == "stops_per_100m":
        # Hard rule 15: L_ref is the measurement -- the mean planned route length of THESE sets.
        measured = sum(float(s["path_length_m"]) for s in scenarios) / len(scenarios)
        if abs(measured - float(lag["reference_length_m"])) > 1e-6:
            raise SystemExit(f"lagrangian.reference_length_m {lag['reference_length_m']} is not the "
                             f"training set's mean planned route length {measured:.6f} m")
    order = scenario_order(len(scenarios), seed)
    return set_paths, [scenarios[i] for i in order], order


def run(cfg: dict[str, Any], seed: int, run_dir: Path, total_timesteps: int, sets: tuple,
        instance: int = 0) -> dict[str, Any]:
    """Train into `run_dir` (made by `create_run_dir`) for `total_timesteps` on `sets` (from
    `load_training_sets`, verified BEFORE the run directory exists, so a refused set never leaves a
    directory that blocks the seed); returns the summary, also written to `run_dir/summary.json`.
    Shared by this script and the training memory check."""
    set_paths, ordered, order = sets
    t = cfg["training"]
    identity = environment_identity(cfg)
    (run_dir / "environment.json").write_text(json.dumps({
        "environment": identity, "scenario_order_sha256": order_sha256(order),
        "scenarios": len(ordered),
        "training_sets": {p.relative_to(REPO_ROOT).as_posix(): text_sha256(p) for p in set_paths},
    }, indent=2), encoding="utf-8")
    scene = REPO_ROOT / cfg["sim"]["scene"]
    client = SimClient(SimConnection.from_config(cfg, instance_id=instance))
    client.connect()
    client.load_scene(str(scene))
    env = NavEnv(client, cfg, ordered, training=True, seed=seed)
    stall = StallTruncation(env, StallRule.from_config(cfg))
    curriculum = Curriculum(stall, stages_from_config(cfg), ordered, int(total_timesteps))
    learner_env = curriculum
    if (cfg.get("diagnostic") or {}).get("unicycle"):
        learner_env = UnicycleAction(curriculum)       # DIAGNOSTIC only (A30 ruling 6)
    memory = MemoryRecord(client.connection.port)     # ruled 2026-10-19: in every run record
    trend = MemoryTrend(run_dir / "memory_trend.jsonl", memory, int(t["memory_sample_every"]),
                        alarm_mb=float(t["memory_alarm_mb"]),
                        settled_from=int(t["memory_check"]["settled_from"]))
    log = EpisodeLog(run_dir / "episodes.jsonl")
    checkpoints = CheckpointCallback(save_freq=int(t["checkpoint_every"]),
                                     save_path=str(run_dir / "checkpoints"), name_prefix="sac")
    stop = StopFile(run_dir)
    started = time.perf_counter()
    error: str | None = None
    stopped: dict[str, Any] | None = None
    model = None
    try:
        model = make_learner(EpisodeRecorder(learner_env), cfg, seed)
        stamp_learner_device(run_dir, str(model.device))
        model.set_logger(configure(str(run_dir / "sb3"), ["csv", "tensorboard"]))
        model.learn(total_timesteps=int(total_timesteps),
                    callback=CallbackList([log, checkpoints, trend, stop]))
        if model.num_timesteps >= int(total_timesteps):
            model.save(run_dir / "final_model.zip")
        else:                                         # the STOP file ended it
            model.save(run_dir / "stopped_model.zip")
            stopped = {"how": "stop_file", "at_timestep": int(model.num_timesteps),
                       "model": "stopped_model.zip"}
            print(f"stopped by {STOP_FILE} at {model.num_timesteps} steps; saved "
                  "stopped_model.zip (not a G4-valid run)")
    except KeyboardInterrupt:
        # Ctrl+C: keep what was learned, record it, and still exit as interrupted.
        if model is not None:
            model.save(run_dir / "interrupted_model.zip")
        stopped = {"how": "keyboard_interrupt",
                   "at_timestep": int(model.num_timesteps) if model is not None else 0,
                   "model": "interrupted_model.zip" if model is not None else None}
        print(f"interrupted; saved {stopped['model']} (not a G4-valid run)")
        raise
    except Exception as exc:
        # Recorded for G4's integrity check, then re-raised: a run that died says so loudly.
        error = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        wall = time.perf_counter() - started
        env.close()
        client.stop()
        summary = {"experiment": cfg["experiment"], "seed": seed, "instance": instance,
                   "port": client.connection.port,
                   "timesteps": int(getattr(log, "num_timesteps", 0)),
                   "episodes": log.episodes, "outcomes": log.outcomes,
                   # Ruled 2026-10-19: expected zero in a static world; any is a construction gap.
                   "at_rest_triggers": int(log.at_rest_triggers),
                   "stationary_triggers": int(log.stationary_triggers),
                   "stall_truncation": {"window_s": stall.rule.window_s,
                                        "min_progress_m": stall.rule.min_progress_m,
                                        "truncations": stall.truncations},
                   "wall_seconds": wall,
                   "steps_per_second": (log.num_timesteps / wall) if wall > 0 else None,
                   "environment_sha256": identity["sha256"],
                   "scenario_order_sha256": order_sha256(order),
                   "total_timesteps": int(total_timesteps),
                   "error": error,
                   "completed": (run_dir / "final_model.zip").is_file(),
                   # Stopped early by the STOP file or Ctrl+C: the model is kept, the run is not
                   # complete (G4's integrity needs total_timesteps and final_model.zip).
                   "stopped": stopped,
                   "memory": memory.report(int(log.episodes)),
                   "memory_trend": trend.report(),
                   "curriculum": curriculum.report(),
                   "diagnostic": cfg.get("diagnostic")}
        if cfg.get("lagrangian") and model is not None:
            summary["lagrangian"] = {
                "cost_limit": model.cost_limit, "cost_unit": model.cost_unit,
                "reference_length_m": model.reference_length_m,
                "pid_gains": list(model.pid_gains),
                "pid_candidate": cfg["lagrangian"]["pid_candidate"],
                "multiplier_max": model.multiplier_max, "cost_window": model.cost_window,
                "final_multiplier": model.lagrange_multiplier,
                "final_integral": model.pid_integral,
                "counted_episodes": sum(e["counted"] for e in model.dual_history),
                "episodes": model.dual_history}
        (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--instance", type=int, default=0,
                        help="simulator instance i: port sim.port + sim.port_stride * i")
    add_display_flag(parser)
    args = parser.parse_args()
    if args.instance < 0:
        raise SystemExit("--instance must be >= 0")

    # Rendering changes no physics (scripts/regress_display_off.py: bit-identical trajectories)
    # and is outside the frozen environment's identity; it costs ~7 ms a step, and the run
    # records what was in force.
    cfg = apply_display(load_config(args.config), args.display)
    seed_everything(args.seed)
    sets = load_training_sets(cfg, args.seed)        # every refusal before the run directory
    run_dir = create_run_dir(REPO_ROOT / "runs" / "train", str(cfg["experiment"]), args.seed, cfg)
    run(cfg, args.seed, run_dir, int(cfg["training"]["total_timesteps"]), sets, args.instance)
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
