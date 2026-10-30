"""Diagnose a training run from its logs, and optionally from a short replay of its checkpoints
(ruled 2026-10-30, before L1's pilot restarts; `pfbnav/eval/training_diag.py` says what each part
can and cannot show).

    python scripts/diag_training_run.py --config configs/base.yaml --seed 0 --run runs/train/L1_sac_shaped_seed9000000
    python scripts/diag_training_run.py --config configs/base.yaml --seed 0 --run runs/train/L1_sac_shaped_seed9000000 --replay

**Logs only** (seconds, no simulator): the entropy coefficient and the losses over training
(`sb3/progress.csv`); each episode's return split into time, stops, goal, contact and the residual
(progress - proximity - intervention), with outcomes, distance and latched steps.

**`--replay`** (a few minutes, one simulator instance): each of `--checkpoints` (default: the
first, the middle and the final model) drives `--episodes` training scenarios -- the first of the
run's own scenario order, in the run's training mode (dropout on), with the STOCHASTIC policy as in
training -- for at most `--max-steps` steps each. Per step: the observation, the sampled and the mean
action, both critics' Q of the sampled action, and every reward term. Nothing learns; the run's
directory is only read (hard rule 1).

The environment is the run's (its `config.yaml`), refused unless it is still the frozen one.
Writes `runs/diag/TRAINDIAG_<timestamp>_seed<seed>/summary.json` (and `replay_steps.npz` with
`--replay`).
"""
from __future__ import annotations

import argparse
import csv
import importlib.util
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.envs.freeze import freeze_problem
from pfbnav.envs.observation import BLOCKS, FULL_DIM, LOOKAHEAD
from pfbnav.eval.training_diag import (
    action_stats,
    column_trajectory,
    observation_ranges,
    reward_split,
    value_stats,
)
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
COLUMNS = ("train/ent_coef", "train/critic_loss", "train/actor_loss", "train/ent_coef_loss",
           "rollout/ep_rew_mean", "rollout/ep_len_mean")


def from_logs(run: Path, run_cfg: dict[str, Any]) -> dict[str, Any]:
    with (run / "sb3" / "progress.csv").open(encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    episodes = [json.loads(line) for line in
                (run / "episodes.jsonl").read_text(encoding="utf-8").splitlines() if line]
    weights = run_cfg["reward"]
    per_episode = []
    for e in episodes:
        split = reward_split(e, weights)
        per_episode.append({"episode": e["episode"], "timestep": e["timestep"],
                            "outcome": e.get("outcome"), "return": e["episode_return"],
                            "length": e["episode_length"],
                            "protective_stops": e.get("protective_stops"),
                            "stopped_steps": e.get("stopped_steps"),
                            "distance_m": e.get("distance_m"),
                            "plan_length_m": e.get("plan_length_m"), **split})
    residual = np.array([p["residual"] for p in per_episode]) if per_episode else np.zeros(0)
    return {"progress_rows": len(rows),
            "trajectories": {c: column_trajectory(rows, c) for c in COLUMNS},
            "episodes": per_episode,
            "residual": {"min": float(residual.min()) if residual.size else None,
                         "max": float(residual.max()) if residual.size else None,
                         "positive_episodes": int((residual > 0).sum())},
            "weights": {k: weights[k] for k in ("k_progress", "time_cost", "k_stop",
                                                "k_proximity", "k_intervention", "goal_bonus",
                                                "contact_penalty")}}


def _train_module():
    spec = importlib.util.spec_from_file_location("train", REPO_ROOT / "scripts" / "train.py")
    if spec is None or spec.loader is None:
        raise SystemExit("cannot load scripts/train.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def default_checkpoints(run: Path) -> list[Path]:
    saved = sorted((run / "checkpoints").glob("sac_*_steps.zip"),
                   key=lambda p: int(p.stem.split("_")[-2]))
    picks = [saved[0], saved[len(saved) // 2]] if saved else []
    if (run / "final_model.zip").is_file():
        picks.append(run / "final_model.zip")
    return list(dict.fromkeys(picks))


def replay(run_cfg: dict[str, Any], seed: int, checkpoints: list[Path], episodes: int,
           max_steps: int, instance: int, out_dir: Path) -> dict[str, Any]:
    import torch as th

    from pfbnav.control.sac import learner_class
    from pfbnav.envs.nav_env import NavEnv
    from pfbnav.sim.client import SimClient, SimConnection

    train = _train_module()
    _paths, ordered, _order = train.load_training_sets(run_cfg, seed)
    client = SimClient(SimConnection.from_config(run_cfg, instance_id=instance))
    client.connect()
    client.load_scene(str(REPO_ROOT / run_cfg["sim"]["scene"]))
    env = NavEnv(client, run_cfg, ordered[:episodes], training=True, seed=seed)
    blocks = (BLOCKS if env.observation_space.shape[0] == FULL_DIM
              else tuple(b for b in BLOCKS if b[0] != LOOKAHEAD))
    results: dict[str, Any] = {}
    arrays: dict[str, np.ndarray] = {}
    try:
        for ckpt in checkpoints:
            model = learner_class(ckpt).load(str(ckpt), device="cpu")
            obs_l, act_l, mean_l, q_l, eps = [], [], [], [], []
            for _ in range(episodes):
                obs, _info = env.reset()
                sums: dict[str, float] = {}
                steps, outcome = 0, "running"
                while steps < max_steps:
                    action, _ = model.predict(obs, deterministic=False)
                    mean, _ = model.predict(obs, deterministic=True)
                    with th.no_grad():
                        o = model.policy.obs_to_tensor(obs)[0]
                        a = th.as_tensor(model.policy.scale_action(action), dtype=th.float32
                                         ).reshape(1, -1)
                        q = th.cat(model.critic(o, a), dim=1).numpy().ravel()
                    obs_l.append(np.asarray(obs, dtype=float))
                    act_l.append(np.asarray(action, dtype=float).ravel())
                    mean_l.append(np.asarray(mean, dtype=float).ravel())
                    q_l.append(q)
                    obs, _r, term, trunc, info = env.step(action)
                    for k, v in info["reward_terms"].items():
                        sums[k] = sums.get(k, 0.0) + float(v)
                    steps += 1
                    outcome = info["outcome"]
                    if term or trunc:
                        break
                eps.append({"steps": steps, "outcome": outcome, "reward_terms": sums,
                            "progress_m": sums.get("progress", 0.0)
                            / float(run_cfg["reward"]["k_progress"])})
            name = ckpt.stem
            q_all = np.vstack(q_l)
            results[name] = {
                "checkpoint": ckpt.as_posix(), "episodes": eps,
                "q": value_stats(q_all), "q_critic_gap": value_stats(q_all[:, 0] - q_all[:, 1]),
                "actions_sampled": action_stats(np.vstack(act_l)),
                "actions_mean": action_stats(np.vstack(mean_l)),
                "observations": observation_ranges(np.vstack(obs_l), env.observation_space.low,
                                                   env.observation_space.high, blocks)}
            arrays[f"{name}_obs"] = np.vstack(obs_l)
            arrays[f"{name}_action"] = np.vstack(act_l)
            arrays[f"{name}_mean_action"] = np.vstack(mean_l)
            arrays[f"{name}_q"] = q_all
    finally:
        env.close()
        client.stop()
    np.savez_compressed(out_dir / "replay_steps.npz", **arrays)
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="for this diagnostic's own record")
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--run", required=True, help="the training run's directory")
    parser.add_argument("--replay", action="store_true")
    parser.add_argument("--checkpoints", nargs="*", default=None)
    parser.add_argument("--episodes", type=int, default=2)
    parser.add_argument("--max-steps", type=int, default=1200)
    parser.add_argument("--instance", type=int, default=0)
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    run = Path(args.run)
    if not run.is_absolute():
        run = REPO_ROOT / run
    run_cfg = load_config(run / "config.yaml")
    train_seed = int(json.loads((run / "summary.json").read_text(encoding="utf-8"))["seed"])
    checkpoints = ([Path(c) for c in args.checkpoints] if args.checkpoints
                   else default_checkpoints(run))
    if args.replay:
        problem = freeze_problem(run_cfg)
        if problem:
            raise SystemExit(f"the run's environment: {problem}")
        if not checkpoints:
            raise SystemExit("no checkpoint to replay")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"TRAINDIAG_{stamp}", args.seed, cfg)
    summary: dict[str, Any] = {"run": run.relative_to(REPO_ROOT).as_posix(),
                               "training_seed": train_seed, "logs": from_logs(run, run_cfg)}
    if args.replay:
        summary["replay"] = {"episodes_per_checkpoint": args.episodes,
                             "max_steps": args.max_steps, "policy": "stochastic, as in training",
                             "checkpoints": replay(run_cfg, train_seed, checkpoints,
                                                   args.episodes, args.max_steps, args.instance,
                                                   out_dir)}
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    logs = summary["logs"]
    ent = logs["trajectories"]["train/ent_coef"]
    print(f"episodes {len(logs['episodes'])}; ent_coef {ent.get('at', {})}")
    print(f"residual (progress - proximity) per episode: min {logs['residual']['min']}, "
          f"max {logs['residual']['max']}, positive in {logs['residual']['positive_episodes']}")
    for name, r in summary.get("replay", {}).get("checkpoints", {}).items():
        print(f"{name}: Q median {r['q'].get('median')}, negative {r['q'].get('negative_share')}; "
              f"saturated {r['actions_sampled']['saturated_share']}, "
              f"near zero {r['actions_sampled']['near_zero_share']}; progress m "
              f"{[round(e['progress_m'], 2) for e in r['episodes']]}")
    print(f"-> {out_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
