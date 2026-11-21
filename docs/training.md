# Training the learned planner

One policy = one experiment config + one seed, run by `scripts/train.py` (the researcher runs it; training is a
long-duration operation). Everything configurable is a YAML key; nothing is hard-coded in the script.

## The arms

| Arm | Config | What differs from `configs/base.yaml` |
|---|---|---|
| L1 | `configs/experiments/L1_sac_shaped.yaml` | proximity shaping `k_proximity` 0.1011, no filter |
| L2 | `L2_sac_lagrangian*.yaml` | SAC-Lagrangian with a stop budget (`lagrangian:` block), no filter |
| L4 | `L4_sac_pfb_shaped.yaml` | PFB in the loop + shaping |
| L5 | `L5_sac_pfb_unshaped.yaml` | PFB in the loop, no shaping (the proposed method) |
| L3 | (an evaluation, not a training run) | L1's policy wrapped by the PFB afterwards |

## Where each setting lives

| Setting | Key | Notes |
|---|---|---|
| SAC hyperparameters | `training.*` (`learning_rate`, `buffer_size`, `batch_size`, `gamma`, `tau`, `net_arch`, `ent_coef`, `device`) | shared by every learned arm |
| Budget | `training.total_timesteps` | 2,000,000 |
| Curriculum | `training.curriculum` | three stages by fraction of the run |
| Stall cut-off | `training.stall_truncation` | training only |
| Reward | `reward.*` | `k_progress`, `time_cost`, `k_stop`, `k_intervention`, `k_proximity`, `goal_bonus`, `contact_penalty` |
| Observation | `env.observation.*`, `scanners.policy_bins`, `scanners.range_max` | 82 channels (`pfbnav/envs/observation.py`) |
| The PFB | `filter.*` | `filter.enabled` is the switch; alpha is frozen |
| Training scenarios | `training.scenarios` | `scenarios/train/*.json` |

## Command line

```powershell
python scripts/train.py --config configs/experiments/L1_sac_shaped.yaml --seed <n> [--instance i] [--display on|off] [--set KEY=VALUE ...]
```

- `--set KEY=VALUE` (repeatable) overrides one existing key for the run. It refuses a missing key (typo) or a
  value of another kind, warns on a ruled section (reward, observation, filter, certified layer, actuation),
  and is recorded in the run's `config.yaml` under `cli_overrides`. `scripts/evaluate.py` takes the same flag.
- `--draw-path` draws the planned route (blue), start (green), goal (yellow) and the robot's track (red) in the
  scene while it trains, so you can see whether the policy follows the route. It needs `--display on`, adds a few
  remote calls per step, and changes nothing the policy learns: use it for a short watched run, not a full one.
  `scripts/evaluate.py` has the same flag.
- `--instance i` uses the simulator on port `sim.port + sim.port_stride * i`, so several runs can share a host.
- A run directory is `runs/train/<experiment>_seed<seed>/` and is never overwritten (hard rule 10). A short
  diagnostic run therefore needs its own label: `--set experiment=DIAG_<name>`.

## Seeds

- G4 pilot seeds: 9000000-9000003. Other pilot-range training seeds: 9000004-9000099 (exploratory, never in H1
  or H2). Confirmatory seeds: 9100001-9100010; once trained they stay in the analysis whatever the outcome
  (hard rule 17), so any change to the setup comes first.
- A diagnostic is labelled `DIAG_` and never cited.

## What a run writes

`config.yaml`, `provenance.json` (commit, host, GPU), `environment.json`, `episodes.jsonl` (one line per episode,
with its outcome, stops and, for a filtered arm, how often the PFB changed the command), `sb3/progress.csv` and
TensorBoard events (`tensorboard --logdir runs/train`), `checkpoints/` every 25,000 steps, `memory_trend.jsonl`,
`summary.json`, and `final_model.zip` at the end. Create a file named `STOP` in the run directory to end a run
cleanly at the next check; the model is saved as `stopped_model.zip`, which is not a complete run.

## TensorBoard

```powershell
python -m tensorboard.main --logdir runs/train
```

(`python -m` avoids the blocked `tensorboard.exe` launcher on some Windows hosts.) Besides SB3's losses and mean
return, `pfbnav/training/tb_stats.py` writes rolling statistics of the last `training.episode_stats_window`
episodes: `episodes/success_rate`, `episodes/outcome_share/<outcome>`, `episodes/stops_per_100m_driven` and
`..._planned`, `episodes/mean_return`, `episodes/mean_length`, `episodes/pfb_modified_share` and
`episodes/pfb_infeasible_share` (filtered arms), and `curriculum/stage`. It only reads the episode records;
`episodes.jsonl` remains the full record.

## Evaluating a trained policy

```powershell
python scripts/evaluate.py --config configs/experiments/L1_sac_shaped.yaml --seed 0 --controller sac --checkpoint runs/train/<run>/final_model.zip --sets scenarios/validation/validation_mapped_only_narrow.json
```

Evaluate on `scenarios/validation/` (never trained on), not on the training sets.
