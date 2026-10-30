
## A30 - 2026-10-30 - The learning setup restarted once: 1M steps and a training-only stall truncation

**Decided by:** supervisor, 2026-10-30 (option (b) of the question put after L1's first pilot seed).
Recorded in `docs/decision_log.md`, 2026-10-30. No test layout was built, read or used. No
confirmatory seed of any method has trained; no classical baseline has been tuned.

**Known at the time.** L1's pilot seed 9000000 (`runs/train/L1_sac_shaped_seed9000000`, commit
`79e98f9`) FAILED G4 at 300k steps: 75 episodes, every one at the 4,000-step horizon (51
`unrecoverable_stop`, 24 `timeout`), 0 training successes, 0/60 at the checkpoint; the controls were
sound (zero-action 0/60, privileged oracle 43/60). A23 had named `total_timesteps` as the first
lever on a learning failure.

**Amendment.** Identical for every learned method (L1, L2, L4, L5, A1 and the exploratory arms):
1. **`training.total_timesteps` = 1,000,000** (was 300,000).
2. **A training-only stall truncation:** an episode is truncated when the last 30 s (600 control
   steps) gained less than 0.05 m of route progress, as a time-limit truncation (bootstrapped, not
   terminal), outcome `stall_truncated` (`pfbnav/training/stall.py`, `training.stall_truncation`).
   "Progress" is the route's arc length `s`, the quantity the progress reward pays for; the tracker
   projects forward only, so the window's progress is `s(t) - s(t - 600)` and back-and-forth motion
   banks none. The window slides from the episode's start (earliest truncation at step 600).
   **Evaluation is unchanged:** the 4,000-step horizon and its outcome rules; G4's checkpoint
   evaluation, the controls and every classical baseline use the environment without the wrapper.
   The wrapper is outside the frozen environment's closure (`env.frozen_code_sha256` unchanged).
3. **The pivot counter restarts.** Seed 9000000 is recorded as G4 FAIL at 300k. This is the only
   change to the learning setup before the pivot rule applies: the restarted pilot runs at seed
   9000001; on its learning or checkpoint failure, one second seed, **9000002** (added to
   `G4_learned_pilot.pilot_training_seeds`); if that fails too, the study pivots to the filter-only
   path (H1prime, A27).

**Before the restart, in order** (ruled):
- **Diagnosis of seed 9000000, offline** (`scripts/diag_training_run.py`): the entropy coefficient
  over training, Q-value sign and scale, per-episode reward components, action statistics,
  observation ranges against normalisation. The run's logs hold the entropy coefficient, the losses
  and per-episode returns; Q-values, actions and observations were never logged, so they are
  measured by replaying the run's checkpoints (first, middle, final) for two training scenarios
  each, stochastic policy, nothing learning. Any defect found is reported before the restart.
- **Memory check** (`scripts/check_training_memory.py`, 100k steps, diagnostic seed 9200000):
  criterion declared in `training.memory_check` -- after the start-up costs (baseline at step 3,000),
  the training process grows at most 100 MB from 10 % of the run to its end. A positive control
  (256 MiB allocated in-process must read as 230-290 MB) runs first. No run of 1M steps or more
  until it passes.

**Consequences, stated.**
- G2's projection scales with `total_timesteps`: 0.74 days at 300k -> ~2.5 days at 1M (N = 4),
  0.37 of the six-week budget. G2 is not rerun: throughput per step does not depend on the run's
  length.
- G4's learning criterion (first and last 10 % of training episodes) is unchanged; truncated
  episodes are episodes, and `stall_truncated` is not a success.
- **L2** (SAC-Lagrangian): its multiplier steps on the mean stops per EPISODE of the last 20 against
  d = 0.1 (A28a). A truncated episode is shorter, so the same behaviour yields fewer stops per
  episode; d is unchanged by this amendment. Flagged for a ruling.
- Training is statistically, not bit-for-bit, reproducible (ruled 2026-10-29); unchanged.
