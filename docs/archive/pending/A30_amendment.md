
## A30 - 2026-10-30 - The learning setup restarted once: 2M steps, a curriculum, a training stall truncation; L2's limit per 100 m

**Decided by:** supervisor, 2026-10-30: option (b) of the question put after L1's first pilot seed,
then the rulings on this amendment's draft (six points), before it was applied. Recorded in
`docs/decision_log.md`, 2026-10-30. No test layout was built, read or used. No confirmatory seed of
any method has trained; no classical baseline has been tuned.

**Known at the time.** L1's pilot seed 9000000 (`runs/train/L1_sac_shaped_seed9000000`, commit
`79e98f9`) FAILED G4 at 300k steps: 75 episodes, every one at the 4,000-step horizon (51
`unrecoverable_stop`, 24 `timeout`), 0 training successes, 0/60 at the checkpoint; the controls were
sound (zero-action 0/60, privileged oracle 43/60). A23 had named `total_timesteps` as the first lever.

**Amendment.** Identical for every learned method (L1, L2, L4, L5, A1 and the exploratory arms). This
is the only change to the learning setup before the pivot rule applies.
1. **`training.total_timesteps` = 2,000,000** (was 300,000). **The seed count is capped by G2's
   budget:** G2 measured 117.3 steps/s aggregate at N = 4; the declared matrix (`n_training_runs`
   25) takes 25 x 2M / 117.3 s = 4.9 days. Six weeks (3,628,800 s) hold at most 212 runs of 2M at
   N = 4 with nothing else running, so whatever count the decision tree (Step 0) yields is capped at
   what fits the budget left after evaluation and tuning, by G2's projection recomputed for that
   count before the confirmatory seeds start.
2. **A training curriculum** (`training.curriculum`, `pfbnav/training/curriculum.py`): 0-20 % of
   training, wide and medium strata only, unmapped rate 0; 20-40 %, all strata, unmapped rate 0;
   40-100 %, the full training distribution (all strata, unmapped rate 0.25). Each seed's
   permutation applies within each stage (a stage walks the seed's order restricted to the scenarios
   it admits, from its start, cycling). "Unmapped rate 0" admits only the scenarios built with no
   unmapped obstacle (the obstacle was drawn independently of the layout, so the layouts' distribution
   is kept and no scenario is altered); "rate 0.25" is the set as built. Stages switch between
   episodes. The test distribution and every evaluation are unchanged.
3. **A training-only stall truncation:** an episode is truncated when the last 30 s (600 control
   steps) gained less than 0.05 m of route progress (arc length along the plan), as a time-limit
   truncation (bootstrapped, not terminal), outcome `stall_truncated` (`pfbnav/training/stall.py`).
   Evaluation keeps the 4,000-step horizon and its outcome rules.
4. **G4's learning criterion (criterion 2) is amended:** the improvement clause is dropped (a
   curriculum confounds a first-versus-last comparison); the criterion is success >= 0.20 over the
   last 10 % of training episodes, every one a full-distribution episode (`stage: full`).
   Criterion 3 (the checkpoint) is unchanged.
5. **The pivot counter restarts.** Seed 9000000 is recorded as G4 FAIL at 300k. The restarted pilot
   runs at seed 9000001; on its learning or checkpoint failure, one second seed, **9000002** (added to
   `G4_learned_pilot.pilot_training_seeds`); a second failure triggers the pivot to H1prime (A27).
6. **L2's limit is restated per distance:** d = **1.0 protective stops per 100 m driven** (equivalent
   to 0.1 per episode at the training set's mean route length). Over the 20-episode window the rate is
   total stops / total distance driven x 100, and the PID's error is scaled
   e = (rate - d) x (L_ref / 100), L_ref = the training set's mean planned route length (10.6716 m,
   2,001 scenarios), so the toy-validated gains keep their dynamics. **Reason:** unit consistency with
   H1 (stops per 100 m), and independence from episode length under truncation. The exploratory
   sensitivity is restated in proportion: L2-d005 at 0.5 and L2-d020 at 2.0 per 100 m. A window with
   zero distance driven makes no update (recorded).

**Memory** (approved as proposed): `scripts/check_training_memory.py` runs ~100k steps first. After
the start-up costs (baseline at step 3,000) the training process grows at most 100 MB from 10 % of
the run to its end; a 256 MiB in-process positive control must read 230-290 MB. No run of 1M steps or
more until it passes. Every training run logs memory (`memory_trend.jsonl`: this process, the
simulator and the GPU's allocated/reserved memory) and raises a MEMORY ALARM, recorded in its
summary, if the training process grows more than 1 GB beyond its settled baseline.

**Diagnostic, never cited** (optional, only if G2's instance count allows): one L1 pilot in unicycle
mode (vy = 0), seed 9000100, 2M steps (`configs/experiments/diag/L1_unicycle.yaml`, experiment
`DIAG_L1_unicycle`, no method ID). It is outside G4 (its seed is not a pilot training seed).

**Order:** apply and relock A30; the offline diagnosis of seed 9000000; the memory check; L1's pilot
at 9000001, with B2 tuning in parallel.
