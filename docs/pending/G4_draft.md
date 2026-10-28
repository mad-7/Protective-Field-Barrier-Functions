# G4 -- L1 pilot seed: a DRAFT for the supervisor (2026-10-03)

Ruled 2026-10-03: L1's pilot seed runs against G4, **criteria declared first**; then the remaining
seeds. G4 does not exist in `preregistration/gates.yaml` or anywhere else. This is a proposal; every
threshold marked PROPOSED is the supervisor's to set. Once ruled, it goes into `gates.yaml` (a
preregistration change: committed by the researcher, with an amendment and a relock).

**What G4 is for.** Not a performance claim -- H1 and H2 make those. G4 decides one thing: whether
the training pipeline works well enough, on the frozen environment, that spending four more L1 seeds
(and later the other methods) on it is sound. It must catch a broken run, not grade a good one.

## Proposed criteria, all of which must hold

**1. Integrity (pass/fail, no thresholds to set):**
- the run reached `training.total_timesteps` and wrote `final_model.zip`;
- `environment.json` records the pinned environment identity (`env.frozen_code_sha256`);
- no NaN or infinity in the logged actor, critic or entropy-coefficient losses, nor in any action;
- `episodes.jsonl` has one line per episode, each with an outcome, timesteps strictly increasing;
- no environment exception (`ScenarioUnusable` or otherwise) ended or skipped an episode.

**2. It learned (training episodes, deterministic thresholds on the log):**
- success rate over the **last 10 %** of training episodes at least **+0.20** (PROPOSED) above the
  first 10 %;
- success rate over the last 10 % at least **0.30** (PROPOSED).

**3. The checkpoint works (deterministic policy, the 60 tune scenarios):**
- success rate at least **0.50** (PROPOSED);
- ground-truth contact in at most **10 %** of episodes (PROPOSED);
- no single outcome in more than **90 %** of episodes (PROPOSED), so a policy that learned only to
  stand still -- all timeouts -- fails even if it never collides.

**4. Recorded, not gated:** environment steps per second on the host (G2 gates throughput), reset
wall clock, protective stops per 100 m, and the outcome distribution.

## What this needs that does not exist yet
- **An evaluation runner** for criterion 3: the deterministic `SACController` over the tune set,
  in one process, writing per-episode outcomes. It is evaluation over a scenario set, so the
  researcher runs it.
- The **training set** built (A13 revision of 2026-10-03), so `scripts/train.py` stops refusing.

## Open questions for the supervisor
1. The four PROPOSED thresholds, or others.
2. Whether criterion 3 should use the tune set at its rate of 0.25, as drafted, or the
   tune-unmapped set (rate 1.0).
3. What happens on a G4 failure: a fix and a fresh pilot seed (the failed run kept, never
   overwritten), or a stop and report.
