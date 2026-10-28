## A28a - 2026-10-27 - A28 finalised: L2's multiplier on measured stops, B3's floors, the cut order, B6's update, the compute stop rule

**Decided by the supervisor on 2026-10-27** (rulings on A28's five open points), recorded in
`docs/decision_log.md`. A28 had already been applied and relocked, so its finalisation is this dated
revision. No test layout was built, read or used. No method has been trained, tuned or evaluated; no
candidate has run.

**What changes.**

1. **L2's multiplier** (`endpoints.yaml`, `declared.L2_multiplier`):
   - lambda is updated from the EMPIRICAL mean of protective stops per episode over the last 20
     completed episodes, compared directly with d = 0.1.
   - eta = 3e-4 per gradient step; lambda_0 = 0; lambda is projected onto [0, lambda_max = 100].
   - lambda is held until 20 episodes have completed.
   - The cost critic still drives the policy gradient.
   - This supersedes A28's "from the cost critic's batch estimate". **Reason:** units consistency
     with the declared d (stops per episode). No (1 - gamma^T) conversion.
2. **B3's candidate 0** (`declared.B3_candidate_0`; ranges in `configs/tuning/b3_dwa.yaml`): the two
   floors are extended to include DWB's own values:
   - window_time 0.05-0.40 s;
   - w_clearance 0.02-5 (log).

   This corrects the declaration before any run; the never-extend rule applies only after tuning.
3. **The cut order** (`gates.yaml`): [L2-d005, L2-d020, L5-nowp, PFB-(a), stop_active, A1] is
   confirmed. New `never_cut`: L1, L2, L4, L5 and all classical comparators B1, B2, B3, B6, B7.
4. **B6's update** (`declared.B6_update`): standard MPPI is confirmed -- the weights average the
   sampled commands, and the limiter is applied inside the rollouts.
   - **Why the earlier form crawled:** averaging the limited sequences fed the limiter's asymmetry
     back into the plan every step. Braking can remove a_decel_max dt = 0.05 m/s per step;
     accelerating can add only a_max dt = 0.025 m/s. B6 settled near 0.08 m/s on an open route.
5. **Compute ties** (`gates.yaml`, `realtime_compute.B6_rule`): the longer horizon is confirmed.
   **Declared outcome:** if even 256 x 15 exceeds 50 ms p95 on the training host, stop and report.
   The criterion is not relaxed.

**What does not change.** Everything else in A28; H1, H2, H1N, H1prime, the pivot, A14a's analysis,
the G4 criteria and the test set.
