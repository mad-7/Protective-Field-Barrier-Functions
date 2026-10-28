## A23 - 2026-10-20 - G4's positive control is the privileged oracle tracker; the episode horizon re-derived; the classical route cap; L5-nowp declared

**Decided by the supervisor on 2026-10-20**, recorded in `docs/decision_log.md` before any change.
No test layout was built, read or used. No method has been trained or evaluated.

**Why.** Emulated on the current tune routes, B1 (holonomic Pure Pursuit, G4's positive control
under A15) deadlocks at its first protective stop on 3/3 routes.
- The whole-field restart (A22) judges the field of B1's fixed command, which never clears.
- Both oracles pass the same routes with zero stops.
- No environment defect is identified. This is the case pre-declared on 2026-10-17.

**What changed.**

1. **G4's controls** (`gates.yaml`, `G4_learned_pilot.controls`):
   - **The positive control is the privileged oracle tracker** (`oracle_tracker_privileged`). It
     tracks the mapped route with feedback, at the speed and body yaw of the free-orientation
     oracle's schedule computed over EVERY obstacle, mapped and unmapped. It must PASS the
     checkpoint criterion.
   - **B1 with the recovery**, under the declared route cap, is evaluated in the same session and
     RECORDED as a baseline finding, never judged. A recovery attempt the emulator refuses still
     stops the gate (ruled 2026-10-16).
   - Zero-action is unchanged: it must FAIL with all timeouts.
2. **The episode horizon, `reward.max_steps`, is 4000 steps (200 s); it was 1500 (75 s).** This is a
   CORRECTION: an A5 dependent of v_max never re-derived after A20.
   - **The derivation rule:** the horizon covers the privileged oracle schedule of every traversable
     built route, x 1.3 margin. It is checked over every built set, and over the test sets once
     built (`tests/test_horizon.py`).
   - `time_cost` per step is unchanged.
   - **Knock-on effects:** fewer episodes per seed at 300k steps (if L1's pilot fails G4 on
     learning, `total_timesteps` is the first lever, not the reward), and a larger evaluation step
     budget, re-costed in G2 and in A14.
3. **The yaw-rate allowance of the oracle schedule** is the largest yaw rate keeping band 0
   admissible, rounded down to 0.05 rad/s, per route.
   - It is 0.20 rad/s on every tune route: 60/60 band-0 traversable at 0.20, 54/60 at 0.25.
   - Some training routes need 0.15 or 0.10.
4. **Every classical baseline (B1, B2, B3, B6, B7) carries the same declared route cap.**
   - At each point of the route, the speed cap is the fastest band whose field is clear of the
     MAPPED obstacles, with a yaw-rate cap at bends. It is computed offline by the oracle restricted
     to the map, then anticipated and deceleration-limited.
   - It is non-privileged. Controllers may go slower, never faster.
   - Learned arms do not get it; the methods state the asymmetry.
5. **Declared: L5-nowp, an exploratory planner-contribution analysis.**
   - L5 without the planner: the lookahead channels removed, a straight-line progress reward.
   - 5 seeds, the same seed permutations as L5, outside the Holm family, reported whatever it shows.
   - Trained after every confirmatory arm, and the first arm cut if throughput falls short.
   - v1 is the pilot study motivating it.
6. **Recorded, not fixed (the freeze of 2026-10-20):** an ideal tracker averages about 0.12 m/s under
   A20-A22.
   - The fields are more conservative than typical industrial configurations, especially in
     rotation (`docs/realism_audit.md`).
   - The oracle mission time is a reference line in the results.

**Also verified before G4, as ruled.** The last rebuild's A13 bypass rule used the current narrow
floor by reference: radius = half_width 0.2086 + narrow min_clearance 0.1987 = 0.4073 m (run
`SCENARIO_SETS_20261020T073343Z`, c748dc2). Every one of the 17 tune routes the privileged schedule
cannot traverse (each with an unmapped obstacle across the mapped route) has a bypass at that floor.
Only a docstring in `bypass.py` still says 0.13 m. That file is in the scenario closure, so the text
is left.

**In the preregistration:** only `gates.yaml` changes (the controls block), as in
`docs/pending/gates_A23.yaml`. No generator or scenario code changes, so `scenarios_sha256` is
unchanged and nothing is rebuilt. The lock is renewed (`--relock A23`).
