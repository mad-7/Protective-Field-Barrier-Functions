## A27 - 2026-10-26 - The contingency hypothesis H1prime and the pivot to the filter-only path

**Decided by the supervisor on 2026-10-26** ("the path to primary results"), recorded in
`docs/decision_log.md`, with the instruction to preregister the contingency hypothesis before B2's
tuning starts. No test layout was built, read or used. No method has been trained or evaluated; no
tuning candidate has run.

**Why.** The project must produce a defensible primary result whether or not reinforcement learning
succeeds. Evidence therefore comes first from the filter on a classical controller (B7 = B2's tuned
parameters plus the PFB), which carries no learning risk, while learning starts in parallel.

**What changes** (`endpoints.yaml`).

1. **H1prime, declared:** B7 triggers fewer protective stops per 100 m than B1, B2 and B6.
   - Logic: intersection-union, as H1: every comparison must reject at alpha in B7's favour; no
     correction within H1prime.
   - Endpoint, population and scan-fault rule as H1: test set, `unmapped_static`, all clearance
     strata pooled; scan-fault stops excluded and reported alongside (A25).
   - Success rate and mission time are reported beside every comparison in the same table.
2. **Its analysis is A14a's:** the rate ratio, B7 over each comparator, continuity 0.5, the
   closed-form t interval. Every arm in H1prime runs once, so the interval has the scenario part
   only, with a t quantile on n - 1 df (the A14a formula with no seed part).
   - **This case was not in the 2026-10-03 calibration grid**, which covered a learned arm against
     a classical one and two learned arms. It must pass the same calibration gate (equal rates, at
     most nominal + 3 Monte Carlo SE on each side) before H1prime is tested. If it fails, stop and
     report.
   - H1prime's test size is decided with A14b, which must give its power alongside H1's.
3. **The pivot.** If L1's or L5's pilot fails G4 twice (both pilot seeds of `G4_learned_pilot`),
   stop and report; the study then pivots to the filter-only path.
   - Under the pivot, the confirmatory family is H1prime alone, at alpha 0.05. H1, H2 and H1N are
     then not tested confirmatorily.
   - Otherwise H1prime is exploratory, and the confirmatory family (H1, H2, conditionally H1N)
     stands unchanged.
4. **The tune-set comparison of B2 and B7** (after alpha is selected; per stratum: protective stops
   per 100 m, success rate, mission time, pfb_infeasible and fallback counts) is tune-set evidence
   for monitoring and framing only. It is never cited as a confirmatory result and does not alter
   any preregistered analysis.

**What does not change.** H1, H2, H1N, the narrow rule, A14a's analysis of them, the alpha grid and
its selection rule, the G4 criteria, and the test set.

**Prerequisites before H1prime can be tested:** B6 implemented and tuned (N = 24); the classical-pair
calibration passed; the test size from A14b.
