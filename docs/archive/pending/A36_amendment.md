# A14b + A36 (draft) - the test size; the PFB's efficacy on RPP (HF-c) as its own confirmatory family

Drafted 2026-11-07 on the RESEARCHER's decisions of that evening (the supervisor's proposal draft of the
same day, with B8 and the learned contrasts, is superseded: those parts are NOT adopted here). Applied
before any test layout is built, read or used: `python docs/pending/apply_A36.py`, commit the four files
by name, then `python scripts/freeze_prereg.py --relock A36` and commit `preregistration/LOCK.json` alone.

---

## A14b - 2026-11-07 - The test size: 50 scenarios per cell (300 in all)

**Decided by the researcher on 2026-11-07**, before any test layout was built, read or used. K = 10 was
fixed by A35.

**Amendment.** Every test cell (stratum x condition: narrow, medium, wide x mapped_only, unmapped_static)
holds **50 scenarios**, 300 in all, built by the frozen generator from the test seed range and locked.
H1, H1prime and HF-c see the 150 unmapped_static scenarios; H2 the 100 narrow ones.

**Known at the time, disclosed.** The refined A14b grid (`runs/power/A14BR_20261107T205953Z_seed0`, decision
log 2026-11-07): H1 reaches 80 % power at no feasible size; H2 at a true +20 pp reaches it at 50 per cell
for seed logit SD <= 0.3 at K = 10 (SD 0.5 needs 75 per cell at K = 10); at a true +15 pp no size does (the
criterion, estimate >= 15 pp AND significant, halves it). **Deviation from the supervisor's rule of
2026-11-07 (ruling 3)**, recorded as the researcher's decision: the size is fixed BEFORE L5's second pilot
seed (9000003) gives the pilot variance, at the 300 cap of that rule (never the 600 extension); if the
measured variance later shows 300 to be short for H2, that is reported, never repaired by enlarging the
locked set. The H1prime/HF-c equal-rate calibration fails at 25 per cell and passes at 50 (below).

## A36 - 2026-11-07 - HF-c: the PFB removes protective stops from RPP (B7 vs B2), a second confirmatory family

**Decided by the researcher on 2026-11-07**, before any test layout was built, read or used. No
confirmatory seed of any method has trained.

**Known at the time, disclosed.** Development set (tune, 60 scenarios, 633 m planned): B2 52.8 protective
stops per 100 m planned (success 0.25), B7 = B2 + PFB 0.47 (success 0.50). This amendment was decided
AFTER those numbers were seen; the held-out test set is untouched. H1's L5-vs-B6 comparison will most
likely be inconclusive at any feasible size (A14b refined grid).

**Amendment.**
1. **HF-c** (filter efficacy, classical): *B7 (B2's tuned parameters plus the PFB at the frozen alpha
   1.0, nothing else) triggers fewer protective stops per 100 m of planned route than B2.*
   - **Family:** `filter_efficacy_family`, ONE member (HF-c), alpha 0.05, separate from
     `confirmatory_family` (H1, H2, H1N), whose Holm level is unchanged.
   - **Claim scope (as A34):** stop removal only. Success, mission time, latched share and moving speed are
     reported descriptively beside it in the same table, with no inference.
   - **Population:** test, unmapped_static, all strata pooled (150 scenarios); each arm once per scenario,
     B1, B2, B3, B6 and B7 on each scenario within ONE scene load (the pairing; determinism per load).
   - **Analysis:** the A14a rate ratio, B7 over B2, continuity 0.5 on each arm's total, stops per 100 m
     planned (A35), scan-fault stops excluded (A25); both arms run once, so the interval has the scenario
     part only, t with n - 1 df (as H1prime); two-sided 95 %. **Rejects in B7's favour when the upper end
     of the log-scale interval is below 0.** The result is reported as the rate ratio with its interval.
   - **Calibration gate, before the test is read:** at equal rates, 150 pairs, pilot route lengths, 10,000
     replications, the one-sided rejection rate on EACH side <= 0.025 + 3 MC SE (0.0297), the A14a gate,
     over rates {0.5, 1, 2, 5, 52.8} per 100 m at difficulty CV {0.5, 1.0}
     (`configs/analysis/hfc_calibration.yaml`, `scripts/calibrate_hfc.py`). If any gated cell fails: stop
     and report; HF-c is not tested. **Disclosed:** a laptop preview (not citable) passed every gated cell
     and failed at 0.25 per 100 m (CV 1.0, 1.5; ~4 expected events, where a zero-event arm's variance
     vanishes) and touched the bound at CV 1.5 (rates 5 and 52.8: 0.0299). Those cells are reported, not
     gated: 0.25 is half of B7's development rate and 1/200 of B2's.
   - **Power:** at the development rates (ratio ~0.009) power is ~1 at 150 scenarios; it is not a planning
     constraint.
2. **Not adopted:** B8 (B6 + PFB; its p95 compute, ~65 ms as a sum, exceeds the 50 ms criterion), and
   L5-vs-L1 / L5-vs-L3 as confirmatory (confirmatory seeds have not trained). L5 vs L1 and L5 vs L3 stay
   exploratory (H5 and the existing H1 comparisons).
3. **H1prime** is unchanged: exploratory (no pivot occurred), analysed and reported from the same run.
4. **The sealed classical test evaluation** (`scripts/evaluate_test.py`) runs the five classical arms on
   the unmapped_static test sets now, sharded over the G2-approved instances (shard = scenario order
   modulo the shard count); mapped_only and the learned arms run later on the same locked sets. A shard
   runs once: a rerun after a crash is recorded in the run and the decision log, never silent.

**What does not change.** H1, H2, H1N and their family; every tuned parameter; alpha 1.0; the seeds and
K (A35); the primary endpoint (A35); A34's secondaries.

**Edits (by `docs/pending/apply_A36.py`):**
- `scenario_sets.yaml`: `test: pending_A14` -> `test: {per_cell: 50}` (A14b).
- `endpoints.yaml`: `filter_efficacy_family` and `HFc` blocks appended.
- `AMENDMENTS.md`: the A14b and A36 sections appended.

---
