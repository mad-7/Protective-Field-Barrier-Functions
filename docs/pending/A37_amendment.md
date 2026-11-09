# A37 (draft) - the PFB on two more host planners: DWA (B9) and field-aware MPPI (B8), exploratory, held-out

Drafted 2026-11-09 on the RESEARCHER's decisions of that day. Applied before either pair runs on a test set:
`python docs/pending/apply_A37.py`, commit the two files by name, then
`python scripts/freeze_prereg.py --relock A37` and commit `preregistration/LOCK.json` alone.

---

## A37 - 2026-11-09 - HF_hosts: the PFB on B3 and B6 (B9, B8), exploratory, on the locked test set

**Decided by the researcher on 2026-11-09.** The aim of the classical comparison is the PFB's impact on a planner, so the
contrast that matters is a planner with and without it. B7 vs B2 alone meets the objection that B2 is a weak host.

**Known at the time, disclosed.** Both halves of the held-out test set have been read for B1, B2, B3, B6 and B7 (HF-c passed,
A36). These two arms were proposed AFTER that, so nothing here can be confirmatory. Their tune-set runs were also seen first
(60 scenarios, monitoring): B3 0.667 success and 130 stops, B9 0.700 and 1; B6 0.833 and 3, B8 0.767 and 0. Neither arm has
touched a test set, and neither has a tuned parameter of its own: B9 is B3's tuned parameters plus the PFB at the frozen alpha
1.0, B8 is B6's, so reading the test cannot have tuned them.

**Amendment.**
1. **HF_hosts (exploratory, outside every confirmatory family):** B9 vs B3 and B8 vs B6, protective stops per 100 m planned,
   on each half of the locked test set, with the rate ratio and its 95 % interval in HF-c's form (scenario part only, t on n - 1
   df, continuity 0.5). Success, mission time and speed are descriptive, with no inference (as A34).
2. **The pairing.** Each scenario runs B3, B9, B6, B8 in that order within one scene load, as every paired comparison must, in
   shards over the G2-approved instances (`scripts/evaluate_test.py --arm-set hosts`). So B3 and B6 run on the test set a SECOND
   time. **The sealed results of 2026-11-08 stay the cited results for B3 and B6;** the reruns serve the pairing only.
3. **Consistency check, declared now:** rerun against sealed for B3 and B6 on each half, reported as success and stop counts; a
   success difference above 0.05 is flagged and reported. Runs differ across scene loads, so identity is not expected.
4. **Low counts, declared now:** the interval is unreliable when an arm records zero or few stops (the refined A14b grid,
   2026-11-07). A comparison with min(stops) < 8 and max(stops) < 30 is flagged low-count and reported as counts with its
   interval, never as a verdict. B8 vs B6 is very likely flagged; B9 vs B3 very likely not.
5. **Real time.** Measured on the tune sets, decision p95: B8 45.0 ms, B9 17.3 ms (criterion 50 ms, gates.yaml). Each run reports
   its own measured p95; the obstacle half is harder than the tune sets, so a p95 above 50 ms there is reported as a finding and a
   limitation of that arm, and the result is not removed or rerun at a smaller budget.

**What does not change.** HF-c and its verdict; H1, H2, H1N, H1prime and their families; every tuned parameter; alpha 1.0; the
locked test sets; the sealed results.

**Edits (by `docs/pending/apply_A37.py`):** `endpoints.yaml`, `exploratory` gains `HF_hosts`; `AMENDMENTS.md`, this section.

---
