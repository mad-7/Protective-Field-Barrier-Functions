# A34 (draft) - H1prime's claim and two declared secondary analyses (text only, no design change)

Drafted 2026-11-06 from the supervisor's ruling on the B2 speed check. Applied BEFORE any test run:
`python docs/pending/apply_A34.py`, commit the two files, then
`python scripts/freeze_prereg.py --relock A34`.

---

## A34 - 2026-11-06 - H1prime claims stop removal only; two secondary analyses declared before the test run

**Decided by the supervisor on 2026-11-06** (ruling on the B2 speed check), recorded in
`docs/decision_log.md`. No test layout was built, read or used. Text only: no endpoint, comparator,
analysis or criterion changes.

**Known at the time.** On B7's 13 tune-set horizon timeouts, B7 and B2 drive at the same speed when
not latched (0.057 m/s pooled each); B7's slowness is B2's tuned regulation, not the filter. B2 spent
27 % of those episodes latched (69 stops), B7 none. A tune-set, post-hoc observation: exploratory.

**Amendment.**
1. **H1prime's claim** is that the filter removes protective stops from a classical planner. It is
   NOT a throughput or success claim. Primary endpoint unchanged: protective stops per 100 m, B7 against
   B1, B2 and B6, intersection-union. Success rate and mission time are reported DESCRIPTIVELY beside
   every comparison, with no inference drawn on them.
2. **Secondary analyses, declared now** (outside the confirmatory family):
   a. **B7 against B2, the paired within-planner contrast:** protective stops per 100 m, the latched
      share of steps, and the moving speed (mean speed over the steps not latched).
   b. **The denominator of the stops endpoint:** stops per 100 m of distance actually driven (the
      primary's own denominator, recovery motion included) and stops per 100 m on completed routes only,
      beside the per-planned-length version A14 already declares.
3. **Recorded with it, as they stand (no change):** the primary divides each arm's pooled protective
   stops (scan-fault stops excluded) by its pooled DRIVEN distance, recovery motion included; every
   episode is pooled, so timeouts and unrecoverable stops contribute their stops and distance and are
   NOT censored (`pfbnav/eval/runner.py`, `outcome_summary`).

**What does not change.** H1prime's statement, comparators, logic, endpoint, analysis, success
criterion and population; H1, H2, H1N; the alpha 1.0 freeze; every tuned parameter.

**Edits** (made by `docs/pending/apply_A34.py`), all in `endpoints.yaml`, block `H1prime`:
- a `claim:` line after `statement:`;
- `reporting:` reworded to "descriptively ... no inference on them (A34)";
- a `secondary:` list after `reporting:` with 2a and 2b.

---
