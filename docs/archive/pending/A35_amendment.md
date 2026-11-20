# A35 (draft) - primary per PLANNED route length; K = 10 and the confirmatory seeds; L5-v2; no improvement round

Drafted 2026-11-07 from the supervisor's two messages of rulings on the 2026-11-07 report. Applied
AFTER A34, before any test data exists: `python docs/pending/apply_A35.py`, commit the two files, then
`python scripts/freeze_prereg.py --relock A35`.

---

## A35 - 2026-11-07 - H1's primary per planned route length; K = 10 and confirmatory seeds 9100001-9100010; L5-v2 exploratory; no improvement round

**Decided by the supervisor on 2026-11-07** (two messages of rulings on the report of that day),
recorded in `docs/decision_log.md`, before any test layout was built, read or used. No confirmatory
seed of any method has trained; no test set exists.

**Known at the time (tune set, 60 scenarios, 633 m of planned route).** L5's pilot checkpoint drove
1,012 m (1.60x its planned length, every stratum 1.45-1.73x): 0.59 protective stops per 100 m driven,
0.95 per 100 m planned. B6: 0.44 driven, 0.47 planned (1.09x). B7: 0.59 driven, 0.47 planned (0.80x).
The ranking of B7 against B6 flips with the denominator; the driven-distance rate rewards a controller
for driving extra.

**Amendment.**
1. **The primary endpoint of H1 and H1prime is protective stops per 100 m of PLANNED route length**:
   each arm's pooled protective stops (scan-fault stops excluded, A25) over its pooled planned route
   length. The rate ratio and its interval (A14a) are unchanged in form; only the denominator moves.
   Conservative for L5 on the pilot (0.59 -> 0.95). Stops per 100 m DRIVEN and on completed routes only
   stay SECONDARY (A34). Every episode is pooled; none is censored.
2. **Seeds.** K = 10 confirmatory seeds per learned arm, fixed before the first confirmatory run and
   never changed: **9100001-9100010**; seed k fixes the training-set permutation shared by every arm.
   Pilot TRAINING seeds come from **9000000-9000099**. The confirmatory block 9100000-9100999 is carved
   out of the layout pilot range (`layout_sets.seed_ranges.pilot`): no diagnostic, pilot or layout may
   draw from it, and its runs appear in reported results.
3. **No learning-setup improvement round and no method change** (filter, observation, reward, emulator)
   before confirmatory training. A DEFECT is fixed as a bug fix under an amendment, identically for
   L1, L2, L4 and L5, and restarts the pilots.
4. **L5-v2, an exploratory arm:** an improved learning setup with at most ONE shared change,
   pilot-range seeds, tune set only; not in the confirmatory family; its configuration and seeds are
   declared (hash) before the confirmatory test evaluation runs. At the HEAD of the cut order.

**What does not change.** H1's and H1prime's statements, comparators, logic and analysis form; H2;
H1N; G4; the alpha freeze; every tuned parameter; the tuning selections already made (the tuning rule
ranked candidates by stops per 100 m driven, as declared at the time; they are not re-selected).

**Recorded with it (no test set exists yet).** The test set's size is deliberately absent from
`scenario_sets.yaml` (`test: pending_A14`). It will be built on the training host after A14b, with the
same generator and validity rules as every set, from the test seed range 3000000-3999999, never
modifying a built test scenario.

**Edits** (made by `docs/pending/apply_A35.py`):
- `endpoints.yaml`: H1's and H1prime's `endpoint:` -> `protective_stops_per_100m_planned`; the
  `analysis.H1_estimand` denominator -> planned route length; H1prime's A34 secondary names driven
  distance as secondary since A35.
- `gates.yaml`: `G2_compute_budget.reduction_order` gains `L5-v2` at its head; `G4_learned_pilot` gains
  `pilot_training_range` and `confirmatory_training_seeds`; `layout_sets.seed_ranges.pilot` notes the
  carve-out.

---
