# A31 (draft) - the pivot rule clarified: L5 alone triggers it; L1 trains as a comparator regardless

Drafted 2026-11-03 from the supervisor's rulings on the 2026-11-03 report (point 3), to be applied
BEFORE any L5 pilot result exists. The researcher appends the section below to
`preregistration/AMENDMENTS.md`, makes the two edits listed under "Edits", commits, and relocks:
`python scripts/freeze_prereg.py --relock A31`.

---

## A31 - 2026-11-03 - The pivot is triggered by L5 alone; a comparator arm's G4 failures are disclosed, not disqualifying

**Decided by the supervisor on 2026-11-03** (rulings on the report of that day, point 3), recorded in
`docs/decision_log.md`, before any L5 pilot result existed. No test layout was built, read or used. No
confirmatory seed of any method has trained.

**Known at the time.** L1's pilot under A30 (seed 9000001, 2M steps) failed G4's learning criterion:
last-10 % success 0.0055 against 0.20 (`runs/train/L1_sac_shaped_seed9000001`). L5's pilot had not
started. A27 item 3 read "if L1's or L5's pilot fails G4 twice ... the study then pivots", and G4's
`on_failure` read "if it fails [the second pilot seed], stop and report" for every learned method.
Read together they let the comparator L1 decide whether the proposed method L5 is tested.

**Amendment.**
1. **The pivot to H1prime is triggered by L5 failing G4 twice** (both of L5's pilot seeds of
   `G4_learned_pilot`, 9000001 then 9000002, under A30's configuration). L1's G4 outcome does not
   trigger it. Under the pivot, A27's consequences stand unchanged (H1prime alone, at alpha 0.05).
2. **If L1 fails G4 twice while L5 passes, L1 still trains** its confirmatory seeds unconditionally,
   as a comparator arm, with its G4 failures disclosed beside every result that uses it. H1 keeps its
   seven comparators (B1, B2, B3, B6, L1, L2, L3).
3. **Recorded with it:** a non-learning L1 is an easy comparator. A win over it carries little
   weight; L2 and B6 carry H1's weight, and the discussion says so.

**What does not change.** G4's criteria, its pilot seeds, the one-second-seed rule, A30's learning
setup, H1's statement and logic, H1prime, and the test set.

**Edits.**
- `preregistration/endpoints.yaml`, `confirmatory_family.pivot.trigger`:
  was `L1's or L5's pilot fails G4 twice (both pilot seeds of G4_learned_pilot)`;
  becomes `L5's pilot fails G4 twice (both pilot seeds of G4_learned_pilot); L1's G4 outcome does not trigger it (A31)`.
- `preregistration/gates.yaml`, `G4_learned_pilot.on_failure.learning_or_checkpoint`: append
  ` A31 (2026-11-03): for L5 a second failure triggers the pivot (endpoints.yaml); for L1 a second
  failure is disclosed and L1 still trains its confirmatory seeds unconditionally as a comparator.`

---

## Open, for the supervisor (not in the amendment until ruled)

- **L2 and L4 are not named.** Both are learned comparators with their own G4 pilots. Does point 2
  apply to them as to L1 (train regardless, failures disclosed), or does a second G4 failure of
  either stop that arm? The draft says nothing about them, so as written G4's "stop and report" still
  applies to L2 and L4.
