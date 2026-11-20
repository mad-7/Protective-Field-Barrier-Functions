# A32 (draft) - L5's pilot seeds after the integrity failure; A31's scope; B7's alpha sensitivity

Drafted 2026-11-03 from the supervisor's rulings on the 2026-11-03 report (points 1, 2 and 4). Applied
AFTER A31, before L5's pilot starts: `python docs/pending/apply_A32.py`, commit the three files, then
`python scripts/freeze_prereg.py --relock A32`.

---

## A32 - 2026-11-03 - L5's pilot at 9000002 and 9000003 after an integrity failure; every learned comparator trains regardless of G4; B7's alpha sensitivity declared

**Decided by the supervisor on 2026-11-03** (rulings on the report of that day), recorded in
`docs/decision_log.md`, before any L5 pilot result and before any test layout was built, read or used.
No confirmatory seed of any method has trained.

**Known at the time.** L5's first pilot (seed 9000001, `runs/train/L5_sac_pfb_unshaped_seed9000001`)
crashed in its first episodes on a PFB defect: the release check under the latch sized its band from
the raw request and raised above the top band (a learned request at 0.688 m/s), the defect A29 fixed in
the certified layer. Fixed as a bug fix (min(|request|, speed_cap)), environment re-frozen 3757aeca ->
6ad7f6af. G4's integrity rule asks for a fresh pilot seed; only 9000002 remained. B7's alpha was
frozen at 1.0, the grid's low edge, with success nearly flat across the grid (0.50-0.52).

**Amendment.**
1. **Pilot seeds:** 9000003 (pilot range) is added to `G4_learned_pilot.pilot_training_seeds`.
   L5's pilot runs at 9000002; its second seed, on a learning or checkpoint failure, is 9000003.
2. **The crashed run is kept** and recorded as an integrity failure of L5. It is not counted toward
   the pivot.
3. **The freeze mapping:** L1's seeds 9000001 and 9000002 are gated under environment 3757aeca (filter
   off, unaffected by the fix), with the G4 controls run under it. L5's gate uses controls rerun under
   6ad7f6af.
4. **A31's scope:** L1, L2 and L4 all train their confirmatory seeds regardless of G4, with their G4
   failures disclosed; only L5 triggers the pivot. An exploratory arm (A1, L5-nowp, L2-d005,
   L2-d020) that fails G4 twice is dropped and the failure reported.
5. **B7's alpha sensitivity, exploratory, declared before any test result:** B7 at alpha 1, 2 and 5
   on the test set, reporting protective stops per 100 m, success and mission time, outside the
   confirmatory family. Why: the declared rule selected alpha 1.0 because success was nearly flat
   across the grid (0.50-0.52), the 200 s horizon capping it (13 of B7's 30 tune-set timeouts were
   horizon timeouts of a slow controller), so stops decided.

**What does not change.** G4's criteria; the frozen alpha 1.0 for L3, L4, L5 and B7; H1, H2, H1N,
H1prime and their analyses; the test set.

**Edits** (made by `docs/pending/apply_A32.py`):
- `gates.yaml`, `G4_learned_pilot.pilot_training_seeds`: `[9000000, 9000001, 9000002]` ->
  `[9000000, 9000001, 9000002, 9000003]`, with a comment line on A32.
- `gates.yaml`, `on_failure.integrity`: appended "; an integrity failure is not counted toward the
  pivot (A32)".
- `gates.yaml`, `on_failure.learning_or_checkpoint`: A31's sentence extended to L2 and L4, and the
  exploratory arms' rule.
- `endpoints.yaml`, `confirmatory_family.pivot.trigger`: "L1's G4 outcome does not trigger it (A31)"
  -> "no other arm's G4 outcome triggers it (A31, A32)".
- `endpoints.yaml`, `exploratory`: a `B7_alpha_sensitivity` entry after `L2_sensitivity`.

---
