## A26 - 2026-10-23 - H3 restated as the PFB-(b)/(a) ablation; the H1 reporting rule; C1 restated

**Decided by the supervisor on 2026-10-23** (the WP2 rulings), recorded in `docs/decision_log.md`
before any filter code. No method has been trained or evaluated. No test layout was used.

**What changed:**
1. **H3** was "mission time decreases with field granularity (1, 3, 5 field sets, continuous PFB)".
   It is restated before any training as an **exploratory ablation: PFB-(b) banded versus PFB-(a)
   continuous** (protective stops, success, mission time).
   - The PFB adopted is (b): the band field, one QP per band, best feasible. Its safe set is the
     complement of the certified safety function's own trigger set (C1 restated).
2. **H1's reporting:** success rate and mission time are reported beside H1 for every comparison, in
   the same table. The discussion leads with the stop-throughput trade-off, and B6 is the decisive
   comparator.

**In the preregistration:** `endpoints.yaml` only (H3, and H1's `reporting`), as in
`docs/pending/endpoints_A26.yaml`. No generator or scenario code changes. The lock is renewed
(`--relock A26`); nothing is rebuilt.
