## A19 - 2026-10-16 - The protective field is never sized below true motion, and contains the rotation sweep; the sets re-stamped

**Decided by the supervisor on 2026-10-16** (three rulings that day, recorded in
`docs/decision_log.md` before any run under them). No test layout was built, read or used; no method
has been trained or evaluated.

**What changed.**

1. **The certified layer's field contains the footprint's full stopping sweep:**
   - translation braking along the velocity direction, and yaw braking at the yaw rate, each over
     response time plus deceleration, then offset by the margins;
   - the yaw braking assumed is `actuation.alpha_max`, which the acceleration limiter applies
     symmetrically;
   - the rotation-blind field was the supervisor's construction until now.
2. **The field is never sized below true motion:**
   - the band uses max(estimated speed, applied post-limiter command speed);
   - the rotation sweep uses the larger of |estimated yaw rate| and |applied yaw rate|;
   - the command understates motion while decelerating, the lagged estimate while accelerating.
   - An invariant test on simulator ground truth asserts it at every step. Real AMRs use an
     unlagged safety-rated speed signal; ours is privileged and lagged.
   - WP2's PFB uses the same construction and composition.
3. **At zero yaw rate, and without an applied command, the field is bit-identical** to the earlier
   one (fields stored before the change). D2, the strata and straight traversability carry over.
4. **The generator's validity rule carries the rotation sweep too.**
   - Every band-0 state of the free-orientation oracle carries the sweep of tangent tracking at
     band 0's top speed: omega = v kappa, kappa the tangent's turn over one lookahead (0.8 m),
     rounded up to 0.05 rad/s.
   - Ruled: "tangent tracking at a bend implies omega = v kappa, non-zero even at band 0".
   - A1's tangent-locked floor carries the same sweep.
5. **The scenario identity (`scenarios_sha256`) changes.** The changed modules lie inside the
   generator's import closure (A18), and the lock is renewed.

**The oracle residual check under the rotation-aware field** (ruled: it must actually run; if
residuals appear, the residual rule decides and the affected sets are rebuilt, not re-stamped):
- **Run:** `runs/analysis/ORACLE_FREE_decide_20261016T215945Z_seed0` (e7f17fe, clean, scene
  537f1bbb), all 2,271 built routes at lethal 0.10 and soft 0.03:
  - every route plannable, no quantisation failure;
  - **residual 0, start-invalid 0, goal-invalid 0 and bend-stop routes 0, in every stratum;
    free floor 0;**
  - the residual rule says re-sample, with nothing to re-sample;
  - A1's tangent-locked floor with the sweep: narrow 5.564 per 100 m (5.486 without it), medium
    0.131, wide 0.040.
- **So the sets are re-stamped, not rebuilt with new routes.** The rebuild must reproduce every
  scenario exactly, checked record by record, ignoring only the module hash
  (`scripts/compare_rebuilt_sets.py`, against the A18 build).

**In the preregistration.** Nothing in `gates.yaml` or `endpoints.yaml` names the field
construction. The lock is renewed for the new `scenarios_sha256` (`--relock A19`).
