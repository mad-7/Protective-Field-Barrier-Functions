## A20 - 2026-10-18 - The protective field contains the motion reachable before braking acts; the acceleration limits, the bands and the strata co-specified with it

**Decided by the supervisor on 2026-10-18** in two rulings, both recorded in `docs/decision_log.md`
before any change:
- "option (a)": the reachable-motion field;
- "one amendment, A20": the construction, the limits, the floor, the bands and the recovery speed.

No test layout was built, read or used. No method has been trained or evaluated.

**Why.** The G4 controls of 2026-10-17 found B1 deadlocked in 45 of 51 tune episodes. Traces showed
stops ending inside the minimum margin:
- after a trigger the command still passes for the response time, and Pure Pursuit kept
  accelerating;
- the field had been sized from the speed and yaw rate at the decision, held constant;
- the traced turns rotated up to twice the sweep.

The safety property is **containment**: everything the footprint sweeps from the stop trigger to
rest lies inside the field in force at the trigger. The 2026-10-17 invariant ("never sized below
true motion") tested the wrong quantity, by the supervisor's own design.

**What changed.**

1. **The field contains the reachable motion (FULL construction).**
   - The reach time T is the response time plus one control step, 0.15 s.
   - The band is chosen from v + a_max T, capped by the velocity box and the wheel budget.
   - The rotation sweep is the interval reachable from |omega| at alpha_max within T, then braking.
   - The swerve an isotropic limiter allows is included.
   - **At rest:** the field covers the isotropic reach from the standstill speed and the yaw
     interval.
   - The at-rest field stays the infimum of the moving fields.
   - The along-heading variant was rejected: it fails containment against holonomic controllers,
     a gap in the supervisor's own first specification.
   - `pfbnav/safety/certified.py` holds the implementation; WP2's PFB adopts the same construction.
2. **The limits are a safety-design parameter, co-specified with the fields, not measured youBot
   properties.**
   - **a_max = 0.5 m/s^2.** Band coherence: a_max T must be below the narrowest band width,
     0.10 m/s.
   - **alpha_max = 1.4 rad/s^2.** a_max over the outermost radius, rounded down: no body point
     accelerates faster than a_max.
   - That 0.5 m/s^2 is achievable (the limiter binds, not the torque cap) is measured in the
     simulator.
   - **G1b is rerun in full**, since its v_max / vy_max were measured at the old limits.
3. **Bands, indexed by reachable speed: [0.10, 0.30, 0.51] m/s.**
   - The top band covers the largest reachable composed speed (0.5074 m/s). This fixes the lateral
     range above 0.40 m/s, which had no field (a defect of the G1b ruling).
   - Band 0 is the creep/departure band.
   - **The recovery speed is re-derived, 0.08 -> 0.02 m/s**, so its reachable speed stays in band 0.
     The distance and attempt caps are unchanged.
4. **The strata.**
   - **F = 0.1735 m**: the smallest per-side clearance with all three D2 gaps >= 0 (binned) under
     the full construction at the new limits (0.1435), plus the recorded 0.03 m buffer. Run:
     `runs/analysis/NARROW_FLOOR_A20_<stamp>_seed0`.
   - **narrow [0.1735, 0.2435], medium [0.2435, 0.40], wide [0.40, 0.80].**
   - F + 0.07 <= 0.30, so the ruled stop condition did not apply.

**In the preregistration** (copies of the frozen files with only these edits, to apply verbatim):
- **`endpoints.yaml`:** H2's population, "narrow (0.13-0.20 m per side)" becomes "narrow
  (0.1735-0.2435 m per side; A20)". File: `docs/pending/endpoints_A20.yaml`.
- **`scenario_sets.yaml`:** the bypass clearance's stated value, 0.13 m becomes 0.1735 m. The
  clearance is read from the strata, so only the text changes. File:
  `docs/pending/scenario_sets_A20.yaml`.
- `gates.yaml` names neither the field nor the strata, so it is unchanged. G1b's criteria stand;
  the gate is rerun under the new limits.
- **`scenarios_sha256` changes.** `certified.py`, `strata.py` and `scenarios.py` lie in the
  generator's import closure (A18). The strata change the layouts, so **every set is REBUILT**,
  not re-stamped.
- The lock is renewed (`--relock A20`).

**Checks, in the ruled order:**
1. The simulator session: the acceleration check, full G1b, the memory check (>= 200 resets, a
   declared growth bound), and the filter-off bit-identity test after the re-freeze.
2. The oracle residual check on the rebuilt sets, with the oracle mission time reported old
   against new.
3. Re-emulation of the six traced B1 episodes, then the containment test (every stop; no stop
   ending inside the minimum margin; its tolerance null-tested under rule 19).
4. The G4 controls, with the pre-declared fallback: if B1 with recovery still fails with no
   environment defect identified, the positive control becomes a privileged oracle tracker run in
   the simulator, and B1's result is recorded as a baseline finding.

**Also re-measured, because the field and the floor moved:** D2's real-scan margins
(`tests/test_d2_real_scan.py`). Their constants in `tests/test_d2_aisle_strata.py` are skipped,
with the reason, until then (hard rule 15).
