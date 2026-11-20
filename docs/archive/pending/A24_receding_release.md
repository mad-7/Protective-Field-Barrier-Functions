## A24 - 2026-10-21 - The receding release: leaving an intrusion inside the at-rest field after a creep stop

**Decided by the supervisor on 2026-10-21**, recorded in `docs/decision_log.md` before any change.
It is a blocking defect under the freeze of 2026-10-20: the pre-declared creep construction gap. No
test layout was built, read or used. No method has been trained or evaluated.

**Why.** The G4 positive control (2026-10-20, recorded as DIAGNOSTIC) ended 14 episodes as
`unrecoverable_stop` after a creep stop, all on routes an unmapped obstacle blocks. The mechanism:
- A creep stop ends a few mm closer to what stopped it, because the creep's reach is contained IN
  the at-rest field. The intrusion therefore lies inside the at-rest field.
- The whole-field restart (A22) blocks every twist, because the at-rest field is the infimum of
  every field. The deadlock is permanent.
- Creep stops are at-rest stops and cannot nest, so a release rule is unavoidable. Accepting the
  deadlock would inflate `unrecoverable_stop` and make creep departure unlearnable, contradicting
  A22.

**What changed** (`pfbnav/safety/certified.py`):
1. **The release.** A robot held at rest with its at-rest field breached is released, after
   `restart_delay`, only for a CREEP twist (at or below both standstill exits) under which, for
   every point inside the at-rest field, the distance from the footprint strictly increases.
   - The centre rule and the closest-point rule both apply, strictly.
   - Every footprint part within the at-rest field's thickness of the point must recede, so a
     rotation swinging any part toward it is blocked.
   - Unobserved bins still block.
2. **The exemption.** It covers only the points inside the at-rest field when the stop released,
   tracked bin to bin.
   - It holds only while the command stays receding and at creep speed.
   - It re-arms immediately on a non-receding command or a speed above creep.
   - It ends when those points leave the at-rest field.
   - Any other breach triggers normally.
3. **Without a creep stop the layer is bit-identical to A22's.**
4. **Known-answer tests** (`tests/test_receding_release.py`):
   - creep in, then the receding release;
   - the restart delay;
   - approach, abeam, spin and over-creep twists blocked;
   - an unobserved bin blocks;
   - a corner-swinging rotation blocked;
   - receding into a second obstacle triggers;
   - receding then approaching re-triggers on the next step;
   - bit-identity with A22 on random scenarios without a creep stop.
5. **`docs/realism_audit.md`:** the industrial counterpart is a reset plus a reduced-speed
   manoeuvring mode. Ours releases automatically: less conservative than an operator reset, and
   consistent with the modelled restart delay.

**Also, in the same rebuild:** the bypass rule references the current narrow floor (verified
2026-10-20: radius 0.2086 + 0.1987 = 0.4073 m, read from the strata). All 17 blocked tune routes
have a bypass at that floor. Only the docstring's stale "0.13 m" is corrected (`bypass.py`).

**In the preregistration:** no file other than this amendment changes.
- **`scenarios_sha256` changes**, because `certified.py` and `bypass.py` are in the scenario
  identity's closure. The sets are REBUILT; their content is unchanged.
- The lock is renewed (`--relock A24`).
- The environment is re-frozen (eighth time) with the filter-off bit-identity test.
- The horizon's schedules check is rerun on the rebuilt sets.
- The G4 controls session is repeated, and its verdict is the one read.
