## A22 - 2026-10-19 - The restart clears the whole field the commanded twist selects; the creep regime; at-rest triggers split

**Decided by the supervisor on 2026-10-19** (restart), **confirmed 2026-10-19** (items 3, 5 and
the supersession), recorded in `docs/decision_log.md` before any change.
No test layout was built, read or used. No method has been trained or evaluated.

**Why.** Under A21's nesting, a restart judged by the two approach rules alone was released and
re-stopped at once (a release-retrigger loop). The reason: containment plus nesting leave a
departure shell of about 9-11 mm in every direction, and a departure whose own field reaches into
that shell triggers again.

**What changed.**

1. **The restart condition.** A restart is permitted only if the WHOLE field the commanded twist
   will select is clear. This supersedes A21 item 2 as its first test.
   - A twist at or below both standstill exit thresholds (0.02 m/s, 0.0592 rad/s) keeps the
     at-rest field. This is the CREEP regime that leaves the shell.
   - Anything faster selects the band of its reachable speed, along its translation. A spin on the
     spot uses the field's present heading.
   - A21's two approach rules (the centre rule and the closest-point rigid-body rule) are kept
     after the whole-field test. They cannot then contradict it.
   - A spin swinging a corner away is permitted at a creep yaw rate. At a normal rate it is blocked
     when the intrusion lies inside the spin's own field. This explicitly supersedes the 2026-09-28
     ruling "spinning away is permitted". A normal-rate spin selects a moving field that contains
     the departure shell, so an intrusion inside it would re-trigger at once; at creep rate the
     at-rest field governs, and nesting keeps it clear. The original ruling's intent (never drive
     toward an intrusion, always leave a way out) is kept by the creep regime.
   - It is stricter than A21's rule wherever a departure would re-trigger. Departures that A21
     released and then re-stopped are now held.
2. **The creep, for each kind of method** (the asymmetry is stated in the write-up):
   - **Learned methods:** creep departure is a learnable skill. `stop_active` stays set while a
     restart is blocked, and the creep speed is inside the action range.
   - **Classical baselines:** the creep is scripted through the declared recovery, which now
     judges candidate twists by the field they select.
3. **The recovery's translation speed is 0.018 m/s** (was 0.02).
   - 0.02 sits exactly on the exit threshold, and at 45 deg the command's magnitude rounds above
     it. 0.018 keeps a 10 % margin.
   - Its reachable speed is 0.093 m/s, in band 0.
   - Rotational equivalent, derived as the yaw thresholds are (over the outermost radius
     0.3374 m): 0.0533 rad/s. Recorded only; the recovery does not turn.
   - An attempt to the full 0.30 m takes about 17 s, so fewer than five attempts fit in a 75 s
     episode and the cap is effectively lower. This is accepted, since recovery is rare under the
     corrected fields. G4 reports every attempt's time, including attempts the episode's end cuts
     short.
4. **The at-rest field is sized from the exit thresholds**, translational and rotational. Creep is
   therefore contained.
   - Verified by test, with a negative control: a field sized from zero does not contain it.
5. **At-rest triggers are counted in two classes** in every static-world run (G4 controls, pilots,
   evaluation):
   - **stationary** (speed and yaw rate at or below the ENTER thresholds): the construction gap,
     expected zero, and any occurrence is reported;
   - **creep** (at rest but above the enter thresholds): at-rest stops not covered by nesting. A
     creep stop that ends the episode as `unrecoverable_stop` means stop and report.
   - Both classes are protective stops and count in H1 like any other stop.

**In the preregistration:** no file other than this amendment changes.
- **`scenarios_sha256` changes**, because `certified.py` is in the scenario identity's import
  closure. The lock is renewed (`--relock A22`).
- The layout sets are rebuilt once, after this relock. The generator's behaviour does not depend
  on the restart condition, so they are built under A21's strata exactly as A21 specified.
