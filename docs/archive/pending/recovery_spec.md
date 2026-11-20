# Classical recovery behaviour -- draft specification for the supervisor (2026-10-16)

**Status:** a draft for a ruling. Nothing is implemented. It turns the 2026-10-13 ruling (3b) into
declared numbers and mechanics, and raises one defect found while drafting it (section 0).

**What the 2026-10-13 ruling already fixes:**
- one recovery behaviour, applied identically to every classical baseline (now B1, B2, B3, B6, B7;
  A16);
- trigger: stopped, with the restart blocked beyond a declared wait after the restart delay;
- motion: at most 0.10 m/s, in a restart-permitted direction maximising clearance
  (v . p <= 0 for every breaching point), for at most a declared distance, then resume;
- attempts per episode are capped;
- stops still count in H1; recoveries are counted and reported;
- a deadlock that recovery cannot clear is an `unrecoverable_stop`;
- learned policies get no scripted recovery, and the methods state the asymmetry.

The numbers marked **[proposed]** are mine and need the ruling.

---

## 0. A defect found while drafting: one kind of deadlock is labelled `timeout`

**Ruling 4 (2026-09-27)** makes an episode that runs out of steps while a stop is latched and
the restart blocked an `unrecoverable_stop`. The 2026-09-28 entry adds that a stop held by an
intrusion inside the minimum margin "ends as `unrecoverable_stop`".

**The code does not do that for the second case:**
- `CertifiedLayer.step` computes `restart_blocked` only while the robot's own (at-rest) field is
  clear.
- With an intrusion inside that field, the latch holds and nothing can release it, but
  `restart_blocked` is False.
- `episode_outcome` then labels the episode `timeout`.

**It has happened.** Both "timeouts" among the 16 episodes traced on 2026-10-13
(`TRACE_20261013T215044Z` trace 2, `TRACE_20261013T221601Z` trace 13) ended at rest with the stop
latched, the at-rest field breached, and `restart_blocked` False. Those were the void B1 control's
traces, so no reported result is affected.

**Why it matters:** A12's narrow-stratum rule counts `unrecoverable_stop` proportions, and
mislabelling them as timeouts would understate the count the rule exists to catch.

**Proposed fix**, applying to every method alike: an episode is `unrecoverable_stop` if at the
step limit the stop is latched and EITHER the restart is blocked OR the robot is at rest with its
own field breached:
`stuck = stop_active and (restart_blocked or (breached and at_rest))`
- `at_rest` keeps a stop that has only just fired while still moving from being counted as
  stuck.
- It touches `pfbnav/envs/reward.py`, so the environment is re-frozen through the filter-off
  bit-identity test, with a known-answer test for the new case.
- Needed whether or not recovery is approved.

---

## 1. Where recovery sits

**A wrapper around the classical controller**, implemented once (`pfbnav/control/recovery.py`)
and applied to B1, B2, B3, B6 and B7 alike. The order within a step is:

> controller -> recovery wrapper -> (B7 only: PFB filter) -> certified layer

- Recovery commands pass through B7's filter and through the certified layer exactly like any
  other command. The emulator is not changed: it still knows nothing about recovery, as ruled
  ("no recovery mode in the emulator").
- **What recovery reads:**
  - the stop flag in the observation;
  - the estimated velocity in the observation;
  - the **120-bin scanner stream** the certified layer evaluates. A16 makes that the classical
    baselines' shared input.
  - Today the environment does not hand that stream to a controller. It would add it to each
    step's `info`, and the runner would pass it to controllers that read it.
  - That is an information-only change to the environment, re-frozen through the bit-identity
    test. B2, B3 and B6 need the stream anyway.
- **Whether a direction is restart-permitted** is judged by the same pure function the certified
  layer uses: `_restart_allowed`, moved out of the class unchanged and covered by the same tests.
  Recovery therefore never asks for a direction the emulator would refuse, and never refuses one
  it would allow.
- **No privileged state.** Recovery counts stopped steps itself (step 0.05 s). It does not read
  the emulator's internal timers or the pose.

## 2. Trigger

A recovery attempt starts when ALL of these hold:
1. the stop flag has been set continuously for **`restart_delay` + `wait`** = 2.0 s + **1.0 s
   [proposed]** = 3.0 s (60 steps);
2. the base controller's own requested direction is **not** restart-permitted on the current
   scan. Recovery never overrides a command the emulator would let through: if the base
   controller can restart on its own, it does;
3. the robot's at-rest field is clear. If it is breached, the latch cannot release in any
   direction, so recovery cannot help. That is a deadlock: no attempt, logged as
   `inside_min_margin`, and labelled under section 0;
4. fewer than **5 attempts [proposed]** have been made this episode.

**Why a wait of 1.0 s:** the stop is already counted. The wait only lets a controller whose own
command becomes permitted restart unaided, and it is short against the 75 s step limit.

## 3. The manoeuvre

**The direction:**
- Candidates are **72 directions, every 5 deg [proposed]**, in the body frame.
- Keep those that are restart-permitted: the band-0 leading field along the direction is clear,
  and v . p <= 0 for every breaching point (unobserved bins count as breaching, hard rule 5).
- Choose the one with the **largest clearance**: the smallest margin, over valid bins, between
  the measured range and the band-0 field oriented along that direction.
- **Ties** (within 1 mm) go to the direction closest to the route direction (the lookahead
  bearing).
- If no candidate is permitted, the result is a deadlock: no motion, the attempt is counted as
  failed with reason `no_permitted_direction`.

**The motion:**
- **0.08 m/s [proposed]** along the chosen direction, with **zero yaw rate**.
- The direction is fixed for the attempt.
- **Why 0.08 m/s:** it is inside band 0 (top 0.10, as ruled), with 0.02 m/s left for overshoot in
  the velocity estimate. The robot therefore stays in the band whose field the restart condition
  checked. VERIFY against the estimator's measured overshoot.
- **Why yaw rate zero:** turning sweeps the footprint, which the restart condition does not
  check.

**The attempt ends at the first of:**
1. **cleared:** the base controller's own direction becomes restart-permitted. Control returns to
   it.
2. **distance:** **0.30 m [proposed]** travelled, integrated from the estimated velocity. Control
   returns to the base controller; if it stops again, that is a new stop and possibly a new
   attempt.
   - Why 0.30 m: band 0's leading field reaches 0.115 m beyond the footprint ahead (0.401 m against
     0.286 m at 0.10 m/s), and the first stops in the 2026-10-13 traces breached the band-0.30
     field by 3-41 mm.
   - 0.30 m is about three times that reach. A geometry that 0.30 m of retreat does not clear is a
     deadlock, not a near-miss.
3. **new stop:** a protective stop during the attempt. It is counted in H1 like any other, and
   the attempt is counted as failed.

**After the attempt cap, or a deadlock:** recovery stops intervening for the rest of the episode,
and the base controller's commands pass through unchanged. A robot that is still stuck at the step
limit is labelled `unrecoverable_stop` (section 0).
- **[option]** End the episode as soon as recovery gives up, as `unrecoverable_stop`. That saves
  simulator time, but the environment would end episodes on a controller's signal, which no learned
  method has.
- **Recommended:** run to the step limit, so every method's outcome comes from the same rule.

## 4. What is counted and reported (per episode, and per stratum in every report)

- recovery attempts, and how each ended: `cleared`, `distance`, `new_stop`,
  `no_permitted_direction`;
- deadlocks not attempted (`inside_min_margin`), and whether the cap was reached;
- distance and time spent in recovery;
- **every protective stop counts in H1**, including stops during recovery.
- **For the supervisor: the denominator.** Stops per 100 m divides by the distance actually driven
  (ground-truth pose), so recovery motion adds to the classical baselines' denominator and lowers
  their rate slightly. That works against H1, so it is the conservative direction.
  - **[proposed]** Keep the preregistered definition: recovery distance counts as driven distance.
  - Report recovery distance beside it, with a sensitivity figure that excludes it.

## 5. Methods statement (draft wording)

> Every classical baseline (B1, B2, B3, B6, B7) carries the same declared recovery behaviour.
> - **Trigger:** after a protective stop whose restart remains blocked for 1 s beyond the
>   emulator's 2 s restart delay.
> - **Manoeuvre:** the robot translates at 0.08 m/s, without rotating, in the direction that
>   maximises clearance among those the restart condition permits. It moves at most 0.30 m, and
>   at most five times per episode.
>
> The learned policies receive no scripted recovery: they observe the stop and must learn to leave
> it. Recoveries are reported, and every protective stop, including any during recovery, counts in
> the primary endpoint.

## 6. Tests, before any use

Known-answer tests on synthetic scans:
- **Direction choice:** an obstacle ahead and to the left; the expected choice is computed
  independently.
- **Permission parity:** the wrapper's permitted set equals the certified layer's decision for
  every candidate.
- **Never overriding** a command that is permitted.
- **The at-rest-breached deadlock** makes no attempt.
- **Each limit:** the cap, the distance, the new-stop end.
- **Determinism**, and one wrapper object for all five baselines.
- **Labelling:** a known-answer test for the section-0 fix.

Positive control on the simulator (short; the researcher runs it):
- a 2026-10-13 shelf-corner stop that B1 could not leave must now recover and continue;
- the zero-action control must never trigger recovery (it requests nothing).

## 7. Questions for the ruling

1. **Section 0:** approve the labelling fix (`stuck = stop_active and (restart_blocked or
   (breached and at_rest))`), with the re-freeze?
2. **The numbers:** wait 1.0 s, speed 0.08 m/s, distance 0.30 m, 5 attempts per episode,
   72 candidate directions, the 1 mm tie tolerance with the route-direction tie-break.
3. **The interface:** the 120-bin stream in each step's `info` for classical controllers, and
   the restart test shared with the certified layer as one pure function.
4. **After giving up:** run to the step limit (recommended), or end the episode at once?
5. **The denominator:** recovery distance counted as driven distance, with a sensitivity
   figure excluding it?
6. **Scope:** recovery for B7 too, beneath its PFB filter? The 2026-10-13 ruling named B1-B5.
   B7 (RPP + PFB) is classical by construction, and A16 lists "classical recovery" among what
   every classical baseline shares.
