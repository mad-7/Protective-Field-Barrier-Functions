# Decision request — the narrow stratum's floor (2026-09-27)

Your 2026-09-26 ruling on the narrow stratum is implemented: slack 0.02 → 0.03, narrow floor
0.12 → **0.13 m** per side, band edges now narrow 0.13–0.20, medium 0.20–0.40, wide 0.40–0.80,
and the sector-min term derived explicitly (`pfbnav/world/strata.py`, commit `1148f5f`).

**It does not fix the problem you were pointing at**, because the binding case is not the one
the precondition test was written to catch. One decision needed before the strata can be locked.

## What binds, and what does not

With half_width 0.2086 m and the committed peak cross-track 0.05 m:

| field | sideways reach | per-side clearance needed at the 0.05 m offset |
|---|---|---|
| moving, top band | 0.25951 m | **0.1009 m** |
| **at rest** | **0.30951 m** | **0.15091 m** |

The at-rest field is wider abeam by construction — `margin` 0.10 all round at rest against
`lateral_margin` 0.05 when moving.

**So moving, the narrow stratum was already admissible at 0.12 m.** Raising the floor to 0.13
widens the buffer; it changes no verdict. **At rest it is not admissible even at 0.13**, because
0.15091 > 0.13.

How far off-centre a robot may come to rest and still be clear:

| stratum | admitted | controller is permitted |
|---|---|---|
| narrow @ 0.13 | **29.1 mm** | 50 mm |
| medium @ 0.20 | 99.1 mm | 50 mm |
| wide @ 0.40 | 299.1 mm | 50 mm |

## Why this is worse than "permanently inside the field"

A latched stop holds **while the field is breached**, and for `restart_delay` after it comes
clear. A latched stop also zeroes the command. So a robot that stops off-centre in a narrow
aisle is inside the at-rest field, cannot be commanded, and therefore cannot recentre to clear
the breach. **The stop is unrecoverable and the episode ends by timeout.**

The emulator is behaving correctly. The aisle is the problem. Your precondition test — drive the
centreline at the derived speed with a lateral offset and record the fraction of steps inside
the field — would report the narrow stratum as *fine*, because while moving it is.

## The options

**1. Keep 0.13 and treat the unrecoverable stop as a real failure mode.** Count it, report it,
and accept that one stop ends a narrow-aisle episode. Honest — it is what a real certified layer
does — and it keeps a 70 mm-wide narrow stratum where the contribution is supposed to show. It
makes the narrow stratum's outcome distribution bimodal, and makes any narrow-aisle stop
terminal rather than costly.

**2. Raise the floor to 0.181 m** (0.15091 + slack 0.03), so a stop anywhere the controller may
legitimately be is recoverable. Follows your standing rule — raise the floor, never shrink the
field — but leaves narrow at 0.181–0.20, **19 mm wide**. That is barely a stratum, and it sits
where the PFB is meant to earn its result.

**3. Change what the strata are derived against.** For example derive the at-rest floor from a
tighter stop-time cross-track bound than the 0.05 m tracking bound, if a controller can be
required to recentre before coming to rest. This needs a criterion that does not exist yet and
would be a new preregistered requirement, not a re-derivation.

**My recommendation: option 1**, with the unrecoverable stop declared, counted and reported per
stratum. Option 2 buys admissibility by deleting the stratum the thesis is about; option 1
keeps the stratum and states the limitation, which is the same trade we took on the
below-the-scan-plane limitation and on `wheel_edge`.

## A second question it raises

If a single stop can end a narrow-aisle episode, is **stops per 100 m** still the right primary
metric there? Per 100 m presumes the robot keeps travelling. In a stratum where the first stop
is terminal, every method scores at most one stop per episode and the metric compresses toward
the episode length rather than the behaviour. The narrow stratum may need its own primary
metric — completion rate, or distance to first stop — reported beside the pooled figure.

## A correction to the derivation's premise

You asked for the **sector-min quantisation term** to be named in the derivation. It now is,
computed exactly rather than assumed: **2.60 mm** at the old 0.12 floor, **1.41 mm** at 0.13,
and it shrinks as the aisle widens.

It is about **a quarter** of what the real scan actually eats. D2 against the real packet
measured the tightest bin clearing by 9.5 mm where the geometry predicts 19.1 mm — a 9.6 mm
loss. Sector-min binning accounts for 2.6 mm; ~1 % range error is ~3.3 mm at a 0.33 m wall; the
~3.7 mm residual is sensor parallax and mesh sampling. **The slack is therefore justified
against the measured 9.6 mm total** — 0.03 covers it about three times over — and the test
asserts that relation rather than the tidier one against the derived term alone. If the slack
were ever set from the derived term, it would be set about four times too small.

## What is waiting on this

- The traversability precondition harness, and the per-stratum unavoidable-stop floor with H1
  reported above it. Both measure against a floor that may move, and the narrow stratum's
  unavoidable-stop floor is dominated by exactly this mechanism.
- Freezing `preregistration/` — the strata sit inside the `layout_sets` block, so locking before
  the decision means re-locking after an amendment.
- Therefore G2, the scenario sets, and SAC.

Everything else from the 2026-09-26 rulings is done: Q1–Q2 preregistration text drafted for
commit (with the pilot seed range and the count and placement rules), Q3 the new dropout
mechanism, Q4 the clamp-versus-budget evidence in the write-up notes, Q5 case six plus the
reconciliation that makes adding a case an amendment rather than a code change, Q6 the
emulator's own warning field with near-miss entries counted, Q7 the display retry and G2's
rendering criteria.
