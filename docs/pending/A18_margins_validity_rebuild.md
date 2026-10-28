## A18 - 2026-10-16 - Planner margins by the oracle criterion; the generator's validity rules; the scenario identity covers its imports; sets rebuilt

**Decided by the supervisor on 2026-10-15 and 2026-10-16** (three rulings, recorded in
`docs/decision_log.md` before any measurement under them). The soft margin is chosen by the rule
declared in those rulings, on the free-orientation run at lethal 0.10
(`runs/analysis/ORACLE_FREE_decide_20261016T003847Z_seed0`, 8906820, clean, scene 537f1bbb). No test layout was
built, read or used; no method has been trained or evaluated.

**What changed.**

1. **Planner margins.** `planner.lethal_margin` 0.03 -> **0.10** (lethal radius 0.2386 -> 0.3086 m).
   - The old lethal radius sat inside the at-rest field abeam (0.2586 m), the D1 defect.
   - 0.07 was ruled first (2026-10-15): the smallest margin clearing every straight at G1b's 50 mm
     tracking bound. At 0.07, though, a tracker 50 mm inside a bend breached band 0 on about 94 %
     of routes.
   - **0.10 was ruled on 2026-10-16, before this amendment:** that rate falls to 0.8-4.2 % with
     plannability at 100 %, and 0.10 is the ceiling the narrow floor allows (centreline band
     2 x (0.13 - 0.10) = 0.06 m, about one grid cell).
   - At 0.10 a tracker 50 mm inside a bend sits exactly on the band-0 side field (zero slack,
     against 0.03 m on straights). The residual inside-bend breaches are controller-attributable
     and count in H1.
   - The sub-cell narrow-aisle plannability check is a permanent regression test.
   - `planner.soft_margin` stays **0.03**, chosen by the declared rule at lethal 0.10. Every
     candidate from 0.03 to 0.30 tied at zero bend-stop routes, every route stayed plannable, and
     the tie goes to the smaller. It was the same at 0.07.
2. **The margin criterion** is amended from "an ideal tracker at the stratum's traversal speed" to
   **an oracle tracker:** it follows the route exactly, in the fastest band whose field is clear,
   deceleration-limited, with a free body orientation subject to yaw continuity, a yaw-rate and
   yaw-acceleration limit and the wheel budget (the field lies along the velocity; the footprint
   turns with the body). **Criterion: zero routes with a band-0 breach.** The per-route
   unavoidable-stop floor is this oracle's, and is platform-specific: holonomic with full knowledge
   of the route. A1's floor is the tangent-locked oracle's (A17).
3. **The generator's validity rules.** A candidate route is kept only if the free-orientation oracle
   finds it valid:
   - **start:** the band-0 field along the route's initial direction is clear at the start, with
     the body at its start yaw (set along the route's first segment, as before);
   - **goal:** some orientation's band-0 field along the final approach is clear on arrival;
   - **no residual:** a continuous orientation path keeps band 0 clear from start to goal.
   On any failure the **goal is re-drawn, keeping the start and the layout**, up to
   `world.layout.validity.goal_resamples` (8) times; after that the next endpoint pair in the
   same layout is tried. The layout is never re-drawn.
   (The count is the implementer's choice, not ruled: it only decides which goal a rejected
   start ends with, never whether a route is valid. For the supervisor to confirm.) Every
   rejection is logged by cause, per scenario and per stratum in the set's manifest
   (`validity_rejections_by_stratum`). A seed whose every candidate fails is a named failure,
   `no_route_passes_the_validity_rules`, never an easier scenario. The oracle's resolution is
   the one the decision run used (`world.layout.validity`: 0.02 m samples, 15 deg orientation grid
   plus the route's own directions, 1 deg heading rounding).
4. **`scenarios_sha256` now covers the generator's whole import closure** (every `pfbnav` module
   `pfbnav.world.scenarios` reaches by import, hashed by path and normalised text). It hashed
   `scenarios.py` alone, so a change to the validity oracle -- or, since A13, to `placement.py` or
   `bypass.py` -- could move every route without moving the identity a set is verified against
   and the lock records. This changes the hash's definition; the relock records the new value.
5. **Every built set is rebuilt** with the above: tune, tune_unmapped, pilot and train. Their
   seeds, sizes, strata and unmapped rates are unchanged (A13 and its revisions). The test sets
   are not built yet (A14) and will be built by this generator.

**What was already known at the time.** On the built sets (made at lethal 0.03, before these
rules):
- **At lethal 0.07** (decision run `ORACLE_FREE_decide_20261015T210142Z`):
  - every route was plannable at every soft margin;
  - residual 0 at soft 0.03-0.13;
  - 20 start-invalid routes, all narrow, and 0 goal-invalid;
  - soft 0.03 chosen.
- **The information phase** (`ORACLE_FREE_info_20261015T223837Z`, soft 0.03):
  - plannability 100 % at both 0.07 and 0.10;
  - the same 20 start-invalid routes at both margins;
  - 50 mm inside-bend breach routes about 94 % at 0.07, and 0.8 % (wide) / 1.8 % (medium) /
    4.2 % (narrow) at 0.10;
  - oracle mission time against constant top speed, median 1.090 at 0.07 and 1.075 at 0.10.
- **At lethal 0.10** (`ORACLE_FREE_decide_20261016T003847Z_seed0`):
  - every route plannable at every soft margin, no quantisation failures;
  - **residual 0 in every stratum at every soft margin**, so the residual rule says re-sample,
    with nothing to re-sample (the two continuity residuals seen at soft 0.20/0.30 under 0.07 are
    gone);
  - **20 start-invalid routes, all narrow, the same 20 as at 0.07; 0 goal-invalid.**
  - The free floor is narrow 0.217 per 100 m and 0 elsewhere: one stop at the start of each of
    those 20 routes, none at a bend, on a straight or at a goal.
  - **A1's tangent-locked floor rises with the lethal margin:** narrow 4.688 per 100 m (374 of 857
    routes) against 2.649 (211) at 0.07; medium 0.131 against 0.053; wide 0.040 against 0.
  - A1's floor also falls as the soft margin grows (all strata pooled: 1.833 at soft 0.03, 0.828 at
    0.10). The declared choice rule uses the free oracle only, so this did not enter it. It is
    reported beside L5's whenever H4 is reported (A17).

The rules re-draw every rejected route, so the rebuilt sets' oracle floor is expected to be
zero.

**In the preregistration.** Nothing in `gates.yaml` or `endpoints.yaml` names a planner margin, so
neither file changes. The lock is renewed for the new `scenarios_sha256` (`--relock A18`).
