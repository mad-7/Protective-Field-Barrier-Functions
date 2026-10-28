# Write-up notes

Wording the thesis must use for WP0 results, fixed at the supervisor's review so that it
is not re-derived (and softened) at writing time. Each note names the ruling it comes from.

## Gates

- **G1b ceilings are "passes everything tested".** v_max = vy_max = 0.50 m/s is the top
  of the tested grid, not a measured limit. The 0.50/0.50 m/s measurement box exceeded
  the feasible set -- (0.50, 0.50) needs 1.0 m/s at the wheel rim against a ~0.617 m/s
  ceiling -- so the grid's corners were never executable. (WP0 results, ruling 6.)
- **G1-ODE's failures are speed-shortfall failures an open-loop test cannot correct.**
  Forward (12.0 %) and translate_while_rotating (12.6 %) fall short of the 10 % criterion
  at the configured limits; diagonal (33.7 %) is additionally wheel-saturated. G1 (Bullet)
  FAIL and G1-ODE FAIL are both reported; the fallback attaches to G1b, which passed.
  (WP0 results, ruling 6; A1, A7.)
- **G1 (Bullet) FAIL stands** as the record of the physics then in force. (D6, A1.)
- **G1b's history belongs in the methods section as a chain**, each link kept on record:
  (1) first run, v_max 0.40 under a per-axis clamp defect -- superseded (A6); (2) re-run,
  0.50/0.50 "passes everything tested", with no wheel budget -- **superseded: measured
  before the wheel budget**; (3) re-run on the current controller, with the budget, three
  invocations: **PASS at v_max 0.40, vy_max 0.50** (2026-09-20,
  `G1b-SPREAD_20260920T215108Z`). Only (3) is quoted. That the chain ends where it began
  numerically -- 0.40 from the defective clamp, 0.40 from the budget -- is a coincidence
  of two different mechanisms, and saying so is part of the record.
- **The ceiling is set by one cell: translate_while_rotating at 0.50 m/s**, 5.033-5.052
  deg of final heading error against a 5.0 deg limit, in all three invocations (spread
  0.019 deg). Not a straddle, not a near miss to be argued away: three independent
  invocations all miss by ~0.05 deg. Every other profile passes at 0.50.
- **The actuation box is non-square (v_max 0.40, vy_max 0.50) because the measurement
  is.** The wheel budget is the physical constraint; the axis ceilings are what the gate
  could demonstrate. The MDP section states the asymmetry, because the policy's
  normalised action scales each axis by its own ceiling.
- **The certified layer's band edges come from this platform's demonstrated limit, not
  from a vehicle specification or a standard.** `band_top_speeds` [0.10, 0.30, 0.40] is
  bounded above by v_max 0.40, which is what G1b could demonstrate on this youBot under
  the wheel budget -- not a manufacturer's rating, not a figure from IEC 61496 or
  ISO 3691-4, and not a speed a real AMR would be certified at. The band *structure*
  (discrete speed-banded fields, a field sized for its band's top speed, a latched stop)
  follows the certified-layer pattern those standards describe; the *numbers* are ours.
  Any claim about protective stops is therefore a claim about this platform's envelope.
- **Under the wheel budget, cross-track fell and heading error rose**
  (translate_while_rotating at 0.50: 25.7 -> 12.7 mm, 0.27 -> 5.05 deg), because the
  direction-preserving projection scales omega with translation. This motivates the WP2
  polytope constraint. (Supervisor, 2026-09-19.)
- **Dynamics-step tuning is closed** (A10): no step above 0.005 s keeps the signed drift
  within the 0.64 deg band, and a tenfold cut in physics work saved ~10 ms of ~55 ms per
  step. Tunnelling was not the limit (zero detection lag at 0.50 m/s at every step).

## Implementation

- **Scanning is preprocessed server-side, and the measurements say why.** Reading the
  scanners over the remote API costs **14-17 ms per step** -- four vision sensors, eight
  round trips at ~1.5-1.8 ms each -- and the cost is latency, not data: the smallest
  buffer measured (656 B) was the slowest (17.0 ms), the largest (4096 B) among the
  fastest (14.4 ms). `lua/scanner_v2.lua` therefore bins inside the simulator and returns
  one packet per step, as v1 did (36 bins, 144 B). The same argument makes the whole step
  one server-side call. (Measured 2026-09-19/20 on the frozen physics, one instance.)
- **Scanner cost is per sensor, not per ray.** Rendering four 120-deg vision sensors costs
  ~1.5 ms each; cutting the rays from 1024 to 164 -- past what the 36- and 120-bin streams
  consume -- saved ~1.1 ms. Resolution is therefore chosen for sensing, not for speed, and
  the scan is sized by `pfbnav.sim.scanning.resolution_for_bins`.
- **Rendering is switched off during training** (`sim.display_enabled: false`, ~7 ms of a
  ~27 ms step), and every run records whether it was on. Trajectories are bit-identical
  either way (`scripts/regress_display_off.py`), so it is a cost saving and not a
  condition of the result.

- **Symmetric test geometry hides a class of bug that asymmetric platforms expose.** Two
  defects survived every check while the robot carried two identical sensors on its
  centreline, and both surfaced within a day of moving to scanners on diagonal corners:
  validity logic that judged coverage by angle from the body origin, as if every sensor sat
  there (it reported a 0.563 m blind zone beside the robot as free space), and case
  placement that took the leading part's lateral offset as a magnitude, which mirrors
  correctly only when the part is mirrored. Symmetry makes wrong answers agree with right
  ones. Industrial AMRs mount scanners on diagonal corners precisely because that covers
  the ring with two units, so the asymmetric case is the realistic one and the one to
  develop against. (2026-09-21/22.)

## Limitations

- **Sensing is "an S300-type layout", never certified sensing.** Two 270 deg scanners
  modelled on the SICK S300 Fast sit flush on diagonal corners (v2 plan section 2.2); the
  centreline pods were the inherited platform. The simulated scanners share the layout and
  field of view, not the certification, and the text must not say or imply otherwise.
  (Supervisor, 2026-09-21.)
- **Obstacles below the scan plane are not seen.** The S300 layout's scan plane sits at
  **0.221 m** above the floor (measured, `LAYOUT_20260922T170201Z`), clearing the wheel tops
  (0.100 m) and the chassis top (0.176 m). Anything lower is invisible to the scanners:
  pallet forks and pallet feet, low kerbs and thresholds, dropped cartons, and a person's
  feet when the rest of them is out of the aisle. The certified layer and the PFB both act
  on this one plane, so a low obstacle produces no protective stop at all -- it is not a
  matter of a late stop. Contacts are still counted, because collisions come from the
  simulator's contact check on the full geometry (hard rule 4), so a low obstacle shows up
  as a collision with no preceding stop. Planar scanning is what an S300-class AMR does;
  the honest statement is the height and what passes under it.
  - **Demonstrated, not merely argued, by G0** (2026-09-26, three invocations). Of the five
    positive contact cases, four would have been stopped by the certified layer at step 15
    with contact at step 36; `wheel_edge` records `certified_would_stop_step: None` -- the
    layer never triggers. Its slab is **0.012 m** high, deliberately low enough to pass under
    the chassis (underside 0.0148 m), so it sits far below the 0.221 m plane. Neither the
    emulator nor the PFB can see it, **by construction**, and this is the concrete case to
    cite for the limitation rather than a hypothetical.

- **Unexplained simulator behaviour: the display setting is sometimes refused mid-run.**
  On 2026-09-20 CoppeliaSim 4.10 accepted `display_enabled = false` and then reported it
  true again part-way through runs -- 3-4 times per G1b invocation, after dozens of
  successful applications. The cause is not known; interaction with the GUI window is the
  untested hypothesis. **It is shown not to affect trajectories:**
  `scripts/regress_display_off.py` ran one G1b profile with rendering on and off in a
  single process and the trajectories were bit-identical (362 rows, no differing field),
  so a refusal costs time and not correctness. Reported as a known, unexplained platform
  behaviour rather than quietly worked around.
  - **It recurred on 2026-09-26, on every instance** -- 5, 3 and 4 refusals out of 66
    episode starts across the three G1b invocations -- so it is not specific to one session.
    The supervisor ruled on 2026-09-26 **not to hunt the cause** but to retry: the setting is
    now applied up to three times, retries are counted separately from refusals, and a start
    that takes on the retry costs a retry rather than being reported as a refusal.
    Escalation to a one-day diagnosis is triggered by refusals above **10 % of starts**, or
    by rendered and non-rendered throughput differing materially. G2 -- the one gate whose
    result *is* wall clock -- reports throughput split by rendered and non-rendered episodes
    and **fails if more than 10 % of its episodes rendered**, so the effect cannot hide in
    the mean of a mixture.

- **No scenario requires replanning around a blocked route** (ruled 2026-10-02, A13). The stack
  plans once on the mapped map and never replans, so an unmapped obstacle is kept only if the
  robot can get past it -- all of them jointly -- within a corridor of half-width W = 1.0 m around
  its planned route, keeping the narrow stratum's 0.13 m floor beyond its footprint. Without this
  rule, at least 29 % of pilot scenarios had obstacles that could be passed only through another
  aisle (2026-10-02 dry run). A pallet left across an aisle the robot was sent down is a real
  warehouse case; it is **outside this study's scope**, and global replanning on detection is the
  future work that addresses it. State it wherever the unmapped condition is described, together
  with where obstacles actually landed per stratum (inside aisles, at aisle mouths, on open floor),
  because that defines what "an unmapped obstacle in the narrow stratum" means.

## The actuation box is non-square, and the asymmetry is physical (PRE-A20)

*Pre-A20 material, kept as the record of what was measured then. Since 2026-10-19 the box is
square, v_max = vy_max = 0.50: see "The reachable-motion field and its limits (A20)" below.*

Confirmed by the supervisor 2026-09-26 (Q4): **v_max 0.40 m/s forward, vy_max 0.50 m/s
lateral**. The MDP section states the asymmetry, and the policy's normalised action scales
each axis by its own ceiling -- `a` in [-1, 1]^3 maps to `(a0 * v_max, a1 * vy_max,
a2 * w_max)`, never to one shared speed, which would let the same normalised magnitude ask
for more lateral than forward motion and bias exploration sideways for no physical reason.

**The evidence that the box is not what constrains the robot** (G1b, 2026-09-26, three
invocations; `runs/gates/G1b-SPREAD_20260926T193955Z_seed0`). Read alone, the clamp binding
counts look as though the commanded box shaped the result: 4,059 clamped steps, concentrated
in the cells whose commanded speed equals the box. How hard each constraint actually bites
says otherwise:

| cell | min `clamp_scale` (the box) | min `budget_scale` (the wheels) |
|---|---|---|
| forward @ 0.50 | 0.999999 | 1.000000 |
| lateral @ 0.50 | 0.999979 | 1.000000 |
| diagonal @ 0.50 | 1.000000 | **0.829** |
| translate-while-rotating @ 0.40 | 1.000000 | **0.819** |
| translate-while-rotating @ 0.50 | 0.999996 | **0.656** |

The box removes at most two parts in 100,000 of the commanded vector; the **mecanum wheel
budget** (`|vx| + |vy| + (l+w)|wz| <= 0.80 * 0.617 m/s`, enforced by direction-preserving
projection) removes up to a third of it. The one failing cell fails while the budget holds it
at 66% of what was asked, so **the 0.50 m/s failure is a capability limit -- the base cannot
translate at 0.5 and rotate at once within its wheel budget -- and not an artefact of the
clamp.** Squaring vy_max down to v_max would therefore discard measured lateral capability
that the wheels do support: G1b passes the lateral profile at every grid speed and at both
declared extensions (0.35, 0.45 m/s).

Report the binding counts **with** the scales. The counts alone invert the reading.

## Reading H2

Recorded as ruled by the supervisor on 2026-09-29.

- **H2's power stays near 50 % at a true effect of exactly +15 percentage points, at any sample
  size.** Its preregistered success criterion is "difference >= +15 pp AND significant after
  Holm". When the true effect equals the threshold, the observed difference falls below it about
  half the time, so more scenarios cannot lift the probability of success above ~0.5. Sample size
  buys power only for true effects above the threshold (Monte Carlo of the frozen analysis: 0.92
  at +20 pp with 60 pairs, 0.96 with 100). A null verdict on H2 therefore does not mean the effect
  is below 15 pp, and a positive one needs the effect clearly above it.
- **H2's confidence interval is reported regardless of the verdict**, so a reader can see where the
  effect lies whichever side of the threshold the point estimate fell.

## Reading the offset analysis

Recorded as ruled by the supervisor on 2026-09-30 (A13).

- **H1 against obstacle offset is descriptive and exploratory, not confirmatory.** It is reported
  against the per-scenario **minimum** surface offset of the unmapped obstacles to the planned
  route.
- **That minimum's distribution is shaped by design.** The first obstacle's offset is capped at
  0.3 m so that every `unmapped_static` scenario has a surface within 0.3 m of the route (the
  validity check); any further obstacle is uniform on [0, 0.6] m. The minimum therefore never
  exceeds 0.3 m, and scenarios with a minimum between 0.3 and 0.6 m do not exist. Say so wherever
  the offset is plotted.
- "First" is an index, not a location: the capped obstacle is placed along the route exactly as
  the others are. The dry run's report of mean position by index is the evidence; quote it.

## Stratum and obstacle location are confounded

Ruled by the supervisor on 2026-10-02.

- **Where unmapped obstacles land depends on the stratum** (A13 as revised, W = 1.0 m; inside an
  aisle / at an aisle mouth / on open floor): narrow 0 / 3 / 97 %, medium 4 / 17 / 79 %, wide
  54 / 7 / 39 %. A narrow-stratum encounter is almost always on open floor; a wide-stratum one is
  inside an aisle more often than not. Stratum and location cannot be separated in this population.
- **Any per-stratum breakdown of H1 reports obstacle location alongside it**, so a difference
  between strata is never read as an effect of aisle width alone.
- **Do not claim a PFB benefit for unmapped obstacles inside narrow aisles.** That case does not
  occur in the test population: an obstacle there would leave no bypass at the narrow floor (1.08 m
  needed, 0.82 m available), so A13 excludes it by construction.

## Platform and method

- **No steps/s figure from WP0 or WP1 is a property of the method.** All of them were
  measured on the **development laptop** -- 4 physical cores, 8 logical, 7.7 GB, CPU-only
  torch, four CoppeliaSim instances sharing those cores, and **GUI rendering in force
  unless stated** (~7 ms per step). Training runs on a **separate GPU machine**, and the
  learner's cost (7.5M SAC gradient updates) has never been timed at all, so these figures
  describe neither the training host nor the whole loop. The compute budget is stated as
  a requirement on the training host (>= 25 steps/s, derived in `docs/requirements.md`)
  and measured there by G2; every run records its host in `provenance.json`.
- **What transfers between machines is shape, not speed:** the dynamics step is frozen at
  0.005 s (A10 -- no larger step holds the drift band, and physics is ~6.5 ms of the step);
  a per-call scan costs 14-17 ms in round trips and so the scan is assembled server-side;
  scanner cost is per sensor, not per ray; rendering changes cost and not results.
- **An arithmetic error is worth reporting, not hiding.** The ">= 100 steps/s" target that
  drove the compute work rested on "7.5M steps do not fit six weeks at 50 steps/s"; 7.5M
  over 3,628,800 s is 2.1 steps/s. Found and corrected 2026-09-20; the replacement is
  derived in the open. Every numeric requirement now carries its derivation, and one whose
  derivation does not reproduce is challengeable. (2026-09-20.)

- **State the engine** in every figure, table and methods sentence. Holonomic fidelity
  varies ~40x across engines (lateral drift 0.47 deg ODE, 3.12 Newton, 20.55 Bullet;
  MuJoCo failed). (A8.)
- **Quote the 0.64 deg solver-sensitivity band** with every drift figure; its
  non-monotonic trend across dynamics steps is numerical noise. (A8.)
- **Velocity estimate is privileged.** The safety layers use filtered pose differencing
  of the ground-truth `youBot_ref` pose, which a real AMR lacks; results carry that caveat
  and the WP2 sensitivity check (noise, one-step latency) is reported with them.
  (WP0 results, ruling 4.)
- **Wheel physics is a hidden sphere proxy.** CoppeliaSim's youBot drives on four hidden
  respondable spheres (r = 0.05 m) on free slipping joints; the visible mecanum meshes do
  no physics. The spheres reach 12.5 mm further sideways than the real wheel. They count
  as collisions (WP0 results, ruling 1), which is conservative against the real platform:
  some counted contacts are ones the real wheel would not make. The speed shortfall's
  residual slip arises at this sphere-floor contact.
- **Collisions honour the scene's collidable flag inside collections**, but not for a
  single handle (measured 2026-09-18). The contact definition is therefore partly set in
  the scene; every run records the scene's SHA-256.
- **Separate simulator invocations are not bit-identical**; repetitions within one
  invocation are. Determinism claims are scoped accordingly.


## Route-level traversability: the oracle, its floor, and the margin asymmetry

Fixed at the supervisor's rulings of 2026-10-15 and 2026-10-16; use this wording.

- **The unavoidable-stop floor is platform-specific: a holonomic base with full knowledge of its
  route.** It is computed by an oracle that follows each planned route exactly, in the fastest
  speed band whose field (along the velocity) is clear, choosing its body orientation within yaw
  continuity, yaw rate, yaw acceleration and the wheel budget. Say so wherever the floor is
  reported. It is not a property of the routes alone.
- **A1 (unicycle) has its own floor**, from the tangent-locked oracle, whose body always faces the
  route. Report it beside L5's.
- **H4's directional expectation was preregistered before any result** (A17): A1 shows a materially
  higher narrow-stratum stop rate than L5. It is exploratory.
- **The planner's lethal margin is 0.10 m** (lethal radius 0.3086 m; ruled 2026-10-16, before A18,
  replacing 0.07). It is the ceiling the narrow floor allows: the centreline band is
  2 x (0.13 - 0.10) = 0.06 m, about one 0.05 m grid cell. A permanent regression test checks that
  the narrowest aisle plans at every sub-cell position.
  - **Straights** clear the band-0 field at G1b's 50 mm tracking bound with 0.03 m to spare (narrow
    aisle centreline: 0.3386 - 0.05 - 0.2586).
  - **At a bend, a tracker 50 mm inside sits exactly on the band-0 side field: zero slack**
    (0.3086 - 0.05 = 0.2586 m).
  - In the information phase the 50 mm inside-bend breach rate fell from about 94 % of routes at
    0.07 to 0.8-4.2 % at 0.10 (wide 0.8 %, medium 1.8 %, narrow 4.2 %), with plannability at 100 %.
  - **The residual inside-bend breaches are controller-attributable and count in H1**, as do stops
    caused by not slowing at bends.

## The protective field's speed signal (ruled 2026-10-16)

Use this wording.

- The certified layer sizes its field from **max(estimated speed, applied post-limiter command
  speed)**, and its rotation sweep from the larger of the estimated and the applied yaw rate.
  - The command understates true motion while decelerating; our low-pass estimate understates it
    while accelerating. The maximum is never below true motion; an invariant test on simulator
    ground truth asserts it at every step.
  - WP2's PFB uses the same composition.
- **A real AMR's safety scanner takes an unlagged, safety-rated speed signal** (typically
  encoder-based incremental inputs to the safety controller). Ours is a privileged signal, the
  simulator's ground-truth pose differenced and low-pass filtered, and it is lagged. The
  composition with the applied command compensates for that lag and is a property of this
  emulation, not a claim about certified hardware.
- **The field contains the footprint's full stopping sweep:** translation braking along the
  velocity, and yaw braking at the current yaw rate (`alpha_max`, which the limiter enforces
  symmetrically), each over response time plus deceleration, then the margins. The
  rotation-blind field used until 2026-10-16 was an environment defect.
- **Corner deadlocks become stop-restart cycles** (2026-10-16). Under the corrected field, a
  classical tracker cutting a bend is stopped earlier and farther from the corner, so its restart
  is rarely blocked: 0 of 80 constructed corner scenes gave a robust restart-blocked stop (the
  synthetic recovery control, closed as a negative result). **Those cycles still count as
  protective stops in H1.**

## The reachable-motion field and its limits (A20, ruled 2026-10-18/19)

- **The safety property is containment:** everything the footprint sweeps from the stop trigger to
  rest lies inside the field in force at the trigger.
  - The 2026-10-16 construction sized the field from the speed and yaw rate at the decision, held
    constant. A command still accelerating through the response time left it: the traced turns
    rotated up to twice the sweep, and stops ended inside the minimum margin.
  - The A20 field contains the motion REACHABLE within T = response time + one control step:
    - the band from v + a_max T;
    - the yaw interval reachable at alpha_max;
    - the swerve an isotropic limiter allows, which a holonomic or learned controller can command;
    - at rest, the isotropic reach from the standstill speed.
- **The acceleration limits are a safety-design parameter co-specified with the fields, not
  measured youBot properties:**
  - a_max 0.5 m/s^2: band coherence, a_max T below the narrowest band width;
  - alpha_max 1.4 rad/s^2: a_max over the outermost radius, so no body point accelerates faster.
  - In the simulator the limiter binds and the torque cap never does. The ACHIEVED acceleration is
    0.78-0.86 x a_max (0.88-0.93 x alpha_max), so the field's reach assumption is conservative.
- **Bands by reachable speed, [0.10, 0.30, 0.51]:**
  - the top band covers the largest reachable composed speed;
  - the uncovered lateral range above 0.40 m/s was a defect of the G1b ruling.
- **Strata from F = 0.1735 m:** the smallest clearance with every D2 gap >= 0 under the new field,
  plus the 0.03 m buffer.
- **v_max = vy_max = 0.50 (the box square again).** G1b's outcome rule under the A20 limits:
  translate_while_rotating passes 0.50 m/s by **0.038 deg** of final heading error (4.947-4.962
  against 5.0; spread 0.015 across three invocations).
  - The result is specific to the A20 limits. Under the pre-A20 limits the same cell failed by
    0.03-0.05 deg, which had set v_max to 0.40.
  - v_max is a tracking ceiling, not a safety parameter: the fields cover reachable speeds to
    0.51 m/s regardless.
- **Memory:** CoppeliaSim's committed memory grows about +1.7 MB per 100 episodes (0.1 standard
  error; Python flat), inside the declared bound of 10 MB / 100. It is recorded in every run, so
  long runs can be checked against it.
- **Nesting and the creep regime (A21).**
  - Every stop begun in motion nests: the at-rest field it ends in lies inside the field in force at
    the trigger.
  - That requires a drift term for a body that turns while it translates, D(V) 2 sin(e(|omega|) / 2).
    The first containment test missed it: it held the body-frame velocity fixed while the body
    turned.
  - Containment plus nesting leave a departure shell (about 9-11 mm) in every direction. A restart
    must clear the whole field its twist selects, and the shell is left by creeping below the
    standstill exit, where the at-rest field governs. This mirrors industrial manoeuvring field
    sets.
  - **The asymmetry, stated:** the creep is a learnable skill for the learned methods (stop_active
    stays set while blocked; the creep speed is inside their action range) and is scripted for the
    classical baselines by their declared recovery.


## Limitations: the fields are slow, above all in rotation (recorded 2026-10-20, A23; not fixed)

- **WITHDRAWN 2026-10-24 (supervisor): an artefact of the oracle schedule's 0.20 rad/s yaw cap, not a property of the fields. With the PFB, the ideal tracker reached the goal on 48 of the 60 tune routes (the other 12 stall on unmapped obstacles it cannot see) at a median 0.37 m/s (0.28-0.42; 2.7 s/m against the oracle's 8.4), with zero unattributed certified stops and no contact (runs/analysis/PFB_ACCEPTANCE_20261024T174707Z_seed0, emulator, sector scan). The text below is kept as recorded.**
- **An ideal tracker averages about 0.12 m/s under A20-A22.** The privileged oracle tracker knows
  every obstacle and follows the schedule exactly, yet needs a median 88 s for a 10 m tune route
  (8.4 s/m), against 0.5 m/s as the platform's ceiling. Two causes:
  - **Rotation.** The body may turn at only 0.20 rad/s (rounded down per route, 0.10-0.20) before
    the swept band-0 field breaches at shelf ends. A 90 deg alignment at each aisle costs about
    8 s.
  - **Band 0.** Where only band 0 is clear, the usable speed is 0.025 m/s, because the reach term
    a_max T is 0.075 of the band's 0.10.
- **This is a property of the emulated certified layer, not of any method.** Its field sizing is
  more conservative than typical industrial configurations, especially in rotation
  (`docs/realism_audit.md`).
- **The results report the oracle mission time as a reference line.** Every method's time is read
  against it, never against the platform's ceiling.
- **A withdrawn figure:** the A20 information run's "mission time 1.22 x constant"
  (`ORACLE_FREE_info_20261019T164220Z`). It gave the faster bands no rotation sweep and rotated in
  place at 1.5 rad/s with no field check.
- **The classical baselines carry a map-based speed and yaw-rate cap the learned arms do not**
  (ruled 2026-10-20). Without it a fixed-speed tracker is stopped at the first tight spot and never
  released under the whole-field restart. Even with it, B1 (an 0.8 m lookahead pointing its field
  across bends) still fails the three tune routes it was emulated on; that is recorded as a
  baseline finding.
- **L5-nowp** is the exploratory planner-contribution arm (A23). v1 is the pilot study that
  motivated it.

## The contribution, C1 (restated by the supervisor, 2026-10-23)

- **C1: a control barrier function whose safe set is the complement of the certified safety
  function's own trigger set.**
  - The PFB keeps every scan return outside the field of the band the robot's reachable speed
    selects: the very field whose breach makes the certified layer stop it (Q1 option (b), one QP
    per band, best feasible).
  - **The technical content** is in the direction, footprint and rotation terms of that field: the
    swept footprint, the travel and swerve terms along the commanded direction, the coupling, and
    nesting. All are built from the same certified parameters (same config keys).
- **Reporting:** success rate and mission time sit beside H1 for every comparison, in the same table.
  The discussion LEADS with the stop-throughput trade-off. **B6 (field-aware MPPI) is the decisive
  comparator.**
- **H3 is restated, before any training, as an exploratory ablation:** PFB-(b) banded vs PFB-(a)
  continuous.
