# Preregistration amendments



Amendments to the committed criteria in `preregistration/gates.yaml`, in date order.

This file is append-only. It records *changes to the preregistration*, and the reasoning
behind them, so the difference between what was planned and what was done is visible
without reading the git history. It never restates the criteria themselves: those live in
`gates.yaml` and are authoritative.

Each amendment records what changed, when, who decided it, and what was already known at
the time. An amendment made *after* seeing a result it affects is marked as such, because
that is the fact a reader needs in order to judge it.

---

## A1 - 2026-09-18 - Gate G1b added; G1 stands as failed

**Decided by:** supervisor, WP0 review.
**Known at the time:** G1 had been run and had failed. Only the forward profile passed, and
only at or below 0.2 m/s; the lateral profile failed the heading-drift criterion at every
speed measured, down to 0.06 m/s.

**Amendment.** `G1b_closed_loop_tracking` is added to `gates.yaml`. `G1_holonomic_fidelity`
is unchanged and its FAIL stands. Both results are reported in the thesis.

**The unicycle fallback moves.** `gates.yaml` states the fallback under `G1` ("unicycle
primary; holonomic becomes caveated ablation"). It now attaches to **G1b** instead: G1's
failure no longer triggers it. The text under `G1` is left as committed rather than edited,
and this amendment governs.

**Reasoning.** G1 measures open-loop fidelity: a constant command applied without feedback
for 5 m. Nothing in the system drives that way - the learned policy and both baselines
re-command every 50 ms. The proposal considered and **rejected** was to re-specify G1 as a
closed-loop test. Re-specifying a gate after it fails is indistinguishable, from outside,
from moving the goalposts. Adding a second gate and reporting both answers the same
question as evidence rather than as argument.

---

## A2 - 2026-09-18 - All gates require three repetitions

**Decided by:** supervisor, WP0 review.

**Amendment.** `repetitions: 3`, all of which must pass, applies to **every** gate. As
committed, `gates.yaml` states it only under `G1b_closed_loop_tracking`. G0 and G1 are
held to it by this amendment rather than by their own text.

**Reasoning.** Gate results to date are single runs. G0's two runs were bit-identical, so
determinism is established for that gate, but a single run cannot distinguish a stable
result from a lucky one, and at least one G1 measurement - forward at 0.2 m/s, 9.80 per
cent against a 10 per cent threshold - sat close enough to its limit that run-to-run
variation would decide it.

---

## A3 - 2026-09-18 - H6 added as an exploratory hypothesis

**Decided by:** supervisor, WP0 review.

**Amendment.** H6 is added to the hypothesis list, marked **exploratory**: the platform's
lateral motion coupling can be characterised as a bounded disturbance and carried
explicitly by a robust PFB.

**Status.** Exploratory, not confirmatory. It was formulated *after* the measurements that
motivate it, which is why it is not eligible for confirmatory testing on the test split.
The specification is in `docs/pfb_robust_spec.md`; no filter is implemented yet.

---

## A4 - 2026-09-18 - Physics configuration frozen before G1b

**Decided by:** supervisor, WP0 review.

**Amendment.** A time-boxed one-day probe of physics engine, timestep and substeps, roller
friction and wheel mass is run on the lateral profile **before** G1b. The best configuration
is adopted, frozen, and recorded. **No tuning after G1b runs.**

**Reasoning.** Tuning the physics after seeing a gate result would make the gate a
measurement of the tuning. Freezing first, and recording what was frozen, keeps G1b a
measurement of the platform. The probe's own results are a diagnostic, not a gate outcome.


**Outcome (2026-09-18).** Frozen at **ODE, dynamics step 0.005 s**, simulation step 0.05 s,
in `configs/base.yaml` under `sim.physics` and applied on every simulator start. Lateral
drift at 0.12 m/s: Bullet 20.55 deg, ODE 0.47, Newton 3.12, MuJoCo 160 (failed). ODE across
dynamics steps 0.005/0.002/0.001 s: 0.47/1.11/0.84 deg. Friction and wheel mass were not
tuned. G1 was measured on Bullet, G1b on ODE; the two results are therefore on different
physics, and G1's FAIL (A1) stands as the record of the configuration then in force.

---

## A5 - 2026-09-18 - `actuation.v_max` is an output of G1b

**Decided by:** supervisor, WP0 review.

**Amendment.** `actuation.v_max` is set by G1b's outcome - the highest speed passing every
profile in every repetition - and every dependent constant is updated together. Until G1b
runs, the configured 0.5 m/s is provisional and no result depending on it is citable.

**Dependents.** `actuation.vy_max`, `reward.time_cost`, `reward.max_steps`, the observation
scaling for the velocity channels, the PFB's stopping distance through entry speed, the
certified layer's speed bands, and G2's compute projection.

---
## A6 - 2026-09-18 - Direction-preserving clamp; G1b re-run; v_max and vy_max unset

**Decided by:** supervisor, D7/G1b review (rulings 1, 2 and 4).

**Finding that prompted it.** G1b as first run (`G1b_20260918T093210Z`, ODE) returned PASS
with v_max_from_gate 0.40 m/s. Its 0.50 m/s failures (`diagonal_45`, 75.9 mm peak;
`translate_while_rotating`, 62.2 mm peak) were caused by `clamp_velocity`, which clipped
each axis independently and so bent commands off their line; the lateral profile was never
commanded above `vy_max` = 0.30 m/s. That result stays on record and is reported. It does
**not** set v_max: it was produced under the defect.

**Amendment.**
1. `clamp_velocity` scales the translational pair (vx, vy) by a single factor, preserving
   direction; the yaw rate is clipped separately. Every binding of either is counted per
   run and reported.
2. The box is an interim limit. **The WP2 target is the mecanum wheel-speed polytope in
   (vx, vy, omega), imposed as constraints inside the PFB QP, not as a post-hoc clamp.**
3. G1b is re-run with a commanded-velocity box at or above every speed tested
   (`configs/gates/g1b_rerun.yaml`: v_max = vy_max = 0.50 m/s), so the clamp cannot decide
   the result. The script refuses to run if the box is below the speeds tested.
4. The lateral profile is additionally run at 0.35 and 0.45 m/s (`lateral_extra_speeds`),
   so with the grid it is tested at 0.10-0.50 m/s in 0.05 m/s steps from 0.30.
5. **Ceilings are lowest-first.** v_max is the highest grid speed at which every profile
   passes in every repetition *and every lower grid speed does too*; vy_max is derived the
   same way from the lateral profile alone. A pass above a failure is not a usable ceiling.
   This is stricter than the committed wording, never looser.
6. v_max and vy_max are set from the re-run, and D8 writes both, with every dependent
   constant, in one commit.

**Reasoning.** The clamp defect contradicted the acceleration limiter's own design, which
scales the step uniformly for the same reason. Fixing it after seeing G1b is acceptable
only because it is declared here before the re-run, and because the first result is kept.

---

## A7 - 2026-09-18 - G1 re-run on ODE as G1-ODE

**Decided by:** supervisor, D7/G1b review (ruling 5).

**Amendment.** G1 is re-run unchanged on the frozen physics (ODE, 0.005 s) and reported
as **G1-ODE**, alongside the recorded Bullet result. The Bullet FAIL stands (A1). Three
repetitions (A2). G1-ODE does not replace G1 and does not reopen A1's fallback decision,
which attaches to G1b.

---

## A8 - 2026-09-18 - Engine dependence reported; external-validity check on Newton

**Decided by:** supervisor, D7/G1b review (rulings 6 and 7).

**Amendment.**
1. Holonomic fidelity depends on the physics engine (lateral drift at 0.12 m/s: Bullet
   20.55 deg, ODE 0.47, Newton 3.12, MuJoCo failed). This is a reported finding. **Every
   figure, table and methods sentence states the engine.**
2. **Added to the evaluation plan: an external-validity check.** The final policies of
   every compared method are evaluated on the Newton engine without retraining, on the
   same test scenarios and with the same metrics as the primary ODE evaluation. It is
   reported as a secondary result; the primary comparison remains on ODE.
3. Wherever drift is reported, the 0.64 deg solver-sensitivity band (ODE at dynamics
   steps 0.005/0.002/0.001 s) is quoted with it; its non-monotonic trend is numerical
   noise.

---

## A9 - 2026-09-18 - v_max is a commanded ceiling; realised-speed map; H6 bound

**Decided by:** supervisor, D7/G1b review (ruling 3).

**Amendment.**
1. `actuation.v_max` is a **commanded ceiling**, not a speed the robot reaches. The G1b
   re-run publishes a measured realised-speed map (commanded against realised speed, per
   profile and speed), reported with every use of v_max.
2. The PFB's stopping distance D(s) and the certified layer's speed bands use the
   **estimated actual** speed, never the command. Using the command is a safety-relevant
   bug. (At the time of this amendment neither layer is implemented; the requirement is
   written into both interfaces and becomes an acceptance test of each.)
3. The speed shortfall is folded into H6's disturbance bound alongside the lateral
   coupling.
   
## A10 - 2026-09-18 - Dynamics-step probe upward; A4's freeze reopened for the step only

**Decided by:** supervisor, WP1 report (ruling 1); approved with additions (0.050 s
candidate, G0 tunnelling criterion, throughput per candidate, 20 Hz confirmed).

**Known at the time:** the training matrix (25 runs x 300k steps = 7.5M steps) does not
fit six weeks at the measured ~50 steps/s aggregate over four instances, before scanners,
policy and PFB exist. The physics step is ~47-59 ms of a ~70-80 ms step. D7 tested
dynamics steps 0.005, 0.002 and 0.001 s -- downward only -- and froze 0.005 s.

**Amendment.** A4's "no tuning after G1b" is reopened for **the dynamics step only**
(engine ODE, friction and mass unchanged). **The control period stays 0.05 s (20 Hz)**; each
control step runs n = 0.05 / dynamics_step physics steps. Candidates **0.005 (frozen,
baseline, n = 10), 0.010 (n = 5), 0.025 (n = 2) and 0.050 s (n = 1)**, all in one process.
A candidate passes if all three hold:

1. **G1b's committed thresholds** hold on G1b's committed grid (every profile at every grid
   speed, plus `lateral_extra_speeds`), one repetition (bit-identical within a process),
   **with the ceilings unchanged**: v_max and vy_max from the gate remain 0.50 m/s;
2. **G0, tunnelling:** G0's five positive cases are detected with **zero detection lag** at
   the candidate step, at the **highest tested approach speed, 0.50 m/s**, with the
   handle/collection validation agreeing on every step; and
3. **the drift band** holds: open-loop lateral heading drift at 0.12 m/s over 5 m (D7's
   measurement) differs from the frozen step's by at most the 0.64 deg solver-sensitivity
   band (`sim.physics.solver_sensitivity_drift_deg`).

**Recorded per candidate, to show whether a larger step helps as well as whether it is
safe:** physics ms per step (alone, and with four instances stepping concurrently), whole
step p95, and aggregate steps/s over four instances.

The **largest passing** candidate is adopted into `sim.physics.dynamic_step`, and **G1b,
G1-ODE and G0 are re-run on it** (three repetitions each, A2) and reported alongside the
0.005 s results, which stand. If no larger candidate passes, 0.005 s stays.

**Reasoning.** A larger dynamics step is the one physics lever that cuts the dominant
per-step cost without touching the controllers. Declaring the criteria before the probe,
and re-running the gates on the adopted step, keeps the change a measured decision rather
than tuning. The probe runs with the wheel budget in force (ruling 2, WP0 results), as
every candidate does, so candidates are compared like for like.

## Open, and not yet amended

- **G0 cannot reach PASS.** Its committed criteria require both a `gate_script` and an
  `env_step_loop` context. The second needs the WP1 Gymnasium environment, which does not
  exist. G0 currently reports INCOMPLETE with every `gate_script` case passing. No
  amendment has been made and none is proposed: the criterion is satisfiable, just not yet.
- **No gate result is citable.** Every run to date records `commit: HEAD, dirty: true`
  because the repository has no commits. D11 directs that this be fixed and the gates
  re-run; the commit is deferred at the researcher's instruction, so the problem stands.
  
---

## A11 - 2026-09-27 - G0 gains a sixth case: a raised obstacle meeting the sensor housing

**Decided by the supervisor on 2026-09-26**, ruling on Q5 of the 2026-09-26 weekly report.
**Made after seeing the result it affects**, and that is the fact a reader needs: G0 passed on
2026-09-26 with its five committed cases (`runs/gates/G0_20260925T185322Z_seed0` and the two
further invocations, all PASS, detection lag 0 in every positive case). This amendment does not
change that verdict and does not touch those five cases. It adds a sixth, whose result is not
yet known.

**What changed.** `G0_contact_validation.positive_cases` gains `raised_housing`, with its own
declared criteria; the five existing cases, the negative case, the approach speed, the lag
bound and the two contexts are unchanged.

**The case.** A slab spanning the robot's side, raised so its underside sits at **0.19 m** and
its top at 0.40 m, approached by a lateral (+y) translation at the committed 0.10 m/s. The
chassis and the wheels pass beneath it, so a scanner housing standing above them is what meets
it. Heights are the S300 layout read-back of 2026-09-22: wheel tops 0.0999 m, chassis top
0.1764 m, scanner housings spanning z 0.104-0.257 m, scan plane 0.2207 m. The underside clears
the chassis by 13.6 mm and sits inside the housings' span; the slab crosses the scan plane, so
this obstacle is **seen**, which is what distinguishes it from `wheel_edge`.

**Its declared criteria.** Detection lag at most 1 step against the geometric prediction, as
for every other positive case, **and** the shape struck must be a scanner housing
(`/youBot/SickS300[*]`), asserted by the same measured-placement mechanism the other cases use
(`expected_shapes` over densified mesh vertices). A case that strikes something else fails and
is reported, rather than passing as a gate on geometry nobody chose.

**What was already known at the time.** That the five committed cases cover the chassis, the
wheels, and the housing's front-outer *corner* (`pod_corner`), but not a raised obstacle
meeting a housing above the wheels; and that `wheel_edge` demonstrated a contact class the
certified layer cannot prevent at all, its slab sitting far below the scan plane. The new case
probes the opposite regime: geometry the scanners can see, on the part of the robot that stands
highest.

**Reasoning.** The gap was real and the researcher had waived it for the 2026-09-26 run
specifically to avoid an unrecorded amendment, which is why this file records it rather than a
commit message. The case is also now load-bearing in the other direction:
`scripts/gate_g0_contact.py` reconciles its case table against this file's `positive_cases`, so
a case defined in code but not declared here **does not run**, and a case declared here that
the script cannot build is a fatal error rather than a quiet pass on the remainder.

**G0 is re-run in full** on the amended criteria: three invocations, both contexts, all six
cases. The 2026-09-26 five-case PASS stands as the result for the criteria in force that day.

---

## A12 - 2026-09-28 - Endpoints, the confirmatory hypotheses, and the narrow-stratum rule

**Decided by the supervisor on 2026-09-27 and 2026-09-28** (ruling 5 on the narrow stratum, and
rulings D3-D5 on the before-the-freeze request). **Made before any policy has trained**: no
controller exists yet, so no method's behaviour can have informed it.

**What changed.** A new file, `preregistration/endpoints.yaml`, copied exactly from the
supervisor's consolidated text. It brings into the preregistration what had lived only in the
research plan: the confirmatory hypotheses **H1 and H2**, the conditional co-primary **H1N**, the
**Holm** family they form, the exploratory hypotheses H3-H6, the test (two-sided Wilcoxon signed
rank, paired by scenario), the unit of analysis, and the reporting rules. Until now the
preregistration locked the gate thresholds but not the hypotheses or the multiplicity correction,
which are the part of a study plan that most needs locking.

**Two clarifications ruled with it** (2026-09-28, D5):

1. **H1's comparators are B1-B4 and L1-L3.** B5 belongs to H5 and L4 to H2, and neither is an H1
   comparator.
2. **H1 and H1N use intersection-union logic**, with Holm applied **only across hypotheses**. Every
   comparison within H1 must reject at alpha, with no correction inside H1; the family Holm
   corrects over is {H1, H2}, widened by H1N only if the narrow rule triggers.

**The narrow-stratum rule.** Pilot the narrow stratum with **150 narrow episodes per method** on the
pilot seed range, **30 per seed** (five seeds). For each method in the confirmatory comparisons --
**B1, B2, B3, B4, L1, L2, L3, L4, L5**; A1 is excluded as exploratory -- compute the **exact one-sided
95 % Clopper-Pearson upper bound** on the proportion of `unrecoverable_stop` outcomes. If that bound
is **at or above 0.02 for any listed method**, H1N joins the confirmatory family as a co-primary
(narrow completion rate, with distance to first stop as its secondary). **At n = 150 the rule passes
only with zero events**: zero in 150 gives an upper bound of 1.98 %, one in 150 gives 3.12 %. The
pilot runs after the methods exist and before any test episode is evaluated.

**Why an upper bound rather than the observed proportion.** The two errors are not symmetric.
Adding a co-primary that was not needed costs some power; withholding one that was needed leaves a
compressed primary unprotected, which is the failure the rule exists to prevent. A pilot of 150 at
the observed proportion would have kept the sole primary with 2 events, which cannot exclude 4.14 %.

**Why any method rather than the pooled rate.** Endpoints must be identical across methods, and one
method's unrecoverable stops must not be averaged away by the others'.

**What was already known at the time.** That the at-rest field had been corrected to the infimum of
the moving fields (2026-09-27), removing the narrow-aisle deadlock in geometry; that the
traversability precondition passed on the real scan with an unavoidable-stop floor of **0.0 per
100 m in every stratum**; and that D2 re-measured 69.5 mm of margin at rest and 19.4 mm at the
tracking bound. On that evidence an unrecoverable stop needs something the precondition does not
model -- an unmapped obstacle, or a controller that stops with an intrusion inside the minimum
margin -- so the rule may well not trigger. It is declared anyway, because whether it triggers is a
fact about the methods, and the methods do not exist yet.

---

## A13 - 2026-09-29 - Route-relative unmapped obstacles; the tune and pilot sets

**Decided by the supervisor on 2026-09-29** (rulings on the scenario-sets request). **Made before
any scenario set exists and before any policy trains.** The measurements that prompted it were
taken on pilot seeds only; no test seed has been generated.

**What changed.** A new file, `preregistration/scenario_sets.yaml`, and one change to the frozen
scenario module, `pfbnav/world/scenarios.py`, which is why the lock moves with this amendment.

**Placement.** Unmapped obstacles were placed uniformly over the hall's free floor before any route
existed. Measured on 40 pilot scenarios per stratum at `unmapped_static`, only 8-15 % put any
obstacle within 0.3 m of the route (narrow 15 %, medium 8 %, wide 12 %). H1's entire population is
`unmapped_static`, so roughly 88 % of it would not have tested the manipulation its condition is
named for.

Placement is now **route-relative in every split** -- train, tune and test -- so training sees the
distribution it is tested on. After the route is planned, each obstacle's offset to its nearest
**surface**, never its centre, is uniform on [0, 0.6] m; its side is uniform left or right; its
position is uniform over the route's arc length, excluding the first 2 m and the last 1 m. An
obstacle is kept only if the goal remains reachable in the true world -- mapped plus unmapped -- at
the lethal inflation radius. A refused obstacle is **re-sampled, never the layout**; if a route
cannot hold its obstacles within 200 draws, the next endpoint pair of the same layout is tried. The
rejection rate is logged per stratum and by cause. Because placement now depends on the route, each
scenario records its unmapped obstacles' geometry, and its `layout_sha256` covers them.

**The validity check.** At least 90 % of `unmapped_static` test scenarios must have an obstacle
surface within 0.3 m of the planned route, verified by a dry run before any test set is locked; the
build script refuses to lock a set that fails it.

**Option (c) was rejected** -- keeping uniform placement and discarding scenarios whose obstacles
landed far from the route -- because it would select layouts, not only obstacle positions.

**The tune set.** 20 scenarios per stratum, at the tune rate of 0.25, with route-relative placement.

**The pilot.** 150 **distinct** narrow scenarios from the pilot seed range, at `unmapped_static`.
Each learned method runs scenario *i* with training seed *i* mod 5, so 30 scenarios per seed and
150 distinct scenarios per method; each classical baseline runs all 150 once. This corrects the
construction implied by "30 per seed": deterministic baselines on 30 scenarios would give only 30
episodes, and a learned method's episodes sharing a scenario are clustered, which the
Clopper-Pearson bound -- a bound on independent trials -- does not allow.

**Not in this amendment.** The test-set size is **A14**, fixed after G2 has run on the training host
(on pilot seeds) and after the Monte Carlo of H1's power. The order is recorded in
`docs/decision_log.md`.

**One inconsistency, resolved before commit (supervisor, 2026-09-30).** As first ruled, every
offset is uniform on [0, 0.6] m, which engages only 62.5 % of scenarios with 1-2 obstacles
(1 - (0.5 + 0.25) / 2) -- below the 90 % check by construction. The two parts of the ruling had
not been checked against each other. Resolved by option (a): **the first obstacle's offset is
uniform on [0, 0.3] m** (`first_obstacle_offset_max_m: 0.3`) and any further obstacle's on
[0, 0.6] m, so every scenario is engaged and the 90 % check stands. Two conditions:

* **"First" is an index, not a location.** The capped obstacle's along-route position, side and
  size are drawn exactly as for the second obstacle, independent of the cap; otherwise close
  obstacles would sit systematically at one end of the route. A known-answer test pins this
  (with the same random stream, the cap changes the offset and nothing else), and the dry run
  reports where each index came to rest, since feasibility refusals can still shift it.
* **The offset analysis is descriptive and exploratory**, reported against the per-scenario
  minimum surface offset, and is not a confirmatory endpoint. Its distribution is shaped by
  design: the cap guarantees one obstacle within 0.3 m.

The same rule applies in train, tune and test.


### A13, revised 2026-10-02 - Feasibility: a local bypass at the narrow floor, not reachability

**Decided by the supervisor on 2026-10-02**, after the first dry run of A13 and before any set is
locked or any policy trains. Measured on tune and pilot seeds only. This revision **replaces**
the feasibility rule stated above ("kept only if the goal remains reachable in the true world");
everything else in A13 stands.

**Why.** The stack plans once, on the mapped map, and never replans. "Reachable in the true world"
was met by detours through other aisles, which the robot cannot take. Planning in the true world
confined to a corridor around the route, at the lethal radius, 29 % of pilot scenarios (43/150)
had no way past their obstacles within +-1.5 m of the route, and 62 % within +-0.75 m; the same
test on the mapped world alone passed in 150/150. In such a scenario every controller stalls or
takes a stop it cannot avoid.

**The rule.** An unmapped obstacle is kept only if, together with every unmapped obstacle already
placed (judged **jointly**: two individually passable obstacles can close a gap together), the
robot can get from start to goal:

* keeping its centre at least `half_width + 0.13 m` from every surface -- the **narrow-stratum
  floor** beyond the footprint, the same admissibility test as any aisle. The lethal radius
  (0.03 m beside the robot) would put the obstacle inside the 0.05 m abeam protective field, a
  stop no controller could avoid. The abeam margin is constant across speed bands, so the floor
  admits the bypass at any band;
* never further than **W = 1.0 m** from the planned route: local avoidance within the corridor
  the route uses, not a detour through another aisle.

A refused obstacle is re-sampled, never the layout. **Fallback, declared before the measurement:**
W = 1.0 m stands unless a tune (20 per stratum) or pilot (150) set cannot be built in full within
the re-sampling cap; only then W = 1.5 m. **It was not needed** (below).

**How it is decided.** A grid of cell centres over the corridor at 0.01 m; the exact distance from
each to every polygon; a cell is admissible at `half_width + 0.13 - 0.005` m; a bypass exists if
start and goal share an 8-connected component (`pfbnav/world/bypass.py`). The half-cell allowance
lets a gap of exactly the floor through at any alignment of the grid -- without it the narrowest
mapped aisles fail the control. Its cost: a gap up to 5 mm per side below the floor may be
admitted, out of the floor's deliberate 0.03 m buffer. A dense sweep (0.115-0.145 m per side in
1 mm steps, five grid alignments) confirms both bounds. **Positive control:** before any obstacle
is placed, each route's empty corridor must pass the same test.

**Measured cost** (dry runs at commit `4384969`, seed 0; tune sets at rate 0.25, and 150
`unmapped_static` scenarios per stratum -- `pilot_narrow` and two probes, `probe_medium` and
`probe_wide`, from pilot seeds 9000150-9000449):

| W (m) | sets built in full | lost to placement | draws per obstacle, narrow / medium / wide | refused for no bypass, % of draws, narrow / medium / wide |
|---|---|---|---|---|
| 0.75 | all | 0 | 87 / 72 / 6 | 6.9 / 14.8 / 28.2 |
| **1.0** | **all** | **0** | **50 / 35 / 4** | **3.6 / 10.8 / 18.6** |
| 1.5 | all | 0 | 40 / 29 / 3 | 2.5 / 9.5 / 15.0 |

Most refusals at every W are `too_close_to_mapped` (the preregistered 0.15 m margin). The positive
control set aside no route in any set at any W. At W = 1.0, at least 12 of 150 narrow and 6 of 150
medium scenarios moved to another endpoint pair because their first route could not hold their
obstacles (a lower bound: scenarios whose draws exceeded 200 per obstacle); none in wide.

**Where obstacles land** (W = 1.0; inside an aisle between two shelf rows / at an aisle's mouth,
within 1.2 m of its end / on open floor):

| stratum | obstacles | aisle | mouth | open floor |
|---|---|---|---|---|
| narrow | 225 | 0 % | 3 % | 97 % |
| medium | 221 | 4 % | 17 % | 79 % |
| wide | 228 | 54 % | 7 % | 39 % |

In the narrow stratum no obstacle can sit inside an aisle: the smallest obstacle (0.25 m), its
0.15 m margin to the shelf behind it, and a bypass at the floor (2 x 0.3386 m) need 1.08 m, while
the widest narrow aisle is 0.82 m. **"An unmapped obstacle in the narrow stratum" therefore means
an obstacle on the open floor of a narrow-aisled layout**, met on the way into or out of the
aisles, never within them. Results state this.

**Limitation, recorded as ruled.** The test population contains no route blockage that requires
replanning. A pallet left across an aisle the robot was sent down is a real warehouse case; it is
outside this study's scope, and global replanning on detection is the future work that addresses
it.


### A13, revised 2026-10-03 - The training set, and the tune-unmapped measurement set

**Decided by the supervisor on 2026-10-03**, before any policy trains. `preregistration/
scenario_sets.yaml` gains two blocks under `scenario_sets`; nothing else in A13 changes.

**The training set.** Equal thirds by stratum, 667 scenarios each (2,001 in all), from the train seed
range at the preregistered train rate of 0.25, with A13's route-relative placement and bypass rule.
**Training seed k visits the whole set in its own permutation, and every method's seed k uses that
same permutation**: seeds carry all training randomness, including order, or the method-level
inference -- seeds as the replicates of a method -- is miscalibrated. **No curriculum in v2.** The
permutation is a function of k alone (`pfbnav/training/order.py`), and every training run records
its hash.

**The tune-unmapped set** is disposable: 20 scenarios per stratum from the tune seed range at an
unmapped rate of 1.0 -- the H1 population's -- for the measurement the extended decision tree plans
on (Step 0): the seed CV, the stop rate and the inter-comparator gaps, at conservative 80 % bounds.
The tune sets' own rate of 0.25 would dilute the stop rate relative to `unmapped_static`. It is
allocated after the existing tune sets, so no tune seed moves, and its building is logged as a tune
regeneration.


---

## A15 - 2026-10-03 - Gate G4: the learned-method pilot

**Decided by the supervisor on 2026-10-03**, before any policy trains. (A14 stays reserved for the
test-set size, as ruled on 2026-09-29; this amendment is numbered after it so that reference holds.)

**What changed.** `preregistration/gates.yaml` gains `G4_learned_pilot`, a generic gate on the
pilot seed of every learned method -- L1, L2, L4, L5 and A1 -- run before that method's
confirmatory seeds. It decides whether the training pipeline works well enough on the frozen
environment to spend confirmatory seeds on it; it makes no performance claim.

**Criteria, all of which must hold.** (1) **Integrity:** the run reached its total timesteps and
wrote its final model; it records the frozen environment identity; no logged loss is NaN or
infinite; the episode log is complete; no environment exception ended or skipped an episode.
(2) **Learning**, on the training episode log: success over the last 10 % of episodes at least 0.20,
and at least 0.15 above the first 10 %. (3) **Checkpoint**, the deterministic policy once on each of
the 60 tune scenarios (rate 0.25): success at least 0.30 and ground-truth contact in at most 10 % of
episodes. (4) **Recorded, not gated:** steps per second, reset wall clock, protective stops per
100 m, the outcome distribution and per-stratum success. The draft's "no single outcome above
90 %" was dropped: it would fail excellent policies, and the success floor already excludes a
policy that stands still.

**Controls, required before any pilot verdict is read:** a zero-action policy must FAIL criterion 3
(all timeouts, no successes), and holonomic Pure Pursuit tracking the planned path -- which is also
baseline B1 -- must PASS it. If either misbehaves, the gate is reported as broken and no verdict is
read.

**Failure handling.** An integrity failure is fixed and a fresh pilot seed runs; the failed run is
kept. A learning or checkpoint failure with integrity intact gets one second pilot seed, unchanged:
if it passes, G4 passes and the first failure is recorded for the seed-CV discussion; if it fails,
stop and report. Any hyperparameter change after a G4 failure applies identically to every learned
method and restarts the pilot.

**Selection, a standing rule.** Pilot seeds come only from the pilot range (9000000, 9000001) and
never enter H1 or H2. After G4 passes, every confirmatory seed trains unconditionally and stays in
the analysis whatever its outcome.


---

## A16 - 2026-10-15 - Comparators: B4 and B5 retired; B6 (field-aware MPPI) and B7 (RPP + PFB) added

**Decided by the supervisor on 2026-10-15**, before WP3 baseline implementation and before any test
evaluation, so each controller is implemented and tuned once. (A14 stays reserved for the test-set
size.)

**Why.** Two gaps in the comparator set, raised before any baseline exists. (1) No classical
controller knew the certified fields, so H1 could not answer "could a hand-designed controller that
avoids the fields do as well without learning?". (2) The attribution analysis -- does the PFB on a
classical controller give the same benefit as training with it? -- used plain Pure Pursuit, not the
strongest reasonable nominal controller.

**What changed.**
- **B4 ("PP + field switching only", the industrial status quo) is retired.** With the certified
  layer beneath every controller it is identical to B1: a design error. **B1 carries the "industrial
  status quo" label** -- a path tracker relying on the safety scanners.
- **B5 (PP + PFB, no learning) is retired**, superseded by B7.
- **B6 = holonomic MPPI with a field-aware cost**, an H1 comparator in B4's place. The cost uses the
  certified layer's actual discrete band fields, not the PFB's continuous field: the hand-designed
  approach is to avoid triggering the fields that stop you, and the PFB field would make MPPI a soft
  PFB. **Declared now:** cost terms -- route tracking, goal progress, a field-breach penalty, hard
  rejection of rollouts intersecting scan points, control smoothness; horizon, sample count and
  temperature (initial values in `configs/base.yaml`, `baselines.mppi`). All tuned under the
  equal-tuning-budget rule on the tune set only. The same route, certified layer, 120-bin scan stream
  and classical recovery as every baseline. Implemented in our stack with vectorised rollouts, as DWA
  is; Nav2 is not run.
- **B7 = Regulated Pure Pursuit + PFB, no learning**, in B5's place: in H5 (exploratory) and in H3's
  granularity study. **H5 is extended to report protective stops as well as mission time**, since
  attribution concerns both.
- **Standing rule: method IDs are never reused** (`configs/methods.yaml`, CLAUDE.md hard rule 20).
  B4 and B5 were never implemented, but frozen documents name them; redefining them would make the
  record ambiguous.

**The classical set after A16**, from the research plan's experimental matrix (B2 and B3 confirmed by
the researcher, 2026-10-15): **B1** holonomic Pure Pursuit, the industrial status quo; **B2** Regulated
Pure Pursuit, Nav2-style, slowing on curvature and near obstacles; **B3** holonomic DWA with a
three-dimensional dynamic window over (vx, vy, omega); **B6** field-aware MPPI; **B7** RPP + PFB. B4
and B5 are retired and their IDs never reused. **All of them share** the same route, the certified
layer beneath, the 120-bin scan stream, the non-square actuation box with the wheel-budget
projection, the classical recovery behaviour, and an equal tuning budget on the tune set only.

**In `preregistration/endpoints.yaml`.** H1 and H1N comparators `[B1, B2, B3, B6, L1, L2, L3]`;
`narrow_rule.methods` `[B1, B2, B3, B6, L1, L2, L3, L4, L5]`; H5 = L5 versus B7, mission time and
protective stops.

**Power.** B6 enters the power analysis fully. H1 keeps seven comparators, so its structure and the
power analysis's shape are unchanged. Step 0 measures the inter-comparator gaps on the tune-unmapped
set including B6, so B6 must be implemented and tuned before that measurement (which waits on trained
learned seeds anyway). Making the strongest comparator secondary to protect power was considered and
rejected: it would leave the objection unanswered, and it is the scope-narrowing move branch 5
forbids. B6 is expected to become the closest comparator, so the 30 % effect of interest will be
measured against it.

**Not affected:** G4, whose positive control is B1.


---

## A17 - 2026-10-15 - H4: a directional expectation, and A1's floor from the tangent-locked oracle

**Decided by the supervisor on 2026-10-15**, before any H4 data exists (A1 has never been trained or
evaluated). H4 stays exploratory, outside the Holm family.

**What changed.** H4 (holonomic L5 versus unicycle A1, deadlocks and protective stops) gains a
directional expectation, preregistered now: **A1 shows a materially higher narrow-stratum protective
stop rate than L5.**

**Why it is expected.** The route-level oracle that sets the planner's margins may choose its body
orientation: a mecanum base can travel at any heading to its body. Under the tangent-locked oracle,
whose body always faces the route as a unicycle's must, 16.2 % of narrow routes had a band-0 breach
(soft margin 0.10). With a free orientation the same routes cleared, 142 of 143 with one body offset
held through each stretch (the 2026-10-15 diagnostic; the formal free-orientation run follows).
Narrow aisles are where holonomy should matter.

**A1's own floor.** The per-route unavoidable-stop floor is platform-specific: a holonomic base with
full knowledge of its route, for L5 and the holonomic baselines. A1's floor is computed with the
tangent-locked oracle and reported beside L5's whenever H4 is reported.

**In `preregistration/endpoints.yaml`.** The exploratory H4 line names the expectation and A1's floor.

---

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

---

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

---

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

---

## A21 - 2026-10-19 - Nesting for stops begun in motion; a rigid-body restart condition; the narrow floor by rule

**Decided by the supervisor on 2026-10-19** in two rulings, both recorded in `docs/decision_log.md`
before any change. No test layout was built, read or used. No method has been trained or evaluated.

**Erratum to A20:** A20's text names its narrow-floor run as `runs/analysis/NARROW_FLOOR_A20_20261019T221401Z_seed0`. The
run is `runs/analysis/NARROW_FLOOR_A20_20261018T202400Z_seed0`.

**Why.** A20's containment test (2026-10-19) held containment at every stop, but a spinning stop
ended 3.54 mm inside the minimum margin.
- The at-rest field the robot stops in carries its own reach, so after a turning stop it reached
  beyond the field in force at the trigger.
- A stop could create its own deadlock in a field that was clear when it triggered.
- The restart condition ignored rotation entirely.

**What changed.**

1. **Nesting (S2).** Every stop that begins from a MOVING state (translating or spinning) ends with
   the at-rest field inside the field in force at the trigger, at every reachable rest pose.
   - **The moving field carries:**
     - the at-rest field's own rotational reach on top of the reachable turn, so the yaw interval
       is +-(e(|omega|) + e(0));
     - at least the at-rest reach in every direction;
     - a term the exhaustive nesting test showed necessary, the drift of a body that turns while it
       translates, D(V) * 2 sin(e(|omega|) / 2) (a holonomic base's command turns with the body).
   - **At-rest states are excluded.** In a static world a stationary robot with a clear at-rest field
     cannot be triggered by static obstacles. An at-rest trigger needs something entering the field
     (dynamic, out of scope), where holding the latch is correct.
     - A counter of at-rest triggers is kept in every static-world run (G4 controls, pilots,
       evaluation). Expected zero; any occurrence is reported as a construction gap.
   - **At-rest classification needs both the estimate and the applied command below the standstill
     threshold, rotation included.** The rotational thresholds are the translational ones over the
     footprint's outermost radius, so a spin is a moving state.
   - **The supervisor's first wording was impossible, and the error was the supervisor's.** It would
     have required the at-rest field, which carries the reach from rest, to nest inside itself
     shifted by that reach.
   - **S1 was rejected:** releasing on a standing field leaves the obstacle in the reach shell, and
     any first move re-triggers.
2. **The restart condition in rigid-body form.** A restart is permitted only if BOTH rules permit it:
   - **the centre rule:** v . p <= 0 for every breaching point, along its bearing;
   - **the closest-point rule:** (v + omega x q) . (p - q) <= 0, with q the footprint point nearest
     p, in the leading field for the commanded twist (its rotation sweep included).
   - It is never more permissive than the 2026-09-28 rule, and stricter in about 2.5 % of random
     translation cases, each a genuine sideways approach. This supersedes "reproduces the current
     rule exactly".
3. **The narrow floor by its rule.**
   - F = the smallest per-side clearance with all three D2 gaps >= 0 (binned) under the field
     above, + 0.03 m: **0.1687 + 0.03 = 0.1987 m.**
   - **The strata: narrow [0.1987, 0.2687], medium [0.2687, 0.40], wide [0.40, 0.80].**
   - The ruling named 0.1926, computed before the coupling term.
   - Run: `runs/analysis/NARROW_FLOOR_A20_20261019T221401Z_seed0` (the script keeps its A20 name).

**In the preregistration** (copies of the frozen files with only these edits, to apply verbatim):
- `endpoints.yaml`: H2's population becomes "narrow (0.1987-0.2687 m per side; A21)".
  File: `docs/pending/endpoints_A21.yaml`.
- `scenario_sets.yaml`: the bypass clearance's stated value becomes 0.1987 m (read from the strata;
  only the text changes). File: `docs/pending/scenario_sets_A21.yaml`.
- **`scenarios_sha256` changes** (`certified.py` and `strata.py` are in the generator's closure),
  and the strata change the narrow and medium layouts, so **the sets are REBUILT.** The lock is
  renewed (`--relock A21`).

**Checks, in the ruled order:** the rebuild and the oracle residual check; the containment test in
the simulator, with its margins-removed control declared before running; then the G4 controls.

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

## A23 - 2026-10-20 - G4's positive control is the privileged oracle tracker; the episode horizon re-derived; the classical route cap; L5-nowp declared

**Decided by the supervisor on 2026-10-20**, recorded in `docs/decision_log.md` before any change.
No test layout was built, read or used. No method has been trained or evaluated.

**Why.** Emulated on the current tune routes, B1 (holonomic Pure Pursuit, G4's positive control
under A15) deadlocks at its first protective stop on 3/3 routes.
- The whole-field restart (A22) judges the field of B1's fixed command, which never clears.
- Both oracles pass the same routes with zero stops.
- No environment defect is identified. This is the case pre-declared on 2026-10-17.

**What changed.**

1. **G4's controls** (`gates.yaml`, `G4_learned_pilot.controls`):
   - **The positive control is the privileged oracle tracker** (`oracle_tracker_privileged`). It
     tracks the mapped route with feedback, at the speed and body yaw of the free-orientation
     oracle's schedule computed over EVERY obstacle, mapped and unmapped. It must PASS the
     checkpoint criterion.
   - **B1 with the recovery**, under the declared route cap, is evaluated in the same session and
     RECORDED as a baseline finding, never judged. A recovery attempt the emulator refuses still
     stops the gate (ruled 2026-10-16).
   - Zero-action is unchanged: it must FAIL with all timeouts.
2. **The episode horizon, `reward.max_steps`, is 4000 steps (200 s); it was 1500 (75 s).** This is a
   CORRECTION: an A5 dependent of v_max never re-derived after A20.
   - **The derivation rule:** the horizon covers the privileged oracle schedule of every traversable
     built route, x 1.3 margin. It is checked over every built set, and over the test sets once
     built (`tests/test_horizon.py`).
   - `time_cost` per step is unchanged.
   - **Knock-on effects:** fewer episodes per seed at 300k steps (if L1's pilot fails G4 on
     learning, `total_timesteps` is the first lever, not the reward), and a larger evaluation step
     budget, re-costed in G2 and in A14.
3. **The yaw-rate allowance of the oracle schedule** is the largest yaw rate keeping band 0
   admissible, rounded down to 0.05 rad/s, per route.
   - It is 0.20 rad/s on every tune route: 60/60 band-0 traversable at 0.20, 54/60 at 0.25.
   - Some training routes need 0.15 or 0.10.
4. **Every classical baseline (B1, B2, B3, B6, B7) carries the same declared route cap.**
   - At each point of the route, the speed cap is the fastest band whose field is clear of the
     MAPPED obstacles, with a yaw-rate cap at bends. It is computed offline by the oracle restricted
     to the map, then anticipated and deceleration-limited.
   - It is non-privileged. Controllers may go slower, never faster.
   - Learned arms do not get it; the methods state the asymmetry.
5. **Declared: L5-nowp, an exploratory planner-contribution analysis.**
   - L5 without the planner: the lookahead channels removed, a straight-line progress reward.
   - 5 seeds, the same seed permutations as L5, outside the Holm family, reported whatever it shows.
   - Trained after every confirmatory arm, and the first arm cut if throughput falls short.
   - v1 is the pilot study motivating it.
6. **Recorded, not fixed (the freeze of 2026-10-20):** an ideal tracker averages about 0.12 m/s under
   A20-A22.
   - The fields are more conservative than typical industrial configurations, especially in
     rotation (`docs/realism_audit.md`).
   - The oracle mission time is a reference line in the results.

**Also verified before G4, as ruled.** The last rebuild's A13 bypass rule used the current narrow
floor by reference: radius = half_width 0.2086 + narrow min_clearance 0.1987 = 0.4073 m (run
`SCENARIO_SETS_20261020T073343Z`, c748dc2). Every one of the 17 tune routes the privileged schedule
cannot traverse (each with an unmapped obstacle across the mapped route) has a bypass at that floor.
Only a docstring in `bypass.py` still says 0.13 m. That file is in the scenario closure, so the text
is left.

**In the preregistration:** only `gates.yaml` changes (the controls block), as in
`docs/pending/gates_A23.yaml`. No generator or scenario code changes, so `scenarios_sha256` is
unchanged and nothing is rebuilt. The lock is renewed (`--relock A23`).

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

## A25 - 2026-10-23 - Scan faults: the plausibility guard; H1 excludes scan-fault stops; G2's over-budget criterion

**Decided by the supervisor on 2026-10-23**, in two rulings recorded in `docs/decision_log.md`
before any change. No test layout was built, read or used. No method has been trained or evaluated.

**Why.** The zero-action G4 control had one stationary trigger (2026-10-22, seed 2000018). The traced
rerun, declared before running, reproduced it only under induced heavy load: 0/20 normal, 1/20
loaded, and the load control passed.
- The trace shows, with the robot still, two consecutive vision-sensor renders corrupted under CPU
  starvation: 104-114 of 120 bins empty (the walls gone), and phantom returns at 0.30-0.47 m near
  the scanner corners.
- These were fresh packets of the current tick, so the staleness check passed them.
- A corrupted render can create a stop or hide an obstacle.
- **It is blocking** because the fault is load-dependent and the PFB adds per-step compute: unfixed,
  it would bias H1 against L5.

**What changed** (environment code; re-freeze 9 with the filter-off bit-identity test):
1. **The plausibility guard** (`pfbnav/sim/scan_guard.py`). Each bin must lie within the range
   spanned by the last accepted scan's bins within +-2 of it, +-6 cm, both widened per step since
   that scan. 6 or more bins outside means a corrupt packet.
2. **Up to 2 re-renders of the same tick** (`rescan` in `lua/scanner_v2.lua`; the stamp is kept).
3. **The fail-safe:** if still corrupt, every bin is unobserved for that step, counted as a
   `scan_fault`.
   - A stop decided on such a packet is a `scan_fault_stop`.
   - It is kept out of the at-rest and stationary (construction-gap) counters.
   - The ring buffer dumps on every scan fault.
4. **Verification** (the 20 loaded runs WITHDRAWN: at a base rate near 1/20, zero events in 20 would
   occur about 36 % of the time without a fix):
   - a deterministic fault injection replaying the two recorded corrupted packets through the read
     path (both flagged, re-rendered, fail-safed, no stationary trigger);
   - the false-alarm regressions (198 recorded good steps, 3,196 emulated moving steps);
   - the re-freeze 9 bit-identity test;
   - in the loop, L1's pilot reports `scan_faults` and any ring dumps.

**In the preregistration** (copies with only these edits, to apply verbatim):
- **`endpoints.yaml`, H1:** stops decided on a scan-faulted packet are excluded from the primary stop
  count and reported alongside it per method. `scan_faults` are reported prominently if they differ
  materially between methods. File: `docs/pending/endpoints_A25.yaml`.
- **`gates.yaml`, G2:** the selected instance count must produce essentially no over-budget steps,
  with the over-budget count recorded per run. A run with more than 0.1 % of its steps scan-faulted
  is flagged, and that host configuration runs fewer instances. File: `docs/pending/gates_A25.yaml`.
- No generator or scenario code changes: `scenarios_sha256` is unchanged and nothing is rebuilt.
  The preregistration text changes, so the lock is renewed (`--relock A25`).

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

## A14a - 2026-10-26 - The statistical analysis; the PFB gain's grid and selection rule

**Decided by the supervisor:**
- the statistical analysis on 2026-10-02 and 2026-10-03 (rate ratio, two-level bootstrap, the
  correction, the degrees of freedom, the decision tree);
- the alpha grid and its selection rule on 2026-10-24;
- the instruction to preregister both, as A14a, before any tuning, on 2026-10-26.

All are recorded in `docs/decision_log.md`. **A14b** (the seed count and the test size) follows,
from the decision tree after Step 0 and G2. No test layout was built, read or used. No method has
been trained or evaluated.

**Why.** `endpoints.yaml` still specifies a two-sided Wilcoxon signed-rank test, paired by scenario,
with learned methods seed-averaged over five seeds and a paired detectability floor in H1's success
criterion.
- The H1 power analysis (2026-10-02) found that this test leans against a seed-averaged L5 when the
  comparator runs once.
- It also found that shared seed effects inflate learned-versus-learned null rejections.
- The supervisor ruled its replacement, and the replacement passed calibration (2026-10-03).

**What changes.**

1. **H1's estimand: the rate ratio.** For L5 over a comparator: (L5's total protective stops / L5's
   total distance) / (the comparator's total stops / its total distance).
   - Scan-fault stops are excluded (A25).
   - 0.5 is added to each arm's total stops (a declared continuity correction).
2. **The interval.** A two-level bootstrap of the log rate ratio:
   - scenarios are resampled jointly across arms (the pairing);
   - a learned arm's seeds are resampled within the arm, independently of every other arm's;
   - a classical arm resamples scenarios only.

   Its variance is computed in closed form for the linearised statistic (`pfbnav/eval/rate_ratio.py`;
   checked against brute-force resampling, and against the full bootstrap within -2 % to +7 %).
   - **Correction:** each learned arm's seed part of the variance is scaled by K/(K-1), and the
     interval uses a t quantile.
   - **Learned against classical:** df = K - 1 of the learned arm (flat; adopted 2026-10-03 after
     passing calibration over the whole grid).
   - **Learned against learned:** Welch-Satterthwaite df over the scenario part (n - 1) and each
     seed part (K - 1).
   - **Declared fallback, not in force:** seeds as units (per-seed log rate ratios against the
     classical arm over the same scenarios, a one-sample t with K - 1 df). If that fails
     calibration too: stop and report.
   - A comparison rejects in L5's favour when the two-sided interval at the comparison's Holm level
     lies wholly below 0 on the log scale.
3. **The confirmatory claim is method-level** (the two-level interval). The policy-scoped interval
   (seeds held fixed) is reported as secondary, never substituted.
4. **H1's logic is unchanged:** intersection-union over B1, B2, B3, B6, L1, L2, L3. Every comparison
   must reject; there is no correction within H1.
   - **The detectability floor is removed from H1's success criterion.** It is redundant given a
     valid test, and kept as a reported quantity: the 95th percentile of the effect under
     within-scenario arm-label swaps.
5. **H2 inherits the analysis.** The estimand is the narrow-stratum success-rate difference, L5 minus
   L4, with the same two-level corrected interval on the mean; L4 and L5 train at the same seed
   count, so it is Welch-Satterthwaite. H2's success criterion is unchanged: a difference of at
   least +15 pp, and significant after Holm.
6. **Family and level:** H1, H2, and H1N if the narrow rule adds it, under Holm's step-down at
   familywise alpha = 0.05. Power is planned at the most stringent step, alpha/3.
7. **The decision tree** (2026-10-03, extended) decides A14b's seed count and test size, and is
   preregistered here as written in the decision log:
   - **Step 0 (planning inputs):**
     - measured on the disposable tune-unmapped set: the stop rate at its lower 80 % bound, the seed
       CV at its upper 80 % bound, and the comparator gaps at the bound nearer the margin;
     - the effect of interest stays a 30 % reduction against the closest comparator;
     - L5's own measured effect is never a power input.
   - **Branch 1:** corrected, 10 seeds, power >= 0.80 at <= 100 per cell gives 10 seeds.
   - **Branch 2:** else, a count <= 20 reaching 0.80, if G2 fits.
   - **Branch 3:** else, H1's method-level claim is restricted to the classical comparators
     (B1, B2, B3, B6; B4 retired by A16), L1-L3 are reported policy-scoped, and H1 is amended.
   - **Branch 4:** sizes {150, 200} per cell for the unmapped-static cells only, subject to G2.
   - **Branch 5:** if nothing affordable reaches 0.80:
     - H1, its effect size and the analysis stay unchanged;
     - the achieved power is preregistered in A14b;
     - a non-significant result is inconclusive;
     - the rate ratio with its interval is the primary result.
   - **H2:** stays method-level at the chosen seed count, with its achieved power preregistered.
   - **Seed count:** L5's count is the larger of H1's and H2's requirements, capped at 20, subject to
     G2. L4 trains at the same count.
8. **The PFB gain alpha** (ruled 2026-10-24), tuned before L5's pilot and then frozen for L3, L4,
   L5 and B7:
   - **Grid:** {1, 2, 5, 10, 20} 1/s.
   - **Comparator:** B7 (B2's tuned parameters plus the PFB, nothing else).
   - **Set:** the tune set at the training unmapped rate.
   - **Selection:** among the settings whose success rate is within 5 percentage points of the
     best, the fewest protective stops per 100 m; ties broken by mission time.

**In the preregistration:** `endpoints.yaml` only (its `test`, `unit`, `reporting`, H1's `success`
and an `analysis` and `tuning` block), as in `docs/pending/endpoints_A14a.yaml`. `gates.yaml` is
unchanged. No generator or scenario code changes, so nothing is rebuilt. The lock is renewed
(`--relock A14a`).

## A27 - 2026-10-26 - The contingency hypothesis H1prime and the pivot to the filter-only path

**Decided by the supervisor on 2026-10-26** ("the path to primary results"), recorded in
`docs/decision_log.md`, with the instruction to preregister the contingency hypothesis before B2's
tuning starts. No test layout was built, read or used. No method has been trained or evaluated; no
tuning candidate has run.

**Why.** The project must produce a defensible primary result whether or not reinforcement learning
succeeds. Evidence therefore comes first from the filter on a classical controller (B7 = B2's tuned
parameters plus the PFB), which carries no learning risk, while learning starts in parallel.

**What changes** (`endpoints.yaml`).

1. **H1prime, declared:** B7 triggers fewer protective stops per 100 m than B1, B2 and B6.
   - Logic: intersection-union, as H1: every comparison must reject at alpha in B7's favour; no
     correction within H1prime.
   - Endpoint, population and scan-fault rule as H1: test set, `unmapped_static`, all clearance
     strata pooled; scan-fault stops excluded and reported alongside (A25).
   - Success rate and mission time are reported beside every comparison in the same table.
2. **Its analysis is A14a's:** the rate ratio, B7 over each comparator, continuity 0.5, the
   closed-form t interval. Every arm in H1prime runs once, so the interval has the scenario part
   only, with a t quantile on n - 1 df (the A14a formula with no seed part).
   - **This case was not in the 2026-10-03 calibration grid**, which covered a learned arm against
     a classical one and two learned arms. It must pass the same calibration gate (equal rates, at
     most nominal + 3 Monte Carlo SE on each side) before H1prime is tested. If it fails, stop and
     report.
   - H1prime's test size is decided with A14b, which must give its power alongside H1's.
3. **The pivot.** If L1's or L5's pilot fails G4 twice (both pilot seeds of `G4_learned_pilot`),
   stop and report; the study then pivots to the filter-only path.
   - Under the pivot, the confirmatory family is H1prime alone, at alpha 0.05. H1, H2 and H1N are
     then not tested confirmatorily.
   - Otherwise H1prime is exploratory, and the confirmatory family (H1, H2, conditionally H1N)
     stands unchanged.
4. **The tune-set comparison of B2 and B7** (after alpha is selected; per stratum: protective stops
   per 100 m, success rate, mission time, pfb_infeasible and fallback counts) is tune-set evidence
   for monitoring and framing only. It is never cited as a confirmatory result and does not alter
   any preregistered analysis.

**What does not change.** H1, H2, H1N, the narrow rule, A14a's analysis of them, the alpha grid and
its selection rule, the G4 criteria, and the test set.

**Prerequisites before H1prime can be tested:** B6 implemented and tuned (N = 24); the classical-pair
calibration passed; the test size from A14b.

## A28 - 2026-10-27 - Real-time criterion, G2's over-budget share, the L2 limit a priori, B3/B6 candidate 0, one invalid-bin convention

**Decided by the supervisor on 2026-10-27** (rulings on the questions of that day), recorded in
`docs/decision_log.md`, with the instruction to amend the preregistration where needed and relock
before any tuning. Applied after A27. No test layout was built, read or used. No method has been
trained, tuned or evaluated; no candidate has run.

**What changes.**

1. **G2** (`gates.yaml`, `G2_compute_budget`): `max_over_budget_share: 0.001`, the numeric reading of
   "essentially no over-budget steps". Scan faults are reported beside it for each instance count.
2. **The real-time criterion** (`gates.yaml`, new `realtime_compute`):
   - p95 compute per step <= 50 ms, the control period, on the training host, for every controller.
   - B6's samples and horizon are removed from its tuning space. They are fixed by a declared
     measurement on the training host: the largest pair of samples {256, 512, 1024} x horizon
     {15, 20, 30}, by samples x horizon, with p95 <= 50 ms in 0.7-0.9 m aisles; a tie goes to the
     longer horizon. `iterations` stays 1.
   - The PFB is measured under the same criterion. A stalled p95 above 50 ms on the training host
     is reported as a finding and limitation.
   - Per-step compute of every controller is reported in the results.
3. **L2** (`endpoints.yaml`, new `declared`; `exploratory`):
   - The cost limit is d = 0.1 protective stops per episode, fixed a priori. This supersedes the
     2026-10-03 ruling to choose d on the tune set, which would have given L2 a per-arm tuning
     budget that no other learned arm has.
   - The multiplier: lambda_0 = 0; eta = 3e-4, applied per gradient step from the cost critic's
     batch estimate; projected to >= 0 and capped at lambda_max = 100. Declared, not tuned.
     Validated on the toy positive control.
   - L2 at d in {0.05, 0.2} (method IDs L2-d005, L2-d020) is an exploratory sensitivity, first in
     the cut order (`G2_compute_budget.reduction_order`).
   - L2's throughput is measured in its pilot. If it is more than 20 % below plain SAC's, the
     result is reported and the plan is redone.
4. **Candidate 0** (`endpoints.yaml`, `declared`):
   - **B6:** noise 0.1/0.1/0.1, smoothness 0.01, temperature 0.01, goal progress 10. Disclosed as
     coming from a manual check on two synthetic routes; not counted in N. The ranges are re-centred
     on these values, with temperature on a log scale that includes 0.01.
   - **B3:** the standard DWA defaults (Nav2 DWB, ROS 2 Humble), cited.
   - The B3 and B6 ranges are approved and committed before the first candidate runs. A winner on a
     range boundary is reported, never extended.
5. **B6's model** (declared):
   - Rollouts respect the acceleration limiter and the wheel budget.
   - No heading term.
   - Smoothness is computed within the rollout; temperature is applied to cost / horizon.
   - When every rollout is rejected, the command is zero, reported as mppi_all_rejected.
   - Rollout fields use the certified composition, max(estimate, rollout command), with reach. The
     lookup table is conservative, never smaller than the certified field.
   - B6's noise is seeded from the scenario, and that seed dependence is recorded.
6. **One invalid-bin convention** (`endpoints.yaml`, `declared`): an invalid bin is an obstacle at
   the footprint boundary along its bearing. It is one shared function for B1 (through the
   recovery), B2, B3, B6 and the PFB. Invalid-bin events are counted per method.
7. **Reporting** (`endpoints.yaml`, new `per_method_reporting`): per method and stratum, report
   pfb_infeasible, dwa_blocked_steps, mppi_all_rejected, invalid_bin_steps and the per-step compute
   (p50, p95, max, with the host).

**What does not change.** H1, H2, H1N, H1prime, the pivot, the narrow rule, A14a's analysis, the
alpha grid and its selection rule, the G4 criteria and the test set.

**Reported for a ruling with this amendment** (implemented as stated, not decided here; decision
log, 2026-10-27):

- **L2's limit in the cost critic's units.** It is converted as d (1 - gamma^T) / ((1 - gamma) T),
  with T = the episode cap of 4,000 steps (Ray, Achiam and Amodei 2019). This is 0.0025 at d = 0.1.
  The conversion is stricter than d per episode when episodes end before the cap.
- **B3's candidate 0 on two range floors.** DWB's window is 0.05 s (the control period), below the
  approved 0.10-0.40. The mapped w_clearance is 0.020, below the approved 0.2-5. Both sit at the
  floors.
- **The cut order.** It is now [L2-d005, L2-d020, L5-nowp, PFB-(a), stop_active, A1]. The frozen
  `reduction_order` [A1, L2] listed L2 itself as cuttable; the plan of record (2026-10-26) trains
  L2 in Stage 3.

## A28a - 2026-10-27 - A28 finalised: L2's multiplier on measured stops, B3's floors, the cut order, B6's update, the compute stop rule

**Decided by the supervisor on 2026-10-27** (rulings on A28's five open points), recorded in
`docs/decision_log.md`. A28 had already been applied and relocked, so its finalisation is this dated
revision. No test layout was built, read or used. No method has been trained, tuned or evaluated; no
candidate has run.

**What changes.**

1. **L2's multiplier** (`endpoints.yaml`, `declared.L2_multiplier`):
   - lambda is updated from the EMPIRICAL mean of protective stops per episode over the last 20
     completed episodes, compared directly with d = 0.1.
   - eta = 3e-4 per gradient step; lambda_0 = 0; lambda is projected onto [0, lambda_max = 100].
   - lambda is held until 20 episodes have completed.
   - The cost critic still drives the policy gradient.
   - This supersedes A28's "from the cost critic's batch estimate". **Reason:** units consistency
     with the declared d (stops per episode). No (1 - gamma^T) conversion.
2. **B3's candidate 0** (`declared.B3_candidate_0`; ranges in `configs/tuning/b3_dwa.yaml`): the two
   floors are extended to include DWB's own values:
   - window_time 0.05-0.40 s;
   - w_clearance 0.02-5 (log).

   This corrects the declaration before any run; the never-extend rule applies only after tuning.
3. **The cut order** (`gates.yaml`): [L2-d005, L2-d020, L5-nowp, PFB-(a), stop_active, A1] is
   confirmed. New `never_cut`: L1, L2, L4, L5 and all classical comparators B1, B2, B3, B6, B7.
4. **B6's update** (`declared.B6_update`): standard MPPI is confirmed -- the weights average the
   sampled commands, and the limiter is applied inside the rollouts.
   - **Why the earlier form crawled:** averaging the limited sequences fed the limiter's asymmetry
     back into the plan every step. Braking can remove a_decel_max dt = 0.05 m/s per step;
     accelerating can add only a_max dt = 0.025 m/s. B6 settled near 0.08 m/s on an open route.
5. **Compute ties** (`gates.yaml`, `realtime_compute.B6_rule`): the longer horizon is confirmed.
   **Declared outcome:** if even 256 x 15 exceeds 50 ms p95 on the training host, stop and report.
   The criterion is not relaxed.

**What does not change.** Everything else in A28; H1, H2, H1N, H1prime, the pivot, A14a's analysis,
the G4 criteria and the test set.

## A28b - 2026-10-27 - L2's multiplier: PID-Lagrangian, three declared gain candidates, selection by the toy control

**Decided by the supervisor on 2026-10-27** (ruling on the L2 toy control's FAIL), recorded in
`docs/decision_log.md`. No test layout was built, read or used. No method has been trained, tuned or
evaluated. The PID candidates below were declared before any toy run with them; the only toy run with
candidate 1 was the fast suite's own 1,000 steps, a regression test, not the control.

**Why.** A28a's projected dual ascent failed the toy control: at 5,000 steps, seeds 0-3 (commit
`171f366`), the last 20 episodes gave 0.00, 3.20, 0.60 and 1.35 stops per episode against
<= 1.25 at d = 1. It met d on average (0.86-1.06 over episodes 250-1,250), but it ran a limit cycle,
with a period of ~300-400 episodes and lambda between 0.01 and 0.27. Only 69-77 % of 20-episode
windows met 1.25 d. The last iterate -- the policy that is evaluated -- therefore depended on the
cycle's phase.
- Option (a), accepting this, was rejected: L2's final checkpoint would depend on the phase, which
  biases H1.
- Option (c), judging on a time average, was rejected: it rewrites a declared criterion after a
  failure, and the evaluated policy is the last iterate.

**What changes** (`endpoints.yaml`, `declared.L2_multiplier`, new `declared.L2_pid_candidates`):

1. **The update** -- a PID-Lagrangian (Stooke, Achiam and Abbeel, ICML 2020), once per completed
   episode k, from c_bar_k, the mean protective stops per episode over the last 20 completed
   episodes:
   - e = c_bar_k - d;
   - I = max(0, I + e), the integral projected as in Stooke et al.'s Algorithm 2, I_0 = 0;
   - lambda = min(100, max(0, K_P e + K_I I + K_D max(0, c_bar_k - c_bar_{k-1}))).

   lambda_0 = 0. lambda is held, with I = 0, until 20 episodes have completed. The first update has
   no derivative term (there is no previous mean). The per-gradient-step update (eta 3e-4) is retired.
2. **Three candidates**, (K_P, K_I, K_D) in declared order:
   - 1 = (0.1, 2e-3, 0), PI;
   - 2 = (0.1, 2e-3, 1.0), PID;
   - 3 = (0.2, 2e-3, 1.0), PID with the stronger proportional term.

   Units: reward units per stop per (stop per episode).
   **Derivation** (as ruled: the integral alone reaches the oscillating run's level of ~0.1-0.3
   within ~100 episodes):
   - In the A28a runs the summed excess over the first 100 updates was 88-116 (mean ~104), so
     K_I = 0.2 / 104 ~ 2e-3. Its lambda's 90th percentile was 0.18-0.20.
   - K_P: a typical excess (|e| median 0.9) moves lambda by 0.1 or 0.2 at once.
   - K_D: the 95th-percentile rise of c_bar per episode (0.1) adds ~0.1, while the mean rise (0.02)
     biases lambda by only ~0.02.
3. **Selection.** The toy control is rerun per candidate: 5,000 steps, seeds 0-3, criteria
   unchanged. Those criteria are: the last 20 episodes at most 1.25 d at d = 1 and at most half of
   plain SAC's; plain SAC at least 2 d; lambda in [0, 100].
   - L2's gains are those of the FIRST candidate, in declared order, that passes on all four seeds.
   - The fraction of 20-episode windows meeting d is reported.
   - If none passes: stop and report.
   - The same gains apply to L2, L2-d005 and L2-d020. Until the selection, every L2 config's
     `pid_candidate` is OPEN, and training refuses it.

**Note for the record.** The ruling writes the integral as K_I sum(e). Stooke et al.'s Algorithm 2
projects the integral onto >= 0 at every update (I <- (I + e)+). That projection is implemented, as
"Stooke et al.'s form". Without it, the integral would wind below zero during long stretches under
the limit, and lambda would stay at 0 until it unwound. This is the windup that the cap at 0 already
prevented in A28a's dual ascent. The reference is in `paper/MDPI_template/references.bib`, marked
VERIFY.

**What does not change.** d = 0.1 a priori; L2-d005 and L2-d020 exploratory and first in the cut
order; the 20-episode window; lambda_max 100; units consistency with d; everything else in A28 and
A28a.
