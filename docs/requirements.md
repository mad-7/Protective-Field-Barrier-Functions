# Requirements carried to later work packages

Each item names where it came from. Items are requirements, not suggestions: a work package
is not complete until its items are met or explicitly amended.

**Every numeric requirement here carries its derivation** -- the inputs, the arithmetic and
the window -- and **a requirement whose derivation does not reproduce is challengeable**:
recompute it, say so, and do not treat the number as binding until it is re-derived or
re-ruled. (Supervisor, 2026-09-20, after the >= 100 steps/s target was traced to an
arithmetic error; see `docs/decision_log.md`.)

## WP1 - environment

- **Per-step budget projection.** Nominal actions are projected into the nominal share of
  the wheel budget (`actuation.wheel_budget.nominal_fraction`) before the filter; the base
  enforces the filter share last. Bindings are counted per episode. (WP0 results, ruling 2.)
- **Wheels at the torque cap, per episode.** `YouBotBase.record_torque_cap()` after every
  step; the episode's `torque_cap_counts` (steps with any wheel at the cap, and wheel-steps
  at the cap) go into the step `info` and the episode summary, and are surfaced in
  training and evaluation logs. If saturation appears, the budget fractions drop -- by a
  logged decision, not automatically. (WP0 closure.)
- **Velocity for the safety layers** is `PoseDifferenceEstimator` over the `youBot_ref` pose,
  reset every episode, never the command or wheel odometry. (Ruling 4; A9.)
- **Per-step cost budget.** The environment's step must fit a stated wall-clock budget, set
  from the profiling pass (`docs/profiling/`) before training is planned. (WP0 closure;
  G2 deferred to WP1.)
- **G0 `env_step_loop` context** runs as soon as the environment steps. (G0, A2.)
- **Layout sets are generated, not authored in scenes** (2026-09-23): the generator emits
  polygons, the bake uses them, and the environment instantiates the same polygons into the
  fixed, hash-pinned scene at episode reset. A layout set's identity is **(seed, layout
  hash, generator code hash)**, stored in its metadata and **verified by every run**.
  Splits come from disjoint seed ranges recorded in `preregistration/`; the test set is
  locked. Tune layouts may be regenerated with a log entry; test layouts may not, absent a
  recorded amendment.
- **Unmapped obstacles are a shared environment hyperparameter**: a low declared rate in
  training (~20-30 % of episodes carrying at least one, preregistered), **identical across
  L1, L2, L4, L5 and A1**, and fixed rates per test stratum (0 % mapped-only, 100 %
  unmapped-static). Never varied per method: an advantage must come from the method, not
  from the world it trained in. Zero in training would make the test a distribution shift;
  a high rate teaches a distrust prior no real AMR's map justifies.
- **Every episode logs the number of unmapped obstacles and each one's lateral offset from
  the planned path**, so H1 is reportable against how far the map understates the world and
  not only as a stratum mean.
- **Every episode checks, at reset, that the instantiated geometry matches the baked
  polygons** in pose and extent, and that each `mapped=False` polygon is absent from the
  grid while present in the scene.
- **Every episode records its reset wall clock** in the run summary, measured by G2, with
  handle and memory use watched across resets: v1 hit accumulation failures at ~70k steps.
- **Observation layout: 82-D, ruled 2026-09-23** -- 36 policy ranges (normalised by
  `scanners.range_max`), 36 validity flags, 3 velocity, 3 lookahead
  (d_wp, sin beta_wp, cos beta_wp), 3 goal (d_g, sin beta_g, cos beta_g), 1 `stop_active`.
  Pose-agnostic: no absolute x, y, theta. The no-planner ablation drops the lookahead
  block, giving 79-D. The filter stream (120 bins, exact bearings) is for the safety layers
  and is never in the observation. Reasoning in `docs/decision_log.md` (hard rule 7).
- **Declared ablation: one L5 seed trained without `stop_active`**, reported with the
  results. `field_id` and `warning` are excluded from the observation because seeing the
  layer's state would make H1 circular, would be unfair to baselines that cannot see it,
  and would couple the policy to a deliberately independent layer; `stop_active` is kept
  only because commands have no effect during a latched stop, which without the flag makes
  those transitions unexplainable to the agent.
- **Declared sensor-dropout augmentation at a low rate during training**, so the 36 validity
  channels are not constant inputs with untrained weights. Without it the sensor-failure
  case is untested, and that must then be stated in the limitations rather than left
  implied. Rate and mechanism are set when the environment's augmentation lands, and are
  reported with the training configuration.
- **The actuation box is non-square: v_max 0.40, vy_max 0.50** (G1b under the wheel
  budget, 2026-09-20, sanctioned by the supervisor). **The policy's normalised action
  must scale each axis by its own ceiling** -- a in [-1, 1]^3 maps to
  (a0 * v_max, a1 * vy_max, a2 * w_max), never to one shared speed -- **and the asymmetry
  is stated in the MDP section of the write-up.** A single ceiling would let the same
  normalised magnitude ask for more lateral than forward motion, biasing exploration
  toward sideways travel for no physical reason. The corner (0.40, 0.50, 0) is outside
  the wheel budget and is projected, direction preserved
  (`tests/test_wheel_budget.py::test_the_box_corner_projects_into_the_budget_with_direction_preserved`).
- **Scanner cost is a design constraint, measured 2026-09-19/20.** The two LiDAR pods
  (stock Hokuyo URG-04LX-UG01 Fast, two vision sensors each covering 240 deg) cost
  ~6-9 ms per step against ~10.8 ms for everything else. The cost is **per sensor
  (~1.5 ms), not per ray**: cutting 1024 rays to 164 saved ~1.1 ms. The pods stay --
  they set `robot.half_length`, and with it the footprint table, lethal radius,
  certified field and D2 strata.
- **The scan is assembled server-side and returned once.** Reading four depth buffers
  per step costs 14-17 ms in round trips alone, independent of buffer size, so
  `lua/scanner_v2.lua` bins inside the simulator and publishes one signal, as v1 did
  (36 bins, 144 B). Its own cost -- the binning loop -- is measured by the profiling
  pass once the environment exists, and counts against the per-step budget.
- **D2 consistency test:** a stationary robot centred in the narrowest aisle stratum
  triggers no protective stop. (D2.)
- **G2** runs as specified once the environment and SAC exist. (WP0 closure: deferred.)

- **Compute target: >= 25 steps/s aggregate on the training host**, with the full
  environment -- scanners, policy and PFB included. **(Supervisor, 2026-09-20; replaces
  the >= 100 steps/s figure, which was RESOLVED as an arithmetic error.)**

  **Derivation.** Total simulator steps the project must run:

  | term | steps |
  |---|---|
  | training (25 runs x 300k, `training.total_timesteps`) | 7.5M |
  | evaluation across controllers, scenarios and seeds | 5-9M |
  | tuning (WP3/WP4) | 3-4M |
  | subtotal | 15.5-20.5M |
  | +20% contingency | **~20-25M** |

  Against the six-week window (`G2_compute_budget.wallclock_budget_weeks: 6` =
  3,628,800 s), ~20M steps need **5.5 steps/s** to fit the window end to end. The
  requirement is set at **25 steps/s**, which completes ~20M steps in **~9.3 days**,
  leaving the rest of the six weeks for analysis, failures and re-runs rather than
  spending the window on a single clean pass. Both figures are stated so the derivation
  reproduces: 5.5 is the floor that fits the window, 25 is the requirement.

  **RESOLVED -- the old figure and its error.** The superseded requirement was ">= 100
  steps/s, because 7.5M steps do not fit six weeks at ~50 steps/s". 7.5M steps over
  3,628,800 s is **2.1 steps/s, not 57**: the arithmetic was wrong by a factor of ~27,
  and at the measured 50 steps/s the training matrix's stepping is ~42 hours. Identified
  2026-09-20 and confirmed by the supervisor as their error. Everything the target
  drove -- A10, the step-floor diagnosis, the scanner measurement, the batched-step plan
  -- is recorded in `docs/decision_log.md` with what survives it.

  **Measured on the training host, by G2, once the environment and SAC exist.** No
  figure from the development laptop stands in for it, and every run now records its
  host (`provenance.json`, `host`).

  **Unmeasured term G2 must cover: 7.5M SAC gradient updates** (one per environment step
  at `training.total_timesteps` x 25 runs). Nothing in WP0 or WP1 has measured an update,
  and on the training host they run alongside the simulator instances; G2's aggregate is
  the environment **and** the learner, not the simulator alone.
- **Per-step budget, derived from the binding target (corrected 2026-09-23):** with N
  instances, each instance's step must fit **N / 25 s** p95, everything included, so
  **4 instances => <= 160 ms**, 3 => 120 ms, 1 => 40 ms. Computed in
  `pfbnav/eval/step_budget.py` from `env.step_budget` (aggregate target and instance count)
  and stored nowhere as a millisecond figure; `scripts/check_step_budget.py` scores a
  profiling pass against it. Robot-side I/O <= 5 ms remains the interim target for that
  part of the step. Two earlier figures are withdrawn: "step <= 100 ms p95" (it allowed
  only 10 steps/s per instance, 40 over four) and "<= 40 ms at 4 instances", which divided
  by the >= 100 steps/s target that was itself withdrawn as an arithmetic error on
  2026-09-20 and survived here uncorrected for three days. The per-step figure is
  re-derived whenever the instance count or the target changes; the aggregate binds.
- **Fewer instances cannot reach the aggregate on this laptop, and that is expected.**
  Measured (step_cost_20260918T194545Z, GUI rendering in force): 12.5 steps/s at one
  instance, 20.8 at two, 34.0 at three, 40.9 at four (worst pass of three). The step's p95
  uses 64 % of its 160 ms budget at four instances and 263 % of its 40 ms budget at one.
  None of these is a statement about the training host, where G2 measures the requirement
  with the learner running.
- **Certified field shape (ruling 2, as corrected):** the Minkowski sum of the footprint
  with the braking segment along the velocity direction, offset by 0.10 ahead and behind
  and 0.05 abeam -- ahead e + D(v) + 0.10, abeam e + 0.05 at every speed, behind e + 0.10,
  rearward travel putting the depth behind; at rest e + 0.10 all round. Rest/moving by
  `StandstillDetector` with hysteresis (enter 0.01, exit 0.02 m/s, VERIFY), transitions
  logged. `pfbnav/safety/certified.py` (`FieldGeometry`); speed from
  `PoseDifferenceEstimator`. Test: the maximum sideways extent equals e(+-90 deg) + 0.05 at
  every speed.
- **Per-stratum traversal speed (ruling 3):** derived (`pfbnav/world/strata.py`), tested,
  logged, and reported. On the Minkowski field every stratum admits the top band --
  **0.40 m/s since 2026-09-20**, when the G1b re-run under the wheel budget put v_max at
  0.40 -- centred and at the 0.05 m tracking offset; in the narrowest aisle at that offset
  19.3 mm of slack remains between field and wall, unchanged, because the field's sideways
  reach does not grow with speed.
- **Display refusals are counted in every run**, not only gate runs:
  `provenance.json` carries `display` = in force / refusals / consistent / starts
  (`SimClient.display_status`, `runio.stamp_display`). **Trigger for diagnosis
  (supervisor, 2026-09-20): stop and report if a refusal becomes fatal, or if the count
  rises with run length.** Until then it is not investigated -- it costs time, not
  correctness. A count nobody keeps cannot show a trend, which is why `starts` is
  recorded beside it.
- **Contact-check warm-up:** the first contact check after the collections are built costs
  0.6-1.5 s (profiling, 2026-09-18). The environment pays it inside `reset()` with a warm-up
  check, so it never lands in a control step.

## WP2 - PFB

- The footprint is `FootprintTable` (e and de/dpsi), and the actuation limits enter the QP as
  the mecanum wheel-speed polytope, not as a post-hoc clamp. (A6; ruling 3.)
  **Motivation measured 2026-09-19:** the direction-preserving budget projection scales
  omega with translation, so under the budget G1b's translate_while_rotating at 0.50
  m/s lost heading (0.27 -> 5.05 deg) while gaining on cross-track (25.7 -> 12.7 mm).
  The polytope constraint should let the QP trade translation for omega.
- **G1b on the current controller** (with the wheel budget) sets v_max for WP2; if it
  lands at 0.40, band tops, traversal speeds and the narrow floor are re-derived in one
  commit. (Supervisor, 2026-09-19.)
- **Acceptance test:** each safety layer is fed a velocity that differs from the command,
  and its field / D(s) follow the velocity. (Ruling 3, D7/G1b report.)
- **Declared sensitivity check** on the velocity estimate: configurable noise and one-step
  latency (`estimation.velocity`), reported with the results. (Ruling 4.)
- H6's bound is refitted to residual slip only, from runs with the budget in force; the
  expected reduction is stated in `docs/pfb_robust_spec.md` §3a. (Ruling 5.)

## WP3 - baselines

- **Every baseline respects the same non-square box and the same budget projection.**
  Holonomic Pure Pursuit, Regulated PP and DWA all command through `v_max` 0.40,
  `vy_max` 0.50 and `project_to_wheel_budget`; none may use a squared box, a larger
  lateral ceiling, or a per-axis clip. A controller allowed more lateral authority than
  another is being compared on its actuation limits rather than on its navigation, and
  the comparison would favour whichever commands the most lateral motion. (Supervisor,
  2026-09-20, with the non-square box.)

## WP5 - evaluation runner

- **All controllers for a given scenario run within one scene load**, so that pairing holds:
  determinism is established within one load of the scene, not across loads -- not even
  across loads in one process (`docs/reproducibility.md`; ruled 2026-10-14, tightening the WP0
  closure's "one process"). A paired comparison whose members ran under different loads is not
  a paired comparison. `pfbnav.eval.runner.run_controllers` runs a scenario's controller set
  back to back on one environment, and `tests/test_runner.py` asserts it never reloads.
- Every figure, table and methods sentence states the engine; drift figures carry the
  0.64 deg solver band. (A8.)
- The Newton external-validity check: final policies evaluated on Newton without
  retraining, reported as secondary. (A8.)

## Carried by A23 (ruled 2026-10-20)

- **The episode horizon is 4000 steps (200 s), a correction.** It is a v_max dependent under A5,
  never re-derived after A20. The derivation rule is in `pfbnav/eval/horizon.py`: it covers the
  privileged oracle schedule of every traversable built route x 1.3, checked over every built set
  by `tests/test_horizon.py`, the test sets too once built. Knock-on effects:
  - **Training:** fewer episodes per seed at 300k steps. If L1's pilot fails G4 on learning,
    `total_timesteps` is the first lever, not the reward.
  - **Evaluation:** a larger step budget per episode. It is re-costed in G2 (training-host
    throughput) and in the A14 test-set size decision.
- **The oracle mission time is reported as a reference line in the results.** It is the
  privileged schedule's time per route, under the fields as built, and is platform- and
  field-specific. Every mission-time figure carries it.
- **Every classical baseline (B1, B2, B3, B6, B7) carries the declared route cap**
  (`pfbnav/control/route_cap.py`). It is computed offline from the MAPPED world, and the controller
  never goes faster. Learned arms do not get it, and the methods state the asymmetry.
- **L5-nowp** (exploratory, outside the Holm family) is trained after every confirmatory arm, and
  is the first arm cut if G2's throughput falls short.

## Carried by A25 (ruled 2026-10-23)

- **H1 excludes scan-fault stops** (stops decided on a fail-safed packet) from the primary stop count
  and reports them alongside it, per method. If `scan_faults` differ materially between methods, that
  is reported prominently. Any run with more than 0.1 % of its steps scan-faulted is flagged
  (`outcome_summary["scan_fault_flag"]`), and that host configuration runs fewer instances.
- **G2:** the selected instance count must produce essentially no over-budget steps. The over-budget
  count is recorded per run (`outcome_summary["over_budget_steps"]`).
- **L1's pilot reports `scan_faults` and any ring-buffer dumps.** That is the in-loop verification of
  the scan-fault fix.

## Deferred to the next legitimate re-freeze (the freeze of 2026-10-23 admits only blocking defects)

- `pfbnav/sim/robot.py`, `limit_acceleration`'s docstring, still names `filter.a_brake`. Since WP2 the
  key is `certified.a_brake`: the `filter:` block lost `a_brake` as a restated constant, and the
  assertion now reads the certified key. A docstring edit moves the environment's identity hash
  (found 2026-10-23 and reverted), so it waits.
