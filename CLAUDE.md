# CLAUDE.md — pfb-nav (v2)

@SESSION_HANDOFF.md
<!-- Imported so a new session knows the state, next steps and working preferences. It is a dated handoff: when it is stale, update or remove it and this line. -->

## Project in one paragraph
PhD research code. A KUKA youBot (mecanum, holonomic) in CoppeliaSim navigates warehouse
layouts with A* global planning and a learned (SAC) local planner. The contribution is the
Protective-Field Barrier (PFB): a LiDAR-sector control barrier function whose zero set is a
continuous, speed- and direction-dependent protective field. A separate certified-layer
emulator (discrete speed-banded fields, latched protective stops) runs beneath every
controller. Primary metric: protective stops per 100 m. This is a scientific codebase:
correctness and reproducibility outrank speed and elegance.

## Commands
- Install: `pip install -e ".[dev]"`
- Fast tests (no simulator): `pytest -m "not sim"`
- Simulator tests: `pytest -m sim` (needs CoppeliaSim running on the configured port)
- Lint: `ruff check .`
- Train (researcher only, see "Long-duration operations"):
  `python scripts/train.py --config configs/experiments/<name>.yaml --seed <n>`

## Architecture
- `pfbnav/sim/` ZeroMQ client, robot kinematics, scanners, ground-truth contact
- `pfbnav/world/` layout generator, map baking, scenario generation and splits
- `pfbnav/safety/pfb.py` PFB filter — pure functions, no simulator imports
- `pfbnav/safety/certified.py` certified-layer emulator — independent of `pfb.py`
- `pfbnav/planning/` A*, lookahead tracking
- `pfbnav/control/` Pure Pursuit, Regulated PP, DWA, SAC wrappers (common interface)
- `pfbnav/envs/` Gymnasium environment
- `pfbnav/eval/` evaluation runner, metrics, statistics
- `lua/` CoppeliaSim child scripts (server-side scanner processing)
- `docs/decision_log.md` every design decision with date and reason

## Conventions
- Body frame: +x forward, +y left, +z up. Bearings in (-pi, pi]. SI units everywhere.
- Every physical parameter lives in `configs/` with its unit in a comment. No magic numbers.
- Pure functions for anything mathematical; simulator I/O stays in `pfbnav/sim/`.
- Type hints on public functions; docstrings state units and frames.
- Scripts take `--config` and `--seed`; every run is created through
  `pfbnav.utils.runio.create_run_dir`, which records config, commit, dirty state, and the
  scene's SHA-256.
- **Measured values go into the config as the upper bound of their measurement bracket.**
  A measurement reported as "within tolerance of x" is configured at the bound that
  over-states the robot, never the best estimate: under-stating a safety footprint or
  envelope is the unsafe direction. (WP0 closure; e.g. half_width 0.2086 from a probe
  upper bound of 0.20856.)
- **Every new emulator analysis uses the sector scan** (`emulate_layout(..., sector_rays=...)`, the
  filter stream's form at the measured ray density); the one-ray default exists only to reproduce
  analyses made before 2026-10-24 (supervisor, 2026-10-24).
- **Determinism is claimed within one scene load only** (ruled 2026-10-14; it said "one
  process" until a reload of the same scene in one process moved trajectories by up to 1.8 mm).
  Repetitions inside one load are bit-identical; across loads or invocations they are not. Paired
  comparisons run their members within one scene load (`docs/reproducibility.md`,
  `docs/requirements.md`).

## Hard rules (never break these)
1. Never modify `preregistration/`, `scenarios/test/`, or anything under `runs/`.
2. Test layouts are never used for tuning, debugging, reward design, or plots during
   development. Use `scenarios/tune/`.
3. `pfbnav/safety/certified.py` must never import from `pfbnav/safety/pfb.py`, and vice
   versa (enforced by `tests/test_architecture.py`).
4. Collisions are defined only by simulator contact checks, never by LiDAR thresholds.
5. Unobserved space is never treated as free. Scanner bins carry validity flags.
6. Every new metric, diagnostic, or statistical routine gets a test on synthetic data with
   a known answer before it is applied to real results.
7. Do not change reward terms, observation layout, or filter equations without an entry in
   `docs/decision_log.md` in the same change.
8. Do not evaluate a filter by wrapping a checkpoint trained without it and calling that
   the method; post-hoc filtering is a separate, labelled baseline (L3).
9. Do not silence exceptions with broad `try/except`. Fail loudly, or return an explicit
   status that is logged and counted.
10. Never delete, rename, or retrain over existing checkpoints or results.
11. Never run long-duration operations yourself. The researcher runs them; you prepare
    them and review their results (see "Long-duration operations" below).
12. Never stage the scene (`scenes/*.ttt`) implicitly: no `git add -A` or `git add .`;
    stage files by name. A scene edit is deliberate, verified by reading back what changed
    (`tests/test_collection_semantics.py` for collision flags), committed alone, and
    updates `sim.scene_sha256` in `configs/base.yaml` in the same commit. Runs refuse a
    scene whose hash differs from the pinned one.
13. A value read through a deprecated API, or a setting written without confirmation,
    is not evidence. Verify a simulator setting by reading it back, and where the effect
    is observable, by the effect (real-time pacing was confirmed by the step moving to
    50 ms). Withdraw findings that rest on an unverified read; do not downgrade them.
14. Coverage and boundary tests sweep geometry densely across the region that matters,
    not at convenient distances. (Every scanner check put its walls at >= 0.6 m, just
    outside a 0.563 m blind zone, and missed it; supervisor, 2026-09-21.)
15. **Where a derived quantity and a measured one both exist, the parameter is set from the
    measurement and the test asserts against the measurement.** The derivation explains the
    number; it never sets it (supervisor, 2026-09-27). **And a measurement is only worth that
    authority if something can show it measured the right thing: every probe carries a
    positive control** (endorsed by the supervisor as this rule's second half, 2026-09-28).
    The worked example is the one that taught both halves. The narrow
    floor's slack was justified against a *measured* 9.6 mm "scan penalty" in preference to a
    *derived* 2.6 mm sector-min term. On 2026-09-27 a positive control in the traversability
    probe showed the 9.6 mm was the test walls' **half-thickness**: a 0.02 m cuboid is placed
    by its centre and presents its surface 10 mm nearer, and the scanner measures the surface.
    The real scan costs about **1 mm**, and the residual against the wall face is +-0.5 mm and
    does not scale with distance. The measurement beat the derivation and was still wrong,
    because nothing had checked what it was pointing at. Two tests had passed on the same
    artefact, each with a tolerance that happened to equal it.
16. Gate overlays never pin measured constants. A gate config carries only its own
    criteria; every measured value is read from `configs/base.yaml`, so a re-measurement
    cannot leave an overlay quietly stating the old number (supervisor, 2026-09-22:
    `configs/gates/g0_rerun.yaml` pinned half_width 0.2085 and ended up 0.1 mm below the
    measured value, under-stating the robot).
17. **Pilot seeds never enter H1 or H2, and confirmatory seeds are never selected.** A learned
    method's pilot seeds come only from the pilot range (G4, `pilot_training_seeds`). After G4
    passes, every confirmatory seed trains unconditionally and stays in the analysis whatever its
    outcome: no seed is rerun, replaced or dropped for how it turned out (supervisor, 2026-10-03,
    amendment A15). A hyperparameter change after a G4 failure applies identically to every
    learned method and restarts the pilot.
18. **Tests and probes cover the whole operating envelope, including the world's spatial extent,
   not only the region a probe was built for** (supervisor, 2026-10-10). Every probe and gate ran
   within a few metres of the origin, while each episode places its hall so the scenario's start
   lands on the robot: halls reached 14.5 m on a floor that ended at 7.5 m, and nothing looked.
   `tests/test_extent.py` now checks every built world against `sim.floor_half_extent_m` at any
   placement yaw, and `tests/test_floor_sim.py` reads the floor and what stands on it back.
19. **A tolerance check applied across multiple columns states its expected false-alarm rate under
   the null before it runs, and is null-tested -- e.g. on an extra load of the unchanged file --
   before it can gate anything** (supervisor, 2026-10-14). The floor regression's saved-file leg
   took the maximum of three reloads as an envelope and checked it across every column of every
   log. Two of the three reloads were identical, velocity estimates amplify sub-micrometre pose
   differences, and the leg "failed" on 0.07 mm/s: INCONCLUSIVE, because nothing had shown what
   the check does when nothing has changed.
20. **Method IDs are never reused** (supervisor, 2026-10-15). A retired ID stays retired, listed in
   `configs/methods.yaml`, so a frozen document that names it still means one thing; a new method
   takes a new ID. B4 (identical to B1 once the certified layer ran beneath every controller) and B5
   (superseded) were retired by A16; B6 (field-aware MPPI) and B7 (RPP + PFB) are new.

## Workflow expectations
- Before editing `pfbnav/safety/`, `lua/`, or `configs/base.yaml`: state the change, the
  reason, and the tests that will verify it; then run `pytest -m "not sim"`.
- For mathematical code (PFB rows, kinematics, footprint geometry): add a finite-difference
  or known-answer test in the same change.
- Keep changes scoped to the current work package. If something outside scope looks wrong,
  report it instead of fixing it silently.
- When unsure about a physical value, mark it `VERIFY` in the config and say so.
- Summarise each session: files changed, tests added, open questions.

## Long-duration operations (researcher runs, Claude reviews)

### What counts as long-duration
Any of the following, regardless of how short it might turn out to be:
- RL training of any length, including pilots and smoke tests beyond ~1,000 steps
- Evaluation over a scenario set (tune or test), or any multi-controller comparison
- Gate scripts that drive the simulator for more than a few minutes (G2 always; G1 if
  all profiles are run)
- Launching or managing CoppeliaSim instances, especially several in parallel
- Hyperparameter or baseline tuning sweeps
- Any command expected to take more than about 5 minutes or to occupy the GPU

Short checks are allowed: `pytest -m "not sim"`, linting, and single `sim`-marked unit
tests against an instance the researcher has already started.

### Handoff: when your implementation is finished
1. Run the fast test suite and report the result.
2. Stop, and write a **Run Request** in the chat, then wait. Do not start the job, and do
   not start it in the background (no `nohup`, `&`, `tmux`, `screen`).
3. The Run Request contains:
   - **Purpose:** which work package / gate / hypothesis this serves
   - **Preconditions:** CoppeliaSim instances, ports, scene, committed files
   - **Command(s):** exact, copy-pasteable, one per line, with `--config` and `--seed`
   - **Expected duration and resources:** estimate, GPU/CPU, disk
   - **Outputs:** exact run directory and files that will be produced
   - **What to watch while it runs:** e.g. TensorBoard scalars, early-failure signs
   - **What to send back:** which files or paths to review when it finishes

### Review: when the researcher reports the run is finished
1. Read the produced files only (summaries, CSVs, logs, `provenance.json`); do not rerun.
2. Check provenance first: commit hash, dirty flag, resolved config, seed.
3. Compare against pre-declared criteria in `preregistration/` where they exist; report
   PASS/FAIL with the numbers. Do not reinterpret or relax criteria.
4. Report anomalies: NaNs, crashes, truncated logs, suspicious outcome distributions,
   timeouts that may hide contacts, seed-to-seed outliers.
5. Propose next steps, including any follow-up Run Request, and wait again.

## v1 legacy
v1 (unicycle, single front scanner, 45-D observation) lives at git tag
`v1-chapter-results`. Do not import from it or copy its assumptions. Migration follows
`docs/migration_v1_to_v2.md`, one module at a time, with tests.

## Current work package

**PLAN OF RECORD (ruled 2026-10-26, "the path to primary results"; decision log):**
- **Stage 1:** L1 pilot + G2; B2 tuned (N = 24); alpha tuned with B7; the B2-vs-B7 tune-set comparison
  (monitoring only); B3, B6 and L2 implemented in parallel (code only).
- **Stage 2 (week 8):** L5 pilot vs G4. Two G4 failures of L1 or L5 -> stop, report, pivot to
  H1prime (A27, B7 vs B1/B2/B6).
- **Stage 3:** L5, L1, L4, then L2; classical tuning in parallel. Cut order: L5-nowp, PFB-(a),
  stop_active, A1.
- **Stage 4:** one test evaluation.
- **Freeze discipline:** environment and certified-layer changes only for blocking defects; the rest
  goes to `docs/requirements.md`.
- **Weekly reports** state progress against the milestones (weeks 7, 8, 9-14, 15-17).
- **Ruled 2026-10-27 (A28, drafted; applied after A27, relocked before any tuning):** G2 over-budget
  share 0.001; real-time criterion p95 compute/step <= 50 ms on the training host for every controller
  (`scripts/measure_realtime.py` fixes B6's samples x horizon; B6 untunable until
  `baselines.mppi.compute_measurement` is set); B6 candidate 0 (disclosed manual check), limiter in
  rollouts, certified composition, conservative table; B3 candidate 0 = Nav2 DWB defaults; one
  invalid-bin convention (`pfbnav/safety/unobserved.py`); L2 d = 0.1 a priori, eta 3e-4 per gradient
  step, lambda_max 100; L2-d005/L2-d020 exploratory, first cut. Re-freeze 12 (`d7cd5e3a`).
- **A28 applied and relocked; A28a (rulings on its five points, 2026-10-27) drafted:** L2's lambda
  steps on the measured mean stops of the last 20 episodes against d (no discounting); B3's
  window_time/w_clearance floors extended to DWB's values; never cut L1, L2, L4, L5, B1-B3, B6, B7;
  standard MPPI update confirmed; if even 256 x 15 fails 50 ms p95, stop and report.
- **A28a applied and relocked (171f366).** L2 toy control FAILED under A28a (2 of 4 seeds; a dual-ascent
  limit cycle). RULED 2026-10-27: PID-Lagrangian (A28b), lambda once per episode; three
  declared gain candidates (`PID_CANDIDATES`); the first to pass the unchanged toy control on seeds
  0-3 is L2's (`validate_l2_toy.py --candidate`); none -> stop and report.
- **A28b applied and relocked (b9c1b42); candidate 1 SELECTED** (passed on seeds 0-3, 2026-10-27).
  Flagged to the supervisor: the cycle persists under the PID (20-episode means still reach 4 after
  episode 750; late windows meeting 1.25 d 0.78-0.82 against A28a's 0.69-0.77).
- **2026-10-29: real-time measured on the training host** (DESKTOP-55VR508): B6 = 1024 x 20 (37.1 ms
  p95), every classical controller and the PFB within 50 ms. **RULED 2026-10-29:** SAC on the GPU for
  every learned arm and seed (`training.device: cuda`, no `auto`, no fallback; provenance carries
  driver/CUDA/cuDNN; training statistically, not bit-for-bit, reproducible); learned arms' inference
  timed on CPU once L1's checkpoints exist; B7's p95 a labelled sum until its tuning runs measure it;
  L2 proceeds with candidate 1 (residual cycle reported as a comparator limitation); projected
  integral and first update without derivative accepted; **no laptop G2: every citable run, all
  tuning included, on the training host.** Next: G2, L1's pilot, then B2 tuning, B7 and alpha.
- **2026-10-29: G2's first run crashed** (a random action at 0.549 m/s; the restart check sized its
  band from the raw request, above the 0.51 top band). Fixed as a blocking defect: band from
  min(|request|, speed_cap). A29 drafted (`docs/pending/A29_amendment.md`); env re-frozen 3757aeca.
  The researcher applies A29, relocks, reruns the filter-off bit-identity test, then G2.
- **2026-10-29: A29 applied and relocked (2c3b5e0).** The host's re-stamp differed from the laptop
  build only by last-bit rounding (max 1.9e-15 m). RULED: re-stamp on the laptop (exact); from now on
  identities/hashes exact and floats within `world.set_verification.float_abs_tol_m` 1e-9 m; all set
  building and verification on the training host (A14 test sets included).
- **2026-10-29: G2 PASS** (`G2_20261029T171718Z`, commit 79e98f9): 4 instances approved, SAC 117 steps/s
  at N = 4 on the GPU, over budget 0. Route schedules recomputed (rows identical). Next: the
  filter-off bit-identity test, then L1's pilot (seed 9000000), the G4 controls, B2 tuning.
WP1 — Environment. WP0 closed on 2026-09-18. **State as of 2026-09-28** (details and
reasoning in `docs/decision_log.md`; the latest entries are at the end of the file).

**Done:** the Gymnasium environment with the certified layer beneath it and the server-side
scan packet; D2, re-measured against the real scan in both states (69.5 mm at rest and
centred, 19.4 mm at the 50 mm tracking bound, narrow floor 0.13 m); **G0 PASS** in both
contexts (five cases, three seeds); **G1b PASS**, v_max 0.40 / vy_max 0.50 unchanged on the
S300 scene; the per-step budget; the traversability precondition, **passed** on all three
strata with an unavoidable-stop floor of 0.0 per 100 m.

**Changed on 2026-09-27/28 and not to be undone without a ruling:** the at-rest field is
`e(psi) + lateral_margin`, the infimum of the moving fields (it was `+ margin`, which made
the field shrink as speed rose); a moving field carries `margin` AHEAD only and
`lateral_margin` abeam and behind (trailing margin ruled 2026-09-28); the restart condition
(band-0 leading field clear AND `v . p <= 0` for every breaching point, with the latch also
requiring the field the robot stands in to be clear); `unrecoverable_stop` as its own outcome;
the emulator's own warning field, `warning_extra` 0.05 ahead and `warning_extra_abeam` 0.02
abeam; the new dropout mechanism (training only, dropped bins read 0.0, never range_max). The
"9.6 mm scan penalty" was the test walls' half-thickness; the real scan costs about 1 mm, and
the narrow slack 0.03 is a deliberate buffer. Identity hashes are of text with line endings
normalised (`pfbnav/utils/hashing.py`) so they agree between this laptop and the training host.

**Answered 2026-09-28:** the before-the-freeze decision request (D1-D5 and the restart
condition), recorded in `docs/decision_log.md`. The hypotheses H1, H2, H1N and the Holm family
now exist in the supervisor's consolidated `endpoints.yaml`.

**PREREGISTRATION FROZEN 2026-09-28** (`5b8c9f8`, lock at `853ce00`): `gates.yaml`,
`endpoints.yaml`, `AMENDMENTS.md` through A12, plus `generator_sha256` and `scenarios_sha256`.
From here any change to those files -- or to `pfbnav/world/layouts.py` or `scenarios.py` -- fails
`tests/test_preregistration.py` until an amendment says why. The G0 dry run placed case six on
`/youBot/SickS300[1]`; the corrected narrow probe measured 0/170 warning steps at the offset.

**Next, in order:**
G0 PASSED with six cases under the frozen criteria (2026-09-28, three seeds, both contexts);
D2 is pinned at the true floor (79.5 / 79.5 / 29.4 mm); the fast suite has no skips.

Ruled 2026-09-29 (order in docs/decision_log.md): A13 now, then dry-run tune + pilot, then G2
on the training host (pilot seeds), then the H1 power analysis, then A14 (test size).
1. A13 is implemented (route-relative placement, `pfbnav/world/placement.py`; `world_of`; pilot
   150 distinct, learned seed i mod 5). Drafts in `docs/pending/` (`scenario_sets.yaml`,
   `A13_amendment.md`). `first_offset_max` RULED 0.3 (2026-09-30, option a; "first" is an index
   not a location; the offset analysis is descriptive). The researcher commits A13 and runs
   `scripts/freeze_prereg.py --relock A13`. Until then ONE test fails by design (the lock).
2. Dry run DONE 2026-10-02: all sets built and engaged, but "reachable in the true world" let
   obstacles block the route (>= 29 % of pilot needs another aisle; the stack never replans).
   RULED: bypass within W = 1.0 m at the narrow floor, jointly (`pfbnav/world/bypass.py`);
   fallback 1.5 only if a set cannot be built in full. W sweep DONE: every set built in full
   at 0.75 / 1.0 / 1.5, so W = 1.0 stands; narrow obstacles land 97 % on open floor, never in an
   aisle. DONE 2026-10-02: A13 revision committed (aaca564), relocked (c88c6df), tune + pilot
   rebuilt (`scenarios/tune/`, `scenarios/pilot/`; identical to the W = 1.0 dry run, verify ok).
   REORDERED 2026-10-02 (supervisor): H1 power analysis now, then SAC, then G2, then A14.
   Power analysis implemented (`pfbnav/eval/power.py`, `scripts/power_h1.py`); the researcher runs
   it (~30 min). Found: the frozen Wilcoxon leans against L5 vs single-run classical comparators,
   and shared seed effects inflate learned-vs-learned null rejections. RUN DONE: decision-point
   power 0.015 at 100/cell. RULED 2026-10-02: H1 = rate ratio, bootstrap CI on its log,
   two-level (method-level) confirmatory, policy-scoped secondary, H2 inherits; floor removed
   from the criterion; alpha/3. Rerun implemented (`pfbnav/eval/rate_ratio.py`,
   `scripts/power_h1_rate_ratio.py`). RUN DONE: all candidates fail calibration (seed level
   under-stated with 5-10 seeds); power capped by seed count (~15 seeds/arm at seed CV 0.3).
   RULED 2026-10-03: K/(K-1) + t + WS correction; seed CV measured on the TUNE set (upper 80 %
   bound); decision tree in docs/decision_log.md. Corrected run implemented
   (`scripts/power_corrected.py`); run CORRECTED_20261003T153838Z in progress (its H2 part is
   superseded: rerun with `--parts h2`). RULED 2026-10-03 (six questions): train set thirds x
   ~2,000, seed k = its own permutation shared by every method, no curriculum; L2 = SAC-Lagrangian
   (toy validation first); WP2 next, L1 in parallel only on a FROZEN environment (env hash in
   provenance, filter-off bit-identical test, G4 criteria declared before L1's pilot seed); WS df
   vs classical only if calibration passes, else flat K-1 (implemented); H2 grid extended, no
   decisive H2 cell until the tune-set seed SD exists.
3. G2 on the training host, on pilot seeds (the script is still a stub).
4. H1 power analysis (Monte Carlo of the frozen analysis), then A14.
5. SAC: core STARTED 2026-10-03 (`pfbnav/control/sac.py`, `pfbnav/training/record.py`,
   `scripts/train.py`). Refuses `filter.enabled` (PFB is WP2, not wired) and an OPEN training set.
   ENVIRONMENT FROZEN 2026-10-03 (`env.frozen_code_sha256`, `pfbnav/envs/freeze.py`; PFB slot in
   place, off = identity; sim bit-identity test). Training set + per-seed order implemented; the
   prereg addition (train, tune_unmapped) and G4 are drafts in docs/pending/. Extended decision
   tree (Step 0, branches 4-5, seeds-as-units fallback) in the decision log. Flat K-1 PASSES
   calibration (H1) and is adopted; H2's grid calibrates. H1 needs branch 4+ at rate 5.
   RULED: H2 stays method-level (policy-scoped secondary only); one seed count for L5 = max of
   H1/H2 needs, cap 20, subject to G2; L4 same. Training + tune-unmapped sets BUILT 2026-10-03
   (96f2a10). Next: sim bit-identity test, G4 ruled + evaluation runner, L1 pilot, then WP2.

**2026-10-09/10: G4 BROKEN, two causes found.** (1) Halls extend off the 15 m floor; RULED
2026-10-10: 35 x 35 m (7 x 7 tiles, the nine kept), `sim.floor_half_extent_m` 17.5, guard test
over every set (test sets too) and every unmapped obstacle, regression old vs new scene before
G0/G1b/traversability carry over (`scripts/scene_extend_floor.py`, `scripts/regress_floor.py`).
A `/Wall` perimeter (0.25 m tall, ~+-7.6 m) found in the scene stands inside the new floor:
awaiting a ruling before the edit. (2) `NavEnv.reset` placed every hall after the first around
the PREVIOUS episode's end pose (pose read before the restart); fixed, with a step-0 check in
every reset; re-freeze to acknowledge. No training run exists. Then both G4 controls rerun.
REVIEWED 2026-10-13: the fix holds (16 traces, step-0 error 0, no collisions); the wall is a
confirmed 0.25 m ring at +-7.6 m; the floor is NOT collidable (in no contact set). The early stops
persist with correct placement: Pure Pursuit breaches the band-0.30 field at shelf corners (+-35 deg)
and the restart (band-0 leading field) never clears for a forward-only controller; the planner's
lethal radius 0.2386 is inside the at-rest field 0.2586. B1 would fail G4 again: awaiting rulings.
RULED 2026-10-13 (decision log): re-freeze acknowledged (re-frozen again for contact.py's docs,
c63a1083); remove /Wall in the floor edit; regression with scan bins predicted from the old wall;
planner margins BY CRITERION (zero breaches for an ideal tracker at traversal speed, plannability
kept), quantified offline (`scripts/quantify_margins.py`) and reported BEFORE values are ruled,
then amendment, rebuild, relock; a declared recovery for ALL classical baselines (B1-B5), none for
learned; floor stays non-collidable. Order: scene edit, regression, quantification, ruling,
amendment/rebuild/relock, recovery, G4 controls.
2026-10-14: scene 537f1bbb COMMITTED (1da1ca9; wall gone, 49 tiles). First regression CONTROL
FAILED (a scene reload is not deterministic); RULED: determinism is per scene LOAD; regression in
one load with the edit in memory, plus a saved-file leg within a 3-load envelope. Margin
quantification PARTIAL (1,938/2,271): plannability holds everywhere, NO pair reaches zero breaches;
straights clear from lethal 0.07; residual at bends/goal (speed, not margins). train_wide to finish.
RULED 2026-10-14: G0, G1b and traversability CARRY OVER to 537f1bbb on the regression's leg 1
(exact); leg 2 INCONCLUSIVE (flawed criterion; hard rule 19). Next: finish the margin quantification
(train_wide) and the route-level traversability report for the supervisor's ruling on values.
2026-10-15: margin report COMPLETE (no pair meets the old criterion). RULED: lethal margin 0.07 (cures
D1; config changes with the rebuild); criterion AMENDED to an ORACLE tracker (fastest clear band,
deceleration-limited; zero routes with a band-0 breach); start/goal validity rules in the generator
(re-sample the goal); bend stops are controller-attributable and count in H1. Next: the soft margin
under the oracle (`scripts/quantify_oracle.py`), residual rule <= 2 % per stratum declared.
RULED 2026-10-15 (A16, comparators): B4 and B5 retired; B6 = field-aware MPPI (H1, certified band
fields, declared in `baselines.mppi`) and B7 = RPP + PFB (H5 with stops, H3). Drafts in docs/pending/;
the researcher applies and relocks. B6 must be implemented and tuned before Step 0.
2026-10-16: lethal margin RULED 0.10 (50 mm inside-bend breaches ~94 % -> 0.8-4.2 %; the ceiling the
narrow floor allows, centreline band 0.06 m; `tests/test_plannability.py` is the permanent sub-cell
regression). Oracle at 0.10: soft 0.03 chosen (all tie at zero), residual 0. A18 APPLIED and relocked
(c1a6e8b): margins, generator validity rules (goal re-drawn, rejections per stratum), scenarios hash
over its import closure. Env re-frozen 645358f2 (bit-identity test passed). Sets REBUILT (31a519f),
oracle check PASS (floor 0 everywhere). A1's tangent floor narrow 5.49/100 m (H4 context).
2026-10-16: classical recovery RULED and IMPLEMENTED (`pfbnav/control/recovery.py`; wait 1.0 s after
the restart delay, counted from the own field clearing; 0.08 m/s translation only, verified against
198 G1b runs; 0.30 m; 5 attempts; 72 directions), identical for B1, B2, B3, B6, B7 and the zero-action
control; none for learned. Classical controllers get ONLY `ClassicalInput`
(`pfbnav/envs/controller_input.py`), never `info`. Outcome fix: stuck also = at rest with the own
field breached (two 2026-10-13 traces relabelled). certified.py and planning/lookahead.py are inside
the scenario identity's closure (A18): editing them moves the lock. Env re-frozen 564802d8 (pending
the bit-identity test). A14 must declare two secondary stop rates (excluding recovery distance;
per planned route length). Next: the sim controls (`tests/test_recovery_sim.py`), then G4 controls,
L1 pilot, B6 before Step 0.

**The researcher works in PowerShell.** Commands handed over must run there: no `sed`, and
**never `>>` on a UTF-8 file** — PowerShell writes UTF-16LE and corrupted AMENDMENTS.md once
(2026-09-27). Use `python -c "open(..., 'a', encoding='utf-8').write(...)"` for appends.

**Compute is settled and closed here.** The >= 100 steps/s target was an arithmetic
error; the requirement is >= 25 steps/s on the **training host** (derived in
`docs/requirements.md`), measured there by G2 once the environment and SAC exist.
Dynamics-step tuning is closed (A10). No throughput figure measured on this laptop is a
property of the training host; every run records its host in `provenance.json`.
Carried requirements are in `docs/requirements.md`. G2 was deferred to WP1 and runs as
specified once the environment and SAC exist.
Thresholds for gates are in `preregistration/gates.yaml`; commit them before running.