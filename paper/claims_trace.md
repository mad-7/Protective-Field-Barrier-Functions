# Claims trace: `paper/MDPI_template/pfb_sensors.tex`, Sections 3-5 and Appendices A-C

Every number, equation and factual claim in Sections 3-5 and the appendices, with its source. Written
2026-10-24 against commit `e1be4c7` plus the uncommitted manuscript. Sources are:
- config keys in `configs/base.yaml` (written `key.path`);
- files and functions (`file::function`);
- decision-log entries (`DL <date>: <heading>`);
- amendments (`A<n>`);
- run directories (`runs/...`).

Derived numbers were computed by the project's own functions from the configuration (the command is
given), never by hand.

`\todo` items left in the text are listed in the session report, grouped by what resolves them.

## Derived quantities (computed 2026-10-24)

```
python -c "from pfbnav.config import load_config; from pfbnav.safety import pfb_field as pf; \
  from pfbnav.safety import certified as cert; from pfbnav.world.strata import certified_params_from_config; ..."
```

| Quantity | Value | Function |
|---|---|---|
| Reach time T | 0.15 s | `pfb_field.py::field_params_from_config` (response_time + sim.dt) |
| v_cap | 0.5074 m/s | `pfb_field.py::planar_speed_cap` (box 0.5/0.5, budget 0.95 x 0.617) |
| omega_cap | 1.5 rad/s | `field_params_from_config`: min(w_max 1.5, 0.5861 / 0.3865 = 1.5165) |
| Lever arm L | 0.3865 m | `robot.half_wheelbase` 0.228 + `robot.half_track` 0.1585 |
| Filter budget | 0.5861 m/s | 0.95 x 0.617 |
| E(0) | 0.0315 rad (1.80 deg) | `pfb_field.py::heading_reach(0)` |
| R | 0.0131 m | `pfb_field.py::rest_creep_reach` |
| Band speed limits | 0.025, 0.225 m/s (band 0, 1); top = v_cap | `certified.py::band_speed_limit` |
| a_max T | 0.075 m/s | 0.5 x 0.15 |
| Lethal / soft radius | 0.3086 / 0.3674 m | `baking.py::planner_radii` |
| Circumscribed (table) | 0.33742 m | `FootprintTable.circumscribed`; config comment `robot.footprint_table` |
| Inscribed (table) | 0.20945 m | config comment `robot.footprint_table` (D8) |
| rho_out in W | 0.3374 m (hull 0.33745) | `PFBConfig.weight_yaw` = `SweptFootprint.outer_radius`^2; DL 2026-10-23 "Two values in the WP2 rulings" (0.3374 from the table) |
| Creep speed / yaw | 0.018 m/s / 0.0533 rad/s | 0.9 x standstill exits (`filter.creep_fraction`) |
| Bypass radius | 0.4073 m | 0.2086 + 0.1987 (A23, A24 text) |

## Section 3: System and Problem Formulation

| Claim | Source |
|---|---|
| Architecture order: PFB slot, certified layer decides on the scan in hand, override, base limits, physics, sensing, velocity estimate | `pfbnav/envs/step_loop.py` module docstring and `StepLoop.step` |
| Classical baselines receive the 120-bin filter stream | `configs/methods.yaml` (classical set); `pfbnav/envs/controller_input.py` |
| Policy receives the 36-bin policy stream | `pfbnav/envs/observation.py` (BLOCKS) |
| CoppeliaSim, ODE, dynamics step 0.005 s, control period 0.05 s | `sim.physics.engine` 1, `sim.physics.dynamic_step`, `sim.dt` |
| Engine sweep: Bullet 20.55, ODE 0.47, Newton 3.12 deg, MuJoCo failed; 5 m lateral at 0.12 m/s | DL 2026-09-18 "D7 engine sweep"; A8 |
| Footprint table per whole degree, periodic spline, 0.95 mm margin | `robot.footprint_table` comment; `pfbnav/safety/footprint_table.py` |
| Inscribed 0.20945, circumscribed 0.33742 m | `robot.footprint_table` comment (FOOTPRINT_20260922T170810Z) |
| Half-length 0.2854, half-width 0.2086 m, upper bounds of brackets | `robot.half_length`, `robot.half_width`; CLAUDE.md conventions |
| Eq. (mixing) | `pfbnav/sim/robot.py::wheels_from_body` (`_MIX_SIGNS`); per-wheel sign vector omitted in the text |
| r_w 0.05 m; L = 0.228 + 0.1585 | `robot.wheel_radius`, `robot.half_wheelbase`, `robot.half_track` |
| Eq. (budget), rim speed = \|vx\| + \|vy\| + L\|w\| | `robot.py::wheel_budget_use`, `WheelBudget` docstring |
| v_rim 0.617, torque-balance, load-dependent | `actuation.wheel_budget.v_rim` comment; DL 2026-09-18 "Wheel ceiling confirmed a torque limit" |
| Filter share 0.95 (0.586 m/s), direction-preserving projection | `actuation.wheel_budget.filter_fraction`; `robot.py::project_to_wheel_budget`; `YouBotBase.command_body_velocity` |
| Nominal share 0.80 described but not applied | `actuation.wheel_budget.nominal_fraction`; `robot.py` docstring; no use in `pfbnav/envs` or `pfbnav/control` (grep): **inconsistency, `\todo` in text** |
| Axis box 0.5/0.5/1.5, direction-preserving | `actuation.v_max`, `vy_max`, `w_max`; `robot.py::clamp_velocity_with_status` |
| Eq. (limiter), isotropic, a_dec when slowing | `robot.py::limit_acceleration` |
| a_max 0.5, alpha_max 1.4 co-specified; band coherence; alpha = a_max / rho_out rounded down; achievability measured | `actuation.a_max`, `actuation.alpha_max` comments; A20 item 2 |
| a_dec 1.0 = a_brake | `actuation.a_decel_max`; `certified.a_brake` |
| Velocity estimate: pose differencing, low-pass 0.10 s, privileged; command over-states, odometry inherits | `estimation.velocity.filter_tau`; `estimation` comment (ruling 4) |
| Two S300-type scanners, diagonal corners, two 135 deg sensors each | DL 2026-09-22 "Scene: the S300-type layout"; `runs/diag/COVMAP-s300_corners_20260922T214324Z_seed0/summary.json` (sensors, fov 135) |
| 270 deg corner argument; adopted after blind zone | DL 2026-09-21 "The scanners leave a blind zone" and "Supervisor's decision" |
| Scan plane 0.2206 m, above wheel tops and chassis top | COVMAP-s300 summary `scan_plane.sensor_world_z_m` (0.22063, 0.22062); DL 2026-09-22 (z = 0.2207, wheel tops 0.0999, chassis top 0.1764) |
| Obstacles below the plane unseen; collisions without stop | DL 2026-09-22 "Re-derivation on the S300 footprint" (limitation) |
| "S300-type layout", no certified-sensing claim | DL 2026-09-21 supervisor decision |
| 576 pixels / 135 deg; 4.27 rays/deg; lowest density within 10 mm at 1-2 m | COVMAP-s300 summary `rays_per_sensor`; `scanners.rays_per_degree` comment (SCANRES) |
| Robot rendered except own casing; footprint + 5 mm self-filter | `pfbnav/sim/scanner.py` docstring; `scanners.self_filter_margin` |
| Policy 36 x 10 deg, filter 120 x 3 deg with measured bearing and validity | `scanners.policy_bins`, `scanners.filter_bins`; `pfbnav/sim/scan_packet.py` docstring |
| Range [0.05, 5.0] m | `scanners.range_min`, `scanners.range_max` |
| Validity: sector seen from footprint edge to 1.0 m beyond, position-aware, occlusion | `scanners.coverage_depth`; `pfbnav/sim/coverage.py`; DL 2026-09-21 "Position-aware validity" |
| Invalid sector breaches every field | `certified.py::field_breached`; CLAUDE.md hard rule 5 |
| Coverage sweep: 20 mm post, 25 mm grid, 7,680 cells; corners all observed, 120/120 valid; pods 3,713 invalid, 60/120 valid | COVMAP-s300 and `runs/diag/COVMAP-centreline_20260922T163600Z_seed0/summary.json` |
| Plausibility: +-2 sectors, +-0.06 m, widened per step, >= 6 sectors, 2 re-renders, fail-safe | `scanners.plausibility.*`; `pfbnav/sim/scan_guard.py`; A25 item 1-3 |
| Emulator independent of PFB; no compliance claim | `certified.py` module docstring; CLAUDE.md hard rule 3 |
| Containment definition | `certified.py::FieldGeometry.support` docstring; A20 "Why" |
| t_r 0.10 s; Eq. (reachtime) T = 0.15 s | `certified.response_time`; `strata.py::certified_params_from_config`; A20 item 1 |
| a_b 1.0 | `certified.a_brake` |
| Eq. (band) | `certified.py::reach_speed`, `FieldGeometry.band_for`, `band_index` |
| Band tops 0.10/0.30/0.51; v_cap 0.5074 covered | `certified.band_top_speeds` and comment; A20 item 3 |
| Band coherence a_max T 0.075 < 0.10 | `CertifiedParams.__post_init__` |
| Band limits 0.025, 0.225 m/s; band 0 creep/departure band | `certified.py::band_speed_limit`; `certified.band_top_speeds` comment |
| Eq. (field) | `certified.py::FieldGeometry.support` (moving branch); `pfb_field.py::PFBField.support` |
| Eq. (restfield) | `certified.py::FieldGeometry.rest_support`; `pfb_field.py::PFBField.rest_support` |
| S(n; e) formula | `pfb_field.py::SweptFootprint` docstring and `support`; `certified.py::FieldGeometry.swept_interval` |
| E(w) formula, omega_cap | `pfb_field.py::heading_reach`; `certified.py::yaw_excursion` |
| D(V) | `certified.py::band_depth`; `pfb_field.py::travel_depth` |
| sigma(c, V) | `certified.py::swerve`; `pfb_field.py::swerve_reach` |
| K(V, w) | `certified.py::coupling`; `pfb_field.py::turning_drift` |
| R | `certified.py::rest_reach`; `pfb_field.py::rest_creep_reach` |
| m_f 0.10, m_l 0.05 | `certified.margin`, `certified.lateral_margin` |
| At-rest field is the infimum; nesting terms | `FieldGeometry.support` docstring; A21 item 1 |
| Eq. (extent), 1,440 normals | `certified.py::FieldGeometry.extent`; `pfb_field.py::N_NORMALS`, `PFBField.extent` |
| Composed speed max(estimate, applied); yaw rate the larger magnitude; heading of the dominating signal | `certified.py::CertifiedLayer.step` docstring and code; A19 item 2 |
| Standstill 0.01/0.02 m/s, 0.0296/0.0592 rad/s; rotational = translational / rho_out | `certified.standstill_*`; `StandstillDetector.update`; config comment (ruled 2026-10-19) |
| Breach -> trigger; active after t_r; latched; restart delay 2.0 s | `CertifiedLayer` docstring, `step`; `certified.restart_delay` |
| Restart: whole field of the commanded twist; creep keeps the at-rest field; two approach rules | `certified.py::_restart_allowed`, `_approach_rules_permit`; A22 item 1; A21 item 2 |
| Creep for classical (recovery) and learned (stop flag in the observation) | A22 item 2 |
| Receding release, exemption, operator-reset counterpart | `certified.py::_recedes`, `step`; A24; `docs/realism_audit.md` |
| Warning field 1.5 D + 0.05 ahead, 0.02 abeam/behind/at rest; never acts; not in observation | `certified.warning_*`; `FieldGeometry.support` (warning branch); config comment |
| Grid 0.05 m; unmapped excluded from the map | `planner.resolution`; `pfbnav/world/baking.py`; DL 2026-09-23 "Map baking" |
| Lethal 0.2086 + 0.10 = 0.3086; soft linear 1 -> 0 at 0.3374 + 0.03 = 0.3674; cost length (1 + kappa_s c), kappa_s 1 | `planner.lethal_margin`, `planner.soft_margin`, `planner.soft_weight`; `baking.py::soft_cost`, `planner_radii`; `astar.py` docstring |
| 8-connected A*, octile admissible, supercover line-of-sight, named failure status | `pfbnav/planning/astar.py` docstring |
| Margins set by the oracle criterion | A18; `planner.lethal_margin` comment |
| Monotone projection, window 1.0 m, lookahead 0.8 m | `planner.track_window`, `planner.lookahead`; `pfbnav/planning/lookahead.py` docstring |
| Def. protective stop | `CertifiedLayer.step` (triggered); `StepLoop._count` |
| Def. scan-fault stop | `LoopCounters.scan_fault_stops`; `endpoints.yaml` H1.scan_faults (A25) |
| Def. contact: chassis vs all incl. floor; wheels vs all minus floor | `pfbnav/sim/contact.py::RobotContact`; DL 2026-09-17 "Collision definition widened"; hard rule 4 |
| Def. unrecoverable stop | `pfbnav/envs/reward.py::episode_outcome` |
| Eq. (sp100), pooled | `LoopCounters.stops_per_100m`; `pfbnav/eval/runner.py` docstring |

## Section 4: The Protective-Field Barrier

| Claim | Source |
|---|---|
| C1: safe set is the complement of the certified trigger set | DL 2026-10-23 "RULED: WP2" (Q1); A26 item 1; `pfb.py` docstring |
| Same inputs, same keys, independent, cross-checked in tests | Q2; `pfb_field.py` docstring; `tests/test_pfb_field.py`; `tests/test_architecture.py` |
| Regimes rest + 3 bands; one problem per regime; best feasible (PFB-(b)) | Q1; `pfb.py::PFB._regimes`, `PFB.filter` |
| PFB-(a) exploratory ablation; its QP not implemented | A26 item 1; `endpoints.yaml` exploratory H3; no implementation (`pfb.py` solves bands only): **`\todo`** |
| Eq. (barrier); theta_0, W_0 | `pfb.py::PFB._now` and `PFB.filter` (heading_now, w_now) |
| delta 0.5 mm, residual -0.27..+0.50 mm | `filter.binning_penalty_m`; Q6; TRAVERSABILITY_20260927T222532Z (config comment) |
| tau = dt + 0.05 | `filter.lag_allowance_s`; Q2; `pfb.py::PFB._rows` |
| Eq. (prediction), Eq. (predicted) | `pfb.py::PFB._rows` |
| theta(u), W(u) | `pfb.py::PFB._rows` (dominant, w_field); `PFB._assumed` |
| Eq. (cbf), clipped | `pfb.py::PFB.filter`; DL 2026-10-23 "Rule 7 entry: the PFB's CBF condition clipped at zero" |
| Deliberate departure from the standard form; reason | DL 2026-10-24 "RULED (supervisor): the WP2 progress report", item 2 |
| alpha 2.0 provisional; tuned | `filter.alpha`; Q4; ruling of 2026-10-24 item 4 |
| Eq. (qp), W = diag(1, 1, rho_out^2) | `pfb.py::PFB._solve`, `PFB._static`, `PFBConfig.weight_yaw`; Q5 |
| Eight half-spaces at 0.95 v_rim; box | `pfb.py::PFB._static` |
| Speed limit: band limit - 0.002; top v_cap; rest 0.018 m/s, yaw 0.9 x 0.0592 | `pfb.py::PFB.speed_limit`, `PFB._static`; `filter.band_speed_margin_m_s`, `filter.creep_fraction` |
| Envelope: two convex cases, holding then braking; yaw alpha_max dt; inscribed 16-gons | `pfb.py::PFB._static` docstring; `_RING` = 16; `PFB.filter` (braking loop) |
| Rest admissible only by the mirrored standstill; bands not below the estimate's | `pfb.py::PFB.standstill`, `PFB._regimes` |
| Nominal passes unchanged if feasible | `pfb.py::PFB.filter` ("nominal") |
| Linearisation at a reachable point | `pfb.py::PFB._reachable_point`, `_solve_regime` |
| Chords over +-0.05 rad/s; tangent in translation | `pfb.py::PFB._linear_rows`; `filter.yaw_trust_rad_s` |
| Tightening 0.5 mm; row window 0.5 m | `filter.linearisation_margin_m`, `filter.row_window_m` |
| Clarabel | `filter.qp_solver`; `pfb.py::PFB._solve`; DL 2026-10-24 "PFB brought inside the per-step budget" |
| True rows, no tolerance; one re-linearisation (solution or braking command) | `pfb.py::PFB._solve_regime`; `filter.linearisations` 2; Q3; DL 2026-10-23 step 3 (item 8) |
| Eq. (fallback) | `pfb.py::PFB.fallback` |
| Braking command checked before infeasible ("brake") | `pfb.py::PFB.filter`; DL 2026-10-23 step 3 (item 9) |
| Infeasibility per method and stratum | Q3; `LoopCounters.pfb_infeasible` |
| Invalid sector -> fallback (unobserved) | `pfb.py::PFB.filter` |
| Latch: releasable commands only | Q7; `pfb.py::PFB.releasable`, `PFB.filter` (latched) |
| Recovery creep passes unaltered when releasable | `tests/test_pfb.py::test_the_recoverys_creep_passes_unaltered_when_releasable` |
| Trust interval and band margin are design parameters selected on tune routes; +-50 % sensitivity after alpha | ruling of 2026-10-24 item 2 |
| Acceptance step: ideal tracker, 60 tune routes, attribution categories | DL 2026-10-23 WP2 rulings (acceptance step); `pfbnav/eval/pfb_acceptance.py`; `scripts/pfb_acceptance.py` |
| Decisive acceptance = simulator on final code | ruling of 2026-10-24 item 1 |
| Table 2 (observation) | `pfbnav/envs/observation.py` (BLOCKS, docstring); `env.observation.goal_scale` 18.0; `planner.lookahead`; `scanners.range_max`; actuation ceilings |
| Pose-agnostic; validity separate; stop flag | `observation.py` docstring |
| Action scaling by own ceilings | `pfbnav/envs/nav_env.py::NavEnv.step` (`command = a * self.ceilings`) |
| Eq. (reward), weights 5.0 / 0.01 / 5.0 / 0.1 / 0 / 200 / 200 | `pfbnav/envs/reward.py::step_reward`; `reward.*` |
| Stop charged once on trigger | `reward.py` module docstring |
| Eq. (intervention), 0 without filter, excludes overrides | `reward.py::intervention_cost`; `NavEnv.step`; ruling of 2026-10-24 item 3; DL 2026-10-24 "Rule 7 entry: the intervention term" |
| Latched zeroing counts (open) | DL 2026-10-24 same entry ("A consequence, for the supervisor"): **`\todo` my decision** |
| P = clip((0.5 - r_min)/0.5); r_min from the body origin; filter stream | `reward.py::proximity_cost`; `NavEnv._proximity`; `env.observation.proximity_threshold` |
| k_prox 0 (L5), 2.0 (L1); L4 not configured | `configs/experiments/L5_sac_pfb_unshaped.yaml`, `L1_sac_shaped.yaml`; no L4 config: **`\todo`** |
| Termination: 0.25 m; collision precedence; 4,000 steps; timeout / unrecoverable | `reward.goal_tolerance`, `reward.max_steps`; `reward.py::episode_outcome` |
| SAC hyperparameters | `training.*` |
| 300,000 steps, fixed after G2 | `training.total_timesteps` comment |
| Dropout: 0.10, 1-6 sectors, 0.01 per step, range 0 + invalid, never on filter stream | `env.observation.dropout`; `gates.yaml` layout_sets.sensor_dropout |
| Training set: 667 per stratum, rate 0.25, per-seed permutation, no curriculum | `scenario_sets.yaml` scenario_sets.train; `gates.yaml` unmapped.episode_rate.train; DL 2026-10-03 six questions (1) |

## Section 5: Experimental Design

| Claim | Source |
|---|---|
| Preregistered before training; amendments dated | `preregistration/AMENDMENTS.md`; `LOCK.json` |
| Hall 14 x 10 m, wall 0.2, shelves 0.8 deep, 1.6 tall, 2-4 rows, 0.70-0.92, alternate ends, 1.2 m end clearance, 1.5 m aisle travel | `world.layout.*` |
| Strata narrow [0.1987, 0.2687], medium [0.2687, 0.40], wide [0.40, 0.80] | `world.aisle_strata`; `endpoints.yaml` H2 population (A21) |
| Floor rule: three binned gaps (at rest centred, at rest at 0.05 offset, moving at offset in every band), 0.1687 + 0.03 | A21 item 3; `scripts/compute_narrow_floor.py` docstring; `world.aisle_strata` comment |
| Route validity by the free-orientation oracle; goal re-drawn; logged | `world.layout.validity` comment; A18 |
| Layout identity; verified at load | `gates.yaml` layout_sets.identity, verified_at |
| Seed ranges | `gates.yaml` layout_sets.seed_ranges |
| Test locked; never used in development | `gates.yaml` locked_splits; CLAUDE.md hard rule 2 |
| Tune 20/stratum at 0.25; tune_unmapped 20/stratum at 1.0; pilot 150 narrow | `scenario_sets.yaml` scenario_sets |
| Test size pending A14 | `scenario_sets.yaml` (test: pending_A14): **`\todo`** |
| Unmapped: 0.25-0.60 m, 0.80 m, 1-2, excluded from map | `gates.yaml` layout_sets.unmapped |
| Rates 0.25 / 0.25 / test 0 and 1.0; shared parameter | `gates.yaml` unmapped.episode_rate |
| Route-relative: [0, 0.6], first [0, 0.3], side uniform, arc excluding 2 m / 1 m, margin 0.15 | `scenario_sets.yaml` unmapped_placement; `gates.yaml` placement.margin_m |
| Bypass: 0.4073 m, W = 1.0 m, joint, re-sample obstacle | `scenario_sets.yaml` feasibility, bypass, on_rejection; A23/A24 text (0.4073) |
| >= 90 % engaged within 0.3 m (unmapped-static test) | `scenario_sets.yaml` validity |
| Table 3 (comparators) | `configs/methods.yaml`; `endpoints.yaml` (H1 comparators, A16); A16; A23 item 5 (L5-nowp); DL 2026-10-03 six questions (2) (L2); ruling of 2026-10-24 item 4 (B7 = B2 tuned + PFB) |
| B4, B5 retired, not reused | `configs/methods.yaml`; CLAUDE.md hard rule 20 |
| Classical set: recovery, route cap, equal tuning budget | `configs/methods.yaml` header; A23 item 4 |
| Recovery: 1.0 s after delay, 0.018 m/s, 72 directions, 0.30 m, 5 attempts | `baselines.recovery.*`; A22 item 3 |
| Route cap: fastest clear band, mapped only, yaw cap, 0.10 m lead, 0.5 m/s^2; never faster; learned none | `baselines.route_cap.*`; A23 item 4 |
| B6 decisive comparator | A26 item 2; `endpoints.yaml` H1.reporting |
| L2: stops per episode <= d, dual ascent, cost critic, unshaped, d in {0.05, 0.1, 0.2}, toy validation | DL 2026-10-03 "Rulings on six questions" (2) |
| L3 separate labelled baseline | CLAUDE.md hard rule 8; `configs/methods.yaml` |
| Alpha grid {1, 2, 5, 10, 20}, 5 pp, fewest SP100, ties by mission time, tune at training rate; frozen for L3, L4, L5, B7 | ruling of 2026-10-24 item 4 (not yet in preregistration): **`\todo`** |
| Oracle tracker: privileged schedule over all obstacles; never a comparator; reference line | A23 item 1, item 6; `baselines.oracle_tracker` comment |
| G0: six cases, 0.1 m/s, <= 1 step, 10 mm negative, two contexts | `gates.yaml` G0_contact_validation; A11 |
| G1b: profiles, 0.10-0.50 m/s, 5 m, 0.05 / 0.02 m, 5 deg, 3 reps, ceilings are output | `gates.yaml` G1b_closed_loop_tracking; A5 |
| G2: >= 25 steps/s aggregate; no over-budget steps; 0.1 % scan-fault flag | `gates.yaml` G2_compute_budget (A25) |
| G4: pilot seeds; integrity; learning 0.20 / 0.15 over 10 %; checkpoint 0.30 / 0.10 on 60 tune scenarios; controls | `gates.yaml` G4_learned_pilot (A15, A23) |
| Standing rule: confirmatory seeds never dropped | `gates.yaml` G4 standing_rule; CLAUDE.md hard rule 17 |
| Seed count | `endpoints.yaml` (5 seeds) vs DL 2026-10-03 (count by power analysis, cap 20): **inconsistency, `\todo`** |
| Horizon 4,000 steps (200 s), rule with 1.3 margin, checked over every built set | `reward.max_steps`, `reward.horizon_margin`; A23 item 2; `tests/test_horizon.py` |
| Metrics: primary; scan-fault exclusion; success and mission time beside; secondary list | `endpoints.yaml` H1 (A25, A26), exploratory.secondary; Q3 (infeasibility) |
| Reporting line (CIs, IQM, detectability floor) predates rulings | `endpoints.yaml` reporting; DL 2026-10-02 rulings: **`\todo`** |
| H1: L5 vs {B1, B2, B3, B6, L1, L2, L3}, unmapped_static, pooled, intersection-union | `endpoints.yaml` H1 |
| H2: L5 vs L4, narrow, conditions pooled and separately, >= +15 pp and significant after Holm | `endpoints.yaml` H2 |
| H1N and narrow rule: 150 per method, Clopper-Pearson one-sided 95 % upper bound >= 0.02, timing | `endpoints.yaml` H1N, narrow_rule |
| Holm, alpha 0.05, family H1, H2 (+H1N) | `endpoints.yaml` alpha, confirmatory_family |
| Exploratory H3-H6 and secondary list | `endpoints.yaml` exploratory (A3, A8, A16, A17, A26); A23 item 5 (L5-nowp) |
| Statistical test | `endpoints.yaml` (Wilcoxon) vs DL 2026-10-02/03 (rate ratio, bootstrap): **inconsistency, `\todo`** |
| Pairing "one process" vs "one scene load" | `endpoints.yaml` pairing vs DL 2026-10-14 ruling and CLAUDE.md: **`\todo`** |
| Engine stated with every result | A8 item 1; `endpoints.yaml` reporting |

## Appendix A: Notation

The values are those traced above.

## Appendix B: Verification findings

| Claim | Source |
|---|---|
| B1: chassis-only check; wheels 17 mm proud (0.1789 vs up to 0.1970) | DL 2026-09-17 "Open: the contact check watches the chassis shape only", "Collision definition widened to chassis + wheels" |
| Wheel spheres not collidable; made collidable; bit-identical regression; half-width 0.2086 | DL 2026-09-18 "Rulings on the WP0 results report: step 1 tooling", "Ruling 1 done"; `robot.half_width` comment |
| G0 six cases incl. raised housing, both contexts, three seeds, PASS | A11; CLAUDE.md (G0 PASSED with six cases, 2026-09-28); `runs/gates/G0_20260928T22*` |
| Guards | `tests/test_collection_semantics.py`, `tests/test_contact.py`, `tests/test_contact_sim.py` |
| B2: pods at +-0.325 m, rays meet 0.563 m, field of the time about 0.26 m abeam; validity by bearing from origin; walls >= 0.6 m | DL 2026-09-21 "The scanners leave a blind zone beside the robot" |
| Position-aware validity; S300 layout; casing defect (0.0094 m) | DL 2026-09-21 "Position-aware validity"; DL 2026-09-22 "Defect: every sensor blinded by its own casing" |
| 7,680 observed; 0.134 m^2 measured vs 0.146 m^2 geometric | DL 2026-09-22 "S300 coverage map"; COVMAP summaries (`unobserved_area_m2` 0.1462) |
| 9.6 mm half-thickness artefact; -0.27..+0.50 mm | `world.narrow_floor_slack` comment; CLAUDE.md hard rule 15 |
| Guards | `tests/test_coverage.py`, `tests/test_coverage_map.py`, `tests/test_scanner_client.py`, `tests/test_d2_real_scan.py` |
| B3: engine numbers; MuJoCo 2.21 m timed out; factor of about forty | DL 2026-09-18 "D7 engine sweep" |
| Reload moved trajectories up to 1.8 mm; determinism per scene load | CLAUDE.md conventions (ruled 2026-10-14) |
| Frozen ODE 0.005 s; 0.64 deg spread; Newton external validity | `sim.physics.*`; A8 |
| Guard | `tests/test_frozen_physics.py` |
| B4: field shrank, > 80 % of bearings, up to 62.6 mm | `certified.lateral_margin` comment (corrected 2026-09-27) |
| A19 rotation; A20 B1 45/51, twice the sweep; A21 3.54 mm; A22 9-11 mm shell; A24 14 unrecoverable | A19, A20 "Why", A21 "Why", A22 "Why", A24 "Why" |
| Guards | `tests/test_certified_layer.py`, `tests/test_certified_rotation.py`, `tests/test_containment.py`, `tests/test_containment_sim.py`, `tests/test_nesting.py`, `tests/test_restart_rigid.py`, `tests/test_receding_release.py`, `tests/test_receding_release_sim.py` |
| B5: one stationary trigger in the zero-action control; 0/20 normal, 1/20 loaded; 104-114 of 120 empty; phantoms 0.30-0.47 m; bias argument | A25 "Why" |
| Guards: fault injection; 198 good steps; 3,196 moving steps | A25 item 4; `tests/test_scan_guard.py`, `tests/test_scan_rescan_sim.py` |

## Appendix C: Realism audit

| Claim | Source |
|---|---|
| Every row of Table A2 | `docs/realism_audit.md` (first pass, 2026-10-20), condensed; "open" rows kept as `\todo` |
| Braking authority 2.3-3.0 m/s^2 | `docs/realism_audit.md`; `actuation` comment (D9) |
| Withdrawal of 0.12 m/s; 48/60; median 0.37 m/s | ruling of 2026-10-24 ("Record"); `runs/analysis/PFB_ACCEPTANCE_20261024T174707Z_seed0/routes.json` (median computed by script, 2026-10-24) |
| Rotation-sweep row's "about 0.2 rad/s" | `docs/realism_audit.md`; rests on the same oracle analysis: **`\todo` my decision** |

## Figures

| Figure | Script | Inputs |
|---|---|---|
| 1 | `paper/figures/scripts/fig1_architecture.py` | `configs/base.yaml`; `pfb_field.py::field_params_from_config` |
| 2 | `fig2_field_geometry.py` | `pfb_field.py::PFBField`; `robot.footprint_table`; config |
| 3 | `fig3_coverage.py` | the two COVMAP runs (cells.csv, summary.json, provenance.json); footprint table of each run's scene (git at the run's commit for the pod era) |
| 4 | `fig4_layout.py` | `scenarios/tune/*.json` (first scenario per stratum with two unmapped obstacles), `world_of`, `bake`, `plan_from_config` |
| 5 | `fig5_pfb_step.py` | emulator (`emulate_layout`, sector scan at the measured ray density), `scenarios/tune/tune_narrow.json` index 0, `pfb.py` |
