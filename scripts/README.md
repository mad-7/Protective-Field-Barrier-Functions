# Scripts, by purpose

Every script takes `--config` and `--seed` and writes its own `runs/<kind>/<name>/` with provenance. Each
one's docstring says what it does, what it refuses and what it writes. Scripts that drive the simulator
or train are run by the researcher on the training host (`CLAUDE.md`, "Long-duration operations").
New readers need only groups 1-3; group 6 is kept so every recorded finding can be reproduced.

## 1. The study pipeline (what produces the results)

| Script | Does |
|---|---|
| `train.py` | Train one SAC policy: one experiment config, one seed |
| `evaluate.py` | Evaluate one controller (a checkpoint or a classical baseline) over scenario sets |
| `tune_classical.py` | Tune a classical baseline (B1, B2, B3, B6): seeded random search, N = 24 |
| `tune_alpha.py` | Tune the PFB's gain alpha with B7; compare B2 and B7 on the tune set |
| `build_scenario_sets.py` | Build the tune, pilot and test scenario sets; lock the test sets |
| `freeze_prereg.py` | Freeze / relock the preregistration after an amendment |
| `measure_realtime.py` | Per-step compute of every controller against the 50 ms criterion |

## 2. Gates (pass/fail checks preregistered in `preregistration/gates.yaml`)

| Script | Gate |
|---|---|
| `gate_g0_contact.py` | G0: the ground-truth contact check |
| `gate_g1_fidelity.py` | G1: holonomic motion fidelity |
| `gate_g1b_tracking.py`, `aggregate_g1b.py` | G1b: closed-loop path tracking, and its aggregation |
| `gate_g2_throughput.py` | G2: training-host throughput with 1-N parallel simulators |
| `gate_g4.py` | G4: a learned method's pilot seed (learning, checkpoint, integrity, controls) |

## 3. Statistics and power

| Script | Does |
|---|---|
| `power_a14b.py` | **Current:** H1's power on the rate x ratio grid at K = 10 (A14b) |
| `power_corrected.py` | The corrected two-level calibration and power (H1, H2) |
| `power_h1.py`, `power_h1_rate_ratio.py` | Earlier power analyses (superseded, kept for the record) |

## 4. Read-only monitors (seconds, no simulator)

| Script | Reads |
|---|---|
| `peek_training.py` | A training run's outcomes per curriculum stage and G4's learning criterion |
| `peek_tuning.py` | A tuning run's candidates so far (score, moving speed, latched share) |
| `resummarize_eval.py` | A finished evaluation under the current summary (both denominators, compute) |
| `diag_training_decline.py` | A training run's success over time and its replay-buffer composition |
| `classify_b7_timeouts.py` | B7's timeouts classified against the oracle |
| `compare_route_speeds.py` | Two controllers' driving speed on the same routes |

## 5. Current diagnostics and controls (short simulator runs)

| Script | Does |
|---|---|
| `trace_episodes.py` | **Current:** per-step traces of chosen episodes (wandering, stop causes, B1's latches) |
| `diag_training_run.py` | A training run from its logs, optionally a short replay of its checkpoints |
| `check_training_memory.py`, `check_memory.py` | Memory growth in training / across resets |
| `validate_l2_toy.py` | L2's toy positive control |
| `pfb_acceptance.py`, `pfb_sim_acceptance.py` | The PFB's acceptance step, emulated and in the simulator |
| `diag_episode_trace.py` | Older per-step trace for B1 and the zero-action control |
| `diag_plot_map.py` | Draw a baked map and a plan over it (offline) |

## 6. Historical: probes, scene edits and one-off investigations

Kept so that every finding in `docs/decision_log.md` can be reproduced; not needed to understand or run
the method. Each docstring names the ruling or question it answered.

- **Robot and sensors:** `probe_footprint.py`, `build_footprint_table.py`, `report_pods.py`,
  `probe_scanner_layout.py`, `diag_scanner_check.py`, `diag_scanner_cost.py`,
  `diag_scanner_resolution.py`, `diag_self_occlusion.py`, `diag_coverage_map.py`, `diag_braking.py`,
  `diag_wheel_torque.py`, `check_acceleration.py`, `probe_dynamics_step.py`, `rescore_a10_drift.py`.
- **Step cost:** `diag_step_floor.py`, `profile_step_cost.py`, `check_step_budget.py`.
- **Scene edits and regressions:** `scene_extend_floor.py`, `diag_scene_inventory.py`, `regress_floor.py`,
  `regress_collidable_flag.py`, `regress_display_off.py`, `g1b_regression.py`.
- **Fields, margins, traversability:** `probe_nesting.py`, `assess_reach_admissibility.py`,
  `compute_narrow_floor.py`, `probe_traversability.py`, `quantify_margins.py`, `quantify_oracle.py`,
  `quantify_oracle_free.py`, `report_margins.py`, `route_schedules.py`, `compare_mission_time.py`,
  `compare_rebuilt_sets.py`.
- **Recovery and early B1 failures:** `check_recovery_speed.py`, `control_recovery_synthetic.py`,
  `search_recovery_scene.py`, `diag_recovery_corner.py`, `reemulate_b1_traces.py`, `relabel_traces.py`,
  `diag_stationary_repro.py`. (`control_recovery_synthetic.py` builds B1 from the config: since B1's
  tuning (2026-11-06) that is the TUNED B1, not the recorded control.)
