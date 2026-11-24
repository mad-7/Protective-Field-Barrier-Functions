# Run Requests for the classical-controller paper (2026-11-24)

Paper: `paper/latex_classical/` (field-blind B1, B2, B3). The researcher runs these (hard rule 11); PowerShell, one command per
line, from the repository root, on the host whose results the paper will cite. Every run records its commit, host and scene hash.
Commit the tree first: the runs record the dirty flag.

## RR1 - Constant-margin comparator (paper Section 6.4, and 4.4)

**Purpose:** is the speed- and direction-dependent field what removes the stops, or does any beam-wise margin do it? Exploratory.

**Narrow-aisle check of the margin.** Margin 0.10 m (declared 2026-11-24, lowered from 0.15 m). The footprint's lateral
extent is 0.2095 m, so the comparator's field abeam is 0.3095 m; the narrowest aisle has a wall 0.2095 + 0.1987 = 0.4082 m from the
centreline when the robot is centred, a slack of 98.7 mm. The committed tracking bound is 50 mm, so a robot displaced by that bound
still has the wall clear of the field. (At the first-declared 0.15 m the slack was 48.7 mm, under the bound, which is why it was lowered.) The
arithmetic is mine, from `configs/base.yaml` and the footprint table; confirm it with the traversability probe if you want it measured.

**Preconditions:** CoppeliaSim running on the configured port with `scenes/warehouse_v2.ttt` (hash pinned in `configs/base.yaml`);
`configs/experiments/CM_constant_margin.yaml` committed (margin 0.10 m, frozen alpha); the controllers' tuned parameters unchanged.

**Commands** (validation sets, seed 0, same sets and order as the paired runs in `experiments/README.md`; one per controller, B3 first):
```
python scripts/evaluate.py --config configs/experiments/CM_constant_margin.yaml --seed 0 --controller dwa --display off --sets scenarios/validation/validation_mapped_only_narrow.json scenarios/validation/validation_mapped_only_medium.json scenarios/validation/validation_mapped_only_wide.json scenarios/validation/validation_unmapped_narrow.json scenarios/validation/validation_unmapped_medium.json scenarios/validation/validation_unmapped_wide.json
python scripts/evaluate.py --config configs/experiments/CM_constant_margin.yaml --seed 0 --controller regulated_pure_pursuit --display off --sets <the same six sets>
python scripts/evaluate.py --config configs/experiments/CM_constant_margin.yaml --seed 0 --controller pure_pursuit --display off --sets <the same six sets>
```
(`--display off` is from `docs/run_requests.md`; drop it if `evaluate.py` does not take it on your checkout.)

**Duration and resources:** each run is 180 episodes of up to 200 s of simulated time; compare the wall time of the matching `*_pfb_*` runs
in `experiments/results/`. One simulator instance each, CPU, a few hundred MB of disk.

**Outputs:** `runs/eval/<controller>_<stamp>_seed0/` with `episodes.jsonl`, `summary.json`, `provenance.json`, `ring/`.

**Watch:** `pfb_infeasible` and the braking share in the narrow stratum (the narrow-aisle check above); a controller frozen at the start;
the simulator crashing (rerun the same command and say so).

**Send back:** the three run directories.

## RR2 - Gain sensitivity (paper Section 6.3)

**Purpose:** the stops of B2 + PFB at alpha in {1, 2, 5, 10, 20} 1/s on the tuning scenarios. Development-set evidence.

**Preconditions to check first:** `scripts/tune_alpha.py` requires `--b2-tuning` (B2's tuning `summary.json`) and `--g2` (a G2 summary for the
host and instance count); neither is in this working tree, and the script also SELECTS an alpha (the frozen alpha 1.0 stays; a selection
at the grid edge is reported). Confirm that it still reads `preregistration/endpoints.yaml` as written and that its per-scenario results can
be tabulated as a sensitivity table. If not, tell me and I will write a sensitivity-only script.

**Command:**
```
python scripts/tune_alpha.py --config configs/base.yaml --seed 0 --b2-tuning <runs/tuning/B2_..._seed0/summary.json> --g2 <runs/gates/G2_<stamp>_seed9000000/summary.json> --instances <k>
```
**Outputs:** the run directory the script prints, with `summary.json` (scores per grid value, recovery). **Send back:** that directory.

## RR3 - Computation time (paper Section 6.6)

**Purpose:** the 95th-percentile decision time of the filter on the final host against the 50 ms period. No simulator.

**Command:**
```
python scripts/measure_realtime.py --config configs/base.yaml --seed 0 --parts pfb classical
```
**Outputs:** the run directory it prints. **Send back:** its summary. Run it on the host the paper cites, with nothing else running.

## Offline items (no run)

- Classifier: done for the three filtered validation runs; output in `experiments/results/filtered_outcome_classes.json`.
  Still to run on the sealed-test runs of B1, B2 and B3, which are not in this working tree:
  `python scripts/classify_filtered_outcomes.py <runs/test/CLASSICAL_shard*_..._seed0 ...>` (its folders are per shard; pass them together).
- HF-c calibration bound and power (paper Section 5.6): read from `runs/test/HFC_CAL_20261107T231644Z_seed0` and the power analysis in `runs/power/`.
