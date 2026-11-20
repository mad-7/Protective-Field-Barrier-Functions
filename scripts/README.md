# Scripts

Every script takes `--config` and `--seed` and writes `runs/<kind>/<name>/` with provenance. Scripts that
drive the simulator or train are run by the researcher on the training host.

| Stage | Script | Does |
|---|---|---|
| Data | `build_scenario_sets.py` | Build the tune, pilot and test scenario sets; lock the test sets |
| Data | `compute_narrow_floor.py`, `route_schedules.py` | Narrow-aisle floor and per-route speed schedules |
| Tune | `tune_classical.py` | Tune B1, B2, B3 or B6: seeded random search, N = 24 |
| Tune | `tune_alpha.py` | Tune the PFB gain alpha with B7 |
| Train | `train.py` | Train one SAC policy: one experiment config, one seed |
| Train | `validate_l2_toy.py` | L2's toy positive control |
| Evaluate | `evaluate.py` | Evaluate one controller (checkpoint or classical baseline) on scenario sets |
| Evaluate | `evaluate_test.py` | The single sealed test-set evaluation |
| Gates | `gate_g2_throughput.py`, `gate_g4.py` | Throughput gate; a learned method's pilot gate |
| Check | `measure_realtime.py` | Per-step compute of every controller against the 50 ms criterion |
