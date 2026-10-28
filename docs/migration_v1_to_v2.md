# v1 → v2 Migration Map

Migrate a module only when a work package needs it, with a test written during the move.

| v1 component | v2 destination | Action | Notes |
|---|---|---|---|
| A* planner (8-connected, LOS simplification) | `pfbnav/planning/astar.py` | Migrate (WP1) | Add known-answer tests; replace try/except fallback with explicit failure status |
| Lookahead / arc-length tracking | `pfbnav/planning/lookahead.py` | Migrate (WP1) | Pure functions; test projection and clamping |
| `bake_occupancy_map.py` (AABB) | `pfbnav/world/baking.py` | Rewrite (WP1) | Exact footprints; unmapped-obstacle exclusion |
| Mecanum kinematics + sign vector | `pfbnav/sim/robot.py` | Migrate (WP0) | Add acceleration limiter; re-verify signs |
| `check_directions.py` | `scripts/gate_g1_fidelity.py` | Supersede (WP0) | Extend to lateral/diagonal profiles |
| Hokuyo Lua child script (36 bins) | `lua/scanner_v2.lua` | Rewrite (WP1) | Dual scanners, 36 + 120 bins, bearings, validity |
| Gymnasium env (45-D, unicycle) | `pfbnav/envs/nav_env.py` | Rewrite (WP1) | Holonomic, contact ground truth, reward v2 |
| `generate_scenarios.py` | `pfbnav/world/scenarios.py` | Rewrite (WP1) | Layout splits, clearance strata |
| Baselines (PP, DWA) | `pfbnav/control/` | Rewrite (WP3) | Holonomic PP, RPP, 3-D DWA |
| `eval_runner.py`, `analyze_results.py`, `cross_seed_analysis.py` | `pfbnav/eval/` | Migrate + extend (WP3–WP5) | New metrics; detectability floors; null-tested |
| `examples/train.py` | `scripts/train.py` | Rewrite (WP4) | Config-driven; uses `create_run_dir` |
| `saved_models/` | — | Do not migrate | Remain at tag `v1-chapter-results` |
