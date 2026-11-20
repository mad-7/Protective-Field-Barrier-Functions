# Onboarding: reading this project in an hour

This page is the entry point for a new reader (a collaborator, a reviewer, an examiner). It changes
nothing in the code. For the rules that govern changes, see `CLAUDE.md`; for why each decision was
made, `docs/decision_log.md`.

## 1. The project in one paragraph

A KUKA youBot (a mecanum, holonomic base) navigates warehouse aisles in the CoppeliaSim simulator. A
global planner (A*) gives a route; a local controller follows it. Beneath every controller sits a
**certified safety layer**, an emulation of an industrial safety laser scanner: speed-dependent
protective fields that trigger a **protective stop** (a latched halt) whenever something enters them.
Stops are safe but costly, so the primary metric is **protective stops per 100 m of planned route**.
The contribution is the **Protective-Field Barrier (PFB)**: a control barrier function built on the
LiDAR scan whose safe set is the same speed- and direction-dependent field. It filters the controller's
command so the robot slows or slides instead of entering the field. The main question (H1): does a
policy trained with reinforcement learning *with the PFB in the loop* (L5) trigger fewer protective
stops than every comparator, classical and learned?

## 2. The pipeline, one control step (20 Hz)

```
scenario (layout + start/goal)            pfbnav/world/       scenarios/*.json
   |
A* global route on the MAPPED map         pfbnav/planning/astar.py
   |
route view: lookahead, goal, route ahead  pfbnav/planning/lookahead.py, pfbnav/envs/route_view.py
   |
controller -> requested command           pfbnav/control/  (SAC policy, or a classical baseline)
   |
PFB filter (if enabled) -> filtered cmd   pfbnav/safety/pfb.py, pfb_field.py
   |
certified layer: fields, latched stops    pfbnav/safety/certified.py
   |
simulator: base, scanners, contact        pfbnav/sim/  (+ lua/scanner_v2.lua server-side)
   |
observation, reward, outcome              pfbnav/envs/observation.py, reward.py, nav_env.py
```

`pfbnav/envs/step_loop.py` runs one step through the bottom half (PFB, certified layer, simulator);
`pfbnav/envs/nav_env.py` is the Gymnasium environment around it. The PFB and the certified layer never
import each other (enforced by a test): the safety layer is independent of the method under test.

## 3. The methods compared (`configs/methods.yaml`)

| ID | What | Role |
|---|---|---|
| **L5** | SAC trained with the PFB in the loop, no proximity shaping | **the proposed method** |
| L1 | SAC with proximity shaping, no filter | H1 comparator |
| L2 | SAC-Lagrangian (stops as a constraint) | H1 comparator |
| L3 | L1's policy with the PFB added afterwards | H1 comparator (post-hoc filtering) |
| L4 | L5 plus proximity shaping | H2 comparator |
| B1 | Pure Pursuit | H1 comparator |
| B2 | Regulated Pure Pursuit | H1 comparator |
| B3 | Dynamic Window Approach | H1 comparator |
| B6 | Field-aware MPPI | H1 comparator (the strongest) |
| B7 | B2 + the PFB, no learning | H1prime, H5 |

Learned arms share one set of SAC hyperparameters; classical arms were each tuned with the same budget
(24 candidates). Retired IDs (B4, B5) stay listed and are never reused.

## 4. The 15 files that matter

| File | What it is |
|---|---|
| `pfbnav/safety/pfb.py` | **The PFB**: the barrier rows and the quadratic program that filters a command |
| `pfbnav/safety/pfb_field.py` | The continuous, speed- and direction-dependent protective field |
| `pfbnav/safety/certified.py` | The certified-layer emulator: banded fields, latched stops, restart rule |
| `pfbnav/envs/step_loop.py` | One control step: PFB slot, certified layer, simulator |
| `pfbnav/envs/nav_env.py` | The Gymnasium environment (reset, step, outcome) |
| `pfbnav/envs/observation.py`, `reward.py` | What the policy sees and how it is rewarded |
| `pfbnav/control/sac.py` | SAC: building the learner, and a checkpoint as a controller |
| `pfbnav/control/regulated_pure_pursuit.py`, `dwa.py`, `mppi.py` | The classical baselines (B2, B3, B6) |
| `pfbnav/control/recovery.py` | The declared recovery every classical baseline gets after a stop |
| `pfbnav/world/layouts.py`, `scenarios.py` | Generating warehouse layouts and scenario sets |
| `pfbnav/eval/runner.py` | Running episodes and summarising them (the stop rates) |
| `pfbnav/eval/rate_ratio.py` | The statistical analysis of H1 (rate ratio and its interval) |
| `configs/base.yaml` | Every physical and ruled parameter, with its unit and source |
| `preregistration/endpoints.yaml`, `gates.yaml` | The frozen hypotheses, analyses and gates |
| `scripts/train.py`, `scripts/evaluate.py` | The two entry points used most |

Everything else in `pfbnav/` is support: simulator I/O (`sim/`), geometry and set building (`world/`),
checks and diagnostics (`eval/`), training wrappers (`training/`: curriculum, stall truncation,
logging).

## 5. Running it

```
pip install -e ".[dev]"
pytest -m "not sim"                       # the fast suite, no simulator (~15-20 min)
python scripts/train.py --config configs/experiments/L5_sac_pfb_unshaped.yaml --seed 9000002
python scripts/evaluate.py --config configs/experiments/L5_sac_pfb_unshaped.yaml --seed 0 \
    --controller sac --checkpoint runs/train/<run>/final_model.zip \
    --sets scenarios/tune/tune_narrow.json scenarios/tune/tune_medium.json scenarios/tune/tune_wide.json
```

Every experiment is one YAML in `configs/experiments/` that overrides `configs/base.yaml`. Every run
writes `runs/<kind>/<name>/` with its config, its commit, its host and the scene's hash
(`provenance.json`), so any result can be traced to the code that made it. `scripts/README.md`
groups the scripts by purpose.

## 6. Why the project looks heavy

It is a **preregistered** study: hypotheses, analyses and gates were frozen before any result
(`preregistration/`), and every later change is a dated, justified amendment. That discipline explains
most of what a newcomer finds surprising:
- **Many guards and checks.** Scripts refuse test layouts during development, an unfrozen environment,
  undeclared seeds, an alpha other than the frozen one. Each refusal exists because a rule requires it.
- **Freezes by hash.** The environment's code is identified by the hash of its import closure
  (`pfbnav/envs/freeze.py`), the scenario sets by theirs; moving or renaming a module changes those
  hashes. That is why the code is NOT reorganised now, even where it could be tidier.
- **Many scripts.** Most are one-off probes and diagnostics from past investigations, kept so every
  recorded finding can be reproduced (`scripts/README.md`, "historical").
- **A long decision log.** It is the project's memory: every ruling, measurement and finding, dated.

## 7. Reading order

1. This page, then `configs/methods.yaml` (who is compared) and `preregistration/endpoints.yaml` (what is
   tested and how).
2. `pfbnav/safety/pfb.py` and `pfbnav/envs/step_loop.py` (the method and where it acts).
3. `CLAUDE.md`, "Current work package": the plan of record and the state of every stage.
4. `docs/decision_log.md`: read the LAST entries for where things stand; search it for a module or a
   result name when you want to know why something is the way it is. It is not meant to be read
   front to back.
5. `preregistration/AMENDMENTS.md` for the history of the frozen design (A1 to the latest).
