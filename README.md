# pfb-nav

Protective-Field Barrier (PFB): a control-barrier-function filter placed between a local controller and a
certified safety layer on an industrial mobile robot (KUKA youBot, CoppeliaSim, ZeroMQ Remote API).
Primary metric: protective stops per 100 m.

Start with `docs/theory/` (certified layer, CBF, PFB, architecture, baselines).

## Setup
```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements-dev.txt   # pinned versions (training host: requirements.txt)
pip install -e . --no-deps
pytest -m "not sim"
```
CoppeliaSim 4.10 (headless) is needed for `pytest -m sim`, training and evaluation.

## Layout
```
pfbnav/            the package
  safety/            certified.py (certified layer), pfb.py + pfb_field.py (the PFB), reach.py, unobserved.py
  control/           B1 pure_pursuit, B2 regulated_pure_pursuit, B3 dwa, B6 mppi, SAC (L1-L5), recovery
  planning/ world/   A* planner; layouts, scenarios, splits
  envs/ sim/         Gymnasium environment; ZeroMQ client, robot, scanner
  training/ eval/    reward shaping, curriculum; runner, metrics, statistics
  utils/ config.py   run directories and provenance; config loading
configs/
  base.yaml          every physical and tuned parameter (single source of truth)
  baselines/         per-baseline parameter files (mirror base.yaml) and shared recovery/route cap
  experiments/       one overlay per method (B7-B10 = baseline + PFB; L1-L5 learned)
  tuning/ gates/     tuning search spaces; gate overlays
  methods.yaml       every method ID, active or retired
scripts/             train, evaluate, tune, gates (see scripts/README.md)
experiments/         the two-stage plan: each baseline alone, then with the PFB
scenarios/ scenes/   train / tune / pilot / test layouts (test frozen); the CoppeliaSim scene
lua/                 CoppeliaSim child scripts
preregistration/     frozen gates, endpoints and amendments
tests/               fast suite (`-m "not sim"`) and simulator tests (`-m sim`)
docs/                theory/, decision_log.md, requirements.md, reproducibility.md, archive/
runs/                outputs, never overwritten, not tracked
```

## Workflow
1. `python scripts/tune_classical.py ...`: tune a baseline on the tune set.
2. `python scripts/evaluate.py --config configs/base.yaml --seed 0 --controller <ctrl> --sets ...`: baseline alone.
3. Same command with `--config configs/experiments/B<n>_..._pfb.yaml`: the baseline with the PFB.
4. `python scripts/train.py --config configs/experiments/L5_sac_pfb_unshaped.yaml --seed <n>`: learned arms.

See `experiments/README.md` for the full table.
