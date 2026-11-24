# Experiments

Two stages, one question: does putting the PFB between a baseline and the certified layer remove protective
stops? Both run on the training host against CoppeliaSim and are started by the researcher. Both use the
tune set only (`scenarios/tune/`); the test sets are never evaluated during development.

Each baseline's parameters live in `configs/baselines/<id>.yaml` (the only place they are written;
`configs/base.yaml` lists the files under `baseline_files`).

| Baseline | `--controller` | Parameters |
|---|---|---|
| B1 Pure Pursuit | `pure_pursuit` | `configs/baselines/B1_pure_pursuit.yaml` |
| B2 Regulated Pure Pursuit | `regulated_pure_pursuit` | `configs/baselines/B2_regulated_pure_pursuit.yaml` |
| B3 holonomic DWA | `dwa` | `configs/baselines/B3_dwa.yaml` |
| B6 field-aware MPPI | `field_aware_mppi` | `configs/baselines/B6_field_aware_mppi.yaml` |

## Stage 0: the baseline ladder (is each baseline working at all?)

Run every baseline alone, in this order, and stop at the first rung where one fails:

1. **Empty hall** (perimeter walls only). Build once, then evaluate with the empty-hall config, which is
   what rebuilds the same world:
   ```powershell
   python scripts/build_empty_hall_set.py --config configs/worlds/empty_hall.yaml --seed 0
   python scripts/evaluate.py --config configs/worlds/empty_hall.yaml --seed 0 --controller <ctrl> --sets scenarios/free/empty_hall.json --draw-path
   ```
2. **Mapped obstacles only** (shelves the planner knows; no unmapped obstacle):
   ```powershell
   python scripts/make_subset.py scenarios/tune/tune_wide.json --mapped-only --out runs/scratch/wide_mapped.json
   python scripts/evaluate.py --config configs/base.yaml --seed 0 --controller <ctrl> --sets runs/scratch/wide_mapped.json --draw-path
   ```
   (repeat with `tune_medium.json` and `tune_narrow.json`).
3. **Unmapped obstacles** (every route has one): `scenarios/tune/tune_unmapped_narrow.json`,
   `tune_unmapped_medium.json`, `tune_unmapped_wide.json`.

A baseline that cannot cross the empty hall is broken before any obstacle is to blame. Run these on the
training host: on a loaded laptop the scan render fails under load and the robot is frozen by the fail-safe
(see `docs/manual_tuning.md`).

## Stage 1: each baseline alone

```powershell
python scripts/evaluate.py --config configs/base.yaml --seed 0 --controller <ctrl> --sets scenarios/tune/tune_narrow.json scenarios/tune/tune_medium.json scenarios/tune/tune_wide.json
```

## Stage 2: the same baseline with the PFB beneath it

The same command with `--pfb on`. It changes `filter.enabled` and nothing else (`pfbnav/integration.py`,
`tests/test_pfb_integration.py`); the PFB runs at the frozen alpha, and the run directory is named
`<controller>_pfb_...`. `configs/experiments/B7_rpp_pfb.yaml`, `B8_mppi_pfb.yaml`, `B9_dwa_pfb.yaml` and
`B10_pp_pfb.yaml` are the same switch as config files (B2, B6, B3 and B1 respectively).

```powershell
python scripts/evaluate.py --config configs/base.yaml --seed 0 --controller <ctrl> --pfb on --sets scenarios/tune/tune_narrow.json scenarios/tune/tune_medium.json scenarios/tune/tune_wide.json
```

## Pairing

Run a baseline's two members back to back on the same host with the same seed and sets; determinism holds
within one scene load, so the pair is a comparison on the same scenarios (stops per 100 m, success, mission
time, contacts). Eight runs in all: four controllers x {alone, `--pfb on`}.

## Results

Validation sets (`scenarios/validation/`, 30 routes per cell, 90 mapped-only and 90 with an unmapped obstacle per row), seed 0,
commit `195519c` (the route-30 rerun of B6 + PFB at `c9e6909`, a results-only commit), host DESKTOP-9EL975R, exploratory.
Run folders are under `experiments/results/`. Contacts and scan faults: zero in every row. "Stops/100 m" is per 100 m of planned route.
Success and timeouts are reported beside stops: a baseline can have no stops because it never moves.

**Mapped obstacles only (90 routes)**

| Baseline | Success alone | Success + PFB | Timeouts alone / + PFB | Unrecoverable alone / + PFB | Stops/100 m alone | Stops/100 m + PFB |
|---|---|---|---|---|---|---|
| B1 | 0 | 90 | 0 / 0 | 90 / 0 | 68.3 | 0.0 |
| B2 | 34 | 68 | 10 / 22 | 46 / 0 | 63.3 | 0.43 |
| B3 | 85 | 90 | 0 / 0 | 5 / 0 | 8.35 | 0.0 |
| B6 | 78 | 76 | 12 / 14 | 0 / 0 | 0.96 | 0.86 |

**With an unmapped obstacle on every route (90 routes)**

| Baseline | Success alone | Success + PFB | Timeouts alone / + PFB | Unrecoverable alone / + PFB | Stops/100 m alone | Stops/100 m + PFB |
|---|---|---|---|---|---|---|
| B1 | 0 | 38 | 0 / 52 | 90 / 0 | 62.5 | 1.2 |
| B2 | 0 | 2 | 4 / 88 | 86 / 0 | 64.0 | 1.6 |
| B3 | 1 | 6 | 75 / 84 | 14 / 0 | 45.7 | 0.0 |
| B6 | 49 | 44 | 41 / 46 | 0 / 0 | 0.60 | 0.70 |

Reading: the PFB removes the protective stops and every unrecoverable stop for B1, B2 and B3. On mapped routes success rises
with it (B1 0 to 90, B2 34 to 68, B3 85 to 90). On unmapped routes the stops go but the routes mostly end in timeouts: the
filter does not make a baseline route around an obstacle it cannot see. B6 builds the certified field into its own cost, has
almost no stops to remove, and shows no difference beyond noise (12 of 90 mapped routes change outcome, 7 one way and 5 the other).

Provenance notes: B6 + PFB has no single complete run. Its mapped set comes from four folders (29 routes of mapped narrow from
`B6_field_aware_mppi_pfb_20261123T224259Z_seed0`, which crashed on route 30; route 30 from
`..._20261124T143103Z_seed0_route30`; mapped medium `..._20261124T105359Z_seed0`; mapped wide `..._20261124T105410Z_seed0`).
Its unmapped row is the 90 complete episodes of the crashed first folder. `..._20261124T105346Z_seed0` crashed at route 25 and
repeats the first 25 episodes exactly; it is not counted. All runs read `dirty: true` (deleted files under `paper/`). Different
scene loads are not bit-identical to each other; routes repeated across loads matched exactly.
