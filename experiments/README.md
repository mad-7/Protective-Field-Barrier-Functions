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

Copy each finished run's `summary.json` and `provenance.json` into `experiments/results/<id>_<alone|pfb>/`
and fill the table below. Nothing here has been run yet.

| Baseline | Success alone | Success + PFB | Stops/100 m alone | Stops/100 m + PFB | Contacts alone / + PFB |
|---|---|---|---|---|---|
| B1 | | | | | |
| B2 | | | | | |
| B3 | | | | | |
| B6 | | | | | |
