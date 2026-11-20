# Experiments

Two independent stages. Both run on the **training host** against CoppeliaSim and are started by the
researcher (CLAUDE.md, "Long-duration operations"). Both use the tune set only (hard rule 2).

## Stage 1: each baseline alone (PFB off)

| Baseline | Config | `--controller` |
|---|---|---|
| B1 Pure Pursuit | `configs/base.yaml` | `pure_pursuit` |
| B2 Regulated Pure Pursuit | `configs/base.yaml` | `regulated_pure_pursuit` |
| B3 holonomic DWA | `configs/base.yaml` | `dwa` |
| B6 field-aware MPPI | `configs/base.yaml` | `field_aware_mppi` |

Parameters: `configs/baselines/<id>.yaml`, the only place they are written; `configs/base.yaml` lists the files
under `baseline_files` and `load_config` merges them.

## Stage 2: the same baselines with the PFB beneath them

| Method | Config | `--controller` |
|---|---|---|
| B10 = B1 + PFB | `configs/experiments/B10_pp_pfb.yaml` | `pure_pursuit` |
| B7 = B2 + PFB | `configs/experiments/B7_rpp_pfb.yaml` | `regulated_pure_pursuit` |
| B9 = B3 + PFB | `configs/experiments/B9_dwa_pfb.yaml` | `dwa` |
| B8 = B6 + PFB | `configs/experiments/B8_mppi_pfb.yaml` | `field_aware_mppi` |

Each overlay differs from base in exactly `experiment` and `filter.enabled` (`tests/test_pfb_integration.py`).
The pairs share a scene load, the same scenarios and the same seed, so stage 1 vs stage 2 is a paired
comparison on stops per 100 m, success and mission time.

## Commands

One run per line (`<cfg>` and `<ctrl>` from the tables; seed 0). Outputs go to `runs/eval/...` with
provenance. After each run, `python scripts/evaluate.py` writes the summary; copy it with
`Copy-Item runs/eval/<run> experiments/results/<id>_<alone|pfb> -Recurse`.

```powershell
python scripts/evaluate.py --config <cfg> --seed 0 --controller <ctrl> --sets scenarios/tune/tune_narrow.json scenarios/tune/tune_medium.json scenarios/tune/tune_wide.json
```
