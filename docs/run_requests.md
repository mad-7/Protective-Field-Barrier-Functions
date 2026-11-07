# Run queue (as of 2026-11-07, after the rulings of that day)

Every run on the training host (DESKTOP-55VR508), PowerShell, one command per line. Phases run in order;
jobs inside a phase run in parallel on separate simulator instances (G2 approves 4: ports 23000, 23002,
23004, 23006 = instances 0-3). `<stamp>` = the timestamp in the folder a previous command printed.
`train.py` and `evaluate.py` now RENDER by default (~25 % slower): add `--display off` to long runs.

---

## Phase A - sync and A35 (nothing else running)

```
git pull --no-edit
python docs/pending/apply_A35.py
git add preregistration/AMENDMENTS.md preregistration/endpoints.yaml preregistration/gates.yaml
git commit -m "A35: primary per planned length; K = 10 seeds 9100001-9100010; L5-v2"
python scripts/freeze_prereg.py --relock A35
git add preregistration/LOCK.json
git commit -m "Relock A35"
python -m pytest tests/test_preregistration.py -q
git push
```
If `git pull` reports an unfinished merge: `git commit --no-edit`, then `git pull --no-edit` again.
The lock test must pass before Phase B.

---

## Phase B - three pilots (ruled 2026-11-07, item 4), instances 1-3, ~17-26 h each

Purpose: every learned arm's G4 pilot before its confirmatory seeds (L4, L2), and a second L5 seed for the
seed-to-seed variance A14b needs. Unchanged setup; a defect found by Phase C restarts them (item 2).
```
python scripts/train.py --config configs/experiments/L4_sac_pfb_shaped.yaml --seed 9000002 --instance 1 --display off
python scripts/train.py --config configs/experiments/L2_sac_lagrangian.yaml --seed 9000002 --instance 2 --display off
python scripts/train.py --config configs/experiments/L5_sac_pfb_unshaped.yaml --seed 9000003 --instance 3 --display off
```
Watch: TensorBoard `rollout/ep_rew_mean`, `time/fps`; any MEMORY ALARM or exception (stop and send it).
Send: each `runs/train/<name>_seed<seed>/summary.json` when it finishes.

---

## Phase C - offline diagnostics (ruled 2026-11-07, item 3), instance 0, one after another

3a, L5's wandering, deterministic (as evaluated) and stochastic:
```
python scripts/trace_episodes.py --config configs/experiments/L5_sac_pfb_unshaped.yaml --seed 0 --controller sac --checkpoint runs/train/L5_sac_pfb_unshaped_seed9000002/final_model.zip --sets scenarios/tune/tune_narrow.json scenarios/tune/tune_medium.json scenarios/tune/tune_wide.json --from-run runs/eval/L5_sac_pfb_unshaped_20261106T130110Z_seed0 --pick wander --ratio 1.5 --max 6
python scripts/trace_episodes.py --config configs/experiments/L5_sac_pfb_unshaped.yaml --seed 0 --controller sac --checkpoint runs/train/L5_sac_pfb_unshaped_seed9000002/final_model.zip --stochastic --sets scenarios/tune/tune_narrow.json scenarios/tune/tune_medium.json scenarios/tune/tune_wide.json --from-run runs/eval/L5_sac_pfb_unshaped_20261106T130110Z_seed0 --pick wander --ratio 1.5 --max 6
```
3b, the causes of the remaining stops, L5 and B7:
```
python scripts/trace_episodes.py --config configs/experiments/L5_sac_pfb_unshaped.yaml --seed 0 --controller sac --checkpoint runs/train/L5_sac_pfb_unshaped_seed9000002/final_model.zip --sets scenarios/tune/tune_narrow.json scenarios/tune/tune_medium.json scenarios/tune/tune_wide.json --from-run runs/eval/L5_sac_pfb_unshaped_20261106T130110Z_seed0 --pick stops --max 6
python scripts/trace_episodes.py --config configs/base.yaml --seed 0 --controller regulated_pure_pursuit --filter on --sets scenarios/tune/tune_narrow.json scenarios/tune/tune_medium.json scenarios/tune/tune_wide.json --from-run runs/tuning/B7_alpha_20261102T195759Z_seed0/candidate_0 --pick stops --max 6
```
3e, B1's failures (tuned values):
```
python scripts/trace_episodes.py --config configs/base.yaml --seed 0 --controller pure_pursuit --sets scenarios/tune/tune_narrow.json scenarios/tune/tune_medium.json scenarios/tune/tune_wide.json --from-run runs/tuning/B1_pure_pursuit_20261104T183811Z_seed0/candidate_13 --pick failed --max 5
```
3c and 3d, read-only, seconds each (both denominators, moving speed, compute):
```
python scripts/resummarize_eval.py --config configs/base.yaml --seed 0 --run runs/tuning/B1_pure_pursuit_20261104T183811Z_seed0/candidate_13
python scripts/resummarize_eval.py --config configs/base.yaml --seed 0 --run runs/tuning/B2_regulated_pure_pursuit_20261030T111716Z_seed0/candidate_23
python scripts/resummarize_eval.py --config configs/base.yaml --seed 0 --run runs/tuning/B6_field_aware_mppi_20261104T141227Z_seed0/candidate_11
python scripts/resummarize_eval.py --config configs/base.yaml --seed 0 --run runs/tuning/B7_alpha_20261102T195759Z_seed0/candidate_0
python scripts/resummarize_eval.py --config configs/base.yaml --seed 0 --run runs/eval/L5_sac_pfb_unshaped_20261106T130110Z_seed0
```
Send: each trace's console output and `runs/diag/TRACE2_<stamp>_seed0/summary.json`; the five
re-summaries' output.

---

## Phase D - A14b power grid (ruled 2026-11-07, second message item 2); CPU only, hours

Can run beside Phases B and C.
```
python scripts/power_a14b.py --config configs/analysis/power_a14b.yaml --seed 0
```
Send: `runs/power/A14B_<stamp>_seed0/summary.json` and the console's last lines.

---

## Phase E - for the CoDIT paper (read-only, seconds)

The PFB's per-step compute (the red placeholder):
```
Select-String -Path runs\realtime\REALTIME_20261029T095844Z_seed0\summary.json -Pattern '"pfb"' -Context 0,20
```
Check Fig. 4 against the runs themselves (writes `paper/codit2026/figures/fig_codit_results.pdf`):
```
$env:FIG_OUT="paper/codit2026/figures"; $env:FIG_TEXT_WIDTH="7.16"; python paper/figures/scripts/fig_codit_results.py --from-runs
```
Send: both outputs (the B1 and B2 per-planned rates come from Phase C's re-summaries).

---

## Later (not runnable yet; commands follow when their prerequisites exist)

1. **Gates for the new pilots:** checkpoint evaluation + `gate_g4.py` for L4, L2 and L5 9000003, with the
   controls `zero_action_20261104T141309Z` and `oracle_tracker_privileged_20261104T170113Z`.
2. **Test size (A14) from Phase D**, a ruling, then building and locking the test set on the host.
3. **Confirmatory training:** K = 10 seeds (9100001-9100010) x L1, L2, L4, L5; ~9 days unrendered on four
   instances. Only after Phases B-C clear (no defect) and A35 is relocked.
4. **The narrow-stratum rule pilot** and **H1prime's calibration check** (scripts to be written).
5. **The one test evaluation** (Stage 4), then the analysis.
