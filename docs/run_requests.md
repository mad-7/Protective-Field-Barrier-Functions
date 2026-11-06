# Run queue (as of 2026-11-06)

Every run on the training host (DESKTOP-55VR508), PowerShell, one command per line. Phases run in
order; jobs inside a phase run in parallel on separate simulator instances (G2 approves 4: ports
23000, 23002, 23004, 23006 = instances 0-3). Skip anything already done. `<stamp>` = the timestamp in
the folder name a previous command printed (`-> runs/...`).

Status key: **READY** (code and rulings in place), **LATER** (ready, waits on an earlier result),
**NOT YET RUNNABLE** (needs code or a ruling first; no command yet).

---

## Phase A - sync, A32, checks (READY; nothing else running)

Purpose: bring the host to the re-freeze 6ad7f6af and apply A32 before any L5 result.

```
git pull --no-edit
git push
python docs/pending/apply_A32.py
git add preregistration/AMENDMENTS.md preregistration/gates.yaml preregistration/endpoints.yaml
git commit -m "A32: L5 pilot seeds 9000002/9000003, A31 scope, B7 alpha sensitivity"
python scripts/freeze_prereg.py --relock A32
git add preregistration/LOCK.json
git commit -m "Relock A32"
git push
python -m pytest tests/test_preregistration.py -q
python -m pytest tests/test_step_loop_sim.py -k filter_off -q
```

The last test needs CoppeliaSim on port 23000. Both tests must pass before Phase B.
Send back: the two test outputs.

---

## Phase B - four parallel jobs (READY after Phase A)

Start four headless CoppeliaSim instances (23000, 23002, 23004, 23006), then one terminal per job.

### B-0. Instance 0: G4 controls under 6ad7f6af, then B1 tuning
Purpose: L5's G4 needs controls run on the current environment (A32 freeze mapping); then B1 (A33).
About 1 h + 1 h, then about 30-35 h.
```
python scripts/evaluate.py --config configs/base.yaml --seed 0 --controller zero --sets scenarios/tune/tune_narrow.json scenarios/tune/tune_medium.json scenarios/tune/tune_wide.json
python scripts/evaluate.py --config configs/base.yaml --seed 0 --controller oracle_tracker --sets scenarios/tune/tune_narrow.json scenarios/tune/tune_medium.json scenarios/tune/tune_wide.json
python scripts/tune_classical.py --config configs/base.yaml --seed 0 --space configs/tuning/b1_pure_pursuit.yaml --g2 runs/gates/G2_20261029T171718Z_seed9000000/summary.json --instances 4 --instance 0
```
Outputs: `runs/eval/zero_action_<stamp>_seed0`, `runs/eval/oracle_tracker_privileged_<stamp>_seed0`,
`runs/tuning/B1_pure_pursuit_<stamp>_seed0`.
Expect: zero-action 0/60, all timeouts; oracle about 43/60.

### B-1. Instance 1: L5 pilot, first seed
Purpose: Stage 2, L5 vs G4 (A32: seed 9000002; 9000001 is a recorded integrity failure). About 17 h+.
```
python scripts/train.py --config configs/experiments/L5_sac_pfb_unshaped.yaml --seed 9000002 --instance 1
```
Output: `runs/train/L5_sac_pfb_unshaped_seed9000002`. Watch: TensorBoard `rollout/ep_rew_mean`,
`time/fps` (>= 25), any MEMORY ALARM, and any exception (stop and send it).

### B-2. Instance 2: B6 tuning (field-aware MPPI)
Purpose: H1/H1prime comparator, tuned with care. About 35 h+.
```
python scripts/tune_classical.py --config configs/base.yaml --seed 0 --space configs/tuning/b6_mppi.yaml --g2 runs/gates/G2_20261029T171718Z_seed9000000/summary.json --instances 4 --instance 2
```

### B-3. Instance 3: B3 tuning (holonomic DWA)
Purpose: H1 comparator. About 30-35 h.
```
python scripts/tune_classical.py --config configs/base.yaml --seed 0 --space configs/tuning/b3_dwa.yaml --g2 runs/gates/G2_20261029T171718Z_seed9000000/summary.json --instances 4 --instance 3
```

Progress of any tuning run (read-only, any time):
```
python scripts/peek_tuning.py --config configs/base.yaml --seed 0 --run runs/tuning/<folder>
```
Send back per tuning run: console output, `provenance.json`, `summary.json`.

---

## Phase C - read-only checks (READY after Phase A; seconds; no simulator)

C-1. B2 vs B7 speed on B7's 13 horizon timeouts (ruled 2026-11-03):
```
python scripts/compare_route_speeds.py --config configs/base.yaml --seed 0 --classification runs/diag/B7TIMEOUTS_20261103T174622Z_seed0 --b2-tuning runs/tuning/B2_regulated_pure_pursuit_20261030T111716Z_seed0
```
C-2. The long unrecoverable-stop episode in L1's training logs (observation of 2026-11-04):
```
Select-String -Path runs\train\L1_sac_shaped_seed9000002\episodes.jsonl -Pattern '"outcome": "unrecoverable_stop"' | ForEach-Object { $_.Line.Substring(0, [Math]::Min(600, $_.Line.Length)) }
```
Send back: both outputs.

---

## Phase D - L5's G4 (LATER: when B-1 finishes and B-0's controls exist)

D-1. Training read (seconds):
```
python scripts/peek_training.py --run runs/train/L5_sac_pfb_unshaped_seed9000002
python scripts/diag_training_decline.py --config configs/experiments/L5_sac_pfb_unshaped.yaml --seed 9000002 --run runs/train/L5_sac_pfb_unshaped_seed9000002
```
D-2. Checkpoint evaluation (instance 0 when free; about 1-1.5 h; the filter is on at the frozen alpha):
```
python scripts/evaluate.py --config configs/experiments/L5_sac_pfb_unshaped.yaml --seed 0 --controller sac --checkpoint runs/train/L5_sac_pfb_unshaped_seed9000002/final_model.zip --sets scenarios/tune/tune_narrow.json scenarios/tune/tune_medium.json scenarios/tune/tune_wide.json
```
D-3. The gate, with the NEW controls from B-0:
```
python scripts/gate_g4.py --config configs/base.yaml --seed 0 --train-run runs/train/L5_sac_pfb_unshaped_seed9000002 --eval-run runs/eval/L5_sac_pfb_unshaped_<stamp>_seed0 --zero-run runs/eval/zero_action_<stamp>_seed0 --positive-run runs/eval/oracle_tracker_privileged_<stamp>_seed0
```
Send back: D-1 output, L5's `summary.json`, the gate's `summary.json`.

D-4. ONLY if D-3 fails on learning or checkpoint: the second seed, unchanged, then D-1 to D-3 with
9000003:
```
python scripts/train.py --config configs/experiments/L5_sac_pfb_unshaped.yaml --seed 9000003 --instance 1
```
Two L5 failures: stop and report; the study pivots to H1prime (A27, A31). An integrity failure is
fixed and does not count (A32).

---

## Phase E - after the tuning runs (LATER)

Review only, by me: B1, B3, B6 selections written into `configs/base.yaml` by hand with a decision-log
entry (as B2). No command for you beyond sending the files.

---

## Phase F - NOT YET RUNNABLE (code or rulings first; I will write the commands)

In plan order; each needs the noted prerequisite.
1. **Step 0, the seed count** (decision tree of 2026-10-03; seed CV on the TUNE set): needs L5 past
   G4. Command written then (`scripts/power_corrected.py` family).
2. **Confirmatory training, if L5 passes G4:** L5, L1, L4 (L3 reuses L1's checkpoints), then L2;
   seeds and count from Step 0. **Missing:** the L4 config (must carry the derived k_proximity) and the
   confirmatory seed list.
3. **A14 / A14b:** the test-set size and power, including H1prime's; the classical-pair calibration
   for H1prime (A27). Needs a ruling and a script.
4. **Test sets:** built and verified on the host after A14 (`scripts/build_scenario_sets.py`).
5. **The narrow rule:** 150 pilot-range narrow episodes per method before any test episode. Needs
   every method tuned/trained and a script.
6. **The one test evaluation** (Stage 4), plus the declared exploratory analyses (B7 alpha
   sensitivity at 1, 2, 5; L2-d005/d020; H3-H6). Last.
7. **Exploratory arms** (A1, L5-nowp, L2-d005, L2-d020): their own G4 pilots; configs for A1 and
   L5-nowp are missing.

If L5 fails G4 twice (pivot): Phase F shrinks to items 3-6 for B7 against B1, B2 and B6 only.
