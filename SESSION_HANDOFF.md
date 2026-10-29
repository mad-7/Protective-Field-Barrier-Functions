# Session handoff, 2026-10-28

For a new Claude Code session opened in this folder. Read `CLAUDE.md` first (hard rules, workflow,
Run Request format); it is identical to the one in the old `pfb-nav` folder. This file holds what
`CLAUDE.md` does not: where the work stands and how the researcher likes to work. The old session's
memory does not carry over to a new folder, so the preferences below are repeated here.

## Project context: read in this order
The project's context is in the repo, not in any session's memory (the old folder's memory is empty).
1. `CLAUDE.md`: what the project is, the 20 hard rules, the workflow, the Run Request format, and
   "Current work package" (the plan of record and the history of every ruling to date).
2. `docs/decision_log.md`: every design decision with its date and reason, newest last. Read the
   2026-10-26 and 2026-10-27 entries first (the path to primary results; A28, A28a, A28b; the L2 toy
   control), then search it for whatever module you are about to touch.
3. `preregistration/` (frozen: `gates.yaml`, `endpoints.yaml`, `AMENDMENTS.md`, `LOCK.json`). Never
   edit it; changes go through a drafted amendment in `docs/pending/`.
4. `docs/requirements.md` (carried requirements, the training-host compute target) and
   `docs/reproducibility.md` (determinism is per scene load).
5. `configs/methods.yaml` (what each method ID is; retired IDs stay retired) and
   `configs/base.yaml` (every measured and ruled parameter).
- The full conversation that produced the 2026-10-27 work (the L2 toy FAIL, the PID ruling, the
  candidate selection, the requirements file) is at
  `C:\Users\PC\.claude\projects\C--Users-PC-Desktop-pfb-nav\e0bf1903-093d-40dc-8853-2d967c48dd5d.jsonl`.
  It is large; search it for a specific detail rather than reading it through.

## How the researcher works (carry these over)
- Concise messages; whole solutions rather than fragments.
- The researcher works in **PowerShell**. Commands handed over must run there: one command per
  fenced block, no `sed`, **never `>>` on a UTF-8 file** (it writes UTF-16LE), appends via
  `python -c "open(..., 'a', encoding='utf-8').write(...)"`, copies with `Copy-Item`.
- Stage files **by name** (never `git add -A` / `git add .`); never stage `paper/` together with code.
- Long-duration work (training, gates that drive the simulator, tuning, evaluations) is run by the
  researcher: write a **Run Request** and wait (CLAUDE.md, "Long-duration operations").
- Supervisor rulings arrive through the researcher, pasted into the chat, and are recorded in
  `docs/decision_log.md`. Amendments are drafted in `docs/pending/`; the researcher applies and
  relocks (`python scripts/freeze_prereg.py --relock <Axx>`).
- Commit trailer used in this project: `Co-Authored-By: Claude <noreply@anthropic.com>` (use the
  attribution the session's own reminder gives).

## This folder
- A copy of the working tree of `C:\Users\PC\Desktop\pfb-nav` as of 2026-10-28 (verified identical
  with `diff -r`, excluding `.git`, caches and `.claude`), connected to
  `https://github.com/mad-7/Protective-Field-Barrier-Functions.git`, branch `main`, history starting
  at two `init` commits. The old folder keeps the full git history (commit hashes quoted in the
  decision log, e.g. `7066d04`, `b9c1b42`, refer to it) and the tag `v1-chapter-results`, which the
  README and `CLAUDE.md` name: it is NOT in this repo unless pushed from there.
- `.claude/worktrees/` is untracked: old session worktrees, a copy of the project each. Do not commit it.
- After copying: `pip install -e .` here, so the editable install points at this folder (the laptop's
  existing one points at the old folder). Checked: from this folder `pfbnav` imports from here, and
  `tests/test_architecture.py`, `tests/test_preregistration.py` and `tests/test_sac_lagrangian.py`
  pass (35 passed). The full fast suite has not been rerun here (it takes about 12 minutes).

## Files added in this folder on 2026-10-28 (uncommitted)
- `requirements.txt`, `requirements-dev.txt`: pinned direct dependencies for the SSH/training host.
- `pyproject.toml`: added `pyzmq`, `shapely`, `psutil`, `tensorboard` to `dependencies`. The code
  imports all four, and a fresh `pip install -e .` would have failed on `shapely` and `psutil`.
- `SESSION_HANDOFF.md`: this file.
- `tests/data/rate_ratio_0329941.py.txt` and `tests/test_a14a_interval.py`: the test no longer reads
  git history (the host's first run had 2 errors for that reason; now fixed, see the decision log).

## Where the work stands
- **Plan of record:** CLAUDE.md, "Current work package". Stage 1 is in progress.
- **A28b (2026-10-27) applied and relocked (`b9c1b42`).** L2's multiplier is a PID-Lagrangian
  (Stooke et al. 2020) updated once per episode. The toy control passed on seeds 0-3 with candidate 1
  (0.1, 2e-3, 0); `pid_candidate: 1` is set in the L2 configs (`7066d04`). Candidates 2 and 3 were
  not run.
- **Update 2026-10-29:** the open points below were ANSWERED (decision log, "Rulings on the
  questions of 2026-10-29"): L2 proceeds with candidate 1; the projection and the first update are
  accepted; no laptop G2 (every citable run on the training host); SAC on the GPU. The real-time
  measurement ran (B6 = 1024 x 20, `41325c4`). The training host is `DESKTOP-55VR508` (RTX 5070 Ti,
  torch 2.11.0+cu128, conda env `mahdi`). Next: G2, then L1's pilot.
- **Open points for the supervisor (as of 2026-10-27; answered 2026-10-29):**
  1. The limit cycle is reduced, not removed, on the toy (20-episode means still reach 4 stops per
     episode; windows meeting 1.25 d from episode 250: 0.78-0.82). Four passes in four may be partly
     luck. Running candidates 2 and 3 to choose among passers is outside the declared rule.
  2. The integral is projected onto >= 0 (Stooke's Algorithm 2); the ruling wrote K_I * sum(e).
  3. The first update has no derivative term.
  4. Whether G2 may be run on the laptop solely to approve laptop instances for B2 and B7-alpha
     tuning while the training-host G2 stays as ruled. The tuning scripts refuse a G2 summary from
     another hostname.
- **Waiting on the training host (no access yet), in order:**
  `python scripts/measure_realtime.py --config configs/base.yaml --seed 0`, then
  `python scripts/gate_g2_throughput.py --config configs/experiments/L1_sac_shaped.yaml --seed 9000000`
  (needs four headless CoppeliaSim instances), then L1's pilot
  `python scripts/train.py --config configs/experiments/L1_sac_shaped.yaml --seed 9000000` only after
  G2 passes. After the real-time run the researcher writes the selected samples x horizon and
  `compute_measurement` into `baselines.mppi` in one commit; if even 256 x 15 exceeds 50 ms p95,
  stop and report (A28a). After G2 passes: B2 tuning (24 candidates), alpha tuning with B7, the
  B2-vs-B7 tune-set comparison. Write those Run Requests when the G2 summary arrives.
- **Possible on the laptop meanwhile:** `pytest -m sim` against one CoppeliaSim instance; a trial of
  `measure_realtime.py` (not evidence, it is defined on the training host).

## Setting up the host
```bash
git clone https://github.com/mad-7/Protective-Field-Barrier-Functions.git
cd Protective-Field-Barrier-Functions
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install -e . --no-deps
pytest -m "not sim"
```
- On a GPU host install the matching CUDA build of torch 2.11.0 first (see the comment in
  `requirements.txt`).
- Not pip-installable: CoppeliaSim 4.10 Edu, run headless, one instance per port
  `sim.port + sim.port_stride * i`, with `scenes/warehouse_v2.ttt` (hash pinned in
  `configs/base.yaml`; runs refuse any other scene). `runs/` is ignored by git: nothing under it
  travels with the clone.
- The training sets and the tune sets are tracked under `scenarios/`; the host needs them as they
  are (hard rule 1 and the scenario hashes in the lock).
