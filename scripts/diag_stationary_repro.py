"""Does the stationary trigger of 2026-10-22 reproduce? (ruled 2026-10-23, declared before running).
The RESEARCHER runs it (hours: hard rule 11).

    python scripts/diag_stationary_repro.py --config configs/base.yaml --seed 0

Scenario 2000018 (tune_narrow, index 18), the zero-action control exactly as G4 ran it (wrapped by the
declared recovery), at the current commit, in ONE scene load: `--runs` (20) full episodes under
normal load, then `--runs` under induced heavy load (`--load-procs` busy processes, default one per
logical core). Every step is traced (`run_<condition>_<k>.jsonl.gz`); the evaluation ring buffer
dumps on every stationary trigger as well.

**Declared outcomes** (the supervisor's, verbatim in effect):
  * reproduces ONLY under load -> a BLOCKING defect (wall-clock leaking into the step); fix and
    re-freeze before L1's pilot;
  * reproduces under BOTH -> a construction gap at that start pose; report with the trace;
  * never reproduces in the 40 -> an unexplained single event; keep the counter, escalate on
    recurrence.

**Positive control of the load** (declared): the loaded condition counts only if its runs show heavy
over-budget stepping -- a median of at least `MIN_LOADED_OVER_BUDGET` (100) over-budget steps per
run, the size of the 2026-10-22 burst (120-259 per episode) -- and the normal condition a median
below it. Otherwise the load was not induced and the verdict is INCONCLUSIVE.

Writes `runs/diag/STATIONARY_REPRO_<stamp>_seed<seed>/` with the traces, ring dumps and
`summary.json` (the counts, the load control and the verdict).
"""
from __future__ import annotations

import argparse
import gzip
import json
import multiprocessing as mp
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.control.pure_pursuit import ZeroAction
from pfbnav.control.recovery import Recovery
from pfbnav.envs.freeze import freeze_problem
from pfbnav.envs.nav_env import NavEnv
from pfbnav.eval.ring import StepRing, step_row
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.scenarios import load_set

REPO_ROOT = Path(__file__).resolve().parents[1]
SEED_OF_INTEREST = 2000018
MIN_LOADED_OVER_BUDGET = 100


def _burn(stop) -> None:
    """One busy process: dense linear algebra until told to stop."""
    a = np.random.default_rng(0).random((400, 400))
    while not stop.is_set():
        a = np.tanh(a @ a / 400.0)


def verdict(normal: list[int], loaded: list[int], normal_ob: list[int],
            loaded_ob: list[int]) -> str:
    """The declared outcome from the runs with a stationary trigger and the over-budget counts.
    Known answers in `tests/test_diag_stationary_repro.py`."""
    load_ok = (statistics.median(loaded_ob) >= MIN_LOADED_OVER_BUDGET
               and statistics.median(normal_ob) < MIN_LOADED_OVER_BUDGET)
    n, ld = sum(1 for x in normal if x > 0), sum(1 for x in loaded if x > 0)
    if not load_ok:
        return "INCONCLUSIVE: the load was not induced (positive control failed)"
    if n == 0 and ld > 0:
        return "REPRODUCES ONLY UNDER LOAD: blocking defect, fix and re-freeze before L1's pilot"
    if n > 0 and ld > 0:
        return "REPRODUCES UNDER BOTH: construction gap at the start pose, report with the trace"
    if n > 0:
        return "REPRODUCES ONLY WITHOUT LOAD: not a declared outcome, stop and report"
    return "NEVER REPRODUCES: unexplained single event, keep the counter, escalate on recurrence"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--set", default="scenarios/tune/tune_narrow.json")
    parser.add_argument("--runs", type=int, default=20)
    parser.add_argument("--load-procs", type=int, default=mp.cpu_count())
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    problem = freeze_problem(cfg)
    if problem:
        raise SystemExit(problem)
    scenarios = load_set(REPO_ROOT / args.set)["scenarios"]
    match = [s for s in scenarios if s["seed"] == SEED_OF_INTEREST]
    if len(match) != 1:
        raise SystemExit(f"scenario {SEED_OF_INTEREST} is not in {args.set}")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"STATIONARY_REPRO_{stamp}", args.seed,
                             cfg)
    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))
    env = NavEnv(client, cfg, match, training=False, seed=args.seed)
    controller = Recovery.from_config(ZeroAction.from_config(cfg), cfg)
    ring = StepRing(int(cfg["evaluation"]["ring_steps"]), run_dir / "ring")
    results: dict[str, list[dict[str, Any]]] = {"normal": [], "loaded": []}
    try:
        for condition in ("normal", "loaded"):
            stop, procs = mp.Event(), []
            if condition == "loaded":
                procs = [mp.Process(target=_burn, args=(stop,), daemon=True)
                         for _ in range(args.load_procs)]
                for p in procs:
                    p.start()
                time.sleep(2.0)
            try:
                for k in range(args.runs):
                    controller.reset()
                    _obs, info = env.reset(options={"index": 0})
                    ring.begin(k if condition == "normal" else args.runs + k)
                    path = run_dir / f"run_{condition}_{k:02d}.jsonl.gz"
                    stationary = over_budget = 0
                    first = None
                    with gzip.open(path, "wt", encoding="utf-8") as f:
                        done, steps = False, 0
                        while not done:
                            action = controller.act(info["controller_input"])
                            _obs, _r, term, trunc, info = env.step(action)
                            steps += 1
                            done = bool(term or trunc)
                            ring.record(steps, info)
                            row = step_row(steps, info)
                            row.pop("ranges_mm", None)      # the full scan only in ring dumps
                            f.write(json.dumps(row, default=float) + "\n")
                            c = info["certified"]
                            if c.get("triggered_stationary"):
                                stationary += 1
                                first = steps if first is None else first
                            over_budget += int(bool(info.get("over_budget")))
                    rec = {"run": k, "condition": condition, "steps": steps,
                           "stationary_triggers": stationary, "first_stationary_step": first,
                           "over_budget_steps": over_budget,
                           "outcome": info["episode"]["outcome"], "dumps": ring.end(
                               info["episode"]["outcome"])}
                    results[condition].append(rec)
                    print(f"{condition} {k + 1}/{args.runs}: stationary {stationary} "
                          f"(first at {first}), over-budget {over_budget}, {rec['outcome']}",
                          flush=True)
            finally:
                stop.set()
                for p in procs:
                    p.join(timeout=10)
    finally:
        env.close()
        client.stop()

    def col(cond: str, key: str) -> list[int]:
        return [int(r[key]) for r in results[cond]]
    summary = {
        "scenario_seed": SEED_OF_INTEREST, "runs_per_condition": args.runs,
        "load_procs": args.load_procs, "min_loaded_over_budget": MIN_LOADED_OVER_BUDGET,
        "runs": results,
        "runs_with_stationary": {c: sum(1 for x in col(c, "stationary_triggers") if x)
                                 for c in results},
        "median_over_budget": {c: statistics.median(col(c, "over_budget_steps"))
                               for c in results},
        "verdict": verdict(col("normal", "stationary_triggers"),
                           col("loaded", "stationary_triggers"),
                           col("normal", "over_budget_steps"),
                           col("loaded", "over_budget_steps")),
    }
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"runs with a stationary trigger: {summary['runs_with_stationary']}; median over-budget "
          f"steps: {summary['median_over_budget']}")
    print(summary["verdict"])
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
