"""Memory growth across >= 200 resets, Python and CoppeliaSim (A20 item 5, ruled 2026-10-18).
Researcher runs it (~15-20 min, one instance; display OFF, browser and editor closed).

    python scripts/check_memory.py --config configs/base.yaml --seed 0

The B1 evaluation of 2026-10-17 stalled after 51 episodes in a host memory exhaustion. This
measures whether either process grows with use: 220 episodes (20 warm-up + 200 measured), each a
full reset (a hall built and removed) and STEPS steps of Pure Pursuit under the declared recovery,
cycling through SCENARIOS scenarios generated here from a dedicated seed range (never a test
layout; the sets are being rebuilt for A20). After every episode, each process's private
(committed) and resident memory is recorded.

**Bound, declared before measuring:** growth per 100 episodes <= 10 MB in EACH process, as the
least-squares slope of private memory against episode over the 200 measured episodes. Over a
training run of ~3,000 episodes that is <= 300 MB per process.
  * PASS: both slopes <= 10 MB / 100 episodes.
  * FAIL: either slope above it -- monotonic growth beyond the bound must be fixed before training.
  * INCONCLUSIVE: the slope's standard error exceeds a third of the bound (3.3 MB / 100), so noise
    could hide or fake a leak. Expected false alarms with no leak: essentially none -- allocator
    fluctuations of a few MB over a 200-episode baseline give a slope error far below the bound,
    and that assumption is checked from the data (the standard error is reported), not assumed.
"""
from __future__ import annotations

import argparse
import json
import os
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import psutil

from pfbnav.config import load_config
from pfbnav.control.pure_pursuit import PurePursuit
from pfbnav.control.recovery import Recovery
from pfbnav.envs.freeze import freeze_problem
from pfbnav.envs.nav_env import NavEnv
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.utils.runio import create_run_dir, stamp_display
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.scenarios import make_scenario
from pfbnav.world.strata import strata_from_config

REPO_ROOT = Path(__file__).resolve().parents[1]
WARMUP = 20
MEASURED = 200
STEPS = 25                       # steps per episode after the reset
SCENARIOS = 20                   # distinct scenarios, cycled
SEED_BASE = 9_100_000            # dedicated to this check: no other set draws from it
BOUND_MB_PER_100 = 10.0          # declared above
MB = 2 ** 20


def coppelia_process(port: int) -> psutil.Process:
    """The CoppeliaSim process: the one listening on the configured port if the OS tells us, else
    the single process whose name contains 'coppelia'. Two candidates and no port answer is an
    error, not a guess."""
    try:
        for conn in psutil.net_connections(kind="tcp"):
            if conn.laddr and conn.laddr.port == port and conn.status == psutil.CONN_LISTEN \
                    and conn.pid:
                return psutil.Process(conn.pid)
    except psutil.AccessDenied:
        pass
    found = [p for p in psutil.process_iter(["name"])
             if "coppelia" in (p.info["name"] or "").lower()]
    if len(found) != 1:
        raise SystemExit(f"cannot identify the CoppeliaSim process ({len(found)} candidates)")
    return found[0]


def memory_mb(process: psutil.Process) -> tuple[float, float]:
    info = process.memory_info()
    private = getattr(info, "private", None)
    if private is None:                       # not Windows: unique set size is the analogue
        private = process.memory_full_info().uss
    return private / MB, info.rss / MB


def slope_per_100(values: list[float]) -> tuple[float, float]:
    """Least-squares slope of `values` against episode, per 100 episodes, and its standard error."""
    y = np.asarray(values, dtype=float)
    x = np.arange(y.size, dtype=float)
    (b, _a), cov = np.polyfit(x, y, 1, cov=True)
    return float(b * 100.0), float(np.sqrt(cov[0, 0]) * 100.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    problem = freeze_problem(cfg)
    if problem:
        raise SystemExit(problem)
    strata = [s.name for s in strata_from_config(cfg)]
    scenarios = []
    seed = SEED_BASE
    while len(scenarios) < SCENARIOS:
        status, scenario, _layout = make_scenario(seed, cfg, split="memcheck",
                                                  stratum=strata[len(scenarios) % len(strata)])
        seed += 1
        if status == "ok" and scenario is not None:
            scenarios.append(scenario.as_dict())
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"MEMORY_{stamp}", args.seed, cfg)
    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))
    sim_proc = coppelia_process(int(client.connection.port))
    py_proc = psutil.Process(os.getpid())
    env = NavEnv(client, cfg, scenarios, training=False, seed=args.seed)
    controller = Recovery.from_config(PurePursuit.from_config(cfg), cfg)
    rows = []
    try:
        for episode in range(WARMUP + MEASURED):
            controller.reset()
            _obs, info = env.reset(options={"index": episode % SCENARIOS})
            for _ in range(STEPS):
                _obs, _r, term, trunc, info = env.step(controller.act(info["controller_input"]))
                if term or trunc:
                    break
            py_priv, py_rss = memory_mb(py_proc)
            sim_priv, sim_rss = memory_mb(sim_proc)
            rows.append({"episode": episode, "python_private_mb": py_priv, "python_rss_mb": py_rss,
                         "sim_private_mb": sim_priv, "sim_rss_mb": sim_rss})
            if episode % 20 == 0:
                print(f"episode {episode}: python {py_priv:.0f} MB private, CoppeliaSim "
                      f"{sim_priv:.0f} MB private")
    finally:
        env.close()
        client.stop()
    measured = rows[WARMUP:]
    report = {"bound_mb_per_100_episodes": BOUND_MB_PER_100, "warmup": WARMUP,
              "measured": len(measured), "steps_per_episode": STEPS, "scenarios": SCENARIOS,
              "processes": {}}
    verdicts = []
    for name in ("python", "sim"):
        slope, se = slope_per_100([r[f"{name}_private_mb"] for r in measured])
        rss_slope, _ = slope_per_100([r[f"{name}_rss_mb"] for r in measured])
        v = ("INCONCLUSIVE" if se > BOUND_MB_PER_100 / 3.0
             else "FAIL" if slope > BOUND_MB_PER_100 else "PASS")
        verdicts.append(v)
        report["processes"][name] = {
            "private_slope_mb_per_100": slope, "private_slope_se": se,
            "rss_slope_mb_per_100": rss_slope,
            "private_first_mb": measured[0][f"{name}_private_mb"],
            "private_last_mb": measured[-1][f"{name}_private_mb"], "verdict": v}
        print(f"{name:6s}: private {measured[0][f'{name}_private_mb']:.0f} -> "
              f"{measured[-1][f'{name}_private_mb']:.0f} MB; slope {slope:+.2f} MB/100 episodes "
              f"(se {se:.2f}); RSS slope {rss_slope:+.2f}; {v}")
    report["verdict"] = ("FAIL" if "FAIL" in verdicts
                         else "INCONCLUSIVE" if "INCONCLUSIVE" in verdicts else "PASS")
    (run_dir / "memory.json").write_text(json.dumps(rows), encoding="utf-8")
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    stamp_display(run_dir, client.display_status())
    print(f"verdict: {report['verdict']}\n-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
