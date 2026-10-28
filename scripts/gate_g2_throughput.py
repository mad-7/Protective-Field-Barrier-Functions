"""Gate G2: training-host throughput with 1..N parallel CoppeliaSim instances, the learner running.
The RESEARCHER runs this on the TRAINING HOST, at the start of L1's pilot (ruled 2026-10-26):

    python scripts/gate_g2_throughput.py --config configs/experiments/L1_sac_shaped.yaml --seed 9000000

**Preconditions:** `max(G2_compute_budget.parallel_instances)` instances running headless on ports
`sim.port + sim.port_stride * i`; `configs/gates/g2.yaml` with `max_over_budget_share` ruled (it
refuses while it is null); the frozen environment; the training set built.

**What it does.** For each N in `G2_compute_budget.parallel_instances` (1, 2, 3, 4), N worker
processes start together (a ready/go barrier), worker i on instance i. Each worker:
  1. loads the scene on its instance and builds the TRAINING environment exactly as
     `scripts/train.py` does (same preflight, sets, verification and seed order), with the per-step
     budget computed for N instances (`env.step_budget.instances` = N, so "over budget" means over
     N / 25 s);
  2. **random phase:** random actions for `measure_seconds`;
  3. **SAC phase:** SAC learning, one gradient update per step, for `measure_seconds` timed from the
     first update. `learning_starts` is lowered to `batch_size` for this measurement only, so the
     window measures updates (at the pilot's 2,000 the first five minutes would have none); every
     other hyperparameter is the experiment's. The update count in the window is recorded as the
     positive control that the learner ran.
Measured by `pfbnav.eval.g2.Meter`; concluded by `pfbnav.eval.g2` (module docstring: the rates, the
rendered/non-rendered split, the verdict per N, the approved instance count, the projection).

Rendering (ruled 2026-09-26): throughput is reported split by rendered and non-rendered episodes,
never pooled; an N with more than `max_rendered_episode_fraction` of episodes rendered FAILS. The
display refusal fraction is reported; above 0.10, or with a material rendered/non-rendered
difference, escalate to the one-day diagnosis.

Writes `runs/gates/G2_<stamp>_seed<seed>/`: config.yaml, provenance.json (with the host),
`n<N>/worker<i>.json` and `.log`, and summary.json (per N: both phases, the verdict; the approved
instance count; the projection; PASS/FAIL). Nothing it measures is written into a config: the
approved count is read from this summary by `scripts/tune_classical.py`, and any change to
`env.step_budget.instances` is a ruling.
"""
from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from pfbnav.config import load_config
from pfbnav.eval.g2 import projection, select_instances, summarise_phase, verdict_n
from pfbnav.eval.runner import SCAN_FAULT_FLAG
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.runio import create_run_dir, host_identity

REPO_ROOT = Path(__file__).resolve().parents[1]
GATE_CONFIG = REPO_ROOT / "configs" / "gates" / "g2.yaml"
READY_TIMEOUT_S = 900.0         # s, scene load + environment build for every worker
POLL_S = 0.5                    # s, the barrier's polling interval


def _train_module():
    spec = importlib.util.spec_from_file_location("train", REPO_ROOT / "scripts" / "train.py")
    if spec is None or spec.loader is None:
        raise SystemExit("cannot load scripts/train.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def criteria() -> dict[str, Any]:
    """The frozen thresholds and G2's own over-budget reading; refuses while the latter is unruled."""
    gates = yaml.safe_load((REPO_ROOT / "preregistration" / "gates.yaml").read_text(encoding="utf-8"))
    g2 = gates["G2_compute_budget"]
    own = yaml.safe_load(GATE_CONFIG.read_text(encoding="utf-8"))
    if own.get("max_over_budget_share") is None:
        raise SystemExit("configs/gates/g2.yaml: max_over_budget_share is not ruled yet (the numeric "
                         "reading of 'essentially no over-budget steps'); G2 refuses to run")
    # Ruled 0.001 on 2026-10-27; once A28 writes it into the preregistration, the two must agree.
    frozen = g2.get("max_over_budget_share")
    if frozen is not None and float(frozen) != float(own["max_over_budget_share"]):
        raise SystemExit(f"configs/gates/g2.yaml says max_over_budget_share "
                         f"{own['max_over_budget_share']}, preregistration/gates.yaml {frozen}")
    return {"parallel_instances": [int(n) for n in g2["parallel_instances"]],
            "measure_seconds": float(g2["measure_seconds"]),
            "min_steps_per_second": float(g2["min_steps_per_second"]),
            "max_rendered_episode_fraction": float(g2["max_rendered_episode_fraction"]),
            "max_over_budget_share": float(own["max_over_budget_share"]),
            "max_scan_fault_share": SCAN_FAULT_FLAG,
            "n_training_runs": int(g2["n_training_runs"]),
            "wallclock_budget_weeks": float(g2["wallclock_budget_weeks"])}


def worker_config(cfg: dict[str, Any], n: int, index: int) -> dict[str, Any]:
    """Instance `index` of `n`: its port, the per-step budget for n instances, and learning_starts
    lowered to batch_size (this measurement only)."""
    out = copy.deepcopy(cfg)
    out["sim"]["port"] = int(cfg["sim"]["port"]) + int(cfg["sim"]["port_stride"]) * index
    out["env"]["step_budget"]["instances"] = int(n)
    out["training"]["learning_starts"] = int(cfg["training"]["batch_size"])
    return out


# ---------------------------------------------------------------------------------- the worker
def worker(args: argparse.Namespace) -> int:
    from stable_baselines3.common.callbacks import BaseCallback

    from pfbnav.control.sac import make_sac
    from pfbnav.envs.nav_env import NavEnv
    from pfbnav.eval.g2 import Meter
    from pfbnav.sim.client import SimClient, SimConnection
    from pfbnav.training.order import scenario_order
    from pfbnav.utils.seeding import seed_everything
    from pfbnav.world.scenarios import load_set, verify

    out = Path(args.out)
    cfg = worker_config(apply_display(load_config(args.config), args.display), args.n, args.index)
    seed_everything(args.seed)
    train = _train_module()
    scenarios: list[dict[str, Any]] = []
    for path in train.preflight(cfg):
        manifest = load_set(path)
        checked = verify(manifest, cfg)
        if not checked["ok"]:
            raise SystemExit(f"{path.name} fails verification: {checked['problems'][:3]}")
        scenarios += manifest["scenarios"]
    ordered = [scenarios[i] for i in scenario_order(len(scenarios), args.seed)]
    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))
    env = NavEnv(client, cfg, ordered, training=True, seed=args.seed)
    meter = Meter(env, lambda: client.display_in_force)
    seconds = float(args.seconds)
    result: dict[str, Any] = {"n": args.n, "instance": args.index, "port": cfg["sim"]["port"],
                              "seed": args.seed}
    try:
        (out / f"worker{args.index}.ready").write_text("", encoding="utf-8")
        go = out / "go"
        waited = time.perf_counter()
        while not go.exists():
            if time.perf_counter() - waited > READY_TIMEOUT_S:
                raise SystemExit("no go signal from the orchestrator")
            time.sleep(POLL_S)

        # Random phase: the window opens with its first reset, so an episode costs its reset.
        meter.start_window()
        meter.reset()
        while meter.elapsed() < seconds:
            _, _, terminated, truncated, _ = meter.step(meter.action_space.sample())
            if terminated or truncated:
                meter.reset()
        result["random"] = meter.stop_window()

        # SAC phase: the window opens at the first gradient update.
        class Window(BaseCallback):
            def __init__(self):
                super().__init__()
                self.updates_at_start: int | None = None

            def _on_step(self) -> bool:
                updates = int(getattr(self.model, "_n_updates", 0))
                if self.updates_at_start is None:
                    if updates > 0:
                        self.updates_at_start = updates
                        meter.start_window()
                    return True
                return meter.elapsed() < seconds

        window = Window()
        model = make_sac(meter, cfg, args.seed)
        model.learn(total_timesteps=10**9, callback=window)
        result["sac"] = meter.stop_window()
        result["sac"]["updates"] = int(model._n_updates) - int(window.updates_at_start or 0)
        result["sac"]["device"] = str(model.device)
        result["sac"]["learning_starts"] = int(cfg["training"]["learning_starts"])
    finally:
        env.close()
        client.stop()
        result["display"] = client.display_status()
        (out / f"worker{args.index}.json").write_text(json.dumps(result, indent=1, default=float),
                                                      encoding="utf-8")
    return 0


# ---------------------------------------------------------------------------- the orchestrator
def run_n(args: argparse.Namespace, n: int, out: Path, seconds: float) -> list[dict[str, Any]]:
    out.mkdir()
    procs, logs = [], []
    for i in range(n):
        log = (out / f"worker{i}.log").open("w", encoding="utf-8")
        logs.append(log)
        procs.append(subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve()), "--worker", "--config", args.config,
             "--seed", str(args.seed), "--display", args.display, "--n", str(n), "--index", str(i),
             "--seconds", str(seconds), "--out", str(out)],
            stdout=log, stderr=subprocess.STDOUT, cwd=str(REPO_ROOT)))
    try:
        started = time.perf_counter()
        while not all((out / f"worker{i}.ready").exists() for i in range(n)):
            dead = [i for i, p in enumerate(procs) if p.poll() is not None]
            if dead:
                raise SystemExit(f"N={n}: worker(s) {dead} exited before ready; see {out}")
            if time.perf_counter() - started > READY_TIMEOUT_S:
                raise SystemExit(f"N={n}: workers not ready after {READY_TIMEOUT_S:.0f} s")
            time.sleep(POLL_S)
        (out / "go").write_text("", encoding="utf-8")
        codes = [p.wait() for p in procs]
    finally:
        for p in procs:
            if p.poll() is None:
                p.kill()
        for log in logs:
            log.close()
    if any(codes):
        raise SystemExit(f"N={n}: worker exit codes {codes}; see {out}/worker*.log")
    return [json.loads((out / f"worker{i}.json").read_text(encoding="utf-8")) for i in range(n)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    add_display_flag(parser)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--n", type=int, help=argparse.SUPPRESS)
    parser.add_argument("--index", type=int, help=argparse.SUPPRESS)
    parser.add_argument("--seconds", type=float, help=argparse.SUPPRESS)
    parser.add_argument("--out", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        return worker(args)

    crit = criteria()
    cfg = apply_display(load_config(args.config), args.display)
    _train_module().preflight(cfg)                    # every refusal train.py makes, before any run
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "gates", f"G2_{stamp}", args.seed, cfg)
    per_n: dict[int, Any] = {}
    verdicts: dict[int, Any] = {}
    for n in crit["parallel_instances"]:
        print(f"N={n}: {n} worker(s), {crit['measure_seconds']:.0f} s random + "
              f"{crit['measure_seconds']:.0f} s SAC", flush=True)
        workers = run_n(args, n, run_dir / f"n{n}", crit["measure_seconds"])
        phases = {ph: summarise_phase([w[ph] for w in workers]) for ph in ("random", "sac")}
        phases["sac"]["updates"] = [w["sac"]["updates"] for w in workers]
        phases["display"] = [w["display"] for w in workers]
        verdicts[n] = verdict_n(phases["sac"], **{k: crit[k] for k in (
            "min_steps_per_second", "max_rendered_episode_fraction", "max_over_budget_share",
            "max_scan_fault_share")})
        per_n[n] = {**phases, "verdict": verdicts[n]}
        s = phases["sac"]
        print(f"  random {phases['random']['aggregate_steps_per_s']}, SAC "
              f"{s['aggregate_steps_per_s']} steps/s; rendered {s['rendered_episode_fraction']}, "
              f"over budget {s['over_budget_share']}, scan faults {s['scan_fault_share']}, "
              f"updates {s['updates']} -> {'PASS' if verdicts[n]['pass'] else verdicts[n]['failed']}",
              flush=True)
    approved = select_instances(verdicts)
    total = int(cfg["training"]["total_timesteps"])
    summary = {
        "verdict": "PASS" if approved is not None else "FAIL",
        "approved_instances": approved,
        "host": host_identity(cfg),
        "criteria": crit,
        "per_n": {str(n): v for n, v in per_n.items()},
        "projection": {str(n): projection(per_n[n]["sac"]["aggregate_steps_per_s"] or 0.0,
                                          crit["n_training_runs"], total,
                                          crit["wallclock_budget_weeks"]) for n in per_n},
        "note": "learning_starts lowered to batch_size in the SAC phase only, so the window "
                "measures updates; the over-budget flag uses the budget for each N",
    }
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=float),
                                          encoding="utf-8")
    print(f"G2 {summary['verdict']}: approved instances {approved}")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0 if approved is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
