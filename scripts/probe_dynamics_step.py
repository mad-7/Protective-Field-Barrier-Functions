"""Dynamics-step probe UPWARD (amendment A10; WP1 report, ruling 1).

D7 froze ODE at a 0.005 s dynamics step after testing only 0.005, 0.002 and 0.001 s. The
compute budget needs a faster physics step, so this probes larger steps, which D7 never
tested -- 0.010, 0.025 and 0.050 s -- against criteria declared in A10 before it runs:

  * **G1b's committed thresholds** on its committed grid (every profile at every grid speed,
    plus the lateral extension), one repetition -- repetitions are bit-identical within a
    process -- with the ceilings unchanged: v_max and vy_max from the gate stay 0.50 m/s;
  * **G0 at the candidate step, tunnelling:** all five positive cases detected with zero
    detection lag at the highest tested approach speed, 0.50 m/s (the committed G0 speed is
    0.10; a larger physics step moves the robot further per physics step, so fast approach
    is where contact could be stepped over);
  * **the drift band**: open-loop lateral heading drift at 0.12 m/s (D7's measurement) within
    the 0.64 deg solver-sensitivity band of the frozen step's drift, i.e.
    |drift(step) - drift(0.005)| <= sim.physics.solver_sensitivity_drift_deg, on *signed*
    drifts (`drift_band_holds`). The first run (DSTEP_20260918T202513Z) compared
    magnitudes, which passed drifts of the same size in the opposite direction; see
    docs/decision_log.md, 2026-09-19, and docs/corrections/.

**Only the dynamics step changes.** The control period stays 0.05 s (20 Hz), and each
control step runs n = 0.05 / dynamics_step physics steps: 10, 5, 2 and 1. The probe asserts
the simulation step is still 0.05 s at every candidate and records n.

It also records, per candidate, whether a larger step *helps*: the bare physics step time
on the probe's own instance, and the per-component cost and aggregate steps/s with
--instances instances stepping concurrently (the profiler's workers, one process each). The largest candidate passing both
criteria is the one to adopt; adopting it is a config change made afterwards, followed by
re-runs of G1b, G1-ODE and G0 on it (ruling 1). This script adopts nothing.

All candidates run in one process, one after another, so their comparison is within the
platform's determinism scope (docs/reproducibility.md). The first candidate must be the
frozen step, the baseline for the drift criterion.

Usage:
    python scripts/probe_dynamics_step.py --config configs/base.yaml --seed 0
"""
from __future__ import annotations

import argparse
import dataclasses
import importlib.util
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pfbnav.config import load_config
from pfbnav.eval.gates import drift_band_holds
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.robot import ActuationLimits
from pfbnav.utils.runio import create_run_dir, stamp_display
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
PREREG = REPO_ROOT / "preregistration"
CANDIDATES_S = (0.005, 0.010, 0.025, 0.050)
CONTROL_PERIOD_S = 0.05     # 20 Hz; unchanged by the probe
G0_APPROACH_M_S = 0.50      # the highest tested approach speed (A10)
DRIFT_SPEED_M_S = 0.12      # D7's lateral drift measurement speed
TIMING_STEPS = 200
PROFILE_STEPS = 200


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name, REPO_ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def time_physics_step(client: SimClient, steps: int) -> dict[str, float]:
    """Wall-clock cost of sim.step with the robot at rest, ms."""
    client.start(settle_steps=5)
    samples = []
    try:
        for _ in range(steps):
            t0 = time.perf_counter_ns()
            client.step()
            samples.append((time.perf_counter_ns() - t0) / 1e6)
    finally:
        client.stop()
    x = np.array(samples)
    return {"mean_ms": float(x.mean()), "p50_ms": float(np.median(x)),
            "p95_ms": float(np.percentile(x, 95))}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--candidates", type=float, nargs="*", default=list(CANDIDATES_S))
    parser.add_argument("--instances", type=int, default=4,
                        help="instances stepping concurrently for the aggregate throughput "
                             "measurement (ports sim.port + sim.port_stride * i)")
    args = parser.parse_args()

    if "## A10" not in (PREREG / "AMENDMENTS.md").read_text(encoding="utf-8"):
        raise SystemExit("amendment A10 is not in preregistration/AMENDMENTS.md; the probe's "
                         "criteria must be declared before it runs")
    cfg = load_config(args.config)
    seed_everything(args.seed)
    frozen = float(cfg["sim"]["physics"]["dynamic_step"])
    if abs(args.candidates[0] - frozen) > 1e-12:
        raise SystemExit(f"the first candidate must be the frozen step {frozen} s, the "
                         "drift baseline")
    band = float(cfg["sim"]["physics"]["solver_sensitivity_drift_deg"])
    criteria = yaml.safe_load((PREREG / "gates.yaml").read_text())["G1b_closed_loop_tracking"]
    g1b, g1 = _load("gate_g1b_tracking"), _load("gate_g1_fidelity")
    g0, prof = _load("gate_g0_contact"), _load("profile_step_cost")
    from pfbnav.eval.profiling import summarise_timings
    from pfbnav.sim.robot import ChassisGeometry
    if abs(float(cfg["sim"]["dt"]) - CONTROL_PERIOD_S) > 1e-12:
        raise SystemExit("sim.dt is not 0.05 s; A10 changes the dynamics step only")
    limits = ActuationLimits.from_config(cfg)
    grid = [float(s) for s in criteria["speeds"]]
    extra = [float(s) for s in criteria["lateral_extra_speeds"]]
    distance = float(criteria["distance"])

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"DSTEP_{stamp}", args.seed, cfg)
    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))

    report: dict[str, Any] = {"amendment": "A10", "frozen_step_s": frozen,
                              "drift_band_deg": band, "candidates": {}}
    try:
        for step_s in args.candidates:
            client.connection = dataclasses.replace(client.connection, dynamic_step=step_s)
            cand_dir = run_dir / f"step_{step_s:.3f}"
            cand_dir.mkdir()
            print(f"== dynamics step {step_s} s")
            timing = time_physics_step(client, TIMING_STEPS)
            client.start(settle_steps=1)
            physics = client.physics_settings()
            client.stop()
            if abs(physics["dynamic_step_size"] - step_s) > 1e-9:
                raise SystemExit(f"asked for {step_s} s, the instance reports {physics}")
            if abs(physics["simulation_time_step"] - CONTROL_PERIOD_S) > 1e-9:
                raise SystemExit(f"the control period changed: {physics}")
            substeps = CONTROL_PERIOD_S / step_s
            if abs(substeps - round(substeps)) > 1e-9:
                raise SystemExit(f"{step_s} s does not divide the 0.05 s control period")
            substeps = round(substeps)

            # Throughput with --instances instances at this step, in their own processes.
            results = prof.profile(args.config, args.instances, PROFILE_STEPS, step_s)
            whole = [sum(r["times"][c][k] for c in prof.COMPONENTS)
                     for r in results for k in range(PROFILE_STEPS)]
            throughput = {
                "instances": args.instances,
                "aggregate_steps_per_s": args.instances * 1000.0 / float(np.mean(whole)),
                "whole_step": summarise_timings(whole),
                "physics_step": summarise_timings(
                    [x for r in results for x in r["times"]["step"]]),
            }
            print(f"   n = {substeps} physics steps per 20 Hz control step; physics step "
                  f"alone: mean {timing['mean_ms']:.1f} ms; with {args.instances} instances: "
                  f"physics {throughput['physics_step']['mean_ms']:.1f} ms, whole step p95 "
                  f"{throughput['whole_step']['p95_ms']:.1f} ms, aggregate "
                  f"{throughput['aggregate_steps_per_s']:.1f} steps/s")

            runs = []
            plan = [(s, p) for s in grid for p in criteria["profiles"]]
            plan += [(s, "lateral") for s in extra]
            for speed, profile in plan:
                runs.append(g1b.run_once(client, cfg, profile, speed, distance, cand_dir, 0))
                r = runs[-1]
                print(f"   {profile:<26} {speed:.2f}  peak {r['max_abs_cross_track']*1e3:7.2f}"
                      f" mm  realised {r['realised_speed']:.3f} m/s")
            gate = g1b.verdict(runs, criteria)
            ceilings_ok = (gate["v_max_from_gate"] == max(grid)
                           and gate["vy_max_from_gate"] == max(grid + extra))

            # G0 at the candidate step: the five positive cases at 0.50 m/s, zero lag.
            g0_dir = cand_dir / "g0"
            g0_dir.mkdir()
            client.start(settle_steps=5)
            _, housing_corner_y, _ = g0.measure_forward_part(client, cfg)
            client.stop()
            g0_cases = []
            for case in g0.build_cases(ChassisGeometry.from_config(cfg), housing_corner_y):
                if not case.expect_contact:
                    continue
                result = g0.run_case(client, cfg, case, G0_APPROACH_M_S, g0_dir)
                g0_cases.append(result)
                print(f"   G0 {case.name:<22} lag {result['detection_lag_steps']} "
                      f"handles-agree {result['handle_validation']['agree']}")
            g0_ok = (len(g0_cases) == 5
                     and all(c.get("detection_lag_steps") == 0 for c in g0_cases)
                     and all(c["handle_validation"]["agree"] for c in g0_cases))

            drift_run = g1.run_profile(client, cfg, g1.PROFILES["lateral"], distance, cand_dir,
                                       speed_scale=DRIFT_SPEED_M_S / limits.vy_max)
            drift = float(drift_run["signed_heading_drift_deg"])
            print(f"   G1b grid: {gate['result']}, v_max {gate['v_max_from_gate']}, vy_max "
                  f"{gate['vy_max_from_gate']}; lateral drift at 0.12 m/s {drift:+.3f} deg")
            report["candidates"][f"{step_s:.3f}"] = {
                "physics": physics, "physics_steps_per_control_step": substeps,
                "physics_step_timing": timing, "throughput": throughput,
                "g0_positive_cases_at_0_50": g0_cases, "g0_zero_lag": g0_ok,
                "g1b": {k: gate[k] for k in ("result", "v_max_from_gate", "vy_max_from_gate",
                                             "per_speed", "lateral_per_speed")},
                "g1b_runs": gate["runs"], "g1b_ceilings_unchanged": ceilings_ok,
                "lateral_drift_deg_signed": drift,
                "lateral_speed_error": drift_run.get("mean_relative_speed_error"),
            }
    finally:
        client.stop()

    base_drift = report["candidates"][f"{frozen:.3f}"]["lateral_drift_deg_signed"]
    passing = []
    for key, c in report["candidates"].items():
        c["drift_within_band"] = drift_band_holds(c["lateral_drift_deg_signed"], base_drift,
                                                  band)
        c["passes_a10"] = bool(c["g1b"]["result"] == "PASS" and c["g1b_ceilings_unchanged"]
                               and c["g0_zero_lag"] and c["drift_within_band"])
        if c["passes_a10"]:
            passing.append(float(key))
    report["largest_passing_step_s"] = max(passing) if passing else None
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float))
    stamp_display(run_dir, client.display_status())

    print(f"\n{'step s':>7} {'n':>3} {'phys ms':>8} {'agg/s':>6} {'G1b':>5} {'ceil':>5} "
          f"{'G0':>5} {'drift':>7} {'band':>5} A10")
    for key, c in report["candidates"].items():
        t = c["throughput"]
        print(f"{key:>7} {c['physics_steps_per_control_step']:>3} "
              f"{t['physics_step']['mean_ms']:8.1f} {t['aggregate_steps_per_s']:6.1f} "
              f"{c['g1b']['result']:>5} {c['g1b_ceilings_unchanged']!s:>5} "
              f"{c['g0_zero_lag']!s:>5} {c['lateral_drift_deg_signed']:+7.3f} "
              f"{c['drift_within_band']!s:>5} {'pass' if c['passes_a10'] else 'FAIL'}")
    print(f"\nlargest step passing A10: {report['largest_passing_step_s']} s  ->  {run_dir}")
    print("Nothing is adopted here: adoption is a config change, then G1b, G1-ODE and G0 "
          "re-run on the adopted step.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
