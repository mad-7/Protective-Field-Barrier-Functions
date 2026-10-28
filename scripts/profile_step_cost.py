"""Profiling pass: per-step cost of the environment's components across 1-4 instances.

NOT A GATE (WP0 closure, 2026-09-18). G2 is deferred to WP1, where the environment exists.
This measures what one environment step will have to do on the robot side, with N
CoppeliaSim instances stepping at once, and reports costs with no PASS/FAIL. Components
costing more than ~2 ms per step on average are flagged.

Components timed, each on every step:
  budget      project_to_wheel_budget on the command (pure; local CPU)
  command     YouBotBase.command_body_velocity (clamp, budget, limiter, 4 joint writes)
  step        sim.step (physics; context for the others)
  contact     RobotContact.check (two checkCollision calls, collections)
  pose        YouBotBase.pose (two remote reads)
  velocity    PoseDifferenceEstimator.update (pure; local CPU)
  torque_cap  YouBotBase.record_torque_cap (four joint-force reads)

One worker process per instance, each with its own ZeroMQ client on port base + i, all
released together by a barrier so the instances step concurrently. The command is a slow
fore-and-aft sinusoid (0.2 m/s peak, 4 s period), so the robot stays near its start pose.

Output goes to docs/profiling/, never to runs/gates/: a report, the summaries, every
per-step sample (samples/), and the provenance (commit, dirty flag, scene hash, config,
core counts). The instance sweep is repeated over --passes interleaved passes, so that
pass-to-pass variation can be told from scaling (WP1 report, ruling 1). Samples above
SPIKE_MS in any component other than the physics step are listed by step, which is how
the contact check's 0.6-1.0 s spike is located.

Usage (instances already running on sim.port + sim.port_stride * i: 23000, 23002,
23004, 23006):
    python scripts/profile_step_cost.py --config configs/base.yaml --seed 0
"""
from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import multiprocessing as mp
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
COMPONENTS = ("budget", "command", "step", "contact", "pose", "velocity", "torque_cap")
COMMAND_PEAK_M_S = 0.2
COMMAND_PERIOD_S = 4.0
WARMUP_STEPS = 20


def worker(instance: int, config_path: str, steps: int, barrier, queue,
           dynamic_step: float | None = None) -> None:
    """Run `steps` profiled steps on instance `instance`; put {component: [ms, ...]}.

    `dynamic_step` overrides the physics substep (A10's probe); None keeps the frozen one.
    """
    import dataclasses

    from pfbnav.config import load_config
    from pfbnav.sim.client import SimClient, SimConnection
    from pfbnav.sim.contact import RobotContact
    from pfbnav.sim.robot import YouBotBase, project_to_wheel_budget
    from pfbnav.sim.velocity_estimate import PoseDifferenceEstimator, VelocityEstimateConfig

    cfg = load_config(config_path)
    connection = SimConnection.from_config(cfg, instance_id=instance)
    if dynamic_step is not None:
        connection = dataclasses.replace(connection, dynamic_step=dynamic_step)
    client = SimClient(connection)
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))
    times = {c: [] for c in COMPONENTS}
    robot = None
    try:
        client.start(settle_steps=WARMUP_STEPS)
        robot = YouBotBase.from_config(client, cfg)
        robot.bind()
        robot.reset_command_state()
        contact = RobotContact.from_config(client, cfg, resolve_names=False)
        contact.bind()
        estimator = PoseDifferenceEstimator(VelocityEstimateConfig.from_config(cfg))
        budget = robot.budget
        dt = float(cfg["sim"]["dt"])
        barrier.wait()
        for k in range(steps):
            vx = COMMAND_PEAK_M_S * math.sin(2.0 * math.pi * k * dt / COMMAND_PERIOD_S)
            command = np.array([vx, 0.0, 0.0])

            t0 = time.perf_counter_ns()
            project_to_wheel_budget(command, budget.nominal_limit, budget.lever_arm)
            t1 = time.perf_counter_ns()
            robot.command_body_velocity(command)
            t2 = time.perf_counter_ns()
            client.step()
            t3 = time.perf_counter_ns()
            contact.check()
            t4 = time.perf_counter_ns()
            pose = robot.pose()
            t5 = time.perf_counter_ns()
            estimator.update(pose)
            t6 = time.perf_counter_ns()
            robot.record_torque_cap()
            t7 = time.perf_counter_ns()
            for name, a, b in zip(COMPONENTS, (t0, t1, t2, t3, t4, t5, t6),
                                  (t1, t2, t3, t4, t5, t6, t7)):
                times[name].append((b - a) / 1e6)
        queue.put({"instance": instance, "times": times,
                   "torque_cap_counts": dict(robot.torque_cap_counts),
                   "physics": client.physics_settings()})
    except BaseException as exc:
        # Not swallowed: reported to the parent at once, then re-raised, so a dead worker
        # fails the pass immediately instead of after the queue's timeout.
        queue.put({"instance": instance, "error": f"{type(exc).__name__}: {exc}"})
        raise
    finally:
        if robot is not None:
            robot.halt()
        client.stop()


def profile(config_path: str, n_instances: int, steps: int,
            dynamic_step: float | None = None) -> list[dict[str, Any]]:
    ctx = mp.get_context("spawn")
    barrier = ctx.Barrier(n_instances)
    queue = ctx.Queue()
    procs = [ctx.Process(target=worker,
                         args=(i, config_path, steps, barrier, queue, dynamic_step))
             for i in range(n_instances)]
    for p in procs:
        p.start()
    results = [queue.get(timeout=600) for _ in procs]
    errors = [r for r in results if "error" in r]
    if errors:
        for p in procs:
            p.join(timeout=10)
        raise SystemExit(f"profiling worker(s) failed: {errors}")
    for p in procs:
        p.join(timeout=60)
        if p.exitcode != 0:
            raise SystemExit(f"a profiling worker exited with code {p.exitcode}")
    return sorted(results, key=lambda r: r["instance"])


def machine_info() -> dict[str, Any]:
    """Core counts and processor, for the throughput numbers to be read against."""
    import os
    import platform

    info: dict[str, Any] = {"logical_cores": os.cpu_count(), "processor": platform.processor(),
                            "platform": platform.platform(), "physical_cores": None}
    if importlib.util.find_spec("psutil") is not None:
        import psutil
        info["physical_cores"] = psutil.cpu_count(logical=False)
    return info


def write_samples(path: Path, times: dict[str, list[float]]) -> None:
    """Per-step samples of every component, ms, one row per step."""
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["step", *COMPONENTS])
        for k in range(len(times[COMPONENTS[0]])):
            writer.writerow([k, *(repr(times[c][k]) for c in COMPONENTS)])


def main() -> int:
    from pfbnav.config import load_config
    from pfbnav.eval.profiling import FLAG_MS, spikes, summarise_timings
    from pfbnav.utils.runio import git_state, scene_provenance
    from pfbnav.utils.seeding import seed_everything

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--instances", type=int, nargs="*", default=[1, 2, 3, 4])
    parser.add_argument("--steps", type=int, default=400)
    parser.add_argument("--passes", type=int, default=3,
                        help="repetitions of the whole instance sweep, interleaved, so that "
                             "pass-to-pass variation can be told from scaling")
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = REPO_ROOT / "docs" / "profiling" / f"step_cost_{stamp}"
    machine = machine_info()
    provenance = {"seed": args.seed, "created_utc": datetime.now(timezone.utc).isoformat(),
                  "instances": args.instances, "steps": args.steps, "passes": args.passes,
                  "machine": machine, **git_state(), **scene_provenance(cfg)}
    out.mkdir(parents=True)
    (out / "provenance.json").write_text(json.dumps(provenance, indent=2))
    with (out / "config.yaml").open("w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    samples_dir = out / "samples"
    samples_dir.mkdir()
    print(f"machine: {machine}")

    collected: dict[int, list[dict[str, Any]]] = {n: [] for n in args.instances}
    for p in range(args.passes):
        for n in args.instances:
            print(f"-- pass {p + 1}/{args.passes}: {n} instance(s), {args.steps} steps each")
            for r in profile(args.config, n, args.steps):
                r["pass"] = p
                collected[n].append(r)
                write_samples(samples_dir / f"n{n}_pass{p}_inst{r['instance']}.csv", r["times"])

    report: dict[str, Any] = {"note": "profiling pass, not a gate: no PASS/FAIL",
                              "flag_ms": FLAG_MS, "machine": machine, "runs": {}}
    for n, results in collected.items():
        per_component = {c: summarise_timings([x for r in results for x in r["times"][c]])
                         for c in COMPONENTS}
        per_pass_throughput = []
        for p in range(args.passes):
            rs = [r for r in results if r["pass"] == p]
            step_ms = [sum(r["times"][c][k] for c in COMPONENTS)
                       for r in rs for k in range(args.steps)]
            per_pass_throughput.append(n * 1000.0 / float(np.mean(step_ms)))
        total = [sum(r["times"][c][k] for c in COMPONENTS)
                 for r in results for k in range(args.steps)]
        spike_list = [{"pass": r["pass"], "instance": r["instance"], "component": c,
                       "steps": spikes(r["times"][c])}
                      for r in results for c in COMPONENTS if c != "step"
                      and spikes(r["times"][c])]
        report["runs"][str(n)] = {
            "physics": results[0]["physics"],
            "components": per_component,
            "whole_step": summarise_timings(total),
            "aggregate_steps_per_s_by_pass": per_pass_throughput,
            "spikes": spike_list,
            "torque_cap_counts": [r["torque_cap_counts"] for r in results],
        }
        print(f"\n== {n} instance(s), {args.passes} passes pooled")
        for c in COMPONENTS:
            s = per_component[c]
            mark = "  <-- above ~2 ms" if s["flagged"] else (
                "  (tail above 2 ms)" if s["p95_over"] else "")
            print(f"   {c:<11} mean {s['mean_ms']:7.3f}  p95 {s['p95_ms']:7.3f}  "
                  f"max {s['max_ms']:8.3f} ms{mark}")
        print("   aggregate steps/s by pass: "
              + ", ".join(f"{v:.1f}" for v in per_pass_throughput))
        for sp in spike_list:
            print(f"   spike: pass {sp['pass']} instance {sp['instance']} {sp['component']} "
                  f"at steps {sp['steps']}")

    (out / "summary.json").write_text(json.dumps(report, indent=2))
    engine = report["runs"][str(args.instances[0])]["physics"]["engine_name"]
    intro = (f"Commit `{provenance['commit']}` (dirty: {provenance['dirty']}), scene "
             f"`{provenance['scene_sha256'][:8]}...`, engine {engine}, {args.steps} steps "
             f"per instance, {args.passes} interleaved passes. Machine: "
             f"{machine['logical_cores']} logical cores"
             + (f", {machine['physical_cores']} physical" if machine["physical_cores"] else "")
             + f", {machine['processor']}. Components above ~{FLAG_MS:g} ms per step on "
             "average are flagged. Per-step samples are in samples/.")
    lines = ["# Per-step cost profile (not a gate)", "", intro, ""]
    for n, run in report["runs"].items():
        lines += [f"## {n} instance(s)", "",
                  "| component | mean ms | p50 ms | p95 ms | max ms | flag |",
                  "|---|---|---|---|---|---|"]
        for c in COMPONENTS:
            s = run["components"][c]
            flag = "above ~2 ms" if s["flagged"] else ("tail" if s["p95_over"] else "")
            lines.append(f"| {c} | {s['mean_ms']:.3f} | {s['p50_ms']:.3f} | "
                         f"{s['p95_ms']:.3f} | {s['max_ms']:.3f} | {flag} |")
        w = run["whole_step"]
        whole = (f"| **whole step** | {w['mean_ms']:.3f} | {w['p50_ms']:.3f} | "
                 f"{w['p95_ms']:.3f} | {w['max_ms']:.3f} | |")
        by_pass = ", ".join(f"{v:.1f}" for v in run["aggregate_steps_per_s_by_pass"])
        lines += [whole, "", f"Aggregate steps/s by pass: {by_pass}", ""]
        for sp in run["spikes"]:
            lines.append(f"- spike: pass {sp['pass']}, instance {sp['instance']}, "
                         f"{sp['component']}, steps {sp['steps']}")
        lines.append("")
    (out / "report.md").write_text("\n".join(lines))
    print(f"\n-> {out}  (profiling pass: no PASS/FAIL)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
