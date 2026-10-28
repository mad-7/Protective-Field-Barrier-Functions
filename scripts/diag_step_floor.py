"""Fixed-cost diagnosis: where the ~45-50 ms per sim.step() goes (supervisor, 2026-09-19).

The A10 probe showed the physics step barely depends on the physics work (10 substeps
per control step down to 1 saved ~10 ms of ~55), and each instance runs at ~12.5 steps/s
whether 1, 2 or 4 run. This measures ms per `sim.step()` with the dynamics step held at
the frozen 0.005 s, on one instance, across:

  warehouse scene (representative -- these may set the floor):
    as_loaded        settings exactly as the scene loads them
    realtime_off     sim.boolparam_realtime_simulation = False
    realtime_on      ... = True (tested explicitly either way)
    idle_fps_0       realtime off, sim.intparam_idle_fps = 0
    display_off      realtime off, sim.boolparam_display_enabled = False (GUI mode only)
  locating the cost (never set the floor):
    dynamics_off     warehouse, sim.boolparam_dynamics_handling_enabled = False
    no_scanners      warehouse with the two LiDAR pods removed. These are the robot's
                     **scanners**, not leftovers: `robot.half_length` is set by the
                     forward pod, the rear pod was fitted to make the envelope symmetric,
                     and `lua/scanner_v2.lua` (WP1) is to drive them. Nothing reads them
                     yet, so a vision sensor no script handles explicitly is rendered by
                     the main script every step for nothing. Removing them is NOT an
                     adoptable configuration: it measures what the scanners cost as they
                     are currently built, which is the budget the WP1 scanner design has
                     to fit. It never sets the floor.
    youbot_alone     every object removed except the youBot tree and the floor it rests on
    empty_scene      every object removed

Every configuration starts from a freshly loaded scene and the whole plan runs `--passes`
times, interleaved: the first run of this diagnosis (STEPFLOOR-gui_20260919T232851Z) made
`youbot_alone` look *slower* than the full scene, which cannot be a property of the
configuration, so single passes in a fixed order are not enough to separate them.

It also verifies stepping mode is in force (`sim.setStepping` level > 0) after connect,
after every scene (re)load and at the end; inventories the scene's scripts and sensors
(script type, enabled, executions per step; vision-sensor resolution), because a vision
sensor is rendered every step whether or not anything reads it; and times a bare remote
call (`sim.getSimulationTime`) as the I/O reference.

Headless versus GUI is a property of how the instance was launched, so `--mode` states it
and is checked against sim.boolparam_headless; the run refuses a mismatch. Every step-cost
measurement so far (the profiling pass, the A10 probe) was taken on GUI instances, so a
GUI run is the like-for-like one, and `display_off` -- rendering switched off inside a GUI
instance -- is the in-GUI equivalent of headless and can set the floor. If `display_off`
is where the floor falls, the environment adopts it at start (a `sim` config flag), which
costs no change to how instances are launched. A headless run is worth doing only to
confirm that nothing else about the launch mode matters.

The scene is modified in memory only (objects removed, parameters set) and reloaded from
disk afterwards. It is **never saved**: the script checks the scene file's SHA-256 before
and after and fails if it changed (hard rule 12).

Decision rule, set in advance (supervisor, 2026-09-19): if the floor -- the cheapest
representative configuration in the mode training will run in -- is below
~15 ms per step, proceed to the batched step and keep the 100 steps/s target; otherwise
stop optimising and re-plan the matrix at WP1 close (more instances, then
total_timesteps, then seeds for A1 and L2 -- never L1, L4, L5).

Usage (one instance on sim.port, launched headless, then again launched with the GUI):
    python scripts/diag_step_floor.py --config configs/base.yaml --seed 0 --mode headless
    python scripts/diag_step_floor.py --config configs/base.yaml --seed 0 --mode gui
"""
from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from pfbnav.config import load_config
from pfbnav.eval.profiling import floor_decision, summarise_timings
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.utils.runio import create_run_dir, file_sha256, stamp_display
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]

WARMUP_STEPS = 5          # not timed: the first contact/physics step after start is a spike
TIMED_STEPS = 200         # per configuration
ROUNDTRIP_CALLS = 200     # bare remote calls, per configuration

REPRESENTATIVE = ("as_loaded", "realtime_off", "realtime_on", "idle_fps_0", "display_off")


# ---------------------------------------------------------------------------------------
# Simulator probes
# ---------------------------------------------------------------------------------------
def stepping_level(sim: Any) -> int:
    """The client's stepping level, read without changing it.

    `sim.setStepping` is cumulative and returns the previous level, so enabling once and
    disabling once reads the level and leaves it where it was.
    """
    level = int(sim.setStepping(True))
    sim.setStepping(False)
    return level


def settings(sim: Any) -> dict[str, Any]:
    return {
        "headless": bool(sim.getBoolParam(sim.boolparam_headless)),
        "realtime_simulation": bool(sim.getBoolParam(sim.boolparam_realtime_simulation)),
        "idle_fps": int(sim.getInt32Param(sim.intparam_idle_fps)),
        "display_enabled": bool(sim.getBoolParam(sim.boolparam_display_enabled)),
        "dynamics_handling_enabled": bool(
            sim.getBoolParam(sim.boolparam_dynamics_handling_enabled)),
        "simulation_time_step": float(
            sim.getFloatParam(sim.floatparam_simulation_time_step)),
        "dynamic_step_size": float(sim.getFloatParam(sim.floatparam_dynamic_step_size)),
    }


def _constant_names(sim: Any, prefix: str, names: tuple[str, ...]) -> dict[int, str]:
    return {int(getattr(sim, f"{prefix}{n}")): n for n in names}


OBJECT_TYPES = ("shape", "joint", "graph", "camera", "dummy", "proximitysensor", "path",
                "visionsensor", "mill", "forcesensor", "light", "mirror", "octree",
                "pointcloud", "script")
SCRIPT_TYPES = ("main", "simulation", "customization", "addon", "sandbox", "passive")


def object_path(sim: Any, handle: int) -> str:
    parts = []
    h = int(handle)
    while h != -1:
        parts.append(str(sim.getObjectAlias(h)))
        h = int(sim.getObjectParent(h))
    return "/" + "/".join(reversed(parts))


def inventory(sim: Any) -> dict[str, Any]:
    """Every object by type, with scripts and sensors listed individually."""
    type_names = _constant_names(sim, "sceneobject_", OBJECT_TYPES)
    script_names = _constant_names(sim, "scripttype_", SCRIPT_TYPES)
    handles = [int(h) for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)]
    counts: dict[str, int] = {}
    scripts, vision, proximity = [], [], []
    for h in handles:
        kind = type_names.get(int(sim.getObjectType(h)), f"unknown({sim.getObjectType(h)})")
        counts[kind] = counts.get(kind, 0) + 1
        if kind == "script":
            stype = int(sim.getObjectInt32Param(h, sim.scriptintparam_type))
            scripts.append({
                "handle": h, "path": object_path(sim, h),
                "type": script_names.get(stype, f"unknown({stype})"),
                "enabled": bool(sim.getObjectInt32Param(h, sim.scriptintparam_enabled)),
            })
        elif kind == "visionsensor":
            vision.append({
                "handle": h, "path": object_path(sim, h),
                "resolution": [int(sim.getObjectInt32Param(h, sim.visionintparam_resolution_x)),
                               int(sim.getObjectInt32Param(h, sim.visionintparam_resolution_y))],
            })
        elif kind == "proximitysensor":
            proximity.append({"handle": h, "path": object_path(sim, h)})
    return {"objects": len(handles), "by_type": counts, "scripts": scripts,
            "vision_sensors": vision, "proximity_sensors": proximity}


def exec_counts(sim: Any, scripts: list[dict[str, Any]]) -> dict[int, int]:
    return {s["handle"]: int(sim.getObjectInt32Param(s["handle"], sim.scriptintparam_execcount))
            for s in scripts}


def scanner_roots(client: SimClient) -> list[int]:
    """Handles of the LiDAR pod subtrees: objects aliased `LiDAR`, wherever they sit.

    These are the robot's scanners (`scanners.policy_bins` / `filter_bins`, to be driven
    by `lua/scanner_v2.lua` in WP1), and the forward pod sets `robot.half_length`. Nothing
    reads them yet, and a vision sensor that no script handles explicitly is rendered by
    the main script every step, so they currently cost per step for nothing. Removing them
    measures that cost; it is not a configuration the environment could adopt, because the
    robot needs scans and the footprint is measured with the pods on.
    """
    sim = client.sim
    roots = [int(h) for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
             if str(sim.getObjectAlias(h)) == "LiDAR"]
    if not roots:
        raise SystemExit("no object aliased 'LiDAR' in the scene; the inventory said there "
                         "were some, so the scene or the alias has changed")
    return roots


def remove_subtrees(client: SimClient, roots: list[int]) -> int:
    """Remove each root and everything under it. The simulation must be stopped."""
    sim = client.sim
    doomed: set[int] = set()
    for root in roots:
        doomed.update(int(h) for h in sim.getObjectsInTree(root, sim.handle_all, 0))
        doomed.add(int(root))
    if doomed:
        sim.removeObjects(sorted(doomed))
    return len(doomed)


def remove_all_except(client: SimClient, keep_roots: list[int]) -> int:
    """Remove every object not in the subtree of a kept root. The simulation must be stopped."""
    sim = client.sim
    keep: set[int] = set()
    for root in keep_roots:
        keep.update(int(h) for h in sim.getObjectsInTree(root, sim.handle_all, 0))
    doomed = [int(h) for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
              if int(h) not in keep]
    if doomed:
        sim.removeObjects(doomed)
    return len(doomed)


# ---------------------------------------------------------------------------------------
# One timed configuration
# ---------------------------------------------------------------------------------------
def time_configuration(client: SimClient, name: str, set_params: dict[str, tuple[str, Any]],
                       scripts: list[dict[str, Any]]) -> dict[str, Any]:
    """Apply `set_params` (param name -> (kind, value)) while stopped, start, time, stop."""
    sim = client.sim
    for pname, (kind, value) in set_params.items():
        setter = {"bool": sim.setBoolParam, "int": sim.setInt32Param}[kind]
        setter(getattr(sim, pname), value)
    client.start(settle_steps=0)      # applies the frozen engine and 0.005 s dynamics step
    in_force = settings(sim)
    level = stepping_level(sim)
    if level < 1:
        raise SystemExit(f"{name}: stepping mode is not in force (level {level})")
    if abs(in_force["dynamic_step_size"] - client.connection.dynamic_step) > 1e-12:
        raise SystemExit(f"{name}: dynamics step {in_force['dynamic_step_size']} is not the "
                         f"frozen {client.connection.dynamic_step}")
    for _ in range(WARMUP_STEPS):
        client.step()
    live = [s for s in scripts if s["handle"] in
            {int(h) for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)}]
    before = exec_counts(sim, live)
    step_ms = []
    for _ in range(TIMED_STEPS):
        t0 = time.perf_counter_ns()
        sim.step()
        step_ms.append((time.perf_counter_ns() - t0) / 1e6)
    after = exec_counts(sim, live)
    call_ms = []
    for _ in range(ROUNDTRIP_CALLS):
        t0 = time.perf_counter_ns()
        sim.getSimulationTime()
        call_ms.append((time.perf_counter_ns() - t0) / 1e6)
    client.stop()
    result = {
        "settings_in_force": in_force, "stepping_level": level,
        "step": summarise_timings(step_ms), "roundtrip_call": summarise_timings(call_ms),
        "script_execs_per_step": {s["path"]: (after[s["handle"]] - before[s["handle"]])
                                  / TIMED_STEPS for s in live},
        "step_samples_ms": step_ms,
        "roundtrip_call_samples_ms": call_ms,
    }
    print(f"  {name:<14} step mean {result['step']['mean_ms']:6.2f} ms  p50 "
          f"{result['step']['p50_ms']:6.2f}  p95 {result['step']['p95_ms']:6.2f}  | call "
          f"{result['roundtrip_call']['mean_ms']:5.2f} ms  | realtime "
          f"{in_force['realtime_simulation']} idle_fps {in_force['idle_fps']} display "
          f"{in_force['display_enabled']} dynamics {in_force['dynamics_handling_enabled']}")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--mode", choices=("headless", "gui"), required=True,
                        help="how the instance was launched; checked against the simulator")
    parser.add_argument("--instance", type=int, default=0)
    parser.add_argument("--passes", type=int, default=3,
                        help="interleaved repetitions of the whole plan; a configuration's "
                             "cost is read across passes, so a slow patch of the machine "
                             "cannot be mistaken for a property of one configuration")
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    scene = REPO_ROOT / cfg["sim"]["scene"]
    scene_hash_before = file_sha256(scene)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"STEPFLOOR-{args.mode}_{stamp}",
                             args.seed, cfg)

    client = SimClient(SimConnection.from_config(cfg, instance_id=args.instance))
    client.connect()
    sim = client.sim
    report: dict[str, Any] = {"mode": args.mode, "instance": args.instance,
                              "warmup_steps": WARMUP_STEPS, "timed_steps": TIMED_STEPS,
                              "stepping_level": {"after_connect": stepping_level(sim)},
                              "configurations": {}}

    headless = bool(sim.getBoolParam(sim.boolparam_headless))
    if headless != (args.mode == "headless"):
        raise SystemExit(f"--mode {args.mode} but the instance reports headless={headless}")

    def reload(label: str) -> None:
        client.load_scene(str(scene))
        report["stepping_level"][f"after_reload_{label}"] = stepping_level(sim)

    reload("initial")
    report["settings_as_loaded"] = settings(sim)
    report["inventory"] = inventory(sim)
    scripts = report["inventory"]["scripts"]
    loaded = report["settings_as_loaded"]
    print(f"{args.mode}: settings as loaded {loaded}")
    print(f"inventory: {report['inventory']['by_type']}; {len(scripts)} scripts, "
          f"{len(report['inventory']['vision_sensors'])} vision sensors, "
          f"{len(report['inventory']['proximity_sensors'])} proximity sensors")

    configs = report["configurations"]
    restore = {"boolparam_realtime_simulation": ("bool", loaded["realtime_simulation"]),
               "intparam_idle_fps": ("int", loaded["idle_fps"]),
               "boolparam_dynamics_handling_enabled": ("bool", True)}
    if args.mode == "gui":           # the display is only touched where there is one
        restore["boolparam_display_enabled"] = ("bool", loaded["display_enabled"])
    rt_off = {"boolparam_realtime_simulation": ("bool", False)}
    # (name, parameters, what to strip from the scene). Every configuration starts from a
    # freshly loaded scene, so the strip is the only difference and the order cannot carry
    # state from one configuration into the next.
    plan: list[tuple[str, dict[str, tuple[str, Any]], str]] = [
        ("as_loaded", {}, ""),
        ("realtime_off", rt_off, ""),
        ("realtime_on", {"boolparam_realtime_simulation": ("bool", True)}, ""),
        ("idle_fps_0", {**rt_off, "intparam_idle_fps": ("int", 0)}, ""),
    ]
    if args.mode == "gui":
        plan.append(("display_off", {**rt_off, "boolparam_display_enabled": ("bool", False)},
                     ""))
    plan += [
        ("dynamics_off", {**rt_off,
                          "boolparam_dynamics_handling_enabled": ("bool", False)}, ""),
        ("no_scanners", {**rt_off}, "scanners"),
        ("youbot_alone", {**rt_off}, "youbot"),
        ("empty_scene", {**rt_off}, "empty"),
    ]
    if args.mode == "gui":
        # The two largest savings measured separately (rendering ~7 ms, the scanner
        # pods ~9.6 ms) may overlap: a vision sensor renders whether or not the display
        # does. Measured together they turn out to be additive (234447Z).
        display_off = {**rt_off, "boolparam_display_enabled": ("bool", False)}
        plan += [
            ("no_scanners_display_off", display_off, "scanners"),
            ("no_scanners_display_off_no_furniture", display_off, "youbot+scanners"),
        ]

    def strip(kinds: str) -> int:
        removed = 0
        for kind in (k for k in kinds.split("+") if k):
            if kind == "scanners":
                removed += remove_subtrees(client, scanner_roots(client))
            elif kind == "youbot":
                removed += remove_all_except(client, [client.object(objects["robot_tree"]),
                                                      client.object(objects["floor"])])
            elif kind == "empty":
                removed += remove_all_except(client, [])
            else:
                raise ValueError(f"unknown strip {kind!r}")
        return removed

    objects = cfg["sim"]["objects"]
    per_pass: dict[str, list[dict[str, Any]]] = {name: [] for name, _, _ in plan}
    try:
        for p in range(args.passes):
            print(f"pass {p}")
            for name, params, kind in plan:
                reload(f"p{p}_{name}")
                removed = strip(kind)
                r = time_configuration(client, name, {**restore, **params}, scripts)
                r["objects_removed"] = removed
                per_pass[name].append(r)
    finally:
        client.stop()
        reload("final")                  # leave the instance on the unmodified scene

    for name, runs in per_pass.items():
        samples = [x for r in runs for x in r["step_samples_ms"]]
        configs[name] = {
            "passes": len(runs),
            "step": summarise_timings(samples),
            "step_mean_ms_by_pass": [r["step"]["mean_ms"] for r in runs],
            "roundtrip_call": summarise_timings(
                [x for r in runs for x in r["roundtrip_call_samples_ms"]]),
            "objects_removed": runs[0]["objects_removed"],
            "settings_in_force": runs[0]["settings_in_force"],
            "stepping_level": [r["stepping_level"] for r in runs],
            "script_execs_per_step": runs[0]["script_execs_per_step"],
            "step_samples_ms": samples,
        }

    scene_hash_after = file_sha256(scene)
    if scene_hash_after != scene_hash_before:
        raise SystemExit("the scene file changed during the diagnosis; it must never be saved")
    report["scene_sha256_before_after"] = [scene_hash_before, scene_hash_after]

    means = {k: v["step"]["mean_ms"] for k, v in configs.items()}
    report["decision"] = floor_decision(means, [c for c in REPRESENTATIVE if c in configs])
    # The binding mode is the one training will run in, which is a decision about how the
    # instances are launched, not a property of this run. Every step-cost measurement so
    # far (profiling, A10) was taken with the GUI, so a GUI run is the like-for-like one;
    # `display_off` is the in-GUI equivalent of headless and can set the floor.
    report["decision"]["measured_in_mode"] = args.mode
    # The robot needs its scanners, so `no_scanners` is not an adoptable configuration
    # and never sets the floor. What it gives is the budget the WP1 scanner design has to
    # fit: the step cost with no scanning at all, against which any implementation (fewer
    # bins, explicit handling, a different scan method) is measured.
    # The scanner-free floor, excluding the variant that also strips the warehouse
    # furniture (which is no scene anyone could train in).
    scanner_free = [c for c in configs
                    if c.startswith("no_scanners") and "no_furniture" not in c]
    if scanner_free:
        best = floor_decision(means, scanner_free)
        report["scanner_budget"] = {
            "floor_ms": best["floor_ms"], "floor_config": best["floor_config"],
            "scanner_cost_ms": report["decision"]["floor_ms"] - best["floor_ms"],
            "decision_if_scanning_were_free": best["decision"],
            "note": "not adoptable: the robot needs scans and the footprint is measured "
                    "with the pods fitted. This is the budget the WP1 scanner design "
                    "must fit, not a configuration to run.",
        }

    samples = {k: v.pop("step_samples_ms") for k, v in configs.items()}
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float))
    stamp_display(run_dir, client.display_status())
    (run_dir / "step_samples_ms.json").write_text(json.dumps(samples))

    print(f"\n{'configuration':<14} {'mean ms':>8} {'p95':>7} {'by pass':>28}  removed")
    for name, c in configs.items():
        by_pass = " ".join(f"{m:6.2f}" for m in c["step_mean_ms_by_pass"])
        print(f"{name:<14} {c['step']['mean_ms']:8.2f} {c['step']['p95_ms']:7.2f} "
              f"{by_pass:>28}  {c['objects_removed']}")

    d = report["decision"]
    print(f"\nfloor {d['floor_ms']:.2f} ms/step ({d['floor_config']}) against "
          f"{d['threshold_ms']} ms -> {d['decision']}  (measured in {args.mode} mode)")
    if "scanner_budget" in report:
        sb = report["scanner_budget"]
        print(f"with no scanning at all ({sb['floor_config']}): {sb['floor_ms']:.2f} ms/step; "
              f"the scanners as built cost {sb['scanner_cost_ms']:.2f} ms of the floor. "
              f"Not adoptable -- it is the budget the WP1 scanner design must fit.")
    print(f"stepping levels: {report['stepping_level']}")
    print(f"-> {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
