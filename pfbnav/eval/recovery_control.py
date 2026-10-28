"""The synthetic recovery control on current geometry (ruled 2026-10-16). Pure; no simulator.

**The scene**, constructed at run time like G0's cases, in the frame of the robot's start pose
(x forward, y left, m): one shelf whose near corner sits ahead-left of the robot, the shelf running
back from it along the robot's left. The goal is round that corner, so the route the planner draws
-- with today's margins, lethal 0.10 -- turns left round it. Holonomic Pure Pursuit (B1) steers at a
point one lookahead along that route, so at the bend it cuts inside, toward the corner, and the
certified layer stops it. Geometry and criteria are declared in
`configs/controls/recovery_synthetic.yaml` before any run.

**The intended stop state:** at rest after that first stop, with the at-rest field clear and the
restart blocked, B1's own requested direction not permitted. Only recovery can then release it.

**The known-answer check** (`predict`): before the simulator is used, this module emulates the step
loop's own chain on the constructed scene -- B1 (bare, or wrapped by the recovery), the certified
layer (`CertifiedLayer`, the emulator itself), the base's clamp, wheel budget and acceleration
limiter (`pfbnav/sim/robot.py`), the pose-differencing velocity estimate, and an exact ray-cast
scan of the shelf -- over a spread of wheel slip, since the platform realises 72-97 % of its
command (G1b). Every slip must produce the intended stop state, the recovery must release it, and B1
must then make the required progress; the bare arm must stay blocked. The simulator then runs the
same scene (`scripts/control_recovery_synthetic.py`) and is judged on the same criteria.
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pfbnav.envs.controller_input import ClassicalInput, RouteView, ScanStream
from pfbnav.envs.route_view import remaining_route, world_to_body
from pfbnav.eval.raycast import bin_bearings, ray_ranges, sector_scan, to_body_frame
from pfbnav.planning.astar import plan_from_config
from pfbnav.planning.lookahead import PathTracker
from pfbnav.safety.certified import CertifiedLayer
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.robot import (
    ActuationLimits,
    WheelBudget,
    clamp_velocity_with_status,
    limit_acceleration,
    project_to_wheel_budget,
)
from pfbnav.sim.velocity_estimate import PoseDifferenceEstimator, VelocityEstimateConfig
from pfbnav.world.baking import Obstacle, bake, planner_radii, polygon_distance
from pfbnav.world.strata import certified_params_from_config

REPO_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class ControlSpec:
    """The declared scene and criteria (`configs/controls/recovery_synthetic.yaml`)."""

    corner: tuple[float, float]        # m, the shelf's near corner in the start frame
    shelf_length: float                # m, back from the corner along -x
    shelf_depth: float                 # m, from the corner along +y
    goal_from_corner: tuple[float, float]   # m, the goal relative to the corner
    map_margin: float                  # m of free floor round the scene in the planner's map
    steps: int                         # control steps per arm
    slips: tuple[float, ...]           # realised / commanded translation, the known-answer spread
    stop_within_steps: int             # the first stop must come this early
    min_at_rest_margin: float          # m, at-rest field clear by at least this at the stop
    min_block_depth: float             # m, B1's band-0 leading field breached by at least this
    min_route_clearance: float         # m, the planned route's clearance from the shelf
    progress_after_release: float      # m of route after the release (ruled: 0.5)
    bare_max_progress: float           # m the bare arm may advance after its stop

    @classmethod
    def from_file(cls, path: str | Path) -> ControlSpec:
        d = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        s, c = d["scene"], d["criteria"]
        cx, cy = (float(v) for v in s["corner_m"])
        gx, gy = (float(v) for v in s["goal_from_corner_m"])
        return cls(corner=(cx, cy),
                   shelf_length=float(s["shelf_length_m"]), shelf_depth=float(s["shelf_depth_m"]),
                   goal_from_corner=(gx, gy),
                   map_margin=float(s["map_margin_m"]), steps=int(d["steps"]),
                   slips=tuple(float(v) for v in d["known_answer"]["slips"]),
                   stop_within_steps=int(c["stop_within_steps"]),
                   min_at_rest_margin=float(c["min_at_rest_margin_m"]),
                   min_block_depth=float(c["min_block_depth_m"]),
                   min_route_clearance=float(c["min_route_clearance_m"]),
                   progress_after_release=float(c["progress_after_release_m"]),
                   bare_max_progress=float(c["bare_max_progress_m"]))


def shelf_polygon(spec: ControlSpec) -> np.ndarray:
    """The shelf in the start frame, counter-clockwise: from the corner back along -x and up +y."""
    cx, cy = spec.corner
    x0, y1 = cx - spec.shelf_length, cy + spec.shelf_depth
    return np.array([[x0, cy], [cx, cy], [cx, y1], [x0, y1]], dtype=float)


def goal_of(spec: ControlSpec) -> tuple[float, float]:
    return (spec.corner[0] + spec.goal_from_corner[0], spec.corner[1] + spec.goal_from_corner[1])


def plan_route(cfg: dict[str, Any], spec: ControlSpec) -> np.ndarray:
    """The planner's own route from the start (0, 0) to the goal, at the configured margins."""
    shelf = shelf_polygon(spec)
    goal = goal_of(spec)
    pts = np.vstack([shelf, [[0.0, 0.0], goal]])
    lo, hi = pts.min(axis=0) - spec.map_margin, pts.max(axis=0) + spec.map_margin
    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    baked = bake([Obstacle(shelf, mapped=True, name="shelf_control")],
                 (float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1])),
                 float(cfg["planner"]["resolution"]),
                 planner_radii(cfg, circumscribed=float(table.circumscribed)))
    plan = plan_from_config(baked, (0.0, 0.0), goal, cfg)
    if not plan.ok:
        raise ValueError(f"the control's route does not plan: {plan.status}")
    return np.asarray(plan.path_world, dtype=float)


def route_clearance(route: np.ndarray, spec: ControlSpec, step: float = 0.005) -> float:
    """The route's least distance to the shelf, m, sampled every `step` along it."""
    pts = np.vstack([np.linspace(a, b, max(2, int(np.ceil(np.linalg.norm(b - a) / step))))
                     for a, b in itertools.pairwise(route)])
    return float(np.min(polygon_distance(pts, shelf_polygon(spec))))


def classical_input(tracker: PathTracker, pose, ranges, bearings, stop_active: bool, velocity,
                    dt: float, valid=None) -> tuple[ClassicalInput, Any]:
    """The typed input, built as `NavEnv._controller_input` builds it, from a start-frame pose."""
    ts = tracker.update(pose)
    route = world_to_body(remaining_route(tracker.path, ts.s), pose)
    ahead = world_to_body(np.vstack([ts.lookahead_world, ts.goal_world]), pose)
    ok = np.ones(len(ranges), bool) if valid is None else np.asarray(valid, dtype=bool)
    inp = ClassicalInput.of(stop_active=stop_active, velocity=velocity,
                            scan=ScanStream.of(ranges, bearings, ok),
                            route=RouteView.of(route, ahead[0], ahead[1], ts.remaining), dt=dt)
    return inp, ts


def _layer(cfg: dict[str, Any]) -> CertifiedLayer:
    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    return CertifiedLayer(certified_params_from_config(cfg), float(cfg["robot"]["half_length"]),
                          float(cfg["robot"]["half_width"]), table.extent)


def stop_state(layer: CertifiedLayer, scan: ScanStream, direction) -> dict[str, float | bool]:
    """At rest: the at-rest field's margin (m, > 0 clear), whether `direction` may restart, and
    how deep the band-0 leading field along it is breached (m, > 0 breached)."""
    at_rest = layer.fields.select(scan.bearings, 0.0, 0.0, at_rest=True)
    heading = math.atan2(float(direction[1]), float(direction[0]))
    lead = layer.fields.extent(scan.bearings, layer.params.band_top_speeds[0], heading,
                               at_rest=False)
    ahead = np.cos(scan.bearings - heading) > 0.0
    depth = float(np.max((lead - scan.ranges)[ahead])) if ahead.any() else -math.inf
    return {"at_rest_margin": float(np.min(scan.ranges - at_rest)),
            "restart_permitted": bool(layer._restart_allowed(scan.ranges, scan.bearings,
                                                             scan.valid, direction)),
            "block_depth": depth}


def emulate(cfg: dict[str, Any], spec: ControlSpec, route: np.ndarray, controller, slip: float,
            steps: int | None = None) -> list[dict[str, Any]]:
    """One arm on the constructed scene: `emulate_layout` with the control's shelf, from (0, 0, 0)."""
    return emulate_layout(cfg, [shelf_polygon(spec)], route, (0.0, 0.0, 0.0), controller, slip,
                          steps or spec.steps)


def emulate_layout(cfg: dict[str, Any], polygons, route: np.ndarray, start_pose, controller,
                   slip: float, steps: int, *, rotation_blind: bool = False,
                   episode: dict[str, Any] | None = None, pfb=None,
                   goal: tuple[float, float] | None = None,
                   goal_tolerance: float | None = None,
                   stall: tuple[int, float] | None = None,
                   sector_rays: int | None = None) -> list[dict[str, Any]]:
    """One arm, emulated step by step in the step loop's order, on any layout (polygons in the
    route's frame) from `start_pose` (x, y, yaw). Its positive control: replaying the 2026-10-13
    corner fixture reproduces the simulator's stops (`tests/test_recovery_control.py`).

    `controller` is a classical controller (B1, or B1 wrapped by the recovery). Translation is
    realised at `slip` x the applied command; rotation as applied. Returns one record per step.
    `rotation_blind=True` hands the layer a zero yaw rate and no applied command. Before A20
    (2026-10-18) that was the field as it was before 2026-10-16; since A20 the reach terms remain
    at zero yaw rate, so it no longer recreates any historical field. Otherwise the layer gets the
    applied command, as the step loop gives it.

    `pfb` (WP2, 2026-10-23): a `pfbnav.safety.pfb.PFB` in the step loop's filter slot. The
    controller's command is its NOMINAL; the PFB's output is what the certified layer receives and
    judges, exactly as `pfb_stage` orders it. It filters on the same scan and estimate the layer
    uses, with the last applied command, and the layer's latch flag. `goal` with `goal_tolerance`
    ends the run once the robot is that close to it. `stall` = (steps, metres) ends it, marked
    `stalled`, once the route progress `s` over the last `steps` steps is below `metres`. Without
    `pfb`, `goal` and `stall`, unchanged.

    `sector_rays` (2026-10-24): the scan is the simulator's FILTER stream -- per sector the nearest
    of that many rays, at its exact bearing (`raycast.sector_scan`) -- instead of one ray per bin
    centre. The PFB is designed for that stream; without it, as before.
    """
    dt = float(cfg["sim"]["dt"])
    bins = int(cfg["scanners"]["filter_bins"])
    range_max = float(cfg["scanners"]["range_max"])
    bearings = bin_bearings(bins)
    polygons = [np.asarray(p, dtype=float) for p in polygons]
    limits, budget = ActuationLimits.from_config(cfg), WheelBudget.from_config(cfg)
    layer = _layer(cfg)
    estimator = PoseDifferenceEstimator(VelocityEstimateConfig.from_config(cfg))
    tracker = PathTracker(route, float(cfg["planner"]["lookahead"]),
                          window=float(cfg["planner"]["track_window"]))
    controller.reset()
    if episode is not None:               # as the runner does after each reset (2026-10-20)
        controller.begin_episode(episode)
    pose = np.asarray(start_pose, dtype=float).copy()
    velocity = estimator.update((float(pose[0]), float(pose[1]), float(pose[2])))
    applied = np.zeros(3)
    stop_active = False
    t = 0.0
    out = []
    for step in range(1, steps + 1):
        decided = pose.copy()                    # the pose this step's decision is taken at
        body = [to_body_frame(p, pose) for p in polygons]
        if sector_rays:
            ranges, bearings = sector_scan(bins, body, range_max, sector_rays)
        else:
            ranges = ray_ranges(bearings, body, range_max)
        inp, ts = classical_input(tracker, pose, ranges, bearings, stop_active, velocity, dt)
        requested = np.asarray(controller.command(inp), dtype=float)
        base = getattr(controller, "base", controller)
        base_request = np.asarray(base.command(inp), dtype=float)
        pfb_info = None
        if pfb is not None:
            res = pfb.filter(requested, velocity, applied, ranges, bearings, np.ones(bins, bool),
                             latched=stop_active)
            pfb_info = {"mode": res.mode, "regime": res.regime, "infeasible": res.infeasible,
                        "reason": res.reason, "u": res.u.tolist(), "heading": res.heading,
                        "yaw_field": res.yaw_field, "nominal": requested.tolist(),
                        "applied_before": applied.tolist()}
            requested = res.u
        sensed = (np.array([velocity[0], velocity[1], 0.0]) if rotation_blind
                  else velocity)
        cert = layer.step(t, sensed, ranges, bearings, np.ones(bins, bool),
                          intended_direction=requested,
                          applied=(None if rotation_blind else applied))
        allowed = layer.override(requested)
        clamped = clamp_velocity_with_status(allowed, limits).applied
        clamped = project_to_wheel_budget(clamped, budget.filter_limit, budget.lever_arm).applied
        applied = limit_acceleration(clamped, applied, dt, limits)
        realised = np.array([applied[0] * slip, applied[1] * slip, applied[2]])
        mid = pose[2] + 0.5 * realised[2] * dt
        c, s = math.cos(mid), math.sin(mid)
        pose = pose + dt * np.array([c * realised[0] - s * realised[1],
                                     s * realised[0] + c * realised[1], realised[2]])
        pose[2] = math.remainder(pose[2], 2.0 * math.pi)
        t += dt
        velocity = estimator.update((float(pose[0]), float(pose[1]), float(pose[2])))
        stop_active = bool(cert["stop_active"])
        clearance = float(min(np.min(polygon_distance(pose[None, :2], p)) for p in polygons))
        # The field's heading at this decision (the layer keeps it), so the field in force at a
        # trigger can be rebuilt for the containment check (A20).
        out.append({"step": step, "s": float(ts.s), "pose": pose.tolist(), "cert": dict(cert),
                    "decision_pose": decided.tolist(), "field_heading": float(layer._heading),
                    "requested": requested.tolist(), "base_request": base_request.tolist(),
                    "scan": ScanStream.of(ranges, bearings, np.ones(bins, bool)),
                    "centre_clearance": clearance,
                    "mode": getattr(controller, "mode", None),
                    **({"pfb": pfb_info, "applied": applied.tolist(),
                        "velocity": np.asarray(velocity, dtype=float).tolist()}
                       if pfb is not None else {})})
        if goal is not None and goal_tolerance is not None and \
                math.hypot(pose[0] - goal[0], pose[1] - goal[1]) <= goal_tolerance:
            out[-1]["reached_goal"] = True
            break
        if stall is not None and len(out) > stall[0] and                 out[-1]["s"] - out[-1 - stall[0]]["s"] < stall[1]:
            out[-1]["stalled"] = True
            break
    return out


def premise(records: list[dict[str, Any]], layer: CertifiedLayer,
            stop_within: int) -> dict[str, Any]:
    """The scene's PREMISE only (ruling (c), 2026-10-16): the first stop, and the state at rest
    after it -- the at-rest margin, whether B1's restart is permitted, the block depth. Reads
    nothing that happens after the robot comes to rest, so the outcome (release, progress,
    contact) cannot steer a search that uses it."""
    first = next((r for r in records if r["cert"]["stop_triggered"]
                  and r["step"] <= stop_within), None)
    if first is None:
        return {"stopped": False}
    rest = next((r for r in records if r["step"] >= first["step"] and r["cert"]["at_rest"]
                 and r["cert"]["stop_active"]), None)
    if rest is None:
        return {"stopped": True, "first_stop_step": first["step"], "rested": False}
    return {"stopped": True, "rested": True, "first_stop_step": first["step"],
            **stop_state(layer, rest["scan"], rest["base_request"])}


def premise_holds(p: dict[str, Any], spec: ControlSpec) -> bool:
    return bool(p.get("rested")) and p["at_rest_margin"] >= spec.min_at_rest_margin         and p["restart_permitted"] is False and p["block_depth"] >= spec.min_block_depth


def analyse(records: list[dict[str, Any]], layer: CertifiedLayer) -> dict[str, Any]:
    """The first stop, the state at rest after it, the release and the progress after it."""
    first = next((r for r in records if r["cert"]["stop_triggered"]), None)
    if first is None:
        return {"stopped": False}
    rest = next((r for r in records if r["step"] >= first["step"] and r["cert"]["at_rest"]
                 and r["cert"]["stop_active"]), None)
    state = (stop_state(layer, rest["scan"], rest["base_request"])
             if rest is not None else None)
    release = next((r for r in records if rest is not None and r["step"] > rest["step"]
                    and not r["cert"]["stop_active"]), None)
    end_s = records[-1]["s"]
    return {"stopped": True, "first_stop_step": first["step"], "s_at_stop": first["s"],
            "rest_step": rest["step"] if rest else None, "stop_state": state,
            "released_step": release["step"] if release else None,
            "s_at_release": release["s"] if release else None,
            "progress_after_release": (end_s - release["s"]) if release else 0.0,
            "progress_after_stop": end_s - first["s"],
            "min_centre_clearance": min(r["centre_clearance"] for r in records)}


def judge(spec: ControlSpec, recovery: dict[str, Any], bare: dict[str, Any],
          recovery_report: dict[str, Any]) -> dict[str, bool]:
    """The declared criteria, one by one (the same for the emulation and the simulator)."""
    st = recovery.get("stop_state") or {}
    return {
        "stops_early": bool(recovery.get("stopped"))
        and recovery["first_stop_step"] <= spec.stop_within_steps,
        "at_rest_field_clear": st.get("at_rest_margin", -1.0) >= spec.min_at_rest_margin,
        "restart_blocked": st.get("restart_permitted") is False
        and st.get("block_depth", -1.0) >= spec.min_block_depth,
        "recovery_released": recovery.get("released_step") is not None
        and recovery_report["recovery_attempts"] >= 1
        and recovery_report["recovery_endings"]["not_released"] == 0,
        "progress_after_release": recovery.get("progress_after_release", 0.0)
        >= spec.progress_after_release,
        "bare_stays_blocked": bool(bare.get("stopped"))
        and bare["progress_after_stop"] <= spec.bare_max_progress,
    }


def predict(cfg: dict[str, Any], spec: ControlSpec, make_b1, make_recovery) -> dict[str, Any]:
    """The known-answer check: every declared slip must meet every criterion."""
    route = plan_route(cfg, spec)
    clearance = route_clearance(route, spec)
    layer = _layer(cfg)
    by_slip = {}
    for slip in spec.slips:
        rec = make_recovery(make_b1())
        rec_records = emulate(cfg, spec, route, rec, slip)
        bare_records = emulate(cfg, spec, route, make_b1(), slip)
        a_rec, a_bare = analyse(rec_records, layer), analyse(bare_records, layer)
        verdict = judge(spec, a_rec, a_bare, rec.episode_report())
        by_slip[slip] = {"recovery": a_rec, "bare": a_bare, "report": rec.episode_report(),
                         "criteria": verdict}
    return {"route": route.tolist(), "route_clearance": clearance,
            "route_clearance_ok": clearance >= spec.min_route_clearance,
            "by_slip": by_slip,
            "all_pass": clearance >= spec.min_route_clearance
            and all(all(v["criteria"].values()) for v in by_slip.values())}

