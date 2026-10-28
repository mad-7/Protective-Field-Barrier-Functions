"""The declared real-time measurement (ruled 2026-10-27), run by the researcher ON THE TRAINING HOST.

**The criterion** (`preregistration/gates.yaml`, `realtime_compute`, A28): p95 compute per step
<= 50 ms, the control period, on the training host, for every controller.

**What this measures**, with no simulator: each part drives the robot KINEMATICALLY through synthetic
straight aisles 0.7, 0.8 and 0.9 m wide (`AISLES`) -- the command passed through the box, the wheel
budget and the acceleration limiter exactly as the base applies it, the pose integrated at the
mid-step heading, the velocity estimate the estimator's first-order filter of the applied command,
the 120-bin filter scan ray-cast against the aisle's walls -- and times each step's decision.

  * **mppi** -- B6 at candidate 0 for every pair of samples {256, 512, 1024} x horizon {15, 20, 30}
    (`PAIRS`), `STEPS` steps per aisle. **The rule:** the LARGEST pair, by samples x horizon (the
    rollout work), whose p95 over all three aisles is <= the limit; a tie in the product goes to the
    longer horizon (confirmed 2026-10-27). **Declared outcome (A28a): if even 256 x 15 exceeds the
    limit, STOP and report** -- B6 cannot meet the criterion on this host, and the criterion is not
    relaxed. The selected pair is written into `baselines.mppi` with this run's
    directory as `compute_measurement`, by the researcher, in one commit.
  * **pfb** -- the PFB on a constant 0.4 m/s request along the aisle, with a box across the aisle
    `BLOCK_AT` ahead, so the robot first moves and then is held (STALLED: the PFB's own output at
    rest, the obstacle within the at-rest field's reach; a step is stalled when the applied speed
    is at or below the certified layer's standstill ENTER speed and the yaw rate below its yaw
    threshold -- the PFB creeps toward the box and settles there). p95 of the moving and of the
    stalled steps, `PFB_STEPS` per aisle. A stalled p95 above the limit is reported as a finding and limitation (ruled 2026-10-27).
  * **classical** -- B1 (Pure Pursuit), B2 (RPP) and B3 (DWA) at their configured values, the same
    aisles, for the results' per-step compute table.

The scenario sets are not used (the aisles are synthetic), so no test or tune layout is involved. Run
once per host: python scripts/measure_realtime.py --config configs/base.yaml --seed 0
Writes runs/realtime/REALTIME_<stamp>_seed<seed>/summary.json (and provenance.json, with the host).
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import time
from collections.abc import Callable
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pfbnav.config import load_config
from pfbnav.envs.controller_input import ClassicalInput, RouteView, ScanStream
from pfbnav.sim.robot import (
    ActuationLimits,
    WheelBudget,
    clamp_velocity,
    limit_acceleration,
    project_to_wheel_budget,
)
from pfbnav.utils.runio import create_run_dir

REPO_ROOT = Path(__file__).resolve().parents[1]
AISLES = (0.7, 0.8, 0.9)                  # m, aisle widths (ruled 2026-10-27: 0.7-0.9 m)
PAIRS = [(s, h) for s in (256, 512, 1024) for h in (15, 20, 30)]   # (samples, horizon_steps)
STEPS = 200                               # timed steps per aisle and setting (10 s of driving)
WARMUP = 5                                # untimed steps first (table builds, caches)
AISLE_LENGTH = 14.0                       # m, the walls run from x = -2 to x = 12
BLOCK_AT = 1.0                            # m, the box across the aisle in the pfb part
PFB_STEPS = 400                           # timed steps per aisle in the pfb part (it stalls ~step 150)
PFB_REQUEST = 0.4                         # m/s, the PFB part's constant request along the aisle
BINS = 120


# ------------------------------------------------------------------ geometry (pure)
def aisle_segments(width: float, block_at: float | None = None) -> np.ndarray:
    """The aisle's walls as world segments (S, 2, 2), m: y = +-width / 2 from x = -2 to 12, and a
    wall across it at x = block_at when given."""
    h = width / 2.0
    segs = [[[-2.0, h], [AISLE_LENGTH - 2.0, h]], [[-2.0, -h], [AISLE_LENGTH - 2.0, -h]]]
    if block_at is not None:
        segs.append([[block_at, -h], [block_at, h]])
    return np.asarray(segs, dtype=float)


def cast(pose: np.ndarray, bearings: np.ndarray, segments: np.ndarray,
         range_max: float) -> np.ndarray:
    """Ranges, m, from pose (x, y, yaw) along body-frame bearings to the nearest segment, range_max
    where nothing is hit."""
    x, y, yaw = (float(v) for v in pose)
    ang = bearings + yaw
    d = np.column_stack([np.cos(ang), np.sin(ang)])                 # (B, 2)
    a, e = segments[:, 0], segments[:, 1] - segments[:, 0]          # (S, 2)
    rel = a - np.array([x, y])                                      # (S, 2)
    den = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]          # (B, S)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (rel[None, :, 0] * e[None, :, 1] - rel[None, :, 1] * e[None, :, 0]) / den
        u = (rel[None, :, 0] * d[:, None, 1] - rel[None, :, 1] * d[:, None, 0]) / den
    hit = (np.abs(den) > 1e-12) & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
    return np.minimum(np.where(hit, t, np.inf).min(axis=1), range_max)


def select_pair(p95: dict[tuple[int, int], float], limit: float) -> tuple[int, int] | None:
    """The declared rule: the largest (samples, horizon) by samples x horizon whose p95 (s) is <=
    `limit`; a tie in the product goes to the longer horizon. None if no pair passes."""
    ok = [pair for pair, t in p95.items() if t <= limit]
    if not ok:
        return None
    return max(ok, key=lambda sh: (sh[0] * sh[1], sh[1]))


def p95(times: list[float]) -> float:
    return float(np.percentile(times, 95)) if times else math.nan


# ------------------------------------------------------------------ the kinematic loop
class Aisle:
    """A robot driven kinematically down one aisle, as the base would apply each command."""

    def __init__(self, cfg: dict[str, Any], width: float, block_at: float | None = None):
        self.cfg = cfg
        self.dt = float(cfg["sim"]["dt"])
        self.limits = ActuationLimits.from_config(cfg)
        self.budget = WheelBudget.from_config(cfg)
        tau = float(cfg["estimation"]["velocity"]["filter_tau"])
        self.alpha = self.dt / (tau + self.dt)
        self.segments = aisle_segments(width, block_at)
        self.bearings = np.array([math.remainder(-math.pi + 2.0 * math.pi * (k + 0.5) / BINS,
                                                 2.0 * math.pi) for k in range(BINS)])
        self.range_max = float(cfg["scanners"]["range_max"])
        self.pose = np.zeros(3)
        self.applied = np.zeros(3)
        self.estimate = np.zeros(3)

    def scan(self) -> ScanStream:
        r = cast(self.pose, self.bearings, self.segments, self.range_max)
        return ScanStream.of(r, self.bearings, np.ones(BINS, dtype=bool))

    def input(self, stop_active: bool = False) -> ClassicalInput:
        """The classical input: the aisle's centreline ahead, in the body frame."""
        x, y, yaw = self.pose
        goal_x = AISLE_LENGTH - 3.0
        world = np.array([[x, 0.0], [min(x + 0.8, goal_x), 0.0], [goal_x, 0.0]])
        c, s = math.cos(yaw), math.sin(yaw)
        rel = world - np.array([x, y])
        body = np.column_stack([c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1]])
        route = RouteView.of(body, body[1], body[2], max(goal_x - x, 1e-6))
        return ClassicalInput.of(stop_active=stop_active, velocity=self.estimate,
                                 scan=self.scan(), route=route, dt=self.dt)

    def apply(self, command: np.ndarray) -> None:
        cmd = clamp_velocity(np.asarray(command, dtype=float), self.limits)
        cmd = project_to_wheel_budget(cmd, self.budget.filter_limit, self.budget.lever_arm).applied
        self.applied = limit_acceleration(cmd, self.applied, self.dt, self.limits)
        vx, vy, w = self.applied
        mid = self.pose[2] + 0.5 * w * self.dt
        self.pose = self.pose + self.dt * np.array([math.cos(mid) * vx - math.sin(mid) * vy,
                                                    math.sin(mid) * vx + math.cos(mid) * vy, w])
        self.estimate = self.estimate + self.alpha * (self.applied - self.estimate)


def time_controller(make: Callable[[], Any], cfg: dict[str, Any], seed: int) -> dict[str, Any]:
    """A classical controller's command, timed, STEPS steps per aisle after WARMUP."""
    out, all_times = {}, []
    for width in AISLES:
        ctrl = make()
        ctrl.reset()
        if hasattr(ctrl, "begin_episode"):
            ctrl.begin_episode({"scenario_seed": seed})
        aisle = Aisle(cfg, width)
        times = []
        for k in range(WARMUP + STEPS):
            inp = aisle.input()
            start = time.perf_counter()
            cmd = ctrl.command(inp)
            spent = time.perf_counter() - start
            aisle.apply(cmd)
            if k >= WARMUP:
                times.append(spent)
        all_times += times
        out[f"{width:.1f}"] = {"p95_ms": 1e3 * p95(times), "max_ms": 1e3 * max(times),
                               "travelled_m": float(aisle.pose[0])}
    out["all"] = {"p95_ms": 1e3 * p95(all_times), "max_ms": 1e3 * max(all_times),
                  "steps": len(all_times)}
    return out


def mppi_part(cfg: dict[str, Any], seed: int, limit: float) -> dict[str, Any]:
    from pfbnav.control.mppi import FieldAwareMPPI

    rows, table = {}, {}
    for samples, horizon in PAIRS:
        c = copy.deepcopy(cfg)
        c["baselines"]["mppi"].update(samples=samples, horizon_steps=horizon)
        res = time_controller(lambda c=c: FieldAwareMPPI.from_config(c), c, seed)
        rows[f"{samples}x{horizon}"] = res
        table[(samples, horizon)] = res["all"]["p95_ms"] / 1e3
        print(f"  B6 {samples} x {horizon}: p95 {res['all']['p95_ms']:.1f} ms", flush=True)
    chosen = select_pair(table, limit)
    return {"pairs": rows, "selected": None if chosen is None else
            {"samples": chosen[0], "horizon_steps": chosen[1]},
            "finding": None if chosen else ("STOP and report (A28a): even 256 x 15 exceeds the "
                                            "limit on this host; the criterion is not relaxed")}


def pfb_part(cfg: dict[str, Any], limit: float) -> dict[str, Any]:
    from pfbnav.safety import pfb_field
    from pfbnav.safety.footprint_table import FootprintTable
    from pfbnav.safety.pfb import PFB, PFBConfig

    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    field = pfb_field.PFBField(table.extent, pfb_field.field_params_from_config(cfg))
    rest_speed = float(cfg["certified"]["standstill_enter"])
    rest_yaw = float(cfg["certified"]["standstill_yaw_enter"])
    out, moving_all, stalled_all = {}, [], []
    for width in AISLES:
        pfb = PFB(field, PFBConfig.from_config(cfg, field))
        aisle = Aisle(cfg, width, block_at=BLOCK_AT)
        moving, stalled = [], []
        for k in range(WARMUP + PFB_STEPS):
            scan = aisle.scan()
            start = time.perf_counter()
            res = pfb.filter(np.array([PFB_REQUEST, 0.0, 0.0]), aisle.estimate, aisle.applied,
                             scan.ranges, scan.bearings, scan.valid)
            spent = time.perf_counter() - start
            at_rest = (float(np.hypot(*aisle.applied[:2])) <= rest_speed
                       and abs(float(aisle.applied[2])) <= rest_yaw)
            aisle.apply(res.u)
            if k >= WARMUP:
                (stalled if at_rest else moving).append(spent)
        moving_all += moving
        stalled_all += stalled
        out[f"{width:.1f}"] = {"moving_p95_ms": 1e3 * p95(moving),
                               "stalled_p95_ms": 1e3 * p95(stalled),
                               "moving_steps": len(moving), "stalled_steps": len(stalled),
                               "stopped_at_x_m": float(aisle.pose[0])}
    stalled_p95 = p95(stalled_all)
    out["all"] = {"moving_p95_ms": 1e3 * p95(moving_all), "stalled_p95_ms": 1e3 * stalled_p95,
                  "moving_steps": len(moving_all), "stalled_steps": len(stalled_all)}
    out["finding"] = (None if stalled_all and stalled_p95 <= limit else
                      "the PFB's stalled p95 exceeds the control period on this host: a finding "
                      "and limitation (ruled 2026-10-27)" if stalled_all else
                      "no stalled step was measured: the part did not hold the robot")
    return out


def classical_part(cfg: dict[str, Any], seed: int) -> dict[str, Any]:
    from pfbnav.control.dwa import HolonomicDWA
    from pfbnav.control.pure_pursuit import PurePursuit
    from pfbnav.control.regulated_pure_pursuit import RegulatedPurePursuit

    return {name: time_controller(lambda k=kind: k.from_config(cfg), cfg, seed)
            for name, kind in (("B1_pure_pursuit", PurePursuit),
                               ("B2_regulated_pure_pursuit", RegulatedPurePursuit),
                               ("B3_holonomic_dwa", HolonomicDWA))}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--parts", nargs="+", default=["mppi", "pfb", "classical"],
                        choices=["mppi", "pfb", "classical"])
    args = parser.parse_args()
    cfg = load_config(args.config)
    gates = yaml.safe_load((REPO_ROOT / "preregistration" / "gates.yaml").read_text(
        encoding="utf-8"))
    if "realtime_compute" not in gates:
        raise SystemExit("preregistration/gates.yaml has no realtime_compute (A28): the criterion "
                         "is preregistered and relocked before it is measured")
    limit = float(gates["realtime_compute"]["p95_max_s"])
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "realtime", f"REALTIME_{stamp}", args.seed, cfg)
    summary: dict[str, Any] = {"limit_ms": 1e3 * limit, "aisles_m": list(AISLES),
                               "steps_per_aisle": STEPS, "parts": {}}
    for part in args.parts:
        print(f"{part} ...", flush=True)
        summary["parts"][part] = (mppi_part(cfg, args.seed, limit) if part == "mppi" else
                                  pfb_part(cfg, limit) if part == "pfb" else
                                  classical_part(cfg, args.seed))
        (run_dir / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    m = summary["parts"].get("mppi")
    if m:
        print(f"B6 selected: {m['selected']} {m['finding'] or ''}")
    p = summary["parts"].get("pfb")
    if p:
        print(f"PFB stalled p95 {p['all']['stalled_p95_ms']:.1f} ms {p['finding'] or ''}")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
