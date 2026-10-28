"""Is a_max = 0.5 m/s^2 (and alpha_max = 1.4 rad/s^2) achievable? A20, ruled 2026-10-18: "measure in
the simulator that 0.5 m/s^2 is achievable (the limiter binds, not the torque cap)". Researcher runs
it (~3 min, one instance, the plain scene with no hall loaded).

    python scripts/check_acceleration.py --config configs/base.yaml --seed 0

Five profiles from rest, each through the base's own path (box, wheel budget, rate limiter):
forward to (v_max, 0, 0), lateral to (0, vy_max, 0), diagonal (0.30, 0.25, 0), yaw to (0, 0, 1.0),
and combined (0.25, 0, 0.6); held, then commanded to zero. Every step logs the applied command,
the simulator's body velocity and how many wheels sit at the torque cap.

**Criteria, declared before running:**
  C1  the LIMITER binds: during every acceleration ramp, each step's applied change is a_max dt
      (translation, as a vector norm) or alpha_max dt (yaw), exactly (1e-9), until the target;
  C2  NOT the torque cap: zero wheel-steps at the torque cap during the acceleration ramps (an
      exact count -- no tolerance, so no false-alarm rate to declare under hard rule 19).
  PASS = C1 and C2 on every profile, AND the positive control below.
**Positive control:** a velocity STEP to the saturating diagonal (v_max, vy_max) with the limiter
bypassed must put wheels at the torque cap; if it does not, the counter is blind and the result is
INCONCLUSIVE.
**Reported, not gating:** the achieved acceleration from the simulator's velocity (the slope over
the ramp's middle 60 %, against the limit), the lag behind the applied command, and the same over
the braking ramp (a_decel_max). A tolerance on these would need its own null test first (rule 19).
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from pfbnav.config import load_config
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.robot import ActuationLimits, YouBotBase
from pfbnav.utils.runio import create_run_dir, stamp_display
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
HOLD_S = 1.0                    # s at the target before the zero command
BRAKE_S = 1.5                   # s after the zero command
EXACT = 1e-9                    # C1's tolerance: the limiter is arithmetic, not physics


def profiles(limits: ActuationLimits) -> dict[str, np.ndarray]:
    return {"forward": np.array([limits.v_max, 0.0, 0.0]),
            "lateral": np.array([0.0, limits.vy_max, 0.0]),
            "diagonal": np.array([0.30, 0.25, 0.0]),
            "yaw": np.array([0.0, 0.0, 1.0]),
            "combined": np.array([0.25, 0.0, 0.6])}


def ramp_slope(t: np.ndarray, value: np.ndarray, target: float) -> float | None:
    """Slope (per s) of `value` over the middle 60 % of its rise toward `target`; None if the
    rise never spans it."""
    lo, hi = 0.2 * target, 0.8 * target
    idx = np.flatnonzero((value >= lo) & (value <= hi))
    if idx.size < 3:
        return None
    return float(np.polyfit(t[idx], value[idx], 1)[0])


def run_profile(client, cfg, command: np.ndarray, *, rate_limit: bool = True) -> dict:
    limits = ActuationLimits.from_config(cfg)
    dt = float(cfg["sim"]["dt"])
    client.stop()
    client.start(settle_steps=20)
    robot = YouBotBase.from_config(client, cfg)
    robot.bind()
    robot.reset_command_state()
    rows = []
    final = _expected_final(command, robot)
    hold: int | None = None                 # first step at the target
    brake: int | None = None                # first step of the zero command
    step = 0
    try:
        while True:
            phase = "ramp" if hold is None else ("hold" if brake is None else "brake")
            applied = robot.command_body_velocity(command if brake is None else np.zeros(3),
                                                  rate_limit=rate_limit)
            client.step()
            capped = robot.record_torque_cap()
            rows.append({"t": step * dt, "applied": applied.tolist(),
                         "true": robot.body_velocity().tolist(), "capped": int(capped),
                         "phase": phase})
            if hold is None and np.allclose(applied, final, atol=1e-12):
                hold = step
            if hold is not None and brake is None and (step - hold) * dt >= HOLD_S:
                brake = step
            if brake is not None and (step - brake) * dt >= BRAKE_S:
                break
            step += 1
            if step * dt > 30.0:
                raise RuntimeError("profile did not finish in 30 s")
    finally:
        robot.halt()
    return {"rows": rows, "dt": dt, "limits": {"a_max": limits.a_max,
                                               "alpha_max": limits.alpha_max,
                                               "a_decel_max": limits.a_decel_max}}


def _expected_final(command: np.ndarray, robot: YouBotBase) -> np.ndarray:
    """The command the base settles on: clamped into the box and projected into the budget."""
    from pfbnav.sim.robot import clamp_velocity, project_to_wheel_budget
    c = clamp_velocity(command, robot.limits)
    if robot.budget is not None:
        c = project_to_wheel_budget(c, robot.budget.filter_limit, robot.budget.lever_arm).applied
    return c


def assess(result: dict) -> dict:
    rows, dt, lim = result["rows"], result["dt"], result["limits"]
    applied = np.array([r["applied"] for r in rows])
    true = np.array([r["true"] for r in rows])
    t = np.array([r["t"] for r in rows])
    ramp = np.array([r["phase"] == "ramp" for r in rows])
    brake = np.array([r["phase"] == "brake" for r in rows])
    prev = np.vstack([np.zeros(3), applied[:-1]])
    dlin = np.hypot(*(applied[:, :2] - prev[:, :2]).T)
    dyaw = np.abs(applied[:, 2] - prev[:, 2])
    final = applied[ramp][-1] if ramp.any() else applied[0]
    lin_target, yaw_target = float(np.hypot(*final[:2])), abs(float(final[2]))
    # C1: every ramp step but the last (which lands on the target) moves by the limit exactly.
    ramp_idx = np.flatnonzero(ramp)
    inner = ramp_idx[:-1] if ramp_idx.size > 1 else ramp_idx
    # Per step: it moved by exactly the limit, or that channel was already at its target.
    at_lin = np.hypot(*applied[inner, :2].T) >= lin_target - EXACT
    at_yaw = np.abs(applied[inner, 2]) >= yaw_target - EXACT
    c1_lin = lin_target == 0.0 or bool(np.all(
        (np.abs(dlin[inner] - lim["a_max"] * dt) < EXACT) | at_lin))
    c1_yaw = yaw_target == 0.0 or bool(np.all(
        (np.abs(dyaw[inner] - lim["alpha_max"] * dt) < EXACT) | at_yaw))
    capped_ramp = int(sum(r["capped"] for r, m in zip(rows, ramp) if m))
    speed = np.hypot(true[:, 0], true[:, 1])
    out = {"C1_limiter_binds": bool(c1_lin and c1_yaw),
           "C2_torque_cap_wheel_steps_in_ramp": capped_ramp,
           "torque_cap_wheel_steps_total": int(sum(r["capped"] for r in rows)),
           "ramp_steps": int(ramp.sum())}
    if lin_target > 0.0:
        s = ramp_slope(t[ramp], speed[ramp], lin_target)
        out["achieved_accel"] = s
        out["achieved_over_a_max"] = None if s is None else s / lim["a_max"]
        b = brake & (speed > 0.0)
        if b.any():
            tb, sb = t[b] - t[b][0], lin_target - speed[b]
            sd = ramp_slope(tb, sb, lin_target)
            out["achieved_decel"] = sd
    if yaw_target > 0.0:
        s = ramp_slope(t[ramp], np.abs(true[ramp, 2]), yaw_target)
        out["achieved_alpha"] = s
        out["achieved_over_alpha_max"] = None if s is None else s / lim["alpha_max"]
    out["passes"] = bool(out["C1_limiter_binds"] and capped_ramp == 0)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    limits = ActuationLimits.from_config(cfg)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"ACCELERATION_{stamp}", args.seed, cfg)
    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))
    report: dict = {"limits": {"a_max": limits.a_max, "alpha_max": limits.alpha_max,
                               "a_decel_max": limits.a_decel_max}, "profiles": {}}
    try:
        for name, command in profiles(limits).items():
            result = run_profile(client, cfg, command)
            (run_dir / f"trace_{name}.json").write_text(json.dumps(result["rows"]),
                                                       encoding="utf-8")
            report["profiles"][name] = assess(result)
            a = report["profiles"][name]
            print(f"{name:9s}: C1 {a['C1_limiter_binds']}, torque-capped wheel-steps in ramp "
                  f"{a['C2_torque_cap_wheel_steps_in_ramp']}, achieved "
                  + (f"{a['achieved_over_a_max']:.3f} x a_max " if a.get("achieved_over_a_max")
                     else "")
                  + (f"{a['achieved_over_alpha_max']:.3f} x alpha_max"
                     if a.get("achieved_over_alpha_max") else ""))
        step = run_profile(client, cfg, np.array([limits.v_max, limits.vy_max, 0.0]),
                           rate_limit=False)
        capped = sum(r["capped"] for r in step["rows"])
        report["positive_control_capped_wheel_steps"] = int(capped)
        print(f"positive control (velocity step, limiter bypassed): {capped} wheel-steps at "
              "the torque cap")
    finally:
        client.stop()
    ok = all(p["passes"] for p in report["profiles"].values())
    blind = report["positive_control_capped_wheel_steps"] == 0
    report["verdict"] = "INCONCLUSIVE (counter blind)" if blind else ("PASS" if ok else "FAIL")
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float),
                                          encoding="utf-8")
    stamp_display(run_dir, client.display_status())
    print(f"verdict: {report['verdict']}\n-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
