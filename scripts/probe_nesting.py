"""Probe the NESTING invariant (ruled 2026-10-19, (a)): does the field in force, in a given state,
contain the at-rest field at every rest pose reachable from that state? Offline, ~2 minutes.

    python scripts/probe_nesting.py --config configs/base.yaml --seed 0

For each state (at rest still / spinning / creeping; moving straight and turning), the rest poses
are those of bang-bang limiter trajectories (12 acceleration directions or none, yaw acceleration
+-alpha_max or none, over the reach time T, then braking). At each rest pose the TARGET field is
placed and its boundary compared, radially, with the field in force at the state: the worst amount
it reaches outside (m; <= 0 nests). Four constructions and targets:
  * current      -- the A20 field; target the at-rest field (which carries the reach from rest);
  * literal_a    -- every field's yaw interval extended by e(0), the target included;
  * moving_ext   -- moving fields extended by e(0) in yaw and by the at-rest reach in position; the
                    at-rest field unextended;
  * standing     -- the A20 field; target the STANDING field, footprint + lateral margin, no reach.
A probe, not a test: it measures, and the constructions other than `current` are patched in here
only, never in the layer.
"""
from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

import pfbnav.safety.certified as C
from pfbnav.config import load_config
from pfbnav.eval.containment import in_frame
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.strata import certified_params_from_config

REPO_ROOT = Path(__file__).resolve().parents[1]
GRID = np.linspace(-math.pi, math.pi, 1440, endpoint=False)
STATES = [("rest, still", 0.0, 0.0, True), ("rest, spin 0.07", 0.0, 0.07, True),
          ("rest, spin 0.7", 0.0, 0.7, True), ("rest, creep 0.02", 0.02, 0.0, True),
          ("moving 0.10", 0.10, 0.0, False), ("moving 0.20 turning 0.3", 0.20, 0.3, False),
          ("moving 0.40", 0.40, 0.0, False)]
VARIANTS = ("current", "literal_a", "moving_ext", "standing")


def rest_poses(P, v0: float, w0: float, dt: float = 0.002) -> list[tuple[float, float, float]]:
    out = []
    for ang in np.radians(np.arange(0, 360, 30)):
        for alpha in (-P.alpha_max, P.alpha_max, 0.0):
            for push in (True, False):
                acc = np.array([math.cos(ang), math.sin(ang)]) * (P.a_max if push else 0.0)
                pos, th, v, w, t = np.zeros(2), 0.0, np.array([v0, 0.0]), w0, 0.0
                while t < P.reach_time:
                    v = v + acc * dt
                    s = np.hypot(*v)
                    if s > P.speed_cap:
                        v *= P.speed_cap / s
                    w = float(np.clip(w + alpha * dt, -P.yaw_cap, P.yaw_cap))
                    c, sn = math.cos(th), math.sin(th)
                    pos = pos + dt * np.array([c * v[0] - sn * v[1], sn * v[0] + c * v[1]])
                    th, t = th + w * dt, t + dt
                while np.hypot(*v) > 1e-9 or abs(w) > 1e-9:
                    s = np.hypot(*v)
                    v = v * max(s - P.a_brake * dt, 0.0) / s if s > 0 else v
                    w = math.copysign(max(abs(w) - P.alpha_brake * dt, 0.0), w)
                    c, sn = math.cos(th), math.sin(th)
                    pos = pos + dt * np.array([c * v[0] - sn * v[1], sn * v[0] + c * v[1]])
                    th += w * dt
                out.append((float(pos[0]), float(pos[1]), th))
    return out


def worst(F, P, target: np.ndarray, v0, w0, at_rest) -> float:
    in_force = F.select(GRID, v0, 0.0, at_rest, yaw_rate=w0)
    pts = np.column_stack([target * np.cos(GRID), target * np.sin(GRID)])
    out = -np.inf
    for pose in rest_poses(P, v0, w0):
        q = in_frame(pts, pose, (0.0, 0.0, 0.0))
        r, b = np.hypot(q[:, 0], q[:, 1]), np.arctan2(q[:, 1], q[:, 0])
        out = max(out, float(np.max(r - np.interp(b, GRID, in_force, period=2 * math.pi))))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--no-record", action="store_true", help="print only")
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    P = certified_params_from_config(cfg)
    F = C.FieldGeometry(FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
                        .extent, P)
    e0, rr = C.yaw_excursion(0.0, P), C.rest_reach(P)
    original_interval, original_support = C.yaw_interval, C.FieldGeometry.support
    results = {}
    try:
        for name in VARIANTS:
            C.yaw_interval, C.FieldGeometry.support = original_interval, original_support
            if name == "literal_a":
                C.yaw_interval = lambda w, p: (-(C.yaw_excursion(abs(w), p) + e0),
                                               C.yaw_excursion(abs(w), p) + e0)
            if name == "moving_ext":
                def support(self, a, top, heading, at_rest, *, warning=False, yaw_rate=0.0):
                    a = np.asarray(a, dtype=float)
                    if at_rest:
                        return original_support(self, a, top, heading, True, warning=warning,
                                                yaw_rate=yaw_rate)
                    p = self.params
                    e = C.yaw_excursion(abs(yaw_rate), p) + e0
                    c = np.cos(a - heading)
                    lead = np.maximum(0.0, c)
                    travel = np.maximum(C.band_depth(top, p) * lead, C.swerve(c, top, p))
                    own = (self.swept_interval(a, -e, e) + travel
                           + np.maximum((p.margin - p.lateral_margin) * lead, rr)
                           + p.lateral_margin)
                    return np.maximum(own, self.rest_support(a, yaw_rate))
                C.FieldGeometry.support = support
            if name == "standing":
                h = F.footprint_support(F.normal_angles) + P.lateral_margin
                target = (h[None, :] * F._inverse_cos(GRID)).min(axis=1)
            else:
                target = F.select(GRID, 0.0, 0.0, True, yaw_rate=0.0)
            results[name] = {s: worst(F, P, target, v, w, r) for s, v, w, r in STATES}
            print(f"{name:10s} " + " | ".join(f"{s}: {x * 1e3:+.1f} mm"
                                              for s, x in results[name].items()))
    finally:
        C.yaw_interval, C.FieldGeometry.support = original_interval, original_support
    print(f"e(0) = {e0:.4f} rad, rest reach = {rr * 1e3:.1f} mm")
    if args.no_record:
        return 0
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "analysis", f"NESTING_{stamp}", args.seed, cfg)
    (run_dir / "summary.json").write_text(json.dumps({"e0": e0, "rest_reach_m": rr,
                                                      "worst_m": results}, indent=2),
                                          encoding="utf-8")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
