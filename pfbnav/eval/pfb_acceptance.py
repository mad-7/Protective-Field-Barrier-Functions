"""The PFB acceptance step (WP2, ruled 2026-10-23), before alpha tuning.

An ideal tracker plus the PFB drives each tune route in the emulator, with the certified layer
beneath. **Every certified stop on static geometry is attributed** to exactly one of the ruled
causes:

  * `unobserved` -- an unobserved bin in the scan the stop was decided on;
  * `infeasible` -- the PFB used its maximum-braking fallback on that step or the one before;
  * `creep` -- the stop triggered with the robot at rest (the creep regime, A22);
  * `prediction` -- the PFB had returned a command its own rows said was safe (`nominal`,
    `filtered`, or `brake`: the braking command, checked against the true rows), and the next state
    differed from what those rows assumed. The difference is named, at least one of:
      `regime`  (the certified field in force was a higher regime than the PFB's),
      `heading` (the field's heading differed by more than `heading_tol`),
      `yaw`     (the |yaw rate| the field was swept for differed by more than `yaw_tol`),
      `motion`  (the body's displacement over the step differed from the command's by more than
                 `motion_tol`: slip, the limiter, or the prediction horizon).

A stop that fits none is **unattributed**. The ruling: unattributed stops must be zero; otherwise stop
and report. Also ruled (Q3): the maximum-braking fallback never produces contact from any fallback
state, checked by rolling pure braking forward from each one (`braking_contact`).
"""
from __future__ import annotations

import math
from collections import Counter
from collections.abc import Sequence
from typing import Any

import numpy as np

REST = -1


def _angle(a: float, b: float) -> float:
    return abs(math.remainder(a - b, 2.0 * math.pi))


def attribute_stops(records: Sequence[dict[str, Any]], dt: float, *, heading_tol: float = 0.02,
                    yaw_tol: float = 0.02, motion_tol: float = 1e-3) -> list[dict[str, Any]]:
    """One entry per certified stop trigger, with its category and (for `prediction`) subtypes.

    Record k holds the decision at its `decision_pose` on its `scan`; the PFB command that led to
    that state was decided in record k - 1."""
    out = []
    for k, rec in enumerate(records):
        c = rec["cert"]
        if not c.get("stop_triggered"):
            continue
        entry: dict[str, Any] = {"step": rec["step"], "s": rec.get("s"), "category": None,
                                 "subtypes": []}
        prev = records[k - 1] if k > 0 else None
        before = records[k - 2] if k > 1 else None
        scan_valid = np.asarray(rec["scan"].valid, dtype=bool)
        pfb_now, pfb_prev = rec.get("pfb"), (prev or {}).get("pfb")
        pfb_before = (before or {}).get("pfb")
        if not scan_valid.all():
            entry["category"] = "unobserved"
        elif any(x is not None and x.get("infeasible") for x in (pfb_now, pfb_prev, pfb_before)):
            entry["category"] = "infeasible"
        elif c.get("triggered_at_rest"):
            entry["category"] = "creep"
        elif pfb_prev is not None and pfb_prev.get("mode") in ("nominal", "filtered", "brake"):
            subtypes = []
            actual = REST if c.get("at_rest") else int(c["field_id"])
            if pfb_prev.get("regime") is not None and actual > int(pfb_prev["regime"]):
                subtypes.append("regime")
            if pfb_prev.get("heading") is not None and not c.get("at_rest") and \
                    _angle(float(rec["field_heading"]), float(pfb_prev["heading"])) > heading_tol:
                subtypes.append("heading")
            if pfb_prev.get("yaw_field") is not None and \
                    abs(abs(float(c["field_yaw_rate"])) - float(pfb_prev["yaw_field"])) > yaw_tol:
                subtypes.append("yaw")
            p0 = np.asarray(prev["decision_pose"], dtype=float)
            p1 = np.asarray(rec["decision_pose"], dtype=float)
            d = p1[:2] - p0[:2]
            cth, sth = math.cos(p0[2]), math.sin(p0[2])
            body = np.array([cth * d[0] + sth * d[1], -sth * d[0] + cth * d[1]])
            commanded = np.asarray(pfb_prev["u"], dtype=float)[:2] * dt
            if float(np.linalg.norm(body - commanded)) > motion_tol:
                subtypes.append("motion")
            if subtypes:
                entry["category"] = "prediction"
                entry["subtypes"] = subtypes
        if entry["category"] is None:
            entry["category"] = "unattributed"
        out.append(entry)
    return out


def footprint_polygon(extent, n: int = 720):
    """The footprint as a polygon in the body frame (shapely)."""
    from shapely.geometry import Polygon

    psi = np.linspace(-math.pi, math.pi, n, endpoint=False)
    e = np.asarray(extent(psi), dtype=float)
    return Polygon(np.column_stack([e * np.cos(psi), e * np.sin(psi)]))


def in_contact(pose, footprint, obstacles) -> bool:
    """Whether the footprint at `pose` (x, y, yaw) touches any obstacle polygon."""
    from shapely import affinity

    placed = affinity.translate(affinity.rotate(footprint, float(pose[2]), origin=(0, 0),
                                                use_radians=True), float(pose[0]), float(pose[1]))
    return any(placed.intersects(o) for o in obstacles)


def braking_contact(pose, applied, pfb, footprint, obstacles, *, dt: float, slip: float,
                    max_steps: int = 400) -> bool:
    """Q3: pure maximum braking (`pfb.fallback`) rolled forward from a fallback state, through the
    emulator's kinematics, until rest. True if the footprint touches anything on the way."""
    pose = np.asarray(pose, dtype=float).copy()
    u = np.asarray(applied, dtype=float).copy()
    for _ in range(max_steps):
        u = pfb.fallback(u)
        realised = np.array([u[0] * slip, u[1] * slip, u[2]])
        mid = pose[2] + 0.5 * realised[2] * dt
        c, s = math.cos(mid), math.sin(mid)
        pose = pose + dt * np.array([c * realised[0] - s * realised[1],
                                     s * realised[0] + c * realised[1], realised[2]])
        if in_contact(pose, footprint, obstacles):
            return True
        if not np.any(u):
            return False
    return False


BEND_ARC_M = 0.8          # m of route ahead over which a bend is judged (planner.lookahead)
BEND_ANGLE_RAD = 0.5      # rad of route heading change within that arc that makes a bend


def route_turn_ahead(points: np.ndarray, s: float, arc: float = BEND_ARC_M) -> float:
    """|heading change| of a polyline between arc length s and s + arc, rad (the segments' headings
    at the two points)."""
    p = np.asarray(points, dtype=float)
    seg = np.diff(p, axis=0)
    lengths = np.hypot(seg[:, 0], seg[:, 1])
    keep = lengths > 1e-12
    seg, lengths = seg[keep], lengths[keep]
    ends = np.cumsum(lengths)
    headings = np.arctan2(seg[:, 1], seg[:, 0])

    def heading_at(x: float) -> float:
        i = int(np.searchsorted(ends, min(max(x, 0.0), ends[-1]), side="left"))
        return float(headings[min(i, len(headings) - 1)])

    return abs(math.remainder(heading_at(s + arc) - heading_at(s), 2.0 * math.pi))


def yaw_profile(records: Sequence[dict[str, Any]], route: np.ndarray) -> list[dict[str, Any]]:
    """Per step: the applied |yaw rate|, the certified field in force ("rest" or the band index) and
    whether the route bends ahead (BEND_ANGLE_RAD within BEND_ARC_M). For the realism audit's
    rotation row (ruled 2026-10-26: report the yaw rates the PFB plus ideal tracker used at bends,
    and in which bands)."""
    out = []
    for r in records:
        c = r["cert"]
        out.append({"yaw": abs(float(r["applied"][2])),
                    "field": "rest" if c.get("at_rest") else int(c["field_id"]),
                    "bend": route_turn_ahead(route, float(r["s"])) >= BEND_ANGLE_RAD})
    return out


def yaw_summary(profiles: Sequence[Sequence[dict[str, Any]]]) -> dict[str, Any]:
    """Pooled over routes: per field ("rest", 0, 1, 2) and bend / straight, the step count and the
    |yaw rate| median, 90th percentile and maximum, rad/s."""
    pooled: dict[str, list[float]] = {}
    for prof in profiles:
        for row in prof:
            key = f"{row['field']}|{'bend' if row['bend'] else 'straight'}"
            pooled.setdefault(key, []).append(row["yaw"])
    return {k: {"steps": len(v), "median": float(np.median(v)),
                "p90": float(np.percentile(v, 90)), "max": float(np.max(v))}
            for k, v in sorted(pooled.items())}


def summarise(routes: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Counts over every route, and the verdict (ruled: unattributed stops must be zero; the
    fallback never produces contact; no contact on the way)."""
    cats: Counter = Counter()
    subs: Counter = Counter()
    for r in routes:
        for st in r["stops"]:
            cats[st["category"]] += 1
            subs.update(st["subtypes"])
    unattributed = cats.get("unattributed", 0)
    rollout = sum(int(r["fallback_rollout_contacts"]) for r in routes)
    contacts = sum(int(r["contact_steps"]) for r in routes)
    ok = unattributed == 0 and rollout == 0 and contacts == 0
    return {
        "routes": len(routes), "reached_goal": sum(1 for r in routes if r["reached_goal"]),
        "stalled": sum(1 for r in routes if r.get("stalled")),
        "stops": sum(cats.values()), "by_category": dict(cats), "prediction_subtypes": dict(subs),
        "fallback_steps": sum(int(r["fallback_steps"]) for r in routes),
        "fallback_rollout_contacts": rollout, "contact_steps": contacts,
        "verdict": "PASS" if ok else "STOP AND REPORT",
    }
