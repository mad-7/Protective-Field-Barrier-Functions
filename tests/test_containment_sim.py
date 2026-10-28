"""Containment of every protective stop, in the simulator (A20, ruled 2026-10-18). `pytest -m sim`.

Replaces the invariant of 2026-10-17 ("the field is never sized below true motion"), which the
supervisor ruled tested the wrong quantity. The safety property is CONTAINMENT: everything the
footprint sweeps from the stop trigger to rest lies inside the field in force at the trigger. And
its consequence: no stop ends with anything inside the minimum margin (the at-rest field).

**Declared before running (hard rule 19):**
  * TOLERANCE_M = 1 mm on both checks, applied to every stop. Expected false alarms under the null
    (a correct field): zero. The instrument's errors are sub-millimetre and mostly toward a LARGER
    field: the footprint is sampled on its own boundary (no outward error), the field's support
    grid errs larger, interpolation on the 0.1 deg grid errs smaller by micrometres, and the poses
    are the simulator's ground truth (read in this test only).
  * NULL TEST, in the same session, before the verdict: stops taken at CONSTANT velocity -- the
    reach terms are unused there, so containment holds by their slack. Zero exceedances required;
    the largest excess is reported. If it shows any, stop and report; the tolerance is not raised.
  * POSITIVE CONTROL (ruled 2026-10-19, (c); declared before running): against the field with its
    MARGINS REMOVED, so the reach terms alone are tested, the with-reach field must contain every
    stop (to the tolerance), and the WITHOUT-reach field (a_max, alpha_max -> 0) must be exceeded
    by more than the tolerance at some stop. The verdict counts only if that comparison fires.
    (The first control, the without-reach field WITH margins, did not fire on 2026-10-19: under
    the A20 limits the margins absorb what the reach terms add, so it could not discriminate.)
    The null test applies the margins-removed check too, so its tolerance is null-tested.
The runs drive the robot into walls placed in memory (removed afterwards; the scene is never
saved), accelerating, turning, swerving and spinning at the trigger -- the cases the 2026-10-18
traces failed.
"""
from __future__ import annotations

import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.envs.step_loop import StepLoop
from pfbnav.eval.containment import containment_excess, footprint_boundary, rest_margin
from pfbnav.safety.certified import FieldGeometry
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.robot import body_point_in_ref

pytestmark = pytest.mark.sim

TOLERANCE_M = 1e-3
WALL = (0.02, 3.0, 0.60)            # m: thin, long, crossing the scan plane
SETTLE = 10
MAX_STEPS = 260
REST_STEPS = 60                      # after a trigger, steps within which the body must be still
GRID = np.linspace(-math.pi, math.pi, 3600, endpoint=False)
_NULL: dict = {}

# (name, wall centre in the body frame (x, y), wall yaw in the body frame, command schedule)
# A wall's long side runs along its local y; yaw 0 puts it across the forward direction.
ACCELERATING = [
    ("forward from rest", (1.0, 0.0), 0.0, lambda k: (0.50, 0.0, 0.0)),
    # Moved 2026-10-19 (c): the first placement tripped at step 2, before any speed. A wall across
    # the way 0.72 m ahead trips once the band-2 field reaches it, while the base is still
    # accelerating and turning (asserted below: speed >= 0.2 m/s and |yaw rate| >= 0.3 rad/s).
    ("turning and accelerating", (0.72, 0.0), 0.0, lambda k: (0.35, 0.0, 0.9)),
    ("swerve toward a side wall", (0.0, -0.62), math.radians(90),
     lambda k: (0.25, 0.0, 0.0) if k < 25 else (0.05, -0.50, 0.0)),
    # The wall's face at 0.37 m (centre 0.38): outside the still at-rest field sideways (~0.28 m)
    # and inside the spinning one (outer corner 0.337 + reach 0.013 + margin 0.05 = 0.400 m). Its
    # face was at 0.41 m in the first run (2026-10-19), where nothing could trip it.
    ("spinning beside a wall", (0.0, 0.38), math.radians(90), lambda k: (0.0, 0.0, 1.5)),
]
CONSTANT = [   # the null: the wall far enough that the robot is at constant velocity when it trips
    ("cruise ahead", (3.2, 0.0), 0.0, lambda k: (0.30, 0.0, 0.0)),
    ("cruise sideways", (0.0, 3.0), math.radians(90), lambda k: (0.0, 0.30, 0.0)),
]


@pytest.fixture(scope="module")
def loop(cfg, sim_client):
    lp = StepLoop(sim_client, cfg)
    sim_client.stop()
    lp.install()
    try:
        yield lp
    finally:
        lp.remove()
        sim_client.stop()
        sim_client.start(settle_steps=20)


@pytest.fixture(scope="module")
def geometry(cfg):
    repo = Path(__file__).resolve().parents[1]
    table = FootprintTable.from_csv(repo / cfg["robot"]["footprint_table"])
    return table.extent, footprint_boundary(table.extent, 360)


def _place(sim_client, cfg, centre, yaw) -> int:
    sim = sim_client.sim
    ref = sim_client.object(cfg["sim"]["objects"]["base_ref"])
    ref_z = float(sim.getObjectPosition(ref, sim.handle_world)[2])
    wall = int(sim.createPrimitiveShape(sim.primitiveshape_cuboid, list(WALL), 0))
    offset = float(cfg["robot"]["yaw_offset"])
    x, y, z = body_point_in_ref([centre[0], centre[1], 0.5 * WALL[2] - ref_z], offset)
    sim.setObjectOrientation(wall, [0.0, 0.0, offset + yaw], ref)
    sim.setObjectPosition(wall, [float(x), float(y), float(z)], ref)
    return wall


def _drive(loop, sim_client, cfg, run) -> list[dict]:
    """One run: reset, wall, the command schedule; every stop with the poses from its decision to
    rest, the field it was taken with, and the scan once still."""
    _name, centre, yaw, schedule = run
    loop.reset(settle_steps=SETTLE)
    wall = _place(sim_client, cfg, centre, yaw)
    stops, states, headings = [], [], []
    try:
        prev = None
        for k in range(MAX_STEPS):
            state = loop.step(schedule(k))
            states.append((prev, state))
            headings.append(float(loop.certified._heading))
            prev = state
        for i, (before, st) in enumerate(states):
            if not st.certified["stop_triggered"] or before is None:
                continue
            poses, j = [before.pose], i
            while j < len(states) and j - i <= REST_STEPS:
                poses.append(states[j][1].pose)
                nxt = states[j + 1][1] if j + 1 < len(states) else None
                if st_rest(states[j][1]) and nxt is not None and \
                        np.allclose(nxt.pose, states[j][1].pose, atol=1e-6):
                    break
                j += 1
            rest = states[min(j, len(states) - 1)][1]
            stops.append({"step": st.step, "frame": before.pose, "poses": poses,
                          "cert": dict(st.certified), "heading": headings[i],
                          "settled": j - i <= REST_STEPS and j < len(states), "rest": rest})
    finally:
        sim_client.sim.removeObjects([wall])
    return stops


def st_rest(state) -> bool:
    return bool(state.certified["at_rest"] and state.certified["stop_active"])


def _assess(stops, fields, boundary) -> list[dict]:
    out = []
    for s in stops:
        c = s["cert"]
        reach = fields.select(GRID, c["field_speed"], s["heading"], c["at_rest"],
                              yaw_rate=c["field_yaw_rate"])
        excess = containment_excess(boundary, s["poses"], s["frame"],
                                    lambda b, reach=reach: np.interp(b, GRID, reach,
                                                                     period=2 * math.pi))
        p = s["rest"].scan
        bearings = np.asarray(p.filter_bearings, dtype=float)
        margin = rest_margin(p.filter_ranges, p.filter_valid,
                             fields.select(bearings, 0.0, 0.0, True))
        out.append({"step": s["step"], "excess_m": excess, "rest_margin_m": margin,
                    "settled": s["settled"], "field_speed": c["field_speed"],
                    "field_yaw_rate": c["field_yaw_rate"]})
    return out


def _fields(cfg, extent, no_reach=False, no_margins=False):
    from pfbnav.world.strata import certified_params_from_config
    params = certified_params_from_config(cfg)
    if no_reach:
        params = dataclasses.replace(params, a_max=1e-9, alpha_max=1e-9)
    if no_margins:                       # the validated minimum, not zero: 0 < lateral <= margin
        params = dataclasses.replace(params, margin=1e-9, lateral_margin=1e-9)
    return FieldGeometry(extent, params)


def _excess(stops, fields, boundary) -> list[float]:
    """Containment excess only, for the margins-removed comparisons."""
    return [r["excess_m"] for r in _assess(stops, fields, boundary)]


def test_null_containment_holds_at_constant_velocity(loop, cfg, sim_client, geometry):
    """The null test of TOLERANCE_M: stops at constant velocity. Zero exceedances required."""
    extent, boundary = geometry
    fields, bare = _fields(cfg, extent), _fields(cfg, extent, no_margins=True)
    rows = []
    for run in CONSTANT:
        stops = _drive(loop, sim_client, cfg, run)
        assert stops, f"{run[0]}: no stop -- the null run tests nothing"
        rows += [{"run": run[0], **r, "bare_excess_m": b}
                 for r, b in zip(_assess(stops, fields, boundary),
                                 _excess(stops, bare, boundary), strict=True)]
    exceed = [r for r in rows if r["excess_m"] > TOLERANCE_M or r["rest_margin_m"] < -TOLERANCE_M
              or r["bare_excess_m"] > TOLERANCE_M]
    _NULL.update(ran=True, exceedances=len(exceed))
    print(f"\nNULL: {len(rows)} stops at constant velocity; exceedances {len(exceed)}; largest "
          f"containment excess {max(r['excess_m'] for r in rows) * 1e3:+.2f} mm (margins removed "
          f"{max(r['bare_excess_m'] for r in rows) * 1e3:+.2f} mm); smallest rest margin "
          f"{min(r['rest_margin_m'] for r in rows) * 1e3:+.2f} mm")
    assert not exceed, f"NULL TEST FAILED -- stop and report, do not raise the tolerance: {exceed}"


def test_every_stop_is_contained_and_none_ends_inside_the_minimum_margin(loop, cfg, sim_client,
                                                                          geometry):
    """The verdict, read only after the null test passed in this session."""
    if not _NULL.get("ran"):
        pytest.fail("the null test must run first in this session: run the module")
    if _NULL["exceedances"]:
        pytest.fail("the null test raised false alarms: stop and report; no verdict is read")
    extent, boundary = geometry
    fields = _fields(cfg, extent)
    bare = _fields(cfg, extent, no_margins=True)
    bare_blind = _fields(cfg, extent, no_reach=True, no_margins=True)
    rows, control, silent = [], [], []
    for run in ACCELERATING:
        stops = _drive(loop, sim_client, cfg, run)
        if not stops:
            silent.append(run[0])
            print(f"{run[0]:28s} NO STOP -- the run tests nothing")
            continue
        new = [{"run": run[0], **r, "bare_excess_m": b, "blind_excess_m": c}
               for r, b, c in zip(_assess(stops, fields, boundary),
                                  _excess(stops, bare, boundary),
                                  _excess(stops, bare_blind, boundary), strict=True)]
        rows += new
        control += [r["blind_excess_m"] for r in new]
        for r in new:   # printed run by run, so a later failure loses nothing
            print(f"{r['run']:28s} step {r['step']:3d}: containment {r['excess_m'] * 1e3:+7.2f} "
                  f"mm (margins removed: with reach {r['bare_excess_m'] * 1e3:+6.2f}, without "
                  f"{r['blind_excess_m'] * 1e3:+6.2f}), rest margin "
                  f"{r['rest_margin_m'] * 1e3:+7.2f} mm, settled {r['settled']}, field "
                  f"{r['field_speed']:.3f} m/s {r['field_yaw_rate']:+.3f} rad/s")
    assert not silent, f"runs with no stop test nothing: {silent}"
    turning = [r for r in rows if r["run"] == "turning and accelerating"]
    assert turning[0]["field_speed"] >= 0.2 and abs(turning[0]["field_yaw_rate"]) >= 0.3, (
        f"the turning run did not trip at speed: {turning[0]} -- it tests nothing")
    print(f"POSITIVE CONTROL (margins removed): without reach the largest excess is "
          f"{max(control) * 1e3:+.2f} mm")
    assert max(control) > TOLERANCE_M, (
        "INCONCLUSIVE: without the reach terms the margins-removed field was not exceeded, so the "
        "check is not shown able to see an under-sized field; no verdict is read")
    assert all(r["settled"] for r in rows), "a stop did not come to rest within the window"
    bad = [r for r in rows
           if r["excess_m"] > TOLERANCE_M or r["bare_excess_m"] > TOLERANCE_M
           or r["rest_margin_m"] < -TOLERANCE_M]
    assert not bad, f"{len(bad)} stops break containment or end inside the minimum margin: {bad}"
