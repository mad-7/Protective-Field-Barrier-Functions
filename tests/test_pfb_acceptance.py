"""The PFB acceptance step's machinery (WP2 step 3, ruled 2026-10-23): known answers for the stop
attribution and the fallback rollout, the ideal tracker, and the emulator's PFB slot."""
import math
from pathlib import Path

import numpy as np
import pytest
from shapely.geometry import Polygon

from pfbnav.config import load_config
from pfbnav.control.ideal_tracker import IdealTracker
from pfbnav.envs.controller_input import ClassicalInput, RouteView, ScanStream
from pfbnav.eval.pfb_acceptance import (
    attribute_stops,
    braking_contact,
    footprint_polygon,
    in_contact,
    summarise,
)
from pfbnav.safety import pfb_field as pf
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.safety.pfb import PFB, PFBConfig

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
TABLE = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
FIELD = pf.PFBField(TABLE.extent, pf.field_params_from_config(CFG))
PFB_ = PFB(FIELD, PFBConfig.from_config(CFG, FIELD))
DT = float(CFG["sim"]["dt"])


class _Scan:
    def __init__(self, valid=True):
        self.valid = np.ones(8, bool) if valid else np.array([True] * 7 + [False])


def _rec(step, *, trig=False, at_rest=False, field_id=2, heading=0.0, yaw=0.0, pfb=None,
         pose=(0.0, 0.0, 0.0), valid=True):
    return {"step": step, "s": 0.1 * step, "scan": _Scan(valid), "field_heading": heading,
            "decision_pose": list(pose), "pfb": pfb,
            "cert": {"stop_triggered": trig, "triggered_at_rest": at_rest, "at_rest": at_rest,
                     "field_id": field_id, "field_yaw_rate": yaw}}


def _pfb(mode="filtered", regime=2, heading=0.0, yaw=0.0, u=(0.3, 0.0, 0.0), infeasible=False):
    return {"mode": mode, "regime": regime, "heading": heading, "yaw_field": yaw, "u": list(u),
            "infeasible": infeasible}


# --------------------------------------------------------------------------- attribution
def test_each_ruled_category_known_answers():
    move = (0.3 * DT, 0.0, 0.0)                         # exactly the commanded displacement
    recs = [_rec(1, pfb=_pfb()), _rec(2, trig=True, valid=False, pfb=_pfb(), pose=move)]
    assert attribute_stops(recs, DT)[0]["category"] == "unobserved"
    recs = [_rec(1, pfb=_pfb(infeasible=True, mode="fallback")),
            _rec(2, trig=True, pfb=_pfb(), pose=move)]
    assert attribute_stops(recs, DT)[0]["category"] == "infeasible"
    recs = [_rec(1, pfb=_pfb(regime=-1)), _rec(2, trig=True, at_rest=True, pfb=_pfb(), pose=move)]
    assert attribute_stops(recs, DT)[0]["category"] == "creep"


@pytest.mark.parametrize(("kw", "subtype"), [
    ({"field_id": 2, "prev_regime": 1}, "regime"),
    ({"heading": 0.5}, "heading"),
    ({"yaw": 0.3}, "yaw"),
    ({"pose": (0.3 * 0.05 * 0.9, 0.0, 0.0)}, "motion"),
])
def test_prediction_errors_are_named(kw, subtype):
    prev = _pfb(regime=kw.pop("prev_regime", 2))
    pose = kw.pop("pose", (0.3 * DT, 0.0, 0.0))
    recs = [_rec(1, pfb=prev), _rec(2, trig=True, pfb=_pfb(), pose=pose, **kw)]
    st = attribute_stops(recs, DT)[0]
    assert st["category"] == "prediction" and subtype in st["subtypes"]


def test_a_stop_the_prediction_cannot_explain_is_unattributed():
    """Everything as the PFB assumed -- same regime, heading, yaw rate and displacement -- and still
    a stop: unattributed (ruled: must be zero; otherwise stop and report)."""
    recs = [_rec(1, pfb=_pfb()), _rec(2, trig=True, pfb=_pfb(), pose=(0.3 * DT, 0.0, 0.0))]
    assert attribute_stops(recs, DT)[0]["category"] == "unattributed"
    summary = summarise([{"stops": attribute_stops(recs, DT), "reached_goal": False,
                          "fallback_steps": 0, "fallback_rollout_contacts": 0,
                          "contact_steps": 0}])
    assert summary["verdict"] == "STOP AND REPORT"


# --------------------------------------------------------------------------- contact
def test_contact_known_answers():
    fp = footprint_polygon(TABLE.extent)
    wall = [Polygon([(0.40, -1.0), (0.42, -1.0), (0.42, 1.0), (0.40, 1.0)])]
    assert not in_contact((0.0, 0.0, 0.0), fp, wall)
    assert in_contact((0.15, 0.0, 0.0), fp, wall)       # the front (0.286 m) past 0.40
    assert not in_contact((0.0, 0.0, math.pi / 2), fp, wall)   # turned: the side (0.21) short of it


def test_the_fallback_braking_rollout():
    """From 0.3 m/s straight at a wall: braking at a_decel stops in v^2/(2 a) = 45 mm plus one step
    at speed; a wall 0.40 m ahead of the front is never reached, one 20 mm ahead is."""
    fp = footprint_polygon(TABLE.extent)
    far = [Polygon([(0.70, -1.0), (0.72, -1.0), (0.72, 1.0), (0.70, 1.0)])]
    near = [Polygon([(0.305, -1.0), (0.33, -1.0), (0.33, 1.0), (0.305, 1.0)])]
    u = np.array([0.3, 0.0, 0.0])
    assert not braking_contact((0.0, 0.0, 0.0), u, PFB_, fp, far, dt=DT, slip=1.0)
    assert braking_contact((0.0, 0.0, 0.0), u, PFB_, fp, near, dt=DT, slip=1.0)


# --------------------------------------------------------------------------- the tracker
def _input(points, goal_dist=5.0):
    pts = np.asarray(points, dtype=float)
    scan = ScanStream.of(np.full(4, 5.0), np.linspace(-3.0, 3.0, 4), np.ones(4, bool))
    return ClassicalInput.of(stop_active=False, velocity=np.zeros(3), scan=scan,
                             route=RouteView.of(pts, pts[-1], np.array([goal_dist, 0.0]), 5.0),
                             dt=DT)


def test_the_ideal_tracker_follows_the_tangent_and_returns_to_the_route():
    t = IdealTracker.from_config(CFG)
    on = t.command(_input([[0.0, 0.0], [1.0, 0.0]]))
    assert on.tolist() == pytest.approx([0.5, 0.0, 0.0])
    off = t.command(_input([[0.0, 0.2], [1.0, 0.2]]))         # route 0.2 m to the left
    assert off[1] > 0.0 and math.hypot(off[0], off[1]) <= 0.5 + 1e-12
    turn = t.command(_input([[0.0, 0.0], [1.0, 0.5]]))         # route heading left
    assert turn[2] > 0.0
    near_goal = t.command(_input([[0.0, 0.0], [1.0, 0.0]], goal_dist=0.25))
    assert near_goal[0] == pytest.approx(0.25)


# --------------------------------------------------------------------------- the emulator slot
def test_the_emulator_runs_the_pfb_in_the_filter_slot():
    """A straight corridor with a wall across its end (the route runs into it, so the tracker never
    stops pushing): the PFB slows and stops the ideal tracker short of the wall, with no contact,
    and any certified stop on the way is attributed (ruled: none unattributed)."""
    from pfbnav.eval.recovery_control import emulate_layout

    route = np.array([[0.0, 0.0], [3.0, 0.0]])
    walls = [np.array([[-1.0, 0.9], [4.0, 0.9], [4.0, 1.0], [-1.0, 1.0]]),
             np.array([[-1.0, -1.0], [4.0, -1.0], [4.0, -0.9], [-1.0, -0.9]]),
             np.array([[2.4, -0.9], [2.5, -0.9], [2.5, 0.9], [2.4, 0.9]])]
    recs = emulate_layout(CFG, walls, route, (0.0, 0.0, 0.0), IdealTracker.from_config(CFG), 1.0,
                          240, pfb=PFB(FIELD, PFBConfig.from_config(CFG, FIELD)), sector_rays=13)
    assert all(r["pfb"] is not None for r in recs)
    assert all(st["category"] != "unattributed" for st in attribute_stops(recs, DT))
    fp = footprint_polygon(TABLE.extent)
    obstacles = [Polygon(w) for w in walls]
    assert not any(in_contact(r["pose"], fp, obstacles) for r in recs)
    assert recs[-1]["pose"][0] > 1.5                 # it drove up the corridor ...
    assert math.hypot(*recs[-1]["applied"][:2]) < 0.05   # ... and came to rest short of the wall


def test_the_declared_stall_cut_off_ends_a_run_without_progress():
    """`stall=(steps, metres)`: a controller that never moves ends after exactly `steps` + 1 records,
    marked stalled; one that drives the corridor is never cut."""
    from pfbnav.eval.recovery_control import emulate_layout

    class _Still:
        def reset(self):
            pass

        def command(self, _inp):
            return np.zeros(3)

    route = np.array([[0.0, 0.0], [3.0, 0.0]])
    far = [np.array([[0.0, 4.0], [3.0, 4.0], [3.0, 4.1], [0.0, 4.1]])]
    recs = emulate_layout(CFG, far, route, (0.0, 0.0, 0.0), _Still(), 1.0, 200, stall=(20, 0.05))
    assert len(recs) == 21 and recs[-1]["stalled"]
    recs = emulate_layout(CFG, far, route, (0.0, 0.0, 0.0), IdealTracker.from_config(CFG), 1.0, 60,
                          stall=(20, 0.05))
    assert len(recs) == 60 and not any(r.get("stalled") for r in recs)


def test_the_simulator_records_pair_each_decision_with_the_packet_and_pose_it_was_taken_on():
    """`scripts/pfb_sim_acceptance.py`: step k's certified and PFB decisions were taken on the packet
    in hand and at the pose BEFORE step k's physics -- step k-1's -- so the record carries step k's
    decisions with step k-1's scan validity, pose and applied command; attributed as the emulator's
    records are."""
    import importlib.util
    from types import SimpleNamespace as NS

    spec = importlib.util.spec_from_file_location("pfb_sim_acceptance",
                                                  REPO / "scripts" / "pfb_sim_acceptance.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    valid_prev = np.array([True] * 7 + [False])
    prev = NS(step=4, pose=np.array([1.0, 2.0, 0.3]), applied=np.array([0.2, 0.0, 0.1]),
              velocity=np.array([0.18, 0.0, 0.09]),
              scan=NS(filter_valid=valid_prev, filter_ranges=np.full(8, 2.0),
                      filter_bearings=np.linspace(-3.0, 3.0, 8)))
    cert = {"stop_triggered": True, "triggered_at_rest": False, "at_rest": False, "field_id": 1,
            "field_yaw_rate": 0.1}
    state = NS(step=5, certified=cert, pfb=_pfb(), decided_on_fault=False, timings_s={"pfb": 0.012},
               over_budget=False, in_contact=False, applied=np.array([0.21, 0.0, 0.12]))
    rec = mod.route_record(state, prev, 0.25, 3.5, (0.1, 0.2, 0.3), (0.4, True))
    assert rec["step"] == 5 and rec["decision_pose"] == [1.0, 2.0, 0.3]
    assert rec["applied_before"] == [0.2, 0.0, 0.1] and rec["pfb_ms"] == pytest.approx(12.0)
    assert np.array_equal(rec["scan"].valid, valid_prev)
    # what re-solving the PFB's decision needs: its scan, estimate and state BEFORE the call
    assert rec["velocity_before"] == [0.18, 0.0, 0.09] and rec["pfb_state_before"] == [0.4, True]
    assert rec["scan_ranges"] == [2.0] * 8 and len(rec["scan_bearings"]) == 8
    stops = attribute_stops([rec], DT)
    assert stops[0]["category"] == "unobserved"


def test_the_traces_keep_the_window_around_every_stop(tmp_path):
    """`write_traces`: one file per stop, holding TRACE_BEFORE records before it to TRACE_AFTER
    after (clipped at the episode's ends), serialisable, with the scan's validity kept."""
    import importlib.util
    import json
    from types import SimpleNamespace as NS

    spec = importlib.util.spec_from_file_location("pfb_sim_acceptance",
                                                  REPO / "scripts" / "pfb_sim_acceptance.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    records = [{"step": k, "s": 0.01 * k, "scan": NS(valid=np.ones(4, bool)), "x": float(k)}
               for k in range(1, 61)]
    names = mod.write_traces(records, [{"step": 40, "category": "unattributed"},
                                       {"step": 3, "category": "creep"}], 77, tmp_path)
    assert names == ["77_40_unattributed.json", "77_3_creep.json"]
    late = json.loads((tmp_path / names[0]).read_text(encoding="utf-8"))["records"]
    assert [r["step"] for r in late] == list(range(40 - mod.TRACE_BEFORE, 40 + mod.TRACE_AFTER + 1))
    early = json.loads((tmp_path / names[1]).read_text(encoding="utf-8"))["records"]
    assert early[0]["step"] == 1 and early[-1]["step"] == 3 + mod.TRACE_AFTER
    assert early[0]["scan_valid"] == [True] * 4 and "scan" not in early[0]


# --------------------------------------------------------------------------- the yaw profile
def test_the_route_turn_ahead_known_answers():
    from pfbnav.eval.pfb_acceptance import route_turn_ahead

    route = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]])          # a right angle at s = 2
    assert route_turn_ahead(route, 0.0) == pytest.approx(0.0)
    assert route_turn_ahead(route, 1.5) == pytest.approx(math.pi / 2)  # the corner within 0.8
    assert route_turn_ahead(route, 2.5) == pytest.approx(0.0)
    assert route_turn_ahead(route, 1.0) == pytest.approx(0.0)        # 0.8 m does not reach it


def test_the_yaw_profile_pools_by_field_and_bend():
    from pfbnav.eval.pfb_acceptance import yaw_profile, yaw_summary

    route = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]])
    recs = [{"s": s, "applied": [0.2, 0.0, w], "cert": {"at_rest": rest, "field_id": band}}
            for s, w, rest, band in ((0.2, 0.1, False, 1), (0.4, -0.3, False, 1),
                                     (1.5, 0.6, False, 0), (1.6, -0.2, True, 0))]
    prof = yaw_profile(recs, route)
    assert [(p["field"], p["bend"]) for p in prof] == [(1, False), (1, False), (0, True),
                                                        ("rest", True)]
    summary = yaw_summary([prof])
    assert summary["1|straight"] == {"steps": 2, "median": pytest.approx(0.2),
                                     "p90": pytest.approx(0.28), "max": pytest.approx(0.3)}
    assert summary["0|bend"]["max"] == pytest.approx(0.6)
    assert summary["rest|bend"]["steps"] == 1
