"""The trace diagnostics of 2026-11-07 (3a, 3b, 3e): known answers on synthetic traces (hard rule 6)."""
import math

import pytest

from pfbnav.eval.trace_analysis import latch_anatomy, stop_causes, wandering

CERT = {"stop_triggered": False, "stop_active": False, "restart_blocked": False, "breached": False,
        "at_rest": False}


def _row(step, x, y, yaw, s, goal, cross=0.0, req=(0.1, 0.0, 0.0), cert=None, pfb=None, near=None):
    return {"step": step, "pose": [x, y, yaw], "s_m": s, "goal_distance_m": goal,
            "cross_track_m": cross, "requested": list(req), "certified": {**CERT, **(cert or {})},
            "pfb": pfb, "nearest_surface": near}


def test_a_straight_run_drives_its_plan_once_and_evenly():
    """3 m straight route, robot drives it exactly in 30 steps of 0.1 m: driven/plan 1.0, 1 m in
    each third, no lateral offset, no overshoot, no flips."""
    rows = [_row(i, 0.1 * i, 0.0, 0.0, 0.1 * i, 3.0 - 0.1 * i) for i in range(31)]
    w = wandering(rows, plan_length_m=3.0, dt=0.05)
    assert w["driven_m"] == pytest.approx(3.0) and w["driven_over_plan"] == pytest.approx(1.0)
    assert w["driven_by_route_third_m"] == pytest.approx([1.0, 1.0, 1.0], abs=0.11)
    assert w["cross_track_max_m"] == 0.0 and not w["goal_overshoot"]
    assert w["lateral_flips_per_s"] == 0.0 and not w["circling_near_goal"]


def test_circling_at_the_goal_is_seen_where_it_happens():
    """After reaching s = 3 m 0.5 m from the goal, the robot circles it twice (radius 0.5 m, 40
    steps, yaw following the circle): all the extra driving falls in the last third, the heading
    turns 4 pi near the goal plus the quarter turn onto the circle (circling), and the goal distance
    stays flat (no overshoot)."""
    rows = [_row(i, 0.1 * i, 0.0, 0.0, 0.1 * i, 3.5 - 0.1 * i) for i in range(31)]
    for k in range(1, 41):
        a = 2 * math.pi * 2 * k / 40
        rows.append(_row(30 + k, 3.5 - 0.5 * math.cos(a), 0.5 * math.sin(a),
                         math.pi / 2 + a, 3.0, 0.5, cross=0.3))
    w = wandering(rows, plan_length_m=3.0, dt=0.05)
    extra = 40 * 2 * 0.5 * math.sin(math.pi / 20)            # 40 chords of the circle
    assert w["driven_by_route_third_m"][2] == pytest.approx(1.0 + extra, rel=0.02)
    assert w["driven_by_route_third_m"][:2] == pytest.approx([1.0, 1.0], abs=0.11)
    assert w["circling_near_goal"] and w["turn_near_goal_rad"] == pytest.approx(4.5 * math.pi, rel=0.01)
    assert not w["goal_overshoot"] and w["cross_track_max_m"] == pytest.approx(0.3)


def test_overshoot_and_oscillation():
    """Approach to 0.1 m of the goal, then back out to 0.6 m: overshoot. The lateral command flips
    sign every step for 20 steps (1 s): 19 flips per second over the 20-step trace."""
    goal = [1.0 - 0.1 * i for i in range(10)] + [0.2 + 0.1 * i for i in range(5)] + [0.6] * 5
    rows = [_row(i, 0.05 * i, 0.0, 0.0, 0.05 * i, g, req=(0.1, 0.05 * (-1) ** i, 0.0))
            for i, g in enumerate(goal)]
    w = wandering(rows, plan_length_m=1.0, dt=0.05)
    assert w["closest_goal_m"] == pytest.approx(0.1) and w["goal_overshoot"]
    assert w["lateral_flips_per_s"] == pytest.approx(19 / (20 * 0.05))
    with pytest.raises(ValueError, match="two steps"):
        wandering(rows[:1], plan_length_m=1.0, dt=0.05)


def test_stop_causes_in_their_declared_order():
    fb = {"mode": "fallback", "infeasible": True}
    ok = {"mode": "filtered", "infeasible": False}
    trig = {"stop_triggered": True, "stop_active": True}
    rows = [
        _row(1, 0, 0, 0, 0.0, 5, pfb=fb),                                         # fallback 2 before
        _row(2, 0, 0, 0, 0.1, 5, pfb=ok),
        _row(3, 0, 0, 0, 0.2, 5, pfb=ok, cert=trig, near={"mapped_m": 0.1, "unmapped_m": 0.05}),
        _row(10, 0, 0, 0, 1.0, 5, pfb=ok),
        _row(11, 0, 0, 0, 1.1, 5, pfb=ok),
        _row(12, 0, 0, 0, 1.2, 5, pfb=ok),
        _row(13, 0, 0, 0, 1.3, 5, pfb=ok, cert=trig, near={"mapped_m": 0.2, "unmapped_m": 0.04}),
        _row(20, 0, 0, 0, 2.0, 5, pfb=ok),
        _row(21, 0, 0, 0, 2.1, 5, pfb=ok),
        _row(22, 0, 0, 0, 2.2, 5, pfb=ok),
        _row(23, 0, 0, 0, 2.3, 5, pfb=ok, cert=trig, near={"mapped_m": 0.03, "unmapped_m": None}),
        _row(30, 0, 0, 0, 3.0, 5),
        _row(31, 0, 0, 0, 3.1, 5),
        _row(32, 0, 0, 0, 3.2, 5),
        _row(33, 0, 0, 0, 3.3, 5, cert=trig, near={"mapped_m": 0.03, "unmapped_m": 0.5}),
    ]
    got = [c["cause"] for c in stop_causes(rows)]
    assert got == ["pfb_fallback", "unmapped_in_field", "filter_active_no_fallback", "no_filter"]


def test_latch_anatomy():
    """Trigger at step 2, latched 2-5, released at 6, re-triggered at 8 and latched to the end."""
    active = [0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1]
    rows = [_row(i, 0, 0, 0, 0.0, 5, cert={"stop_active": bool(a),
                                            "stop_triggered": a and (i == 0 or not active[i - 1])},
                 ) | {"recovery": {"mode": "recover" if 5 <= i <= 6 else "track"}}
            for i, a in enumerate(active)]
    a = latch_anatomy(rows)
    assert a["triggers"] == 2 and a["releases"] == 1 and a["longest_latch_steps"] == 5
    assert a["latched_share"] == pytest.approx(9 / 12)
    assert a["recovery_mode_steps"] == {"track": 10, "recover": 2} and a["final"]["stop_active"]


def test_filter_intervention_and_latched_shares():
    """10 steps: modes 6 nominal, 2 filtered, 1 brake, 1 fallback -> intervention 0.4, fallback 0.1;
    2 steps latched -> 0.2. Without any PFB decision both shares are None."""
    modes = ["nominal"] * 6 + ["filtered", "filtered", "brake", "fallback"]
    rows = [_row(i, 0.1 * i, 0, 0, 0.1 * i, 1.0 - 0.1 * i, pfb={"mode": m},
                 cert={"stop_active": i in (3, 4)}) for i, m in enumerate(modes)]
    w = wandering(rows, plan_length_m=1.0, dt=0.05)
    assert w["pfb_intervention_share"] == pytest.approx(0.4)
    assert w["pfb_fallback_share"] == pytest.approx(0.1) and w["latched_share"] == pytest.approx(0.2)
    bare = wandering([_row(i, 0.1 * i, 0, 0, 0.1 * i, 1.0) for i in range(5)], plan_length_m=1.0,
                     dt=0.05)
    assert bare["pfb_intervention_share"] is None
