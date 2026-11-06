"""Known answers for holonomic Pure Pursuit (B1) and the zero-action control (hard rule 6).

Both take the typed classical input (ruled 2026-10-16). The arithmetic is the one B1 used on the
observation's lookahead and goal channels before then: bearing of the lookahead point, linear
slow-down within the goal radius."""
import math
from pathlib import Path

import numpy as np
import pytest
from synthetic_inputs import make_input

from pfbnav.config import load_config
from pfbnav.control.base import ClassicalController
from pfbnav.control.pure_pursuit import PurePursuit, ZeroAction

PP = {"cruise_speed": 0.25, "k_omega": 1.5, "goal_slowdown": 0.5, "v_max": 0.4, "vy_max": 0.5,
      "w_max": 1.0}


def inp(beta, goal_m=10.0):
    return make_input(lookahead=(0.8 * math.cos(beta), 0.8 * math.sin(beta)),
                      goal=(goal_m, 0.0))


def test_straight_ahead_it_drives_forward_at_cruise_and_does_not_turn():
    pp = PurePursuit(**PP)
    assert np.allclose(pp.command(inp(0.0)), [0.25, 0.0, 0.0])
    assert np.allclose(pp.act(inp(0.0)), [0.25 / 0.4, 0.0, 0.0])


def test_a_point_to_the_left_is_reached_sideways_while_turning_toward_it():
    """beta = 90 deg: all translation lateral (+y), yaw rate 1.5 x pi/2 = 2.36, clipped to 1.0."""
    pp = PurePursuit(**PP)
    c = pp.command(inp(math.pi / 2))
    assert c[0] == pytest.approx(0.0, abs=1e-12) and c[1] == pytest.approx(0.25)
    assert c[2] == pytest.approx(1.0)
    assert pp.command(inp(-0.4))[2] == pytest.approx(-0.6)          # unclipped, right turn


def test_it_slows_linearly_inside_the_goal_radius():
    pp = PurePursuit(**PP)
    assert pp.command(inp(0.0, goal_m=0.25))[0] == pytest.approx(0.125)
    assert pp.command(inp(0.0, goal_m=0.0))[0] == pytest.approx(0.0)
    assert pp.command(inp(0.0, goal_m=0.5))[0] == pytest.approx(0.25)


def test_it_refuses_anything_but_the_typed_input_and_impossible_parameters():
    pp = PurePursuit(**PP)
    with pytest.raises(TypeError, match="ClassicalInput"):
        pp.command(np.zeros(82))
    with pytest.raises(ValueError):
        PurePursuit(**{**PP, "cruise_speed": 0.45})                  # above v_max 0.4
    with pytest.raises(ValueError):
        PurePursuit(**{**PP, "k_omega": 0.0})


def test_both_are_classical_controllers_and_the_zero_control_commands_nothing():
    cfg = load_config(Path(__file__).resolve().parents[1] / "configs" / "base.yaml")
    pp, zero = PurePursuit.from_config(cfg), ZeroAction.from_config(cfg)
    assert isinstance(pp, ClassicalController) and isinstance(zero, ClassicalController)
    assert pp.input_kind == zero.input_kind == "classical"
    assert np.array_equal(zero.act(inp(1.0)), np.zeros(3))
    assert np.array_equal(zero.command(inp(1.0)), np.zeros(3))
    assert np.all(np.abs(pp.act(inp(2.0))) <= 1.0)


# --- B1's own lookahead (ruled 2026-11-04, A33) ----------------------------------------------------
def _env_inputs(n_scenarios=6, poses_per=40):
    """Classical inputs built EXACTLY as `NavEnv._controller_input` builds them, offline: tune
    scenarios planned as the environment plans them, a PathTracker at planner.lookahead, poses along
    each route and off it (cross-track up to 0.15 m, heading up to +-1 rad), the route view from
    `remaining_route` and `world_to_body`."""
    from pfbnav.envs.controller_input import ClassicalInput, RouteView, ScanStream
    from pfbnav.envs.route_view import remaining_route, world_to_body
    from pfbnav.planning.astar import OK, plan_from_config
    from pfbnav.planning.lookahead import PathTracker, point_at
    from pfbnav.world.baking import bake, planner_radii
    from pfbnav.world.scenarios import load_set, world_of

    cfg = load_config(Path(__file__).resolve().parents[1] / "configs" / "base.yaml")
    from pfbnav.safety.footprint_table import FootprintTable
    root = Path(__file__).resolve().parents[1]
    circ = float(FootprintTable.from_csv(root / cfg["robot"]["footprint_table"]).circumscribed)
    rng = np.random.default_rng(0)
    scan = ScanStream.of(np.full(120, 5.0), np.linspace(-math.pi, math.pi, 120, endpoint=False),
                         np.ones(120, bool))
    out = []
    for name in ("tune_narrow", "tune_medium", "tune_wide"):
        for s in load_set(root / "scenarios" / "tune" / f"{name}.json")["scenarios"][:n_scenarios // 3]:
            layout = world_of(s, cfg)
            baked = bake(layout.obstacles, layout.bounds, float(cfg["planner"]["resolution"]),
                         planner_radii(cfg, circumscribed=circ))
            plan = plan_from_config(baked, s["start"][:2], s["goal"], cfg)
            assert plan.status == OK
            tracker = PathTracker(plan.path_world, float(cfg["planner"]["lookahead"]),
                                  window=float(cfg["planner"]["track_window"]))
            for arc in np.linspace(0.0, tracker.length, poses_per):
                xy = point_at(plan.path_world, arc) + rng.uniform(-0.15, 0.15, 2)
                pose = np.array([xy[0], xy[1], rng.uniform(-math.pi, math.pi)])
                st = tracker.update(pose)
                route = world_to_body(remaining_route(tracker.path, st.s), pose)
                ahead = world_to_body(np.vstack([st.lookahead_world, st.goal_world]), pose)
                out.append(ClassicalInput.of(
                    stop_active=False, velocity=np.zeros(3), scan=scan,
                    route=RouteView.of(route, ahead[0], ahead[1], st.remaining), dt=0.05))
    return cfg, out


def test_candidate_0_is_b1_as_before_bit_for_bit_on_environment_built_inputs():
    """Ruling 3 (2026-11-04): candidate 0 (lookahead = planner.lookahead, 0.8 m) reproduces B1
    exactly. On every environment-built input the configured B1 and B1 without a lookahead of its
    own (the controller as it was) give bit-identical commands and actions: at the shared distance
    B1 steers at the environment's own point. The closed loop then repeats bit for bit within one
    scene load (determinism per load)."""
    cfg, inputs = _env_inputs()
    # Candidate 0 as the tuning ran it (B1's block before tuning, A33), in the configured world.
    cand0 = {"cruise_speed": 0.25, "k_omega": 1.5, "goal_slowdown": 0.5, "lookahead": 0.8}
    configured = PurePursuit.from_config(
        {**cfg, "baselines": {**cfg["baselines"], "pure_pursuit": cand0}})
    assert configured.lookahead == float(cfg["planner"]["lookahead"]) == 0.8
    assert not configured.own_point
    a = cfg["actuation"]
    before = PurePursuit(cruise_speed=0.25, k_omega=1.5, goal_slowdown=0.5, v_max=float(a["v_max"]),
                         vy_max=float(a["vy_max"]), w_max=float(a["w_max"]))
    assert len(inputs) >= 200
    for x in inputs:
        assert np.array_equal(configured.command(x), before.command(x))
        assert np.array_equal(configured.act(x), before.act(x))


def test_the_own_lookahead_point_is_the_environments_at_the_same_distance():
    """At any other distance B1 walks its own arc along the route view. That computation, forced
    at 0.8 m, lands on the environment's own lookahead point to within 1e-9 m on every input (the
    same arc, interpolated in the body frame instead of the world frame), so the switch at the
    shared distance is not a discontinuity of substance. Shorter and longer distances move the
    point along the route, clamped at the goal."""
    cfg, inputs = _env_inputs()
    p = cfg["baselines"]["pure_pursuit"]
    a = cfg["actuation"]
    kw = {"cruise_speed": float(p["cruise_speed"]), "k_omega": float(p["k_omega"]),
              "goal_slowdown": float(p["goal_slowdown"]), "v_max": float(a["v_max"]),
              "vy_max": float(a["vy_max"]), "w_max": float(a["w_max"])}
    forced = PurePursuit(**kw, lookahead=0.8, env_lookahead=None)
    assert forced.own_point
    worst = max(float(np.hypot(*(forced.lookahead_point(x) - np.asarray(x.route.lookahead))))
                for x in inputs)
    assert worst < 1e-9
    short = PurePursuit(**kw, lookahead=0.3, env_lookahead=0.8)
    far = PurePursuit(**kw, lookahead=50.0, env_lookahead=0.8)
    for x in inputs[:50]:
        assert np.allclose(far.lookahead_point(x), x.route.goal, atol=1e-9)   # clamped at the goal
        d_short = float(np.hypot(*(short.lookahead_point(x) - x.route.points[0])))
        assert d_short <= 0.3 + 1e-9                         # a chord never exceeds its arc
    with pytest.raises(ValueError, match="lookahead"):
        PurePursuit(**kw, lookahead=0.0)
