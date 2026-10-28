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
