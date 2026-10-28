"""Known-answer tests for the mecanum wheel-speed budget (ruling 2, WP0 results report)."""
import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.sim.robot import (
    ActuationLimits,
    ChassisGeometry,
    WheelBudget,
    clamp_velocity,
    limit_acceleration,
    project_to_wheel_budget,
    wheel_budget_use,
    wheels_from_body,
)

CFG = load_config("configs/base.yaml")
GEOM = ChassisGeometry.from_config(CFG)
LEVER = GEOM.half_wheelbase + GEOM.half_track


def test_budget_use_is_exactly_the_fastest_wheels_rim_speed():
    """The whole point of the formula: it is the wheel-speed limit, not an approximation."""
    rng = np.random.default_rng(2)
    for _ in range(500):
        cmd = rng.uniform([-0.6, -0.6, -1.5], [0.6, 0.6, 1.5])
        rim = np.max(np.abs(wheels_from_body(cmd, GEOM))) * GEOM.wheel_radius
        assert wheel_budget_use(cmd, LEVER) == pytest.approx(rim, rel=1e-12, abs=1e-12)


def test_g1_diagonal_needed_more_than_the_rim_could_give():
    # G1-ODE's diagonal (0.50, 0.30) asked 0.80 m/s at the rim -- 16 rad/s -- and got 0.617.
    assert wheel_budget_use([0.5, 0.3, 0.0], LEVER) == pytest.approx(0.80)


def test_projection_preserves_direction_and_turn_ratio():
    cmd = np.array([0.5, 0.5, 0.4])
    result = project_to_wheel_budget(cmd, 0.4936, LEVER)
    assert result.bound
    assert wheel_budget_use(result.applied, LEVER) == pytest.approx(0.4936)
    # Parallel in all three components: one scale factor.
    assert result.applied == pytest.approx(cmd * result.scale)


def test_projection_leaves_an_admissible_command_untouched():
    cmd = np.array([0.1, -0.1, 0.2])
    result = project_to_wheel_budget(cmd, 0.5, LEVER)
    assert not result.bound and result.scale == 1.0
    assert result.applied == pytest.approx(cmd)


def test_projection_never_exceeds_the_limit():
    rng = np.random.default_rng(5)
    for _ in range(2000):
        cmd = rng.uniform(-2.0, 2.0, size=3)
        out = project_to_wheel_budget(cmd, 0.586, LEVER).applied
        assert wheel_budget_use(out, LEVER) <= 0.586 + 1e-12


def test_box_budget_and_limiter_compose_without_leaving_either():
    """Box and budget are convex; the limiter moves between two admissible points."""
    limits = ActuationLimits.from_config(CFG)
    budget = WheelBudget.from_config(CFG)
    rng = np.random.default_rng(9)
    applied = np.zeros(3)
    for _ in range(2000):
        cmd = clamp_velocity(rng.uniform(-3.0, 3.0, size=3), limits)
        cmd = project_to_wheel_budget(cmd, budget.filter_limit, budget.lever_arm).applied
        applied = limit_acceleration(cmd, applied, 0.05, limits)
        assert wheel_budget_use(applied, budget.lever_arm) <= budget.filter_limit + 1e-12
        assert abs(applied[0]) <= limits.v_max + 1e-12
        assert abs(applied[1]) <= limits.vy_max + 1e-12


def test_configured_budget_leaves_the_filter_authority():
    budget = WheelBudget.from_config(CFG)
    assert budget.nominal_fraction == pytest.approx(0.80)
    assert budget.filter_fraction == pytest.approx(0.95)
    assert budget.nominal_limit < budget.filter_limit <= budget.v_rim
    assert budget.lever_arm == pytest.approx(0.228 + 0.1585)


def test_a_budget_without_filter_authority_is_refused():
    with pytest.raises(ValueError, match="authority"):
        WheelBudget(v_rim=0.617, nominal_fraction=0.95, filter_fraction=0.95, lever_arm=0.4)


# ----------------------------------------------------------------------
# count_at_torque_cap (WP0 closure: per-episode saturation counter)
# ----------------------------------------------------------------------
def test_torque_cap_count_matches_the_diagnostic():
    # WHEEL_TORQUE_20260918T164140Z: fr and rl at the 2.5 N m cap, fl and rr well below.
    from pfbnav.sim.robot import count_at_torque_cap

    assert count_at_torque_cap([-0.774, 2.500, -2.500, 0.886], [2.5] * 4) == 2


def test_torque_cap_count_tolerance_and_validation():
    from pfbnav.sim.robot import count_at_torque_cap

    assert count_at_torque_cap([2.4976, 2.49], [2.5, 2.5]) == 1     # within 0.1 %
    assert count_at_torque_cap([0.0, 0.0], [2.5, 2.5]) == 0
    with pytest.raises(ValueError, match="match"):
        count_at_torque_cap([1.0], [2.5, 2.5])
    with pytest.raises(ValueError, match="positive"):
        count_at_torque_cap([1.0], [0.0])


def test_the_box_corner_projects_into_the_budget_with_direction_preserved():
    """The box's corner (v_max, vy_max, 0) is outside the budget.

    Since 2026-10-19 the box is square, v_max = vy_max = 0.50 (G1b under the A20 limits), so a
    command may sit at both ceilings at once: 0.50 + 0.50 = 1.00 m/s at the rim against a
    nominal share of 0.80 x 0.617 = 0.4936. The corner is therefore a command the projection
    must shorten, and it must shorten it along its own direction -- 45 deg off the nose --
    rather than clip an axis and turn the robot somewhere else. (PRE-A20: the non-square box
    0.40 / 0.50, a 0.90 m/s corner 51.3 deg off the nose; supervisor, 2026-09-20.)
    """
    limits = ActuationLimits.from_config(CFG)
    budget = WheelBudget.from_config(CFG)
    corner = np.array([limits.v_max, limits.vy_max, 0.0])

    assert wheel_budget_use(corner, LEVER) == pytest.approx(1.00)
    assert wheel_budget_use(corner, LEVER) > budget.nominal_limit

    result = project_to_wheel_budget(corner, budget.nominal_limit, LEVER)
    assert result.bound
    # Inside the budget, exactly on it, and parallel to the command in all three axes.
    assert wheel_budget_use(result.applied, LEVER) == pytest.approx(budget.nominal_limit)
    assert result.applied == pytest.approx(corner * result.scale)
    assert result.scale == pytest.approx(budget.nominal_limit / 1.00)
    # Direction preserved: the heading of the translation is unchanged.
    assert np.arctan2(result.applied[1], result.applied[0]) == pytest.approx(
        np.arctan2(corner[1], corner[0]))
    # And each axis is still inside its own ceiling, which shortening cannot break.
    assert abs(result.applied[0]) <= limits.v_max + 1e-12
    assert abs(result.applied[1]) <= limits.vy_max + 1e-12


def test_the_box_corner_with_rotation_also_keeps_its_turn_ratio():
    """The same corner while turning: translation and omega shrink by one factor."""
    limits = ActuationLimits.from_config(CFG)
    budget = WheelBudget.from_config(CFG)
    corner = np.array([limits.v_max, limits.vy_max, 0.5 * limits.w_max])
    result = project_to_wheel_budget(corner, budget.nominal_limit, LEVER)
    assert result.bound
    assert result.applied == pytest.approx(corner * result.scale)
    # The ratio of turning to translating is what a clip would have destroyed.
    assert (result.applied[2] / np.hypot(*result.applied[:2])
            == pytest.approx(corner[2] / np.hypot(*corner[:2])))
