"""Known-answer tests for placing a layout in the scene (hard rule 6). Pure half.

`tests/test_layout_shapes_sim.py` builds the shapes in a real instance; this pins the
arithmetic that decides where they go -- above all the transform that lets an episode start
without teleporting a dynamic robot.
"""
import math

import numpy as np
import pytest

from pfbnav.sim.layout_shapes import (
    Pose2D,
    rectangle_of,
    start_pose_error,
    transform_placing,
    wrap,
)


def test_wrap_puts_angles_in_the_half_open_turn():
    assert wrap(0.0) == 0.0
    assert wrap(math.pi) == pytest.approx(math.pi)
    assert wrap(-math.pi) == pytest.approx(math.pi)
    assert wrap(3 * math.pi) == pytest.approx(math.pi)
    assert wrap(math.radians(370.0)) == pytest.approx(math.radians(10.0))


def test_the_identity_transform_changes_nothing():
    p = Pose2D()
    assert list(p.apply([[3.0, -1.0]])[0]) == [3.0, -1.0]
    assert p.apply_yaw(0.7) == pytest.approx(0.7)


def test_a_pure_translation_and_a_pure_rotation():
    assert list(Pose2D(x=1.0, y=2.0).apply([[0.0, 0.0]])[0]) == [1.0, 2.0]
    turned = Pose2D(yaw=math.pi / 2).apply([[1.0, 0.0]])[0]
    assert list(turned) == pytest.approx([0.0, 1.0])


def test_invert_round_trips_points_and_angles():
    t = Pose2D(x=-3.2, y=7.1, yaw=1.1)
    points = np.array([[0.0, 0.0], [5.0, -2.0], [-1.5, 9.0]])
    assert t.invert().apply(t.apply(points)) == pytest.approx(points)
    assert t.invert().apply_yaw(t.apply_yaw(0.3)) == pytest.approx(0.3)


def test_the_transform_puts_the_scenario_start_where_the_robot_stands():
    """The point of it: the layout moves, the robot does not."""
    start = (3.0, 4.0, math.radians(30.0))
    robot = (-1.0, 0.5, math.radians(-100.0))
    t = transform_placing(start, robot)
    assert list(t.apply([start[:2]])[0]) == pytest.approx(list(robot[:2]))
    assert t.apply_yaw(start[2]) == pytest.approx(robot[2])


def test_the_start_pose_error_is_zero_where_the_hall_was_built_around_the_robot():
    start = (3.0, 4.0, math.radians(30.0))
    robot = (-1.0, 0.5, math.radians(-100.0))
    position, yaw = start_pose_error(transform_placing(start, robot), start, robot)
    assert position == pytest.approx(0.0, abs=1e-12) and yaw == pytest.approx(0.0, abs=1e-12)


def test_the_start_pose_error_of_a_hall_built_around_a_stale_pose():
    """The 2026-10-10 bug, in numbers: the hall placed around the previous episode's end pose
    (5.764 m up the world's +y, facing +y) while the robot is back at the origin. Known answer:
    the robot stands 5.764 m from the start along the start's own heading, behind it."""
    start = (4.0, 5.0, 0.0)
    stale = (0.0, 5.764, math.pi / 2)
    origin = (0.0, 0.0, math.pi / 2)
    position, yaw = start_pose_error(transform_placing(start, stale), start, origin)
    assert position == pytest.approx(5.764) and yaw == pytest.approx(0.0, abs=1e-12)


def test_the_start_pose_error_reports_a_quarter_turn_and_wraps():
    start = (1.0, 2.0, math.radians(170.0))
    t = transform_placing(start, (0.0, 0.0, 0.0))
    _p, quarter = start_pose_error(t, start, (0.0, 0.0, math.pi / 2))
    _p, across = start_pose_error(t, start, (0.0, 0.0, math.radians(-20.0)))
    assert quarter == pytest.approx(math.pi / 2)
    assert across == pytest.approx(math.radians(20.0))   # wrapped, not 340 deg


def test_the_transform_is_rigid_so_the_layout_keeps_its_shape():
    t = transform_placing((2.0, -3.0, 0.4), (10.0, 10.0, 2.0))
    a, b = np.array([[1.0, 1.0]]), np.array([[4.0, 5.0]])
    before = float(np.linalg.norm(b - a))
    after = float(np.linalg.norm(t.apply(b) - t.apply(a)))
    assert after == pytest.approx(before)


def test_rectangle_of_known_answers():
    axis_aligned = np.array([[0.0, 0.0], [4.0, 0.0], [4.0, 1.0], [0.0, 1.0]])
    cx, cy, length, width, yaw = rectangle_of(axis_aligned)
    assert (cx, cy) == pytest.approx((2.0, 0.5))
    assert (length, width) == pytest.approx((4.0, 1.0))
    assert yaw == pytest.approx(0.0)


def test_rectangle_of_reports_the_long_axis_angle():
    turned = np.array([[0.0, 0.0], [2.0, 2.0], [1.0, 3.0], [-1.0, 1.0]])
    cx, cy, length, width, yaw = rectangle_of(turned)
    assert (cx, cy) == pytest.approx((0.5, 1.5))
    assert length == pytest.approx(math.hypot(2.0, 2.0))
    assert width == pytest.approx(math.hypot(1.0, 1.0))
    assert yaw == pytest.approx(math.pi / 4)


def test_a_tall_rectangle_still_reports_its_long_side_as_the_length():
    tall = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 5.0], [0.0, 5.0]])
    _cx, _cy, length, width, yaw = rectangle_of(tall)
    assert (length, width) == pytest.approx((5.0, 1.0))
    assert abs(yaw) == pytest.approx(math.pi / 2)


def test_a_polygon_that_is_not_a_rectangle_is_refused_not_approximated():
    with pytest.raises(ValueError, match="not a rectangle"):
        rectangle_of(np.array([[0.0, 0.0], [4.0, 0.0], [3.0, 2.0], [0.0, 1.0]]))
    with pytest.raises(ValueError, match="rectangle"):
        rectangle_of(np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]))
