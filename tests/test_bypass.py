"""Known-answer tests for the local-bypass test (A13 revision, 2026-10-02; hard rules 6, 14). Pure."""
import numpy as np
import pytest

from pfbnav.world.bypass import BypassRule, Corridor
from pfbnav.world.layouts import rectangle

HALF_WIDTH, FLOOR = 0.2086, 0.13
RULE = BypassRule(radius=HALF_WIDTH + FLOOR, corridor=1.0, resolution=0.01)


def lane(clearance, shift=0.0, length=2.0):
    """Two walls the length of the route, `clearance` per side beyond the footprint."""
    half = HALF_WIDTH + clearance
    walls = [rectangle(-2.0, shift + half, length + 2.0, shift + half + 1.0),
             rectangle(-2.0, shift - half - 1.0, length + 2.0, shift - half)]
    route = np.array([[0.0, shift], [length, shift]])
    return route, walls


def test_the_radius_is_read_from_the_robot_and_the_narrow_floor():
    cfg = {"robot": {"half_width": 0.2086},
           "world": {"aisle_strata": [{"name": "narrow", "min_clearance": 0.13}],
                     "layout": {"unmapped": {"route_relative": {
                         "bypass": {"corridor_half_width": 1.0, "resolution": 0.01}}}}}}
    rule = BypassRule.from_config(cfg)
    assert rule.radius == pytest.approx(0.3386) and rule.corridor == 1.0
    assert BypassRule.from_config(cfg, corridor=1.5).corridor == 1.5


def test_the_rule_refuses_impossible_values():
    with pytest.raises(ValueError):
        BypassRule(radius=0.3, corridor=1.0, resolution=0.0)
    with pytest.raises(ValueError):
        BypassRule(radius=0.3, corridor=0.0, resolution=0.01)


@pytest.mark.parametrize("shift", [0.0, 0.0025, 0.005, 0.0075, 0.00999])
def test_a_gap_at_the_floor_always_passes_and_one_below_floor_less_half_a_cell_never(shift):
    """The dense sweep: per-side clearance 0.115-0.145 m in 1 mm steps, at five alignments of
    the walls against the grid. At or above the floor the gap is ALWAYS admitted (the narrowest
    mapped aisles sit at exactly the floor, so anything else would fail the positive control);
    more than half a cell below it, NEVER. In between is the documented approximation."""
    tol = RULE.tolerance
    for mm in range(115, 146):
        c = mm / 1000.0
        route, walls = lane(c, shift)
        got = Corridor(route, walls, route[0], route[-1], RULE).passable([])
        if c >= FLOOR - 1e-12:
            assert got, f"a {c:.3f} m gap was refused at shift {shift}"
        elif c < FLOOR - tol - 1e-9:
            assert not got, f"a {c:.3f} m gap was admitted at shift {shift}"


def test_an_obstacle_leaving_a_floor_gap_beside_it_is_passable_and_one_cm_less_is_not():
    """The case itself: a square on the route, a wall on one side, nothing on the other."""
    route = np.array([[0.0, 0.0], [4.0, 0.0]])
    wall = rectangle(-2.0, 0.9, 6.0, 1.9)
    for c, want in ((0.14, True), (0.13, True), (0.12, False), (0.10, False)):
        # The obstacle's top face sits 2 * (half_width + c) below the wall; below it, a second
        # wall closes the other side so the only way is between the obstacle and `wall`.
        top = 0.9 - 2 * (HALF_WIDTH + c)
        obstacle = rectangle(1.8, top - 0.4, 2.2, top)
        floor_wall = rectangle(-2.0, top - 1.4, 6.0, top - 0.4)
        corridor = Corridor(route + [0.0, top + HALF_WIDTH + c], [wall],
                            [0.0, top + HALF_WIDTH + c], [4.0, top + HALF_WIDTH + c],
                            BypassRule(RULE.radius, 2.0, RULE.resolution))
        assert corridor.passable([obstacle, floor_wall]) is want, c


def test_a_bypass_beyond_w_is_refused_and_within_w_admitted():
    """A barrier across the route leaves a way round only 1.14 m to the side
    (0.8 m + the 0.3386 m radius): a detour at W = 1.0, local avoidance at W = 1.5."""
    route = np.array([[0.0, 0.0], [6.0, 0.0]])
    barrier = rectangle(3.0, -10.0, 3.3, 0.8)
    for w, want in ((1.0, False), (1.5, True)):
        rule = BypassRule(RULE.radius, w, RULE.resolution)
        assert Corridor(route, [], route[0], route[-1], rule).passable([barrier]) is want, w


def test_obstacles_are_judged_jointly():
    """Each obstacle alone leaves a gap of 0.8 or 1.4 m; together they leave 0.2 m."""
    route = np.array([[0.0, 0.0], [6.0, 0.0]])
    walls = [rectangle(-2.0, 1.0, 8.0, 2.0), rectangle(-2.0, -2.0, 8.0, -1.0)]
    a = rectangle(3.0, -0.2, 3.4, 1.0)
    b = rectangle(3.0, -1.0, 3.4, -0.4)
    corridor = Corridor(route, walls, route[0], route[-1], RULE)
    assert corridor.passable([]) is True                  # the positive control
    assert corridor.passable([a]) is True
    assert corridor.passable([b]) is True
    assert corridor.passable([a, b]) is False


def test_an_obstacle_on_the_goal_closes_the_bypass():
    route = np.array([[0.0, 0.0], [3.0, 0.0]])
    corridor = Corridor(route, [], route[0], route[-1], RULE)
    assert not corridor.passable([rectangle(2.8, -0.2, 3.2, 0.2)])


def test_clearance_is_exact_where_it_decides_and_never_understated_elsewhere():
    route = np.array([[0.0, 0.0], [2.0, 0.0]])
    poly = rectangle(1.0, 0.5, 1.2, 0.7)
    corridor = Corridor(route, [], route[0], route[-1], RULE)
    d = corridor.clearance([poly])
    exact = np.hypot(np.maximum(0.0, np.maximum(1.0 - corridor.points[:, 0],
                                                 corridor.points[:, 0] - 1.2)),
                     np.maximum(0.0, np.maximum(0.5 - corridor.points[:, 1],
                                                 corridor.points[:, 1] - 0.7)))
    within = exact < RULE.radius + RULE.resolution
    assert within.sum() > 1000
    assert d[within] == pytest.approx(exact[within], abs=1e-12)
    assert np.all(d[~within] >= exact[~within] - 1e-12)
