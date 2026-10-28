"""Known answers for the declared classical recovery (ruled 2026-10-13 (3b) and 2026-10-16).

Synthetic scans ray-cast against body-frame boxes (`synthetic_inputs.py`). The field numbers the
scenes rely on, from the configured footprint: at rest the field reaches 0.336 m ahead and 0.260 m
abeam; band 0's leading field 0.401 m ahead and 0.325 m abeam. Each scene asserts its own
preconditions, so a change of footprint or margins fails here by name rather than passing on a
scene that no longer means what it says.
"""
import math
from pathlib import Path

import numpy as np
import pytest
from synthetic_inputs import box, make_input, ray_scan

from pfbnav.config import load_config
from pfbnav.control.pure_pursuit import PurePursuit, ZeroAction
from pfbnav.control.recovery import ENDINGS, Recovery, RecoveryGeometry, choose_direction
from pfbnav.safety.certified import CertifiedLayer
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.world.strata import certified_params_from_config

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
GEOM = RecoveryGeometry.from_config(CFG)
R = CFG["baselines"]["recovery"]
TRIGGER = 60                         # (restart_delay 2.0 s + wait 1.0 s) / 0.05 s

WALL_AHEAD = ray_scan([box(0.37, -1.0, 0.6, 1.0)])          # blocks straight ahead only
CORNER_LEFT = ray_scan([box(0.28, 0.28, 0.88, 1.28)])        # a shelf corner at 45 deg, 0.40 m
RING = ray_scan([box(0.37, -2, 0.57, 2), box(-0.57, -2, -0.37, 2),
                 box(-2, 0.29, 2, 0.49), box(-2, -0.49, 2, -0.29)])   # nowhere to go
INSIDE_MARGIN = ray_scan([box(0.30, -1.0, 0.6, 1.0)])         # inside the at-rest field
OPEN = ray_scan()
AHEAD, AT_35 = (0.8, 0.0), (0.8 * math.cos(math.radians(35)), 0.8 * math.sin(math.radians(35)))
# Twists for the restart test, which since 2026-10-19 clears the WHOLE field a twist selects:
# B1's cruise along a heading, and the recovery's creep (at rest by the standstill rule).
CRUISE = 0.25


def twist(speed, heading):
    return (speed * math.cos(heading), speed * math.sin(heading))


def emulator() -> CertifiedLayer:
    """The certified layer exactly as the step loop builds it."""
    table = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
    return CertifiedLayer(certified_params_from_config(CFG), float(CFG["robot"]["half_length"]),
                          float(CFG["robot"]["half_width"]), table.extent)


class Forward:
    """A classical base controller asking for one fixed body-frame direction."""

    name, input_kind = "probe", "classical"

    def __init__(self, direction=(1.0, 0.0), speed=0.25):
        d = np.asarray(direction, dtype=float)
        self.cmd = np.array([*(speed * d / np.linalg.norm(d)), 0.0])
        self.ceilings = np.array([0.40, 0.50, 1.50])

    def reset(self):
        pass

    def command(self, inp):
        return self.cmd.copy()

    def act(self, inp):
        return self.command(inp) / self.ceilings


def wrapped(base=None) -> Recovery:
    return Recovery.from_config(base if base is not None else Forward(), CFG)


def stopped(scan, lookahead=AHEAD, velocity=(0.0, 0.0, 0.0)):
    return make_input(stop=True, scan=scan, lookahead=lookahead, velocity=velocity)


def is_recovery(cmd, rec) -> bool:
    return np.isclose(np.hypot(cmd[0], cmd[1]), rec.speed) and cmd[2] == 0.0


# ----------------------------------------------------------------------
# The scenes mean what they say
# ----------------------------------------------------------------------
def test_the_scenes_preconditions():
    assert not GEOM.at_rest_breached(WALL_AHEAD)
    assert not GEOM.permitted(WALL_AHEAD, twist(CRUISE, 0.0))
    assert not GEOM.at_rest_breached(CORNER_LEFT)
    assert not GEOM.permitted(CORNER_LEFT, twist(CRUISE, math.radians(35)))
    assert not GEOM.at_rest_breached(RING)
    assert GEOM.at_rest_breached(INSIDE_MARGIN)
    assert GEOM.permitted(OPEN, twist(CRUISE, 0.0))
    # the creep regime: the at-rest field governs, and it is clear in every scene but INSIDE_MARGIN
    for scan in (WALL_AHEAD, CORNER_LEFT, RING):
        assert GEOM.permitted(scan, twist(R["speed"], 0.0))
    assert not GEOM.permitted(INSIDE_MARGIN, twist(R["speed"], math.pi))


def test_recovery_judges_permission_exactly_as_the_emulator_does():
    """Parity: the wrapper's geometry is the certified layer's, and so is its restart test."""
    layer = emulator()
    for scan in (WALL_AHEAD, CORNER_LEFT, RING, INSIDE_MARGIN, OPEN):
        for k in range(72):
            for speed in (R["speed"], CRUISE):
                d = twist(speed, 2 * math.pi * k / 72)
                assert GEOM.permitted(scan, d) == layer._restart_allowed(
                    scan.ranges, scan.bearings, scan.valid, d)
    for h in np.linspace(-math.pi, math.pi, 13):
        assert np.array_equal(GEOM.fields.extent(OPEN.bearings, GEOM.band0, h, at_rest=False),
                              layer.fields.extent(OPEN.bearings, GEOM.band0, h, at_rest=False))


# ----------------------------------------------------------------------
# The direction
# ----------------------------------------------------------------------
def _brute_clearance(scan, heading):
    """The ruled clearance, computed independently of `RecoveryGeometry.clearance`: the
    footprint's boundary sampled densely as POINTS (not segments), translated the full
    manoeuvre, its least distance to every scan point; negative where a point is inside."""
    table = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
    psi = np.linspace(-math.pi, math.pi, 5760, endpoint=False)
    e = table.extent(psi)
    boundary = np.column_stack([e * np.cos(psi), e * np.sin(psi)])
    travel = float(R["max_distance"])
    r = np.where(scan.valid, scan.ranges, table.extent(scan.bearings))   # the shared convention

    pts = np.column_stack([r * np.cos(scan.bearings), r * np.sin(scan.bearings)])
    pts = pts - travel * np.array([math.cos(heading), math.sin(heading)])
    d = np.min(np.linalg.norm(pts[:, None, :] - boundary[None, :, :], axis=2), axis=1)
    inside = np.hypot(pts[:, 0], pts[:, 1]) < table.extent(np.arctan2(pts[:, 1], pts[:, 0]))
    return float(np.min(np.where(inside, -d, d)))


def _check_choice(scan, route_bearing):
    """The choice is permitted by the emulator's own test, and its clearance is the best of the
    permitted directions' (brute force) within the tie tolerance and the brute force's own
    sampling error (a chord of 5,760 points on a 0.34 m footprint: under 0.02 mm)."""
    layer = emulator()
    h = choose_direction(GEOM, scan, route_bearing, R["directions"], R["tie_tolerance"],
                         R["speed"])
    permitted = [math.remainder(2 * math.pi * k / 72, 2 * math.pi) for k in range(72)
                 if layer._restart_allowed(scan.ranges, scan.bearings, scan.valid,
                                           twist(R["speed"], 2 * math.pi * k / 72))]
    assert any(abs(math.remainder(h - p, 2 * math.pi)) < 1e-9 for p in permitted)
    best = max(_brute_clearance(scan, p) for p in permitted)
    assert _brute_clearance(scan, h) >= best - R["tie_tolerance"] - 5e-5
    assert GEOM.clearance(scan, h) == pytest.approx(_brute_clearance(scan, h), abs=5e-5)
    return h


def test_a_corner_ahead_left_sends_it_back_and_away_and_the_choice_is_the_ruled_one():
    """The 2026-10-13 situation: Pure Pursuit heading at 35 deg into a shelf corner. With the
    clearance measured after the full manoeuvre (ruled 2026-10-16), recovery backs away to the
    right, away from the corner, instead of creeping along the route."""
    h = _check_choice(CORNER_LEFT, math.radians(35))
    assert math.sin(h) < 0.0 and math.cos(h) < 0.0, math.degrees(h)


def test_a_wall_ahead_sends_it_straight_back():
    """By symmetry the unique best direction is straight back, whatever the route says."""
    for route in (0.0, math.radians(35)):
        h = _check_choice(WALL_AHEAD, route)
        assert abs(math.remainder(h - math.pi, 2 * math.pi)) < 1e-9, math.degrees(h)


def test_ties_go_to_the_direction_closest_to_the_route():
    import copy

    class Flat(RecoveryGeometry):
        def permitted(self, scan, direction):
            return True

        def clearance(self, scan, heading):
            return 0.5                                   # every direction ties

    flat = copy.copy(GEOM)
    flat.__class__ = Flat
    for route in (0.0, math.radians(100), math.radians(-170)):
        h = choose_direction(flat, OPEN, route, 72, 0.001, R["speed"])
        assert abs(math.remainder(h - route, 2 * math.pi)) <= math.radians(2.5) + 1e-12


def test_the_footprint_distance_known_answers():
    """Signed: zero on the boundary, the radial gap straight ahead, negative inside."""
    table = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
    ahead = float(table.extent(np.array([0.0]))[0])
    d = GEOM.footprint_distance([[ahead + 0.1, 0.0], [ahead, 0.0], [0.05, 0.0]])
    assert d[0] == pytest.approx(0.1, abs=1e-3) and abs(d[1]) < 1e-3 and d[2] < -0.1


def test_no_permitted_direction_only_inside_the_minimum_margin():
    """Since 2026-10-19 a creep is judged on the at-rest field, so every direction is permitted
    whenever that field is clear -- the walled-in RING too, where the choice is the direction with
    the most clearance after the full manoeuvre (the ruled metric). Nothing is permitted only when
    the at-rest field itself is breached (and there the deadlock check comes first)."""
    h = choose_direction(GEOM, RING, 0.0, R["directions"], R["tie_tolerance"], R["speed"])
    assert h is not None
    best = max(GEOM.clearance(RING, 2 * math.pi * k / 72) for k in range(72))
    assert GEOM.clearance(RING, h) >= best - R["tie_tolerance"]
    assert choose_direction(GEOM, INSIDE_MARGIN, 0.0, R["directions"], R["tie_tolerance"],
                            R["speed"]) is None


# ----------------------------------------------------------------------
# The trigger
# ----------------------------------------------------------------------
def test_it_waits_the_restart_delay_plus_the_wait_then_moves_slowly_without_turning():
    rec = wrapped()
    for _ in range(TRIGGER - 1):
        assert np.array_equal(rec.command(stopped(WALL_AHEAD)), rec.base.cmd)
    cmd = rec.command(stopped(WALL_AHEAD))
    assert is_recovery(cmd, rec) and rec.attempts == 1
    assert np.all(np.abs(rec.act(stopped(WALL_AHEAD))) <= 1.0)


def test_the_wait_counts_from_when_the_robots_own_field_comes_clear():
    """While the at-rest field is still breached the emulator's restart delay has not started,
    so neither has recovery's."""
    rec = wrapped()
    for _ in range(40):
        rec.command(stopped(INSIDE_MARGIN, velocity=(0.05, 0.0, 0.0)))   # still moving
    for _ in range(TRIGGER - 1):
        assert np.array_equal(rec.command(stopped(WALL_AHEAD)), rec.base.cmd)
    assert is_recovery(rec.command(stopped(WALL_AHEAD)), rec)


def test_a_permitted_base_command_is_never_overridden():
    rec = wrapped()
    for _ in range(5 * TRIGGER):
        assert np.array_equal(rec.command(stopped(OPEN)), rec.base.cmd)
    assert rec.attempts == 0


def test_second_negative_control_a_direction_that_becomes_permitted_on_its_own():
    """Ruled 2026-10-16: a stop whose base direction becomes permitted before the trigger must
    not trigger recovery."""
    rec = wrapped()
    for _ in range(TRIGGER - 10):
        rec.command(stopped(WALL_AHEAD))
    for _ in range(3 * TRIGGER):
        assert np.array_equal(rec.command(stopped(OPEN)), rec.base.cmd)
    assert rec.attempts == 0 and rec.gave_up is None


def test_negative_control_zero_action_never_triggers_recovery():
    rec = wrapped(ZeroAction.from_config(CFG))
    for scan in (WALL_AHEAD, CORNER_LEFT, RING):
        for _ in range(5 * TRIGGER):
            assert np.array_equal(rec.command(stopped(scan)), np.zeros(3))
    assert rec.attempts == 0 and rec.gave_up is None


def test_inside_the_minimum_margin_it_records_a_deadlock_and_never_moves():
    rec = wrapped()
    for _ in range(3 * TRIGGER):
        assert np.array_equal(rec.command(stopped(INSIDE_MARGIN)), rec.base.cmd)
    assert rec.gave_up == "inside_min_margin" and rec.attempts == 0
    assert rec.episode_report()["recovery_deadlock"] == "inside_min_margin"


def test_walled_in_it_creeps_toward_the_most_room():
    """The RING ended in `no_permitted_direction` until 2026-10-19. With the creep judged on the
    clear at-rest field, an attempt begins, toward the direction with the most clearance."""
    rec = wrapped()
    cmd = np.zeros(3)
    for _ in range(TRIGGER):
        cmd = rec.command(stopped(RING))
    assert is_recovery(cmd, rec) and rec.attempts == 1 and rec.gave_up is None


# ----------------------------------------------------------------------
# How an attempt ends
# ----------------------------------------------------------------------
def _start(rec, scan=WALL_AHEAD):
    cmd = np.zeros(3)
    for _ in range(TRIGGER):
        cmd = rec.command(stopped(scan))
    assert is_recovery(cmd, rec)
    return cmd


def moving(scan, v=0.08):
    return make_input(stop=False, scan=scan, velocity=(v, 0.0, 0.0))


def test_an_attempt_ends_when_the_base_direction_is_permitted_again():
    rec = wrapped()
    _start(rec)
    assert is_recovery(rec.command(moving(WALL_AHEAD)), rec)        # released, still blocked
    assert np.array_equal(rec.command(moving(OPEN)), rec.base.cmd)  # cleared: back to the base
    assert rec.endings["cleared"] == 1 and rec.mode == "pass"


def test_an_attempt_ends_at_the_distance_limit_integrated_from_the_estimate():
    rec = wrapped()
    _start(rec)
    steps = 0
    while rec.mode == "recover":
        rec.command(moving(WALL_AHEAD, v=0.08))
        steps += 1
    # 0.30 m at 0.08 m/s and 0.05 s per step: 75 steps of travel.
    assert steps == 75 and rec.endings["distance"] == 1
    assert rec.distance_m == pytest.approx(0.30, abs=1e-9)


def test_an_attempt_ends_on_a_new_stop_and_on_a_stop_that_never_released():
    rec = wrapped()
    _start(rec)
    rec.command(moving(WALL_AHEAD))
    assert np.array_equal(rec.command(stopped(WALL_AHEAD)), rec.base.cmd)
    assert rec.endings["new_stop"] == 1
    rec2 = wrapped()
    _start(rec2)
    rec2.command(stopped(WALL_AHEAD))                                 # emulator did not release
    assert rec2.endings["not_released"] == 1


def test_attempts_are_capped_per_episode_then_recovery_stops():
    rec = wrapped()
    for attempt in range(1, R["max_attempts"] + 1):
        calls = 0
        while rec.attempts < attempt:                                 # the step of the new stop
            rec.command(stopped(WALL_AHEAD))                          # counts toward the next
            calls += 1                                                # trigger
            assert calls <= TRIGGER
        rec.command(moving(WALL_AHEAD))
        rec.command(stopped(WALL_AHEAD))                              # new stop
    assert rec.attempts == R["max_attempts"] and rec.endings["new_stop"] == R["max_attempts"]
    for _ in range(3 * TRIGGER):
        assert np.array_equal(rec.command(stopped(WALL_AHEAD)), rec.base.cmd)
    assert rec.gave_up == "cap" and rec.attempts == R["max_attempts"]


def test_the_report_and_a_reset():
    rec = wrapped()
    _start(rec)
    rec.command(moving(WALL_AHEAD))
    rec.command(moving(OPEN))                                         # cleared
    report = rec.episode_report()
    assert set(report["recovery_endings"]) == set(ENDINGS)
    assert report["recovery_attempts"] == 1 and report["recovery_distance_m"] > 0.0
    (attempt,) = report["recovery_attempt_log"]
    assert attempt["start_step"] == TRIGGER and attempt["ending"] == "cleared"
    assert attempt["steps"] == 2 and attempt["distance_m"] == pytest.approx(2 * 0.08 * 0.05)
    assert report["recovery_time_s"] == pytest.approx(2 * 0.05)
    assert attempt["time_s"] == pytest.approx(2 * 0.05)
    assert report["recovery_deadlock"] is None
    rec.reset()
    assert rec.episode_report()["recovery_attempts"] == 0 and rec.mode == "pass"


def test_an_attempt_cut_short_by_the_episode_end_still_reports_its_time():
    """Ruled 2026-10-19: attempt times are reported in G4. An attempt in progress when the
    episode ends has no ending, but its steps and time are already in the log."""
    rec = wrapped()
    _start(rec)
    for _ in range(3):
        rec.command(moving(WALL_AHEAD))
    (attempt,) = rec.episode_report()["recovery_attempt_log"]
    assert attempt["ending"] is None
    assert attempt["steps"] == 4 and attempt["time_s"] == pytest.approx(4 * 0.05)


def test_it_is_deterministic_and_keeps_its_base_controllers_identity():
    a, b = wrapped(PurePursuit.from_config(CFG)), wrapped(PurePursuit.from_config(CFG))
    seq = [stopped(CORNER_LEFT, lookahead=AT_35)] * (TRIGGER + 3) + [moving(CORNER_LEFT)] * 20
    assert all(np.array_equal(a.command(i), b.command(i)) for i in seq)
    assert a.name == "B1_pure_pursuit" and a.input_kind == "classical"
    with pytest.raises(TypeError):
        Recovery.from_config(type("Learned", (), {"name": "L5", "input_kind": "observation"})(),
                             CFG)


def test_the_corner_stop_fixture_still_reproduces_the_2026_10_13_route():
    """The simulator positive control (`tests/test_recovery_sim.py`) replays scenario 2000057 at
    that day's lethal margin. Its premise, checked without a simulator: the world rebuilds to its
    recorded identity, and the route planned at 0.03 passes the recorded stop pose (it did, by
    5.7 mm, 1.45 m along)."""
    import copy
    import json

    from pfbnav.planning.astar import plan_from_config
    from pfbnav.planning.lookahead import project
    from pfbnav.world.baking import bake, planner_radii
    from pfbnav.world.scenarios import _circumscribed, world_of

    fixture = json.loads((REPO / "tests" / "data" / "corner_stop_20261013.json")
                         .read_text(encoding="utf-8"))
    cfg = copy.deepcopy(CFG)
    cfg["planner"]["lethal_margin"] = fixture["planner_lethal_margin"]
    world = world_of(fixture["scenario"], cfg)              # raises if the identity moved
    baked = bake(world.mapped, world.bounds, float(cfg["planner"]["resolution"]),
                 planner_radii(cfg, circumscribed=_circumscribed(cfg)))
    plan = plan_from_config(baked, tuple(fixture["scenario"]["start"][:2]),
                            tuple(fixture["scenario"]["goal"]), cfg)
    where = project(np.asarray(plan.path_world),
                    np.asarray(fixture["first_stop_layout_pose"][:2]), window=1e9)
    assert where.distance < 0.02 and where.s == pytest.approx(1.45, abs=0.05)
