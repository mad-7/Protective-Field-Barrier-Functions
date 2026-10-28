"""The receding release (A24, ruled 2026-10-21): the six known answers the ruling names.

A creep stop ends with the intrusion a few mm inside the at-rest field, which under the whole-field
restart (A22) could never be left. Released only for a CREEP twist under which every point inside the
at-rest field strictly recedes; those points (only those) are exempt while the command keeps receding
at creep. Synthetic scans; the disc robot (radius 0.21 m) of `tests/test_certified_layer.py`, whose
at-rest field is a ring of about 0.273 m, and a rectangle for the corner-swing case.
"""
import math

import numpy as np
import pytest
from test_certified_layer import BEARINGS, DT, HALF_LENGTH, HALF_WIDTH, PARAMS, _bin, clear, layer

from pfbnav.safety.certified import CertifiedLayer

CREEP = 0.015                     # m/s, below the 0.02 standstill exit
INSIDE = 0.268                    # m: inside the disc's at-rest field (~0.273), outside its 0.21


def _held(cl, ahead=INSIDE, t=0.0):
    """Creep toward an intrusion ahead until it stops, and hold at rest inside the at-rest field.
    Returns (time, ranges, valid)."""
    ranges, valid = clear()
    ranges = ranges.copy()
    ranges[_bin(0.0)] = 0.29                        # outside the at-rest field: no stop
    s = cl.step(t, (CREEP, 0.0, 0.0), ranges, BEARINGS, valid, intended_direction=(CREEP, 0.0))
    assert not s["stop_triggered"]
    t += DT
    ranges[_bin(0.0)] = ahead                       # the creep carried it inside
    s = cl.step(t, (CREEP, 0.0, 0.0), ranges, BEARINGS, valid, intended_direction=(CREEP, 0.0))
    assert s["stop_triggered"] and s["triggered_at_rest"] and not s["triggered_stationary"]
    for _ in range(4):                              # past the response time, now at rest
        t += DT
        s = cl.step(t, (0.0, 0.0, 0.0), ranges, BEARINGS, valid, intended_direction=(0.0, 0.0))
    assert s["stop_active"] and s["breached"] and s["at_rest"]
    return t, ranges, valid


def _hold_with(cl, t, ranges, valid, twist, steps):
    s = None
    for _ in range(steps):
        t += DT
        s = cl.step(t, (0.0, 0.0, 0.0), ranges, BEARINGS, valid, intended_direction=twist)
    return t, s


WAIT = int(PARAMS.restart_delay / DT) + 2


def test_creep_in_then_the_receding_release():
    cl = layer()
    t, ranges, valid = _held(cl)
    stops = cl.stops
    t, s = _hold_with(cl, t, ranges, valid, (-CREEP, 0.0, 0.0), WAIT)
    assert not s["stop_active"] and s["receding"] and cl.receding_releases == 1
    # Receding: the point drifts outward, still inside for a while -- no new stop.
    for k in range(10):
        t += DT
        ranges[_bin(0.0)] = INSIDE + 0.0005 * (k + 1)
        s = cl.step(t, (-CREEP, 0.0, 0.0), ranges, BEARINGS, valid,
                    intended_direction=(-CREEP, 0.0, 0.0))
        assert not s["stop_triggered"] and not s["stop_active"]
    # Once it has left the at-rest field the exemption ends by itself.
    ranges[_bin(0.0)] = 0.30
    t += DT
    s = cl.step(t, (-CREEP, 0.0, 0.0), ranges, BEARINGS, valid, intended_direction=(-CREEP, 0.0))
    assert not s["receding"] and cl._recede is None and cl.stops == stops


def test_the_release_waits_for_the_restart_delay():
    cl = layer()
    t, ranges, valid = _held(cl)
    # The hold began during `_held`'s last steps; 1.6 s more is short of the 2.0 s delay.
    t, s = _hold_with(cl, t, ranges, valid, (-CREEP, 0.0, 0.0), int(1.6 / DT))
    assert s["stop_active"] and s["restart_blocked"]
    t, s = _hold_with(cl, t, ranges, valid, (-CREEP, 0.0, 0.0), int(0.6 / DT))
    assert not s["stop_active"]


@pytest.mark.parametrize("twist", [(CREEP, 0.0, 0.0),            # toward it
                                   (0.0, CREEP, 0.0),            # exactly abeam: not strict
                                   (0.0, 0.0, 0.05),             # a spin on the spot
                                   (-0.05, 0.0, 0.0),            # receding, but above creep
                                   (-CREEP, 0.0, 0.07)])         # receding, yaw above creep
def test_an_approach_or_anything_short_of_a_strictly_receding_creep_stays_blocked(twist):
    cl = layer()
    t, ranges, valid = _held(cl)
    t, s = _hold_with(cl, t, ranges, valid, twist, 3 * WAIT)
    assert s["stop_active"] and s["restart_blocked"] and cl.receding_releases == 0


def test_an_unobserved_bin_inside_still_blocks():
    cl = layer()
    t, ranges, valid = _held(cl)
    valid = valid.copy()
    valid[_bin(math.pi)] = False                    # blind behind: breaches every field
    t, s = _hold_with(cl, t, ranges, valid, (-CREEP, 0.0, 0.0), 3 * WAIT)
    assert s["stop_active"] and s["restart_blocked"]


def _rectangle():
    def extent(psi):
        c, s = np.abs(np.cos(psi)), np.abs(np.sin(psi))
        with np.errstate(divide="ignore"):
            return np.minimum(np.where(c > 1e-12, HALF_LENGTH / c, np.inf),
                              np.where(s > 1e-12, HALF_WIDTH / s, np.inf))
    return CertifiedLayer(PARAMS, HALF_LENGTH, HALF_WIDTH, extent)


def test_a_rotation_swinging_a_corner_toward_the_intrusion_is_blocked():
    """A point 30 mm off the front-left corner, ahead of it at 80 deg: backing straight away
    recedes from it; the same backing with a counter-clockwise creep rotation swings the corner
    toward it and is blocked; the clockwise one swings it away and is released."""
    corner = np.array([HALF_LENGTH, HALF_WIDTH])
    point = corner + 0.03 * np.array([math.cos(math.radians(80)), math.sin(math.radians(80))])
    bearing, rng = math.atan2(point[1], point[0]), float(np.hypot(*point))
    outcomes = {}
    for w in (0.0, 0.05, -0.05):
        cl = _rectangle()
        ranges, valid = clear()
        ranges = ranges.copy()
        ranges[_bin(bearing)] = rng
        s = cl.step(0.0, (0.0, 0.0, 0.0), ranges, BEARINGS, valid)
        assert s["stop_triggered"]                    # inside the at-rest field from the start
        t, s = 0.0, None
        for _ in range(WAIT + 4):
            t += DT
            s = cl.step(t, (0.0, 0.0, 0.0), ranges, BEARINGS, valid,
                        intended_direction=(-CREEP, 0.0, w))
        outcomes[w] = s["stop_active"]
    assert outcomes == {0.0: False, 0.05: True, -0.05: False}


def test_receding_into_a_second_obstacle_triggers():
    cl = layer()
    t, ranges, valid = _held(cl)
    t, s = _hold_with(cl, t, ranges, valid, (-CREEP, 0.0, 0.0), WAIT)
    assert not s["stop_active"]
    stops = cl.stops
    ranges[_bin(math.pi)] = INSIDE                  # something behind enters the at-rest field
    t += DT
    s = cl.step(t, (-CREEP, 0.0, 0.0), ranges, BEARINGS, valid, intended_direction=(-CREEP, 0.0))
    assert s["stop_triggered"] and cl.stops == stops + 1 and not s["receding"]


def test_receding_then_approaching_retriggers_on_the_next_step():
    cl = layer()
    t, ranges, valid = _held(cl)
    t, s = _hold_with(cl, t, ranges, valid, (-CREEP, 0.0, 0.0), WAIT)
    assert not s["stop_active"]
    stops = cl.stops
    t += DT                                         # the command turns back toward the point
    s = cl.step(t, (-CREEP, 0.0, 0.0), ranges, BEARINGS, valid, intended_direction=(CREEP, 0.0))
    assert s["stop_triggered"] and cl.stops == stops + 1 and cl._recede is None


class _NoRelease(CertifiedLayer):
    """A22's rule: the release machinery never engages."""

    def _recedes(self, *args, **kwargs):
        return False


def test_without_a_creep_stop_the_layer_is_bit_identical_to_a22():
    """Random moving scenarios -- approaches, stops, restarts, rotations, blind bins -- with no
    robot ever held at rest inside its at-rest field: every state the layer reports equals A22's
    (the release disabled), and no release happens."""
    rng = np.random.default_rng(11)
    for _ in range(8):
        new = layer()
        old = _NoRelease(PARAMS, HALF_LENGTH, HALF_WIDTH,
                         lambda psi: np.full_like(np.asarray(psi, dtype=float), 0.21))
        ranges, valid = clear()
        for k in range(150):
            ranges = ranges.copy()
            if rng.random() < 0.1:
                ranges[rng.integers(0, ranges.size)] = rng.uniform(0.29, 1.5)
            if rng.random() < 0.05:
                ranges[:] = 4.0
            speed = rng.choice([0.0, 0.1, 0.25, 0.4])
            twist = (speed * math.cos(k / 20), speed * math.sin(k / 20), rng.uniform(-0.5, 0.5))
            a = new.step(k * DT, twist, ranges, BEARINGS, valid, intended_direction=twist)
            b = old.step(k * DT, twist, ranges, BEARINGS, valid, intended_direction=twist)
            assert a == b
        assert new.receding_releases == 0
