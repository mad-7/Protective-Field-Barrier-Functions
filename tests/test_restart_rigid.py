"""The restart condition (ruled 2026-10-19): a restart must clear the WHOLE field the commanded twist
will select, keeping both approach rules (the centre rule and the closest-point rigid-body rule).
Known answers, no simulator.

  * never more permissive than the 2026-09-28 centre rule (randomised, against a copy kept here);
    the stricter fraction is reported (pinned below);
  * a re-triggering departure is blocked: an intrusion in the departure shell (outside the at-rest
    field, inside the moving field) blocks a normal-speed departure in any direction;
  * a creep departure is permitted: at or below both standstill exit thresholds the twist keeps
    the at-rest field, which nesting leaves clear;
  * a spin into an intrusion is blocked, and so is a reversing turn into an obstacle. A spin
    swinging a corner away is permitted at a creep yaw rate; at a normal rate it is blocked when the
    intrusion lies inside the spin's own (sense-symmetric) field, which would re-trigger.
"""
import math
from pathlib import Path

import numpy as np

from pfbnav.config import load_config
from pfbnav.safety.certified import RESTART_ABEAM_TOL, CertifiedLayer
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.world.strata import certified_params_from_config

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
TABLE = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
LAYER = CertifiedLayer(certified_params_from_config(CFG), float(CFG["robot"]["half_length"]),
                       float(CFG["robot"]["half_width"]), TABLE.extent)
P = LAYER.params
BEARINGS = -math.pi + (2 * math.pi / 120) * (np.arange(120) + 0.5)
REST = LAYER.fields.extent(BEARINGS, 0.0, 0.0, at_rest=True)
CREEP = 0.015                       # m/s, safely below the 0.02 exit threshold


def translation_rule_2026_09_28(layer, ranges, bearings, valid, direction) -> bool:
    """The translation-only rule as it stood before 2026-10-19, copied verbatim (the reference)."""
    d = np.asarray(direction, dtype=float).ravel()[:2]
    speed = float(math.hypot(d[0], d[1]))
    if speed < 1e-9:
        return True
    heading = float(math.atan2(d[1], d[0]))
    b, r, v = (np.asarray(bearings, dtype=float), np.asarray(ranges, dtype=float),
               np.asarray(valid, dtype=bool))
    field = layer.fields.extent(b, layer.params.band_top_speeds[0], heading, at_rest=False)
    breaching = ~v | (r < field)
    ahead = np.cos(b - heading) > RESTART_ABEAM_TOL
    leading_clear = not bool(np.any(breaching & ahead))
    reach = np.where(v, r, 1.0)
    v_dot_p = reach * (d[0] * np.cos(b) + d[1] * np.sin(b)) / speed
    away = bool(np.all(v_dot_p[breaching] <= RESTART_ABEAM_TOL * reach[breaching]))
    return leading_clear and away


def test_never_more_permissive_than_the_2026_09_28_centre_rule():
    """Randomised property (ruled): where the centre rule blocks, so does this one. The stricter
    fraction is pinned and reported: 968 / 3000 at seed 11 under the whole-field rule (it was
    85 / 3000 under the leading-field form) -- every extra block a departure whose own field holds an
    intrusion, which would re-trigger."""
    rng = np.random.default_rng(11)
    stricter, blocked = 0, 0
    for _ in range(3000):
        ranges, valid = np.full(120, 5.0), np.ones(120, bool)
        for _k in range(rng.integers(1, 4)):
            i = rng.integers(0, 120)
            ranges[i] = REST[i] + rng.uniform(-0.05, 0.15)
        if rng.random() < 0.1:
            valid[rng.integers(0, 120)] = False
        heading = rng.uniform(-math.pi, math.pi)
        d = 0.05 * np.array([math.cos(heading), math.sin(heading)])
        want = translation_rule_2026_09_28(LAYER, ranges, BEARINGS, valid, d)
        got = LAYER._restart_allowed(ranges, BEARINGS, valid, d)
        assert not (got and not want), "more permissive than the 2026-09-28 rule"
        stricter += want and not got
        blocked += not want
    assert 300 < blocked < 2700
    assert stricter == 968


def _in_the_shell(bearing_deg: float, beyond: float = 0.005):
    """One intrusion just outside the at-rest field (by `beyond`), at a bearing: the departure
    shell -- clear for the at-rest field, inside every moving field."""
    ranges, valid = np.full(120, 5.0), np.ones(120, bool)
    k = int(np.argmin(np.abs(BEARINGS - math.radians(bearing_deg))))
    ranges[k] = REST[k] + beyond
    return ranges, valid


def test_a_re_triggering_departure_is_blocked_in_every_direction():
    ranges, valid = _in_the_shell(0.0)                   # just ahead, outside the at-rest field
    for ang in np.radians(np.arange(0, 360, 30)):
        twist = (0.25 * math.cos(ang), 0.25 * math.sin(ang), 0.0)
        assert not LAYER._restart_allowed(ranges, BEARINGS, valid, twist), math.degrees(ang)


def test_a_creep_departure_is_permitted_in_every_direction():
    ranges, valid = _in_the_shell(0.0)
    for ang in np.radians(np.arange(0, 360, 30)):
        twist = (CREEP * math.cos(ang), CREEP * math.sin(ang), 0.0)
        assert LAYER._restart_allowed(ranges, BEARINGS, valid, twist), math.degrees(ang)


def test_spins_blocked_at_a_normal_rate_permitted_at_a_creep_rate():
    """An intrusion in the shell beside the front-left side: a 1 rad/s spin either way selects a
    moving field that holds it (blocked; it would re-trigger); a spin below the rotational exit
    threshold keeps the at-rest field (permitted, either way)."""
    ranges, valid = _in_the_shell(55.0)
    for w in (1.0, -1.0):
        assert not LAYER._restart_allowed(ranges, BEARINGS, valid, (0.0, 0.0, w))
    slow = 0.9 * P.standstill_yaw_exit
    for w in (slow, -slow):
        assert LAYER._restart_allowed(ranges, BEARINGS, valid, (0.0, 0.0, w))


def test_a_reversing_turn_into_an_obstacle_is_blocked():
    ranges, valid = _in_the_shell(55.0)
    for w in (0.4, -0.4):
        assert not LAYER._restart_allowed(ranges, BEARINGS, valid, (-0.05, 0.0, w))


def test_asking_for_nothing_is_not_blocked_and_nothing_near_permits_everything():
    ranges, valid = _in_the_shell(55.0)
    assert LAYER._restart_allowed(ranges, BEARINGS, valid, (0.0, 0.0, 0.0))
    open_ranges, open_valid = np.full(120, 5.0), np.ones(120, bool)
    for twist in ((0.25, 0.0, 0.0), (0.0, 0.0, 1.0), (-0.1, 0.2, 0.5)):
        assert LAYER._restart_allowed(open_ranges, BEARINGS, open_valid, twist)


def test_an_unobserved_bin_blocks_every_departure_creep_included():
    """Hard rule 5: unobserved space is never free -- it breaches every field."""
    ranges, valid = np.full(120, 5.0), np.ones(120, bool)
    valid[int(np.argmin(np.abs(BEARINGS - math.radians(55.0))))] = False
    for twist in ((0.0, 0.0, 1.0), (CREEP, 0.0, 0.0), (-0.25, 0.0, 0.0)):
        assert not LAYER._restart_allowed(ranges, BEARINGS, valid, twist)
