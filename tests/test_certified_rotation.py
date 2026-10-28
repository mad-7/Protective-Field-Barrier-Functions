"""The rotation sweep of the protective field (ruled 2026-10-16; since A20, 2026-10-18, the
interval of headings REACHABLE within T). Known answers, no simulator.

The field contains the footprint swept through every heading reachable from the yaw rate before the
body is at rest -- accelerating at alpha_max for T, then braking -- with the translation reach and
the margins on top (`tests/test_reach.py` covers containment of whole trajectories).
  * the fields are pinned against stored A21 fields (`tests/data/field_golden_A21_20261019.npz`;
    the A20 file, `field_golden_A20_20261018.npz`, is kept -- A21's nesting terms changed them),
    so a later change that moves them fails here until it is ruled. The 2026-10-16 bit-identity
    against the rotation-blind field (`field_golden_20261016.npz`, kept) ended with A20, which
    changes the field everywhere by design;
  * the rotation sweep matches a brute-force union of rotated footprints, erring only larger;
  * the yaw braking it assumes is the limiter's own symmetric `alpha_max` (one config value);
  * the certified layer uses the estimated yaw rate: a rotating robot stops where a still one
    does not.
(Monotonicity in |omega| and in speed is in `tests/test_certified_layer.py`.)
"""
import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.eval.raycast import bin_bearings
from pfbnav.safety.certified import CertifiedLayer, FieldGeometry, yaw_excursion, yaw_interval
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.robot import ActuationLimits, limit_acceleration
from pfbnav.world.strata import certified_params_from_config

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
PARAMS = certified_params_from_config(CFG)
TABLE = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
FIELDS = FieldGeometry(TABLE.extent, PARAMS)
GOLDEN = np.load(REPO / "tests" / "data" / "field_golden_A21_20261019.npz")
HEADINGS = (0.0, 0.7, -1.3, 2.4, math.pi)


@pytest.mark.parametrize("name", ["b120", "b720"])
def test_the_fields_are_the_stored_a21_fields(name):
    """Every band, five headings, protective and warning, at yaw rates 0 and 0.6 rad/s, and at
    rest: bit-identical to the fields stored when A21 was adopted (2026-10-19)."""
    b = GOLDEN[f"{name}_bearings"]
    for bi, top in enumerate(PARAMS.band_top_speeds):
        for hi, h in enumerate(HEADINGS):
            for w in (False, True):
                for ri, rate in enumerate((0.0, 0.6)):
                    want = GOLDEN[f"{name}_band{bi}_h{hi}_w{int(w)}_r{ri}"]
                    assert np.array_equal(FIELDS.extent(b, top, h, False, warning=w,
                                                        yaw_rate=rate), want)
    for w in (False, True):
        for ri, rate in enumerate((0.0, 0.6)):
            assert np.array_equal(FIELDS.extent(b, 0.0, 0.0, True, warning=w, yaw_rate=rate),
                                  GOLDEN[f"{name}_rest_w{int(w)}_r{ri}"])


def test_the_reachable_yaw_excursion_known_answers():
    """T = 0.15 s, alpha_max = alpha_brake = 1.4 rad/s^2, yaw cap 1.5 rad/s (A20)."""
    assert PARAMS.reach_time == pytest.approx(0.15)
    a = PARAMS.alpha_max
    assert yaw_excursion(0.0, PARAMS) == pytest.approx(a * 0.15 ** 2)        # from rest: alpha T^2
    w_T = 1.0 + a * 0.15
    assert yaw_excursion(1.0, PARAMS) == pytest.approx(1.0 * 0.15 + 0.5 * a * 0.0225
                                                       + w_T ** 2 / (2 * a))
    assert yaw_excursion(1.5, PARAMS) == pytest.approx(1.5 * 0.15 + 1.5 ** 2 / (2 * a))  # capped
    assert yaw_interval(-1.0, PARAMS) == pytest.approx((-yaw_excursion(1.0, PARAMS),
                                                         yaw_excursion(1.0, PARAMS)))


def test_the_yaw_braking_assumed_is_the_limiters_own_symmetric_bound():
    """By construction (ruled 2026-10-16): one config value, `actuation.alpha_max`, sizes the
    field and bounds the limiter's yaw change both ways, so a stop's yaw ramps down at the rate
    the field assumed."""
    assert PARAMS.alpha_brake == float(CFG["actuation"]["alpha_max"])
    limits = ActuationLimits.from_config(CFG)
    dt = float(CFG["sim"]["dt"])
    w, ws = 1.5, []
    while w > 0.0:                                  # a stop: the command is zeroed
        w = float(limit_acceleration(np.zeros(3), np.array([0.0, 0.0, w]), dt, limits)[2])
        ws.append(w)
    steps = np.diff([1.5, *ws])
    assert np.allclose(steps[:-1], -limits.alpha_max * dt)       # decelerates at alpha_max
    up = limit_acceleration(np.array([0.0, 0.0, 1.5]), np.zeros(3), dt, limits)[2]
    assert up == pytest.approx(limits.alpha_max * dt)             # and accelerates the same


@pytest.mark.parametrize("rate", [0.3, -0.8, 1.5])
def test_the_swept_support_is_exactly_the_union_of_rotated_footprints(rate):
    """Against a brute force over finely sampled angles: never below it (it is the exact union),
    and above it by no more than the brute force's own sampling can miss."""
    lo, hi = yaw_interval(rate, PARAMS)
    radius = float(np.max(TABLE.extent(np.linspace(-math.pi, math.pi, 2880, endpoint=False))))
    coarse = np.linspace(-3.0, 3.0, 37)
    cases = ((FIELDS.normal_angles, 0.0, hi, FIELDS.swept_support(FIELDS.normal_angles, hi)),
             (coarse, 0.0, hi, FIELDS.swept_support(coarse, hi)),
             (coarse, lo, hi, FIELDS.swept_interval(coarse, lo, hi)))
    for n, a, b, swept in cases:
        phis = np.linspace(a, b, 600)
        brute = np.max([FIELDS.footprint_support(n - phi) for phi in phis], axis=0)
        assert np.all(swept >= brute - 1e-9), "the sweep must contain every rotated footprint"
        sampling = radius * abs(phis[1] - phis[0]) / 2
        assert np.all(swept <= brute + sampling + 1e-9), "and be no larger than the union"


def test_the_field_is_continuous_in_the_yaw_rate():
    """The regression for the bug the simulator control found (2026-10-16): a noise-level yaw
    rate must move the field by a noise-level amount, not by a fixed pad."""
    b = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    still = FIELDS.extent(b, 0.0, 0.0, True)
    radius = float(np.max(TABLE.extent(b)))
    for w in (1e-6, -1e-5, 1e-4):
        drift = float(np.max(np.abs(FIELDS.extent(b, 0.0, 0.0, True, yaw_rate=w) - still)))
        grown = yaw_excursion(abs(w), PARAMS) - yaw_excursion(0.0, PARAMS)
        assert drift <= radius * grown + 1e-9, (w, drift)


def test_the_field_contains_every_rotated_footprint_boundary_point():
    """Containment, read radially: every boundary point of the footprint rotated through the
    sweep lies inside the at-rest field at that yaw rate (the margins included)."""
    rate = 1.2
    psi = np.linspace(-math.pi, math.pi, 1441)
    e = TABLE.extent(psi)
    lo, hi = yaw_interval(rate, PARAMS)
    for phi in np.linspace(lo, hi, 25):
        bearings = np.array([math.remainder(p + phi, 2 * math.pi) for p in psi])
        field = FIELDS.extent(bearings, 0.0, 0.0, True, yaw_rate=rate)
        assert np.all(e + PARAMS.lateral_margin <= field + 1e-9)


def test_a_rotating_robot_stops_where_a_still_one_does_not():
    """The layer reads the yaw rate from the estimated velocity. An obstacle just outside the
    at-rest field at the footprint's front-left diagonal: no stop at rest without rotation, a
    stop when the robot turns toward it."""
    b = bin_bearings(int(CFG["scanners"]["filter_bins"]))
    still = FIELDS.extent(b, 0.0, 0.0, True)
    turning = FIELDS.extent(b, 0.0, 0.0, True, yaw_rate=1.0)
    k = int(np.argmax(turning - still))
    assert turning[k] - still[k] > 0.01
    ranges = np.full(b.shape, 5.0)
    ranges[k] = 0.5 * (still[k] + turning[k])
    valid = np.ones(b.shape, bool)

    def stops(yaw_rate):
        layer = CertifiedLayer(PARAMS, float(CFG["robot"]["half_length"]),
                               float(CFG["robot"]["half_width"]), TABLE.extent)
        return layer.step(0.0, (0.0, 0.0, yaw_rate), ranges, b, valid)["stop_triggered"]

    assert not stops(0.0)
    assert stops(1.0)
