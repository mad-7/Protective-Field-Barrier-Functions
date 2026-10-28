"""Known-answer tests for G0's case placement on ASYMMETRIC geometry (hard rule 6).

The supervisor's ruling of 2026-09-22: place the corner case at the envelope's outermost
bearing and assert that each positive case strikes the shape it was placed for. Symmetric
test geometry is what hid both the position-blind validity logic and the magnitude-based
placement, so every case here is asymmetric: a housing on one side only, at one height.
"""
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "gate_g0_contact.py"


def _load():
    spec = importlib.util.spec_from_file_location("gate_g0_contact", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    # Registered before exec: the module defines a dataclass, and @dataclass looks its
    # owning module up in sys.modules.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


M = _load()


def box(x0, x1, y0, y1, z0, z1, step=0.005):
    """A box sampled densely, as `shape_points` samples a real mesh."""
    def axis(a, b):
        return np.linspace(a, b, max(2, round((b - a) / step) + 1))
    g = np.meshgrid(axis(x0, x1), axis(y0, y1), axis(z0, z1), indexing="ij")
    return np.column_stack([a.ravel() for a in g])


def test_densify_fills_a_face_a_narrow_corridor_would_otherwise_pass_between():
    # A 1 m square face, as a primitive gives it: four corners, two triangles.
    v = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]])
    tris = np.array([[0, 1, 2], [0, 2, 3]])
    dense = M.densify(v, tris, 0.01)
    mid = dense[(np.abs(dense[:, 0] - 0.5) < 0.02) & (np.abs(dense[:, 1] - 0.5) < 0.02)]
    assert len(mid) > 0            # the middle of the face is now represented
    assert np.allclose(dense[:, 2], 0.0)
    assert dense[:, 0].max() <= 1.0 + 1e-12 and dense[:, 1].max() <= 1.0 + 1e-12


def test_densify_refuses_a_nonpositive_step():
    with pytest.raises(ValueError):
        M.densify(np.zeros((3, 3)), np.array([[0, 1, 2]]), 0.0)


# A robot 0.30 m long and 0.20 m wide, with a housing on the RIGHT (-y) front corner only,
# mounted high (0.10-0.26 m), and a low wheel on each side (0 - 0.10 m).
ROBOT = {
    "chassis": box(-0.30, 0.30, -0.18, 0.18, 0.02, 0.20),
    "housing_front_right": box(0.20, 0.34, -0.22, -0.08, 0.10, 0.26),
    "wheel_front_right": box(0.15, 0.29, -0.21, -0.19, 0.00, 0.10),
    "wheel_front_left": box(0.15, 0.29, 0.19, 0.21, 0.00, 0.10),
}


def test_shape_support_is_the_furthest_corner_along_the_direction():
    assert M.shape_support(ROBOT["housing_front_right"], (1.0, 0.0)) == pytest.approx(0.34)
    assert M.shape_support(ROBOT["housing_front_right"], (0.0, -1.0)) == pytest.approx(0.22)


def test_outermost_bearing_finds_the_asymmetric_corner():
    # An envelope with one bump, at -35 deg: the argmax must be there, not at 0 or +35.
    def extent(psi):
        p = np.asarray(psi, dtype=float)
        return 0.30 + 0.05 * np.exp(-((np.degrees(p) + 35.0) / 4.0) ** 2)

    assert math.degrees(M.outermost_bearing(extent)) == pytest.approx(-35.0, abs=0.5)


def test_outermost_bearing_stays_within_the_forward_window():
    def extent(psi):
        p = np.asarray(psi, dtype=float)
        return 0.30 + 0.10 * np.exp(-((np.degrees(p) - 170.0) / 4.0) ** 2)

    assert abs(math.degrees(M.outermost_bearing(extent))) <= 90.0


def test_a_post_on_the_right_is_met_by_the_right_hand_housing():
    name, support = M.first_to_strike(ROBOT, (0.50, -0.20, 0.0), (0.54, -0.16, 0.40),
                                      (1.0, 0.0, 0.0))
    assert name == "housing_front_right"
    assert support == pytest.approx(0.34)


def test_the_same_post_mirrored_is_met_by_something_else_entirely():
    # The magnitude-based placement of the old script mirrored the corner's offset and so
    # would have put the post here, on the side with no housing, and still called the case
    # the housing's corner. On this robot the chassis corner meets it instead.
    name, support = M.first_to_strike(ROBOT, (0.50, 0.16, 0.0), (0.54, 0.20, 0.40),
                                      (1.0, 0.0, 0.0))
    assert name == "chassis" and support == pytest.approx(0.30)


def test_a_post_outside_the_chassis_on_the_bare_side_is_met_by_the_wheel():
    name, _ = M.first_to_strike(ROBOT, (0.50, 0.19, 0.0), (0.54, 0.21, 0.40),
                                (1.0, 0.0, 0.0))
    assert name == "wheel_front_left"


def test_a_low_slab_can_only_be_struck_by_a_wheel():
    name, _ = M.first_to_strike(ROBOT, (0.10, 0.19, 0.0), (0.60, 0.25, 0.012),
                                (0.0, 1.0, 0.0))
    assert name == "wheel_front_left"


def test_a_tall_wall_across_the_front_is_struck_by_the_housing_not_the_chassis():
    name, support = M.first_to_strike(ROBOT, (0.50, -0.60, 0.0), (0.55, 0.60, 0.40),
                                      (1.0, 0.0, 0.0))
    assert name == "housing_front_right" and support == pytest.approx(0.34)


def test_driving_backwards_uses_the_rear_support():
    name, support = M.first_to_strike(ROBOT, (-0.55, -0.60, 0.0), (-0.50, 0.60, 0.40),
                                      (-1.0, 0.0, 0.0))
    assert name == "chassis" and support == pytest.approx(0.30)


def test_a_diagonal_approach_meets_the_corner_that_protrudes_along_it():
    # The corner case: approached along its own bearing, the housing leads; the same post
    # driven into along +x is met by the chassis instead (measured on the real robot).
    diagonal = (math.cos(math.radians(-30.0)), math.sin(math.radians(-30.0)), 0.0)
    corner = np.array([0.30, -0.17])
    centre = corner + 0.20 * np.array(diagonal[:2])
    lo = (centre[0] - 0.02, centre[1] - 0.02, 0.0)
    hi = (centre[0] + 0.02, centre[1] + 0.02, 0.40)
    name, _ = M.first_to_strike(ROBOT, lo, hi, diagonal)
    assert name == "housing_front_right"


def test_a_vertical_or_zero_drive_is_refused():
    with pytest.raises(ValueError, match="horizontal"):
        M.first_to_strike(ROBOT, (0.5, -0.2, 0.0), (0.54, -0.16, 0.4), (0.0, 0.0, 1.0))
    with pytest.raises(ValueError, match="horizontal"):
        M.first_to_strike(ROBOT, (0.5, -0.2, 0.0), (0.54, -0.16, 0.4), (0.0, 0.0, 0.0))


def test_an_obstacle_no_shape_can_reach_is_refused():
    with pytest.raises(ValueError, match="beyond the robot"):
        M.first_to_strike(ROBOT, (0.5, 3.0, 0.0), (0.54, 3.1, 0.4), (1.0, 0.0, 0.0))


def test_obstacle_box_is_the_case_centre_plus_or_minus_half_its_size():
    case = M.Case("c", (0.04, 0.04, 0.40), (0.50, -0.20, 0.20), (1.0, 0.0, 0.0), 0.0,
                  True, 0.30, "note")
    lo, hi = M.obstacle_box(case)
    assert lo == pytest.approx((0.48, -0.22, 0.0))
    assert hi == pytest.approx((0.52, -0.18, 0.40))


def test_expected_shapes_attaches_the_struck_shape_to_positive_cases_only():
    positive = M.Case("post", (0.04, 0.04, 0.40), (0.50, -0.20, 0.20), (1.0, 0.0, 0.0),
                      0.0, True, 0.30, "note")
    negative = M.Case("pass_by", (2.0, 0.05, 0.40), (0.0, 0.40, 0.20), (1.0, 0.0, 0.0),
                      0.0, False, 1.0, "note")
    out = M.expected_shapes([positive, negative], ROBOT)
    assert out[0].expect_shape == "housing_front_right"
    assert out[1].expect_shape is None


def test_a_case_that_strikes_another_shape_fails_even_with_a_perfect_lag():
    case = M.Case("post", (0.04, 0.04, 0.40), (0.50, -0.20, 0.20), (1.0, 0.0, 0.0), 0.0,
                  True, 0.30, "note", expect_shape="housing_front_right")
    rows = [{"contact": 0, "hit": "", "part": "", "by_handle": ""},
            {"contact": 1, "hit": "wheel_front_right", "part": "wheel",
             "by_handle": "wheel_front_right"}]
    out = M.summarise_case(case, [False, True], [0.01, -0.001], rows)
    assert out["passed"] is False
    assert "placed to strike housing_front_right" in out["reason"]
    assert out["struck"] == "wheel_front_right"


def test_a_case_that_strikes_its_own_shape_is_not_failed_here():
    case = M.Case("post", (0.04, 0.04, 0.40), (0.50, -0.20, 0.20), (1.0, 0.0, 0.0), 0.0,
                  True, 0.30, "note", expect_shape="housing_front_right")
    rows = [{"contact": 0, "hit": "", "part": "", "by_handle": ""},
            {"contact": 1, "hit": "housing_front_right", "part": "housing",
             "by_handle": "housing_front_right|chassis"}]
    out = M.summarise_case(case, [False, True], [0.01, -0.001], rows)
    assert "passed" not in out            # the criteria decide, in verdict()
    assert out["expect_shape"] == "housing_front_right"


# ------------------------------------------- case six, and preregistration deciding the list
def test_a_case_not_yet_preregistered_is_defined_but_not_run():
    """Adding a case to a gate is an amendment, not a code change (supervisor 2026-09-22,
    again 2026-09-26). Case six exists in the script; until `gates.yaml` declares it, it must
    not run, or its result would look like part of a preregistered gate."""
    g0 = _load()
    cases = [g0.Case("a", (1, 1, 1), (1, 0, 0), (1, 0, 0), 0.0, True, 0.1, ""),
             g0.Case("six", (1, 1, 1), (1, 0, 0), (1, 0, 0), 0.0, True, 0.1, ""),
             g0.Case("neg", (1, 1, 1), (1, 0, 0), (1, 0, 0), 0.0, False, 0.1, "")]
    run, pending = g0.declared_cases(
        cases, {"positive_cases": ["a"], "negative_case": "neg"})
    assert [c.name for c in run] == ["a", "neg"]
    assert pending == ["six"]


def test_a_declared_case_the_script_cannot_build_is_fatal():
    """The other direction is worse: passing the buildable cases while one declared case never
    ran would be a false verdict, so it exits rather than reporting PASS."""
    g0 = _load()
    cases = [g0.Case("a", (1, 1, 1), (1, 0, 0), (1, 0, 0), 0.0, True, 0.1, ""),
             g0.Case("neg", (1, 1, 1), (1, 0, 0), (1, 0, 0), 0.0, False, 0.1, "")]
    with pytest.raises(SystemExit, match="cannot build"):
        g0.declared_cases(cases, {"positive_cases": ["a", "absent"], "negative_case": "neg"})


def test_the_raised_slab_clears_the_chassis_and_sits_inside_the_housings_span():
    """Known answer from the S300 read-back (2026-09-22): wheel tops 0.0999 m, chassis top
    0.1764 m, housings z 0.104-0.257 m, scan plane 0.2207 m. The slab's underside must clear
    the chassis -- otherwise the chassis is struck and the case is not about the housing -- and
    must sit low enough that a housing reaches it."""
    g0 = _load()
    chassis_top, housing_top, scan_plane = 0.1764, 0.257, 0.2207
    assert g0.HOUSING_SLAB_BOTTOM_M > chassis_top
    assert g0.HOUSING_SLAB_BOTTOM_M < housing_top
    assert g0.HOUSING_SLAB_TOP_M > scan_plane, (
        "the slab must cross the scan plane, so this case is SEEN, unlike wheel_edge")


def test_case_six_is_driven_abeam_and_raised():
    from pfbnav.config import load_config
    g0 = _load()
    geom = g0.ChassisGeometry.from_config(
        load_config(Path(__file__).resolve().parents[1] / "configs" / "base.yaml"))
    case = next(c for c in g0.build_cases(geom, (0.3, 0.1), 0.3)
                if c.name == "raised_housing")
    assert case.drive_body == (0.0, 1.0, 0.0) and case.expect_contact
    low, high = g0.obstacle_box(case)
    assert low[2] == pytest.approx(g0.HOUSING_SLAB_BOTTOM_M)
    assert high[2] == pytest.approx(g0.HOUSING_SLAB_TOP_M)
