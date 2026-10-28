"""Known-answer tests for depth pixels -> robot-frame points (hard rule 6).

The first scanner run put a wall in the wrong place (2026-09-21). These pin the
conversion on synthetic sensors whose truth is known, under every convention, so the
simulator's own convention can be identified by which one reconstructs a known wall.
"""
import math

import numpy as np
import pytest

from pfbnav.sim.depth_geometry import (
    apply_matrix,
    cast_to_wall,
    matrix_from_pose,
    pixel_tangents,
    sensor_points,
    wall_residuals,
)

FOV = math.radians(120.0)
RES = 80
CONVENTIONS = [(m, e) for m in (False, True) for e in (False, True)]


def reconstruct(depth, matrix, *, mirrored, euclidean, far):
    hit = depth < far - 1e-9
    pts = sensor_points(depth, FOV, mirrored=mirrored, euclidean=euclidean)
    return apply_matrix(matrix, pts)[hit]


# ----------------------------------------------------------------------
# Pixel directions
# ----------------------------------------------------------------------
def test_pixel_tangents_span_the_field_of_view_symmetrically():
    t = pixel_tangents(RES, FOV, mirrored=False)
    assert t[0] == pytest.approx(-t[-1])
    # Edges of the outermost pixels, half a pixel in from the image border.
    assert abs(t[0]) < math.tan(0.5 * FOV)
    assert np.all(np.diff(t) > 0)


def test_mirroring_reverses_the_pixel_order():
    a = pixel_tangents(RES, FOV, mirrored=False)
    b = pixel_tangents(RES, FOV, mirrored=True)
    assert b == pytest.approx(a[::-1])


def test_pixel_tangents_reject_nonsense():
    with pytest.raises(ValueError):
        pixel_tangents(0, FOV, mirrored=False)
    with pytest.raises(ValueError):
        pixel_tangents(RES, math.pi, mirrored=False)


# ----------------------------------------------------------------------
# Depth conventions
# ----------------------------------------------------------------------
def test_perpendicular_and_euclidean_agree_on_the_axis():
    # The centre pixel is (almost) on the axis, where the two meanings coincide.
    d = np.full(RES + 1, 2.0)                 # odd count: a pixel sits on the axis
    fov = FOV
    a = sensor_points(d, fov, mirrored=False, euclidean=False)
    b = sensor_points(d, fov, mirrored=False, euclidean=True)
    mid = RES // 2
    assert a[mid] == pytest.approx(b[mid])


def test_euclidean_depth_is_the_distance_from_the_sensor():
    d = np.full(RES, 3.0)
    p = sensor_points(d, FOV, mirrored=False, euclidean=True)
    assert np.linalg.norm(p, axis=1) == pytest.approx(np.full(RES, 3.0))


def test_perpendicular_depth_is_the_distance_along_the_axis():
    d = np.full(RES, 3.0)
    p = sensor_points(d, FOV, mirrored=False, euclidean=False)
    assert p[:, 2] == pytest.approx(np.full(RES, 3.0))


# ----------------------------------------------------------------------
# Matrices
# ----------------------------------------------------------------------
def test_matrix_from_pose_points_the_sensor_along_its_yaw():
    m = matrix_from_pose(math.radians(30.0), [0.3, 0.0, 0.2])
    ahead = apply_matrix(m, [[0.0, 0.0, 1.0]])[0]
    origin = apply_matrix(m, [[0.0, 0.0, 0.0]])[0]
    d = ahead - origin
    assert math.degrees(math.atan2(d[1], d[0])) == pytest.approx(30.0)
    assert origin == pytest.approx([0.3, 0.0, 0.2])


def test_apply_matrix_rejects_the_wrong_shape():
    with pytest.raises(ValueError):
        apply_matrix([1.0] * 9, [[0.0, 0.0, 0.0]])


# ----------------------------------------------------------------------
# The identification the scanner check relies on
# ----------------------------------------------------------------------
@pytest.mark.parametrize("truth", CONVENTIONS)
def test_the_true_convention_reconstructs_the_wall_exactly(truth):
    mirrored, euclidean = truth
    # A front-left sensor like the pods': axis at +30 deg, 0.3 m forward of the origin.
    m = matrix_from_pose(math.radians(30.0), [0.3, 0.0, 0.2])
    far = 5.0
    depth = cast_to_wall(FOV, RES, m, math.radians(90.0), 1.0,
                         mirrored=mirrored, euclidean=euclidean, far=far)
    pts = reconstruct(depth, m, mirrored=mirrored, euclidean=euclidean, far=far)
    assert pts.shape[0] > 10
    assert np.abs(wall_residuals(pts, math.radians(90.0), 1.0)).max() < 1e-9


@pytest.mark.parametrize("truth", CONVENTIONS)
def test_every_wrong_convention_puts_the_wall_elsewhere(truth):
    m = matrix_from_pose(math.radians(30.0), [0.3, 0.0, 0.2])
    far = 5.0
    depth = cast_to_wall(FOV, RES, m, math.radians(90.0), 1.0,
                         mirrored=truth[0], euclidean=truth[1], far=far)
    for guess in CONVENTIONS:
        if guess == truth:
            continue
        pts = reconstruct(depth, m, mirrored=guess[0], euclidean=guess[1], far=far)
        worst = np.abs(wall_residuals(pts, math.radians(90.0), 1.0)).max()
        assert worst > 0.05, f"{guess} cannot be told apart from {truth}"


def test_a_mirrored_reading_tilts_a_wall_by_twice_the_sensor_axis():
    # The signature seen on 2026-09-21: axis at 30 deg, wall at 0 deg, reconstruction at
    # ~60 deg. A reflection across the axis turns a line at angle a into one at 2*axis - a.
    m = matrix_from_pose(math.radians(30.0), [0.3, 0.0, 0.2])
    far = 5.0
    depth = cast_to_wall(FOV, RES, m, math.radians(90.0), 1.0,
                         mirrored=True, euclidean=False, far=far)
    wrong = reconstruct(depth, m, mirrored=False, euclidean=False, far=far)
    slope = np.polyfit(wrong[:, 0], wrong[:, 1], 1)[0]
    assert math.degrees(math.atan(slope)) == pytest.approx(60.0, abs=0.5)


def test_wall_residuals_known_answer():
    pts = np.array([[0.0, 1.0, 0.0], [2.0, 1.0, 0.0], [-1.0, 1.5, 0.0]])
    r = wall_residuals(pts, math.radians(90.0), 1.0)
    assert r == pytest.approx([0.0, 0.0, 0.5])
