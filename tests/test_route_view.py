"""Known answers for the body-frame route helpers behind the classical input (hard rule 6)."""
import math

import numpy as np
import pytest

from pfbnav.envs.route_view import remaining_route, world_to_body
from pfbnav.planning.lookahead import to_body

PATH = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 3.0]])


def test_a_point_ahead_of_a_robot_facing_plus_y_is_on_its_body_x_axis():
    assert np.allclose(world_to_body([[1.0, 3.0]], (1.0, 1.0, math.pi / 2)), [[2.0, 0.0]])
    assert np.allclose(world_to_body([[0.0, 1.0]], (1.0, 1.0, math.pi / 2)), [[0.0, 1.0]])


def test_it_agrees_with_the_trackers_own_body_transform():
    rng = np.random.default_rng(3)
    for _ in range(50):
        pose = rng.uniform(-5, 5, 3)
        pt = rng.uniform(-5, 5, 2)
        d, s, c = to_body(pt, pose)
        assert np.allclose(world_to_body([pt], pose)[0], [d * c, d * s], atol=1e-12)


def test_the_remaining_route_starts_at_the_projection_and_keeps_every_vertex_beyond():
    assert np.allclose(remaining_route(PATH, 1.0), [[1.0, 0.0], [2.0, 0.0], [2.0, 3.0]])
    assert np.allclose(remaining_route(PATH, 3.0), [[2.0, 1.0], [2.0, 3.0]])
    assert np.allclose(remaining_route(PATH, 2.0), [[2.0, 0.0], [2.0, 3.0]])   # on a vertex
    end = remaining_route(PATH, 9.0)                                             # clipped
    assert end.shape == (2, 2) and np.allclose(end, [[2.0, 3.0], [2.0, 3.0]])
    with pytest.raises(ValueError):
        remaining_route(np.array([[0.0, 0.0]]), 0.0)
