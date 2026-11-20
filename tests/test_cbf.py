"""Known answers for the generic CBF filter (hard rule 6)."""
import numpy as np
import pytest

from pfbnav.safety.cbf import barrier_floor, filter_command


def test_a_safe_nominal_passes_unchanged():
    u, ok = filter_command(np.array([0.2, 0.1, 0.0]), np.array([[1.0, 0.0, 0.0]]), np.array([0.0]))
    assert ok and np.array_equal(u, [0.2, 0.1, 0.0])


def test_one_row_projects_onto_the_half_space():
    # vx <= 0.1 written as -vx >= -0.1; nominal 0.4 -> 0.1, other components untouched.
    u, ok = filter_command(np.array([0.4, 0.3, 0.0]), np.array([[-1.0, 0.0, 0.0]]),
                           np.array([-0.1]))
    assert ok and np.allclose(u, [0.1, 0.3, 0.0])


def test_a_slanted_row_gives_the_orthogonal_projection():
    # x + y >= 1 from the origin: the closest point is (0.5, 0.5).
    u, ok = filter_command(np.zeros(2), np.array([[1.0, 1.0]]), np.array([1.0]))
    assert ok and np.allclose(u, [0.5, 0.5])


def test_weights_move_the_correction_to_the_cheaper_axis():
    # x + y >= 1 with y twice as costly (w_y = 4): the correction goes mostly to x.
    u, _ = filter_command(np.zeros(2), np.array([[1.0, 1.0]]), np.array([1.0]),
                          weights=np.array([1.0, 4.0]))
    assert np.allclose(u, [0.8, 0.2]) and np.isclose(u[0] + u[1], 1.0)


def test_two_rows_meet_at_their_corner():
    # x >= 1 and y >= 1 from the origin -> (1, 1).
    u, ok = filter_command(np.zeros(2), np.eye(2), np.array([1.0, 1.0]))
    assert ok and np.allclose(u, [1.0, 1.0])


def test_the_result_satisfies_every_row_and_is_not_beaten_by_a_random_feasible_point():
    rng = np.random.default_rng(0)
    rows = np.array([[1.0, 0.0], [0.0, 1.0], [-1.0, -1.0]])
    bounds = np.array([0.2, -0.5, -1.5])
    u_nom = np.array([-1.0, 2.0])
    u, ok = filter_command(u_nom, rows, bounds)
    assert ok and np.all(rows @ u >= bounds - 1e-9)
    for _ in range(2000):
        p = rng.uniform(-3, 3, 2)
        if np.all(rows @ p >= bounds):
            assert np.sum((u - u_nom) ** 2) <= np.sum((p - u_nom) ** 2) + 1e-9


def test_contradictory_rows_are_reported_infeasible_not_papered_over():
    u, ok = filter_command(np.zeros(1), np.array([[1.0], [-1.0]]), np.array([1.0, 0.0]))
    assert not ok and np.array_equal(u, [0.0])


def test_the_discrete_condition_floor():
    assert barrier_floor(1.0, alpha=1.0, dt=0.05) == pytest.approx(0.95)
    assert barrier_floor(-0.3, alpha=1.0, dt=0.05) == 0.0            # unsafe now: must reach zero
    with pytest.raises(ValueError):
        barrier_floor(1.0, alpha=0.0, dt=0.05)
    with pytest.raises(ValueError):
        barrier_floor(1.0, alpha=30.0, dt=0.05)


def test_bad_shapes_are_refused():
    with pytest.raises(ValueError):
        filter_command(np.zeros(2), np.ones((1, 3)), np.zeros(1))
