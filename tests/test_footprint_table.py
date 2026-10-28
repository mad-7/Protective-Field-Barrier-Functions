"""Known-answer and finite-difference tests for the footprint table (ruling 3, WP0)."""
import math

import numpy as np
import pytest

from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.footprint import (
    footprint_table_from_polygon,
    radial_extent,
    radial_extents,
)


def _regular_polygon(radius, n=720):
    t = np.linspace(-math.pi, math.pi, n, endpoint=False)
    return np.column_stack([radius * np.cos(t), radius * np.sin(t)])


RECT = np.array([[0.35, 0.2085], [-0.35, 0.2085], [-0.35, -0.2085], [0.35, -0.2085]])


def test_a_circle_has_constant_extent_and_zero_derivative():
    table, margin = footprint_table_from_polygon(_regular_polygon(0.3))
    psi = np.linspace(-math.pi, math.pi, 1001)
    assert table.extent(psi) == pytest.approx(0.3, abs=2e-5)
    assert table.derivative(psi) == pytest.approx(0.0, abs=2e-3)
    assert margin < 3e-5


def test_rectangle_extents_on_the_axes():
    table, margin = footprint_table_from_polygon(RECT)
    assert table.extent(0.0) == pytest.approx(0.35, abs=margin + 1e-9)
    assert table.extent(math.pi / 2) == pytest.approx(0.2085, abs=margin + 1e-9)
    assert table.circumscribed == pytest.approx(math.hypot(0.35, 0.2085), abs=margin + 1e-3)


def test_the_spline_never_falls_below_the_polygon():
    """The safety property: under-stating e(psi) is the unsafe direction."""
    for poly in (RECT, _regular_polygon(0.3, n=12)):
        table, _ = footprint_table_from_polygon(poly)
        psi = np.linspace(-math.pi, math.pi, 20011)   # denser than the build's check
        truth = radial_extents(poly, psi)
        assert np.all(table.extent(psi) >= truth - 1e-9)


def test_rectangle_needs_a_margin_and_it_stays_small():
    # The corners are kinks the spline rounds off; the margin covers the undershoot.
    _, margin = footprint_table_from_polygon(RECT)
    assert 0.0 < margin < 0.005


def test_derivative_matches_finite_differences():
    table, _ = footprint_table_from_polygon(RECT)
    h = 1e-6
    for psi in np.linspace(-3.0, 3.0, 61):
        fd = (table.extent(psi + h) - table.extent(psi - h)) / (2 * h)
        assert table.derivative(psi) == pytest.approx(fd, rel=1e-4, abs=1e-5)


def test_periodic_across_the_wrap():
    table, _ = footprint_table_from_polygon(RECT)
    assert table.extent(math.pi - 1e-9) == pytest.approx(table.extent(-math.pi + 1e-9), abs=1e-7)
    assert table.derivative(math.pi - 1e-9) == pytest.approx(
        table.derivative(-math.pi + 1e-9), abs=1e-5)
    assert table.extent(0.3 + 2 * math.pi) == pytest.approx(table.extent(0.3))


def test_call_returns_the_pair_the_pfb_uses():
    table, _ = footprint_table_from_polygon(RECT)
    e, de = table(0.4)
    assert e == pytest.approx(table.extent(0.4)) and de == pytest.approx(table.derivative(0.4))


def test_csv_round_trip(tmp_path):
    table, margin = footprint_table_from_polygon(RECT)
    path = tmp_path / "table.csv"
    table.to_csv(path, {"source": "unit test", "margin_m": repr(margin)})
    again = FootprintTable.from_csv(path)
    assert again.extents == pytest.approx(table.extents, abs=0)


def test_malformed_tables_are_refused(tmp_path):
    with pytest.raises(ValueError, match="360"):
        FootprintTable(np.ones(359))
    with pytest.raises(ValueError, match="positive"):
        FootprintTable(np.zeros(360))


def test_vectorised_extents_match_the_scalar_routine():
    rng = np.random.default_rng(3)
    for poly in (RECT, _regular_polygon(0.3, n=12),
                 np.array([[0.35, 0.02], [0.2, 0.21], [-0.3, 0.19], [-0.35, -0.05],
                           [0.1, -0.2]])):
        psi = rng.uniform(-math.pi, math.pi, 400)
        scalar = np.array([radial_extent(poly, float(b)) for b in psi])
        assert radial_extents(poly, psi) == pytest.approx(scalar, rel=1e-12, abs=1e-12)
