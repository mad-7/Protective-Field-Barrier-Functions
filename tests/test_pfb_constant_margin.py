"""The CONSTANT-MARGIN comparator (2026-11-24): the PFB's filter with its field replaced by the footprint's
own extent plus one margin, the same in every regime and for every heading and yaw rate. Known answers
on the field, finite-difference gradients, and the monotone effect of the margin on a wall ahead.
The margin values here are test values; the value the comparator is run with is declared in its config."""
import itertools
import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.safety import pfb_field as pf
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.safety.pfb import PFB, REST, PFBConfig

REPO = Path(__file__).resolve().parents[1]
BASE = load_config(REPO / "configs" / "base.yaml")
TABLE = FootprintTable.from_csv(REPO / BASE["robot"]["footprint_table"])
FIELD = pf.PFBField(TABLE.extent, pf.field_params_from_config(BASE))
N = 120
BEARINGS = -math.pi + (np.arange(N) + 0.5) * 2 * math.pi / N
OK = np.ones(N, bool)


def _cfg(margin):
    f = {**BASE["filter"], "field_model": "constant_margin", "constant_margin_m": margin}
    return {**BASE, "filter": f}


def _pfb(margin):
    return PFB(FIELD, PFBConfig.from_config(_cfg(margin), FIELD))


def _wall_ahead(distance, range_max=5.0):
    c = np.cos(BEARINGS)
    with np.errstate(divide="ignore"):
        r = np.where(c > 1e-6, distance / c, range_max)
    return np.minimum(r, range_max)


def test_the_default_config_is_the_pfb_field():
    assert PFBConfig.from_config(BASE, FIELD).constant_margin is None


def test_a_constant_margin_without_a_value_is_refused():
    f = {**BASE["filter"], "field_model": "constant_margin"}
    with pytest.raises(ValueError, match="constant_margin_m"):
        PFBConfig.from_config({**BASE, "filter": f}, FIELD)
    f["constant_margin_m"] = None
    with pytest.raises(ValueError, match="constant_margin_m"):
        PFBConfig.from_config({**BASE, "filter": f}, FIELD)


def test_a_negative_margin_and_an_unknown_model_are_refused():
    with pytest.raises(ValueError, match=">= 0"):
        PFBConfig.from_config(_cfg(-0.01), FIELD)
    f = {**BASE["filter"], "field_model": "constant"}
    with pytest.raises(ValueError, match="field_model"):
        PFBConfig.from_config({**BASE, "filter": f}, FIELD)


def test_the_field_is_the_footprint_extent_plus_the_margin_in_every_regime():
    """Known answer: rho(psi) = e(psi) + m exactly, and the same for every regime, heading and yaw rate."""
    m = 0.30
    pfb = _pfb(m)
    expected = np.asarray(TABLE.extent(BEARINGS), dtype=float) + m
    for regime in (REST, 0, 1, 2):
        for heading, w in ((0.0, 0.0), (1.1, 0.7), (-2.0, 1.5)):
            rho, _d_psi, d_head, d_w = pfb._field(regime, BEARINGS, heading, w)
            assert np.allclose(rho, expected, atol=1e-12)
            assert not d_head.any() and not d_w.any()


def test_the_bearing_slope_is_the_finite_difference_of_the_field():
    pfb = _pfb(0.25)
    rng = np.random.default_rng(3)
    b = rng.uniform(-math.pi, math.pi, 200)
    _rho, d_psi, _h, _w = pfb._field(1, b, 0.0, 0.0)
    eps = 1e-3
    fd = (pfb._field(1, b + eps, 0.0, 0.0)[0] - pfb._field(1, b - eps, 0.0, 0.0)[0]) / (2 * eps)
    assert np.allclose(d_psi, fd, atol=2e-3)


@pytest.mark.parametrize("regime", [REST, 0, 1, 2])
def test_the_row_gradients_are_the_finite_differences(regime):
    pfb = _pfb(0.25)
    rng = np.random.default_rng(regime + 11)
    pts = rng.uniform(0.35, 1.5, 40)[:, None] * np.column_stack(
        [np.cos(b := rng.uniform(-math.pi, math.pi, 40)), np.sin(b)])
    for u in (np.array([0.2, 0.05, 0.3]), np.array([-0.1, 0.15, -0.6])):
        _g, grad = pfb._rows(u, pts, regime, 0.05, 0.4, 0.1)
        eps = 1e-6
        for j in range(3):
            du = np.zeros(3)
            du[j] = eps
            fd = (pfb._rows(u + du, pts, regime, 0.05, 0.4, 0.1)[0]
                  - pfb._rows(u - du, pts, regime, 0.05, 0.4, 0.1)[0]) / (2 * eps)
            assert np.allclose(grad[:, j], fd, atol=1e-4, rtol=1e-4), (regime, u, j)


def test_a_clear_nominal_passes_unchanged():
    pfb = _pfb(0.25)
    u = np.array([0.2, 0.0, 0.0])
    res = pfb.filter(u, u, u, np.full(N, 5.0), BEARINGS, OK)
    assert res.mode == "nominal" and np.allclose(res.u, u)


def test_a_larger_margin_leaves_a_slower_command_at_the_same_wall():
    """The wall at 0.95 m ahead (margins up to 0.15 m stay feasible there): the filtered forward
    speed does not rise with the margin, and the true rows hold at the solution for each margin."""
    u_prev = np.array([0.3, 0.0, 0.0])
    speeds = []
    for m in (0.02, 0.05, 0.10, 0.15):
        pfb = _pfb(m)
        res = pfb.filter([0.35, 0.0, 0.0], u_prev, u_prev, _wall_ahead(0.95), BEARINGS, OK)
        assert not res.infeasible
        pts = _wall_ahead(0.95)[:, None] * np.column_stack([np.cos(BEARINGS), np.sin(BEARINGS)])
        g, _ = pfb._rows(res.u, pts, res.regime, 0.3, 0.0, 0.0)
        rhs = (1 - pfb.c.alpha * pfb.c.dt) * pfb._now(pts, res.regime, 0.0, 0.0)
        assert np.all(g >= rhs - 1e-6)
        speeds.append(float(res.u[0]))
    assert all(x >= y - 1e-9 for x, y in itertools.pairwise(speeds))
    assert speeds[0] > speeds[-1]
