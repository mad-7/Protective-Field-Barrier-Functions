"""The PFB's QP (WP2 step 2, ruled 2026-10-23): known answers, before any closed loop. Synthetic scans
(walls as sector-minimum ranges), the configured footprint and parameters."""
import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.safety import pfb_field as pf
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.safety.pfb import PFB, REST, PFBConfig

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
FIELD = pf.PFBField(FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"]).extent,
                    pf.field_params_from_config(CFG))
CONF = PFBConfig.from_config(CFG, FIELD)
N = 120
BEARINGS = -math.pi + (np.arange(N) + 0.5) * 2 * math.pi / N
OK = np.ones(N, bool)


def _pfb():
    return PFB(FIELD, CONF)


def _wall_ahead(distance, range_max=5.0):
    """A wall across the way at `distance` (m) ahead: ranges distance / cos(psi) in front."""
    c = np.cos(BEARINGS)
    with np.errstate(divide="ignore"):
        r = np.where(c > 1e-6, distance / c, range_max)
    return np.minimum(r, range_max)


def _open():
    return np.full(N, 5.0)


def _limits_hold(pfb, u, u_prev):
    c = pfb.c
    assert abs(u[0]) + abs(u[1]) + c.lever * abs(u[2]) <= c.budget + 1e-6
    assert abs(u[0]) <= c.v_max + 1e-9 and abs(u[1]) <= c.vy_max + 1e-9
    assert abs(u[2]) <= c.w_max + 1e-9
    step = math.hypot(*(u[:2] - u_prev[:2]))
    slower = math.hypot(*u[:2]) <= math.hypot(*u_prev[:2]) + 1e-9
    # the limiter's two cases: a_max dt, or a_decel dt for a step that reduces speed
    assert step <= c.a_max * c.dt + 1e-6 or (slower and step <= c.a_decel * c.dt + 1e-6)
    assert abs(u[2] - u_prev[2]) <= c.alpha_max * c.dt + 1e-9


# --------------------------------------------------------------------------- gradients
@pytest.mark.parametrize("regime", [REST, 0, 1, 2])
def test_the_row_gradients_are_the_finite_differences(regime):
    """Q3: analytic gradients verified by finite differences, every row, every command component."""
    pfb = _pfb()
    rng = np.random.default_rng(regime + 5)
    pts = rng.uniform(0.35, 1.5, 40)[:, None] * np.column_stack(
        [np.cos(b := rng.uniform(-math.pi, math.pi, 40)), np.sin(b)])
    for u in (np.array([0.2, 0.05, 0.3]), np.array([-0.1, 0.15, -0.6]),
              np.array([0.012, 0.0, 0.03])):
        _g, grad = pfb._rows(u, pts, regime, 0.05, 0.4, 0.1)
        eps = 1e-6
        for j in range(3):
            du = np.zeros(3)
            du[j] = eps
            fd = (pfb._rows(u + du, pts, regime, 0.05, 0.4, 0.1)[0]
                  - pfb._rows(u - du, pts, regime, 0.05, 0.4, 0.1)[0]) / (2 * eps)
            assert np.allclose(grad[:, j], fd, atol=1e-4, rtol=1e-4), (regime, u, j)


# --------------------------------------------------------------------------- the QP
def test_a_clear_nominal_passes_unchanged():
    res = _pfb().filter([0.2, 0.0, 0.0], [0.2, 0.0, 0.0], [0.2, 0.0, 0.0], _open(), BEARINGS, OK)
    assert res.mode == "nominal" and np.array_equal(res.u, [0.2, 0.0, 0.0]) and res.solves == 0


def test_a_wall_ahead_slows_the_robot_and_every_true_row_holds():
    pfb = _pfb()
    u_prev = np.array([0.3, 0.0, 0.0])
    res = pfb.filter([0.35, 0.0, 0.0], u_prev, u_prev, _wall_ahead(0.95), BEARINGS, OK)
    assert res.mode == "filtered" and not res.infeasible
    assert res.u[0] < 0.35
    _limits_hold(pfb, res.u, u_prev)
    # the true rows of the chosen regime hold at the solution
    pts = _wall_ahead(0.95)[:, None] * np.column_stack([np.cos(BEARINGS), np.sin(BEARINGS)])
    g, _ = pfb._rows(res.u, pts, res.regime, 0.3, 0.0, 0.0)
    rhs = (1 - CONF.alpha * CONF.dt) * pfb._now(pts, res.regime, 0.0, 0.0)
    assert np.all(g >= rhs - 1e-6)


def test_sliding_past_is_preferred_to_stopping():
    """A wall ahead-left only: the minimal modification keeps speed and leans away, rather than
    braking (it is the least change in the W norm)."""
    r = _open()
    sector = (BEARINGS > 0.2) & (BEARINGS < 0.9)
    r[sector] = 0.75
    u_prev = np.array([0.2, 0.0, 0.0])
    res = _pfb().filter([0.22, 0.0, 0.0], u_prev, u_prev, r, BEARINGS, OK)
    assert not res.infeasible and res.u[0] > 0.0


def test_an_unobserved_bin_falls_back_to_maximum_braking():
    """The shared convention (ruled 2026-10-27): the unobserved bin enters the rows as an obstacle
    on the footprint's boundary. It lies inside every regime's field and cannot leave it in one
    step, so the step is infeasible and falls back to maximum braking -- the command it was before
    the convention, still labelled "unobserved" -- now after the QPs were tried, and counted."""
    pfb = _pfb()
    blind = OK.copy()
    blind[10] = False
    u_prev = np.array([0.3, 0.1, 0.4])
    res = pfb.filter([0.3, 0.1, 0.4], u_prev, u_prev, _open(), BEARINGS, blind)
    assert res.infeasible and res.reason == "unobserved" and res.mode == "fallback"
    assert res.unobserved == 1 and res.solves > 0
    s0, s1 = math.hypot(0.3, 0.1), math.hypot(*res.u[:2])
    assert s1 == pytest.approx(s0 - CONF.a_decel * CONF.dt)
    assert res.u[2] == pytest.approx(0.4 - CONF.alpha_max * CONF.dt)
    # parallel to the previous command: braking along itself
    assert res.u[0] / res.u[1] == pytest.approx(3.0)


def test_inside_the_field_with_no_way_out_falls_back():
    pfb = _pfb()
    r = np.full(N, 0.30)                                     # surrounded, inside every field
    u_prev = np.array([0.2, 0.0, 0.0])
    res = pfb.filter([0.2, 0.0, 0.0], u_prev, u_prev, r, BEARINGS, OK)
    assert res.infeasible and res.reason == "infeasible"


def test_holding_still_close_to_the_field_is_not_infeasible():
    """At rest, a wall 3 mm outside the at-rest field and the nominal pushing into it: the
    tightened linearised rows cannot admit standing still (0.5 mm > alpha dt x 3 mm), but the
    braking command satisfies every TRUE row, so the step is `brake`, not infeasible (found
    2026-10-23 on a blocked tune route)."""
    pfb = _pfb()
    ahead = float(FIELD.extent(np.array([0.0]), 0.0, 0.0, 0.0, at_rest=True)[0])
    r = _wall_ahead(ahead + CONF.delta + 0.003)
    zero = np.zeros(3)
    res = pfb.filter([0.3, 0.0, 0.0], zero, zero, r, BEARINGS, OK)
    assert not res.infeasible and res.mode in ("brake", "filtered")
    pts = r[:, None] * np.column_stack([np.cos(BEARINGS), np.sin(BEARINGS)])
    g, _ = pfb._rows(res.u, pts, res.regime, 0.0, 0.0, 0.0)
    rhs = (1 - CONF.alpha * CONF.dt) * np.maximum(pfb._now(pts, res.regime, 0.0, 0.0), 0.0)
    assert np.all(g >= rhs)


def _random_row_cases(seed, count):
    """Random scans with commands up to the top band's speed and 1.5 rad/s, in every regime, the
    estimate a lagging fraction of the command."""
    rng = np.random.default_rng(seed)
    for _ in range(count):
        r = rng.uniform(0.3, 2.5, N)
        r = np.minimum(r, np.roll(r, 1))
        pts = r[:, None] * np.column_stack([np.cos(BEARINGS), np.sin(BEARINGS)])
        speed, heading = rng.uniform(0.0, 0.51), rng.uniform(-math.pi, math.pi)
        u = np.array([speed * math.cos(heading), speed * math.sin(heading), rng.uniform(-1.5, 1.5)])
        regime = int(rng.integers(-1, 3))
        if regime == REST:
            u[:2] *= 0.018 / max(speed, 1e-9)
            u[2] = float(np.clip(u[2], -0.05, 0.05))
        v_hat = u * rng.uniform(0.6, 1.0)
        yield pts, u, regime, (math.hypot(*v_hat[:2]), math.atan2(v_hat[1], v_hat[0]), v_hat[2])


def test_the_motion_grid_with_its_margin_bounds_the_minimum_over_every_fraction():
    """Ruled 2026-10-26: every row holds over the realised fraction lambda in [0, 1]. With the
    declared grid and `motion_grid_margin`, each row is at or below its minimum over a dense grid
    (201 fractions, no margin), on random scans with commands up to the top band and 1.5 rad/s.
    Positive control: without the margin the grid overstates some rows, so the check can fail."""
    import dataclasses

    dense = PFB(FIELD, dataclasses.replace(CONF, motion_fractions=tuple(np.linspace(0, 1, 201)),
                                           motion_grid_margin=0.0))
    bare = PFB(FIELD, dataclasses.replace(CONF, motion_grid_margin=0.0))
    grid = _pfb()
    worst_with, worst_without = -math.inf, -math.inf
    for pts, u, regime, est in _random_row_cases(19, 200):
        g_dense, _ = dense._rows(u, pts, regime, *est)
        near = g_dense < CONF.row_window                # the rows the QP and the check use
        g_grid, _ = grid._rows(u, pts, regime, *est)
        g_bare, _ = bare._rows(u, pts, regime, *est)
        worst_with = max(worst_with, float((g_grid - g_dense)[near].max(initial=-math.inf)))
        worst_without = max(worst_without, float((g_bare - g_dense)[near].max(initial=-math.inf)))
    assert worst_with <= 1e-12
    assert worst_without > 0.0


def test_the_vectorised_rows_equal_a_plain_per_fraction_reference():
    """Equivalence (ruling 1 of 2026-10-26: speed-ups exact and equivalence-tested): the rows equal
    a plain computation -- each fraction separately, the field through `PFBField.extent` -- to 1e-12,
    on random scans and commands."""
    pfb = _pfb()
    c = CONF
    tau = c.dt + c.lag
    for pts, u, regime, (speed_hat, heading_hat, w_hat) in _random_row_cases(29, 40):
        g, _ = pfb._rows(u, pts, regime, speed_hat, heading_hat, w_hat)
        speed = math.hypot(u[0], u[1])
        heading = math.atan2(u[1], u[0]) if speed >= speed_hat and speed > 1e-9 else heading_hat
        w_field = max(abs(w_hat), abs(u[2]))
        rows = []
        for f in c.motion_fractions:
            phi = -u[2] * f * tau
            rot = np.array([[math.cos(phi), -math.sin(phi)], [math.sin(phi), math.cos(phi)]])
            q = (pts - f * tau * u[:2]) @ rot.T
            psi = np.arctan2(q[:, 1], q[:, 0])
            if regime == REST:
                rho = FIELD.extent(psi, 0.0, 0.0, w_field, at_rest=True)
            else:
                rho = FIELD.band_extent(psi, regime, heading, w_field)
            rows.append(np.hypot(q[:, 0], q[:, 1]) - rho - c.delta - c.motion_grid_margin)
        assert g == pytest.approx(np.min(rows, axis=0), abs=1e-12)


def test_the_row_memo_changes_no_decision():
    """Equivalence (2026-10-26): the filter with its per-call row memo and without it (`_filter`
    called directly, so nothing is memoised) returns the same command, mode, regime and solves on
    scans that exercise the QP, with identical state carried between steps."""
    rng = np.random.default_rng(37)
    memo, plain = _pfb(), _pfb()
    exercised = 0
    for _ in range(25):
        r = np.minimum(_wall_ahead(rng.uniform(0.7, 1.2)), rng.uniform(1.0, 3.0, N))
        u_prev = np.array([rng.uniform(0.15, 0.35), rng.uniform(-0.1, 0.1), rng.uniform(-0.3, 0.3)])
        u_nom = u_prev + np.array([0.05, 0.0, 0.0]) + rng.uniform(-0.03, 0.03, 3)
        a = memo.filter(u_nom, u_prev, u_prev, r, BEARINGS, OK)
        b = plain._filter(u_nom, u_prev, u_prev, r, BEARINGS, OK)
        assert np.array_equal(a.u, b.u) and (a.mode, a.regime, a.solves) == (b.mode, b.regime,
                                                                          b.solves)
        exercised += a.mode == "filtered"
    assert exercised >= 15 and memo._memo is None


def test_the_support_cache_returns_exactly_what_a_fresh_field_computes_and_cannot_be_altered():
    from pfbnav.safety import pfb_field as pf

    rng = np.random.default_rng(31)
    cached = pf.PFBField(FIELD.footprint.extent, FIELD.params)
    uncached = pf.PFBField(FIELD.footprint.extent, FIELD.params)
    uncached._CACHE_SIZE = 0                         # neither the field nor the sweep is cached
    got = None
    for _ in range(30):
        args = (float(rng.choice(FIELD.params.band_tops)), float(rng.uniform(-math.pi, math.pi)),
                float(rng.uniform(-1.5, 1.5)))
        for _repeat in range(2):                                   # a miss, then a hit
            got = cached.support_and_partials(*args)
            fresh = uncached.support_and_partials(*args)
            for a, b in zip(got, fresh, strict=True):
                assert np.array_equal(a, b)
    assert not uncached._support_cache and not uncached._sweep_cache
    with pytest.raises(ValueError):
        got[0][0] = 1.0                                            # read-only


def test_the_rows_hold_over_the_realised_fraction_of_a_turn_known_case():
    """The geometry of the 2026-10-26 unattributed stop, reduced: a return behind and to the right
    of a robot creeping forward while turning clockwise. Over the full horizon the turn carries the
    return to where the field is thinner and the row reads clear (+5.8 mm); at a smaller realised
    fraction the return is inside the field (-1.6 mm). The full horizon alone would admit the
    command; the rows over every fraction do not."""
    import dataclasses

    full = PFB(FIELD, dataclasses.replace(CONF, motion_fractions=(1.0,), motion_grid_margin=0.0))
    every = PFB(FIELD, dataclasses.replace(CONF, motion_grid_margin=0.0))
    psi = -1.99
    pts = np.array([[0.38 * math.cos(psi), 0.38 * math.sin(psi)]])
    u = np.array([0.0226, -0.0045, -0.4932])
    est = (0.022, 0.0, -0.33)
    g_full, _ = full._rows(u, pts, 0, *est)
    g_every, _ = every._rows(u, pts, 0, *est)
    assert g_full[0] == pytest.approx(0.00581, abs=1e-4) and g_full[0] > 0.0
    assert g_every[0] == pytest.approx(-0.00159, abs=1e-4) and g_every[0] < 0.0


def test_random_scans_either_hold_every_true_row_or_say_infeasible():
    pfb = _pfb()
    rng = np.random.default_rng(11)
    for _ in range(25):
        r = rng.uniform(0.4, 3.0, N)
        r = np.minimum(r, np.roll(r, 1))
        v = rng.uniform(-0.2, 0.3, 2)
        u_prev = np.array([v[0], v[1], rng.uniform(-0.3, 0.3)])
        u_nom = u_prev + rng.uniform(-0.05, 0.05, 3)
        res = pfb.filter(u_nom, u_prev, u_prev, r, BEARINGS, OK)
        if res.infeasible:
            continue
        _limits_hold(pfb, res.u, u_prev)
        if res.mode in ("filtered", "brake"):
            pts = r[:, None] * np.column_stack([np.cos(BEARINGS), np.sin(BEARINGS)])
            speed_hat = math.hypot(*u_prev[:2])
            g, _ = pfb._rows(res.u, pts, res.regime, speed_hat, math.atan2(v[1], v[0]),
                             float(u_prev[2]))
            heading_now = math.atan2(v[1], v[0])
            rhs = (1 - CONF.alpha * CONF.dt) * np.maximum(
                pfb._now(pts, res.regime, heading_now, abs(float(u_prev[2]))), 0.0)
            assert np.all(g >= rhs - 1e-6)


def test_the_weights_are_the_ruled_ones():
    """Q5: W = diag(1, 1, rho_out^2), rho_out read from the footprint table (0.3374 m)."""
    assert CONF.weight_yaw == pytest.approx(FIELD.footprint.outer_radius ** 2)
    assert FIELD.footprint.outer_radius == pytest.approx(0.33742, abs=1e-4)
    assert CONF.delta == pytest.approx(0.0005)


# --------------------------------------------------------------------------- under the latch
def test_under_the_latch_only_releasable_commands_pass():
    pfb = _pfb()
    r = _wall_ahead(0.33)          # inside the at-rest field (0.352 ahead), outside the footprint (0.286)
    creep_back = np.array([-0.018, 0.0, 0.0])
    toward = np.array([0.018, 0.0, 0.0])
    fast_back = np.array([-0.3, 0.0, 0.0])
    assert pfb.filter(creep_back, np.zeros(3), np.zeros(3), r, BEARINGS, OK,
                      latched=True).mode == "latched"
    assert np.array_equal(pfb.filter(creep_back, np.zeros(3), np.zeros(3), r, BEARINGS, OK,
                                     latched=True).u, creep_back)
    zeroed = pfb.filter(toward, np.zeros(3), np.zeros(3), r, BEARINGS, OK, latched=True)
    assert np.array_equal(zeroed.u, np.zeros(3)) and zeroed.reason == "not_releasable"
    assert np.array_equal(pfb.filter(fast_back, np.zeros(3), np.zeros(3), r, BEARINGS, OK,
                                     latched=True).u, np.zeros(3))


def test_the_recoverys_creep_passes_unaltered_when_releasable():
    """Q7: B7's recovery commands pass unaltered when releasable. The recovery creeps at 0.018 m/s
    along a direction whose at-rest field is clear (`pfbnav/control/recovery.py`)."""
    pfb = _pfb()
    r = _open()
    r[(BEARINGS > -0.4) & (BEARINGS < 0.4)] = 0.9            # something ahead, outside the field
    for heading in np.linspace(-math.pi, math.pi, 12, endpoint=False):
        cmd = 0.018 * np.array([math.cos(heading), math.sin(heading), 0.0])
        out = pfb.filter(cmd, np.zeros(3), np.zeros(3), r, BEARINGS, OK, latched=True)
        assert np.array_equal(out.u, cmd)


def test_clarabel_and_osqp_give_the_same_filtered_commands():
    """The default solver (clarabel, 2026-10-24) against the reference (osqp) on the same random
    scans: the same mode, and commands within 1e-5 (both solve to 1e-9..1e-10; the answers are
    then checked against the true rows either way)."""
    import dataclasses

    rng = np.random.default_rng(23)
    both = 0
    for _ in range(25):
        r = np.minimum(_wall_ahead(rng.uniform(0.7, 1.2)), rng.uniform(1.0, 3.0, N))
        u_prev = np.array([rng.uniform(0.15, 0.35), rng.uniform(-0.1, 0.1), rng.uniform(-0.3, 0.3)])
        u_nom = u_prev + np.array([0.05, 0.0, 0.0]) + rng.uniform(-0.03, 0.03, 3)
        out = []
        for solver in ("clarabel", "osqp"):
            pfb = PFB(FIELD, dataclasses.replace(CONF, qp_solver=solver))
            out.append(pfb.filter(u_nom, u_prev, u_prev, r, BEARINGS, OK))
        assert out[0].mode == out[1].mode
        assert out[0].u == pytest.approx(out[1].u, abs=1e-5)
        both += out[0].mode == "filtered"
    assert both >= 15                                 # the QP was exercised, not only bypassed


def test_the_field_follows_the_velocity_estimate_not_the_command():
    """Ruling 3 of 2026-09 (the WP1/WP2 acceptance test): fed an estimated velocity that differs from
    the command, the field follows the velocity. Moving sideways at 0.25 m/s (band 1) while the
    previous command and the nominal are a slow creep forward: no candidate is offered below band
    1, and the field's heading is the estimate's (+y), not the command's (+x)."""
    pfb = _pfb()
    v_hat = np.array([0.0, 0.25, 0.0])
    u_prev = np.array([0.05, 0.0, 0.0])
    res = pfb.filter([0.05, 0.0, 0.0], v_hat, u_prev, _open(), BEARINGS, OK)
    assert pf.band_of(0.25, FIELD.params) == 2 and res.regime == 2
    assert res.heading == pytest.approx(math.pi / 2)
    # the same command with the estimate agreeing with it: the command's own band (0.05 m/s is above
    # band 0's limit, so band 1), heading +x
    res = _pfb().filter([0.05, 0.0, 0.0], u_prev, u_prev, _open(), BEARINGS, OK)
    assert pf.band_of(0.05, FIELD.params) == 1 and res.regime == 1
    assert res.heading == pytest.approx(0.0)
