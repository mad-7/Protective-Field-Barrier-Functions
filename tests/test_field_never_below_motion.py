"""The field is never sized below true motion (ruled 2026-10-16). Synthetic profiles, no simulator.

The band and the rotation sweep take max(estimate, applied command). A base that follows its command
with a lag and with slip is driven through acceleration and deceleration. Its true velocity is
known to the test only: at every decision the band and sweep must be at least those its true
motion implies. The control: the estimate alone falls below true motion while accelerating, and
the command alone while decelerating, so each half would fail where the maximum holds. The
simulator version, on ground-truth velocity, is in `tests/test_step_loop_sim.py`.
"""
import math
from pathlib import Path

import numpy as np

from pfbnav.config import load_config
from pfbnav.safety.certified import CertifiedLayer, band_index, reach_speed, yaw_excursion
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.robot import ActuationLimits, limit_acceleration
from pfbnav.sim.velocity_estimate import PoseDifferenceEstimator, VelocityEstimateConfig
from pfbnav.world.strata import certified_params_from_config

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
PARAMS = certified_params_from_config(CFG)
DT = float(CFG["sim"]["dt"])


def _profile(command_of_step, steps=120, lag=0.08, slip=0.85):
    """Drive a lagged, slipping base; return (decision, truth at decision) pairs and the
    signals each half of the composition would have used."""
    table = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
    layer = CertifiedLayer(PARAMS, float(CFG["robot"]["half_length"]),
                           float(CFG["robot"]["half_width"]), table.extent)
    limits = ActuationLimits.from_config(CFG)
    est = PoseDifferenceEstimator(VelocityEstimateConfig.from_config(CFG))
    pose, true_v, applied = np.zeros(3), np.zeros(3), np.zeros(3)
    velocity = est.update((0.0, 0.0, 0.0))
    b = np.linspace(-math.pi, math.pi, 120, endpoint=False)
    ranges, valid = np.full(120, 5.0), np.ones(120, bool)
    rows = []
    for k in range(steps):
        cert = layer.step(k * DT, velocity, ranges, b, valid, applied=applied)
        rows.append((cert, true_v.copy(), velocity.copy(), applied.copy()))
        applied = limit_acceleration(np.asarray(command_of_step(k), float), applied, DT, limits)
        target = np.array([slip * applied[0], slip * applied[1], applied[2]])
        true_v = true_v + (target - true_v) * (DT / (lag + DT))     # the base lags its command
        c, s = math.cos(pose[2]), math.sin(pose[2])
        pose = pose + DT * np.array([c * true_v[0] - s * true_v[1], s * true_v[0] + c * true_v[1],
                                     true_v[2]])
        velocity = est.update((float(pose[0]), float(pose[1]), float(pose[2])))
    return rows


def _accelerate_then_stop(k):
    return (0.40, 0.0, 1.2) if k < 60 else (0.0, 0.0, 0.0)


def test_band_and_sweep_never_fall_below_true_motion():
    tops = PARAMS.band_top_speeds
    for profile in (_accelerate_then_stop,
                    lambda k: (0.25, 0.30, -1.0) if k < 50 else (0.0, 0.0, 0.0)):
        for cert, truth, _est, _app in _profile(profile):
            true_speed = math.hypot(truth[0], truth[1])
            # A20: the band is chosen from the REACHABLE speed, and the sweep is the interval
            # reachable from |omega|; both are monotone, so composing above truth keeps them above.
            if not cert["at_rest"]:
                assert cert["field_id"] >= band_index(reach_speed(true_speed, PARAMS), tops)
            assert cert["field_speed"] >= true_speed - 1e-12
            assert cert["yaw_interval"][1] >= yaw_excursion(abs(truth[2]), PARAMS) - 1e-12


def test_each_half_alone_would_fall_below_true_motion():
    """The positive control: the composition is needed, not decorative."""
    rows = _profile(_accelerate_then_stop)
    est_short = any(math.hypot(*e[:2]) < math.hypot(*t[:2]) - 1e-3 for _c, t, e, _a in rows)
    cmd_short = any(math.hypot(*a[:2]) < math.hypot(*t[:2]) - 1e-3 for _c, t, _e, a in rows)
    assert est_short, "the estimate never lagged true speed: the profile tests nothing"
    assert cmd_short, "the command never understated true speed: the profile tests nothing"
