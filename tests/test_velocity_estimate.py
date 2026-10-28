"""Known-answer tests for the pose-differencing velocity estimate (ruling 4, WP0)."""
import math

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.sim.velocity_estimate import (
    PoseDifferenceEstimator,
    VelocityEstimateConfig,
    body_velocity_from_poses,
)

DT = 0.05


def _path(twist, steps, yaw0=0.3):
    """World poses of a body moving at constant body twist (vx, vy, wz), exactly."""
    vx, vy, wz = twist
    poses, x, y, yaw = [], 0.0, 0.0, yaw0
    for _ in range(steps):
        poses.append((x, y, yaw))
        if abs(wz) < 1e-12:
            x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * DT
            y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * DT
        else:  # exact integration of a constant twist over one step
            s0, c0 = math.sin(yaw), math.cos(yaw)
            yaw1 = yaw + wz * DT
            s1, c1 = math.sin(yaw1), math.cos(yaw1)
            x += (vx * (s1 - s0) + vy * (c1 - c0)) / wz
            y += (vx * (c0 - c1) + vy * (s1 - s0)) / wz
            yaw = yaw1
    return poses


def test_straight_line_is_recovered_exactly():
    poses = _path((0.4, -0.2, 0.0), 3)
    assert body_velocity_from_poses(poses[0], poses[1], DT) == pytest.approx(
        [0.4, -0.2, 0.0], abs=1e-12)


def test_an_arc_is_recovered_to_the_stated_bound():
    # Relative speed error (w dt)^2 / 24 at w = 1.5 rad/s: 2.3e-4.
    poses = _path((0.4, 0.0, 1.5), 3)
    est = body_velocity_from_poses(poses[0], poses[1], DT)
    assert est[2] == pytest.approx(1.5, rel=1e-12)
    assert abs(est[0] - 0.4) / 0.4 <= (1.5 * DT) ** 2 / 24 * 1.01
    assert est[1] == pytest.approx(0.0, abs=1e-12)


def test_heading_wrap_is_handled():
    est = body_velocity_from_poses((0.0, 0.0, math.pi - 0.01), (0.0, 0.0, -math.pi + 0.01), DT)
    assert est[2] == pytest.approx(0.02 / DT)


def test_filter_step_response_is_first_order():
    cfg = VelocityEstimateConfig(dt=DT, filter_tau=0.1, latency_steps=0, noise_std=(0, 0, 0))
    est = PoseDifferenceEstimator(cfg)
    poses = _path((0.5, 0.0, 0.0), 12)
    out = [est.update(p) for p in poses]
    alpha = DT / (0.1 + DT)
    for n in range(1, 12):
        assert out[n][0] == pytest.approx(0.5 * (1 - (1 - alpha) ** n), rel=1e-9)


def test_one_step_latency_delays_by_exactly_one_step():
    base = VelocityEstimateConfig(dt=DT, filter_tau=0.1, latency_steps=0, noise_std=(0, 0, 0))
    late = VelocityEstimateConfig(dt=DT, filter_tau=0.1, latency_steps=1, noise_std=(0, 0, 0))
    a, b = PoseDifferenceEstimator(base), PoseDifferenceEstimator(late)
    poses = _path((0.3, 0.1, 0.5), 10)
    out_a = [a.update(p) for p in poses]
    out_b = [b.update(p) for p in poses]
    assert out_b[0] == pytest.approx([0, 0, 0])
    for n in range(1, 10):
        assert out_b[n] == pytest.approx(out_a[n - 1], abs=1e-15)


def test_noise_is_seeded_and_has_the_configured_spread():
    cfg = VelocityEstimateConfig(dt=DT, filter_tau=0.0, latency_steps=0,
                                 noise_std=(0.02, 0.01, 0.05))
    poses = _path((0.3, 0.0, 0.0), 4001)
    run1 = [PoseDifferenceEstimator(cfg, np.random.default_rng(7)) for _ in range(2)]
    seqs = [[e.update(p) for p in poses] for e in run1]
    assert np.array_equal(np.array(seqs[0]), np.array(seqs[1]))
    err = np.array(seqs[0][1:]) - np.array([0.3, 0.0, 0.0])
    assert err.std(axis=0) == pytest.approx([0.02, 0.01, 0.05], rel=0.06)


def test_reset_forgets_the_previous_episode():
    cfg = VelocityEstimateConfig(dt=DT, filter_tau=0.1, latency_steps=1, noise_std=(0, 0, 0))
    est = PoseDifferenceEstimator(cfg)
    for p in _path((0.5, 0.0, 0.0), 10):
        est.update(p)
    est.reset()
    assert est.update((5.0, 5.0, 1.0)) == pytest.approx([0, 0, 0])


def test_config_is_committed_with_the_sensitivity_knobs_off():
    cfg = VelocityEstimateConfig.from_config(load_config("configs/base.yaml"))
    assert cfg.latency_steps == 0 and cfg.noise_std == (0.0, 0.0, 0.0)
    assert cfg.dt == pytest.approx(0.05)
