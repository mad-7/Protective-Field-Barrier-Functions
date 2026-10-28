"""Known-answer tests for the estimate-overshoot measure (hard rule 6), on synthetic runs."""
import numpy as np
import pytest

from pfbnav.eval.overshoot import overshoot_ratio
from pfbnav.sim.velocity_estimate import VelocityEstimateConfig

DT = 0.05
CFG = VelocityEstimateConfig(dt=DT, filter_tau=0.10, latency_steps=0, noise_std=(0.0, 0.0, 0.0))


def _run(speeds, yaw=0.3):
    """Poses of a straight run at the given speed per step (pose k+1 follows step k)."""
    x = np.concatenate([[0.0], np.cumsum(np.asarray(speeds) * DT)])
    return np.column_stack([x * np.cos(yaw), x * np.sin(yaw), np.full(x.size, yaw)])


def test_a_robot_realising_ninety_percent_of_its_command_gives_a_ratio_of_point_nine():
    n = 400
    poses = _run(np.full(n, 0.09))
    out = overshoot_ratio(poses, np.full(n + 1, 0.10), CFG)
    assert out["ratio"] == pytest.approx(0.90, abs=1e-6)
    assert out["max_estimate"] == pytest.approx(0.09, abs=1e-6)


def test_an_overshooting_robot_is_caught():
    """The positive control: a realised speed above the command must show a ratio above 1."""
    speeds = np.concatenate([np.full(100, 0.12), np.full(300, 0.10)])
    out = overshoot_ratio(_run(speeds), np.full(401, 0.10), CFG)
    assert out["ratio"] > 1.15


def test_one_command_per_pose_is_required():
    with pytest.raises(ValueError):
        overshoot_ratio(_run(np.full(10, 0.1)), np.full(10, 0.1), CFG)
