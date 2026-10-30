"""The training-run diagnosis (ruled 2026-10-30): known answers on synthetic data (hard rule 6)."""
import numpy as np
import pytest

from pfbnav.envs.observation import BLOCKS
from pfbnav.eval import training_diag as d

W = {"time_cost": 0.01, "k_stop": 5.0, "goal_bonus": 200.0, "contact_penalty": 200.0}


def test_reward_split_of_a_stuck_episode():
    # 4000 steps, 3 stops, unrecoverable; return -1090 -> time -40, stops -15, residual -1035.
    s = d.reward_split({"episode_length": 4000, "protective_stops": 3,
                        "outcome": "unrecoverable_stop", "episode_return": -1090.0}, W)
    assert s["time"] == pytest.approx(-40.0) and s["stop"] == -15.0
    assert s["goal"] == 0.0 and s["contact"] == 0.0
    assert s["residual"] == pytest.approx(-1035.0)
    assert s["progress_lower_bound"] == 0.0 and s["penalties_at_least"] == pytest.approx(1035.0)


def test_reward_split_of_a_success_and_a_collision():
    ok = d.reward_split({"episode_length": 1000, "protective_stops": 0, "outcome": "success",
                         "episode_return": 250.0}, W)
    assert ok["goal"] == 200.0 and ok["residual"] == pytest.approx(60.0)
    assert ok["progress_lower_bound"] == pytest.approx(60.0)
    hit = d.reward_split({"episode_length": 10, "protective_stops": 1, "outcome": "collision",
                          "episode_return": -205.1}, W)
    assert hit["contact"] == -200.0 and hit["residual"] == pytest.approx(0.0, abs=1e-12)


def test_action_stats_known_answers():
    a = np.array([[1.0, 0.0, 0.0], [-0.97, 0.0, 0.01], [1.0, 1.0, -1.0], [0.0, 0.02, -0.03]])
    s = d.action_stats(a)
    assert s["saturated_share"] == [0.75, 0.25, 0.25]
    assert s["all_axes_saturated_share"] == 0.25
    assert s["near_zero_share"] == 0.25
    with pytest.raises(ValueError):
        d.action_stats(np.zeros((0, 3)))


def test_value_stats_sign_and_scale_and_non_finite():
    s = d.value_stats(np.array([-3.0, -1.0, 1.0, 5.0]))
    assert s["negative_share"] == 0.5 and s["mean"] == 0.5 and s["min"] == -3.0 and s["max"] == 5.0
    assert d.value_stats(np.array([1.0, np.nan]))["non_finite"] == 1


def test_observation_ranges_flags_out_of_bounds_and_constant_channels():
    dim = sum(w for _, w in BLOCKS)
    low, high = np.zeros(dim), np.ones(dim)
    obs = np.full((3, dim), 0.5)
    obs[:, 0] = [0.1, 0.2, 1.3]                      # a scan channel above its bound once
    out = d.observation_ranges(obs, low, high, BLOCKS)
    assert out["scan"]["outside_bounds"] == 1 and out["scan"]["observed_max"] == 1.3
    assert out["scan"]["constant_channels"] == 35
    assert out["certified"]["constant_channels"] == 1 and out["goal"]["outside_bounds"] == 0
    with pytest.raises(ValueError):
        d.observation_ranges(obs[:, :10], low, high, BLOCKS)


def test_column_trajectory_skips_blanks_and_reads_fractions():
    rows = [{"time/total_timesteps": str(t), "train/ent_coef": "" if t == 0 else str(1.0 / t)}
            for t in range(11)]
    out = d.column_trajectory(rows, "train/ent_coef", fractions=(0.0, 1.0))
    assert out["rows"] == 10 and out["at"]["0"]["value"] == 1.0
    assert out["at"]["1"] == {"timestep": 10.0, "value": 0.1}
    assert d.column_trajectory(rows, "missing")["rows"] == 0
