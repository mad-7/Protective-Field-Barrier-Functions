"""Stopping a training run without losing the model (2026-10-30): the STOP file and Ctrl+C."""
import gymnasium as gym
import numpy as np
import pytest
from gymnasium import spaces
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback, CallbackList

from pfbnav.training.record import STOP_FILE, StopFile


class Line(gym.Env):
    observation_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
    action_space = spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        return np.zeros(2, dtype=np.float32), {}

    def step(self, action):
        return np.zeros(2, dtype=np.float32), 0.0, False, False, {}


class At(BaseCallback):
    """Runs `fn` once at step `t`."""
    def __init__(self, t, fn):
        super().__init__()
        self.t, self.fn = t, fn

    def _on_step(self):
        if self.num_timesteps == self.t:
            self.fn()
        return True


def _model():
    return SAC("MlpPolicy", Line(), buffer_size=1000, learning_starts=50, device="cpu", seed=0)


def test_the_stop_file_ends_learn_at_the_next_check_and_the_model_saves(tmp_path):
    model, stop = _model(), StopFile(tmp_path, every=100)
    model.learn(total_timesteps=10_000, callback=CallbackList(
        [At(150, lambda: (tmp_path / STOP_FILE).write_text("")), stop]))
    assert stop.stopped_at == 200 and model.num_timesteps == 200
    model.save(tmp_path / "stopped_model.zip")
    assert SAC.load(tmp_path / "stopped_model.zip").policy is not None


def test_without_the_file_training_runs_to_the_end(tmp_path):
    model, stop = _model(), StopFile(tmp_path, every=100)
    model.learn(total_timesteps=300, callback=stop)
    assert stop.stopped_at is None and model.num_timesteps == 300


def test_a_leftover_stop_file_is_refused_before_training(tmp_path):
    (tmp_path / STOP_FILE).write_text("")
    with pytest.raises(FileExistsError):
        StopFile(tmp_path)


def test_after_ctrl_c_mid_learn_the_model_still_saves_and_loads(tmp_path):
    def interrupt():
        raise KeyboardInterrupt
    model = _model()
    with pytest.raises(KeyboardInterrupt):
        model.learn(total_timesteps=10_000, callback=At(120, interrupt))
    model.save(tmp_path / "interrupted_model.zip")
    loaded = SAC.load(tmp_path / "interrupted_model.zip")
    obs = np.zeros(2, dtype=np.float32)
    assert np.allclose(loaded.predict(obs, deterministic=True)[0],
                       model.predict(obs, deterministic=True)[0])
