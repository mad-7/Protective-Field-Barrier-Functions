"""Training-only stall truncation (A30, ruled 2026-10-30): known answers, the bootstrap, the scope."""
from collections import deque
from pathlib import Path

import gymnasium as gym
import numpy as np
import pytest
from gymnasium import spaces

from pfbnav.config import load_config
from pfbnav.envs.freeze import import_closure
from pfbnav.training.stall import OUTCOME, StallRule, StallTruncation, stalled

REPO_ROOT = Path(__file__).resolve().parents[1]
RULE = StallRule(window_s=30.0, min_progress_m=0.05, window_steps=600)


def test_the_rule_is_the_ruled_one_in_the_config():
    cfg = load_config(REPO_ROOT / "configs" / "base.yaml")
    assert StallRule.from_config(cfg) == RULE
    assert cfg["training"]["total_timesteps"] == 2_000_000
    assert cfg["reward"]["max_steps"] == 4000              # evaluation's horizon unchanged


def test_a_window_that_is_not_whole_steps_is_refused():
    cfg = {"training": {"stall_truncation": {"window_s": 30.01, "min_progress_m": 0.05}},
           "sim": {"dt": 0.05}}
    with pytest.raises(ValueError, match="whole number"):
        StallRule.from_config(cfg)
    cfg["training"]["stall_truncation"] = {"window_s": 30.0, "min_progress_m": 0.0}
    with pytest.raises(ValueError, match="positive"):
        StallRule.from_config(cfg)


def test_stalled_known_answers():
    short = deque([0.0] * 600)
    assert stalled(short, RULE) == (False, None)             # window not full
    assert stalled(deque([0.0] * 600 + [0.049]), RULE)[0]
    assert not stalled(deque([0.0] * 600 + [0.05]), RULE)[0]  # exactly the minimum is progress
    assert stalled(deque([1.0] + [2.0] * 600), RULE) == (False, 1.0)


class FakeNav(gym.Env):
    """Steps s by `rates[t]` m per step; ends on its own at `horizon` (truncated) or `goal_at`."""
    observation_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
    action_space = spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32)

    def __init__(self, rates, horizon=4000, goal_at=None, s0=0.0):
        self.rates, self.horizon, self.goal_at, self.s0 = rates, horizon, goal_at, s0
        self.episode_info = {"scenario_seed": 7}
        self.loop = self

    def report(self):
        return {"steps": self.t, "protective_stops": 0}

    def reset(self, *, seed=None, options=None):
        self.t, self.s = 0, self.s0
        return np.zeros(2, dtype=np.float32), {}

    def step(self, action):
        gained = self.rates(self.t)
        self.s += gained
        self.t += 1
        info = {"s_m": self.s, "progress_m": gained, "outcome": "running",
                "certified": {"stop_active": True}}
        if self.goal_at is not None and self.t >= self.goal_at:
            info["outcome"] = "success"
            return np.zeros(2, dtype=np.float32), 1.0, True, False, info
        if self.t >= self.horizon:
            info["outcome"] = "timeout"
            return np.zeros(2, dtype=np.float32), 0.0, False, True, info
        return np.zeros(2, dtype=np.float32), 0.0, False, False, info


def run_until_done(env):
    env.reset()
    t = 0
    while True:
        _o, _r, term, trunc, info = env.step(np.zeros(1))
        t += 1
        if term or trunc:
            return t, term, trunc, info


def test_a_robot_that_never_moves_is_truncated_at_the_window_and_bootstrapped():
    env = StallTruncation(FakeNav(lambda t: 0.0, s0=0.3), RULE)
    t, term, trunc, info = run_until_done(env)
    assert (t, term, trunc) == (600, False, True)
    assert info["outcome"] == OUTCOME and info["episode"]["outcome"] == OUTCOME
    assert info["episode"]["scenario_seed"] == 7 and info["episode"]["steps"] == 600
    assert info["episode"]["stall"]["progress_in_window_m"] == 0.0
    assert info["episode"]["stall"]["stop_active"] is True
    assert env.truncations == 1


@pytest.mark.parametrize("per_window", np.round(np.arange(0.040, 0.0601, 0.001), 4))
def test_dense_sweep_of_steady_speeds_around_the_threshold(per_window):
    """A steady creep of `per_window` m per 30 s: truncated at step 600 below 0.05, never at or
    above (to the end of a 4000-step horizon)."""
    rate = per_window / 600.0
    env = StallTruncation(FakeNav(lambda t: rate), RULE)
    t, _term, trunc, info = run_until_done(env)
    if per_window < 0.05 - 1e-9:
        assert (t, info["outcome"]) == (600, OUTCOME)
    elif per_window > 0.05 + 1e-9:
        assert (t, info["outcome"], trunc) == (4000, "timeout", True)


@pytest.mark.parametrize("stop_at", [600, 601, 1000, 2345, 3399])
def test_a_robot_that_stops_later_is_truncated_one_window_after_it_stopped(stop_at):
    env = StallTruncation(FakeNav(lambda t: 0.02 if t < stop_at else 0.0), RULE)
    t, _term, _trunc, info = run_until_done(env)
    # 0.02 m per moving step: the window [t-600, t] holds less than 0.05 m once it holds at most
    # two moving steps (0.04 m; three hold 0.06 m), i.e. from step stop_at + 598.
    assert t == stop_at + 598 and info["outcome"] == OUTCOME


def test_the_env_s_own_ends_pass_through_untouched():
    t, term, trunc, info = run_until_done(StallTruncation(FakeNav(lambda t: 0.0, goal_at=300), RULE))
    assert (t, term, trunc, info["outcome"]) == (300, True, False, "success")
    t, term, trunc, info = run_until_done(
        StallTruncation(FakeNav(lambda t: 0.001, horizon=700), RULE))
    assert (t, info["outcome"]) == (700, "timeout")


def test_reset_clears_the_window():
    env = StallTruncation(FakeNav(lambda t: 0.0), RULE)
    env.reset()
    for _ in range(599):
        env.step(np.zeros(1))
    env.reset()
    for _ in range(599):
        assert not env.step(np.zeros(1))[3]


def test_sb3_marks_the_truncation_as_a_time_limit_and_the_buffer_bootstraps():
    """The ruling's 'bootstrapped, not terminal': SB3's vector wrapper flags the truncation and the
    replay buffer of SAC (and of SAC-Lagrangian, its subclass) stores it as a timeout, so the
    sampled `dones` is 0 for it."""
    from stable_baselines3 import SAC
    from stable_baselines3.common.vec_env import DummyVecEnv
    venv = DummyVecEnv([lambda: StallTruncation(FakeNav(lambda t: 0.0), RULE)])
    model = SAC("MlpPolicy", venv, buffer_size=1000, learning_starts=10_000, device="cpu", seed=0)
    assert model.replay_buffer.handle_timeout_termination
    model.learn(total_timesteps=601)
    assert model.replay_buffer.dones[599, 0] == 1.0
    assert model.replay_buffer.timeouts[599, 0] == 1.0
    sample = model.replay_buffer._get_samples(np.array([599]))
    assert float(sample.dones[0, 0]) == 0.0


def test_training_only_scope():
    """Only training code uses the wrapper; the frozen environment's closure does not contain it."""
    users = sorted(p.relative_to(REPO_ROOT).as_posix()
                   for p in list((REPO_ROOT / "scripts").glob("*.py"))
                   + list((REPO_ROOT / "pfbnav").rglob("*.py"))
                   if "pfbnav.training.stall" in p.read_text(encoding="utf-8"))
    assert users == ["scripts/train.py"]
    assert REPO_ROOT / "pfbnav" / "training" / "stall.py" not in import_closure()
