"""The toy positive control for L2, SAC-Lagrangian (ruled 2026-10-03; settings ruled 2026-10-27).

A constrained problem whose answer is known (docstring of `tests/test_sac_lagrangian.py`): reward
0.05 a for an action a in [-1, 1], a "protective stop" (reported as NavEnv reports one,
info["certified"]["stop_triggered"]) whenever a > 0, a limit of d = 1 stop per T = 4-step episode.
Shared by the fast suite (1,000 steps, candidate 1, seed 0) and `scripts/validate_l2_toy.py` (the
researcher's 5,000-step run per seed and PID candidate, which selects L2's gains; A28b). No simulator.
"""
from __future__ import annotations

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from pfbnav.control.sac_lagrangian import PID_CANDIDATES, RULED

T, D = 4, 1.0                    # steps per episode; the cost limit, stops per episode
REWARD_SCALE = 0.05              # the toy's reward per unit of action (docstring)
TOY_TRAINING = {"algo": "sac", "learning_rate": 3e-3, "buffer_size": 5000, "learning_starts": 50,
                "batch_size": 64, "tau": 0.05, "gamma": 0.9, "train_freq": 1, "gradient_steps": 1,
                "ent_coef": "auto", "net_arch": [32, 32], "device": "cpu"}
TOY_CFG = {"training": TOY_TRAINING, "filter": {"enabled": False},
           "reward": {"k_stop": 0.0, "k_proximity": 0.0, "max_steps": T},
           "lagrangian": {"cost_limit": D, **RULED, "pid_candidate": 1}}


def toy_cfg(candidate: int) -> dict:
    """TOY_CFG with one of the declared PID candidates (A28b)."""
    if candidate not in PID_CANDIDATES:
        raise ValueError(f"PID candidate {candidate} is not declared: {sorted(PID_CANDIDATES)}")
    return {**TOY_CFG, "lagrangian": {**TOY_CFG["lagrangian"], "pid_candidate": candidate}}


class StopToy(gym.Env):
    """The toy above. Records each finished episode's stop count."""

    def __init__(self):
        self.observation_space = spaces.Box(0.0, 1.0, shape=(1,), dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32)
        self.episode_costs: list[int] = []
        self.t = self.stops = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.t = self.stops = 0
        return np.zeros(1, dtype=np.float32), {}

    def step(self, action):
        a = float(np.clip(np.asarray(action).ravel()[0], -1.0, 1.0))
        self.t += 1
        triggered = a > 0.0
        self.stops += int(triggered)
        done = self.t >= T
        if done:
            self.episode_costs.append(self.stops)
        return (np.array([self.t / T], dtype=np.float32), REWARD_SCALE * a, done, False,
                {"certified": {"stop_triggered": bool(triggered)}})


def mean_action(model) -> float:
    """The deterministic action averaged over the episode's T observations."""
    obs = np.array([[k / T] for k in range(T)], dtype=np.float32)
    return float(np.mean(model.predict(obs, deterministic=True)[0]))
