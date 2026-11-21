"""Draw the route and the robot's track while a policy trains (`scripts/train.py --draw-path`). Training only.

A Gymnasium wrapper that hands each reset, step and episode end to a `pfbnav.sim.path_overlay.PathOverlay`,
so the planned route (blue), start and goal (green, yellow) and the track driven (red) appear in the scene.
It changes nothing the learner sees: observations, rewards and the done flags pass through untouched, and the
overlay only draws (no physics, no scan return). It costs a few remote calls per step, so it is for watching a
run, not for a full-length one.

It sits outside the environment's import closure (like the stall and curriculum wrappers), so the
environment's identity does not move.
"""
from __future__ import annotations

from typing import Any

import gymnasium as gym


class PathOverlayWrapper(gym.Wrapper):
    """`base` is the `NavEnv` the overlay reads the plan, the placement transform and the pose from; `env` is
    whatever wraps it on the way to the learner."""

    def __init__(self, env: gym.Env, base: Any, overlay: Any):
        super().__init__(env)
        self._base = base
        self._overlay = overlay

    def reset(self, **kwargs: Any):
        obs, info = self.env.reset(**kwargs)
        self._overlay.begin(self._base, info)
        return obs, info

    def step(self, action: Any):
        obs, reward, terminated, truncated, info = self.env.step(action)
        self._overlay.step(self._base, info)
        if terminated or truncated:
            self._overlay.end()
        return obs, reward, terminated, truncated, info

    def close(self):
        self._overlay.end()
        return self.env.close()
