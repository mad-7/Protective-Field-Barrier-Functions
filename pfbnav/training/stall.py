"""Training-only stall truncation (ruled 2026-10-30, A30).

L1's first pilot (seed 9000000, 300k steps) spent its 75 episodes at the 200 s horizon: 51 ended
latched (`unrecoverable_stop`), the robot held for 3,200-3,990 of 4,000 steps, and nothing it did
there could change the outcome. The supervisor ruled a truncation for TRAINING only:

    truncate an episode after 30 s with less than 0.05 m of progress, as a time-limit truncation
    (bootstrapped, not terminal). Evaluation keeps the 4,000-step horizon and its outcome rules.

**Progress** is the route's arc length `s` (`pfbnav/planning/lookahead.py`), the quantity the
progress reward pays for. `PathTracker` projects forward only, so `s` never decreases and the
progress over a window is simply `s(t) - s(t - W)`: a robot cannot bank progress by going back and
forth. The window slides: at every step from the `W`-th on (W = window_s / dt = 600 steps at 20 Hz),
the episode is truncated if the last W steps gained less than `min_progress_m`. The window opens at
the episode's start, so the earliest truncation is at step W.

**Bootstrapped, not terminal.** The wrapper returns `terminated=False, truncated=True`. SB3's vector
wrapper turns that into `info["TimeLimit.truncated"] = True`, and the replay buffer
(`handle_timeout_termination=True`, SB3's default and the cost buffer's) then keeps the next state's
value in the target, as for the 4,000-step horizon. A stall is not a failure the policy is told
ends the world; it is the end of the episode's budget.

**Outcome.** A truncated episode is `stall_truncated`, its own outcome, never a `timeout` or an
`unrecoverable_stop` (those are evaluation's, at the full horizon). The record says whether a stop
was latched at the time, so a stuck-in-a-stop stall stays distinguishable from a slow one.

**Where it applies.** Only `scripts/train.py` (and the training memory check, which runs the same
code) wraps the environment; evaluation, G4's checkpoint evaluation, the controls and every
classical baseline use `NavEnv` as it is (`tests/test_stall_truncation.py` checks that no other
script imports this module). The wrapper sits outside the frozen environment's import closure, so
the environment's identity does not move.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import gymnasium as gym

OUTCOME = "stall_truncated"


@dataclass(frozen=True)
class StallRule:
    """`training.stall_truncation`, read once. `window_steps` = round(window_s / dt)."""
    window_s: float
    min_progress_m: float
    window_steps: int

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> StallRule:
        block = cfg["training"]["stall_truncation"]
        window_s = float(block["window_s"])
        min_progress = float(block["min_progress_m"])
        dt = float(cfg["sim"]["dt"])
        if window_s <= 0.0 or min_progress <= 0.0 or dt <= 0.0:
            raise ValueError("stall_truncation: window_s, min_progress_m and sim.dt must be positive")
        steps = round(window_s / dt)
        if abs(steps * dt - window_s) > 1e-9:
            raise ValueError(f"stall_truncation.window_s {window_s} is not a whole number of "
                             f"control steps of {dt} s")
        return cls(window_s=window_s, min_progress_m=min_progress, window_steps=int(steps))


def stalled(s_history: deque, rule: StallRule) -> tuple[bool, float | None]:
    """(stalled, progress over the window, m). `s_history` holds s at steps t-W..t (W+1 values,
    oldest first); with fewer, the window is not yet full and nothing is stalled."""
    if len(s_history) < rule.window_steps + 1:
        return False, None
    gained = float(s_history[-1] - s_history[0])
    return gained < rule.min_progress_m, gained


class StallTruncation(gym.Wrapper):
    """Truncates a training episode whose route progress over the last `window_s` is below
    `min_progress_m`. Reads `s_m` and `progress_m` from `NavEnv`'s step info; on a truncation it
    builds the episode record `NavEnv` builds at its own ends (`episode_info` + the loop's report)
    with the outcome `stall_truncated`."""

    def __init__(self, env: gym.Env, rule: StallRule):
        super().__init__(env)
        self.rule = rule
        self._s: deque = deque(maxlen=rule.window_steps + 1)
        self.truncations = 0

    def reset(self, **kwargs):
        self._s.clear()
        return self.env.reset(**kwargs)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        if not self._s:
            # s before the first step: the step's s less the progress it made.
            self._s.append(float(info["s_m"]) - float(info["progress_m"]))
        self._s.append(float(info["s_m"]))
        if terminated or truncated:
            return obs, reward, terminated, truncated, info
        is_stalled, gained = stalled(self._s, self.rule)
        if not is_stalled:
            return obs, reward, terminated, truncated, info
        self.truncations += 1
        base: Any = self.env.unwrapped                 # NavEnv
        info = dict(info)
        info["outcome"] = OUTCOME
        info["episode"] = {
            **base.episode_info, **base.loop.report(), "outcome": OUTCOME,
            "stall": {"window_s": self.rule.window_s, "min_progress_m": self.rule.min_progress_m,
                      "progress_in_window_m": gained,
                      "stop_active": bool(info.get("certified", {}).get("stop_active", False))}}
        return obs, reward, False, True, info
