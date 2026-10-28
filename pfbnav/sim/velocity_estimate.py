"""Body-velocity estimate from filtered pose differencing. Pure; no simulator I/O.

Ruling 4 on the WP0 results report. The safety layers need the robot's *actual* velocity:
the command over-states it by 8-28 % on this platform and lags it in every deceleration,
and wheel odometry agrees with the command to 0.006 % because the shortfall is ground slip
(G1b re-run, 2026-09-18). So neither is an estimate of actual velocity. This estimator
differences the ground-truth pose of `youBot_ref` and low-pass filters the result.

**Privileged information.** The pose differenced here is the simulator's ground truth. A
real AMR has no such signal: it would estimate velocity from wheel odometry fused with an
IMU and scan matching, all of which drift or lag. Results that depend on this estimate
state that it is privileged, and WP2's acceptance test includes a declared sensitivity
check with configurable noise and one-step latency (`noise_std`, `latency_steps`).

Differencing uses the midpoint heading, so a straight line is recovered exactly and an arc
at yaw rate w to within a relative (w dt)^2 / 24 in speed -- 2.3e-4 at the configured
1.5 rad/s and 0.05 s. The filter is first order, alpha = dt / (tau + dt).

Frames: pose (x, y, yaw) in the world frame, m and rad; estimate (vx, vy, wz) in the body
frame at the latest pose, m/s and rad/s.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class VelocityEstimateConfig:
    dt: float                   # s, physics step between poses
    filter_tau: float           # s, first-order low-pass time constant; 0 disables it
    latency_steps: int          # steps by which the estimate is delayed (sensitivity check)
    noise_std: tuple[float, float, float]  # (m/s, m/s, rad/s) added to each raw difference

    def __post_init__(self) -> None:
        if self.dt <= 0.0 or self.filter_tau < 0.0 or self.latency_steps < 0:
            raise ValueError("need dt > 0, filter_tau >= 0, latency_steps >= 0")
        if len(self.noise_std) != 3 or any(s < 0.0 for s in self.noise_std):
            raise ValueError("noise_std must be three non-negative values")

    @property
    def alpha(self) -> float:
        return self.dt / (self.filter_tau + self.dt)

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> VelocityEstimateConfig:
        v = cfg["estimation"]["velocity"]
        noise = [float(s) for s in v["noise_std"]]
        if len(noise) != 3:
            raise ValueError(f"estimation.velocity.noise_std needs 3 values, got {noise}")
        return cls(dt=float(cfg["sim"]["dt"]), filter_tau=float(v["filter_tau"]),
                   latency_steps=int(v["latency_steps"]),
                   noise_std=(noise[0], noise[1], noise[2]))


def body_velocity_from_poses(previous: tuple[float, float, float],
                             current: tuple[float, float, float], dt: float) -> np.ndarray:
    """Raw (vx, vy, wz) in the body frame from two world poses dt apart, midpoint heading."""
    dx, dy = current[0] - previous[0], current[1] - previous[1]
    dyaw = math.remainder(current[2] - previous[2], 2.0 * math.pi)
    mid = previous[2] + dyaw / 2.0
    c, s = math.cos(mid), math.sin(mid)
    return np.array([(c * dx + s * dy) / dt, (-s * dx + c * dy) / dt, dyaw / dt])


class PoseDifferenceEstimator:
    """Filtered, optionally noisy and delayed, body velocity from successive poses.

    The robot is assumed at rest when the estimator is reset -- true at every episode
    start, since the base is settled before the first command -- so the first estimate
    is zero rather than undefined.
    """

    def __init__(self, config: VelocityEstimateConfig, rng: np.random.Generator | None = None):
        self.config = config
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self.reset()

    def reset(self) -> None:
        self._previous: tuple[float, float, float] | None = None
        self._filtered = np.zeros(3)
        self._delay: deque[np.ndarray] = deque(
            [np.zeros(3)] * self.config.latency_steps, maxlen=self.config.latency_steps + 1)

    def update(self, pose: tuple[float, float, float]) -> np.ndarray:
        """Feed the pose after one physics step; returns the current (delayed) estimate."""
        if self._previous is not None:
            raw = body_velocity_from_poses(self._previous, pose, self.config.dt)
            if any(self.config.noise_std):
                raw = raw + self.rng.normal(0.0, self.config.noise_std)
            self._filtered = self._filtered + self.config.alpha * (raw - self._filtered)
        self._previous = (float(pose[0]), float(pose[1]), float(pose[2]))
        self._delay.append(self._filtered.copy())
        return self._delay[0].copy()
