"""How far the velocity ESTIMATE can exceed the command (ruled 2026-10-16, recovery speed check).

Classical recovery moves at 0.08 m/s so that the robot stays in band 0 (top 0.10 m/s), the band
whose field the restart condition checked. The emulator selects the band from the ESTIMATED
velocity, so the question is whether that estimate can exceed the command by enough to cross 0.10.

The measure is the largest ratio, over a run, of the estimated speed at step k to the largest
command issued before step k. A ratio above 1 is overshoot; recovery is safe while
`speed x ratio < band 0's top`. The estimate is the environment's own `PoseDifferenceEstimator`,
replayed over recorded ground-truth poses.
"""
from __future__ import annotations

from typing import Any

import numpy as np
from numpy.typing import ArrayLike

from pfbnav.sim.velocity_estimate import PoseDifferenceEstimator, VelocityEstimateConfig


def estimated_speeds(poses: ArrayLike, config: VelocityEstimateConfig) -> np.ndarray:
    """The estimator's speed, m/s, after each world pose (x, y, yaw) of a run, in order."""
    est = PoseDifferenceEstimator(config)
    return np.array([float(np.hypot(*est.update((float(p[0]), float(p[1]), float(p[2])))[:2]))
                     for p in np.asarray(poses, dtype=float)])


def overshoot_ratio(poses: ArrayLike, command_speeds: ArrayLike,
                    config: VelocityEstimateConfig) -> dict[str, Any]:
    """{'ratio', 'step', 'max_estimate', 'max_command'} for one run.

    `command_speeds[k]` is the translational command, m/s, issued at step k; the pose after it is
    `poses[k + 1]`. The ratio compares the estimate at step k with the largest command issued at
    steps before k.
    """
    e = estimated_speeds(poses, config)
    c = np.asarray(command_speeds, dtype=float)
    if c.shape != e.shape:
        raise ValueError("one command per pose")
    issued = np.maximum.accumulate(c)[:-1]
    ratio = e[1:] / np.maximum(issued, 1e-9)
    k = int(np.argmax(ratio))
    return {"ratio": float(ratio[k]), "step": k + 1, "max_estimate": float(e.max()),
            "max_command": float(c.max())}
