"""The interfaces controllers present to the evaluation runner (WP3-WP5).

Every method is compared on the same scenarios in the same process (`docs/reproducibility.md`). An
episode begins with `reset`, and each step yields a normalised action (a_vx, a_vy, a_wz) in
[-1, 1]^3, which the environment scales by each axis's own ceiling. What a controller is GIVEN
depends on its kind (`input_kind`):

  * `"observation"` -- the learned policies: the 82-D observation, nothing else.
  * `"classical"`   -- the classical baselines, their recovery and the zero-action control: a typed
                       `ClassicalInput` (stop flag, velocity estimate, 120-bin stream, route), never
                       the environment's `info` dict (ruled 2026-10-16; `pfbnav/envs/controller_input.py`).
                       A classical controller also exposes `command`, its body-frame command in m/s
                       and rad/s, and `ceilings`, so that the recovery wrapper can read the
                       direction it is asking for.
"""
from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class Controller(Protocol):
    name: str

    def reset(self) -> None:
        """Forget anything carried from the previous episode."""

    def act(self, observation: np.ndarray) -> np.ndarray:
        """A normalised action, shape (3,), in [-1, 1]."""
        ...


@runtime_checkable
class ClassicalController(Protocol):
    name: str
    input_kind: str            # "classical"
    ceilings: np.ndarray       # (v_max, vy_max, w_max), m/s and rad/s

    def reset(self) -> None:
        """Forget anything carried from the previous episode."""

    def command(self, inp) -> np.ndarray:
        """The body-frame command (vx, vy, wz), m/s and rad/s, from a `ClassicalInput`."""
        ...

    def act(self, inp) -> np.ndarray:
        """`command` normalised by `ceilings` and clipped to [-1, 1]."""
        ...


def input_kind(controller) -> str:
    """"classical" or "observation" (the default, for the learned policies)."""
    kind = getattr(controller, "input_kind", "observation")
    if kind not in ("classical", "observation"):
        raise ValueError(f"{getattr(controller, 'name', controller)}: unknown input kind {kind!r}")
    return kind
