"""SAC: the learner for L1, L4, L5 and A1 (and L2's base), and the controller checkpoints become.

Built from the `training:` block of the config and nothing else, so two methods differ only in
what their experiment configs say (`filter.enabled`, `reward.k_proximity`, ...), never in code.
Stable-Baselines3's SAC; the network is an MLP on the 82-D observation.

**Determinism is within one process** (CLAUDE.md): the same seed and the same transitions give
the same updates here, bit for bit, on the CPU. Separate invocations against the simulator are
not claimed to repeat.

**Device (ruled 2026-10-29):** every learned arm and seed trains on the GPU, no mixing; training is
statistically, not bit-for-bit, reproducible. `training.device` is `cuda` or `cpu`, never `auto`,
and `cuda` on a host without CUDA is refused: SB3 would otherwise fall back to the CPU silently.
"""
from __future__ import annotations

import zipfile
from pathlib import Path
from typing import Any

import numpy as np
from stable_baselines3 import SAC

#: The `training:` keys SAC reads, with the SB3 argument each one becomes.
_KEYS = {"learning_rate": float, "buffer_size": int, "learning_starts": int, "batch_size": int,
         "tau": float, "gamma": float, "train_freq": int, "gradient_steps": int}


#: The devices a learner may be configured on (ruled 2026-10-29). `auto` is not one: it lets a
#: host without CUDA train on the CPU without saying so, mixing devices across arms or seeds.
DEVICES = ("cuda", "cpu")


def learner_device(name: str) -> str:
    """The configured learner device, checked: one of `DEVICES`, and `cuda` only where CUDA is
    available. Raises rather than falling back."""
    if name not in DEVICES:
        raise ValueError(f"training.device is {name!r}; it must be one of {DEVICES} (ruled "
                         "2026-10-29: one device for every learned arm and seed, no fallback)")
    if name == "cuda":
        import torch
        if not torch.cuda.is_available():
            raise RuntimeError("training.device is 'cuda' but torch sees no CUDA device on this "
                               f"host (torch {torch.__version__}); refusing to train on the CPU")
    return name


def sac_kwargs(training: dict[str, Any]) -> dict[str, Any]:
    """The SB3 keyword arguments for the `training:` block. Every key is required: a missing
    hyperparameter must not fall back to a library default nobody chose."""
    if training.get("algo") != "sac":
        raise ValueError(f"training.algo is {training.get('algo')!r}, not 'sac'")
    missing = sorted((set(_KEYS) | {"net_arch", "ent_coef", "device"}) - set(training))
    if missing:
        raise ValueError(f"training: is missing {missing}")
    kwargs = {k: cast(training[k]) for k, cast in _KEYS.items()}
    ent = training["ent_coef"]
    kwargs["ent_coef"] = ent if isinstance(ent, str) else float(ent)
    kwargs["policy_kwargs"] = {"net_arch": [int(n) for n in training["net_arch"]]}
    kwargs["device"] = learner_device(str(training["device"]))
    return kwargs


def make_sac(env, cfg: dict[str, Any], seed: int, *, tensorboard_log: str | None = None) -> SAC:
    """A fresh SAC learner on `env`, seeded."""
    return SAC("MlpPolicy", env, seed=int(seed), verbose=0, tensorboard_log=tensorboard_log,
               **sac_kwargs(cfg["training"]))


def learner_class(checkpoint: str | Path) -> type[SAC]:
    """The class that saved `checkpoint`: SAC-Lagrangian (L2) if it carries a cost critic, else SAC.
    Loading an L2 checkpoint as plain SAC fails on its extra state, so this is decided, not tried."""
    with zipfile.ZipFile(checkpoint) as archive:
        names = set(archive.namelist())
    if "cost_critic.pth" in names:
        # Imported here: sac_lagrangian imports this module.
        from pfbnav.control.sac_lagrangian import SACLagrangian
        return SACLagrangian
    return SAC


class SACController:
    """A trained checkpoint as a `Controller`: deterministic (the mean action), no exploration.
    The actor alone acts; for L2 the cost critic and the multiplier are training-time only."""

    def __init__(self, checkpoint: str | Path, name: str, device: str = "cpu"):
        path = Path(checkpoint)
        if not path.is_file():
            raise FileNotFoundError(f"no checkpoint at {path}")
        self.name = name
        self.checkpoint = path
        self.model = learner_class(path).load(str(path), device=device)

    def reset(self) -> None:
        """SAC's policy is memoryless; nothing carries between episodes."""

    def act(self, observation: np.ndarray) -> np.ndarray:
        action, _ = self.model.predict(np.asarray(observation, dtype=np.float32),
                                       deterministic=True)
        return np.clip(np.asarray(action, dtype=float).ravel(), -1.0, 1.0)
