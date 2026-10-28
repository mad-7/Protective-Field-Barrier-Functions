"""The order in which a training seed visits the training set (ruled 2026-10-03).

Seed index k draws its OWN permutation of the training set, and every method's seed k uses that
same permutation: seeds carry all training randomness, including order, or the method-level
inference (seeds as the replicates of a method) is miscalibrated. The permutation is a function of
k alone -- not of the method, the config or anything else -- drawn from its own named stream so it
cannot collide with any other use of the seed. No curriculum in v2: the order is a uniform
permutation, nothing is sorted by difficulty.
"""
from __future__ import annotations

import hashlib

import numpy as np

#: A fixed label folded into the seed, so the order's stream is separate from every other stream
#: seeded by the same k (network initialisation, exploration, the dropout arc).
ORDER_STREAM = 0x0D3E5


def scenario_order(n: int, seed: int) -> np.ndarray:
    """The permutation of range(n) that training seed `seed` uses."""
    if n <= 0:
        raise ValueError("a training set needs at least one scenario")
    return np.random.default_rng([ORDER_STREAM, int(seed)]).permutation(n)


def order_sha256(order: np.ndarray) -> str:
    """A short identity for a permutation, recorded with every training run."""
    return hashlib.sha256(np.asarray(order, dtype=np.int64).tobytes()).hexdigest()
