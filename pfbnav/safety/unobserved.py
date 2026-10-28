"""One convention for an unobserved scan bin, shared by every consumer of the 120-bin filter stream
(ruled 2026-10-27): B1 (through the classical recovery, its only use of the scan), B2, B3, B6 and the
PFB.

**An INVALID bin is an obstacle at the footprint boundary along its bearing:** a point at e(psi), the
extent of the footprint the consumer itself judges contact with, on that bin's bearing. Unobserved
space is never free (hard rule 5), and the footprint's edge is the nearest an obstacle can stand on
that bearing without being in contact. A command that moves away from the point is therefore
admissible, one that moves toward it is not, and standing still touches it without entering it.

Each consumer passes its OWN footprint representation (`extent`), so that the point lies exactly on
the boundary it tests against: a consumer that over-states the footprint for safety (B3's and B6's
cell maxima) would otherwise see a point at the exact boundary as already inside.

The certified layer keeps its own rule -- an invalid bin breaches every field (`certified.py`,
`field_breached`) -- which this does not replace: the layer is the emulated certified device, the
consumers here are controllers and the PFB. The environment counts the steps with any invalid bin
(`invalid_bin_steps`, per episode), so the count is reported for every method alike.

Pure; no simulator, no import of `certified.py` or `pfb.py` (hard rule 3 holds either way).
"""
from __future__ import annotations

from collections.abc import Callable

import numpy as np
from numpy.typing import ArrayLike


def resolve_unobserved(ranges: ArrayLike, bearings: ArrayLike, valid: ArrayLike,
                       extent: Callable[[np.ndarray], ArrayLike]) -> tuple[np.ndarray, int]:
    """The ranges with every invalid bin replaced by the footprint's extent on its bearing, m, and
    the number of bins replaced. `extent(bearings)` is the consumer's footprint boundary distance,
    m, along body-frame bearings, rad. Valid bins are returned unchanged (a copy)."""
    r = np.array(ranges, dtype=float, copy=True)
    b = np.asarray(bearings, dtype=float)
    ok = np.asarray(valid, dtype=bool)
    if not (r.shape == b.shape == ok.shape) or r.ndim != 1:
        raise ValueError("ranges, bearings and valid must be equal-length vectors")
    n = int(np.count_nonzero(~ok))
    if n:
        edge = np.asarray(extent(b[~ok]), dtype=float).reshape(-1)
        if edge.shape != (n,) or not np.all(np.isfinite(edge)) or np.any(edge <= 0.0):
            raise ValueError("the footprint extent must be finite and positive on every bearing")
        r[~ok] = edge
    return r, n
