"""The scan plausibility guard (the scan-fault fix, ruled 2026-10-23). Pure.

Under heavy CPU load the vision-sensor render can come out corrupted: on 2026-10-23 (traced rerun,
scenario 2000018) two consecutive packets with the robot still had 104-114 of 120 bins read empty --
the walls gone -- and phantom returns at 0.30-0.47 m near the scanner corners. Fresh packets of the
current tick, so the staleness check passed them. A corrupted render can create a stop that never
should have been (into the primary metric) or hide a real obstacle (a missed stop).

**The check.** In a static world one step of motion bounds how far a scan can change: each bin's
range must lie within the range spanned by the LAST ACCEPTED scan's bins within `neighbour_bins` of
it (bearings shift with rotation, occlusion edges sweep), widened by `range_tolerance` (one step of
translation at the top speed plus noise). Both widen in proportion to the steps since that scan was
accepted. Bins valid in both scans are compared; a newly unobserved bin needs no test (it already
breaches every field, hard rule 5). `corrupt_bins` or more bins outside the bound make the packet
implausible.

**What the step loop does with it** (`pfbnav/envs/step_loop.py`): up to `rerenders` re-renders of the
same tick; the first plausible packet is used; if none, every bin is marked unobserved for that step
(`all_unobserved`: a fail-safe stop), counted as a scan fault. Stops decided on a fail-safed packet are
counted apart and excluded from H1's primary stop count (ruled 2026-10-23).
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import ArrayLike

from pfbnav.sim.scan_packet import ScanPacket


def out_of_bounds(prev_ranges: ArrayLike, prev_valid: ArrayLike, new_ranges: ArrayLike,
                  new_valid: ArrayLike, *, neighbour_bins: int, range_tolerance: float) -> int:
    """How many bins, valid in both scans, fall outside [min - tol, max + tol] of the previous
    scan's valid bins within `neighbour_bins` (circularly). A bin whose neighbourhood holds no
    valid previous range is not tested."""
    p = np.asarray(prev_ranges, dtype=float)
    pv = np.asarray(prev_valid, dtype=bool)
    r = np.asarray(new_ranges, dtype=float)
    rv = np.asarray(new_valid, dtype=bool)
    if not (p.shape == pv.shape == r.shape == rv.shape) or p.ndim != 1:
        raise ValueError("scans must be 1-D and of one length")
    n = p.size
    idx = (np.arange(n)[:, None] + np.arange(-neighbour_bins, neighbour_bins + 1)[None, :]) % n
    vals, ok = p[idx], pv[idx]
    lo = np.where(ok, vals, np.inf).min(axis=1) - range_tolerance
    hi = np.where(ok, vals, -np.inf).max(axis=1) + range_tolerance
    tested = rv & pv & ok.any(axis=1)
    return int(np.sum(tested & ((r < lo) | (r > hi))))


def all_unobserved(packet: ScanPacket) -> ScanPacket:
    """The fail-safe packet: every bin of both streams unobserved (ranges 0.0, as a dropped bin
    reads -- never range_max)."""
    def blank(a: np.ndarray) -> np.ndarray:
        out = np.zeros_like(a)
        out.setflags(write=False)
        return out

    def invalid(a: np.ndarray) -> np.ndarray:
        out = np.zeros_like(a, dtype=bool)
        out.setflags(write=False)
        return out
    return dataclasses.replace(packet, policy_ranges=blank(packet.policy_ranges),
                               policy_valid=invalid(packet.policy_valid),
                               filter_ranges=blank(packet.filter_ranges),
                               filter_valid=invalid(packet.filter_valid))


@dataclass
class ScanGuard:
    """The last accepted scan and the thresholds (`scanners.plausibility` in the config)."""

    neighbour_bins: int
    range_tolerance: float
    corrupt_bins: int
    rerenders: int
    _ranges: np.ndarray | None = None
    _valid: np.ndarray | None = None
    _since: int = 0

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> ScanGuard:
        g = cfg["scanners"]["plausibility"]
        return cls(neighbour_bins=int(g["neighbour_bins"]),
                   range_tolerance=float(g["range_tolerance_m"]),
                   corrupt_bins=int(g["corrupt_bins"]), rerenders=int(g["rerenders"]))

    def reset(self, packet: ScanPacket) -> None:
        """An episode's first scan is the reference (nothing to compare it with)."""
        self.accept(packet)

    def outside(self, packet: ScanPacket) -> int:
        if self._ranges is None or self._valid is None:
            return 0
        m = self._since + 1
        return out_of_bounds(self._ranges, self._valid, packet.filter_ranges,
                             packet.filter_valid, neighbour_bins=self.neighbour_bins * m,
                             range_tolerance=self.range_tolerance * m)

    def plausible(self, packet: ScanPacket) -> bool:
        return self.outside(packet) < self.corrupt_bins

    def accept(self, packet: ScanPacket) -> None:
        self._ranges = np.array(packet.filter_ranges, dtype=float)
        self._valid = np.array(packet.filter_valid, dtype=bool)
        self._since = 0

    def reject(self) -> None:
        self._since += 1
