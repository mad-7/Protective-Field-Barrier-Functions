"""The scan packet: one tick's scan, assembled in the simulator and decoded here.

This module is the **contract** between `lua/scanner_v2.lua` and the environment, and it
is written before the Lua so the Lua has something to satisfy. Pure functions: no
simulator imports, no I/O.

Why a packet at all (measured 2026-09-20): reading the four vision sensors over the
remote API costs 14-17 ms per step in round trips alone, and the cost is latency rather
than data -- the smallest buffer measured was the slowest. The scan is therefore binned
inside the simulator and returned as **one buffer signal per tick**, ~1.7 kB, as v1 did
with its 36 bins.

Two streams, from one scan (`configs/base.yaml`, `scanners`):

  * **policy** -- `policy_bins` (36) uniform sectors over the full turn, the observation
    the learner sees. Bearings are implied by the binning: bin i is centred at
    -pi + (i + 0.5) * 2pi / n, so they are derived here rather than transmitted.
  * **filter** -- `filter_bins` (120) with their bearings transmitted explicitly, because
    the PFB evaluates its barrier at the bearing each range was actually measured at, and
    a bin centre is not that bearing.

**One tick.** A scan stitched from sensors read at different simulation times is not a
scan of one instant, and the robot has moved between them. The packet carries
`sim_time`: **the simulation time of the state the scan describes**, which is the end of
the step during which it was taken. The scanner runs in the simulator's sensing phase,
after that step's physics, where the simulator's own clock still reads the time the step
*started*; stamping that would put every scan one period in the past, as it did before
version 2 (measured 2026-09-21). A client that has just stepped compares `sim_time`
against its own clock and they agree.

**Validity, not free space (hard rule 5).** A bin is valid when the scan actually covered
that bearing. A covered bin with no return inside `range_max` is valid at `range_max`:
that is a measurement of free space. A bin no ray covered -- a blind spot between the
pods, or a sector occluded by the robot itself -- is **invalid**, and the certified layer
and the PFB treat it as breached rather than free. The distinction cannot be recovered
from the range alone, which is why validity is transmitted.

Frames and units: **body frame**, +x chassis-forward, +y left; bearings in (-pi, pi]
measured from +x; ranges in metres from the body origin, not from a sensor. The origin is
`youBot_ref`'s, but its axes are not: the body frame is `youBot_ref` rotated by
`robot.yaw_offset` (pi/2), and the scanner applies that rotation before taking bearings.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

# Bumped whenever the layout below changes. The Lua writes it, the decoder refuses
# anything else: a scanner and an environment that disagree about the layout would
# otherwise produce plausible, wrong ranges.
FORMAT_VERSION = 2

# Header, in order. Six floats, then the arrays.
HEADER_FIELDS = ("version", "sim_time", "n_policy", "n_filter", "range_min", "range_max")
HEADER_LEN = len(HEADER_FIELDS)

# Validity travels as a float (the packet is one float table); anything else is an error.
VALID, INVALID = 1.0, 0.0

# Bearings and ranges are compared against the configuration with this tolerance, m/rad.
TOL = 1e-6


def bin_bearings(n: int) -> np.ndarray:
    """Centre bearings of `n` uniform sectors over the full turn, radians in (-pi, pi]."""
    if n < 1:
        raise ValueError(f"need at least one bin, got {n}")
    edges = -math.pi + 2.0 * math.pi * (np.arange(n) + 0.5) / n
    return edges


@dataclass(frozen=True)
class ScanPacket:
    """One tick's scan, decoded. Arrays are read-only views over the decoded buffer."""

    sim_time: float
    policy_ranges: np.ndarray      # m, (policy_bins,)
    policy_valid: np.ndarray       # bool, (policy_bins,)
    filter_bearings: np.ndarray    # rad in (-pi, pi], (filter_bins,), ascending
    filter_ranges: np.ndarray      # m, (filter_bins,)
    filter_valid: np.ndarray       # bool, (filter_bins,)
    range_min: float
    range_max: float

    @property
    def policy_bearings(self) -> np.ndarray:
        """Centre bearings of the policy sectors, rad -- implied by the binning."""
        return bin_bearings(self.policy_ranges.size)

    @property
    def fully_observed(self) -> bool:
        """Whether every bin of both streams was covered by the scan."""
        return bool(self.policy_valid.all() and self.filter_valid.all())


def encode_scan(sim_time: float, policy_ranges: ArrayLike, policy_valid: ArrayLike,
                filter_bearings: ArrayLike, filter_ranges: ArrayLike,
                filter_valid: ArrayLike, *, range_min: float,
                range_max: float) -> list[float]:
    """Build a packet's float table, in the layout `lua/scanner_v2.lua` must produce.

    Exists so the format has one definition rather than two: the tests and the fake
    scanner encode with this, the Lua mirrors it, and `decode_scan` is what the
    environment actually uses.
    """
    pr = np.asarray(policy_ranges, dtype=float)
    pv = np.asarray(policy_valid, dtype=bool)
    fb = np.asarray(filter_bearings, dtype=float)
    fr = np.asarray(filter_ranges, dtype=float)
    fv = np.asarray(filter_valid, dtype=bool)
    if pr.shape != pv.shape or pr.ndim != 1:
        raise ValueError("policy ranges and validity must be 1-D and the same length")
    if not (fb.shape == fr.shape == fv.shape) or fb.ndim != 1:
        raise ValueError("filter bearings, ranges and validity must be 1-D and the same length")
    return [float(FORMAT_VERSION), float(sim_time), float(pr.size), float(fb.size),
            float(range_min), float(range_max),
            *pr.tolist(), *np.where(pv, VALID, INVALID).tolist(),
            *fb.tolist(), *fr.tolist(), *np.where(fv, VALID, INVALID).tolist()]


def _validity(values: np.ndarray, what: str) -> np.ndarray:
    if not np.all((values == VALID) | (values == INVALID)):
        raise ValueError(f"{what} carries a value that is neither {VALID} nor {INVALID}")
    return values == VALID


def decode_scan(values: ArrayLike, *, policy_bins: int, filter_bins: int,
                range_min: float, range_max: float) -> ScanPacket:
    """Decode a packet and check it against the configuration it must match.

    Every mismatch raises: a scan whose length, version, bin counts or range limits differ
    from the configuration is a scanner and an environment that disagree, and the ranges
    it carries cannot be trusted just because they are numbers. An invalid bin's range is
    not checked -- there was no measurement -- but it must still be finite, so a NaN
    cannot travel unnoticed into an observation.
    """
    v = np.asarray(values, dtype=float).ravel()
    expected = HEADER_LEN + 2 * policy_bins + 3 * filter_bins
    if v.size != expected:
        raise ValueError(f"packet has {v.size} floats, the configuration implies {expected} "
                         f"({policy_bins} policy bins, {filter_bins} filter bins)")
    if not np.all(np.isfinite(v)):
        raise ValueError("packet carries a non-finite value")

    version, sim_time, n_policy, n_filter, r_min, r_max = v[:HEADER_LEN]
    if int(version) != FORMAT_VERSION:
        raise ValueError(f"packet format version {int(version)}, this decoder is "
                         f"{FORMAT_VERSION}: the scanner and the environment disagree")
    if int(n_policy) != policy_bins or int(n_filter) != filter_bins:
        raise ValueError(f"packet declares {int(n_policy)}/{int(n_filter)} bins, the "
                         f"configuration asks for {policy_bins}/{filter_bins}")
    if abs(r_min - range_min) > TOL or abs(r_max - range_max) > TOL:
        raise ValueError(f"packet was taken with range [{r_min}, {r_max}] m, the "
                         f"configuration says [{range_min}, {range_max}]")
    if sim_time < 0.0:
        raise ValueError(f"packet carries a negative simulation time {sim_time}")

    i = HEADER_LEN
    policy_ranges = v[i:i + policy_bins]
    i += policy_bins
    policy_valid = _validity(v[i:i + policy_bins], "policy validity")
    i += policy_bins
    filter_bearings = v[i:i + filter_bins]
    i += filter_bins
    filter_ranges = v[i:i + filter_bins]
    i += filter_bins
    filter_valid = _validity(v[i:i + filter_bins], "filter validity")

    for name, ranges, valid in (("policy", policy_ranges, policy_valid),
                                ("filter", filter_ranges, filter_valid)):
        measured = ranges[valid]
        if measured.size and (measured.min() < range_min - TOL
                              or measured.max() > range_max + TOL):
            raise ValueError(f"{name} stream carries a valid range outside "
                             f"[{range_min}, {range_max}] m: "
                             f"[{measured.min()}, {measured.max()}]")
    if filter_bins:
        if np.any(filter_bearings <= -math.pi - TOL) or np.any(filter_bearings > math.pi + TOL):
            raise ValueError("filter bearings must lie in (-pi, pi]")
        if np.any(np.diff(filter_bearings) <= 0.0):
            raise ValueError("filter bearings must be strictly ascending, so a consumer "
                             "can pair them with a field evaluated at the same bearings")

    for array in (policy_ranges, policy_valid, filter_bearings, filter_ranges, filter_valid):
        array.flags.writeable = False
    return ScanPacket(sim_time=float(sim_time), policy_ranges=policy_ranges,
                      policy_valid=policy_valid, filter_bearings=filter_bearings,
                      filter_ranges=filter_ranges, filter_valid=filter_valid,
                      range_min=float(range_min), range_max=float(range_max))


def assert_current(packet: ScanPacket, sim_time: float, dt: float) -> None:
    """Raise unless the packet was taken at the step it is being used for.

    A packet older than one control period is a scan of where the robot was, not of where
    it is; silently accepting one would put a stale obstacle into the observation and the
    barrier. The tolerance is half a step, so a packet from this tick passes and one from
    the previous tick does not.
    """
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")
    age = sim_time - packet.sim_time
    if abs(age) > 0.5 * dt:
        raise ValueError(f"scan packet is {age:+.4f} s from the current step at {sim_time} s "
                         f"(dt {dt} s): the scanner did not run this tick")
