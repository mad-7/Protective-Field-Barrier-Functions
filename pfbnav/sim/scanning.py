"""Scanner geometry: how many rays a scan must render to serve the configured streams.

Pure functions. The scanners are built from vision sensors, whose cost per step scales
with the rays they render (measured 2026-09-19: four 256-wide sensors cost ~9.4 ms of a
20.2 ms step, against 36 + 120 = 156 bearings actually consumed). Sizing the sensors is
therefore a compute decision as well as a sensing one, and it is made here rather than by
a number written into a script.
"""
from __future__ import annotations

import math

FULL_TURN_DEG = 360.0


def resolution_for_bins(fov_deg: float, bins_over_360: int, *, oversample: float = 1.0) -> int:
    """Rays one sensor of `fov_deg` must render so a full turn carries `bins_over_360`.

    `oversample` multiplies that: a scan sampled exactly at the bin spacing gives one ray
    per bin, with nothing to spare for a bin whose bearing falls between two rays, so a
    factor above 1 is the honest setting for a stream that needs exact bearings. The
    result is rounded up: a fractional ray does not exist, and rounding down would give a
    stream fewer samples than it was specified with.
    """
    if not 0.0 < fov_deg <= FULL_TURN_DEG:
        raise ValueError(f"field of view must be in (0, 360] deg, got {fov_deg}")
    if bins_over_360 < 1:
        raise ValueError(f"need at least one bin, got {bins_over_360}")
    if oversample <= 0.0:
        raise ValueError(f"oversample must be positive, got {oversample}")
    # Rounded before the ceiling: the scene's 120 deg sensors read 120.0000033 deg -- the
    # angle is stored in single precision -- and a ceiling taken on 80.0000022 gives 81
    # rays for a stream that needs 80 (SCANCHECK_20260921T144640Z). Four decimals of a
    # ray count absorbs single-precision noise and is far below one ray.
    exact = round(bins_over_360 * oversample * fov_deg / FULL_TURN_DEG, 4)
    return max(1, math.ceil(exact))


def rays_per_step(resolutions: list[int]) -> int:
    """Total rays rendered per step by a set of sensors, one entry per sensor."""
    if any(r < 1 for r in resolutions):
        raise ValueError("every sensor renders at least one ray")
    return int(sum(resolutions))
