"""The robot's footprint extent e(psi) from a 1-degree table. Pure functions.

Ruling 3 on the WP0 results report: the footprint the PFB uses is the measured envelope
(the collision-probe sweep), stored as a table of e(psi) at every whole degree and
interpolated by a periodic cubic spline, which gives e and de/dpsi everywhere.

e(psi) is the distance from the body origin to the footprint's edge along body bearing
psi -- the term the barrier subtracts from the free range in that sector. The table is
built so that the spline never falls below the true envelope between knots
(`pfbnav.sim.footprint.footprint_table_from_polygon`); under-stating e is the unsafe
direction, since the filter would then treat an obstacle already touching the robot as
clear.

No simulator imports: `pfbnav/safety/` stays pure, and this module may be used by both
`pfb.py` and `certified.py` without either importing the other.

Frames: body frame, +x forward, +y left; bearings in radians, any value (wrapped).
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline

TABLE_STEP_DEG = 1.0
TABLE_SIZE = 360


class FootprintTable:
    """e(psi) and de/dpsi from 360 values at bearings -180, -179, ..., 179 degrees."""

    def __init__(self, extents: np.ndarray | list[float]):
        values = np.asarray(extents, dtype=float)
        if values.shape != (TABLE_SIZE,):
            raise ValueError(f"need {TABLE_SIZE} extents, one per degree, got {values.shape}")
        if np.any(values <= 0.0):
            raise ValueError("every extent must be positive: the origin is inside the robot")
        self.extents = values
        knots = np.radians(np.arange(-180.0, 180.0 + TABLE_STEP_DEG / 2, TABLE_STEP_DEG))
        # Periodic: the value at +180 deg repeats the value at -180 deg.
        self._spline = CubicSpline(knots, np.append(values, values[0]), bc_type="periodic")

    @staticmethod
    def _wrap(psi: float | np.ndarray) -> float | np.ndarray:
        return (np.asarray(psi) + math.pi) % (2.0 * math.pi) - math.pi

    def extent(self, psi: float | np.ndarray) -> float | np.ndarray:
        """e(psi), metres."""
        out = self._spline(self._wrap(psi))
        return float(out) if np.ndim(out) == 0 else out

    def derivative(self, psi: float | np.ndarray) -> float | np.ndarray:
        """de/dpsi, metres per radian."""
        out = self._spline(self._wrap(psi), 1)
        return float(out) if np.ndim(out) == 0 else out

    def __call__(self, psi: float) -> tuple[float, float]:
        """(e, de/dpsi) at one bearing, the pair `pfb.footprint_extent` returns."""
        return float(self.extent(psi)), float(self.derivative(psi))

    @property
    def inscribed(self) -> float:
        """Smallest knot extent, m (the spline's minimum is within the build margin)."""
        return float(self.extents.min())

    @property
    def circumscribed(self) -> float:
        """Largest knot extent, m."""
        return float(self.extents.max())

    def to_csv(self, path: str | Path, header: dict[str, str] | None = None) -> None:
        """Write the table, with `header` entries as leading `# key: value` lines."""
        lines = [f"# {k}: {v}" for k, v in (header or {}).items()]
        with Path(path).open("w", newline="") as f:
            f.writelines(line + "\n" for line in lines)
            writer = csv.writer(f)
            writer.writerow(["bearing_deg", "extent_m"])
            for deg, e in zip(range(-180, 180), self.extents):
                writer.writerow([deg, repr(float(e))])

    @classmethod
    def from_csv(cls, path: str | Path) -> FootprintTable:
        """Read a table written by `to_csv`; the bearings must be -180..179 in order."""
        with Path(path).open(newline="") as f:
            rows = [r for r in csv.reader(line for line in f if not line.startswith("#"))]
        if rows[0] != ["bearing_deg", "extent_m"]:
            raise ValueError(f"unexpected header {rows[0]} in {path}")
        bearings = [int(r[0]) for r in rows[1:]]
        if bearings != list(range(-180, 180)):
            raise ValueError(f"{path} must list bearings -180..179 in order, one per degree")
        return cls(np.array([float(r[1]) for r in rows[1:]]))
