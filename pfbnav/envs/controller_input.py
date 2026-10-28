"""What a classical controller is allowed to see (supervisor, 2026-10-16: enforced).

Classical controllers (B1, B2, B3, B6, B7, their recovery, and the zero-action control) receive a
`ClassicalInput` each step, never the environment's `info` dict, which carries privileged
quantities: the ground-truth pose, the emulator's timers and internal flags, contact names. The
object holds exactly four things, as ruled, plus the control period:

  * `stop_active`  -- the certified layer's stop flag, the one the learned policy also observes;
  * `velocity`     -- the estimated body velocity (vx, vy, wz), m/s and rad/s, the same estimate
                      the certified layer uses;
  * `scan`         -- the 120-bin stream the certified layer evaluates (ranges m, bearings rad in
                      the body frame, validity flags; hard rule 5: an invalid bin is not free);
  * `route`        -- the planned route ahead, in the body frame: the remaining polyline from the
                      robot's projection onto it, the lookahead point and the goal.

Every array is a read-only copy and every other field a plain float or bool, so nothing reached
through the object leads back to the environment (`tests/test_controller_inputs.py`). The body
frame is +x forward, +y left; nothing here is a world pose.
"""
from __future__ import annotations

from dataclasses import dataclass, fields

import numpy as np
from numpy.typing import ArrayLike


def _frozen(values: ArrayLike, shape_tail: tuple[int, ...] = ()) -> np.ndarray:
    a = np.array(values, dtype=float, copy=True)
    if shape_tail and a.shape[1:] != shape_tail:
        raise ValueError(f"expected shape (n, {shape_tail}), got {a.shape}")
    a.setflags(write=False)
    return a


@dataclass(frozen=True, slots=True)
class ScanStream:
    """The certified layer's 120-bin stream: ranges m, bearings rad (body frame), validity."""

    ranges: np.ndarray
    bearings: np.ndarray
    valid: np.ndarray

    @classmethod
    def of(cls, ranges: ArrayLike, bearings: ArrayLike, valid: ArrayLike) -> ScanStream:
        r, b = _frozen(ranges), _frozen(bearings)
        v = np.array(valid, dtype=bool, copy=True)
        v.setflags(write=False)
        if not (r.shape == b.shape == v.shape) or r.ndim != 1:
            raise ValueError("ranges, bearings and valid must be equal-length vectors")
        return cls(r, b, v)


@dataclass(frozen=True, slots=True)
class RouteView:
    """The route ahead in the body frame, m: `points` from the robot's projection on the route
    to the goal (at least two), the lookahead point, the goal, and the arc length remaining."""

    points: np.ndarray
    lookahead: np.ndarray
    goal: np.ndarray
    remaining_m: float

    @classmethod
    def of(cls, points: ArrayLike, lookahead: ArrayLike, goal: ArrayLike,
           remaining_m: float) -> RouteView:
        p = _frozen(points, (2,))
        if len(p) < 2:
            raise ValueError("a route needs at least two points")
        return cls(p, _frozen(lookahead), _frozen(goal), float(remaining_m))


@dataclass(frozen=True, slots=True)
class ClassicalInput:
    """One step's input to a classical controller. `dt` is the control period, s."""

    stop_active: bool
    velocity: np.ndarray
    scan: ScanStream
    route: RouteView
    dt: float

    @classmethod
    def of(cls, *, stop_active: bool, velocity: ArrayLike, scan: ScanStream, route: RouteView,
           dt: float) -> ClassicalInput:
        v = _frozen(velocity)
        if v.shape != (3,):
            raise ValueError("velocity is (vx, vy, wz)")
        if not isinstance(scan, ScanStream) or not isinstance(route, RouteView):
            raise TypeError("scan and route must be ScanStream and RouteView")
        return cls(bool(stop_active), v, scan, route, float(dt))


ALLOWED_FIELDS = {
    ClassicalInput: ("stop_active", "velocity", "scan", "route", "dt"),
    ScanStream: ("ranges", "bearings", "valid"),
    RouteView: ("points", "lookahead", "goal", "remaining_m"),
}


def field_names(kind: type) -> tuple[str, ...]:
    """The declared fields of one of the input types, in order."""
    return tuple(f.name for f in fields(kind))
