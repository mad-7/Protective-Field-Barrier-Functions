"""Metrics for the WP0 gates. Pure functions; no simulator, no file I/O.

Thresholds live in `preregistration/gates.yaml` and are never hard-coded here: these
functions compute the quantities, the gate scripts compare them against the committed
criteria. Every function is exercised on synthetic data with a known answer in
`tests/test_gate_metrics.py` before it is applied to a real run (hard rule 6).

Units: metres, seconds, radians in, degrees out only where a criterion is stated in
degrees.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Any, TypeAlias

import numpy as np

Vector: TypeAlias = Sequence[float] | np.ndarray
Path: TypeAlias = Sequence[Sequence[float]] | np.ndarray


def unwrap_angles(yaw: Vector) -> np.ndarray:
    """Remove the 2*pi jumps from a yaw series so successive differences are physical.

    A heading logged in (-pi, pi] jumps by 2*pi when it crosses the wrap, which would
    otherwise read as several hundred degrees of drift in a single step.
    """
    series = np.asarray(yaw, dtype=float)
    if series.ndim != 1 or series.size == 0:
        raise ValueError(f"yaw must be a non-empty 1-D series, got shape {series.shape}")
    return np.unwrap(series)


def path_length(positions: Path) -> float:
    """Arc length along a logged (x, y) path, in metres.

    Arc length rather than straight-line displacement, because a profile that drifts
    still travels the distance the gate asks for.
    """
    points = np.asarray(positions, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError(f"positions must be (N, 2), got shape {points.shape}")
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def steady_state_start(
    applied_speed: Vector, target_speed: float, *, tol: float, settle_steps: int
) -> int:
    """First index at which the profile is in steady state.

    The acceleration limiter ramps every command, so the opening steps are a transient
    that says nothing about tracking fidelity. Steady state starts `settle_steps` after
    the applied command first comes within `tol` (relative) of the target, which is the
    earliest moment the commanded speed is actually the target speed.

    Raises if the target is never reached: a profile that never got up to speed has no
    steady state, and averaging over its ramp would flatter it.
    """
    if target_speed <= 0.0:
        raise ValueError(f"target_speed must be positive, got {target_speed}")
    if settle_steps < 0:
        raise ValueError(f"settle_steps must be >= 0, got {settle_steps}")
    series = np.asarray(applied_speed, dtype=float)
    reached = np.flatnonzero(np.abs(series - target_speed) <= tol * target_speed)
    if reached.size == 0:
        raise ValueError(
            f"the applied speed never came within {tol:.3f} relative of {target_speed} "
            f"(closest was {float(np.min(np.abs(series - target_speed))):.4f} away); "
            "the profile has no steady state to measure"
        )
    start = int(reached[0]) + settle_steps
    if start >= series.size:
        raise ValueError(
            f"steady state would start at index {start} but the profile is only "
            f"{series.size} steps long; run the profile for longer"
        )
    return start


def relative_speed_error(measured: Path, commanded: Path) -> float:
    """Mean |measured speed - commanded speed| / commanded speed over the rows given.

    Speeds are magnitudes of the planar velocity, so a diagonal profile is scored on the
    speed it was asked for rather than per axis. Feed this the steady-state slice only.
    """
    m = np.asarray(measured, dtype=float)
    c = np.asarray(commanded, dtype=float)
    if m.shape != c.shape or m.ndim != 2 or m.shape[1] != 2:
        raise ValueError(f"measured and commanded must both be (N, 2); got {m.shape}, {c.shape}")
    if m.size == 0:
        raise ValueError("no samples to average over")
    speed_c = np.linalg.norm(c, axis=1)
    if np.any(speed_c <= 0.0):
        raise ValueError("a commanded speed of zero has no relative error")
    speed_m = np.linalg.norm(m, axis=1)
    return float(np.mean(np.abs(speed_m - speed_c) / speed_c))


def signed_heading_drift_deg(yaw: Vector, commanded_yaw_rate: float, dt: float) -> float:
    """Signed degrees by which the realised heading change departs from the commanded one.

    Positive is counter-clockwise (about +z), the body frame's sense of yaw. The sign is
    what a *comparison* between two drifts needs: two runs that drift 0.8 deg in opposite
    directions differ by 1.6 deg, not by zero (A10 probe review, 2026-09-19).
    """
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")
    unwrapped = unwrap_angles(yaw)
    steps = unwrapped.size - 1
    realised = float(unwrapped[-1] - unwrapped[0])
    commanded = commanded_yaw_rate * dt * steps
    return math.degrees(realised - commanded)


def heading_drift_deg(yaw: Vector, commanded_yaw_rate: float, dt: float) -> float:
    """Degrees by which the realised heading change departs from the commanded one.

    The magnitude of `signed_heading_drift_deg`, which is what an absolute threshold such
    as G1's 5 deg names. For a pure-translation profile `commanded_yaw_rate` is zero; for
    a profile that deliberately rotates, the drift is measured against the rotation that
    was asked for, which is the same criterion applied to a non-zero command.

    Never compare two of these values with each other -- the sign is gone. Compare signed
    drifts with `drift_band_holds`.
    """
    return abs(signed_heading_drift_deg(yaw, commanded_yaw_rate, dt))


def drift_band_holds(drift_deg: float, reference_deg: float, band_deg: float) -> bool:
    """Whether a signed drift differs from a signed reference drift by at most the band.

    A10's third criterion: a candidate step's open-loop drift may differ from the frozen
    step's by at most the solver-sensitivity band. That band was measured on three drifts
    of one sign (+0.47, +1.11, +0.84 deg, D7), so it is a band on signed values; comparing
    magnitudes lets an equal drift in the opposite direction pass as "no difference".
    """
    if band_deg < 0.0:
        raise ValueError(f"band must be non-negative, got {band_deg}")
    return abs(drift_deg - reference_deg) <= band_deg


def detection_lag_steps(contact: Sequence[bool], distance: Vector, *, touch_at: float = 0.0) -> int:
    """Steps between the geometric distance reaching `touch_at` and contact being reported.

    Positive means the contact check fired late, negative means early, zero means the same
    step. `distance` is an independent geometric measurement (`sim.checkDistance`), not a
    prediction from the commanded motion: predicting the contact step from the commanded
    speed would fold in the acceleration limiter and the wheel radius, turning a test of
    the contact check into a test of the motion model.
    """
    flags = np.asarray(contact, dtype=bool)
    gap = np.asarray(distance, dtype=float)
    if flags.shape != gap.shape or flags.ndim != 1:
        raise ValueError(f"contact and distance must be the same 1-D length; "
                         f"got {flags.shape}, {gap.shape}")
    touched = np.flatnonzero(gap <= touch_at)
    fired = np.flatnonzero(flags)
    if touched.size == 0:
        raise ValueError(
            f"the geometric distance never reached {touch_at}; the approach did not make "
            "contact, so there is no lag to measure"
        )
    if fired.size == 0:
        raise ValueError(
            "the contact check never fired although the geometric distance reached "
            f"{touch_at}; this is a missed contact, not a lag"
        )
    return int(fired[0]) - int(touched[0])


def count_contact_steps(contact: Sequence[bool]) -> int:
    """How many steps reported contact. The negative case requires this to be zero."""
    return int(np.count_nonzero(np.asarray(contact, dtype=bool)))


def diagnose_direction(
    pose_along_command: float,
    wheel_along_command: float,
    target_speed: float,
    *,
    stall_fraction: float = 0.05,
) -> str:
    """Interpret two independent readings of which way the robot actually went.

    `pose_along_command` projects the mean body velocity *derived from the pose* onto the
    commanded direction; `wheel_along_command` does the same for the velocity
    *reconstructed from the wheel joints*. Both are in m/s, signed.

    The two cannot separate an inverted `wheel_sign` from a `yaw_offset` that is pi out,
    and this function does not pretend otherwise: `body_from_wheels` divides by the
    *configured* sign vector, so a wrong sign vector makes the wheel reconstruction return
    the command that was intended while the pose shows the opposite. Both faults therefore
    produce the same disagreement. What separates them is the wheel layout: if the joints
    named fl and fr measure at body +x, the frame is right and the sign vector is not.
    """
    if target_speed <= 0.0:
        raise ValueError(f"target_speed must be positive, got {target_speed}")
    if abs(pose_along_command) < stall_fraction * target_speed:
        return ("the robot barely moved: check that nothing else writes the rolling joints "
                "and that they are in velocity control mode")
    if pose_along_command > 0 and wheel_along_command > 0:
        return "motion follows the command"
    if pose_along_command < 0 < wheel_along_command:
        return ("the robot moved opposite to the command while the joints report the "
                "command that was asked for: robot.wheel_sign is inverted, or "
                "robot.yaw_offset is pi out. The wheel layout test separates them - front "
                "wheels measuring at body +x means the frame is right and the sign vector "
                "is wrong, so flip all four entries")
    if pose_along_command < 0 and wheel_along_command < 0:
        return ("the joints are running opposite to the command and the robot follows "
                "them: something other than this controller is driving the rolling joints")
    return ("the joints report the opposite of both the command and the realised motion, "
            "which the motion does follow: a joint read-back sign issue, not an actuation "
            "one")


def stop_index(speeds: Vector, *, threshold: float) -> int:
    """First index at which the speed series has fallen to `threshold` or below.

    Raises if it never does: a brake that did not bring the robot to rest within the
    logged window has no stopping distance, and truncating the window would report a
    shorter one than the truth.
    """
    if threshold < 0.0:
        raise ValueError(f"threshold must be >= 0, got {threshold}")
    series = np.asarray(speeds, dtype=float)
    if series.ndim != 1 or series.size == 0:
        raise ValueError(f"speeds must be a non-empty 1-D series, got shape {series.shape}")
    stopped = np.flatnonzero(series <= threshold)
    if stopped.size == 0:
        raise ValueError(
            f"the speed never fell to {threshold} m/s (lowest was {float(series.min()):.4f}); "
            "the robot did not stop inside the logged window"
        )
    return int(stopped[0])


def effective_deceleration(initial_speed: float, braking_distance: float) -> float:
    """The constant deceleration that would cover `braking_distance` from `initial_speed`.

    Defined as v^2 / (2 d), which is the quantity the PFB's stopping-distance model uses:
    `stop_distance = v * t_react + v^2 / (2 * a_brake) + margin`. Measuring a mean
    deceleration over time instead would answer a different question, because what the
    barrier needs to know is how much *distance* a given speed costs, not how much time.
    """
    if initial_speed <= 0.0:
        raise ValueError(f"initial_speed must be positive, got {initial_speed}")
    if braking_distance <= 0.0:
        raise ValueError(f"braking_distance must be positive, got {braking_distance}")
    return float(initial_speed ** 2 / (2.0 * braking_distance))


def mean_deceleration(speeds: Vector, dt: float) -> float:
    """Average rate at which speed was shed, in m/s^2. Reported alongside the effective one.

    A brake that is not constant gives a different answer here than `effective_deceleration`
    does, and the gap between the two says how far from constant it was.
    """
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")
    series = np.asarray(speeds, dtype=float)
    if series.ndim != 1 or series.size < 2:
        raise ValueError(f"need at least two samples, got shape {series.shape}")
    return float((series[0] - series[-1]) / (dt * (series.size - 1)))


def braking_distance(positions: Path, onset_index: int, stop_steps: int) -> float:
    """Path travelled while braking, in metres.

    `onset_index` is the index of the **first sample logged after** the brake was
    commanded, and `stop_steps` how many further samples until the robot was at rest.

    The slice starts one sample *earlier*, and that is the whole point of this function.
    A sample logged at step k records the pose after step k has been simulated, so the
    travel produced by the very first braking step sits between sample `onset_index - 1`
    and `onset_index`. Starting the path at `onset_index` discards it. On the 2026-09-17
    run that was 11.8 mm of a 16.7 mm stop: the distance came out 70 per cent short and the
    deceleration correspondingly inflated, to physically impossible values.
    """
    points = np.asarray(positions, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError(f"positions must be (N, 2), got shape {points.shape}")
    if onset_index < 1:
        raise ValueError(
            f"onset_index must be at least 1, got {onset_index}: the sample before the "
            "brake is needed, because the first braking step's travel lies before it"
        )
    if stop_steps < 0:
        raise ValueError(f"stop_steps must be >= 0, got {stop_steps}")
    end = onset_index + stop_steps + 1
    if end > len(points):
        raise ValueError(
            f"the run stops at sample {end} but only {len(points)} were logged"
        )
    return path_length(points[onset_index - 1:end])


def cross_track_error(point: Vector, origin: Vector, direction: Vector) -> float:
    """Signed perpendicular distance from `point` to the line through `origin`, in metres.

    Positive to the left of `direction`. Signed rather than absolute because the sign is
    what distinguishes a controller that oscillates about the path from one that sits
    offset to one side, and those are different faults.
    """
    p = np.asarray(point, dtype=float)
    o = np.asarray(origin, dtype=float)
    d = np.asarray(direction, dtype=float)
    if p.shape != (2,) or o.shape != (2,) or d.shape != (2,):
        raise ValueError("point, origin and direction must each be 2-D")
    norm = float(np.hypot(d[0], d[1]))
    if norm <= 0.0:
        raise ValueError("direction must be a non-zero vector")
    unit = d / norm
    offset = p - o
    return float(unit[0] * offset[1] - unit[1] * offset[0])


def rms(values: Vector) -> float:
    """Root mean square of a series."""
    series = np.asarray(values, dtype=float)
    if series.size == 0:
        raise ValueError("no values to average")
    return float(np.sqrt(np.mean(series ** 2)))


def steady_speed(travelled: Vector, t: Vector, *, from_m: float) -> float:
    """Mean realised speed over the part of a run after `from_m` metres of travel, m/s.

    Distance over time between the first sample at or past `from_m` and the last sample,
    so the acceleration ramp is excluded. `travelled` is cumulative arc length in metres
    and `t` the matching times in seconds. Used for the realised-speed map (ruling 3 on
    the G1b report), where the gap between commanded and realised speed is the quantity
    of interest; raises if the run never gets past `from_m`, rather than returning a
    speed measured over the ramp.
    """
    d = np.asarray(travelled, dtype=float)
    time = np.asarray(t, dtype=float)
    if d.shape != time.shape or d.ndim != 1:
        raise ValueError(f"travelled and t must be matching 1-D arrays, got {d.shape}, {time.shape}")
    past = np.flatnonzero(d >= from_m)
    if len(past) == 0 or past[0] == len(d) - 1:
        raise ValueError(f"run never travels past {from_m} m with samples to spare "
                         f"(reached {d.max() if len(d) else 0.0:.3f} m)")
    i = int(past[0])
    return float((d[-1] - d[i]) / (time[-1] - time[i]))


def contact_agreement(by_collection: Sequence[bool],
                      by_handle: Sequence[Sequence[str]]) -> dict[str, Any]:
    """Compare per-step contact from the collection check with the per-handle check.

    `by_collection[i]` is RobotContact's verdict at step i; `by_handle[i]` names the
    robot shapes that touched an obstacle at step i when each of the robot's shapes was
    checked as a single handle (which ignores the collidable flag). They agree at a step
    when both say contact or both say none. Ruling 1 on the WP0 results report: after the
    wheel spheres' flag is turned on, the two must agree on every staged contact; a
    disagreement means some shape the contact check cannot see touched an obstacle.
    """
    if len(by_collection) != len(by_handle):
        raise ValueError("the two contact records must cover the same steps")
    disagreements = [
        {"step": i, "collection": bool(c), "handles": list(h)}
        for i, (c, h) in enumerate(zip(by_collection, by_handle))
        if bool(c) != bool(h)
    ]
    return {"steps": len(by_collection), "agree": not disagreements,
            "disagreements": disagreements}
