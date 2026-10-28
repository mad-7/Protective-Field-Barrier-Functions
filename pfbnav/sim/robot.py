"""KUKA youBot mecanum base: kinematics, actuation limits, pose readout.

Frames and units
----------------
Body frame: +x forward, +y left, +z up. Linear velocities in m/s, yaw rate in
rad/s, wheel speeds in rad/s, lengths in m. Yaw is wrapped to (-pi, pi].

Wheel order is ``[fl, fr, rl, rr]`` everywhere, including `robot.wheel_sign`
and `sim.objects.wheels` in the configuration.

The mecanum mixing is migrated unchanged from v1 (`YouBot/YouBotSimModel.py`);
the wheel sign vector and the base_ref yaw offset are *not* migrated, because
they describe the v1 scene. Both are configuration values marked VERIFY and are
re-measured on the v2 scene by gate G1.

Everything above `YouBotBase` is a pure function of its arguments and carries no
simulator dependency.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, TypeAlias

import numpy as np

# Any 1-D run of floats: a list, a tuple, or a numpy array.
Vector: TypeAlias = Sequence[float] | np.ndarray

WHEEL_ORDER: tuple[str, ...] = ("fl", "fr", "rl", "rr")

# Sign of each (vx, vy, (L+W)*wz) contribution to each wheel, in WHEEL_ORDER:
#   w_i = (sx_i * vx + sy_i * vy + sw_i * (L + W) * wz) / r
_MIX_SIGNS = np.array(
    [
        [1.0, -1.0, -1.0],  # fl
        [1.0, +1.0, +1.0],  # fr
        [1.0, +1.0, -1.0],  # rl
        [1.0, -1.0, +1.0],  # rr
    ]
)


@dataclass(frozen=True)
class ChassisGeometry:
    """Chassis and drive-train geometry. Lengths in m, yaw_offset in rad."""

    half_length: float       # chassis half-extent along body +x
    half_width: float        # chassis half-extent along body +y
    wheel_radius: float      # m
    half_wheelbase: float    # L: half distance between front and rear axles
    half_track: float        # W: half distance between left and right wheels
    wheel_sign: tuple[float, ...] = (1.0, 1.0, 1.0, 1.0)
    yaw_offset: float = 0.0  # rotation from the base_ref +x axis to chassis forward

    def __post_init__(self) -> None:
        if self.wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be positive")
        if len(self.wheel_sign) != 4:
            raise ValueError(f"wheel_sign needs 4 entries in order {WHEEL_ORDER}")
        if not all(abs(abs(s) - 1.0) < 1e-12 for s in self.wheel_sign):
            raise ValueError(f"wheel_sign entries must be +1 or -1, got {self.wheel_sign}")

    @property
    def lw(self) -> float:
        """L + W, the mecanum yaw lever arm, in m."""
        return self.half_wheelbase + self.half_track

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> ChassisGeometry:
        r = cfg["robot"]
        return cls(
            half_length=float(r["half_length"]),
            half_width=float(r["half_width"]),
            wheel_radius=float(r["wheel_radius"]),
            half_wheelbase=float(r["half_wheelbase"]),
            half_track=float(r["half_track"]),
            wheel_sign=tuple(float(s) for s in r["wheel_sign"]),
            yaw_offset=float(r["yaw_offset"]),
        )


@dataclass(frozen=True)
class ActuationLimits:
    """Commanded-velocity box and acceleration limits."""

    v_max: float         # m/s, bound on |vx|
    vy_max: float        # m/s, bound on |vy|
    w_max: float         # rad/s, bound on |wz|
    a_max: float         # m/s^2, bound on a (vx, vy) change that does not reduce speed
    alpha_max: float     # rad/s^2, bound on the |wz| change
    a_decel_max: float   # m/s^2, bound on a (vx, vy) change that reduces speed

    def __post_init__(self) -> None:
        for name in ("v_max", "vy_max", "w_max", "a_max", "alpha_max", "a_decel_max"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be positive")

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> ActuationLimits:
        a = cfg["actuation"]
        return cls(
            v_max=float(a["v_max"]),
            vy_max=float(a["vy_max"]),
            w_max=float(a["w_max"]),
            a_max=float(a["a_max"]),
            alpha_max=float(a["alpha_max"]),
            a_decel_max=float(a["a_decel_max"]),
        )


@dataclass(frozen=True)
class WheelBudget:
    """The mecanum wheel-speed budget |vx| + |vy| + (l+w)|wz| <= limit (ruling 2, WP0).

    For a mecanum base the fastest wheel's rim speed is exactly |vx| + |vy| + (l+w)|wz|,
    so this is the wheel-speed limit written in body velocities. `v_rim` is the measured
    rim-speed ceiling, and two shares of it are configured:

      * `filter` -- the ENFORCED share, for every method: the base projects every command into
        it, by direction-preserving projection, never by clipping, and the PFB's QP optimises
        directly in that polytope;
      * `nominal` -- a smaller share used only offline, to plan speeds (the free-orientation
        oracle's schedule and the classical route cap). No command is projected into it (ruled
        2026-10-26: one share for all methods; earlier descriptions said nominal controllers
        were projected into it first, which no code did).
    """

    v_rim: float             # m/s, measured rim-speed ceiling (VERIFY: load-dependent)
    nominal_fraction: float  # share of v_rim the offline planners plan with; never enforced
    filter_fraction: float   # share of v_rim enforced on every command
    lever_arm: float         # m, l + w = half_wheelbase + half_track

    def __post_init__(self) -> None:
        if self.v_rim <= 0.0 or self.lever_arm <= 0.0:
            raise ValueError("v_rim and lever_arm must be positive")
        if not 0.0 < self.nominal_fraction < self.filter_fraction <= 1.0:
            raise ValueError("need 0 < nominal_fraction < filter_fraction <= 1, so the "
                             "filter keeps authority the nominal command cannot consume")

    @property
    def nominal_limit(self) -> float:
        return self.nominal_fraction * self.v_rim

    @property
    def filter_limit(self) -> float:
        return self.filter_fraction * self.v_rim

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> WheelBudget:
        b = cfg["actuation"]["wheel_budget"]
        r = cfg["robot"]
        return cls(
            v_rim=float(b["v_rim"]),
            nominal_fraction=float(b["nominal_fraction"]),
            filter_fraction=float(b["filter_fraction"]),
            lever_arm=float(r["half_wheelbase"]) + float(r["half_track"]),
        )


def wheel_budget_use(command: Vector, lever_arm: float) -> float:
    """|vx| + |vy| + (l+w)|wz|, m/s: the fastest wheel's rim speed for this command."""
    cmd = np.asarray(command, dtype=float)
    if cmd.shape != (3,):
        raise ValueError(f"command must be (vx, vy, wz), got shape {cmd.shape}")
    return float(abs(cmd[0]) + abs(cmd[1]) + lever_arm * abs(cmd[2]))


@dataclass(frozen=True)
class BudgetResult:
    applied: np.ndarray   # (vx, vy, wz) within the budget
    scale: float          # factor applied to the whole command; 1.0 when it did not bind

    @property
    def bound(self) -> bool:
        return self.scale < 1.0


def project_to_wheel_budget(command: Vector, limit: float, lever_arm: float) -> BudgetResult:
    """Scale (vx, vy, wz) by one factor so that |vx| + |vy| + (l+w)|wz| <= limit.

    Direction-preserving in all three components: a command that asks for too much keeps
    its heading and its ratio of turning to translating, and only slows. Clipping one
    component would change what the command does, not just how fast.
    """
    if limit <= 0.0:
        raise ValueError(f"limit must be positive, got {limit}")
    cmd = np.asarray(command, dtype=float)
    use = wheel_budget_use(cmd, lever_arm)
    if use <= limit:
        return BudgetResult(cmd.copy(), 1.0)
    scale = limit / use
    return BudgetResult(cmd * scale, scale)


def count_at_torque_cap(torques: Vector, caps: Vector, *, rel_tol: float = 1e-3) -> int:
    """How many wheels are exerting their full torque, |torque| >= cap * (1 - rel_tol).

    A velocity-controlled wheel at its torque cap is not reaching its commanded speed: the
    wheel ceiling v_rim is the point where the 2.5 N m cap balances the load (confirmed
    2026-09-18), so a wheel at the cap is a wheel the budget did not keep below saturation.
    """
    t = np.abs(np.asarray(torques, dtype=float))
    c = np.asarray(caps, dtype=float)
    if t.shape != c.shape:
        raise ValueError(f"torques {t.shape} and caps {c.shape} must match")
    if np.any(c <= 0.0):
        raise ValueError("torque caps must be positive")
    return int(np.sum(t >= c * (1.0 - rel_tol)))


def wrap_angle(theta: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    wrapped = math.atan2(math.sin(theta), math.cos(theta))
    return math.pi if wrapped == -math.pi else wrapped


def rotate(vec_xy: Vector, theta: float) -> np.ndarray:
    """Rotate a 2-D vector by `theta` rad, counter-clockwise."""
    c, s = math.cos(theta), math.sin(theta)
    x, y = float(vec_xy[0]), float(vec_xy[1])
    return np.array([c * x - s * y, s * x + c * y])


def world_to_body(vec_xy: Vector, yaw: float) -> np.ndarray:
    """Express a world-frame 2-D vector in a body frame at heading `yaw` (rad)."""
    return rotate(vec_xy, -yaw)


def wheels_from_body(command: Vector, geom: ChassisGeometry) -> np.ndarray:
    """Inverse kinematics: body velocity -> wheel speeds, in WHEEL_ORDER.

    `command` is (vx, vy, wz) in (m/s, m/s, rad/s). The returned speeds are in
    rad/s and already carry the per-wheel sign vector, so they are the values to
    hand straight to `sim.setJointTargetVelocity`.
    """
    cmd = np.asarray(command, dtype=float)
    if cmd.shape != (3,):
        raise ValueError(f"command must be (vx, vy, wz), got shape {cmd.shape}")
    scaled = np.array([cmd[0], cmd[1], geom.lw * cmd[2]])
    speeds = _MIX_SIGNS @ scaled / geom.wheel_radius
    return speeds * np.asarray(geom.wheel_sign, dtype=float)


def body_from_wheels(wheel_speeds: Vector, geom: ChassisGeometry) -> np.ndarray:
    """Forward kinematics: wheel speeds (rad/s, WHEEL_ORDER) -> (vx, vy, wz).

    Left inverse of `wheels_from_body`: the mecanum mixing matrix has full column
    rank, so this recovers the body velocity exactly for a consistent set of
    wheel speeds, and returns the least-squares fit otherwise.
    """
    speeds = np.asarray(wheel_speeds, dtype=float)
    if speeds.shape != (4,):
        raise ValueError(f"expected 4 wheel speeds in order {WHEEL_ORDER}, got {speeds.shape}")
    unsigned = speeds / np.asarray(geom.wheel_sign, dtype=float)
    scaled = (_MIX_SIGNS.T @ unsigned) * geom.wheel_radius / 4.0
    return np.array([scaled[0], scaled[1], scaled[2] / geom.lw])


@dataclass(frozen=True)
class ClampResult:
    """A clamped command and whether, and how hard, each limit bound."""

    applied: np.ndarray        # (vx, vy, wz) inside the commanded-velocity box
    translation_scale: float   # factor applied to (vx, vy); 1.0 when it did not bind
    omega_clipped: bool        # True when |wz| was clipped to w_max

    @property
    def translation_bound(self) -> bool:
        return self.translation_scale < 1.0


def clamp_velocity_with_status(command: Vector, limits: ActuationLimits) -> ClampResult:
    """Clamp (vx, vy, wz) into the commanded-velocity box, preserving direction.

    The translational pair is scaled by a single factor, the largest s <= 1 with
    |s*vx| <= v_max and |s*vy| <= vy_max, so a command that leaves the box is shortened
    along its own direction. Per-axis clipping, used until amendment A6, bent diagonal
    commands towards an axis and produced G1b's 0.50 m/s failures. The yaw rate is a
    separate channel and is clipped on its own.

    This box is an interim limit. The WP2 target is the mecanum wheel-speed polytope in
    (vx, vy, wz), imposed as constraints inside the PFB QP rather than after it (A6).
    """
    cmd = np.asarray(command, dtype=float)
    if cmd.shape != (3,):
        raise ValueError(f"command must be (vx, vy, wz), got shape {cmd.shape}")
    scale = 1.0
    if abs(cmd[0]) > limits.v_max:
        scale = min(scale, limits.v_max / abs(cmd[0]))
    if abs(cmd[1]) > limits.vy_max:
        scale = min(scale, limits.vy_max / abs(cmd[1]))
    wz = float(np.clip(cmd[2], -limits.w_max, limits.w_max))
    applied = np.array([cmd[0] * scale, cmd[1] * scale, wz])
    # Guard the box against the last-bit rounding of the division above.
    applied[0] = float(np.clip(applied[0], -limits.v_max, limits.v_max))
    applied[1] = float(np.clip(applied[1], -limits.vy_max, limits.vy_max))
    return ClampResult(applied, scale, abs(cmd[2]) > limits.w_max)


def clamp_velocity(command: Vector, limits: ActuationLimits) -> np.ndarray:
    """Clamp (vx, vy, wz) into the commanded-velocity box; see clamp_velocity_with_status."""
    return clamp_velocity_with_status(command, limits).applied


def limit_acceleration(
    command: Vector,
    previous: Vector,
    dt: float,
    limits: ActuationLimits,
) -> np.ndarray:
    """Rate-limit a body-velocity command against the previously applied one.

    The linear part is limited on the *magnitude* of the change: the step
    (dvx, dvy) is scaled down uniformly when its norm exceeds the applicable
    limit times dt, so the direction of the velocity change is preserved and the
    limit is isotropic. Per-axis clipping would instead bend a diagonal
    acceleration towards an axis. The yaw rate is limited independently by
    `alpha_max`, since yaw and translation are separate channels of the mixing.

    Which limit applies depends on whether the command reduces speed. A step that
    does not is bounded by `a_max`; a step that does is bounded by `a_decel_max`,
    which the configuration holds at or below `certified.a_brake`. That is what makes
    the PFB's stopping-distance model conservative *by construction*: no
    controller can command a stop harder than the barrier assumed when it decided
    how close the robot could approach. Bypassing the limiter with
    `rate_limit=False` deliberately breaks that guarantee and exists only for
    calibration scripts that need a true velocity step.

    Applying this after `clamp_velocity` keeps the result inside the velocity
    box: the output lies on the segment between `previous` and the clamped
    command, and the box is convex.
    """
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")
    cmd = np.asarray(command, dtype=float)
    prev = np.asarray(previous, dtype=float)
    if cmd.shape != (3,) or prev.shape != (3,):
        raise ValueError("command and previous must both be (vx, vy, wz)")

    delta_lin = cmd[:2] - prev[:2]
    slowing = float(np.hypot(cmd[0], cmd[1])) < float(np.hypot(prev[0], prev[1]))
    max_dv = (limits.a_decel_max if slowing else limits.a_max) * dt
    norm = float(np.hypot(delta_lin[0], delta_lin[1]))
    if norm > max_dv:
        delta_lin = delta_lin * (max_dv / norm)

    max_dw = limits.alpha_max * dt
    delta_w = float(np.clip(cmd[2] - prev[2], -max_dw, max_dw))

    return np.array([prev[0] + delta_lin[0], prev[1] + delta_lin[1], prev[2] + delta_w])


def _bbox_corners_in_ref(
    bbox_min: Vector, bbox_max: Vector, matrix_shape_to_ref: Vector
) -> np.ndarray:
    """The eight bounding-box corners, expressed in the base_ref frame. Shape (8, 3)."""
    m = np.asarray(matrix_shape_to_ref, dtype=float)
    if m.shape != (12,):
        raise ValueError(f"expected a 12-element CoppeliaSim matrix, got shape {m.shape}")
    transform = m.reshape(3, 4)
    rot, translation = transform[:, :3], transform[:, 3]

    lo = np.asarray(bbox_min, dtype=float)
    hi = np.asarray(bbox_max, dtype=float)
    if lo.shape != (3,) or hi.shape != (3,):
        raise ValueError("bbox_min and bbox_max must each have 3 entries")
    if np.any(hi < lo):
        raise ValueError(f"bbox_max {hi} is not componentwise >= bbox_min {lo}")

    corners = np.array(
        [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    )
    return corners @ rot.T + translation


def body_point_in_ref(point_body: Vector, yaw_offset: float) -> np.ndarray:
    """Express a body-frame point in base_ref coordinates. Inverse of the body rotation.

    The body frame is the base_ref frame rotated by `yaw_offset` about +z, so a body
    point maps back by +yaw_offset. Used to place probe geometry at a known position
    in the robot's own frame without doing the world-frame arithmetic by hand.
    """
    p = np.asarray(point_body, dtype=float)
    if p.shape != (3,):
        raise ValueError(f"point_body must be (x, y, z), got shape {p.shape}")
    xy = rotate(p[:2], yaw_offset)
    return np.array([xy[0], xy[1], p[2]])


def ref_point_in_body(point_ref: Vector, yaw_offset: float) -> np.ndarray:
    """Express a base_ref point in body-frame coordinates. Inverse of `body_point_in_ref`.

    The body frame is the base_ref frame rotated by `yaw_offset` about +z, so a point
    given in base_ref coordinates maps into the body frame by -yaw_offset.
    """
    return body_point_in_ref(point_ref, -yaw_offset)


def bbox_corners_in_body(
    bbox_min: Vector,
    bbox_max: Vector,
    matrix_shape_to_ref: Vector,
    yaw_offset: float,
) -> np.ndarray:
    """The eight bounding-box corners of a shape, in the body frame. Shape (8, 3), metres.

    `chassis_extent_in_body` reduces these to half-extents, which is all a rectangular
    footprint needs. The corners themselves are what a *polygon* footprint is built from,
    so they are exposed rather than collapsed.
    """
    in_ref = _bbox_corners_in_ref(bbox_min, bbox_max, matrix_shape_to_ref)
    c, s = math.cos(-yaw_offset), math.sin(-yaw_offset)
    return np.column_stack(
        [
            c * in_ref[:, 0] - s * in_ref[:, 1],
            s * in_ref[:, 0] + c * in_ref[:, 1],
            in_ref[:, 2],
        ]
    )


def chassis_extent_in_body(
    bbox_min: Vector,
    bbox_max: Vector,
    matrix_shape_to_ref: Vector,
    yaw_offset: float,
) -> tuple[float, float, float]:
    """Half-extents (m) of a shape's bounding box, expressed in the body frame.

    `bbox_min` / `bbox_max` are the box corners in the shape's own frame, as
    reported by `sim.getObjectFloatParam(h, sim.objfloatparam_objbbox_*)`.
    `matrix_shape_to_ref` is the 12-element row-major CoppeliaSim transform from
    the shape frame to the base_ref frame (`sim.getObjectMatrix(shape, ref)`).
    The body frame is the base_ref frame rotated by `yaw_offset` about +z, so
    coordinates transform by -yaw_offset.

    Returns (half_length, half_width, half_height): half-extents of the
    axis-aligned box in the body frame that contains all eight transformed
    corners. This is the quantity `robot.half_length` / `robot.half_width` hold.
    """
    in_body = bbox_corners_in_body(bbox_min, bbox_max, matrix_shape_to_ref, yaw_offset)
    half = np.max(np.abs(in_body), axis=0)
    return float(half[0]), float(half[1]), float(half[2])


def tightest_yaw_offset(
    bbox_min: Vector,
    bbox_max: Vector,
    matrix_shape_to_ref: Vector,
    *,
    resolution: float = 1e-4,
) -> tuple[float, tuple[float, float, float]]:
    """The yaw offset that makes a shape's bounding box axis-aligned in the body frame.

    Scans `yaw_offset` over [0, pi) at `resolution` rad and returns the value that
    minimises the in-plane area (half_length * half_width) of the body-frame
    extents, together with those extents. The extents have period pi in the yaw
    offset, so [0, pi) covers every distinct alignment; the second admissible
    alignment is the returned value plus pi/2, which swaps length and width.

    This answers "is the pose reference frame rotated against the chassis, and by
    how much" directly, instead of inferring it from whether the configured
    half-extents happen to look swapped. Which of the two alignments is *forward*
    is not a geometric question and is not answered here: gate G1 settles it by
    driving the robot.

    For a rectangular footprint the two alignments have exactly equal area, so
    which of them this returns depends on where the scan grid falls. Callers must
    therefore always consider both the returned offset and offset + pi/2; the
    caller that reports this to a human prints both. A square footprint is fully
    degenerate -- every offset gives the same area -- and the first scanned
    minimum is returned. The reported offset is only accurate to `resolution`.
    """
    if resolution <= 0.0:
        raise ValueError(f"resolution must be positive, got {resolution}")
    in_ref = _bbox_corners_in_ref(bbox_min, bbox_max, matrix_shape_to_ref)

    offsets = np.arange(0.0, math.pi, resolution)
    c = np.cos(-offsets)[:, None]
    s = np.sin(-offsets)[:, None]
    x_body = c * in_ref[None, :, 0] - s * in_ref[None, :, 1]
    y_body = s * in_ref[None, :, 0] + c * in_ref[None, :, 1]

    half_x = np.max(np.abs(x_body), axis=1)
    half_y = np.max(np.abs(y_body), axis=1)
    best = int(np.argmin(half_x * half_y))
    half_z = float(np.max(np.abs(in_ref[:, 2])))
    return float(offsets[best]), (float(half_x[best]), float(half_y[best]), half_z)


@dataclass(frozen=True)
class SceneObjects:
    """Scene object paths, from `sim.objects` in the configuration."""

    base_ref: str
    chassis: str
    robot_tree: str
    floor: str
    wheels: tuple[str, ...]

    def __post_init__(self) -> None:
        if len(self.wheels) != 4:
            raise ValueError(f"sim.objects.wheels needs 4 paths in order {WHEEL_ORDER}")

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> SceneObjects:
        objects = cfg["sim"]["objects"]
        return cls(
            base_ref=str(objects["base_ref"]),
            chassis=str(objects["chassis"]),
            robot_tree=str(objects["robot_tree"]),
            floor=str(objects["floor"]),
            wheels=tuple(str(p) for p in objects["wheels"]),
        )


class YouBotBase:
    """Simulator-facing wrapper around the youBot mecanum base.

    Holds no control logic: it resolves handles, converts between the body frame
    and the wheel joints, applies the actuation limits, and reads back pose and
    velocity. `bind()` must be called after the simulation has been started,
    because object handles are only valid for a running scene.
    """

    def __init__(
        self,
        client,
        geometry: ChassisGeometry,
        limits: ActuationLimits,
        objects: SceneObjects,
        dt: float,
        budget: WheelBudget | None = None,
    ):
        if dt <= 0.0:
            raise ValueError(f"dt must be positive, got {dt}")
        self.client = client
        self.geometry = geometry
        self.limits = limits
        self.objects = objects
        self.dt = dt
        self.base_handle: int | None = None
        self.wheel_handles: tuple[int, ...] = ()
        self._applied = np.zeros(3)
        self.budget = budget
        self.torque_caps: np.ndarray | None = None
        # Per episode: steps with any wheel at the cap, and wheel-steps at the cap.
        self.torque_cap_counts = {"steps": 0, "wheel_steps": 0}
        self.last_clamp: ClampResult | None = None
        self.last_budget: BudgetResult | None = None
        self.clamp_counts = {"translation": 0, "omega": 0, "budget": 0}

    @classmethod
    def from_config(cls, client, cfg: dict[str, Any]) -> YouBotBase:
        return cls(
            client,
            ChassisGeometry.from_config(cfg),
            ActuationLimits.from_config(cfg),
            SceneObjects.from_config(cfg),
            float(cfg["sim"]["dt"]),
            WheelBudget.from_config(cfg),
        )

    @property
    def sim(self) -> Any:
        return self.client.sim

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------
    def bind(self) -> None:
        """Resolve handles and bring the base to rest. Call after `start()`."""
        self.base_handle = self.client.object(self.objects.base_ref)
        self.wheel_handles = tuple(self.client.object(p) for p in self.objects.wheels)
        self.torque_caps = np.array([abs(float(self.sim.getJointTargetForce(h)))
                                     for h in self.wheel_handles])
        self.halt()

    def reset_command_state(self) -> None:
        """Forget the previously applied command; the base is assumed at rest.

        Must be called on every episode reset, otherwise the acceleration limiter
        would ramp the first command of a new episode down from the velocity the
        previous episode ended at.
        """
        self._applied = np.zeros(3)
        self.last_clamp = None
        self.last_budget = None
        self.clamp_counts = {"translation": 0, "omega": 0, "budget": 0}
        self.torque_cap_counts = {"steps": 0, "wheel_steps": 0}

    @property
    def applied_command(self) -> np.ndarray:
        """The last command actually written to the wheels, (vx, vy, wz)."""
        return self._applied.copy()

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------
    def command_body_velocity(
        self, command: Vector, *, rate_limit: bool = True
    ) -> np.ndarray:
        """Clamp to the axis box, project into the wheel budget, rate-limit, and apply.

        The budget applied here is the *filter* share, the one share enforced on every method's
        command (ruled 2026-10-26). Box and budget are both convex, so the acceleration limiter,
        which moves between two points inside them, cannot leave them.

        Returns the command actually applied. (The reward's intervention term compares the
        request with the PFB's output, not with this; ruled 2026-10-24.)
        `rate_limit=False` is for calibration scripts that need a velocity step.
        """
        if self.base_handle is None:
            raise RuntimeError("call bind() before commanding the base")
        clamp = clamp_velocity_with_status(command, self.limits)
        # Every binding is counted, so a run reports how often its commands left the
        # box instead of having them silently reshaped (A6).
        self.last_clamp = clamp
        self.clamp_counts["translation"] += int(clamp.translation_bound)
        self.clamp_counts["omega"] += int(clamp.omega_clipped)
        applied = clamp.applied
        if self.budget is not None:
            budget = project_to_wheel_budget(applied, self.budget.filter_limit,
                                             self.budget.lever_arm)
            self.last_budget = budget
            self.clamp_counts["budget"] += int(budget.bound)
            applied = budget.applied
        if rate_limit:
            applied = limit_acceleration(applied, self._applied, self.dt, self.limits)
        self._write_wheels(applied)
        self._applied = applied
        return applied.copy()

    def record_torque_cap(self) -> int:
        """Read the wheel torques after a step; count and return the wheels at the cap.

        Ruling on WP0 closure: a per-episode counter, surfaced in training and evaluation
        logs. v_rim is load-dependent, so if saturation appears in practice the budget
        fractions drop -- by decision, logged, not automatically.
        """
        if self.torque_caps is None:
            raise RuntimeError("call bind() before reading wheel torques")
        torques = [float(self.sim.getJointForce(h)) for h in self.wheel_handles]
        at_cap = count_at_torque_cap(torques, self.torque_caps)
        self.torque_cap_counts["steps"] += int(at_cap > 0)
        self.torque_cap_counts["wheel_steps"] += at_cap
        return at_cap

    def halt(self) -> None:
        """Write zero wheel speeds immediately, bypassing the rate limiter."""
        self._write_wheels(np.zeros(3))
        self._applied = np.zeros(3)

    def _write_wheels(self, command: np.ndarray) -> None:
        speeds = wheels_from_body(command, self.geometry)
        for handle, speed in zip(self.wheel_handles, speeds):
            self.sim.setJointTargetVelocity(handle, float(speed))

    # ------------------------------------------------------------------
    # Readouts
    # ------------------------------------------------------------------
    def pose(self) -> tuple[float, float, float]:
        """World pose (x, y, yaw) of the body frame; x, y in m, yaw in (-pi, pi]."""
        pos = self.sim.getObjectPosition(self.base_handle, -1)
        ori = self.sim.getObjectOrientation(self.base_handle, -1)
        return float(pos[0]), float(pos[1]), wrap_angle(float(ori[2]) + self.geometry.yaw_offset)

    def body_velocity(self) -> np.ndarray:
        """Measured (vx, vy, wz) in the body frame, from the simulator."""
        lin, ang = self.sim.getObjectVelocity(self.base_handle)
        _, _, yaw = self.pose()
        vx, vy = world_to_body((lin[0], lin[1]), yaw)
        return np.array([vx, vy, float(ang[2])])

    def wheel_speeds(self) -> np.ndarray:
        """Measured wheel speeds (rad/s) in WHEEL_ORDER, as reported by the joints."""
        return np.array([float(self.sim.getJointVelocity(h)) for h in self.wheel_handles])

    def body_velocity_from_wheels(self) -> np.ndarray:
        """Body velocity reconstructed from the measured wheel speeds.

        Independent of the pose readout, so a disagreement with `body_velocity()`
        points at the sign vector or the yaw offset rather than at the physics.
        Gate G1 compares the two.
        """
        return body_from_wheels(self.wheel_speeds(), self.geometry)
