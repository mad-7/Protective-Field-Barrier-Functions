"""Gate G0: validation of the ground-truth contact check.

Criteria come from `preregistration/gates.yaml` and are never hard-coded here. Each
positive case drives the robot at the committed approach speed into an obstacle placed
to strike one specific part, and compares the step at which `RobotContact` fires against
the step at which an independent geometric measurement says the surfaces met. The
negative case drives past an obstacle at a fixed clearance and requires no contact at all.

The geometric reference is `sim.checkDistance`, a different simulator primitive from
`sim.checkCollision`. Predicting the contact step from the commanded speed instead would
fold in the acceleration limiter and the wheel radius, which would make this a test of
the motion model rather than of the contact check.

**Validation, G0 only (ruling 1 on the WP0 results report).** At every step each of the
robot's shapes is also checked as a *single handle*, which ignores the collidable flag,
and the result is compared with `RobotContact`, which checks collections and so honours
it. They must agree on every step; a disagreement names a shape that touched an obstacle
without the contact check seeing it. This costs one call per shape per step, which is
why it lives here and never in the environment's step loop. The geometric reference is
likewise taken per handle over exactly the set that defines a contact -- the chassis,
checked by handle whatever its flag, plus every other shape whose collidable flag is on --
so it does not depend on the separate *measurable* flag, which `checkDistance` honours
inside a collection.

`gates.yaml` requires all cases to pass in two contexts.

  * **`gate_script`** drives each case from this script's own loop.
  * **`env_step_loop`** drives the same cases through `pfbnav/envs/step_loop.py`, the loop
    the environment runs -- one server-side scan packet, the pose, and the ground-truth
    contact check per step. The point is not to repeat the first context but to test the
    contact check WHERE EPISODES ACTUALLY RUN: a different call order, a scan read in the
    middle of it, and the same `RobotContact`.

**The certified layer is evaluated but not allowed to override in the `env_step_loop`
context** (`certified.enabled: false` in the overlay). A positive case has to reach contact
for the contact check to be validated at all, and the emulator would stop the robot first;
what it WOULD have done is recorded per step instead (`certified_would_stop_step`), so the
run still says so. This is the one place the layer is switched off, and never for a result.

Usage:
    python scripts/gate_g0_contact.py --config configs/base.yaml --seed 0

(`configs/gates/g0_rerun.yaml` was the overlay for the WP0 re-run and is superseded: D8
has written the measured half_width into base.yaml, and the overlay now under-states it.)
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pfbnav.config import load_config
from pfbnav.eval.gates import contact_agreement, count_contact_steps, detection_lag_steps
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.contact import RobotContact
from pfbnav.sim.robot import (
    ChassisGeometry,
    SceneObjects,
    YouBotBase,
    body_point_in_ref,
)
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.runio import create_run_dir, stamp_display
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
PREREG = REPO_ROOT / "preregistration" / "gates.yaml"

# Gap between the robot's envelope and the obstacle at the start of a positive approach.
APPROACH_GAP_M = 0.20
# Clearance held by the negative case, from `gates.yaml`'s case name.
PARALLEL_PASS_GAP_M = 0.010
# How far the negative case travels alongside the obstacle.
PARALLEL_PASS_TRAVEL_M = 1.00
# Obstacle dimensions, in metres.
WALL_SPAN_M, WALL_THICKNESS_M, WALL_HEIGHT_M = 1.20, 0.05, 0.40
# A slab low enough to pass under the chassis (whose underside is 0.0148 m up), so only
# a wheel can reach it.
WHEEL_SLAB_HEIGHT_M = 0.012
# A slim post stood at the forward LiDAR housing's front-outer corner, so that the
# corner of the housing is what meets it. "Pod" is the LiDAR housing, confirmed by the
# researcher on 2026-09-17, so this case is a robot part being struck, like wheel_edge,
# rather than an obstacle corner.
CORNER_POST_EDGE_M = 0.04
CORNER_POST_HEIGHT_M = 0.40
# Case six (amendment, supervisor 2026-09-26 on Q5): a RAISED slab whose underside clears the
# chassis, so only something standing above the chassis can reach it. Measured heights from the
# S300 layout read-back (2026-09-22): wheel tops 0.0999 m, chassis top 0.1764 m, the scanner
# housings span z 0.104-0.257 m, the scan plane sits at 0.2207 m. An underside at 0.19 m is
# 13.6 mm above the chassis and inside the housings' span, so the housing is the only part of
# the base that can meet it; the slab crosses the scan plane, so it is also SEEN, which
# distinguishes this case from `wheel_edge` (unseeable by construction).
HOUSING_SLAB_BOTTOM_M = 0.19
HOUSING_SLAB_TOP_M = 0.40
# How finely a shape's mesh is sampled when deciding which part a case strikes.
MESH_STEP_M = 0.01


@dataclass(frozen=True)
class Case:
    """One approach: where the obstacle goes, which way the robot drives, what is expected.

    `offset_body` is the obstacle centre in the robot's body frame at the start pose.
    `drive_body` is a unit direction in the body frame, scaled by the committed approach
    speed. `expect_contact` distinguishes the positive cases from the negative one.
    """

    name: str
    size: tuple[float, float, float]
    offset_body: tuple[float, float, float]
    drive_body: tuple[float, float, float]
    yaw_in_body: float
    expect_contact: bool
    travel_m: float
    note: str
    # The shape this case is meant to strike, measured from the scene by `first_to_strike`
    # and attached in `expected_shapes`; None for the negative case.
    expect_shape: str | None = None


def shape_support(corners: np.ndarray, direction: tuple[float, float]) -> float:
    """How far a shape reaches along a body-frame direction, m: max of corner . u."""
    u = np.asarray(direction, dtype=float)
    u = u / float(np.linalg.norm(u))
    return float(np.max(np.asarray(corners, dtype=float)[:, :2] @ u))


def outermost_bearing(extent, window_deg: float = 90.0, step_deg: float = 0.5) -> float:
    """Bearing (rad) where the footprint reaches furthest, within +-window of straight on.

    The corner case is placed here. "The forward-most shape" stopped being well defined
    when the leading part became bearing-dependent (supervisor, 2026-09-22): on the S300
    layout the chassis leads dead ahead, the front scanner leads from about -10 to -45 deg,
    and a wheel leads beyond that, so the case has to be placed from the envelope itself.
    """
    b = np.radians(np.arange(-window_deg, window_deg + 1e-9, step_deg))
    return float(b[int(np.argmax(np.asarray(extent(b), dtype=float)))])


def first_to_strike(boxes: dict[str, np.ndarray], obstacle_min: tuple[float, float, float],
                    obstacle_max: tuple[float, float, float],
                    drive: tuple[float, float, float]) -> tuple[str, float]:
    """Which shape an axis-aligned obstacle meets first as the robot drives into it.

    `boxes` maps a shape's name to its body-frame points -- mesh vertices from
    `shape_points`, or a box's corners in a test. The obstacle is an axis-aligned body-frame
    box and `drive` an axis-aligned unit direction. Only shapes with a point inside the
    obstacle's span on the two axes across the drive can be struck by it
    (which is what makes `wheel_edge`'s low slab a wheel case, and what puts a corner post
    on one side of the robot rather than the other); of those, the one reaching furthest
    along the drive is struck first. Returns (name, its support along the drive, m).

    This is the assertion the supervisor asked for on 2026-09-22: a case that contacts the
    wrong part while passing is worse than one that errors.
    """
    if not boxes:
        raise ValueError("no shapes")
    d = np.asarray(drive, dtype=float)
    if abs(d[2]) > 1e-12 or np.linalg.norm(d[:2]) < 1e-12:
        raise ValueError(f"the drive must be horizontal and non-zero, got {drive}")
    u = d[:2] / float(np.linalg.norm(d[:2]))
    w = np.array([-u[1], u[0]])                 # across the drive, in the floor plane
    lo = np.asarray(obstacle_min, dtype=float)
    hi = np.asarray(obstacle_max, dtype=float)
    if np.any(hi < lo):
        raise ValueError("the obstacle box is inside out")
    # The corridor the obstacle sweeps out as the robot drives into it: its shadow on the
    # plane across the drive. For an axis-aligned drive this is just the obstacle's span on
    # the other two axes; for a diagonal approach it is the box's width across that drive.
    corners = np.array([[x, y] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])])
    across = corners @ w
    w_lo, w_hi = float(across.min()), float(across.max())
    reachable = {}
    for name, pts in boxes.items():
        c = np.asarray(pts, dtype=float)
        offset = c[:, :2] @ w
        inside = ((offset >= w_lo) & (offset <= w_hi)
                  & (c[:, 2] >= lo[2]) & (c[:, 2] <= hi[2]))
        if inside.any():
            reachable[name] = float(np.max(c[inside, :2] @ u))
    if not reachable:
        raise ValueError("no shape can reach the obstacle: it is placed beyond the robot")
    name = max(reachable, key=lambda n: reachable[n])
    return name, reachable[name]


def build_cases(geom: ChassisGeometry, corner_xy: tuple[float, float],
                corner_bearing: float) -> list[Case]:
    """The cases named in `gates.yaml`, placed from the measured envelope.

    `corner_xy` is the envelope's outermost point within +-90 deg of straight on, measured
    (`outermost_bearing` on the footprint table), so `pod_corner` lands on the corner that
    actually leads rather than on an assumed one.
    """
    reach_x = geom.half_length + APPROACH_GAP_M
    reach_y = geom.half_width + APPROACH_GAP_M
    approach_travel = APPROACH_GAP_M + 0.10  # enough to make contact and press in a little

    return [
        Case("front_wall_head_on", (WALL_THICKNESS_M, WALL_SPAN_M, WALL_HEIGHT_M),
             (reach_x, 0.0, WALL_HEIGHT_M / 2), (1.0, 0.0, 0.0), 0.0, True, approach_travel,
             "flat wall square on to the front; the forward LiDAR housing leads"),
        Case("rear_wall", (WALL_THICKNESS_M, WALL_SPAN_M, WALL_HEIGHT_M),
             (-reach_x, 0.0, WALL_HEIGHT_M / 2), (-1.0, 0.0, 0.0), 0.0, True, approach_travel,
             "flat wall square on to the rear; the rear LiDAR housing leads"),
        Case("side_wall_lateral", (WALL_SPAN_M, WALL_THICKNESS_M, WALL_HEIGHT_M),
             (0.0, reach_y, WALL_HEIGHT_M / 2), (0.0, 1.0, 0.0), 0.0, True, approach_travel,
             "flat wall abeam; the wheels lead, standing 17 mm proud of the chassis"),
        # Approached ALONG the outermost bearing, not along +x: the corner leads only in
        # its own direction, and a forward drive would put the chassis on the post first
        # (measured 2026-09-23). The robot is holonomic, so the approach is a translation.
        Case("pod_corner", (CORNER_POST_EDGE_M, CORNER_POST_EDGE_M, CORNER_POST_HEIGHT_M),
             (corner_xy[0] + APPROACH_GAP_M * math.cos(corner_bearing),
              corner_xy[1] + APPROACH_GAP_M * math.sin(corner_bearing),
              CORNER_POST_HEIGHT_M / 2),
             (math.cos(corner_bearing), math.sin(corner_bearing), 0.0), 0.0, True,
             approach_travel,
             "post at the envelope's outermost forward point, so that corner leads; the "
             "name is the historical label for the geometry this case probes"),
        Case("wheel_edge", (WALL_SPAN_M, WALL_THICKNESS_M, WHEEL_SLAB_HEIGHT_M),
             (geom.half_wheelbase, reach_y, WHEEL_SLAB_HEIGHT_M / 2), (0.0, 1.0, 0.0), 0.0,
             True, approach_travel,
             "slab too low for the chassis to reach, level with a front wheel"),
        # Case six. Raised so the chassis and the wheels pass under it and a scanner housing
        # is what meets it. Spans the whole side rather than being centred on a housing's x,
        # so the case does not assume which housing leads: whichever shape above the slab's
        # underside reaches furthest abeam is struck, and `expected_shapes` records it.
        Case("raised_housing",
             (WALL_SPAN_M, WALL_THICKNESS_M, HOUSING_SLAB_TOP_M - HOUSING_SLAB_BOTTOM_M),
             (0.0, reach_y,
              (HOUSING_SLAB_BOTTOM_M + HOUSING_SLAB_TOP_M) / 2), (0.0, 1.0, 0.0), 0.0,
             True, approach_travel,
             "raised slab clearing the chassis, so a scanner housing above the wheels "
             "leads; unlike wheel_edge it crosses the scan plane and is seen"),
        # Long enough, and centred far enough along, to flank the whole traverse: the
        # robot's nose ends at travel + half_length, and a wall that stops short of it
        # would quietly stop testing before the run does.
        Case("parallel_pass_gap_10mm",
             (PARALLEL_PASS_TRAVEL_M + 2 * geom.half_length + 0.20, WALL_THICKNESS_M,
              WALL_HEIGHT_M),
             (PARALLEL_PASS_TRAVEL_M / 2,
              geom.half_width + PARALLEL_PASS_GAP_M + WALL_THICKNESS_M / 2,
              WALL_HEIGHT_M / 2),
             (1.0, 0.0, 0.0), 0.0, False, PARALLEL_PASS_TRAVEL_M,
             "drive alongside a wall held 10 mm off the envelope; any contact is false"),
    ]


def declared_cases(cases: list[Case], criteria: dict[str, Any]) -> tuple[list[Case], list[str]]:
    """Keep only the cases `gates.yaml` declares; report the ones it does not.

    Adding a case to a gate is an amendment, not a code change (supervisor, 2026-09-22 and
    again 2026-09-26 approving case six). Before this check the case table lived only in this
    script, so a sixth case would have run the moment it was written, and its result would
    have looked like part of a preregistered gate. Now the preregistration decides:

      * a case defined here but not declared there is **not run**, and is reported as pending;
      * a case declared there but not defined here is an **error** -- preregistration names a
        case the gate cannot run, and passing the others would be a false verdict.

    Returns (cases to run, names defined but not declared).
    """
    declared = [str(n) for n in criteria["positive_cases"]] + [str(criteria["negative_case"])]
    defined = {c.name for c in cases}
    missing = [n for n in declared if n not in defined]
    if missing:
        raise SystemExit(
            f"preregistration declares cases this script cannot build: {missing}; "
            "G0 cannot report a verdict while a declared case is unrunnable")
    pending = [c.name for c in cases if c.name not in declared]
    keep = [c for c in cases if c.name in declared]
    required = {str(k): str(v) for k, v in (criteria.get("case_required_shapes") or {}).items()}
    unknown = [n for n in required if n not in {c.name for c in keep}]
    if unknown:
        raise SystemExit(f"case_required_shapes names cases that do not run: {unknown}")
    wrong = [(c.name, required[c.name], c.expect_shape) for c in keep
             if c.name in required
             and not str(c.expect_shape or "").startswith(required[c.name])]
    if wrong:
        raise SystemExit(
            "a case is placed to strike a shape its declared criteria do not allow: "
            + "; ".join(f"{n} requires {req} but is placed to strike {got}"
                        for n, req, got in wrong)
            + ". The case geometry needs rework; it is not a verdict.")
    return keep, pending


def densify(vertices: np.ndarray, triangles: np.ndarray, step: float) -> np.ndarray:
    """Points covering a mesh's surface no coarser than `step` (m), vertices included.

    A primitive's mesh can be as coarse as a box's eight corners, and a narrow corridor --
    the corner post's is 56 mm across -- can then pass between the vertices of a face that
    really does cross it. Sampling each triangle on a barycentric grid removes that gap.
    """
    if step <= 0.0:
        raise ValueError("step must be positive")
    v = np.asarray(vertices, dtype=float).reshape(-1, 3)
    out = [v]
    for tri in np.asarray(triangles, dtype=int).reshape(-1, 3):
        a, b, c = v[tri[0]], v[tri[1]], v[tri[2]]
        longest = max(float(np.linalg.norm(b - a)), float(np.linalg.norm(c - b)),
                      float(np.linalg.norm(a - c)))
        n = math.ceil(longest / step)
        if n <= 1:
            continue
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing="ij")
        keep = (i + j) <= n
        wa = i[keep] / n
        wb = j[keep] / n
        out.append(np.outer(wa, a) + np.outer(wb, b) + np.outer(1.0 - wa - wb, c))
    return np.vstack(out)


def shape_points(client: SimClient, cfg: dict[str, Any]) -> dict[str, np.ndarray]:
    """Every robot shape's MESH VERTICES in the body frame, by name. (n, 3) each.

    Vertices, not bounding boxes: a shape's bounding box can overstate it badly, and here
    it does. Measured 2026-09-23 on the pinned scene, `wheel_respondable_fl`'s box reaches
    0.3133 m forward and 0.2437 m sideways while the contact probe measures that same wheel
    at 0.2085 m sideways (FOOTPRINT_20260922T170810Z), so a box-based expectation named it
    the leading part almost everywhere. The vertices are what the contact check sees.
    """
    geom = ChassisGeometry.from_config(cfg)
    objects = SceneObjects.from_config(cfg)
    sim = client.sim
    base_ref = client.object(objects.base_ref)
    points = {}
    for handle in client.shapes_in_tree(client.object(objects.robot_tree)):
        vertices, indices, _normals = sim.getShapeMesh(handle)
        v = np.asarray(vertices, dtype=float).reshape(-1, 3)
        if v.size == 0:
            raise RuntimeError(f"shape {sim.getObjectAlias(handle, 2)} has no mesh")
        dense = densify(v, np.asarray(indices, dtype=int).reshape(-1, 3), MESH_STEP_M)
        points[str(sim.getObjectAlias(handle, 2))] = apply_matrix_to_body(
            [float(m) for m in sim.getObjectMatrix(handle, base_ref)], dense, geom.yaw_offset)
    if not points:
        raise RuntimeError("the robot subtree contains no shapes to measure")
    return points


def apply_matrix_to_body(matrix: list[float], points: np.ndarray,
                         yaw_offset: float) -> np.ndarray:
    """Points in a shape's own frame -> the body frame, via the shape-to-base_ref matrix."""
    r = np.asarray(matrix, dtype=float).reshape(3, 4)
    in_ref = np.asarray(points, dtype=float) @ r[:, :3].T + r[:, 3]
    c, s = math.cos(-yaw_offset), math.sin(-yaw_offset)
    return np.column_stack([c * in_ref[:, 0] - s * in_ref[:, 1],
                            s * in_ref[:, 0] + c * in_ref[:, 1], in_ref[:, 2]])


def obstacle_box(case: Case) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """The case's obstacle as an axis-aligned body-frame box (min, max), m."""
    return (tuple(o - s / 2 for o, s in zip(case.offset_body, case.size, strict=True)),
            tuple(o + s / 2 for o, s in zip(case.offset_body, case.size, strict=True)))


def expected_shapes(cases: list[Case], boxes: dict[str, np.ndarray]) -> list[Case]:
    """Each positive case with the shape it must strike attached, measured from the scene."""
    out = []
    for case in cases:
        if not case.expect_contact:
            out.append(case)
            continue
        lo, hi = obstacle_box(case)
        name, _ = first_to_strike(boxes, lo, hi, case.drive_body)
        out.append(replace(case, expect_shape=name))
    return out


def place_obstacle(sim, base_ref: int, case: Case, yaw_offset: float) -> int:
    """Create the case's obstacle at its body-frame offset from the robot's start pose.

    The obstacle is not parented to the robot: it is positioned relative to `base_ref`
    once, then stays put in the world while the robot drives into it.
    """
    handle = int(sim.createPrimitiveShape(sim.primitiveshape_cuboid, list(case.size)))
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 0)
    sim.setObjectOrientation(handle, base_ref, [0.0, 0.0, yaw_offset + case.yaw_in_body])
    sim.setObjectPosition(
        handle, base_ref, body_point_in_ref(case.offset_body, yaw_offset).tolist()
    )
    return handle


def geometric_gap(sim, robot_shapes: list[int], obstacle: int) -> float:
    """Least distance between any of `robot_shapes` and the obstacle, in metres.

    Taken per handle, over the shapes whose collidable flag is on: the reference must see
    exactly the geometry that defines a contact. Returns 0.0 once the surfaces meet or
    overlap: `checkDistance` does not report penetration depth, which is all this gate
    needs. A reading that the simulator declines to produce is an error, not a zero.
    """
    least = math.inf
    for shape in robot_shapes:
        result, data, _ = sim.checkDistance(shape, obstacle, 0.0)
        if not result:
            raise RuntimeError(
                f"sim.checkDistance returned no data for robot shape {shape} against the "
                "case obstacle; without an independent geometric reference the detection "
                "lag is unmeasurable")
        least = min(least, float(data[6]))
    return least


def touching_by_handle(sim, shapes: dict[int, str], chassis: int, obstacles: int,
                       obstacles_no_floor: int) -> list[str]:
    """Names of the robot shapes touching an obstacle, each checked as a single handle.

    A single handle is checked whatever its collidable flag, so this sees every shape.
    Same obstacle sets as RobotContact: the chassis against everything outside the robot,
    every other shape against that minus the floor.
    """
    touching = []
    for handle, name in shapes.items():
        against = obstacles if handle == chassis else obstacles_no_floor
        if sim.checkCollision(handle, against)[0]:
            touching.append(name)
    return touching


def run_case(client: SimClient, cfg: dict[str, Any], case: Case, speed: float,
             run_dir: Path, context: str = "gate_script", *, required_prefix: str | None = None) -> dict[str, Any]:
    """Drive one case to completion and return its verdict and per-step log."""
    geom = ChassisGeometry.from_config(cfg)
    objects = SceneObjects.from_config(cfg)
    dt = float(cfg["sim"]["dt"])

    client.start(settle_steps=20)
    robot = YouBotBase.from_config(client, cfg)
    robot.bind()
    contact = RobotContact.from_config(client, cfg)
    contact.bind()

    sim = client.sim
    base_ref = client.object(objects.base_ref)
    all_shapes = {h: str(sim.getObjectAlias(h, 2))
                  for h in client.shapes_in_tree(client.object(objects.robot_tree))}
    # The shapes that define a contact: RobotContact checks the chassis as a single handle,
    # which ignores the flag -- and the chassis's flag is off in this scene -- and every
    # other shape through a collection, which honours it. The reference must match.
    chassis_handle = client.object(objects.chassis)
    collidable_shapes = [h for h in all_shapes
                         if h == chassis_handle or client.special_flags(h)["collidable"]]
    # Ruling on WP0 closure: the chassis's flag is off in this scene, and it counts only
    # because it is checked by handle. Assert it is checked, every way it must be.
    if chassis_handle not in all_shapes or chassis_handle not in collidable_shapes:
        raise RuntimeError("the chassis is missing from G0's checked handles")
    if contact.chassis_handle != chassis_handle:
        raise RuntimeError("RobotContact is not checking the configured chassis handle")
    obstacle = place_obstacle(sim, base_ref, case, geom.yaw_offset)
    contact.bind()  # the obstacle exists now; rebinding costs nothing and removes a doubt

    command = [axis * speed for axis in case.drive_body]
    approach_steps = math.ceil(case.travel_m / (speed * dt))
    ramp_steps = math.ceil(speed / (float(cfg["actuation"]["a_max"]) * dt))
    max_steps = approach_steps + ramp_steps + 20

    rows: list[dict[str, Any]] = []
    flags: list[bool] = []
    gaps: list[float] = []
    by_handle: list[list[str]] = []
    try:
        for step in range(max_steps):
            applied = robot.command_body_velocity(command)
            client.step()
            reading = contact.check()
            gap = geometric_gap(sim, collidable_shapes, obstacle)
            by_handle.append(touching_by_handle(
                sim, all_shapes, contact.chassis_handle, contact.obstacles_handle,
                contact.obstacles_no_floor_handle))
            pose = robot.pose()
            measured = robot.body_velocity()
            flags.append(bool(reading.in_contact))
            gaps.append(gap)
            rows.append({
                "step": step, "t": round(step * dt, 4),
                "x": pose[0], "y": pose[1], "yaw": pose[2],
                "cmd_vx": applied[0], "cmd_vy": applied[1], "cmd_wz": applied[2],
                "vx": measured[0], "vy": measured[1], "wz": measured[2],
                "gap_m": gap, "contact": int(reading.in_contact),
                "part": reading.part, "hit": "|".join(reading.names),
                "by_handle": "|".join(by_handle[-1]),
            })
            if reading.in_contact and case.expect_contact:
                break
    finally:
        robot.halt()
        sim.removeObjects([obstacle])
        client.stop()

    with (run_dir / f"{case.name}.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    summary = summarise_case(case, flags, gaps, rows, required_prefix)
    summary["handle_validation"] = contact_agreement(flags, by_handle)
    summary["context"] = context
    return summary


def run_case_in_env_loop(client: SimClient, cfg: dict[str, Any], case: Case, speed: float,
                         run_dir: Path, context: str = "env_step_loop", *, required_prefix: str | None = None) -> dict[str, Any]:
    """The same case, driven through the environment's step loop (`env_step_loop`).

    The contact check is the one the environment calls, in the order the environment calls
    it: command, physics step, scan packet, pose, contact. The per-handle validation of
    ruling 1 is done here, around the loop, so the loop itself stays the one episodes run.

    The certified layer is evaluated and reported but may not override the command
    (`certified.enabled` false in the copy used here): a positive case must reach contact
    for this gate to validate anything, and the emulator would stop the robot first. The
    step at which it WOULD have stopped is recorded.
    """
    import copy

    from pfbnav.envs.step_loop import StepLoop

    geom = ChassisGeometry.from_config(cfg)
    objects = SceneObjects.from_config(cfg)
    dt = float(cfg["sim"]["dt"])
    local = copy.deepcopy(cfg)
    local["certified"]["enabled"] = False

    loop = StepLoop(client, local)
    sim = client.sim
    obstacle: int | None = None
    rows: list[dict[str, Any]] = []
    flags: list[bool] = []
    gaps: list[float] = []
    by_handle: list[list[str]] = []
    would_stop_step: int | None = None

    client.stop()
    loop.install()
    try:
        base_ref = client.object(objects.base_ref)

        def place() -> None:
            nonlocal obstacle
            obstacle = place_obstacle(sim, base_ref, case, geom.yaw_offset)

        loop.reset(settle_steps=20, after_start=place)
        all_shapes = {h: str(sim.getObjectAlias(h, 2))
                      for h in client.shapes_in_tree(client.object(objects.robot_tree))}
        chassis_handle = client.object(objects.chassis)
        collidable = [h for h in all_shapes
                      if h == chassis_handle or client.special_flags(h)["collidable"]]
        loop.contact.bind()          # the obstacle exists now

        command = [axis * speed for axis in case.drive_body]
        approach_steps = math.ceil(case.travel_m / (speed * dt))
        ramp_steps = math.ceil(speed / (float(cfg["actuation"]["a_max"]) * dt))
        for step in range(approach_steps + ramp_steps + 20):
            state = loop.step(command)
            gap = geometric_gap(sim, collidable, obstacle)
            touching = touching_by_handle(sim, all_shapes, loop.contact.chassis_handle,
                                          loop.contact.obstacles_handle,
                                          loop.contact.obstacles_no_floor_handle)
            by_handle.append(touching)
            flags.append(bool(state.in_contact))
            gaps.append(gap)
            if would_stop_step is None and state.certified["stop_active"]:
                would_stop_step = step
            rows.append({
                "step": step, "t": round(step * dt, 4),
                "x": state.pose[0], "y": state.pose[1], "yaw": state.pose[2],
                "cmd_vx": command[0], "cmd_vy": command[1], "cmd_wz": command[2],
                "vx": state.velocity[0], "vy": state.velocity[1], "wz": state.velocity[2],
                "gap_m": gap, "contact": int(state.in_contact),
                "part": "", "hit": "|".join(state.contact_names),
                "by_handle": "|".join(touching),
                "certified_stop_active": int(bool(state.certified["stop_active"])),
                "scan_seconds": state.timings_s.get("scan", 0.0),
                "step_seconds": state.timings_s["whole"],
            })
            if state.in_contact and case.expect_contact:
                break
    finally:
        if obstacle is not None and bool(sim.isHandle(obstacle)):
            sim.removeObjects([obstacle])
        loop.remove()
        client.stop()

    with (run_dir / f"{case.name}_{context}.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    summary = summarise_case(case, flags, gaps, rows, required_prefix)
    summary["handle_validation"] = contact_agreement(flags, by_handle)
    summary["context"] = context
    # What the layer would have done, had it been allowed to: reported, never hidden.
    summary["certified_would_stop_step"] = would_stop_step
    summary["step_seconds_p95"] = float(np.percentile(
        [r["step_seconds"] for r in rows], 95)) if rows else None
    return summary


def summarise_case(case: Case, flags: list[bool], gaps: list[float],
                   rows: list[dict[str, Any]],
                   required_prefix: str | None = None) -> dict[str, Any]:
    """Turn one case's log into a verdict, with the reason when it fails."""
    result: dict[str, Any] = {
        "case": case.name, "note": case.note, "steps": len(flags),
        "min_gap_m": min(gaps) if gaps else None,
        "contact_steps": count_contact_steps(flags),
    }
    if case.expect_contact:
        try:
            lag = detection_lag_steps(flags, gaps)
        except ValueError as exc:
            result.update(passed=False, reason=str(exc), detection_lag_steps=None)
            return result
        hit = next((r["hit"] for r in rows if r["contact"]), "")
        part = next((r["part"] for r in rows if r["contact"]), "")
        struck = next((r["by_handle"] for r in rows if r["contact"]), "")
        result.update(detection_lag_steps=lag, part=part, hit=hit,
                      expect_shape=case.expect_shape, struck=struck)
        # The case must strike the part it was placed for; striking another part while
        # "passing" would report a gate on geometry nobody chose (supervisor, 2026-09-22).
        if case.expect_shape is not None and case.expect_shape not in struck.split("|"):
            result.update(passed=False,
                          reason=f"{case.name} was placed to strike {case.expect_shape} "
                                 f"but {struck or 'nothing'} was struck first")
            return result
        # The declared requirement is checked against what was STRUCK, not only against what
        # the placement predicted: the prediction is geometry, this is the run (A11).
        if required_prefix and not any(s.startswith(required_prefix)
                                       for s in struck.split("|") if s):
            result.update(passed=False,
                          reason=f"{case.name} must strike {required_prefix}* per its "
                                 f"declared criteria, but struck {struck or 'nothing'}")
            return result
    else:
        result.update(detection_lag_steps=None)
    return result


def verdict(summaries: list[dict[str, Any]], criteria: dict[str, Any],
            contexts_run: list[str] | None = None) -> dict[str, Any]:
    """Apply the committed criteria. Thresholds are read, never restated."""
    max_lag = int(criteria["max_detection_lag_steps"])
    max_false = int(criteria["max_false_contacts"])
    for item in summaries:
        if "passed" in item:
            continue
        if item["detection_lag_steps"] is None:
            item["passed"] = item["contact_steps"] <= max_false
            item["reason"] = ("" if item["passed"] else
                              f"{item['contact_steps']} false contacts, at most "
                              f"{max_false} allowed")
        else:
            within = abs(item["detection_lag_steps"]) <= max_lag
            item["passed"] = within
            item["reason"] = ("" if within else
                              f"detection lag {item['detection_lag_steps']} steps, at most "
                              f"{max_lag} allowed")
    validation_ok = all(item.get("handle_validation", {}).get("agree", True)
                        for item in summaries)
    ran = set(contexts_run or ["gate_script"])
    contexts = {str(c): ("RUN" if c in ran else "NOT_RUN") for c in criteria["contexts"]}
    all_cases_pass = all(item["passed"] for item in summaries)
    complete = all(state == "RUN" for state in contexts.values())
    # Per context, so a report says which one failed rather than only that something did.
    per_context = {}
    for name in sorted({str(item.get("context", "gate_script")) for item in summaries}):
        here = [item for item in summaries if item.get("context", "gate_script") == name]
        per_context[name] = {
            "cases": len(here), "passed": all(item["passed"] for item in here),
            "failing": [item["case"] for item in here if not item["passed"]]}
    return {
        "cases": summaries,
        "contexts": contexts,
        "per_context": per_context,
        "all_cases_pass_in_run_contexts": all_cases_pass,
        # Not a gates.yaml criterion: a check on the contact definition itself (ruling 1).
        # Reported separately, and a disagreement fails the run's exit status.
        "handle_validation_agrees": validation_ok,
        "result": ("PASS" if all_cases_pass and complete
                   else "FAIL" if not all_cases_pass else "INCOMPLETE"),
        "incomplete_reason": ("" if complete else
                              f"contexts not run: {sorted(c for c, s in contexts.items() if s != 'RUN')}; "
                              "G0 is not passed until every context in gates.yaml passes"),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    add_display_flag(parser)
    parser.add_argument("--dry-run", action="store_true",
                        help="measure the robot, report which shape each case strikes, and "
                             "stop without driving anything")
    parser.add_argument("--cases", nargs="+",
                        help="run only these cases (a smoke check; a gate run needs all of "
                             "them, and a partial run says so in its summary)")
    parser.add_argument("--contexts", nargs="+", default=["gate_script"],
                        choices=["gate_script", "env_step_loop"],
                        help="which contexts to run; gates.yaml requires both to pass")
    args = parser.parse_args()

    cfg = load_config(args.config)
    cfg = apply_display(cfg, args.display)
    seed_everything(args.seed)
    criteria = yaml.safe_load(PREREG.read_text())["G0_contact_validation"]
    speed = float(criteria["approach_speed"])

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "gates", f"G0_{stamp}", args.seed, cfg)

    scene = REPO_ROOT / cfg["sim"]["scene"]
    if not scene.is_file():
        raise SystemExit(f"scene {scene} does not exist")

    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(scene))

    geom = ChassisGeometry.from_config(cfg)

    # One short start purely to measure the robot, so the corner case is placed on the
    # corner that actually leads and every case knows the shape it must strike.
    client.start(settle_steps=5)
    boxes = shape_points(client, cfg)
    physics = client.physics_settings()
    objects = SceneObjects.from_config(cfg)
    shape_flags = {str(client.sim.getObjectAlias(h, 2)): client.special_flags(h)
                   for h in client.shapes_in_tree(client.object(objects.robot_tree))}
    client.stop()
    off = sorted(name.rsplit("/", 1)[-1] for name, f in shape_flags.items()
                 if not f["collidable"])
    print(f"physics in force: {physics}")
    print(f"robot shapes: {len(shape_flags)}, collidable flag off on {len(off)}: "
          f"{', '.join(off) or 'none'}")
    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    psi = outermost_bearing(table.extent)
    reach = float(table.extent(psi))
    corner_xy = (reach * math.cos(psi), reach * math.sin(psi))
    print(f"envelope's outermost forward point: {reach:.4f} m at "
          f"{math.degrees(psi):+.1f} deg -> ({corner_xy[0]:+.4f}, {corner_xy[1]:+.4f}) m")

    cases, pending = declared_cases(
        expected_shapes(build_cases(geom, corner_xy, psi), boxes), criteria)
    placement = [{"case": c.name, "obstacle_min": obstacle_box(c)[0],
                  "obstacle_max": obstacle_box(c)[1], "drive": c.drive_body,
                  "expect_shape": c.expect_shape, "note": c.note} for c in cases]
    (run_dir / "placement.json").write_text(json.dumps(
        {"outermost_bearing_deg": math.degrees(psi), "outermost_reach_m": reach,
         "corner_xy_m": corner_xy, "cases": placement}, indent=2, default=float))
    print("case placement (which shape each case must strike, measured):")
    for item in placement:
        print(f"  {item['case']:<24} -> {item['expect_shape']}")
    for name in pending:
        print(f"  {name:<24} -> DEFINED BUT NOT PREREGISTERED, not run (amendment pending)")
    if args.dry_run:
        print(f"dry run: nothing driven -> {run_dir}")
        client.stop()
        return 0

    if args.cases:
        unknown = [name for name in args.cases if name not in {c.name for c in cases}]
        if unknown:
            raise SystemExit(f"unknown cases {unknown}")
        cases = [c for c in cases if c.name in args.cases]
        print(f"PARTIAL: only {[c.name for c in cases]} -- not a gate run")

    required = {str(k): str(v)
                for k, v in (criteria.get("case_required_shapes") or {}).items()}
    by_context: dict[str, list[dict[str, Any]]] = {}
    try:
        for context in args.contexts:
            print(f"\n=== context: {context}")
            runner = run_case if context == "gate_script" else run_case_in_env_loop
            summaries = []
            for case in cases:
                print(f"-- {case.name}: {case.note}")
                print(f"   must strike {case.expect_shape}")
                summaries.append(runner(client, cfg, case, speed, run_dir, context,
                                        required_prefix=required.get(case.name)))
                print(f"   {summaries[-1]}")
            by_context[context] = summaries
    finally:
        client.stop()

    summaries = [item for context in args.contexts for item in by_context[context]]
    report = verdict(summaries, criteria, contexts_run=list(args.contexts))
    report["criteria"] = criteria
    if args.cases:
        # A subset cannot be a verdict: say so in the file, not only on the terminal.
        report["result"] = "PARTIAL"
        report["partial_cases"] = list(args.cases)
    report["approach_speed"] = speed
    report["physics"] = physics
    report["robot_shape_flags"] = shape_flags
    report["placement"] = placement
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float))
    stamp_display(run_dir, client.display_status())

    print(f"\nG0 {report['result']}  ->  {run_dir}")
    if report["incomplete_reason"]:
        print(f"   {report['incomplete_reason']}")
    for item in report["cases"]:
        mark = "ok  " if item["passed"] else "FAIL"
        agree = item["handle_validation"]["agree"]
        print(f"   {mark} {item['case']:<24} lag={item['detection_lag_steps']} "
              f"contacts={item['contact_steps']} handles-agree={agree} "
              f"{item.get('reason', '')}")
        for d in item["handle_validation"]["disagreements"][:5]:
            print(f"        step {d['step']}: collection={d['collection']} "
                  f"handles={d['handles']}")
    verdict_line = ("agrees on every step" if report["handle_validation_agrees"]
                    else "DISAGREES")
    print(f"   handle/collection validation: {verdict_line}")
    ok = report["result"] in ("PASS", "INCOMPLETE") and report["handle_validation_agrees"]
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
