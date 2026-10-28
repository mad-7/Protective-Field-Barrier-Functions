"""Instantiate a generated layout into the scene, and check it went in as described.

Simulator I/O; the geometry it works from is pure (`pfbnav/world/layouts.py`). Option 1 of
2026-09-23: the generator is the single source of truth, the bake uses its polygons, and the
same polygons are built here as shapes the scanners can see and the contact check can count.

**The layout moves, not the robot.** A scenario's start is a pose in the layout's own frame;
placing it in the world would mean teleporting a dynamic robot every episode. Instead the
layout is placed so that the scenario's start coincides with wherever the robot actually
stands after `start()`, by a rigid transform the environment keeps (`Pose2D`). Nothing about
the robot is moved, and layout coordinates stay exactly what the generator produced.

**Shapes are created while the simulation runs, so they vanish when it stops.** That is the
episode's cleanup: a reset is stop, start, instantiate, and no handle survives a stop. The
v1 failure this avoids is handle accumulation over tens of thousands of steps (supervisor,
2026-09-23, item 5); `report()` counts what exists so a run can watch for it anyway.

**Static and non-respondable.** Measured on 2026-09-23
(`tests/test_unmapped_obstacle_contact.py`): `sim.checkCollision` sees a non-respondable
shape inside a collection, so an obstacle is counted by the ground-truth contact check while
the robot passes through it rather than being shoved. A collision ends the episode; a shove
would corrupt the trajectory that produced it.

**Every episode checks what it built** (item 4): `verify` reads each shape's pose and extent
back out of the scene, maps it into layout coordinates, and compares it with the polygon it
came from. A mismatch is returned as a list, not raised, so the environment can record what
drifted and fail the episode honestly.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import ArrayLike

from pfbnav.world.baking import Obstacle


@dataclass(frozen=True)
class Pose2D:
    """A rigid transform from layout coordinates into world coordinates."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def apply(self, points: ArrayLike) -> np.ndarray:
        p = np.atleast_2d(np.asarray(points, dtype=float))
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.column_stack([self.x + c * p[:, 0] - s * p[:, 1],
                                self.y + s * p[:, 0] + c * p[:, 1]])

    def invert(self) -> Pose2D:
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        return Pose2D(x=-(c * self.x - s * self.y), y=-(s * self.x + c * self.y),
                      yaw=-self.yaw)

    def apply_yaw(self, yaw: float) -> float:
        return wrap(self.yaw + float(yaw))


def wrap(angle: float) -> float:
    """An angle in (-pi, pi]."""
    a = math.atan2(math.sin(angle), math.cos(angle))
    return math.pi if a == -math.pi else a


def transform_placing(start_layout, robot_world) -> Pose2D:
    """The transform that puts a layout-frame start pose at the robot's world pose.

    Both poses are (x, y, yaw). This is what lets an episode begin without moving the robot.
    """
    sx, sy, syaw = (float(v) for v in start_layout)
    rx, ry, ryaw = (float(v) for v in robot_world)
    yaw = wrap(ryaw - syaw)
    c, s = math.cos(yaw), math.sin(yaw)
    return Pose2D(x=rx - (c * sx - s * sy), y=ry - (s * sx + c * sy), yaw=yaw)


def start_pose_error(transform: Pose2D, start_layout, robot_world) -> tuple[float, float]:
    """How far the robot's measured world pose is from the scenario's start, in the layout.

    Returns (position error in m, absolute yaw error in rad). Zero when the layout was placed
    around the pose the robot actually occupies. Found 2026-10-10: built from a pose read
    before the restart, every hall after the first was placed around the previous episode's
    end pose, metres from where the robot stood.
    """
    inverse = transform.invert()
    xy = inverse.apply([[float(robot_world[0]), float(robot_world[1])]])[0]
    yaw = inverse.apply_yaw(float(robot_world[2]))
    return (float(math.hypot(xy[0] - float(start_layout[0]), xy[1] - float(start_layout[1]))),
            abs(wrap(yaw - float(start_layout[2]))))


def rectangle_of(polygon: ArrayLike) -> tuple[float, float, float, float, float]:
    """A rectangle polygon as (cx, cy, length, width, yaw); yaw is its long axis.

    The generator makes rectangles; anything else is refused here rather than approximated,
    because a shape that does not match its polygon is the failure this module exists to
    prevent.
    """
    p = np.asarray(polygon, dtype=float)
    if p.shape != (4, 2):
        raise ValueError(f"expected a 4-point rectangle, got shape {p.shape}")
    edges = np.roll(p, -1, axis=0) - p
    lengths = np.linalg.norm(edges, axis=1)
    if not (math.isclose(lengths[0], lengths[2], rel_tol=1e-6)
            and math.isclose(lengths[1], lengths[3], rel_tol=1e-6)):
        raise ValueError("opposite sides differ: this polygon is not a rectangle")
    if abs(float(np.dot(edges[0], edges[1]))) > 1e-6 * lengths[0] * lengths[1]:
        raise ValueError("adjacent sides are not perpendicular: not a rectangle")
    centre = p.mean(axis=0)
    long_axis = edges[0] if lengths[0] >= lengths[1] else edges[1]
    return (float(centre[0]), float(centre[1]), float(max(lengths[0], lengths[1])),
            float(min(lengths[0], lengths[1])), wrap(math.atan2(long_axis[1], long_axis[0])))


class LayoutShapes:
    """The shapes one episode's layout put in the scene."""

    def __init__(self, client, cfg: dict[str, Any]):
        self.client = client
        self.cfg = cfg
        layout_cfg = cfg["world"]["layout"]
        self.shelf_height = float(layout_cfg["shelf_height"])
        self.wall_height = float(layout_cfg["wall_height"])
        self.unmapped_height = float(layout_cfg["unmapped"]["height"])
        self.handles: dict[str, int] = {}
        self.transform = Pose2D()

    def height_of(self, obstacle: Obstacle) -> float:
        if not obstacle.mapped:
            return self.unmapped_height
        return self.wall_height if obstacle.name.startswith("wall_") else self.shelf_height

    def instantiate(self, obstacles, transform: Pose2D, floor_z: float = 0.0) -> dict[str, int]:
        """Build every obstacle -- mapped and unmapped alike -- and return its handles."""
        if self.handles:
            raise RuntimeError("this episode's shapes are still in the scene; remove() first")
        sim = self.client.sim
        self.transform = transform
        for obstacle in obstacles:
            cx, cy, length, width, yaw = rectangle_of(obstacle.polygon)
            height = self.height_of(obstacle)
            handle = int(sim.createPrimitiveShape(
                sim.primitiveshape_cuboid, [length, width, height], 0))
            sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
            sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 0)
            world = transform.apply([[cx, cy]])[0]
            sim.setObjectPosition(handle, [float(world[0]), float(world[1]),
                                           float(floor_z + height / 2)], sim.handle_world)
            sim.setObjectOrientation(handle, [0.0, 0.0, transform.apply_yaw(yaw)],
                                     sim.handle_world)
            sim.setObjectAlias(handle, f"layout_{obstacle.name}")
            self.handles[obstacle.name] = handle
        return dict(self.handles)

    def verify(self, obstacles, *, position_tol: float = 1e-3,
               extent_tol: float = 1e-3, yaw_tol: float = 1e-3) -> list[dict[str, Any]]:
        """Read every shape back and compare it with the polygon it came from.

        Returns the mismatches, empty when the scene matches the layout. Run at every reset
        (supervisor, 2026-09-23): drift within an episode is what option 1 exists to prevent.
        """
        sim = self.client.sim
        problems: list[dict[str, Any]] = []
        inverse = self.transform.invert()
        for obstacle in obstacles:
            handle = self.handles.get(obstacle.name)
            if handle is None or not bool(sim.isHandle(handle)):
                problems.append({"name": obstacle.name, "field": "handle",
                                 "detail": "missing from the scene"})
                continue
            cx, cy, length, width, yaw = rectangle_of(obstacle.polygon)
            pos = [float(v) for v in sim.getObjectPosition(handle, sim.handle_world)]
            back = inverse.apply([pos[:2]])[0]
            if abs(back[0] - cx) > position_tol or abs(back[1] - cy) > position_tol:
                problems.append({"name": obstacle.name, "field": "position",
                                 "expected": (cx, cy), "found": (back[0], back[1])})
            world_yaw = float(sim.getObjectOrientation(handle, sim.handle_world)[2])
            if abs(wrap(world_yaw - self.transform.apply_yaw(yaw))) > yaw_tol:
                problems.append({"name": obstacle.name, "field": "yaw",
                                 "expected": self.transform.apply_yaw(yaw),
                                 "found": world_yaw})
            half = [float(v) for v in sim.getVector3Property(handle, "bbHSize")]
            got = (2 * half[0], 2 * half[1], 2 * half[2])
            want = (length, width, self.height_of(obstacle))
            if any(abs(g - w) > extent_tol for g, w in zip(got, want, strict=True)):
                problems.append({"name": obstacle.name, "field": "extent",
                                 "expected": want, "found": got})
        return problems

    def remove(self) -> int:
        """Take this episode's shapes out. Returns how many were removed."""
        sim = self.client.sim
        alive = [h for h in self.handles.values() if bool(sim.isHandle(h))]
        if alive:
            sim.removeObjects(alive)
        removed = len(alive)
        self.handles = {}
        return removed

    def report(self) -> dict[str, Any]:
        """What this episode has in the scene, for the per-episode log (item 5)."""
        sim = self.client.sim
        alive = sum(1 for h in self.handles.values() if bool(sim.isHandle(h)))
        return {"shapes": len(self.handles), "alive": alive,
                "transform": {"x": self.transform.x, "y": self.transform.y,
                              "yaw": self.transform.yaw}}


def unmapped_absent_from_bake(layout, baked) -> list[dict[str, Any]]:
    """Check each `mapped=False` polygon is absent from the baked grid (item 4, pure half).

    The scene half is `LayoutShapes.verify`; together they are the per-episode assertion
    that the world and the map differ exactly where they are meant to.
    """
    problems = []
    for obstacle in layout.unmapped:
        centre = np.asarray(obstacle.polygon, dtype=float).mean(axis=0)
        row, col = baked.cell_of([centre[0]], [centre[1]])
        if bool(baked.occupied[row, col][0]):
            problems.append({"name": obstacle.name, "field": "occupied",
                             "detail": "an unmapped obstacle reached the planner's map"})
    if sorted(baked.unmapped) != sorted(o.name for o in layout.unmapped):
        problems.append({"name": "*", "field": "unmapped_list",
                         "expected": sorted(o.name for o in layout.unmapped),
                         "found": sorted(baked.unmapped)})
    return problems
