"""Ground-truth robot contact from `sim.checkCollision`.

Per docs/decision_log.md (2026-09-21, "Ground-truth contact"), a collision is
defined only by the simulator's geometric contact check. LiDAR ranges never define
a collision: v1 used a 0.20 m range threshold from the robot centre, which is
inside the chassis half-length and therefore both missed real contacts and
invented false ones.

Two checks, unioned:

  * the **chassis**, passed as a handle, against a collection of every object outside the
    robot's subtree;
  * every **other shape in the robot's subtree** against the same collection with the floor's
    tree also taken out.

**What a contact can be with: hall geometry and unmapped obstacles, never the floor.** A
collection honours the collidable flag (2026-09-18, `tests/test_collection_semantics.py`), and
every floor tile is NOT collidable (read back 2026-10-13, `tests/test_floor_sim.py`), so the floor
is in neither set: the second check's exclusion of the floor tree is redundant, not load-bearing.
This docstring said until 2026-10-13 that the chassis was checked against the floor, so that a
tipped or dropped robot would be counted; that never held on this scene. Ruled 2026-10-13
(supervisor): the floor stays non-collidable -- the wheel spheres are collidable and rest on it
permanently -- so the contact metric concerns obstacles only. The target set is asserted by
`tests/test_nav_env_sim.py::test_the_contact_target_set_is_the_hall_and_its_unmapped_obstacles`.

The second group is defined structurally rather than as a list of parts, because a
list goes stale. Measured on 2026-09-17, the wheels stand about 17 mm proud of the
chassis laterally and the forward LiDAR pod 65 mm proud of it longitudinally, so a
wall can strike either with the chassis still clear. A chassis-only check missed
exactly those collisions, and under-counting is the dangerous direction for the
project's headline metric. Taking the whole subtree means a sensor or fixture added
to the robot later is covered the day it appears, instead of the day someone
remembers to add it here.

The robot's own subtree is excluded from both obstacle sets, so arm links and
wheels resting against the chassis are never contacts.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from pfbnav.sim.robot import SceneObjects

# sim.getObjectAlias mode 2: the object's full scene path, for diagnostic logging.
_ALIAS_FULL_PATH = 2


@dataclass(frozen=True)
class ContactReading:
    """Outcome of one ground-truth contact check.

    `part` names which check fired, "chassis" or "attachments" (wheels, sensor pods,
    arm links), and is empty when there is no contact. `handles` and `names` hold the
    colliding *pair* that `sim.checkCollision` reports: the robot-side object first,
    the obstacle second.
    Both sides are kept, because knowing which part was struck is the useful half of
    the diagnostic once the robot side is more than one shape.
    """

    in_contact: bool
    part: str = ""
    handles: tuple[int, ...] = ()
    names: tuple[str, ...] = ()

    def __bool__(self) -> bool:
        return self.in_contact


class RobotContact:
    """Ground-truth contact check over the robot's collidable envelope.

    `bind()` must be called after the simulation has been started and again after
    every restart: collections created through the remote API while a simulation is
    running are destroyed when it stops.
    """

    def __init__(self, client, objects: SceneObjects, *, resolve_names: bool = True):
        self.client = client
        self.objects = objects
        self.resolve_names = resolve_names
        self.chassis_handle: int | None = None
        self.attachments_entity: int | None = None
        self.obstacles_handle: int | None = None
        self.obstacles_no_floor_handle: int | None = None

    @classmethod
    def from_config(cls, client, cfg: dict[str, Any], **kwargs) -> RobotContact:
        return cls(client, SceneObjects.from_config(cfg), **kwargs)

    @property
    def sim(self) -> Any:
        return self.client.sim

    def bind(self) -> None:
        """Resolve handles and build both obstacle sets."""
        self.chassis_handle = self.client.object(self.objects.chassis)
        robot_tree = self.client.object(self.objects.robot_tree)
        floor = self.client.object(self.objects.floor)

        attachments = [h for h in self.client.shapes_in_tree(robot_tree)
                       if h != self.chassis_handle]
        if not attachments:
            raise RuntimeError(
                f"{self.objects.robot_tree!r} contains no shapes besides the chassis, so "
                "the wheels and sensor pods would not be checked for contact and "
                "collisions against them would be under-counted silently"
            )
        self.attachments_entity = self.client.entity_for(attachments)

        self.obstacles_handle = self.client.collection_of_everything_except(robot_tree)
        self.obstacles_no_floor_handle = self.client.collection_of_everything_except(
            robot_tree, floor
        )

    def check(self) -> ContactReading:
        """Return the current contact state of the robot.

        Called once per physics step, after `sim.step()`. Any remote API failure
        propagates: a contact check that silently returned "no contact" would corrupt
        the collision counts that every headline metric rests on.
        """
        if self.chassis_handle is None or self.attachments_entity is None:
            raise RuntimeError("call bind() after starting the simulation")

        for part, entity, obstacles in (
            ("chassis", self.chassis_handle, self.obstacles_handle),
            ("attachments", self.attachments_entity, self.obstacles_no_floor_handle),
        ):
            result, handles = self.sim.checkCollision(entity, obstacles)
            if result:
                return self._reading(part, handles)
        return ContactReading(in_contact=False)

    def _reading(self, part: str, handles) -> ContactReading:
        colliding = tuple(int(h) for h in (handles or ()))
        if not self.resolve_names:
            return ContactReading(in_contact=True, part=part, handles=colliding)
        names = tuple(str(self.sim.getObjectAlias(h, _ALIAS_FULL_PATH)) for h in colliding)
        return ContactReading(in_contact=True, part=part, handles=colliding, names=names)
