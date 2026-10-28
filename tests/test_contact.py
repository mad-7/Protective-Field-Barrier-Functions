"""Wiring tests for the ground-truth contact check, against a fake simulator.

These do not test collision geometry -- that is the simulator's job. They pin the
contract that makes the check ground truth: the chassis against everything
including the floor, every other robot shape against everything except the floor,
the robot's own subtree excluded from both, and no failure mode in which a contact
is silently reported as "clear".
"""
import pytest

from pfbnav.sim.contact import RobotContact
from pfbnav.sim.robot import SceneObjects

OBJECTS = SceneObjects(
    base_ref="/youBot/youBot_ref",
    chassis="/youBot",
    robot_tree="/youBot",
    floor="/floor",
    wheels=tuple(f"/youBot/rollingJoint_{w}" for w in ("fl", "fr", "rl", "rr")),
)

CHASSIS = ROBOT_TREE = 11
FLOOR = 20
OBSTACLES, OBSTACLES_NO_FLOOR, ATTACHMENTS_ENTITY = 900, 901, 902

# What FakeClient.shapes_in_tree reports for the robot: the chassis plus four parts.
ROBOT_SHAPES = (CHASSIS, 101, 102, 103, 104)


class FakeSim:
    """Records `checkCollision` arguments and replays a scripted result per call."""

    def __init__(self, results=None, handles=(41, 42)):
        self.results = list(results or [])
        self.handles = handles
        self.calls = []

    def checkCollision(self, entity1, entity2):
        self.calls.append((entity1, entity2))
        result = self.results.pop(0) if self.results else False
        return result, (list(self.handles) if result else [])

    def getObjectAlias(self, handle, _mode):
        return f"object_{handle}"


class FakeClient:
    """Minimal stand-in for SimClient, recording how the entities were built."""

    def __init__(self, sim, robot_shapes=ROBOT_SHAPES):
        self.sim = sim
        self.robot_shapes = list(robot_shapes)
        self.handles = {"/youBot": CHASSIS, "/youBot/youBot_ref": 12, "/floor": FLOOR}
        self.handles.update({path: 50 + i for i, path in enumerate(OBJECTS.wheels)})
        self.excluded: list[tuple[int, ...]] = []
        self.entities: list[tuple[int, ...]] = []

    def object(self, path):
        return self.handles[path]

    def collection_of_everything_except(self, *roots):
        self.excluded.append(tuple(roots))
        return OBSTACLES if len(roots) == 1 else OBSTACLES_NO_FLOOR

    def shapes_in_tree(self, root):
        assert root == ROBOT_TREE
        return list(self.robot_shapes)

    def entity_for(self, handles):
        self.entities.append(tuple(handles))
        return ATTACHMENTS_ENTITY


def _bound(sim, **kwargs):
    client = FakeClient(sim)
    contact = RobotContact(client, OBJECTS, **kwargs)
    contact.bind()
    return client, contact


# ----------------------------------------------------------------------
# What bind() builds
# ----------------------------------------------------------------------
def test_bind_excludes_the_robot_subtree_from_both_obstacle_sets():
    client, _ = _bound(FakeSim())
    assert client.excluded == [(ROBOT_TREE,), (ROBOT_TREE, FLOOR)], (
        "the chassis set keeps the floor so a tip-over is counted; the attachment set "
        "drops it because the wheels rest on it"
    )


def test_bind_takes_every_robot_shape_except_the_chassis():
    """Structural, not a list of parts: a sensor added later is covered on day one."""
    client, _ = _bound(FakeSim())
    assert client.entities == [tuple(h for h in ROBOT_SHAPES if h != CHASSIS)]


def test_a_robot_with_no_shapes_besides_the_chassis_is_an_error():
    """Silently checking the chassis alone would under-count collisions."""
    client = FakeClient(FakeSim(), robot_shapes=(CHASSIS,))
    with pytest.raises(RuntimeError, match="no shapes besides the chassis"):
        RobotContact(client, OBJECTS).bind()


# ----------------------------------------------------------------------
# What check() asks
# ----------------------------------------------------------------------
def test_check_asks_the_chassis_then_the_attachments():
    sim = FakeSim()
    _, contact = _bound(sim)
    contact.check()
    assert sim.calls == [(CHASSIS, OBSTACLES), (ATTACHMENTS_ENTITY, OBSTACLES_NO_FLOOR)]


def test_no_collision_reports_clear():
    reading = _bound(FakeSim())[1].check()
    assert reading.in_contact is False
    assert reading.part == ""
    assert reading.handles == ()
    assert not reading


def test_a_chassis_collision_short_circuits_the_attachment_check():
    sim = FakeSim(results=[True])
    _, contact = _bound(sim)
    reading = contact.check()
    assert reading.in_contact is True
    assert reading.part == "chassis"
    assert reading.handles == (41, 42)
    assert reading.names == ("object_41", "object_42")
    assert len(sim.calls) == 1, "no need to ask again once the chassis has answered"


def test_an_attachment_collision_is_reported_when_the_chassis_is_clear():
    """The 17 mm wheel band and the 65 mm LiDAR band the chassis-only check missed."""
    sim = FakeSim(results=[False, True])
    _, contact = _bound(sim)
    reading = contact.check()
    assert reading.in_contact is True
    assert reading.part == "attachments"
    assert sim.calls[1] == (ATTACHMENTS_ENTITY, OBSTACLES_NO_FLOOR)
    assert reading


def test_a_collision_without_handles_is_still_a_collision():
    _, contact = _bound(FakeSim(results=[True], handles=()))
    reading = contact.check()
    assert reading.in_contact is True
    assert reading.handles == ()


def test_name_resolution_can_be_switched_off():
    _, contact = _bound(FakeSim(results=[True]), resolve_names=False)
    reading = contact.check()
    assert reading.in_contact is True
    assert reading.part == "chassis"
    assert reading.names == ()


def test_checking_before_binding_raises():
    contact = RobotContact(FakeClient(FakeSim()), OBJECTS)
    with pytest.raises(RuntimeError):
        contact.check()


def test_a_simulator_failure_propagates():
    """A failed contact check must never be swallowed into "no contact"."""
    class BrokenSim(FakeSim):
        def checkCollision(self, entity1, entity2):
            raise RuntimeError("remote API error")

    _, contact = _bound(BrokenSim())
    with pytest.raises(RuntimeError, match="remote API error"):
        contact.check()
