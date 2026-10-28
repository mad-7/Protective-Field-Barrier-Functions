"""WP0 item 2: exercise the ground-truth contact check against the real simulator.

Requires a running CoppeliaSim (`pytest -m sim`). `tests/test_contact.py` pins the
wiring against a fake simulator; this pins what only a real instance can answer:

  * that the obstacle collections really exclude the robot's own subtree, so the
    wheels and arm links cannot register as collisions,
  * that the robot reads clear at spawn,
  * that an actual overlap reads as contact, and clears when it is removed,
  * that a wall in the band outside the chassis but inside a wheel or a sensor pod
    is counted -- the under-count this check was widened to fix.

Hard rule 4 makes `sim.checkCollision` the sole definition of a collision, so this
path carries every headline metric.

Run with `-s` to see the reported handles.
"""
from __future__ import annotations

import pytest

from pfbnav.sim.contact import RobotContact
from pfbnav.sim.robot import ChassisGeometry, SceneObjects, body_point_in_ref

pytestmark = pytest.mark.sim

# Side length of the temporary probe cuboid, in metres. Smaller than the chassis
# so it sits wholly inside it when placed at the chassis origin.
PROBE_SIZE_M = 0.1

# Thin walls placed outside the chassis but inside an attachment, measured 2026-09-17:
# laterally between the chassis face (0.1789 m) and the wheels (about 0.1958 m), and
# forward between the chassis face (0.2854 m) and the LiDAR pod (about 0.3500 m).
BAND_WALL_SIZE_M = (0.9, 0.006, 0.5)
BAND_WALL_Z_M = 0.05
BAND_CASES = [
    (1, 0.188, "wheels, lateral"),
    (0, 0.320, "LiDAR pod, forward"),
]


@pytest.fixture(scope="module")
def contact(cfg, sim_client) -> RobotContact:
    sensor = RobotContact.from_config(sim_client, cfg)
    sensor.bind()
    return sensor


def test_obstacle_collections_exclude_the_robot_subtree(cfg, sim_client, contact):
    sim = sim_client.sim
    objects = SceneObjects.from_config(cfg)
    robot_tree = sim_client.object(objects.robot_tree)
    floor = sim_client.object(objects.floor)
    in_robot = {int(h) for h in sim.getObjectsInTree(robot_tree, sim.handle_all, 0)}

    for label, handle in (("chassis set", contact.obstacles_handle),
                          ("wheel set", contact.obstacles_no_floor_handle)):
        members = {int(h) for h in sim.getCollectionObjects(handle)}
        print(f"\n{label}: {len(members)} objects")
        assert members, f"the {label} is empty; nothing could ever be detected"
        leaked = members & in_robot
        assert not leaked, (
            f"the {label} contains {len(leaked)} objects from the robot's own subtree "
            f"(handles {sorted(leaked)}); self-contact would be counted as a collision"
        )

    chassis_members = {int(h) for h in sim.getCollectionObjects(contact.obstacles_handle)}
    wheel_members = {int(h) for h in sim.getCollectionObjects(contact.obstacles_no_floor_handle)}
    floor_tree = {int(h) for h in sim.getObjectsInTree(floor, sim.handle_all, 0)}
    assert floor_tree & chassis_members, (
        "the floor must stay in the chassis obstacle set, or a tip-over goes uncounted"
    )
    assert not (floor_tree & wheel_members), (
        "the floor must be out of the wheel obstacle set, or the wheels report contact "
        "with the ground on every single step"
    )


def test_robot_is_clear_at_spawn(sim_client, contact):
    """A robot that reads as colliding at spawn would poison every run."""
    for _ in range(5):
        sim_client.step()
    reading = contact.check()
    assert not reading.in_contact, (
        f"contact at the spawn pose on the {reading.part} check, with "
        f"{reading.names or reading.handles}; either the spawn pose is inside scene "
        "geometry or an obstacle set is wrong"
    )


def test_contact_is_detected_for_a_real_overlap_and_clears(sim_client, contact):
    """Place a cuboid inside the chassis, confirm detection, remove it, confirm clear.

    Also records whether an object created *after* `bind()` is seen without
    rebinding. The 2026-09-17 run answered yes, so WP1 scenario generation need not
    rebind after spawning obstacles; it is re-checked here rather than assumed.
    """
    sim = sim_client.sim
    position = sim.getObjectPosition(contact.chassis_handle, -1)

    probe = int(sim.createPrimitiveShape(sim.primitiveshape_cuboid, [PROBE_SIZE_M] * 3))
    try:
        sim.setObjectInt32Param(probe, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(probe, sim.shapeintparam_respondable, 0)
        sim.setObjectPosition(probe, -1, position)

        without_rebind = contact.check()
        contact.bind()
        with_rebind = contact.check()
        print(f"\nprobe cuboid inside the chassis: detected without rebind="
              f"{without_rebind.in_contact}, after rebind={with_rebind.in_contact}, "
              f"part={with_rebind.part}, with {with_rebind.names or with_rebind.handles}")

        assert with_rebind.in_contact, (
            "a cuboid placed inside the chassis was not detected even after rebinding"
        )
        assert with_rebind.part == "chassis"
    finally:
        sim.removeObjects([probe])

    contact.bind()
    assert not contact.check().in_contact, (
        "the robot still reports contact after the probe was removed; the contact check "
        "is latching or a collection holds a stale handle"
    )


@pytest.mark.parametrize("axis, offset, label", BAND_CASES)
def test_a_wall_outside_the_chassis_but_inside_an_attachment_is_counted(
    cfg, sim_client, contact, axis, offset, label
):
    """Regression test for the under-count that widened this check.

    Each wall sits outside the chassis face and inside an attachment: a wheel
    sideways, the LiDAR pod forwards. Before the attachments joined the contact
    entity these were real collisions that reported clear.
    """
    geom = ChassisGeometry.from_config(cfg)
    sim = sim_client.sim
    base_ref = sim_client.object(SceneObjects.from_config(cfg).base_ref)

    # Thin along the axis being tested, long across it.
    span, thickness, height = BAND_WALL_SIZE_M[0], BAND_WALL_SIZE_M[1], BAND_WALL_SIZE_M[2]
    size = [span, span, height]
    size[axis] = thickness
    point = [0.0, 0.0, BAND_WALL_Z_M]
    point[axis] = offset

    wall = int(sim.createPrimitiveShape(sim.primitiveshape_cuboid, size))
    try:
        sim.setObjectInt32Param(wall, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(wall, sim.shapeintparam_respondable, 0)
        sim.setObjectOrientation(wall, base_ref, [0.0, 0.0, float(geom.yaw_offset)])
        sim.setObjectPosition(wall, base_ref,
                              body_point_in_ref(point, geom.yaw_offset).tolist())

        contact.bind()
        reading = contact.check()
        chassis_alone = sim.checkCollision(contact.chassis_handle, contact.obstacles_handle)[0]
        print(f"\nwall at {offset} m ({label}): contact={reading.in_contact} via "
              f"{reading.part or 'nothing'} {reading.names or ''}; "
              f"chassis-only check would report {bool(chassis_alone)}")

        assert reading.in_contact, (
            f"a wall at {offset} m ({label}) was not counted. Either that part is not in "
            "the contact entity, or its measured face is wrong."
        )
        assert reading.part == "attachments", (
            f"the wall was caught by the {reading.part} check; it is meant to sit outside "
            "the chassis face, so the band being tested is not the one intended"
        )
    finally:
        sim.removeObjects([wall])

    contact.bind()
    assert not contact.check().in_contact
