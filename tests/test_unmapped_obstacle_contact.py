"""Does a non-respondable obstacle register as a collision? (`pytest -m sim`)

Layouts are generated as polygons and instantiated into the scene at reset (decision,
2026-09-23), and an unmapped obstacle is one of them: absent from the planner's map,
present in the world. Those shapes are static and non-respondable, so the robot passes
THROUGH them physically rather than being shoved -- and the whole design then rests on one
question the supervisor asked on 2026-09-23:

    does `sim.checkCollision` see a non-respondable shape inside a collection?

`respondable` and `collidable` are different flags with different semantics: respondable
governs the dynamics solver, collidable governs collision *queries*. If a non-respondable
obstacle were invisible to the query, every strike against an unmapped obstacle would go
uncounted and the headline safety result would be silently wrong (hard rule 4 makes the
contact check the sole definition of a collision).

Each shape here exists in memory only and is removed afterwards; the scene is never saved.
"""
from __future__ import annotations

import numpy as np
import pytest

from pfbnav.sim.contact import RobotContact
from pfbnav.sim.robot import SceneObjects, body_point_in_ref

pytestmark = pytest.mark.sim

BOX = (0.30, 0.30, 0.40)        # a shelf-sized block, in metres
AHEAD_M = 0.20                  # placed to overlap the robot's front, so contact is certain


@pytest.fixture
def contact(cfg, sim_client):
    sensor = RobotContact.from_config(sim_client, cfg)
    sensor.bind()
    return sensor


def place_block(sim_client, cfg, *, respondable: bool, ahead_m: float = AHEAD_M) -> int:
    sim = sim_client.sim
    objects = SceneObjects.from_config(cfg)
    ref = sim_client.object(objects.base_ref)
    ref_z = float(sim.getObjectPosition(ref, sim.handle_world)[2])
    block = int(sim.createPrimitiveShape(sim.primitiveshape_cuboid, list(BOX), 0))
    sim.setObjectInt32Param(block, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(block, sim.shapeintparam_respondable, 1 if respondable else 0)
    x, y, z = body_point_in_ref([ahead_m, 0.0, 0.5 * BOX[2] - ref_z],
                                float(cfg["robot"]["yaw_offset"]))
    sim.setObjectPosition(block, [float(x), float(y), float(z)], ref)
    return block


def flags(sim, handle) -> dict[str, bool]:
    return {"static": bool(sim.getObjectInt32Param(handle, sim.shapeintparam_static)),
            "respondable": bool(sim.getObjectInt32Param(
                handle, sim.shapeintparam_respondable))}


@pytest.mark.parametrize("respondable", [True, False])
def test_a_block_overlapping_the_robot_registers_whether_or_not_it_is_respondable(
        cfg, sim_client, contact, respondable):
    """The ruling's question, asked of the simulator rather than of the documentation."""
    block = place_block(sim_client, cfg, respondable=respondable)
    try:
        contact.bind()                       # the obstacle exists now
        reading = contact.check()
        print(f"\nrespondable={respondable}: flags {flags(sim_client.sim, block)}, "
              f"contact={reading.in_contact}, part={reading.part}, names={reading.names}")
        assert reading.in_contact, (
            "a non-respondable obstacle was invisible to the contact check: every strike "
            "against an unmapped obstacle would go uncounted")
    finally:
        if bool(sim_client.sim.isHandle(block)):
            sim_client.sim.removeObjects([block])


def test_a_non_respondable_block_clear_of_the_robot_does_not_register(cfg, sim_client,
                                                                     contact):
    """The other side: the check must not fire merely because a shape exists."""
    block = place_block(sim_client, cfg, respondable=False, ahead_m=2.0)
    try:
        contact.bind()
        assert not contact.check().in_contact
    finally:
        if bool(sim_client.sim.isHandle(block)):
            sim_client.sim.removeObjects([block])


def test_the_robot_is_not_pushed_by_a_non_respondable_block(cfg, sim_client, contact):
    """Why non-respondable: the robot passes through rather than being shoved, so a run
    measures the controller's behaviour and not a collision impulse."""
    sim = sim_client.sim
    objects = SceneObjects.from_config(cfg)
    ref = sim_client.object(objects.base_ref)
    before = np.array(sim.getObjectPosition(ref, sim.handle_world), dtype=float)
    block = place_block(sim_client, cfg, respondable=False)
    try:
        contact.bind()
        for _ in range(10):
            sim_client.step()
        after = np.array(sim.getObjectPosition(ref, sim.handle_world), dtype=float)
        assert np.linalg.norm(after[:2] - before[:2]) < 0.01, (
            f"the robot moved {np.linalg.norm(after[:2] - before[:2]):.4f} m: the block "
            "pushed it, so it is not behaving as non-respondable")
        assert contact.check().in_contact, "still overlapping, so still a contact"
    finally:
        if bool(sim_client.sim.isHandle(block)):
            sim_client.sim.removeObjects([block])
