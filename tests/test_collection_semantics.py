"""Does `sim.checkCollision` see the same shapes through a handle and through a collection?

Requires a running CoppeliaSim (`pytest -m sim`).

The third footprint probe (FOOTPRINT_20260918T133012Z) found eight robot shapes -- the four
`wheel_respondable_*` spheres and the four `intermediateLink_*` -- that register against a
probe plate when passed as a single handle, and do not register when the same shapes are
passed inside a multi-item collection. The collidable envelope, measured against a
collection, put the robot's side at 0.1958 m; the wheel spheres, probed one by one, reach
0.2085 m.

This test pins that down at one pose with nothing else in play: a thin plate placed at
+90 deg where it cuts the sphere and the link but not the visual wheel mesh, then asked
three ways. It also reads each shape's collidable flag, the most likely explanation being
that collections honour that flag and explicit handles do not.

Reports rather than asserting a particular answer: which semantics CoppeliaSim has is a
fact to record. It asserts only that the probe is placed as intended.
"""
from __future__ import annotations

import math

import pytest

from pfbnav.sim.robot import ChassisGeometry, SceneObjects, body_point_in_ref

pytestmark = pytest.mark.sim

PLATE_THICKNESS_M = 0.006
# Plate centre along body +y: covers [0.201, 0.207] m, cutting the sphere (to 0.2083) and
# the link (to 0.2037) but clear of the wheel mesh (to 0.1958).
PLATE_OFFSET_M = 0.204


def _flag(sim, handle, name):
    """The object's special-property bit `name`, or None if this build lacks the call."""
    if not hasattr(sim, "getObjectSpecialProperty"):
        return None
    bit = getattr(sim, f"objectspecialproperty_{name}")
    return bool(int(sim.getObjectSpecialProperty(handle)) & int(bit))


def test_handle_versus_collection(cfg, sim_client):
    geom = ChassisGeometry.from_config(cfg)
    objects = SceneObjects.from_config(cfg)
    sim = sim_client.sim
    base_ref = sim_client.object(objects.base_ref)
    shapes = sim_client.shapes_in_tree(sim_client.object(objects.robot_tree))
    names = {h: str(sim.getObjectAlias(h, 2)) for h in shapes}
    left = [h for h, n in names.items()
            if n.endswith(("_fl", "_rl")) and ("wheel_respondable" in n
                                                or n.endswith(("intermediateLink_fl",
                                                               "intermediateLink_rl"))
                                                or "swedishWheel" in n)]

    plate = int(sim.createPrimitiveShape(
        sim.primitiveshape_cuboid, [PLATE_THICKNESS_M, 3.0, 3.0]))
    sim.setObjectInt32Param(plate, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(plate, sim.shapeintparam_respondable, 0)
    sim.setObjectOrientation(plate, base_ref, [0.0, 0.0, geom.yaw_offset + math.pi / 2])
    sim.setObjectPosition(plate, base_ref, body_point_in_ref(
        [0.0, PLATE_OFFSET_M, 0.5], geom.yaw_offset).tolist())

    lines = ["", (f"plate at body +y = {PLATE_OFFSET_M} m, "
                  f"{PLATE_THICKNESS_M * 1000:.0f} mm thick"), ""]
    lines.append(f"{'handle':>7} {'in coll.':>9} {'collid.':>8} {'measur.':>8} "
                 f"{'respond.':>9}  shape")
    results = {}
    try:
        whole = sim_client.entity_for(shapes)
        for h in sorted(left, key=lambda h: names[h]):
            alone = bool(sim.checkCollision(plate, h)[0])
            # A two-item collection: this shape and the chassis, which the plate misses.
            chassis = sim_client.object(objects.chassis)
            pair = sim_client.entity_for([h, chassis])
            in_pair = bool(sim.checkCollision(plate, pair)[0])
            results[names[h]] = (alone, in_pair)
            respondable = bool(sim.getObjectInt32Param(h, sim.shapeintparam_respondable))
            lines.append(f"{alone!s:>7} {in_pair!s:>9} {_flag(sim, h, 'collidable')!s:>8} "
                         f"{_flag(sim, h, 'measurable')!s:>8} {respondable!s:>9}  {names[h]}")
        whole_hit = bool(sim.checkCollision(plate, whole)[0])
        lines.append("")
        lines.append(f"whole-robot collection touched: {whole_hit}")
    finally:
        sim.removeObjects([plate])
    print("\n".join(lines))

    spheres = [v for k, v in results.items() if "wheel_respondable" in k]
    meshes = [v for k, v in results.items() if "swedishWheel" in k]
    assert spheres, "no wheel_respondable shapes found on the left side; check the names"
    assert all(alone for alone, _ in spheres), (
        "the plate does not cut the wheel spheres even as single handles; it is misplaced, "
        "so the comparison means nothing")
    assert not any(alone for alone, _ in meshes), (
        "the plate cuts the visual wheel meshes too; it is too far in to separate them")


def test_wheel_spheres_are_collidable_in_the_scene(cfg, sim_client):
    """Ruling 1 on the WP0 results report: the four wheel spheres count as collisions.

    They carry the wheels' physics and reach 12.5 mm past the visual wheel mesh sideways,
    so with their flag off a wheel could press a shelf with no contact recorded. The flag
    lives in the scene file, so this guards against a scene re-saved without it.
    """
    objects = SceneObjects.from_config(cfg)
    sim = sim_client.sim
    shapes = sim_client.shapes_in_tree(sim_client.object(objects.robot_tree))
    spheres = {str(sim.getObjectAlias(h, 2)): sim_client.special_flags(h)["collidable"]
               for h in shapes if "wheel_respondable" in str(sim.getObjectAlias(h, 2))}
    assert len(spheres) == 4, f"expected four wheel spheres, found {sorted(spheres)}"
    off = sorted(name for name, on in spheres.items() if not on)
    assert not off, f"collidable flag off on {off}; see the WP0 results rulings (ruling 1)"
