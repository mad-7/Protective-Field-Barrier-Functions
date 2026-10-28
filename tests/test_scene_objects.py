"""WP0: every scene object named in the configuration must resolve.

Requires a running CoppeliaSim (`pytest -m sim`).

The chassis and contact tests only ever resolve `chassis`, `base_ref` and
`robot_tree`, so nothing so far has checked `sim.objects.wheels`. A wheel path
that does not resolve makes `YouBotBase.bind()` fail at the first rollout rather
than here, which is the wrong place to find out.

The subtree listing is a diagnostic, not a check: it prints the real paths under
the robot so a mismatch between the configuration and the scene can be read off
directly instead of guessed at.
"""
from __future__ import annotations

import pytest

from pfbnav.sim.client import SimObjectNotFound
from pfbnav.sim.robot import (
    WHEEL_ORDER,
    ChassisGeometry,
    SceneObjects,
    ref_point_in_body,
)

pytestmark = pytest.mark.sim

# How far a wheel joint may sit from its configured lever arm before the
# configuration is treated as wrong. The mixing uses these directly.
WHEEL_POSITION_TOL_M = 0.002


def _configured_paths(objects: SceneObjects) -> dict[str, str]:
    paths = {
        "base_ref": objects.base_ref,
        "chassis": objects.chassis,
        "robot_tree": objects.robot_tree,
    }
    paths.update({f"wheels[{name}]": path for name, path in zip(WHEEL_ORDER, objects.wheels)})
    return paths


def test_youbot_subtree_listing(cfg, sim_client):
    """Print every object under the robot, with its type. Read this on a failure above."""
    sim = sim_client.sim
    objects = SceneObjects.from_config(cfg)
    root = sim_client.object(objects.robot_tree)

    type_names = {
        sim.object_shape_type: "shape",
        sim.object_joint_type: "joint",
        sim.object_dummy_type: "dummy",
    }
    handles = [int(h) for h in sim.getObjectsInTree(root, sim.handle_all, 0)]
    lines = [f"\nobjects under {objects.robot_tree} ({len(handles)} total):"]
    for handle in handles:
        kind = type_names.get(sim.getObjectType(handle), f"type {sim.getObjectType(handle)}")
        lines.append(f"  {kind:<6} {sim.getObjectAlias(handle, 2)}")
    print("\n".join(lines))

    assert handles, f"{objects.robot_tree} has no subtree; the robot is not a model root"


def test_every_configured_object_resolves(cfg, sim_client):
    """Resolve every `sim.objects` path, reporting all failures at once.

    Failures are collected rather than raised on the first miss so that one run
    tells you every path that needs correcting, not just the earliest one.
    """
    objects = SceneObjects.from_config(cfg)
    resolved: dict[str, int] = {}
    missing: dict[str, str] = {}
    for key, path in _configured_paths(objects).items():
        try:
            resolved[key] = sim_client.object(path)
        except SimObjectNotFound:
            missing[key] = path

    print("\nresolved scene objects:")
    for key, handle in resolved.items():
        print(f"  {key:<14} {_configured_paths(objects)[key]:<34} -> handle {handle}")

    assert not missing, (
        "these sim.objects paths do not exist in the scene:\n"
        + "\n".join(f"  {key}: {path}" for key, path in missing.items())
        + "\nSee the subtree listing printed by test_youbot_subtree_listing for the real "
        "paths, and correct configs/base.yaml (not this test)."
    )


def test_wheel_layout_matches_the_configuration(cfg, sim_client):
    """Each wheel sits where its name says, at the configured lever arms.

    The mecanum mixing assumes `[fl, fr, rl, rr]` are physically front-left,
    front-right, rear-left, rear-right. Two swapped labels give a robot that still
    moves and still closes a square, but mixes the wrong wheels -- so the layout is
    checked rather than trusted. The lever arms `half_wheelbase` and `half_track`
    enter the mixing directly and are measured here from the joints themselves.
    """
    geom = ChassisGeometry.from_config(cfg)
    objects = SceneObjects.from_config(cfg)
    sim = sim_client.sim
    base_ref = sim_client.object(objects.base_ref)

    expected_sign = {"fl": (+1, +1), "fr": (+1, -1), "rl": (-1, +1), "rr": (-1, -1)}
    rows, wrong_quadrant, wrong_lever = [], [], []
    for name, path in zip(WHEEL_ORDER, objects.wheels):
        handle = sim_client.object(path)
        body = ref_point_in_body(sim.getObjectPosition(handle, base_ref), geom.yaw_offset)
        rows.append((name, body))
        sx, sy = expected_sign[name]
        if not (sx * body[0] > 0 and sy * body[1] > 0):
            wrong_quadrant.append(name)
        if (abs(abs(body[0]) - geom.half_wheelbase) > WHEEL_POSITION_TOL_M
                or abs(abs(body[1]) - geom.half_track) > WHEEL_POSITION_TOL_M):
            wrong_lever.append(name)

    print("\n".join([
        "",
        "wheel positions in the body frame (+x forward, +y left), m",
        (f"  configured lever arms: half_wheelbase {geom.half_wheelbase:.4f}, "
        f"half_track {geom.half_track:.4f}"),
    ] + [f"  {name}: x={body[0]:+.4f}  y={body[1]:+.4f}  z={body[2]:+.4f}"
         for name, body in rows]))

    assert not wrong_quadrant, (
        f"these wheels are not in the quadrant their name claims: {wrong_quadrant}. "
        "The mecanum mixing would drive the wrong wheels, which a square-driving test "
        "would not reveal. Fix the order of sim.objects.wheels in configs/base.yaml."
    )
    assert not wrong_lever, (
        f"these wheels are further than {WHEEL_POSITION_TOL_M} m from the configured "
        f"lever arms: {wrong_lever}. robot.half_wheelbase / half_track feed the mixing "
        "directly, so update them (not this test) from the printed positions."
    )


def test_the_four_wheel_joints_are_distinct(cfg, sim_client):
    """Four different joints, in [fl, fr, rl, rr] order.

    Two paths resolving to the same handle would make the mecanum mixing command
    one wheel twice and leave another dead, which a square-driving smoke test can
    easily fail to notice.
    """
    objects = SceneObjects.from_config(cfg)
    handles = [sim_client.object(path) for path in objects.wheels]
    assert len(set(handles)) == 4, (
        f"the four wheel paths resolve to {len(set(handles))} distinct handles: "
        f"{dict(zip(WHEEL_ORDER, handles))}"
    )
    sim = sim_client.sim
    for name, handle in zip(WHEEL_ORDER, handles):
        assert sim.getObjectType(handle) == sim.object_joint_type, (
            f"sim.objects.wheels[{name}] is not a joint"
        )
