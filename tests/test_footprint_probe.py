"""Measure the robot's real collidable envelope with `sim.checkCollision`.

Requires a running CoppeliaSim (`pytest -m sim`).

`test_wheels_against_the_chassis_footprint` bounds the envelope from bounding
boxes, which is exact for the chassis (its box is axis-aligned in the body frame
to 0.001 rad) but badly conservative for the wheels, whose frames are rolled about
their axles: on 2026-09-17 it reported the wheels reaching 0.3133 m along body +x,
which `half_wheelbase + wheel_radius = 0.278 m` says is impossible.

This measures the same quantity with the API that *defines* a collision. A thin
plate, aligned with the body frame, is swept along a body axis; at each offset it
is asked whether it touches the chassis, the wheels, or any part of the robot. The
outermost offset at which a group still responds is that group's outer edge, to the
sweep resolution. What matters for hard rule 4 is the gap between the chassis edge
and the whole-robot edge: geometry in that band can be struck without the contact
check seeing it.

The sweep starts *inside* the chassis, so a clean profile is contact-then-no-contact
with no re-entry. Both the starting contact and the monotonicity are asserted: on the
first attempt the plate's orientation was never set, and the profile it produced was
meaningless. Reporting a number from a misplaced probe is worse than reporting none.

Reports rather than fails on the envelope itself. Changing what counts as a
collision is a methodological decision, not a test outcome.
"""
from __future__ import annotations

import numpy as np
import pytest

from pfbnav.sim.robot import ChassisGeometry, SceneObjects, body_point_in_ref

pytestmark = pytest.mark.sim

# Sweep resolution in metres. Also the uncertainty on every edge reported here.
SWEEP_STEP = 0.0025

# How far inside the measured envelope the configured footprint may sit before it is
# treated as under-stating the robot. One sweep step, no more: this is the safety-relevant
# direction, so the allowance is the measurement resolution and nothing else.
FOOTPRINT_TOL_M = SWEEP_STEP

# The probe plate: thin along the swept axis, long across it, and tall enough to
# reach from below the wheels to above the folded arm.
PLATE_THICKNESS_M = 0.006
PLATE_SPAN_M = 0.90
PLATE_HEIGHT_M = 0.50
PLATE_CENTRE_Z_M = 0.25


def _make_plate(sim, axis: int, base_ref: int, yaw_offset: float) -> int:
    """A thin slab whose own axes are aligned with the robot's body frame.

    Setting the orientation relative to base_ref is what makes the sweep mean
    anything: the slab's thin axis has to point along the axis being swept.
    """
    size = [PLATE_SPAN_M, PLATE_SPAN_M, PLATE_HEIGHT_M]
    size[axis] = PLATE_THICKNESS_M
    plate = int(sim.createPrimitiveShape(sim.primitiveshape_cuboid, size))
    sim.setObjectInt32Param(plate, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(plate, sim.shapeintparam_respondable, 0)
    sim.setObjectOrientation(plate, base_ref, [0.0, 0.0, float(yaw_offset)])
    return plate


def _sweep_axis(sim, plate, base_ref, yaw_offset, axis, sign, offsets, groups):
    """Contact profile of each group over `offsets`, as a list of booleans.

    `offsets` are distances from the body origin; `sign` picks the direction along
    `axis`. Both directions are swept because the robot is not symmetric a priori --
    the forward LiDAR pod had no counterpart until a rear one was fitted, and a sweep
    of one side only would have missed it.
    """
    profiles: dict[str, list[bool]] = {name: [] for name in groups}
    for offset in offsets:
        point = [0.0, 0.0, PLATE_CENTRE_Z_M]
        point[axis] = sign * float(offset)
        sim.setObjectPosition(plate, base_ref, body_point_in_ref(point, yaw_offset).tolist())
        for name, entity in groups.items():
            profiles[name].append(bool(sim.checkCollision(plate, entity)[0]))
    return profiles


def _edge_of(offsets, hits) -> tuple[float | None, bool]:
    """(outermost offset with contact, whether contact ran unbroken from the start).

    A gap is expected for a group that does not span the body centre -- the wheels, for
    instance, are not touched by a plate passing between them. It is only a fault for the
    chassis, which the sweep deliberately starts inside.
    """
    if not any(hits):
        return None, True
    last = max(i for i, hit in enumerate(hits) if hit)
    return float(offsets[last]), all(hits[: last + 1])


@pytest.mark.parametrize("axis, sign, axis_name, lo, hi", [
    (1, +1, "body +y (left)", 0.100, 0.300),
    (1, -1, "body -y (right)", 0.100, 0.300),
    (0, +1, "body +x (forward)", 0.200, 0.450),
    (0, -1, "body -x (rear)", 0.200, 0.450),
])
def test_collidable_envelope_along_a_body_axis(cfg, sim_client, axis, sign, axis_name, lo, hi):
    geom = ChassisGeometry.from_config(cfg)
    objects = SceneObjects.from_config(cfg)
    sim = sim_client.sim
    base_ref = sim_client.object(objects.base_ref)

    wheel_shapes = [h for path in objects.wheels
                    for h in sim_client.shapes_in_tree(sim_client.object(path))]
    groups = {
        "chassis": sim_client.entity_for([sim_client.object(objects.chassis)]),
        "wheels": sim_client.entity_for(wheel_shapes),
        "whole robot": sim_client.entity_for(
            sim_client.shapes_in_tree(sim_client.object(objects.robot_tree))),
    }
    offsets = np.arange(lo, hi + SWEEP_STEP / 2, SWEEP_STEP)

    plate = _make_plate(sim, axis, base_ref, geom.yaw_offset)
    try:
        profiles = _sweep_axis(sim, plate, base_ref, geom.yaw_offset, axis, sign,
                               offsets, groups)
    finally:
        sim.removeObjects([plate])

    edges = {name: _edge_of(offsets, hits) for name, hits in profiles.items()}
    configured = geom.half_width if axis == 1 else geom.half_length
    half_plate = PLATE_THICKNESS_M / 2.0

    def bracket(edge: float) -> tuple[float, float]:
        """Interval containing the true outer face, from the last offset that touched.

        The plate spans [offset - half, offset + half], so contact means
        offset - half < face, and the first offset that missed gives
        face <= offset + step - half. Reporting `edge + half` instead would
        overstate every face by a plate thickness.
        """
        return edge - half_plate, edge + SWEEP_STEP - half_plate

    lines = ["", (f"collidable envelope along {axis_name}, swept at {SWEEP_STEP} m "
                 f"over [{lo}, {hi}]"),
             f"  configured half-extent: {configured:.4f} m", ""]
    for name, (edge, clean) in edges.items():
        if edge is None:
            lines.append(f"  {name:<12} : no contact anywhere in the swept range")
        else:
            low, high = bracket(edge)
            lines.append(f"  {name:<12} : outer face in ({low:.4f}, {high:.4f}] m"
                         + ("" if clean else "   (contact not continuous from the start)"))
    lines.append("")
    for name, hits in profiles.items():
        lines.append(f"  {name:<12} : {''.join('#' if h else '.' for h in hits)}")

    chassis_edge, chassis_clean = edges["chassis"]
    wheels_edge, _ = edges["wheels"]
    robot_edge, _ = edges["whole robot"]
    if chassis_edge is not None:
        # Differences between groups are measured with the same plate, so the plate
        # thickness cancels: only the sweep step limits them.
        if wheels_edge is not None:
            lines.append(f"\n  wheels beyond the chassis     : {wheels_edge - chassis_edge:+.4f} m"
                         f" (+/- {SWEEP_STEP} m)")
        if robot_edge is not None:
            lines.append(f"  any geometry beyond the chassis: {robot_edge - chassis_edge:+.4f} m"
                         f" (+/- {SWEEP_STEP} m)")
            lines.append("  RobotContact covers that band through its attachments check; the "
                         "footprint must cover it too.")
            envelope_low, _ = bracket(robot_edge)
            lines.append(f"\n  footprint margin over the robot envelope: "
                         f"{configured - envelope_low:+.4f} m")
            lines.append("  negative means the configured footprint sits inside the robot, "
                         "which the PFB cannot be allowed to believe.")
    print("\n".join(lines))

    assert chassis_edge is not None, (
        f"the chassis never responded anywhere in [{lo}, {hi}] m along {axis_name}, even "
        f"though the sweep starts at {lo} m, inside its {configured:.4f} m half-extent. "
        "The probe is misplaced, so nothing here is a measurement."
    )
    assert chassis_clean, (
        "the chassis contact profile is not a clean inside-to-outside step; the probe "
        "is catching something other than what the sweep assumes"
    )
    assert robot_edge is not None and robot_edge >= chassis_edge - 1e-9, (
        "the whole-robot envelope is inside the chassis envelope, which is impossible; "
        "the collections or the probe transform are wrong"
    )
    envelope_low, _ = bracket(robot_edge)
    assert configured >= envelope_low - FOOTPRINT_TOL_M, (
        f"configs/base.yaml gives a {configured:.4f} m half-extent along {axis_name}, but "
        f"collidable geometry reaches at least {envelope_low:.4f} m. The footprint the PFB "
        "is given would sit inside the robot, so the filter would treat obstacles that are "
        "already touching the robot as clear."
    )
