"""WP0 item 3: measure the chassis footprint in the scene and report it.

Requires a running CoppeliaSim (`pytest -m sim`). The point of this test is not
to confirm what the configuration already says: it measures the real chassis
shape and prints a report that either corroborates `robot.half_length` and
`robot.half_width` or gives the exact values to put in `configs/base.yaml`.

Run it with `-s` to see the report on a pass as well as on a failure:

    pytest -m sim tests/test_chassis_geometry.py -s

The report also measures `robot.yaw_offset` directly from the chassis-to-base_ref
transform, rather than inferring it from whether the configured half-extents look
swapped -- on the 2026-09-17 run that inference stayed silent while the offset was
in fact a quarter turn out. A rectangular footprint has two equal-area alignments,
so both are printed; which one is *forward* is a question only gate G1 can answer.
"""
from __future__ import annotations

import math

import pytest

from pfbnav.sim.robot import (
    ChassisGeometry,
    SceneObjects,
    chassis_extent_in_body,
    tightest_yaw_offset,
    wrap_angle,
)

pytestmark = pytest.mark.sim

# How far the chassis box may exceed the configured envelope before it is an error.
# A footprint error of a centimetre moves the PFB zero set by a centimetre, which is
# the scale the protective fields are tuned at.
CHASSIS_TOL_M = 0.01

# Accepted residual rotation between the chassis box and the body frame. Measured
# 0.0010 rad on 2026-09-17 and accepted in docs/decision_log.md: it costs 0.2 mm of
# footprint and 0.057 deg of heading, both far below gate G1's thresholds. This guards
# against a future scene edit rotating the base by something that actually matters.
ALIGN_TOL_RAD = 0.005

# Scan step of the frame-alignment diagnostic, in rad. Also the uncertainty on the
# alignment angle it reports.
ALIGN_RESOLUTION = 1e-4


def _bbox_in_shape_frame(sim, handle: int) -> tuple[list[float], list[float]]:
    """Bounding-box corners of an object, in its own frame, in metres."""
    axes = ("x", "y", "z")
    lo = [float(sim.getObjectFloatParam(handle, getattr(sim, f"objfloatparam_objbbox_min_{a}")))
          for a in axes]
    hi = [float(sim.getObjectFloatParam(handle, getattr(sim, f"objfloatparam_objbbox_max_{a}")))
          for a in axes]
    return lo, hi


def _shapes_in(sim, root: int) -> list[int]:
    return [int(h) for h in sim.getObjectsInTree(root, sim.object_shape_type, 0)]


def _group_extent(sim, handles, base_ref: int, yaw_offset: float) -> tuple[float, float, float]:
    """Half-extents in the body frame of the union of several shapes' bounding boxes."""
    half = [0.0, 0.0, 0.0]
    for handle in handles:
        lo, hi = _bbox_in_shape_frame(sim, handle)
        one = chassis_extent_in_body(lo, hi, sim.getObjectMatrix(handle, base_ref), yaw_offset)
        half = [max(a, b) for a, b in zip(half, one)]
    return half[0], half[1], half[2]


def test_which_parts_reach_past_the_configured_footprint(cfg, sim_client):
    """Name every robot shape that sticks out past the configured envelope.

    The envelope measurement says *how far* something protrudes; it does not say
    *what*. Attributing an overhang to the wrong part -- the arm rather than the
    sensor pod, say -- sends the fix to the wrong place, so the parts are listed
    here instead of inferred. Reports rather than fails: which parts are allowed to
    protrude is a decision about the robot, not a property of the code.
    """
    geom = ChassisGeometry.from_config(cfg)
    objects = SceneObjects.from_config(cfg)
    sim = sim_client.sim
    base_ref = sim_client.object(objects.base_ref)

    rows = []
    for handle in sim_client.shapes_in_tree(sim_client.object(objects.robot_tree)):
        lo, hi = _bbox_in_shape_frame(sim, handle)
        half = chassis_extent_in_body(lo, hi, sim.getObjectMatrix(handle, base_ref),
                                      geom.yaw_offset)
        rows.append((max(half[0] - geom.half_length, half[1] - geom.half_width),
                     half, str(sim.getObjectAlias(handle, 2))))
    rows.sort(reverse=True)

    print("\n".join([
        "",
        (f"robot shapes against the configured envelope "
        f"({geom.half_length:.4f} x {geom.half_width:.4f} m)"),
        "  (bounding boxes, so these over-state rotated parts; the probe sweep is exact)",
        "",
        "   overhang   length    width   shape",
    ] + [f"  {over:+8.4f} {half[0]:8.4f} {half[1]:8.4f}   {alias}"
         for over, half, alias in rows[:12]]))

    assert rows, "the robot tree contains no shapes"


def test_wheels_against_the_chassis_footprint(cfg, sim_client):
    """Report the robot's real planar envelope against the configured footprint.

    `robot.half_length` / `half_width` describe the chassis shape, and the contact
    check watches that shape alone. The wheels hang off it on their own links, so
    if they stick out past it then a side obstacle can strike a wheel without
    touching the chassis and the contact check will not see it. This measures the
    overhang instead of arguing about it; it reports rather than fails, because
    what to do about a real overhang is a design decision, not a test outcome.
    """
    geom = ChassisGeometry.from_config(cfg)
    objects = SceneObjects.from_config(cfg)
    sim = sim_client.sim
    base_ref = sim_client.object(objects.base_ref)

    chassis = _group_extent(sim, [sim_client.object(objects.chassis)], base_ref, geom.yaw_offset)
    wheel_shapes = [h for path in objects.wheels
                    for h in _shapes_in(sim, sim_client.object(path))]
    wheels = _group_extent(sim, wheel_shapes, base_ref, geom.yaw_offset)
    everything = _group_extent(
        sim, _shapes_in(sim, sim_client.object(objects.robot_tree)), base_ref, geom.yaw_offset
    )

    print("\n".join([
        "",
        "planar envelope in the body frame (half-extents, m)",
        f"  chassis shape only  : length {chassis[0]:.4f}  width {chassis[1]:.4f}",
        f"  wheels ({len(wheel_shapes)} shapes)  : length {wheels[0]:.4f}  width {wheels[1]:.4f}",
        f"  whole robot tree    : length {everything[0]:.4f}  width {everything[1]:.4f}",
        "",
        (f"  wheel overhang beyond the chassis: length {wheels[0] - chassis[0]:+.4f}  "
        f"width {wheels[1] - chassis[1]:+.4f} m"),
        (f"  tree overhang beyond the chassis : length {everything[0] - chassis[0]:+.4f}  "
        f"width {everything[1] - chassis[1]:+.4f} m"),
        "",
        "  a positive overhang means geometry outside the shape the contact check",
        "  watches, and outside the footprint the PFB is given.",
    ]))

    assert everything[0] >= chassis[0] - 1e-9 and everything[1] >= chassis[1] - 1e-9, (
        "the whole-tree envelope is smaller than the chassis alone, which is impossible; "
        "the shape enumeration or the transform is wrong"
    )


def test_chassis_fits_inside_the_configured_footprint(cfg, sim_client):
    """The chassis box against the configured envelope, plus the frame alignment.

    `robot.half_length` / `half_width` describe the whole robot envelope, chassis
    plus wheels, so the chassis alone must fit inside them with room to spare
    laterally. `tests/test_footprint_probe.py` is the authority on the envelope
    itself; this checks the chassis and the body frame.
    """
    geom = ChassisGeometry.from_config(cfg)
    objects = SceneObjects.from_config(cfg)

    chassis = sim_client.object(objects.chassis)
    base_ref = sim_client.object(objects.base_ref)
    lo, hi = _bbox_in_shape_frame(sim_client.sim, chassis)
    matrix = sim_client.sim.getObjectMatrix(chassis, base_ref)

    measured = chassis_extent_in_body(lo, hi, matrix, geom.yaw_offset)
    quarter_turn = chassis_extent_in_body(lo, hi, matrix, geom.yaw_offset + math.pi / 2)

    # Measured independently of the configured offset, so the report says what the
    # frame actually is instead of only whether it looks close to what was expected.
    aligned_at, aligned_half = tightest_yaw_offset(lo, hi, matrix, resolution=ALIGN_RESOLUTION)
    other_half = chassis_extent_in_body(lo, hi, matrix, aligned_at + math.pi / 2)
    misalignment = wrap_angle(geom.yaw_offset - aligned_at)

    d_length = measured[0] - geom.half_length
    d_width = measured[1] - geom.half_width

    report = "\n".join([
        "",
        "chassis geometry measured from the scene",
        f"  scene              : {cfg['sim']['scene']}",
        f"  chassis object     : {objects.chassis}",
        f"  pose reference     : {objects.base_ref}",
        (f"  bbox in shape frame: min={[round(v, 4) for v in lo]} "
        f"max={[round(v, 4) for v in hi]} m"),
        f"  yaw_offset applied : {geom.yaw_offset:+.4f} rad",
        "",
        "  quantity          envelope        chassis    difference",
        f"  half_length       {geom.half_length:9.4f}   {measured[0]:9.4f}   {d_length:+9.4f} m",
        f"  half_width        {geom.half_width:9.4f}   {measured[1]:9.4f}   {d_width:+9.4f} m",
        f"  half_height               -   {measured[2]:9.4f}           - m",
        "",
        (f"  with yaw_offset + pi/2: half_length={quarter_turn[0]:.4f} "
        f"half_width={quarter_turn[1]:.4f} m"),
        "",
        "  frame alignment measured from the transform, independent of the config",
        (f"  (+/- {ALIGN_RESOLUTION:g} rad; the two alignments have equal area, so both "
        "are listed):"),
        (f"    yaw_offset {aligned_at:+.4f} rad -> half_length={aligned_half[0]:.4f} "
        f"half_width={aligned_half[1]:.4f} m"),
        (f"    yaw_offset {aligned_at + math.pi / 2:+.4f} rad -> "
        f"half_length={other_half[0]:.4f} half_width={other_half[1]:.4f} m"),
        f"    configured yaw_offset is {misalignment:+.4f} rad from the first of these",
        "",
        "  the chassis must not exceed the envelope; a negative difference is the",
        "  wheels standing proud of it, which is expected laterally.",
    ])
    print(report)

    off_axis = min(abs(wrap_angle(misalignment - k * math.pi / 2)) for k in range(-2, 3))
    assert off_axis <= ALIGN_TOL_RAD, (
        f"{report}\n\nrobot.yaw_offset is {off_axis:.4f} rad away from any alignment of "
        f"the chassis box, beyond the {ALIGN_TOL_RAD} rad accepted residual. The body "
        "frame is skewed against the chassis, so every pose reading and every body-frame "
        "velocity carries that bias."
    )
    # Signed, not absolute: the chassis being *smaller* than the envelope is the normal
    # case, since the wheels and the sensor pods are what set the envelope.
    assert d_length <= CHASSIS_TOL_M and d_width <= CHASSIS_TOL_M, (
        f"{report}\n\nthe chassis box reaches past the envelope in configs/base.yaml, so "
        "the footprint the PFB is given sits inside the robot."
    )


def test_chassis_extends_beyond_the_v1_collision_threshold(cfg, sim_client):
    """The v1 LiDAR collision threshold was inside the chassis; record that it is.

    docs/decision_log.md justifies ground-truth contact by noting that v1's
    0.20 m range threshold sits below the chassis half-length. That claim is a
    statement about this scene, so it is measured rather than asserted in prose.
    """
    geom = ChassisGeometry.from_config(cfg)
    objects = SceneObjects.from_config(cfg)
    chassis = sim_client.object(objects.chassis)
    base_ref = sim_client.object(objects.base_ref)
    lo, hi = _bbox_in_shape_frame(sim_client.sim, chassis)

    measured = chassis_extent_in_body(lo, hi, sim_client.sim.getObjectMatrix(chassis, base_ref),
                                      geom.yaw_offset)
    v1_threshold = 0.20
    print(f"\nv1 collision threshold {v1_threshold} m vs measured half_length "
          f"{measured[0]:.4f} m, half_width {measured[1]:.4f} m")
    assert measured[0] > v1_threshold, (
        "the measured chassis half-length is inside the v1 0.20 m LiDAR threshold; "
        "re-check the chassis object path before trusting any v1 comparison"
    )
