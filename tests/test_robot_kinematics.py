"""Known-answer, round-trip, and finite-step tests for the mecanum base.

No simulator: every function under test here is pure.
"""
import math

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.sim.client import SimConnection, host_from_endpoint, port_for_instance
from pfbnav.sim.robot import (
    WHEEL_ORDER,
    ActuationLimits,
    ChassisGeometry,
    SceneObjects,
    bbox_corners_in_body,
    body_from_wheels,
    body_point_in_ref,
    chassis_extent_in_body,
    clamp_velocity,
    clamp_velocity_with_status,
    limit_acceleration,
    ref_point_in_body,
    tightest_yaw_offset,
    wheels_from_body,
    world_to_body,
    wrap_angle,
)

GEOM = ChassisGeometry(
    half_length=0.29, half_width=0.19, wheel_radius=0.05,
    half_wheelbase=0.228, half_track=0.158,
)
GEOM_FLIPPED = ChassisGeometry(
    half_length=0.29, half_width=0.19, wheel_radius=0.05,
    half_wheelbase=0.228, half_track=0.158, wheel_sign=(-1.0, -1.0, -1.0, -1.0),
)
LIMITS = ActuationLimits(v_max=0.5, vy_max=0.3, w_max=1.5, a_max=1.0, alpha_max=3.0,
                         a_decel_max=1.0)
# A limiter whose deceleration bound differs from its acceleration bound, so the two
# branches can be told apart.
ASYMMETRIC = ActuationLimits(v_max=0.5, vy_max=0.3, w_max=1.5, a_max=2.0, alpha_max=3.0,
                             a_decel_max=0.5)
DT = 0.05


# ----------------------------------------------------------------------
# Inverse kinematics: one known answer per commanded degree of freedom
# ----------------------------------------------------------------------
def test_pure_forward_spins_all_wheels_equally():
    # vx / r = 1.0 / 0.05 = 20 rad/s on every wheel.
    assert wheels_from_body([1.0, 0.0, 0.0], GEOM) == pytest.approx([20.0] * 4)


def test_pure_left_strafe_is_the_x_pattern():
    # +y (left) strafe: fl and rr roll backward, fr and rl roll forward.
    assert wheels_from_body([0.0, 1.0, 0.0], GEOM) == pytest.approx([-20.0, 20.0, 20.0, -20.0])


def test_pure_ccw_yaw_drives_the_left_side_backward():
    # (L + W) / r = 0.386 / 0.05 = 7.72 rad/s; left wheels back, right wheels forward.
    expected = [-7.72, 7.72, -7.72, 7.72]
    assert wheels_from_body([0.0, 0.0, 1.0], GEOM) == pytest.approx(expected)


def test_zero_command_stops_every_wheel():
    assert wheels_from_body([0.0, 0.0, 0.0], GEOM) == pytest.approx([0.0] * 4)


def test_wheel_sign_negates_every_wheel_speed():
    cmd = [0.3, -0.2, 0.4]
    assert wheels_from_body(cmd, GEOM_FLIPPED) == pytest.approx(-wheels_from_body(cmd, GEOM))


def test_wheel_order_is_the_documented_one():
    assert WHEEL_ORDER == ("fl", "fr", "rl", "rr")


def test_wheel_sign_must_be_plus_or_minus_one():
    with pytest.raises(ValueError):
        ChassisGeometry(0.29, 0.19, 0.05, 0.228, 0.158, wheel_sign=(1.0, 1.0, 1.0, 0.5))


# ----------------------------------------------------------------------
# Forward kinematics: exact left inverse of the mixing
# ----------------------------------------------------------------------
@pytest.mark.parametrize("geom", [GEOM, GEOM_FLIPPED])
def test_forward_kinematics_inverts_the_mixing(geom):
    rng = np.random.default_rng(0)
    for _ in range(200):
        cmd = rng.uniform(-1.0, 1.0, size=3)
        assert body_from_wheels(wheels_from_body(cmd, geom), geom) == pytest.approx(cmd)


def test_forward_kinematics_known_answer():
    # All wheels at 20 rad/s can only mean 1 m/s forward.
    assert body_from_wheels([20.0, 20.0, 20.0, 20.0], GEOM) == pytest.approx([1.0, 0.0, 0.0])


def test_forward_kinematics_rejects_wrong_length():
    with pytest.raises(ValueError):
        body_from_wheels([1.0, 2.0, 3.0], GEOM)


# ----------------------------------------------------------------------
# Frames
# ----------------------------------------------------------------------
@pytest.mark.parametrize("theta, expected", [
    (0.0, 0.0), (math.pi, math.pi), (-math.pi, math.pi),
    (3 * math.pi, math.pi), (2 * math.pi + 0.5, 0.5), (-2 * math.pi - 0.5, -0.5),
])
def test_wrap_angle_known_answers(theta, expected):
    assert wrap_angle(theta) == pytest.approx(expected)


def test_world_to_body_known_answer():
    # Robot heading +90 deg: a world +x vector points along body -y (right).
    assert world_to_body([1.0, 0.0], math.pi / 2) == pytest.approx([0.0, -1.0], abs=1e-12)


def test_world_to_body_preserves_length():
    rng = np.random.default_rng(1)
    for _ in range(50):
        vec, yaw = rng.uniform(-3, 3, size=2), rng.uniform(-math.pi, math.pi)
        assert np.linalg.norm(world_to_body(vec, yaw)) == pytest.approx(np.linalg.norm(vec))


# ----------------------------------------------------------------------
# Velocity box
# ----------------------------------------------------------------------
def test_clamp_scales_the_translational_pair_by_one_factor():
    # (9, -9) leaves the box; the tighter axis is vy (0.3/9), so both shrink by 1/30.
    assert clamp_velocity([9.0, -9.0, 9.0], LIMITS) == pytest.approx([0.3, -0.3, 1.5])


def test_clamp_preserves_the_direction_of_a_diagonal():
    # G1b's failing case: 0.50 m/s at 45 deg is (0.3536, 0.3536). Per-axis clipping gave
    # (0.3536, 0.3) and bent it 4.7 deg; one factor keeps it at 45 deg, at 0.3*sqrt(2).
    v = 0.5 / np.sqrt(2.0)
    result = clamp_velocity_with_status([v, v, 0.0], LIMITS)
    assert result.applied == pytest.approx([0.3, 0.3, 0.0])
    assert np.arctan2(result.applied[1], result.applied[0]) == pytest.approx(np.pi / 4)
    assert result.translation_scale == pytest.approx(0.3 / v)
    assert result.translation_bound and not result.omega_clipped


def test_clamp_binding_on_the_forward_axis():
    result = clamp_velocity_with_status([1.0, 0.1, 0.0], LIMITS)
    assert result.applied == pytest.approx([0.5, 0.05, 0.0])
    assert result.translation_scale == pytest.approx(0.5)


def test_clamp_clips_omega_separately_and_reports_it():
    result = clamp_velocity_with_status([0.1, 0.1, -4.0], LIMITS)
    assert result.applied == pytest.approx([0.1, 0.1, -1.5])
    assert result.omega_clipped and not result.translation_bound
    assert result.translation_scale == 1.0


def test_clamp_output_is_always_inside_the_box_and_parallel_to_the_input():
    rng = np.random.default_rng(11)
    for _ in range(2000):
        cmd = rng.uniform(-3.0, 3.0, size=3)
        out = clamp_velocity(cmd, LIMITS)
        assert abs(out[0]) <= LIMITS.v_max and abs(out[1]) <= LIMITS.vy_max
        assert abs(out[2]) <= LIMITS.w_max
        # Parallel and same sense: zero cross product, non-negative dot product.
        assert cmd[0] * out[1] - cmd[1] * out[0] == pytest.approx(0.0, abs=1e-12)
        assert cmd[0] * out[0] + cmd[1] * out[1] >= 0.0


def test_clamp_leaves_an_admissible_command_untouched():
    cmd = [0.2, -0.1, 0.7]
    assert clamp_velocity(cmd, LIMITS) == pytest.approx(cmd)


# ----------------------------------------------------------------------
# Acceleration limiter
# ----------------------------------------------------------------------
def test_single_step_is_a_max_times_dt():
    applied = limit_acceleration([0.5, 0.0, 0.0], [0.0, 0.0, 0.0], DT, LIMITS)
    assert applied == pytest.approx([LIMITS.a_max * DT, 0.0, 0.0])


def test_limiter_preserves_the_direction_of_the_velocity_change():
    # Requested change (0.3, 0.4) has norm 0.5; scaled to 0.05 it stays 3:4.
    applied = limit_acceleration([0.3, 0.4, 0.0], [0.0, 0.0, 0.0], DT, LIMITS)
    assert applied[:2] == pytest.approx([0.03, 0.04])
    assert np.linalg.norm(applied[:2]) == pytest.approx(LIMITS.a_max * DT)


def test_limiter_is_isotropic_over_direction():
    rng = np.random.default_rng(2)
    for _ in range(50):
        angle = rng.uniform(-math.pi, math.pi)
        cmd = [10.0 * math.cos(angle), 10.0 * math.sin(angle), 0.0]
        applied = limit_acceleration(cmd, [0.0, 0.0, 0.0], DT, LIMITS)
        assert np.linalg.norm(applied[:2]) == pytest.approx(LIMITS.a_max * DT)


def test_limiter_passes_through_a_small_change():
    applied = limit_acceleration([0.21, 0.0, 0.05], [0.20, 0.0, 0.0], DT, LIMITS)
    assert applied == pytest.approx([0.21, 0.0, 0.05])


def test_limiter_bounds_deceleration_the_same_way():
    applied = limit_acceleration([0.0, 0.0, 0.0], [0.5, 0.0, 0.0], DT, LIMITS)
    assert applied == pytest.approx([0.45, 0.0, 0.0])


def test_yaw_rate_is_limited_by_alpha_max():
    applied = limit_acceleration([0.0, 0.0, 1.5], [0.0, 0.0, 0.0], DT, LIMITS)
    assert applied[2] == pytest.approx(LIMITS.alpha_max * DT)


def test_yaw_limit_is_independent_of_the_linear_limit():
    # A saturated linear step must not eat into the yaw step, or vice versa.
    applied = limit_acceleration([1.0, 0.0, 1.5], [0.0, 0.0, 0.0], DT, LIMITS)
    assert applied == pytest.approx([LIMITS.a_max * DT, 0.0, LIMITS.alpha_max * DT])


def test_ramp_reaches_the_target_in_the_predicted_number_of_steps():
    target = [LIMITS.v_max, 0.0, 0.0]
    steps_needed = math.ceil(LIMITS.v_max / (LIMITS.a_max * DT))
    applied = np.zeros(3)
    for step in range(steps_needed):
        applied = limit_acceleration(target, applied, DT, LIMITS)
        reached = applied[0] == pytest.approx(LIMITS.v_max)
        assert reached == (step == steps_needed - 1), f"target reached at step {step}"


def test_clamp_then_limit_stays_inside_the_velocity_box():
    rng = np.random.default_rng(3)
    applied = np.zeros(3)
    for _ in range(500):
        applied = limit_acceleration(
            clamp_velocity(rng.uniform(-5.0, 5.0, size=3), LIMITS), applied, DT, LIMITS
        )
        assert abs(applied[0]) <= LIMITS.v_max + 1e-12
        assert abs(applied[1]) <= LIMITS.vy_max + 1e-12
        assert abs(applied[2]) <= LIMITS.w_max + 1e-12


def test_limiter_bounds_every_step_of_a_random_command_stream():
    rng = np.random.default_rng(4)
    previous = np.zeros(3)
    for _ in range(500):
        applied = limit_acceleration(rng.uniform(-5.0, 5.0, size=3), previous, DT, LIMITS)
        assert np.linalg.norm(applied[:2] - previous[:2]) <= LIMITS.a_max * DT + 1e-12
        assert abs(applied[2] - previous[2]) <= LIMITS.alpha_max * DT + 1e-12
        previous = applied


def test_limiter_rejects_a_non_positive_dt():
    with pytest.raises(ValueError):
        limit_acceleration([0.1, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0, LIMITS)


# ----------------------------------------------------------------------
# Chassis bounding box -> body-frame half-extents
# ----------------------------------------------------------------------
IDENTITY = [1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0]


def _yaw_matrix(theta, translation=(0.0, 0.0, 0.0)):
    c, s = math.cos(theta), math.sin(theta)
    return [c, -s, 0.0, translation[0],
            s, c, 0.0, translation[1],
            0.0, 0.0, 1.0, translation[2]]


def test_extent_of_a_centred_box_is_its_half_size():
    half = chassis_extent_in_body([-0.29, -0.19, 0.0], [0.29, 0.19, 0.1], IDENTITY, 0.0)
    assert half == pytest.approx((0.29, 0.19, 0.1))


def test_a_quarter_turn_in_the_shape_frame_swaps_length_and_width():
    half = chassis_extent_in_body(
        [-0.29, -0.19, 0.0], [0.29, 0.19, 0.1], _yaw_matrix(math.pi / 2), 0.0
    )
    assert half == pytest.approx((0.19, 0.29, 0.1))


def test_a_yaw_offset_of_ninety_degrees_swaps_them_back():
    half = chassis_extent_in_body(
        [-0.29, -0.19, 0.0], [0.29, 0.19, 0.1], _yaw_matrix(math.pi / 2), math.pi / 2
    )
    assert half == pytest.approx((0.29, 0.19, 0.1))


@pytest.mark.parametrize("theta", np.linspace(-math.pi, math.pi, 9))
def test_matching_yaw_offset_cancels_any_shape_frame_rotation(theta):
    half = chassis_extent_in_body(
        [-0.29, -0.19, 0.0], [0.29, 0.19, 0.1], _yaw_matrix(theta), theta
    )
    assert half == pytest.approx((0.29, 0.19, 0.1))


def test_an_off_centre_box_reports_the_larger_half_extent():
    # The half-extents must cover the whole chassis around the body origin, so an
    # off-centre box is reported by its farthest corner, not by its size / 2.
    half = chassis_extent_in_body([0.0, 0.0, 0.0], [0.58, 0.38, 0.1], IDENTITY, 0.0)
    assert half == pytest.approx((0.58, 0.38, 0.1))


def test_a_translation_shifts_the_extent():
    half = chassis_extent_in_body(
        [-0.29, -0.19, 0.0], [0.29, 0.19, 0.1], _yaw_matrix(0.0, (0.1, 0.0, 0.0)), 0.0
    )
    assert half == pytest.approx((0.39, 0.19, 0.1))


# The chassis shape frame measured in scenes/warehouse_v2.ttt on 2026-09-17: the
# /youBot frame is X-up, so shape y -> body x, shape z -> body y, shape x -> body z,
# with the chassis sitting 0.0958 m above the base_ref origin.
YOUBOT_SHAPE_TO_REF = [0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       1.0, 0.0, 0.0, 0.0958]
YOUBOT_BBOX_LO = [-0.081, -0.1785, -0.2851]
YOUBOT_BBOX_HI = [0.081, 0.1785, 0.2851]


@pytest.mark.parametrize("point, yaw_offset, expected", [
    ([1.0, 0.0, 0.0], 0.0, [1.0, 0.0, 0.0]),
    ([1.0, 0.0, 0.0], math.pi / 2, [0.0, 1.0, 0.0]),
    ([0.0, 1.0, 0.0], math.pi / 2, [-1.0, 0.0, 0.0]),
    ([0.0, 0.0, 0.3], math.pi / 2, [0.0, 0.0, 0.3]),
    ([1.0, 2.0, 3.0], 0.0, [1.0, 2.0, 3.0]),
])
def test_body_point_in_ref_known_answers(point, yaw_offset, expected):
    assert body_point_in_ref(point, yaw_offset) == pytest.approx(expected, abs=1e-12)


def test_body_point_in_ref_inverts_the_extent_rotation():
    # The chassis long axis is body +x; at yaw_offset = pi/2 it must land on ref +y,
    # which is where the measured 0.2851 m half-extent actually lies in the scene.
    assert body_point_in_ref([0.2851, 0.0, 0.0], math.pi / 2) == pytest.approx(
        [0.0, 0.2851, 0.0], abs=1e-12
    )


@pytest.mark.parametrize("point, yaw_offset, expected", [
    ([1.0, 0.0, 0.0], math.pi / 2, [0.0, -1.0, 0.0]),
    ([0.0, 1.0, 0.0], math.pi / 2, [1.0, 0.0, 0.0]),
    ([1.0, 2.0, 3.0], 0.0, [1.0, 2.0, 3.0]),
])
def test_ref_point_in_body_known_answers(point, yaw_offset, expected):
    assert ref_point_in_body(point, yaw_offset) == pytest.approx(expected, abs=1e-12)


def test_ref_point_in_body_matches_the_measured_wheel_layout():
    """The scene's front-left wheel sits at base_ref (-0.1585, +0.228); in the body
    frame that must read (+0.228, +0.1585): forward and to the left."""
    assert ref_point_in_body([-0.1585, 0.228, 0.0504], math.pi / 2) == pytest.approx(
        [0.228, 0.1585, 0.0504], abs=1e-9
    )


@pytest.mark.parametrize("yaw_offset", [0.0, 0.3, math.pi / 2, -1.2, math.pi])
def test_the_two_frame_maps_are_inverses(yaw_offset):
    rng = np.random.default_rng(11)
    for _ in range(20):
        point = rng.uniform(-2.0, 2.0, size=3)
        assert ref_point_in_body(body_point_in_ref(point, yaw_offset), yaw_offset) == (
            pytest.approx(point, abs=1e-12)
        )


def test_body_point_in_ref_rejects_a_wrong_sized_point():
    with pytest.raises(ValueError):
        body_point_in_ref([1.0, 2.0], 0.0)


def test_tightest_offset_is_zero_for_an_already_aligned_box():
    offset, half = tightest_yaw_offset([-0.29, -0.19, 0.0], [0.29, 0.19, 0.1], IDENTITY)
    assert offset == pytest.approx(0.0, abs=1e-4)
    assert half == pytest.approx((0.29, 0.19, 0.1))


def test_tightest_offset_recovers_a_known_rotation():
    truth = 0.3
    offset, half = tightest_yaw_offset(
        [-0.29, -0.19, 0.0], [0.29, 0.19, 0.1], _yaw_matrix(truth), resolution=1e-4
    )
    assert abs(offset - truth) <= 1e-4
    assert half[:2] == pytest.approx((0.29, 0.19), abs=1e-4)


def test_tightest_offset_returns_one_of_the_two_equal_area_alignments():
    # A box rotated by 2.0 rad is aligned at 2.0 and, with length and width swapped,
    # at 2.0 - pi/2 = 0.4292. Both have the same area, so either is a valid answer.
    truth = 2.0
    offset, half = tightest_yaw_offset(
        [-0.29, -0.19, 0.0], [0.29, 0.19, 0.1], _yaw_matrix(truth), resolution=1e-4
    )
    assert min(abs(offset - truth), abs(offset - (truth - math.pi / 2))) <= 1e-4
    assert sorted(half[:2]) == pytest.approx([0.19, 0.29], abs=1e-4)


def test_the_second_alignment_swaps_length_and_width():
    # The property every caller relies on when it reports both candidates.
    lo, hi, matrix = [-0.29, -0.19, 0.0], [0.29, 0.19, 0.1], _yaw_matrix(2.0)
    offset, half = tightest_yaw_offset(lo, hi, matrix, resolution=1e-4)
    other = chassis_extent_in_body(lo, hi, matrix, offset + math.pi / 2)
    assert other[:2] == pytest.approx((half[1], half[0]), abs=1e-4)
    assert other[2] == pytest.approx(half[2])


def test_tightest_offset_on_the_measured_youbot_frame():
    offset, half = tightest_yaw_offset(YOUBOT_BBOX_LO, YOUBOT_BBOX_HI, YOUBOT_SHAPE_TO_REF)
    # A rectangle is aligned at the offset and at offset + pi/2; both are minima, so the
    # diagnostic pins the misalignment to a quarter turn and G1 decides which is forward.
    assert (offset % (math.pi / 2)) == pytest.approx(0.0, abs=1e-4)
    assert sorted(half[:2]) == pytest.approx([0.1785, 0.2851], abs=1e-4)
    assert half[2] == pytest.approx(0.1768, abs=1e-4), "chassis top above base_ref"


def test_the_measured_youbot_frame_needs_a_quarter_turn_for_forward():
    # Long axis along body +x, which is what robot.half_length must describe.
    half = chassis_extent_in_body(
        YOUBOT_BBOX_LO, YOUBOT_BBOX_HI, YOUBOT_SHAPE_TO_REF, math.pi / 2
    )
    assert half == pytest.approx((0.2851, 0.1785, 0.1768), abs=1e-4)
    assert half[0] > half[1]


def test_tightest_offset_rejects_a_non_positive_resolution():
    with pytest.raises(ValueError):
        tightest_yaw_offset([-0.1] * 3, [0.1] * 3, IDENTITY, resolution=0.0)


def test_extent_rejects_an_inverted_box():
    with pytest.raises(ValueError):
        chassis_extent_in_body([0.1, 0.0, 0.0], [0.0, 0.1, 0.1], IDENTITY, 0.0)


def test_extent_rejects_a_wrong_sized_matrix():
    with pytest.raises(ValueError):
        chassis_extent_in_body([-0.1] * 3, [0.1] * 3, IDENTITY[:9], 0.0)


# ----------------------------------------------------------------------
# Endpoint parsing and parallel-instance ports
# ----------------------------------------------------------------------
def test_host_is_taken_from_the_endpoint():
    assert host_from_endpoint("tcp://127.0.0.1") == "127.0.0.1"


@pytest.mark.parametrize("endpoint", [
    "127.0.0.1",            # no transport
    "udp://127.0.0.1",      # unsupported transport
    "tcp://127.0.0.1:23000",  # port belongs in sim.port
    "tcp://",               # empty host
])
def test_a_malformed_endpoint_is_rejected(endpoint):
    with pytest.raises(ValueError):
        host_from_endpoint(endpoint)


def test_parallel_instances_offset_the_port():
    assert port_for_instance(23000, 0) == 23000
    assert port_for_instance(23000, 3) == 23003
    assert [port_for_instance(23000, i, 2) for i in range(4)] == [23000, 23002, 23004, 23006]
    with pytest.raises(ValueError):
        port_for_instance(23000, -1)
    with pytest.raises(ValueError):
        port_for_instance(23000, 1, 0)


# ----------------------------------------------------------------------
# The shipped configuration must build every WP0 dataclass
# ----------------------------------------------------------------------
def test_base_config_builds_the_robot_dataclasses():
    cfg = load_config("configs/base.yaml")
    geom = ChassisGeometry.from_config(cfg)
    limits = ActuationLimits.from_config(cfg)
    objects = SceneObjects.from_config(cfg)
    connection = SimConnection.from_config(cfg, instance_id=2)

    assert geom.lw == pytest.approx(geom.half_wheelbase + geom.half_track)
    # The box is NOT assumed square or forward-dominant. It is square again since 2026-10-19
    # (v_max = vy_max = 0.50, G1b under the A20 limits); PRE-A20 it was v_max 0.40 (G1b failed
    # 0.50 on translate_while_rotating) and vy_max 0.50. Nothing in the kinematics needs
    # an ordering -- the clamp scales each axis against its own ceiling -- and the real
    # constraint is the wheel budget, checked in tests/test_footprint_config.py.
    budget = cfg["actuation"]["wheel_budget"]
    filter_limit = budget["filter_fraction"] * budget["v_rim"]
    assert 0.0 < limits.v_max <= filter_limit
    assert 0.0 < limits.vy_max <= filter_limit
    assert len(objects.wheels) == len(WHEEL_ORDER)
    assert connection.port == cfg["sim"]["port"] + 2 * cfg["sim"]["port_stride"]
    assert connection.dt == pytest.approx(cfg["sim"]["dt"])


# ----------------------------------------------------------------------
# Agreement with the canonical youBot kinematics in Modern Robotics
# ----------------------------------------------------------------------
def _modern_robotics_wheels(vx, vy, wz, r, l, w):
    """Lynch & Park, Modern Robotics eq. 13.10, the youBot's H(0) matrix.

    `u = (1/r) H(0) (omega_bz, v_bx, v_by)`, with wheels numbered counter-clockwise
    1=front-left, 2=front-right, 3=rear-right, 4=rear-left. Reordered on return to this
    project's [fl, fr, rl, rr].

    Checking against this rather than against v1 matters: v1 is the same lineage as our
    code, so agreeing with it could only ever confirm that a shared assumption was applied
    consistently, not that it was right.
    """
    H = np.array([[-l - w, 1.0, -1.0],
                  [l + w, 1.0, 1.0],
                  [l + w, 1.0, -1.0],
                  [-l - w, 1.0, 1.0]])
    fl, fr, rr, rl = (H @ np.array([wz, vx, vy])) / r
    return np.array([fl, fr, rl, rr])


@pytest.mark.parametrize("vx, vy, wz", [
    (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
    (0.5, 0.3, 0.0), (0.2, -0.4, 0.7), (-0.6, 0.1, -0.3),
])
def test_mixing_matches_modern_robotics_known_answers(vx, vy, wz):
    expected = _modern_robotics_wheels(vx, vy, wz, GEOM.wheel_radius,
                                       GEOM.half_wheelbase, GEOM.half_track)
    assert wheels_from_body([vx, vy, wz], GEOM) == pytest.approx(expected, abs=1e-12)


def test_mixing_matches_modern_robotics_over_random_twists():
    rng = np.random.default_rng(7)
    for _ in range(500):
        vx, vy, wz = rng.uniform(-1.0, 1.0, 3)
        expected = _modern_robotics_wheels(vx, vy, wz, GEOM.wheel_radius,
                                           GEOM.half_wheelbase, GEOM.half_track)
        assert wheels_from_body([vx, vy, wz], GEOM) == pytest.approx(expected, abs=1e-12)


# ----------------------------------------------------------------------
# The deceleration branch of the limiter (D9)
# ----------------------------------------------------------------------
def test_slowing_down_uses_the_deceleration_limit():
    # From 0.4 m/s towards rest: bounded by a_decel_max (0.5), not a_max (2.0).
    applied = limit_acceleration([0.0, 0.0, 0.0], [0.4, 0.0, 0.0], DT, ASYMMETRIC)
    assert applied[0] == pytest.approx(0.4 - ASYMMETRIC.a_decel_max * DT)


def test_speeding_up_uses_the_acceleration_limit():
    applied = limit_acceleration([0.5, 0.0, 0.0], [0.1, 0.0, 0.0], DT, ASYMMETRIC)
    assert applied[0] == pytest.approx(0.1 + ASYMMETRIC.a_max * DT)


def test_a_turn_at_constant_speed_is_not_treated_as_slowing():
    """Rotating the velocity without shrinking it is not a deceleration."""
    speed = 0.3
    prev = [speed, 0.0, 0.0]
    cmd = [speed * math.cos(0.4), speed * math.sin(0.4), 0.0]
    applied = limit_acceleration(cmd, prev, DT, ASYMMETRIC)
    assert np.linalg.norm(np.asarray(applied[:2]) - np.asarray(prev[:2])) == pytest.approx(
        ASYMMETRIC.a_max * DT
    )


def test_an_emergency_stop_cannot_beat_the_configured_deceleration():
    """The guarantee D9 buys: no command reaches rest faster than a_decel_max allows."""
    v0, applied = 0.5, np.array([0.5, 0.0, 0.0])
    steps = 0
    while float(np.hypot(applied[0], applied[1])) > 1e-9:
        applied = limit_acceleration([0.0, 0.0, 0.0], applied, DT, LIMITS)
        steps += 1
        assert steps < 1000
    # Reaching rest takes at least v0 / (a_decel_max * dt) steps, whatever was commanded.
    assert steps >= math.ceil(v0 / (LIMITS.a_decel_max * DT))


def test_bypassing_the_limiter_is_the_only_way_to_break_the_guarantee():
    """`rate_limit=False` exists for calibration and deliberately voids D9's guarantee."""
    stopped = limit_acceleration([0.0, 0.0, 0.0], [0.5, 0.0, 0.0], DT, LIMITS)
    assert stopped[0] > 0.0, "through the limiter, one step cannot reach rest from v_max"


def test_limits_reject_a_non_positive_deceleration_bound():
    with pytest.raises(ValueError, match="a_decel_max"):
        ActuationLimits(v_max=0.5, vy_max=0.3, w_max=1.5, a_max=1.0, alpha_max=3.0,
                        a_decel_max=0.0)


# ----------------------------------------------------------------------
# bbox_corners_in_body: the polygon footprint's building block
# ----------------------------------------------------------------------
def test_corners_of_an_axis_aligned_box_are_the_eight_combinations():
    corners = bbox_corners_in_body([-0.29, -0.19, 0.0], [0.29, 0.19, 0.1], IDENTITY, 0.0)
    assert corners.shape == (8, 3)
    expected = {(x, y, z) for x in (-0.29, 0.29) for y in (-0.19, 0.19) for z in (0.0, 0.1)}
    assert {tuple(np.round(c, 9)) for c in corners} == {
        tuple(np.round(e, 9)) for e in expected
    }


@pytest.mark.parametrize("yaw_offset", [0.0, 0.3, math.pi / 2, -1.1])
def test_the_extent_is_the_largest_absolute_corner(yaw_offset):
    """The two functions must never disagree: one is a reduction of the other."""
    lo, hi = [-0.081, -0.1785, -0.2851], [0.081, 0.1785, 0.2851]
    corners = bbox_corners_in_body(lo, hi, YOUBOT_SHAPE_TO_REF, yaw_offset)
    extent = chassis_extent_in_body(lo, hi, YOUBOT_SHAPE_TO_REF, yaw_offset)
    assert np.max(np.abs(corners), axis=0) == pytest.approx(extent)
