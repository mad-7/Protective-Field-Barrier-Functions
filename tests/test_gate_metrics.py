"""Known-answer tests for the WP0 gate metrics, on synthetic data (hard rule 6).

No simulator. Every case here has an answer that can be worked out by hand, so a
metric that drifts shows up as a failing arithmetic check rather than as a plausible
number in a gate summary.
"""
import math

import numpy as np
import pytest

from pfbnav.eval.gates import (
    braking_distance,
    contact_agreement,
    count_contact_steps,
    cross_track_error,
    detection_lag_steps,
    diagnose_direction,
    drift_band_holds,
    effective_deceleration,
    heading_drift_deg,
    mean_deceleration,
    path_length,
    relative_speed_error,
    rms,
    signed_heading_drift_deg,
    steady_speed,
    steady_state_start,
    stop_index,
    unwrap_angles,
)


# ----------------------------------------------------------------------
# unwrap_angles
# ----------------------------------------------------------------------
def test_unwrap_leaves_a_continuous_series_alone():
    assert unwrap_angles([0.0, 0.1, 0.2]) == pytest.approx([0.0, 0.1, 0.2])


def test_unwrap_removes_the_jump_at_the_wrap():
    # +3.0 -> -3.0 is a -6.0 step, which is physically +0.283 rad the other way.
    assert unwrap_angles([0.0, 3.0, -3.0]) == pytest.approx(
        [0.0, 3.0, 3.0 + (2 * math.pi - 6.0)]
    )


def test_unwrap_rejects_an_empty_series():
    with pytest.raises(ValueError):
        unwrap_angles([])


# ----------------------------------------------------------------------
# path_length
# ----------------------------------------------------------------------
def test_path_length_of_a_straight_line():
    assert path_length([[0.0, 0.0], [3.0, 4.0]]) == pytest.approx(5.0)


def test_path_length_sums_the_segments_not_the_displacement():
    # Out 1 m and back: 2 m travelled, 0 m displaced.
    assert path_length([[0.0, 0.0], [1.0, 0.0], [0.0, 0.0]]) == pytest.approx(2.0)


def test_path_length_of_a_single_point_is_zero():
    assert path_length([[1.0, 2.0]]) == 0.0


def test_path_length_rejects_the_wrong_shape():
    with pytest.raises(ValueError):
        path_length([0.0, 1.0, 2.0])


# ----------------------------------------------------------------------
# steady_state_start
# ----------------------------------------------------------------------
def test_steady_state_starts_after_the_ramp_plus_settle():
    applied = [0.1, 0.2, 0.3, 0.4, 0.5, 0.5, 0.5, 0.5]
    # First within 2% of 0.5 at index 4; +2 settle steps -> 6.
    assert steady_state_start(applied, 0.5, tol=0.02, settle_steps=2) == 6


def test_steady_state_with_no_settle_is_the_first_arrival():
    assert steady_state_start([0.25, 0.5, 0.5], 0.5, tol=0.02, settle_steps=0) == 1


def test_a_profile_that_never_reaches_the_target_has_no_steady_state():
    with pytest.raises(ValueError, match="never came within"):
        steady_state_start([0.1, 0.2, 0.3], 0.5, tol=0.02, settle_steps=0)


def test_a_profile_too_short_for_the_settle_window_is_an_error():
    with pytest.raises(ValueError, match="only"):
        steady_state_start([0.5, 0.5], 0.5, tol=0.02, settle_steps=5)


# ----------------------------------------------------------------------
# relative_speed_error
# ----------------------------------------------------------------------
def test_relative_speed_error_known_answer():
    measured = [[0.45, 0.0], [0.45, 0.0]]
    commanded = [[0.50, 0.0], [0.50, 0.0]]
    assert relative_speed_error(measured, commanded) == pytest.approx(0.10)


def test_relative_speed_error_is_zero_for_perfect_tracking():
    cmd = [[0.3, 0.4]] * 5
    assert relative_speed_error(cmd, cmd) == pytest.approx(0.0)


def test_relative_speed_error_scores_the_magnitude_not_the_axes():
    # Same speed, rotated 90 degrees: the magnitude matches, so the error is zero.
    assert relative_speed_error([[0.0, 0.5]], [[0.5, 0.0]]) == pytest.approx(0.0)


def test_relative_speed_error_averages_over_samples():
    measured = [[0.50, 0.0], [0.40, 0.0]]   # errors 0.0 and 0.2
    commanded = [[0.50, 0.0], [0.50, 0.0]]
    assert relative_speed_error(measured, commanded) == pytest.approx(0.10)


def test_relative_speed_error_rejects_a_zero_command():
    with pytest.raises(ValueError):
        relative_speed_error([[0.1, 0.0]], [[0.0, 0.0]])


# ----------------------------------------------------------------------
# heading_drift_deg
# ----------------------------------------------------------------------
def test_pure_translation_drift_is_the_whole_heading_change():
    assert heading_drift_deg([0.0, 0.0, math.radians(3.0)], 0.0, 0.05) == pytest.approx(3.0)


def test_no_drift_when_the_heading_holds():
    assert heading_drift_deg([0.2, 0.2, 0.2], 0.0, 0.05) == pytest.approx(0.0)


def test_drift_is_measured_against_a_commanded_rotation():
    # 4 steps at 0.05 s commanding 1.0 rad/s = 0.2 rad expected; 0.2 realised -> no drift.
    yaw = [0.0, 0.05, 0.10, 0.15, 0.20]
    assert heading_drift_deg(yaw, 1.0, 0.05) == pytest.approx(0.0)


def test_drift_against_a_commanded_rotation_that_was_not_achieved():
    # Same command, but only 0.10 rad realised: 0.10 rad short.
    yaw = [0.0, 0.025, 0.05, 0.075, 0.10]
    assert heading_drift_deg(yaw, 1.0, 0.05) == pytest.approx(math.degrees(0.10))


def test_drift_survives_the_angle_wrap():
    # Crossing pi must not read as hundreds of degrees.
    yaw = [math.pi - 0.05, -math.pi + 0.05]
    assert heading_drift_deg(yaw, 0.0, 0.05) == pytest.approx(math.degrees(0.10))


def test_drift_rejects_a_non_positive_dt():
    with pytest.raises(ValueError):
        heading_drift_deg([0.0, 0.1], 0.0, 0.0)


def test_signed_drift_keeps_the_direction():
    assert signed_heading_drift_deg([0.0, math.radians(0.8)], 0.0, 0.05) == pytest.approx(0.8)
    assert signed_heading_drift_deg([0.0, -math.radians(0.8)], 0.0, 0.05) == pytest.approx(-0.8)


def test_unsigned_drift_is_the_magnitude_of_the_signed_one():
    yaw = [0.0, -0.01, -0.02]
    assert heading_drift_deg(yaw, 0.0, 0.05) == pytest.approx(
        abs(signed_heading_drift_deg(yaw, 0.0, 0.05)))


# ----------------------------------------------------------------------
# drift_band_holds (A10's third criterion)
# ----------------------------------------------------------------------
def test_drift_band_holds_for_a_same_sign_drift_inside_the_band():
    assert drift_band_holds(0.47, 0.84, 0.64)


def test_drift_band_fails_for_a_reversed_sign_drift_of_similar_size():
    # The A10 probe case: frozen step +0.831 deg, 0.025 s step -0.802 deg. The magnitudes
    # differ by 0.029 deg, the drifts by 1.633 deg; the band is 0.64 deg.
    assert abs(abs(-0.802) - abs(0.831)) <= 0.64          # the defective comparison passed
    assert not drift_band_holds(-0.802, 0.831, 0.64)


def test_drift_band_fails_every_reversed_sign_candidate_from_the_a10_probe():
    for drift in (-0.177, -0.802, -1.323):
        assert not drift_band_holds(drift, 0.831, 0.64)


def test_drift_band_edge_is_inclusive():
    assert drift_band_holds(1.0, 0.5, 0.5)
    assert not drift_band_holds(1.0 + 1e-9, 0.5, 0.5)


def test_drift_band_rejects_a_negative_band():
    with pytest.raises(ValueError):
        drift_band_holds(0.0, 0.0, -0.1)


# ----------------------------------------------------------------------
# detection_lag_steps
# ----------------------------------------------------------------------
def test_zero_lag_when_contact_fires_on_the_touching_step():
    assert detection_lag_steps([False, False, True, True], [0.10, 0.05, 0.00, 0.00]) == 0


def test_positive_lag_when_contact_fires_late():
    assert detection_lag_steps([False, False, False, True], [0.10, 0.05, 0.00, 0.00]) == 1


def test_negative_lag_when_contact_fires_early():
    assert detection_lag_steps([False, True, True, True], [0.10, 0.05, 0.00, 0.00]) == -1


def test_a_contact_that_never_fires_is_an_error_not_a_lag():
    with pytest.raises(ValueError, match="missed contact"):
        detection_lag_steps([False, False, False], [0.10, 0.05, 0.00])


def test_an_approach_that_never_touches_has_no_lag():
    with pytest.raises(ValueError, match="never reached"):
        detection_lag_steps([False, False], [0.10, 0.05])


def test_lag_rejects_mismatched_lengths():
    with pytest.raises(ValueError):
        detection_lag_steps([False, True], [0.1])


# ----------------------------------------------------------------------
# count_contact_steps
# ----------------------------------------------------------------------
def test_count_contact_steps_known_answer():
    assert count_contact_steps([False, True, True, False, True]) == 3


def test_the_negative_case_wants_zero():
    assert count_contact_steps([False] * 200) == 0


def test_count_of_an_empty_series_is_zero():
    assert count_contact_steps([]) == 0


# ----------------------------------------------------------------------
# A worked profile end to end, with the answer computed by hand
# ----------------------------------------------------------------------
def test_a_synthetic_forward_profile_scores_as_expected():
    dt, target = 0.05, 0.5
    applied = np.concatenate([np.arange(0.05, 0.5, 0.05), np.full(40, 0.5)])
    start = steady_state_start(applied, target, tol=0.02, settle_steps=2)
    measured = np.column_stack([np.full(applied.size, 0.49), np.zeros(applied.size)])
    commanded = np.column_stack([applied, np.zeros(applied.size)])

    # Steady state only: measured 0.49 against a commanded 0.5 is a 2% error.
    error = relative_speed_error(measured[start:], commanded[start:])
    assert error == pytest.approx(0.02, abs=1e-9)

    yaw = np.zeros(applied.size)
    assert heading_drift_deg(yaw, 0.0, dt) == pytest.approx(0.0)

    xs = np.cumsum(measured[:, 0] * dt)
    assert path_length(np.column_stack([xs, np.zeros_like(xs)])) == pytest.approx(
        0.49 * dt * (applied.size - 1)
    )


# ----------------------------------------------------------------------
# diagnose_direction
# ----------------------------------------------------------------------
def test_agreeing_positive_readings_mean_the_command_is_followed():
    assert "follows the command" in diagnose_direction(0.48, 0.49, 0.5)


def test_a_stalled_robot_is_named_as_such():
    assert "barely moved" in diagnose_direction(0.01, 0.49, 0.5)


def test_the_g0_signature_points_at_the_sign_vector_or_the_frame():
    """The 2026-09-17 G0 run: the robot retreated while the joints reported the command."""
    message = diagnose_direction(-0.0922, +0.0922, 0.1)
    assert "wheel_sign is inverted" in message
    assert "yaw_offset is pi out" in message
    assert "wheel layout test separates them" in message


def test_both_readings_negative_points_at_something_else_driving_the_joints():
    assert "other than this controller" in diagnose_direction(-0.48, -0.49, 0.5)


def test_a_read_back_sign_issue_is_not_an_actuation_one():
    assert "read-back sign issue" in diagnose_direction(+0.48, -0.49, 0.5)


def test_diagnose_rejects_a_non_positive_target_speed():
    with pytest.raises(ValueError):
        diagnose_direction(0.1, 0.1, 0.0)


# ----------------------------------------------------------------------
# Braking metrics
# ----------------------------------------------------------------------
def test_stop_index_known_answer():
    assert stop_index([0.5, 0.3, 0.1, 0.004, 0.0], threshold=0.005) == 3


def test_stop_index_when_already_stopped():
    assert stop_index([0.0, 0.0], threshold=0.005) == 0


def test_a_robot_that_never_stopped_has_no_stopping_distance():
    with pytest.raises(ValueError, match="did not stop"):
        stop_index([0.5, 0.4, 0.3], threshold=0.005)


def test_effective_deceleration_known_answers():
    # v^2 / (2 d): 1.0 m/s stopping in 0.5 m is exactly 1.0 m/s^2.
    assert effective_deceleration(1.0, 0.5) == pytest.approx(1.0)
    assert effective_deceleration(0.5, 0.125) == pytest.approx(1.0)


@pytest.mark.parametrize("a, v0", [(1.0, 0.5), (2.0, 0.4), (0.5, 0.2), (3.0, 1.0)])
def test_effective_deceleration_inverts_the_textbook_distance(a, v0):
    """For constant deceleration a, the distance is v0^2/(2a); recover a from it."""
    distance = v0 ** 2 / (2 * a)
    assert effective_deceleration(v0, distance) == pytest.approx(a)


def test_effective_deceleration_rejects_a_zero_distance():
    with pytest.raises(ValueError):
        effective_deceleration(0.5, 0.0)


def test_mean_deceleration_known_answer():
    # 1.0 -> 0.4 m/s over 3 steps of 0.1 s is 0.6 / 0.3 = 2.0 m/s^2.
    assert mean_deceleration([1.0, 0.8, 0.6, 0.4], 0.1) == pytest.approx(2.0)


def _constant_brake(v0: float, a: float, dt: float):
    """Speed samples and the distance travelled, for a constant-deceleration brake."""
    n = int(v0 / a / dt) + 1
    speeds = [max(v0 - a * dt * i, 0.0) for i in range(n)]
    # Trapezoidal, which is what summing |delta position| between logged samples gives.
    distance = sum((speeds[i] + speeds[i + 1]) / 2 * dt for i in range(n - 1))
    return speeds, distance


def test_mean_and_effective_agree_for_a_constant_brake():
    """A constant 1.0 m/s^2 brake from 0.5 m/s, sampled at 50 ms, recovers 1.0 exactly."""
    dt, a, v0 = 0.05, 1.0, 0.5
    speeds, distance = _constant_brake(v0, a, dt)
    assert distance == pytest.approx(v0 ** 2 / (2 * a), rel=1e-12)
    assert mean_deceleration(speeds, dt) == pytest.approx(a, rel=1e-9)
    assert effective_deceleration(v0, distance) == pytest.approx(a, rel=1e-9)


def test_integrating_the_speed_series_instead_of_the_path_biases_the_answer():
    """Guards the braking script against measuring distance the cheap way.

    Summing speed * dt over the samples is a rectangular integral: it carries a half-step
    of extra distance and reports a deceleration about 9 per cent low. `a_brake` sits
    directly under the PFB's safety margin, so that bias is not acceptable and the script
    measures travelled path instead.
    """
    dt, a, v0 = 0.05, 1.0, 0.5
    speeds, _ = _constant_brake(v0, a, dt)
    rectangular = sum(s * dt for s in speeds[:-1])
    assert effective_deceleration(v0, rectangular) == pytest.approx(0.909, abs=1e-3)


def test_mean_deceleration_rejects_a_single_sample():
    with pytest.raises(ValueError):
        mean_deceleration([0.5], 0.05)


# ----------------------------------------------------------------------
# braking_distance: the off-by-one that inflated the 2026-09-17 measurement
# ----------------------------------------------------------------------
def _straight_line(steps_m):
    """Positions along +x with the given per-step travels, starting at the origin."""
    xs, x = [0.0], 0.0
    for d in steps_m:
        x += d
        xs.append(x)
    return [[v, 0.0] for v in xs]


def test_braking_distance_includes_the_first_braking_step():
    # Travels: two under power, then braking travels of 0.012, 0.005, 0.000.
    positions = _straight_line([0.02, 0.02, 0.012, 0.005, 0.000])
    # Samples 0,1,2 are pre-brake; the first post-brake sample is index 3.
    assert braking_distance(positions, onset_index=3, stop_steps=2) == pytest.approx(0.017)


def test_starting_at_the_onset_instead_would_lose_the_first_step():
    """The bug this function exists to prevent, kept as an explicit contrast."""
    positions = _straight_line([0.02, 0.02, 0.012, 0.005, 0.000])
    naive = path_length(positions[3:6])
    assert naive == pytest.approx(0.005)
    assert braking_distance(positions, 3, 2) == pytest.approx(0.017)
    # A 70 per cent under-count, which is what was observed on the real run.
    assert naive / braking_distance(positions, 3, 2) == pytest.approx(0.294, abs=1e-3)


def test_braking_distance_of_an_immediate_stop_is_the_single_step():
    positions = _straight_line([0.02, 0.012, 0.000])
    assert braking_distance(positions, onset_index=2, stop_steps=1) == pytest.approx(0.012)


def test_braking_distance_needs_the_sample_before_the_brake():
    with pytest.raises(ValueError, match="at least 1"):
        braking_distance(_straight_line([0.01, 0.01]), onset_index=0, stop_steps=1)


def test_braking_distance_rejects_a_window_past_the_log():
    with pytest.raises(ValueError, match="only"):
        braking_distance(_straight_line([0.01, 0.01]), onset_index=1, stop_steps=99)


# ----------------------------------------------------------------------
# Closed-loop tracking metrics (gate G1b)
# ----------------------------------------------------------------------
def test_a_point_on_the_line_has_no_cross_track_error():
    assert cross_track_error([2.0, 0.0], [0.0, 0.0], [1.0, 0.0]) == pytest.approx(0.0)


def test_cross_track_is_positive_to_the_left_of_the_direction():
    assert cross_track_error([0.0, 0.3], [0.0, 0.0], [1.0, 0.0]) == pytest.approx(0.3)


def test_cross_track_is_negative_to_the_right():
    assert cross_track_error([0.0, -0.3], [0.0, 0.0], [1.0, 0.0]) == pytest.approx(-0.3)


def test_cross_track_is_measured_perpendicular_not_along_the_path():
    """Moving along the path must not register as error, however far."""
    assert cross_track_error([100.0, 0.05], [0.0, 0.0], [1.0, 0.0]) == pytest.approx(0.05)


def test_cross_track_on_a_diagonal_path():
    # Path along (1,1)/sqrt(2); the point (1,0) lies 1/sqrt(2) to its right.
    assert cross_track_error([1.0, 0.0], [0.0, 0.0], [1.0, 1.0]) == pytest.approx(
        -1.0 / math.sqrt(2.0)
    )


def test_cross_track_is_independent_of_the_direction_vector_length():
    a = cross_track_error([0.0, 0.3], [0.0, 0.0], [1.0, 0.0])
    b = cross_track_error([0.0, 0.3], [0.0, 0.0], [17.0, 0.0])
    assert a == pytest.approx(b)


def test_cross_track_rejects_a_zero_direction():
    with pytest.raises(ValueError, match="non-zero"):
        cross_track_error([0.0, 1.0], [0.0, 0.0], [0.0, 0.0])


def test_rms_known_answer():
    assert rms([3.0, 4.0]) == pytest.approx(math.sqrt(12.5))


def test_rms_of_a_constant_is_its_magnitude():
    assert rms([-0.02] * 50) == pytest.approx(0.02)


def test_rms_penalises_excursions_more_than_the_mean_would():
    steady = [0.02] * 10
    spiky = [0.0] * 9 + [0.2]
    assert sum(map(abs, steady)) == pytest.approx(sum(map(abs, spiky)))
    assert rms(spiky) > rms(steady)


def test_rms_rejects_an_empty_series():
    with pytest.raises(ValueError):
        rms([])


# ----------------------------------------------------------------------
# steady_speed (realised-speed map)
# ----------------------------------------------------------------------
def test_steady_speed_excludes_the_ramp():
    # Accelerate at 1 m/s^2 to 0.4 m/s (0.08 m over 0.4 s), then cruise for 20 s.
    dt = 0.05
    t = np.arange(0.0, 20.4 + 1e-9, dt)
    d = np.where(t <= 0.4, 0.5 * t ** 2, 0.08 + 0.4 * (t - 0.4))
    assert steady_speed(d, t, from_m=1.0) == pytest.approx(0.4, abs=1e-12)
    # Including the ramp would under-state it.
    assert d[-1] / t[-1] < 0.4


def test_steady_speed_refuses_a_run_that_never_gets_past_the_threshold():
    t = np.arange(0.0, 5.0, 0.05)
    with pytest.raises(ValueError, match="never travels past"):
        steady_speed(0.1 * t, t, from_m=1.0)


# ----------------------------------------------------------------------
# contact_agreement (handle versus collection, G0 only)
# ----------------------------------------------------------------------
def test_contact_agreement_on_matching_records():
    report = contact_agreement([False, False, True, True], [[], [], ["wheel"], ["wheel", "pod"]])
    assert report["agree"] and report["disagreements"] == []


def test_contact_agreement_names_a_shape_the_collection_missed():
    # The case ruling 1 exists for: a flag-off wheel sphere touches, the collection is blind.
    report = contact_agreement([False, False, True], [[], ["wheel_respondable_fl"], ["x"]])
    assert not report["agree"]
    assert report["disagreements"] == [
        {"step": 1, "collection": False, "handles": ["wheel_respondable_fl"]}]


def test_contact_agreement_flags_a_collection_contact_no_handle_explains():
    report = contact_agreement([True], [[]])
    assert report["disagreements"][0]["collection"] is True


def test_contact_agreement_refuses_records_of_different_length():
    with pytest.raises(ValueError, match="same steps"):
        contact_agreement([True, False], [[]])
