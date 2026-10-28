"""Known-answer tests for pfbnav/sim/scanning.py (hard rule 6)."""
import pytest

from pfbnav.sim.scanning import rays_per_step, resolution_for_bins


def test_four_quadrant_sensors_share_the_bins():
    # 120 bins over 360 deg, four sensors of 90 deg: 30 rays each.
    assert resolution_for_bins(90.0, 120) == 30


def test_a_full_turn_sensor_carries_every_bin():
    assert resolution_for_bins(360.0, 120) == 120


def test_oversampling_multiplies_the_rays():
    assert resolution_for_bins(90.0, 120, oversample=2.0) == 60
    assert resolution_for_bins(90.0, 36, oversample=1.5) == 14      # 13.5 -> 14


def test_rounds_up_rather_than_short_changing_a_stream():
    # 100 bins over 360, 90 deg sensor: 25 exactly; 101 bins needs 26, not 25.
    assert resolution_for_bins(90.0, 100) == 25
    assert resolution_for_bins(90.0, 101) == 26


def test_a_field_of_view_read_back_with_float_noise_does_not_add_a_ray():
    # The scene reports its 120 deg sensors as 120.0000033 deg (2026-09-21).
    assert resolution_for_bins(120.00000333930423, 120, oversample=2.0) == 80


def test_never_returns_zero_rays():
    assert resolution_for_bins(1.0, 1) == 1


def test_rejects_an_impossible_field_of_view():
    for fov in (0.0, -90.0, 361.0):
        with pytest.raises(ValueError):
            resolution_for_bins(fov, 120)


def test_rejects_a_non_positive_bin_count_or_oversample():
    with pytest.raises(ValueError):
        resolution_for_bins(90.0, 0)
    with pytest.raises(ValueError):
        resolution_for_bins(90.0, 120, oversample=0.0)


def test_rays_per_step_sums_the_sensors():
    assert rays_per_step([256, 256, 256, 256]) == 1024
    assert rays_per_step([30, 30, 30, 30]) == 120


def test_rays_per_step_rejects_an_empty_sensor():
    with pytest.raises(ValueError):
        rays_per_step([30, 0])
