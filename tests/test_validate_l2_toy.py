"""The L2 toy control's verdict and its descriptive window fractions (A28b), on known answers.

No learning: synthetic episode costs whose answers are counted by hand (hard rule 6).
"""
import importlib.util
from pathlib import Path

import pytest

PATH = Path(__file__).resolve().parents[1] / "scripts" / "validate_l2_toy.py"
_spec = importlib.util.spec_from_file_location("validate_l2_toy", PATH)
script = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(script)


def test_the_verdict_needs_all_four_criteria():
    plain = [4] * 30
    assert script.verdict(plain, [1] * 30, [0.0, 0.2], 100.0)["verdict"] == "PASS"
    # 1.25 d is the bound itself: 25 ones and 5 twos in the last 20 -> mean 1.25, still a pass.
    assert script.verdict(plain, [1] * 15 + [2] * 5 + [1] * 10, [0.1], 100.0)["checks"][
        "meets_limit"] is True
    fail = script.verdict(plain, [1] * 10 + [2] * 20, [0.1], 100.0)        # last 20: mean 2.0
    assert fail["verdict"] == "FAIL" and fail["lagrangian_last"] == 2.0
    assert fail["checks"] == {"binds": True, "meets_limit": False, "halves_plain": True,
                              "multiplier_in_range": True}
    assert script.verdict([1] * 30, [0] * 30, [0.0], 100.0)["checks"]["binds"] is False
    assert script.verdict(plain, [0] * 30, [0.0, 101.0], 100.0)["checks"][
        "multiplier_in_range"] is False


def test_the_window_fractions_count_moving_means_at_or_below_the_limit():
    # 40 episodes: 20 of cost 2, then 20 of cost 0. 21 full windows; the window ending at episode
    # index 19 + j holds (20 - j) twos, mean 2 (20 - j) / 20 <= 1 for j >= 10: 11 of the 21.
    costs = [2] * 20 + [0] * 20
    fr = script.window_fractions(costs, 1.0, window=20, late=30)
    assert fr["windows"] == 21
    assert fr["all"] == pytest.approx(11 / 21)
    # <= 1.25 needs (20 - j) / 10 <= 1.25, i.e. j >= 7.5: j = 8..20, 13 of 21.
    assert fr["all_125"] == pytest.approx(13 / 21)
    # Windows ending at index 30 or later: j = 11..20, all ten meet d.
    assert fr["late"] == 1.0 and fr["late_125"] == 1.0
    # Exactly on the limit counts as meeting it.
    assert script.window_fractions([1] * 25, 1.0, window=20, late=0)["all"] == 1.0


def test_the_window_fractions_with_too_few_episodes_are_none():
    fr = script.window_fractions([1] * 19, 1.0, window=20)
    assert fr == {"all": None, "all_125": None, "late": None, "late_125": None, "windows": 0}
    assert script.window_fractions([1] * 25, 1.0, window=20, late=100)["late"] is None
