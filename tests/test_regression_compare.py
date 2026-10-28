"""Known-answer tests for the trajectory regression comparison (hard rule 6)."""
import csv

import pytest

from pfbnav.eval.regression import compare_trajectories


def _write(path, rows):
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return path


ROWS = [{"step": i, "x": 0.1 * i, "yaw": 1.5707963 + 1e-6 * i} for i in range(5)]


def test_identical_files_are_identical(tmp_path):
    a = _write(tmp_path / "a.csv", ROWS)
    b = _write(tmp_path / "b.csv", ROWS)
    result = compare_trajectories(a, b)
    assert result["identical"]
    assert result["max_abs_difference"]["x"] == 0.0


def test_a_last_bit_difference_is_caught_and_located(tmp_path):
    changed = [dict(r) for r in ROWS]
    changed[3]["yaw"] = repr(float(ROWS[3]["yaw"]) + 2e-16)   # one ulp-scale change
    a = _write(tmp_path / "a.csv", ROWS)
    b = _write(tmp_path / "b.csv", changed)
    result = compare_trajectories(a, b)
    assert not result["identical"]
    assert result["first_difference"]["row"] == 3
    assert result["first_difference"]["column"] == "yaw"
    assert result["max_abs_difference"]["yaw"] == pytest.approx(2e-16, abs=1e-16)


def test_a_different_length_is_not_identical(tmp_path):
    a = _write(tmp_path / "a.csv", ROWS)
    b = _write(tmp_path / "b.csv", ROWS[:4])
    result = compare_trajectories(a, b)
    assert not result["identical"]
    assert result["first_difference"]["column"] == "<row count>"
    assert result["rows"] == (5, 4)


def test_unknown_columns_are_refused(tmp_path):
    a = _write(tmp_path / "a.csv", ROWS)
    with pytest.raises(ValueError, match="not in both"):
        compare_trajectories(a, a, columns=("x", "vz"))
