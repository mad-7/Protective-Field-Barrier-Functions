"""Known answers for the reload envelope and the trajectory-column filter (ruled 2026-10-14)."""
import csv

import pytest

from pfbnav.eval.regression import (
    compare_runs,
    reload_envelope,
    trajectory_columns,
    within_envelope,
)


def _run(tmp_path, name, rows, file="g1b.csv"):
    d = tmp_path / name
    d.mkdir()
    with (d / file).open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    return d


def _traj(xs, hit="", seconds=0.0):
    return [{"step": i, "x": x, "hit": hit, "step_seconds": seconds + i} for i, x in enumerate(xs)]


def test_wall_clock_columns_are_not_trajectory():
    assert trajectory_columns(["x", "step_seconds", "mean_ms", "hit", "t"]) == ("x", "hit", "t")


def test_identical_runs_compare_identical_despite_different_timings(tmp_path):
    a = _run(tmp_path, "a", _traj([0.0, 0.1, 0.2], seconds=0.01))
    b = _run(tmp_path, "b", _traj([0.0, 0.1, 0.2], seconds=0.07))
    assert compare_runs(a, b)["identical"]


def test_the_envelope_is_the_largest_pairwise_deviation(tmp_path):
    l1 = _run(tmp_path, "l1", _traj([0.0, 0.100, 0.200]))
    l2 = _run(tmp_path, "l2", _traj([0.0, 0.101, 0.200]))
    l3 = _run(tmp_path, "l3", _traj([0.0, 0.099, 0.203]))
    env = reload_envelope([l1, l2, l3])["g1b.csv"]
    assert env["x"] == pytest.approx(0.003)          # l1 vs l3 at row 2
    assert env["<rows>"] == 0.0 and env["hit"] == 0.0


def test_a_deviation_inside_passes_and_one_outside_fails(tmp_path):
    l1 = _run(tmp_path, "l1", _traj([0.0, 0.100, 0.200]))
    l2 = _run(tmp_path, "l2", _traj([0.0, 0.102, 0.200]))
    env = reload_envelope([l1, l2])
    c = _run(tmp_path, "c", _traj([0.0, 0.100, 0.200]))
    near = _run(tmp_path, "near", _traj([0.0, 0.1015, 0.200]))
    far = _run(tmp_path, "far", _traj([0.0, 0.100, 0.2025]))
    assert within_envelope(c, near, env)["within"]
    out = within_envelope(c, far, env)
    assert not out["within"] and out["excess"][0]["column"] == "x"


def test_a_contact_that_changes_is_outside_an_envelope_where_contacts_never_did(tmp_path):
    l1 = _run(tmp_path, "l1", _traj([0.0, 0.1], hit="/wall"))
    l2 = _run(tmp_path, "l2", _traj([0.0, 0.1], hit="/wall"))
    env = reload_envelope([l1, l2])
    c = _run(tmp_path, "c", _traj([0.0, 0.1], hit="/wall"))
    d = _run(tmp_path, "d", _traj([0.0, 0.1], hit="/shelf"))
    out = within_envelope(c, d, env)
    assert not out["within"] and out["excess"][0]["column"] == "hit"


def test_a_different_row_count_is_outside_a_zero_row_envelope(tmp_path):
    l1 = _run(tmp_path, "l1", _traj([0.0, 0.1, 0.2]))
    l2 = _run(tmp_path, "l2", _traj([0.0, 0.1, 0.2]))
    env = reload_envelope([l1, l2])
    c = _run(tmp_path, "c", _traj([0.0, 0.1, 0.2]))
    d = _run(tmp_path, "d", _traj([0.0, 0.1]))
    assert any(e["column"] == "<rows>" for e in within_envelope(c, d, env)["excess"])
