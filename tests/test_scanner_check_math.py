"""Known-answer tests for the scanner check's evaluation (hard rule 6).

`scripts/diag_scanner_check.py` decides which pixel convention the simulator uses and
whether the published scan is right, so its own evaluation is checked first on synthetic
captures whose answer is known by construction.
"""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.depth_geometry import cast_to_wall, matrix_from_pose
from pfbnav.sim.scan_packet import bin_bearings, decode_scan, encode_scan

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "diag_scanner_check.py"
REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
R_MIN = float(CFG["scanners"]["range_min"])
R_MAX = float(CFG["scanners"]["range_max"])
POLICY = int(CFG["scanners"]["policy_bins"])
FILTER = int(CFG["scanners"]["filter_bins"])
FOV = math.radians(120.0)
RES = 80
# The four sensors as the scanner reported them: axes +-30 and +-150 deg.
SENSORS = [matrix_from_pose(math.radians(a), [0.3 * math.copysign(1, math.cos(math.radians(a))),
                                              0.0, 0.2])
           for a in (30.0, -30.0, 150.0, -150.0)]


def _load():
    spec = importlib.util.spec_from_file_location("diag_scanner_check", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


M = _load()


def capture(normal_deg=None, distance=1.0, *, mirrored, euclidean):
    """A raw capture as the simulator would give it, under a chosen truth."""
    out = []
    for m in SENSORS:
        if normal_deg is None:
            depth = np.full(RES, R_MAX)
        else:
            depth = cast_to_wall(FOV, RES, m, math.radians(normal_deg), distance,
                                 mirrored=mirrored, euclidean=euclidean, far=R_MAX)
        out.append({"depth": depth.tolist(), "matrix": m, "fov": FOV})
    return out


def packet(filter_ranges, filter_valid=None):
    fb = bin_bearings(FILTER)
    fv = np.ones(FILTER, bool) if filter_valid is None else filter_valid
    values = encode_scan(1.0, np.full(POLICY, R_MAX), np.ones(POLICY, bool), fb,
                         filter_ranges, fv, range_min=R_MIN, range_max=R_MAX)
    return decode_scan(values, policy_bins=POLICY, filter_bins=FILTER,
                       range_min=R_MIN, range_max=R_MAX)


def wall_packet(normal_deg, distance):
    b = bin_bearings(FILTER)
    c = np.cos(b - math.radians(normal_deg))
    with np.errstate(divide="ignore"):
        r = np.where(c > 1e-9, distance / c, np.inf)
    return packet(np.clip(r, R_MIN, R_MAX))


# ----------------------------------------------------------------------
# Identifying the convention from a raw capture
# ----------------------------------------------------------------------
@pytest.mark.parametrize("truth", M.CONVENTIONS)
@pytest.mark.parametrize("normal", [90.0, 0.0, 180.0])
def test_the_raw_evaluation_names_the_true_convention(truth, normal):
    base = capture(None, mirrored=truth[0], euclidean=truth[1])
    wall = capture(normal, 1.0, mirrored=truth[0], euclidean=truth[1])
    result = M.raw_hypotheses(base, wall, math.radians(normal), 1.0, R_MAX)
    assert result["best"] == M.label(*truth)
    assert result["conventions"][M.label(*truth)]["max_abs_m"] < 1e-6


def test_the_first_runs_signature_is_identified_as_mirrored():
    # 2026-09-21: a mirrored sensor read as direct put the left wall on a 60 deg line.
    base = capture(None, mirrored=True, euclidean=False)
    wall = capture(90.0, 1.0, mirrored=True, euclidean=False)
    result = M.raw_hypotheses(base, wall, math.radians(90.0), 1.0, R_MAX)
    assert result["best"] == "mirrored/perpendicular"
    assert result["conventions"]["direct/perpendicular"]["max_abs_m"] > 0.1


def test_selection_does_not_depend_on_the_hypothesis():
    # Every convention is scored on the same pixels: those whose depth changed.
    base = capture(None, mirrored=False, euclidean=False)
    wall = capture(0.0, 1.0, mirrored=False, euclidean=False)
    counts = {v["selected"] for v in M.raw_hypotheses(base, wall, 0.0, 1.0, R_MAX)
              ["conventions"].values()}
    assert len(counts) == 1 and counts.pop() > 0


# ----------------------------------------------------------------------
# The published packet against a wall
# ----------------------------------------------------------------------
def test_an_exact_packet_has_zero_residual():
    base = packet(np.full(FILTER, R_MAX))
    res = M.packet_residuals(base, wall_packet(90.0, 1.0), math.radians(90.0), 1.0)
    assert res["bins"] > 10 and res["max_abs_m"] < 1e-9


def test_a_packet_one_centimetre_long_reads_as_ten_millimetres():
    base = packet(np.full(FILTER, R_MAX))
    far = wall_packet(0.0, 1.01)                  # built for a wall 1 cm further out
    res = M.packet_residuals(base, far, 0.0, 1.0)
    assert res["max_abs_m"] == pytest.approx(0.01, abs=1e-9)


def test_bins_facing_away_from_the_wall_are_not_scored():
    base = packet(np.full(FILTER, R_MAX))
    res = M.packet_residuals(base, wall_packet(90.0, 1.0), math.radians(90.0), 1.0)
    assert all(abs(r["bearing_deg"] - 90.0) <= M.FACING_DEG for r in res["by_bearing"])


def test_a_wall_nobody_saw_reports_no_bins():
    base = packet(np.full(FILTER, R_MAX))
    assert M.packet_residuals(base, base, math.radians(90.0), 1.0) == {"bins": 0}


# ----------------------------------------------------------------------
# The robot must never see itself
# ----------------------------------------------------------------------
def test_a_free_baseline_has_no_self_hits():
    table = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
    assert M.self_hits(packet(np.full(FILTER, R_MAX)), table)["count"] == 0


def test_a_return_inside_the_footprint_is_a_self_hit():
    table = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
    r = np.full(FILTER, R_MAX)
    b = bin_bearings(FILTER)
    i = int(np.argmin(np.abs(b)))                 # dead ahead: the forward pod is there
    r[i] = float(np.atleast_1d(table.extent(np.array([b[i]])))[0])
    hits = M.self_hits(packet(r), table)
    assert hits["count"] == 1
    assert hits["bearings_deg"][0] == pytest.approx(math.degrees(b[i]))


def test_an_invalid_bin_is_not_a_self_hit():
    table = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
    r = np.full(FILTER, R_MAX)
    r[0] = R_MIN
    v = np.ones(FILTER, bool)
    v[0] = False
    assert M.self_hits(packet(r, v), table)["count"] == 0


# ----------------------------------------------------------------------
# Only the wall's face is scored
# ----------------------------------------------------------------------
def test_points_past_the_walls_ends_are_not_on_its_face():
    # A 6 m wall along x at y = 1: its face spans |x| <= 2.9 once the margin is taken off.
    pts = np.array([[0.0, 1.0], [2.8, 1.0], [2.95, 1.0], [-3.2, 1.0]])
    got = M.on_face(pts, math.radians(90.0), 3.0)
    assert got.tolist() == [True, True, False, False]


def test_an_end_face_hit_no_longer_inflates_the_worst_residual():
    # 2026-09-21: pixels hitting the 5 cm end faces read ~50 mm off the plane and set the
    # maximum. On an infinite synthetic wall nothing is excluded that is on the face.
    base = capture(None, mirrored=True, euclidean=False)
    wall = capture(90.0, 1.0, mirrored=True, euclidean=False)
    r = M.raw_hypotheses(base, wall, math.radians(90.0), 1.0, R_MAX, half_length=100.0)
    assert r["conventions"]["mirrored/perpendicular"]["max_abs_m"] < 1e-6


# ----------------------------------------------------------------------
# A scale error and an offset are told apart by distance
# ----------------------------------------------------------------------
def test_distance_fit_reads_a_pure_scale_as_slope_without_intercept():
    f = M.distance_fit({0.6: 0.006, 1.0: 0.010, 2.0: 0.020})      # 1 per cent long
    assert f["slope_mm_per_m"] == pytest.approx(10.0)
    assert f["intercept_mm"] == pytest.approx(0.0, abs=1e-9)


def test_distance_fit_reads_a_pure_offset_as_intercept_without_slope():
    f = M.distance_fit({0.6: 0.010, 1.0: 0.010, 2.0: 0.010})
    assert f["slope_mm_per_m"] == pytest.approx(0.0, abs=1e-9)
    assert f["intercept_mm"] == pytest.approx(10.0)


def test_distance_fit_needs_two_distances():
    assert M.distance_fit({1.0: 0.01}) is None


# ----------------------------------------------------------------------
# Cleanup survives a wall the simulator already removed (2026-09-21)
# ----------------------------------------------------------------------
class _FakeSim:
    def __init__(self, valid):
        self.valid = valid
        self.removed = []

    def isHandle(self, handle):
        return self.valid

    def removeObjects(self, handles):
        if not self.valid:
            raise RuntimeError("354: in sim.removeObjects: found invalid handles.")
        self.removed.extend(handles)


class _FakeClient:
    def __init__(self, valid):
        self.sim = _FakeSim(valid)


def test_a_wall_already_deleted_by_the_simulator_is_not_removed_again():
    client = _FakeClient(valid=False)            # deleted when the run stopped
    M.remove_wall(client, 501)
    assert client.sim.removed == []


def test_a_live_wall_is_removed():
    client = _FakeClient(valid=True)
    M.remove_wall(client, 501)
    assert client.sim.removed == [501]


def test_no_wall_means_nothing_to_remove():
    client = _FakeClient(valid=True)
    M.remove_wall(client, None)
    assert client.sim.removed == []
