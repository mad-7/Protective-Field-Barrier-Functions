"""The footprint probe end to end, against a fake simulator with a known robot (rule 6).

The first version of `scripts/probe_footprint.py` reported no contact for any shape on the
real simulator, because a solid that swallows a part whole intersects none of its surfaces
and `sim.checkCollision` tests surfaces. This fake reproduces that semantics -- a plate
touches a part only when the plate's slab overlaps the part's projection, and a
non-collidable part never touches -- so the probe's scan, bisection, frame transforms and
envelope assembly are checked against a robot whose answer is known before a run is spent.

The robot: a 0.28 x 0.18 m chassis box, four 0.05 m wheels at (+-0.228, +-0.1585), a
collidable pod reaching 0.35 m forward, a visual-only mast reaching 0.40 m to the rear, a
0.8 mm and a 0.2 mm fin reaching about 0.19 m to the right (thinner than the plate, as the
arm cylinder that stopped the second real run was), and a part wholly inside the chassis.
"""
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.sim.robot import ref_point_in_body

REPO_ROOT = Path(__file__).resolve().parents[1]
CFG = load_config(REPO_ROOT / "configs" / "base.yaml")
YAW = float(CFG["robot"]["yaw_offset"])


def _load_script():
    spec = importlib.util.spec_from_file_location(
        "probe_script", REPO_ROOT / "scripts" / "probe_footprint.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["probe_script"] = module
    spec.loader.exec_module(module)
    return module


class Box:
    def __init__(self, cx, cy, a, b, collidable=True):
        self.c, self.half, self.collidable = np.array([cx, cy]), (a, b), collidable

    def interval(self, n):
        centre = float(self.c @ n)
        reach = self.half[0] * abs(n[0]) + self.half[1] * abs(n[1])
        return centre - reach, centre + reach

    def corners_body(self):
        a, b = self.half
        return [self.c + [sx * a, sy * b] for sx in (-1, 1) for sy in (-1, 1)]


class Sphere:
    def __init__(self, cx, cy, r, collidable=True):
        self.c, self.r, self.collidable = np.array([cx, cy]), r, collidable

    def interval(self, n):
        centre = float(self.c @ n)
        return centre - self.r, centre + self.r

    def corners_body(self):
        return [self.c + [sx * self.r, sy * self.r] for sx in (-1, 1) for sy in (-1, 1)]


class FakeSim:
    primitiveshape_cuboid = 0
    shapeintparam_static = "static"
    shapeintparam_respondable = "respondable"

    def __init__(self, shapes):
        self.shapes = shapes                     # handle -> part
        self.plate = None
        self.plate_pose = {"gamma": 0.0, "pos": np.zeros(3)}
        self.calls = 0

    def createPrimitiveShape(self, kind, size):
        self.plate, self.thickness = 999, size[0]
        return self.plate

    def setObjectInt32Param(self, handle, key, value):
        pass

    def getObjectInt32Param(self, handle, key):
        return 1

    def getObjectAlias(self, handle, option):
        return f"/robot/part{handle}"

    def getObjectMatrix(self, handle, relative_to):
        # Shape frame: ref-aligned, at the part's centre expressed in ref.
        part = self.shapes[handle]
        c = np.array([part.c[0], part.c[1], 0.0])
        cref = np.array([c[0] * math.cos(YAW) - c[1] * math.sin(YAW),
                         c[0] * math.sin(YAW) + c[1] * math.cos(YAW), 0.0])
        return [1, 0, 0, cref[0], 0, 1, 0, cref[1], 0, 0, 1, cref[2]]

    def setObjectOrientation(self, handle, relative_to, euler):
        self.plate_pose["gamma"] = float(euler[2])

    def setObjectPosition(self, handle, relative_to, pos):
        self.plate_pose["pos"] = np.asarray(pos, dtype=float)

    def checkCollision(self, a, entity):
        self.calls += 1
        # CoppeliaSim's rule, measured 2026-09-18: a collection skips a shape whose
        # collidable flag is off; a single handle is checked regardless.
        in_collection = isinstance(entity, list)
        members = entity if in_collection else [entity]
        bearing = self.plate_pose["gamma"] - YAW
        n = np.array([math.cos(bearing), math.sin(bearing)])
        d = float(ref_point_in_body(self.plate_pose["pos"], YAW)[:2] @ n)
        half = self.thickness / 2.0
        for h in members:
            part = self.shapes[h]
            lo, hi = part.interval(n)
            overlaps = lo - half <= d <= hi + half
            swallowed = d - half < lo and hi < d + half   # inside the slab: no surface cut
            if (part.collidable or not in_collection) and overlaps and not swallowed:
                return [1, None]
        return [0, None]

    def removeObjects(self, handles):
        self.plate = None


class FakeClient:
    def __init__(self, shapes):
        self.sim = FakeSim(shapes)
        self._paths = {CFG["sim"]["objects"]["base_ref"]: 0,
                       CFG["sim"]["objects"]["chassis"]: 1}

    def object(self, path):
        return self._paths.get(path, 1)

    def shapes_in_tree(self, root):
        return list(self.sim.shapes)

    def object_bbox(self, handle):
        # Axis-aligned in ref, around the part's centre: conservative, as a real box is.
        part = self.sim.shapes[handle]
        pts = np.array([[p[0] * math.cos(YAW) - p[1] * math.sin(YAW),
                         p[0] * math.sin(YAW) + p[1] * math.cos(YAW)]
                        for p in part.corners_body()])
        c = np.array([part.c[0] * math.cos(YAW) - part.c[1] * math.sin(YAW),
                      part.c[0] * math.sin(YAW) + part.c[1] * math.cos(YAW)])
        rel = pts - c
        return ([rel[:, 0].min(), rel[:, 1].min(), 0.0], [rel[:, 0].max(), rel[:, 1].max(), 0.1])

    def entity_for(self, handles):
        return list(handles)

    def special_flags(self, handle):
        on = self.sim.shapes[handle].collidable
        return {"collidable": on, "measurable": on, "detectable": on}


@pytest.fixture(scope="module")
def probed():
    shapes = {
        # The chassis's collidable flag is off, as in the real scene: a collection skips
        # it, but the contact check counts it by handle, so the envelope must include it.
        1: Box(0.0, 0.0, 0.28, 0.18, collidable=False),
        2: Sphere(0.228, 0.1585, 0.05), 3: Sphere(0.228, -0.1585, 0.05),
        4: Sphere(-0.228, 0.1585, 0.05), 5: Sphere(-0.228, -0.1585, 0.05),
        6: Box(0.325, 0.0, 0.025, 0.025),                         # pod, reaches 0.35
        7: Box(-0.37, 0.0, 0.03, 0.01, collidable=False),         # mast, reaches -0.40
        # Fins seen edge-on along -y, thinner than the 6 mm plate, so each registers only
        # in two bands as wide as it is thin. 0.8 mm: the 5 mm scan steps over both bands
        # (checked by hand: samples at 1.4 and 3.6 mm from its centre, bands at 2.6-3.4),
        # and the 0.5 mm fine retry finds it.
        # 0.2 mm: thinner than the fine step too, so it falls back to its box.
        8: Box(0.05, -0.19, 0.02, 0.0004),
        10: Box(-0.05, -0.19, 0.02, 0.0001),
        9: Box(0.10, 0.05, 0.02, 0.02),                           # inside the chassis
    }
    script = _load_script()
    client = FakeClient(shapes)
    bearings = np.radians(np.arange(-180.0, 180.0, 5.0))
    shape_bearings = np.radians(np.arange(-180.0, 180.0, 10.0))
    return script, script.probe(client, CFG, bearings, shape_bearings), bearings


def test_collidable_classification(probed):
    """By flag, not by whether a single handle registers: the mast registers as a handle
    (flags are ignored there) but its flag is off, so the contact check cannot see it."""
    _, result, _ = probed
    kinds = {r["name"]: r["collidable"] for r in result["records"]}
    mast = next(r for r in result["records"] if r["name"] == "/robot/part7")
    assert mast["registers_by_handle"] is True
    assert kinds["/robot/part7"] is False
    assert kinds["/robot/part8"] is True        # the fin is found despite being thin
    assert all(v for k, v in kinds.items() if k != "/robot/part7")


def test_per_shape_support_matches_the_geometry(probed):
    script, result, _ = probed
    tol = script.TOL_M
    wheel = next(r for r in result["records"] if r["name"] == "/robot/part2")
    # Shape bearings run from -180 in 10 deg steps; +90 deg is index 27.
    assert wheel["support_probe"][27] == pytest.approx(0.1585 + 0.05, abs=tol)
    pod = next(r for r in result["records"] if r["name"] == "/robot/part6")
    assert pod["support_probe"][18] == pytest.approx(0.35, abs=tol)
    # Upper bounds: never below the truth.
    assert pod["support_probe"][18] >= 0.35 - 1e-12


def test_collidable_envelope_excludes_the_visual_only_mast(probed):
    script, result, bearings = probed
    tol = script.TOL_M
    env = script.envelope(bearings, result["collidable_support"])
    assert env["half_length_front"] == pytest.approx(0.35, abs=tol)
    assert env["half_length_rear"] == pytest.approx(0.28, abs=tol)     # chassis, no mast
    assert env["half_width_left"] == pytest.approx(0.2085, abs=tol)    # wheel face


def test_collidable_support_is_never_below_the_true_geometry(probed):
    _, result, bearings = probed
    parts = [Box(0.0, 0.0, 0.28, 0.18), Sphere(0.228, 0.1585, 0.05),
             Sphere(0.228, -0.1585, 0.05), Sphere(-0.228, 0.1585, 0.05),
             Sphere(-0.228, -0.1585, 0.05), Box(0.325, 0.0, 0.025, 0.025),
             Box(0.05, -0.19, 0.02, 0.0004), Box(-0.05, -0.19, 0.02, 0.0001)]
    for i, b in enumerate(bearings):
        n = np.array([math.cos(b), math.sin(b)])
        truth = max(p.interval(n)[1] for p in parts)
        assert truth <= result["collidable_support"][i] + 1e-12
        # Tight to the tolerance, except where the thin fin's box was added (right side).
        assert result["collidable_support"][i] <= truth + 0.021


def test_a_fin_thicker_than_the_fine_step_is_measured_by_the_retry(probed):
    script, result, _ = probed
    fin = next(r for r in result["records"] if r["name"] == "/robot/part8")
    assert fin["candidate"]
    # Shape bearings from -180 in 10 deg steps; -90 deg is index 9.
    assert fin["how"][9] == "probe-fine"
    assert fin["support_probe"][9] == pytest.approx(0.1904, abs=script.TOL_M)
    assert "/robot/part8" in result["thin_shapes"]


def test_a_fin_thinner_than_the_fine_step_falls_back_to_its_box(probed):
    _, result, _ = probed
    fin = next(r for r in result["records"] if r["name"] == "/robot/part10")
    assert fin["how"][9] == "bbox"
    # The box is exact for this axis-aligned fin; the point is that it is never below.
    assert fin["support_probe"][9] >= 0.1901 - 1e-12
    assert "/robot/part10" in result["thin_shapes"]


def test_a_part_inside_the_chassis_is_classified_but_not_probed(probed):
    _, result, _ = probed
    inner = next(r for r in result["records"] if r["name"] == "/robot/part9")
    assert inner["collidable"] and not inner["candidate"]
    assert inner["support_probe"] is None


def test_the_chassis_counts_although_its_flag_is_off(probed):
    _, result, _ = probed
    chassis = next(r for r in result["records"] if r["is_chassis"])
    assert chassis["collidable_flag"] is False and chassis["collidable"] is True
