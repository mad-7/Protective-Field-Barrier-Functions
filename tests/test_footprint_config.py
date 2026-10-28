"""D8: the committed footprint table and the config agree with each other (ruling 3)."""
import json
import math
from pathlib import Path

import pytest

from pfbnav.config import load_config
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.utils.runio import REPO_ROOT, file_sha256

CFG = load_config("configs/base.yaml")
TABLE_PATH = REPO_ROOT / CFG["robot"]["footprint_table"]


@pytest.fixture(scope="module")
def table():
    return FootprintTable.from_csv(TABLE_PATH)


def _header(path: Path) -> dict[str, str]:
    lines = [line[2:].strip() for line in path.read_text().splitlines() if line.startswith("# ")]
    return dict(line.split(": ", 1) for line in lines)


def _equivalence_path(equivalences, start, goal):
    """The entries joining scene hash `start` to `goal` (either direction), or None."""
    frontier, seen = [(start, [])], {start}
    while frontier:
        node, path = frontier.pop(0)
        if node == goal:
            return path
        for e in equivalences:
            for a, b in ((e["from"], e["to"]), (e["to"], e["from"])):
                if a == node and b not in seen:
                    seen.add(b)
                    frontier.append((b, [*path, e]))
    return None


def test_the_equivalence_path_follows_chains_and_refuses_gaps():
    entries = [{"from": "A", "to": "B"}, {"from": "C", "to": "B"}, {"from": "D", "to": "E"}]
    assert [e["from"] for e in _equivalence_path(entries, "A", "C")] == ["A", "C"]
    assert _equivalence_path(entries, "A", "E") is None
    assert _equivalence_path(entries, "A", "A") == []


def test_the_table_was_built_from_the_configured_scene():
    header = _header(TABLE_PATH)
    assert header["scene"] == CFG["sim"]["scene"]
    configured = file_sha256(REPO_ROOT / CFG["sim"]["scene"])
    built_from = header["scene_sha256"]
    if built_from != configured:
        # A scene file changes when anything in it is saved, the camera view included. A
        # measurement stays valid across such a save only if the geometry is identical, and
        # that has to be SHOWN: docs/scene/equivalence.json records each such pair with the
        # inventory diff that proved it (every object's type, pose and bounding box).
        # Edits chain (camera save 2026-09-25, then wall and floor 2026-10-14), so the table's
        # scene is joined to the configured one by a PATH of recorded equivalences, and every hop
        # on it must carry its evidence.
        equivalences = json.loads(
            (REPO_ROOT / "docs" / "scene" / "equivalence.json").read_text())["equivalences"]
        path = _equivalence_path(equivalences, built_from, configured)
        assert path is not None, (
            "the scene changed since the footprint table was built, and no recorded "
            "equivalence covers it; re-run the probe and rebuild the table")
        robot = CFG["sim"]["objects"]["robot_tree"]
        for evidence in path:
            assert (REPO_ROOT / evidence["evidence"]).is_file(), (
                f"the equivalence cites {evidence['evidence']}, which is missing")
            # The evidence must support the entry's scope: a whole-scene entry touched nothing;
            # a robot-scoped entry (an edit that changed the world, 2026-10-14) touched nothing
            # under the robot's tree, which is all the footprint table measures.
            diff = json.loads((REPO_ROOT / evidence["evidence"]).read_text(encoding="utf-8"))
            touched = (list(diff["added"]) + list(diff["removed"])
                       + [m["path"] for m in diff["moved"]]
                       + [r["path"] for r in diff["resized"]])
            if evidence.get("scope") == "robot":
                assert not [p for p in touched if p == robot or p.startswith(robot + "/")], (
                    f"{evidence['to'][:12]}: a robot-scoped equivalence whose evidence shows the "
                    "robot changed")
            else:
                assert not touched, (f"{evidence['to'][:12]}: a whole-scene equivalence whose "
                                     "evidence shows changes")
    assert header["engine"] == "ODE"


def test_half_width_covers_the_measured_sides(table):
    # The rectangle's half_width is the lethal radius's base: it must not sit inside the
    # probe's sides. The table carries a spline margin, so compare with the knots minus it.
    margin = float(_header(TABLE_PATH)["spline_margin_added_m"])
    for side in (math.pi / 2, -math.pi / 2):
        assert CFG["robot"]["half_width"] >= table.extent(side) - margin - 1e-6


def test_half_length_covers_the_measured_front_and_rear(table):
    margin = float(_header(TABLE_PATH)["spline_margin_added_m"])
    for end in (0.0, math.pi):
        assert CFG["robot"]["half_length"] >= table.extent(end) - margin - 1e-6


def test_d8_planner_radii(table):
    lethal = CFG["robot"]["half_width"] + CFG["planner"]["lethal_margin"]
    soft = table.circumscribed + CFG["planner"]["soft_margin"]
    # 0.3086 since the 2026-10-16 ruling (lethal margin 0.03 -> 0.07 -> 0.10). It must cover the
    # at-rest field abeam, half_width + lateral_margin = 0.2586, which 0.2386 did not (D1).
    assert lethal == pytest.approx(0.3086, abs=1e-9)
    assert lethal > CFG["robot"]["half_width"] + CFG["certified"]["lateral_margin"]
    # 0.3674 on the S300 footprint (circumscribed 0.33742 + 0.03); it was 0.3819 with the
    # pods, whose fore-and-aft reach set the circumscribed radius.
    assert soft == pytest.approx(0.3674, abs=5e-4)
    assert lethal < soft


def test_axis_ceilings_sit_inside_the_budget_on_their_own_axes():
    a, b = CFG["actuation"], CFG["actuation"]["wheel_budget"]
    filter_limit = b["filter_fraction"] * b["v_rim"]
    # Ruled 2026-10-19: G1b's outcome under the A20 limits, v_max 0.50 and vy_max 0.50 -- the
    # box is square again (translate_while_rotating passes 0.50 by 0.038 deg). PRE-A20 it was
    # v_max 0.40 (the 2026-09-20 re-run failed 0.50 on translate_while_rotating), non-square
    # because the measurement was.
    assert a["v_max"] == pytest.approx(0.5) and a["vy_max"] == pytest.approx(0.5)
    # A pure forward or lateral command at its ceiling needs its own speed at the rim.
    assert a["v_max"] <= filter_limit and a["vy_max"] <= filter_limit
