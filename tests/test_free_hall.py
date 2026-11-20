"""The empty-hall builder: known answers, verification of what it builds, and its refusals."""
from pathlib import Path

import numpy as np

from pfbnav.config import load_config
from pfbnav.world.free_hall import NOT_EMPTY, build_free_set, make_free_scenario
from pfbnav.world.scenarios import SCENARIO_OK, Scenario, verify, world_of

ROOT = Path(__file__).resolve().parents[1]
CFG = load_config(ROOT / "configs" / "worlds" / "empty_hall.yaml")
BASE = load_config(ROOT / "configs" / "base.yaml")


def _diff(a, b, path=""):
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) | set(b), key=str):
            out += _diff(a.get(k, "<missing>"), b.get(k, "<missing>"), f"{path}.{k}" if path else k)
        return out
    return [] if a == b else [path]


def test_the_overlay_differs_from_base_only_in_its_world():
    assert _diff(CFG, BASE) == ["experiment", "world.free_hall", "world.layout.rows"]


def test_the_hall_has_only_walls_and_every_scenario_is_a_valid_open_floor_route():
    manifest = build_free_set(range(2_900_000, 2_900_006), CFG, "tune")
    assert manifest["built"] == 6 and manifest["failures"] == []
    x0, y0, x1, y1 = CFG["world"]["layout"]["bounds"]
    margin = CFG["world"]["free_hall"]["wall_margin_m"] + CFG["world"]["layout"]["wall_thickness"]
    for s in manifest["scenarios"]:
        assert s["unmapped_count"] == 0 and s["aisle_travel_m"] == 0.0
        assert s["path_length_m"] >= CFG["world"]["free_hall"]["min_route_m"]
        for x, y in (s["start"][:2], s["goal"]):
            assert x0 + margin - 1e-9 <= x <= x1 - margin + 1e-9
            assert y0 + margin - 1e-9 <= y <= y1 - margin + 1e-9
        assert len(world_of(s, CFG).obstacles) == 4                 # the four walls, nothing else


def test_the_set_verifies_under_its_own_config_and_not_under_one_with_shelves():
    manifest = build_free_set(range(2_900_000, 2_900_003), CFG, "tune")
    assert verify(manifest, CFG)["ok"]
    wrong = verify(manifest, BASE)
    assert not wrong["ok"] and any(p["field"] == "layout_sha256" for p in wrong["problems"])


def test_the_build_is_deterministic():
    a = make_free_scenario(2_900_001, CFG, split="tune")[1]
    b = make_free_scenario(2_900_001, CFG, split="tune")[1]
    assert isinstance(a, Scenario) and a == b


def test_a_config_with_shelves_is_refused_not_papered_over():
    status, scenario, _ = make_free_scenario(2_900_000, BASE | {"world": {
        **BASE["world"], "free_hall": CFG["world"]["free_hall"]}}, split="tune")
    assert status == NOT_EMPTY and scenario is None


def test_start_and_goal_are_in_opposite_ends_of_the_hall():
    s = make_free_scenario(2_900_002, CFG, split="tune")
    assert s[0] == SCENARIO_OK
    start, goal = np.array(s[1].start[:2]), np.array(s[1].goal)
    x0, x1 = CFG["world"]["layout"]["bounds"][0], CFG["world"]["layout"]["bounds"][2]
    assert start[0] < x0 + 0.5 * (x1 - x0) < goal[0]
