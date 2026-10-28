"""Known-answer tests for scenario generation (hard rule 6). Pure; no simulator.

A scenario is what an episode runs, so the tests that matter are: the plan really threads an
aisle (or the scenario is refused by name), the unmapped rate is the one asked for, the set
records what it achieved rather than what it intended, a locked set cannot be overwritten,
and verification catches a world that has moved under a claimed identity.
"""
import copy
import functools
import itertools
import json
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.world.layouts import generate, rectangle
from pfbnav.world.oracle import (
    GOAL_INVALID,
    RESIDUAL,
    ROUTE_OK,
    START_INVALID,
    VALIDITY_FAILURES,
)
from pfbnav.world.scenarios import (
    NO_ENDPOINTS,
    NO_VALID_ROUTE,
    SCENARIO_OK,
    Scenario,
    ValidityCheck,
    aisle_centres,
    aisle_rectangles,
    aisle_travel,
    build_set,
    episode_unmapped_count,
    load_set,
    make_scenario,
    scenario_sha256,
    scenarios_sha256,
    verify,
    world_of,
    write_set,
)

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")


# ----------------------------------------------------------------------
# Aisle geometry and the measure that decides a scenario
# ----------------------------------------------------------------------
def test_aisle_rectangles_are_the_gaps_between_rows_where_they_overlap():
    layout = generate(1_000_003, CFG, stratum="narrow")
    rects = aisle_rectangles(layout)
    shelves = sorted((o for o in layout.mapped if o.name.startswith("shelf_")),
                     key=lambda o: float(o.polygon[:, 1].min()))
    assert len(rects) == len(shelves) - 1
    for (x0, y0, x1, y1), (lower, upper) in zip(rects, itertools.pairwise(shelves),
                                                strict=True):
        assert y1 - y0 == pytest.approx(layout.metadata["aisle_width_m"], abs=1e-9)
        assert x0 == pytest.approx(max(lower.polygon[:, 0].min(), upper.polygon[:, 0].min()))
        assert x1 == pytest.approx(min(lower.polygon[:, 0].max(), upper.polygon[:, 0].max()))


def test_aisle_travel_known_answers():
    rect = [(0.0, 0.0, 10.0, 1.0)]
    through = np.array([[0.0, 0.5], [10.0, 0.5]])
    assert aisle_travel(through, rect) == pytest.approx(10.0, abs=0.05)
    beside = np.array([[0.0, 2.0], [10.0, 2.0]])
    assert aisle_travel(beside, rect) == pytest.approx(0.0)
    # Along the aisle for 5 m, then out through its edge: the vertical leg from y = 0.5 to
    # y = 1.0 is inside too, so the answer is 5.5 m, not 5.
    half = np.array([[0.0, 0.5], [5.0, 0.5], [5.0, 2.0], [10.0, 2.0]])
    assert aisle_travel(half, rect) == pytest.approx(5.5, abs=0.1)


def test_aisle_travel_is_zero_without_aisles():
    assert aisle_travel(np.array([[0.0, 0.0], [1.0, 0.0]]), []) == 0.0


def test_aisle_centres_include_the_ends_and_the_gaps():
    layout = generate(1_000_005, CFG, stratum="medium")
    centres = aisle_centres(layout)
    rows = layout.metadata["rows"]
    assert len(centres) == rows + 1
    assert centres == sorted(centres)


# ----------------------------------------------------------------------
# Scenarios
# ----------------------------------------------------------------------
# Every scenario now passes the validity rules (a free-orientation oracle check, a few seconds
# each), so builds are cached and shared by the tests that do not need seeds of their own.
@functools.cache
def _scenario(seed, stratum=None, episode_rate=0.0, split=""):
    return make_scenario(seed, CFG, stratum=stratum, episode_rate=episode_rate, split=split)


@functools.cache
def _built(lo, hi, split, episode_rate):
    return build_set(range(lo, hi), CFG, split, episode_rate=episode_rate)


def _shared_set():
    """A small set shared by the manifest, lock and verification tests; a deep copy, so a test
    that tampers with it cannot leak into the next."""
    return copy.deepcopy(_built(1_000_080, 1_000_083, "train", 0.5))


@pytest.mark.parametrize("stratum", ["narrow", "medium", "wide"])
def test_a_scenario_threads_an_aisle_of_its_own_stratum(stratum):
    status, scenario, layout = _scenario(1_000_003, stratum, 0.0, "train")
    assert status == SCENARIO_OK and scenario is not None
    assert scenario.stratum == stratum
    assert scenario.aisle_travel_m >= float(CFG["world"]["layout"]["min_aisle_travel"])
    band = next(s for s in __import__("pfbnav.world.strata", fromlist=["x"])
                .strata_from_config(CFG) if s.name == stratum)
    assert band.min_clearance <= scenario.clearance_m < band.max_clearance
    assert layout is not None


def test_a_scenario_is_reproducible_from_its_seed():
    a = _scenario(1_000_009, None, 1.0)[1]
    b = make_scenario(1_000_009, CFG, episode_rate=1.0)[1]
    assert a is not None and b is not None
    assert a.scenario_sha256 == b.scenario_sha256
    assert a.start == b.start and a.goal == b.goal


def test_the_scenario_hash_covers_the_endpoints_not_just_the_world():
    scenario = _scenario(1_000_003, "narrow", 0.0, "train")[1]
    assert scenario is not None
    moved = scenario_sha256(scenario.layout_sha256,
                            (scenario.start[0] + 0.5, scenario.start[1], scenario.start[2]),
                            scenario.goal)
    assert moved != scenario.scenario_sha256


def test_an_impossible_requirement_is_a_named_failure_not_an_easier_scenario():
    """Ask for more aisle travel than the hall can offer: the scenario must be refused."""
    cfg = copy.deepcopy(CFG)
    cfg["world"]["layout"]["min_aisle_travel"] = 500.0
    status, scenario, _ = make_scenario(1_000_003, cfg, stratum="narrow")
    assert status == NO_ENDPOINTS and scenario is None


def test_the_start_faces_along_its_plan():
    scenario = _scenario(1_000_003, "medium", 0.0, "train")[1]
    assert scenario is not None
    assert -np.pi <= scenario.start[2] <= np.pi


def test_unmapped_offsets_are_recorded_one_per_obstacle():
    """Ruling 3: H1 is reportable against how far the map understates the world."""
    scenario = _scenario(1_000_009, None, 1.0)[1]
    assert scenario is not None
    assert len(scenario.unmapped_offsets_m) == scenario.unmapped_count
    assert all(d >= 0.0 for d in scenario.unmapped_offsets_m)


# ----------------------------------------------------------------------
# The validity rules (ruled 2026-10-15)
# ----------------------------------------------------------------------
class _Scripted:
    """A stand-in for `ValidityCheck` that returns verdicts from a script (then `ok`) and
    records the starts it was shown -- the bookkeeping tests need failures on demand."""

    def __init__(self, verdicts):
        self.verdicts = list(verdicts)
        self.starts = []

    def for_layout(self, mapped_polygons):
        def check(path, start_xy):
            self.starts.append((float(start_xy[0]), float(start_xy[1])))
            return self.verdicts.pop(0) if self.verdicts else ROUTE_OK
        return check


def test_a_built_scenario_passes_the_validity_rules_when_checked_afresh():
    """Re-plan each scenario of a set from what it records and run the rules on the result,
    independently of the build: every one must come back `ok`."""
    from pfbnav.planning.astar import OK, plan_from_config
    from pfbnav.world.baking import bake, planner_radii
    from pfbnav.world.scenarios import _circumscribed

    manifest = _shared_set()
    assert manifest["built"] >= 2
    check = ValidityCheck(CFG)
    radii = planner_radii(CFG, circumscribed=_circumscribed(CFG))
    for s in manifest["scenarios"]:
        world = world_of(s, CFG)
        baked = bake(world.mapped, world.bounds, float(CFG["planner"]["resolution"]), radii)
        start = (float(s["start"][0]), float(s["start"][1]))
        result = plan_from_config(baked, start, tuple(s["goal"]), CFG)
        assert result.status == OK
        verdict = check.for_layout([o.polygon for o in world.mapped])(result.path_world, start)
        assert verdict == ROUTE_OK, (s["seed"], verdict)


def test_a_rejected_route_redraws_the_goal_keeps_the_start_and_is_logged_by_cause():
    base = make_scenario(1_000_003, CFG, stratum="narrow", validity=_Scripted([]))[1]
    script = _Scripted([RESIDUAL, START_INVALID, GOAL_INVALID])
    status, scenario, _ = make_scenario(1_000_003, CFG, stratum="narrow", validity=script)
    assert status == SCENARIO_OK and scenario is not None and base is not None
    assert scenario.validity_rejections == {START_INVALID: 1, GOAL_INVALID: 1, RESIDUAL: 1}
    assert scenario.goal_resamples >= 3      # plus any re-drawn goal that was unusable
    assert set(script.starts) == {base.start[:2]}, "the start is kept; only the goal moves"
    assert scenario.start[:2] == base.start[:2]
    assert scenario.layout_sha256 == base.layout_sha256
    assert scenario.goal != base.goal


def test_a_scenario_that_never_passes_is_a_named_failure_with_its_rejections():
    status, scenario, layout = make_scenario(1_000_003, CFG, stratum="narrow", attempts=2,
                                             validity=_Scripted([RESIDUAL] * 1000))
    assert status == NO_VALID_ROUTE and scenario is None
    logged = layout.metadata["validity_rejections"]
    assert logged[START_INVALID] == 0 and logged[GOAL_INVALID] == 0
    limit = int(CFG["world"]["layout"]["validity"]["goal_resamples"])
    assert 2 <= logged[RESIDUAL] <= 2 * (limit + 1)


def test_a_set_logs_validity_rejections_per_stratum_with_every_cause_present():
    manifest = _shared_set()
    tally = manifest["validity_rejections_by_stratum"]
    assert tally
    for counts in tally.values():
        assert set(counts) == {*VALIDITY_FAILURES, "goal_resamples", "built", "failed_seeds"}
    assert sum(c["built"] for c in tally.values()) == manifest["built"]
    assert sum(c["failed_seeds"] for c in tally.values()) == len(manifest["failures"])


def test_a_parallel_build_is_the_serial_build():
    """Three seeds on two workers, so one process builds two scenarios in turn and anything
    carried between them (the field-shape cache) is exercised."""
    seeds = [1_000_003, 1_000_009, 1_000_080]
    serial = build_set(seeds, CFG, "train", episode_rate=0.0)
    parallel = build_set(seeds, CFG, "train", episode_rate=0.0, workers=2)
    assert serial["built"] == 3
    assert parallel == serial


def test_the_scenarios_hash_covers_the_modules_the_generator_imports():
    """It hashed scenarios.py alone, so a change to the oracle (or to A13's placement and bypass)
    could move every route without moving the identity a set is verified against."""
    from pfbnav.envs.freeze import import_closure

    names = {p.name for p in import_closure("pfbnav.world.scenarios", REPO)}
    assert {"scenarios.py", "oracle.py", "route_check.py", "placement.py",
            "bypass.py", "astar.py"} <= names


# ----------------------------------------------------------------------
# The preregistered rate
# ----------------------------------------------------------------------
def test_episode_unmapped_count_honours_the_rate():
    rng = np.random.default_rng(0)
    draws = [episode_unmapped_count(rng, 0.25, (1, 2)) for _ in range(4000)]
    with_any = sum(1 for d in draws if d > 0)
    assert 0.22 < with_any / len(draws) < 0.28
    assert set(draws) <= {0, 1, 2}


def test_a_rate_of_zero_and_one_are_exact():
    rng = np.random.default_rng(0)
    assert all(episode_unmapped_count(rng, 0.0, (1, 2)) == 0 for _ in range(50))
    assert all(episode_unmapped_count(rng, 1.0, (1, 2)) > 0 for _ in range(50))


def test_a_rate_outside_zero_to_one_is_refused():
    rng = np.random.default_rng(0)
    with pytest.raises(ValueError):
        episode_unmapped_count(rng, 1.5, (1, 2))


# ----------------------------------------------------------------------
# Sets: manifest, lock, verification
# ----------------------------------------------------------------------
def test_a_set_records_the_rate_it_achieved_not_only_the_one_requested():
    manifest = _shared_set()
    assert manifest["built"] + len(manifest["failures"]) == 3
    assert manifest["episode_rate_requested"] == 0.5
    achieved = manifest["episode_rate_achieved"]
    assert 0.0 <= achieved <= 1.0
    counted = sum(1 for s in manifest["scenarios"] if s["unmapped_count"] > 0)
    assert achieved == pytest.approx(counted / manifest["built"])


def test_a_set_carries_the_module_hashes(tmp_path):
    manifest = _shared_set()
    assert manifest["scenarios_sha256"] == scenarios_sha256()
    for entry in manifest["scenarios"]:
        assert entry["generator_sha256"] == manifest["generator_sha256"]


def test_a_locked_set_cannot_be_overwritten(tmp_path):
    manifest = _shared_set()
    path = tmp_path / "test_set.json"
    write_set(path, manifest, lock=True)
    assert load_set(path)["locked"] is True
    with pytest.raises(PermissionError, match="locked"):
        write_set(path, manifest)


def test_an_unlocked_set_may_be_rewritten(tmp_path):
    manifest = _shared_set()
    path = tmp_path / "tune_set.json"
    write_set(path, manifest)
    write_set(path, manifest)          # tune layouts are disposable (2026-09-23)
    assert load_set(path)["built"] == manifest["built"]


def test_verification_passes_on_a_set_this_generator_produced(tmp_path):
    manifest = _shared_set()
    report = verify(manifest, CFG)
    assert report["ok"], report["problems"]
    assert report["checked"] == manifest["built"]


def test_verification_catches_a_world_that_moved_under_a_claimed_identity(tmp_path):
    manifest = _shared_set()
    manifest["scenarios"][0]["layout_sha256"] = "0" * 64
    report = verify(manifest, CFG)
    assert not report["ok"]
    assert any(p["field"] == "layout_sha256" for p in report["problems"])


def test_verification_catches_a_changed_generator():
    manifest = _shared_set()
    manifest["scenarios_sha256"] = "f" * 64
    report = verify(manifest, CFG)
    assert not report["ok"]
    assert any(p.get("field") == "scenarios_sha256" for p in report["problems"])


def test_a_set_round_trips_through_json(tmp_path):
    manifest = _shared_set()
    path = write_set(tmp_path / "set.json", manifest)
    again = load_set(path)
    assert again["scenarios"] == json.loads(json.dumps(manifest["scenarios"], default=float))
    assert Scenario(**again["scenarios"][0]).seed == manifest["scenarios"][0]["seed"]


def test_an_empty_set_reports_no_achieved_rate_rather_than_zero():
    manifest = build_set([], CFG, "train", episode_rate=0.25)
    assert manifest["built"] == 0
    assert manifest["episode_rate_achieved"] is None, "0/0 is not a rate"


def test_rectangle_helper_is_counter_clockwise():
    poly = rectangle(0.0, 0.0, 2.0, 1.0)
    # Shoelace: positive area means counter-clockwise, which the polygon tests assume.
    x, y = poly[:, 0], poly[:, 1]
    area = 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
    assert area > 0
