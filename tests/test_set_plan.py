"""Known-answer tests for scenario-set planning (hard rule 6). Pure; nothing is generated."""
import pytest

from pfbnav.world.set_plan import (
    ENGAGED_OFFSET_M,
    SetRequest,
    allocate_blocks,
    engagement,
)

RANGES = {"tune": (2_000_000, 2_999_999), "test": (3_000_000, 3_999_999),
          "pilot": (9_000_000, 9_999_999)}


def req(name, split="test", size=10, **kw):
    return SetRequest(name=name, split=split, stratum=kw.get("stratum"),
                      episode_rate=kw.get("rate", 0.0), size=size, lock=kw.get("lock", False))


def test_blocks_are_contiguous_from_the_range_start_and_sized_by_headroom():
    blocks = allocate_blocks(RANGES, [req("a", size=10), req("b", size=5)], headroom=4)
    assert blocks["a"] == (3_000_000, 3_000_039)
    assert blocks["b"] == (3_000_040, 3_000_059)


def test_no_two_sets_share_a_seed_even_across_splits():
    requests = [req("t1", split="tune", size=50), req("x1", size=30), req("x2", size=30),
                req("p1", split="pilot", size=30)]
    blocks = allocate_blocks(RANGES, requests, headroom=10)
    seeds = [s for lo, hi in blocks.values() for s in range(lo, hi + 1)]
    assert len(seeds) == len(set(seeds))


def test_every_block_stays_inside_its_split():
    requests = [req("t1", split="tune", size=50), req("x1", size=30),
                req("p1", split="pilot", size=30)]
    blocks = allocate_blocks(RANGES, requests, headroom=10)
    for r in requests:
        lo, hi = blocks[r.name]
        assert RANGES[r.split][0] <= lo <= hi <= RANGES[r.split][1]


def test_the_same_plan_gives_the_same_seeds():
    requests = [req("a"), req("b", split="pilot")]
    assert allocate_blocks(RANGES, requests, 5) == allocate_blocks(RANGES, requests, 5)


def test_an_overflowing_plan_is_refused_not_shortened():
    with pytest.raises(ValueError, match="no room"):
        allocate_blocks({"test": (0, 99)}, [req("a", size=30)], headroom=4)


def test_an_unknown_split_and_a_duplicate_name_are_refused():
    with pytest.raises(ValueError, match="no preregistered seed range"):
        allocate_blocks(RANGES, [req("a", split="train")], headroom=2)
    with pytest.raises(ValueError, match="duplicate"):
        allocate_blocks(RANGES, [req("a"), req("a")], headroom=2)


def test_a_set_request_refuses_impossible_values():
    with pytest.raises(ValueError, match="positive size"):
        req("a", size=0)
    with pytest.raises(ValueError, match="outside"):
        req("a", rate=1.5)


def test_engagement_known_answer():
    # 5 scenarios: one without obstacles; of the four with, two within the threshold.
    got = engagement([None, 0.10, ENGAGED_OFFSET_M, 0.31, 2.5])
    assert got["scenarios"] == 5 and got["with_unmapped"] == 4 and got["engaged"] == 2
    assert got["engaged_share"] == pytest.approx(0.5)


def test_engagement_with_no_obstacles_is_none_not_zero():
    """A mapped_only set has no unmapped obstacles; 0/0 is not "never engaged"."""
    assert engagement([None, None])["engaged_share"] is None


# ---------------------------------------------------------- the build script's plan (pure parts)
def _script():
    import importlib.util
    from pathlib import Path
    path = Path(__file__).resolve().parents[1] / "scripts" / "build_scenario_sets.py"
    spec = importlib.util.spec_from_file_location("build_sets", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RATES = {"episode_rate": {"train": 0.25, "tune": 0.25,
                          "test": {"mapped_only": 0.0, "unmapped_static": 1.0}}}
SIZES = {"tune": {"per_stratum": 20}, "pilot": {"size": 30, "condition": "unmapped_static"},
         "test": {"per_cell": 100}}


def test_the_plan_refuses_without_preregistered_sizes():
    """A test set's size fixes the power of H1 and H2; the script may not choose it."""
    with pytest.raises(SystemExit, match="not preregistered"):
        _script().plan({"layout_sets": {"unmapped": RATES}})


def test_the_plan_is_ten_sets_with_rates_read_not_restated():
    plan = _script().plan({"layout_sets": {"unmapped": RATES}, "scenario_sets": SIZES})
    by_name = {r.name: r for r in plan}
    assert sorted(by_name) == sorted(
        ["tune_narrow", "tune_medium", "tune_wide", "pilot_narrow"]
        + [f"test_{c}_{s}" for c in ("mapped_only", "unmapped_static")
           for s in ("narrow", "medium", "wide")])
    assert by_name["test_unmapped_static_wide"].episode_rate == 1.0
    assert by_name["test_mapped_only_narrow"].episode_rate == 0.0
    assert by_name["tune_medium"].episode_rate == 0.25
    assert by_name["pilot_narrow"].size == 30 and by_name["pilot_narrow"].episode_rate == 1.0
    # Only the test sets are locked; tune is disposable and the pilot is development.
    assert {r.name for r in plan if r.lock} == {n for n in by_name if n.startswith("test_")}


def test_the_whole_plan_fits_the_preregistered_ranges():
    plan = _script().plan({"layout_sets": {"unmapped": RATES}, "scenario_sets": SIZES})
    blocks = allocate_blocks(RANGES, plan, headroom=1)
    seeds = [s for lo, hi in blocks.values() for s in range(lo, hi + 1)]
    assert len(seeds) == len(set(seeds)) == 3 * 20 + 30 + 6 * 100


def test_the_pilot_assignment_gives_each_seed_thirty_distinct_scenarios():
    """A13: 150 distinct scenarios; learned seed i mod 5, so 30 per seed and no scenario run
    twice by one method -- the independence the Clopper-Pearson bound assumes."""
    from collections import Counter

    from pfbnav.world.set_plan import learned_seed_for
    per_seed = Counter(learned_seed_for(i) for i in range(150))
    assert per_seed == {s: 30 for s in range(5)}
    assert [learned_seed_for(i) for i in range(7)] == [0, 1, 2, 3, 4, 0, 1]
    with pytest.raises(ValueError):
        learned_seed_for(-1)


PLACEMENT = {"offset_to_nearest_surface_m": [0.0, 0.6], "first_obstacle_offset_max_m": 0.3,
             "arc_excluded_from_start_m": 2.0, "arc_excluded_to_goal_m": 1.0,
             "attempts_per_obstacle": 200}


def test_the_config_matches_the_placement_it_would_preregister():
    from pathlib import Path

    from pfbnav.config import load_config
    from pfbnav.world.set_plan import placement_mismatches
    cfg = load_config(Path(__file__).resolve().parents[1] / "configs" / "base.yaml")
    assert placement_mismatches(cfg, PLACEMENT) == []
    drifted = {**PLACEMENT, "first_obstacle_offset_max_m": 0.6}
    assert placement_mismatches(cfg, drifted) == [
        "first_offset_max: config 0.3, preregistered 0.6"]


def test_the_rejection_summary_known_answer():
    from pfbnav.world.set_plan import rejection_summary
    scenarios = [{"placement_attempts": 10, "placement_rejections": {"a": 6, "b": 2}},
                 {"placement_attempts": 5, "placement_rejections": {"a": 1}},
                 {"placement_attempts": 0, "placement_rejections": {}}]
    failures = [{"status": "lost"}, {"status": "other"}]
    got = rejection_summary(scenarios, failures, "lost")
    assert got["attempts"] == 15 and got["rejected"] == 9 and got["by_cause"] == {"a": 7, "b": 2}
    assert got["rejection_rate"] == pytest.approx(0.6)
    assert got["scenarios_lost_to_placement"] == 1
    assert rejection_summary([], [], "lost")["rejection_rate"] is None


def test_obstacle_location_known_answers():
    from pfbnav.world.set_plan import (
        AISLE,
        AISLE_MOUTH,
        OPEN_FLOOR,
        location_of,
        location_summary,
    )
    aisles = [(2.0, 3.0, 10.0, 3.8)]
    assert location_of((5.0, 3.4), aisles, 1.2) == AISLE
    assert location_of((2.0, 3.0), aisles, 1.2) == AISLE               # the boundary is inside
    assert location_of((1.0, 3.4), aisles, 1.2) == AISLE_MOUTH
    assert location_of((11.2, 3.8), aisles, 1.2) == AISLE_MOUTH
    assert location_of((0.7, 3.4), aisles, 1.2) == OPEN_FLOOR         # beyond the mouth
    assert location_of((5.0, 4.0), aisles, 1.2) == OPEN_FLOOR         # outside the band
    got = location_summary([AISLE, OPEN_FLOOR, OPEN_FLOOR, AISLE_MOUTH])
    assert got["counts"] == {AISLE: 1, AISLE_MOUTH: 1, OPEN_FLOOR: 2}
    assert got["shares"][OPEN_FLOOR] == pytest.approx(0.5)
    assert location_summary([])["shares"][AISLE] is None


def test_the_preregistered_corridor_is_checked_once_it_exists():
    from pathlib import Path

    from pfbnav.config import load_config
    from pfbnav.world.set_plan import placement_mismatches
    cfg = load_config(Path(__file__).resolve().parents[1] / "configs" / "base.yaml")
    assert placement_mismatches(cfg, {**PLACEMENT, "bypass": {"corridor_half_width_m": 1.0}}) == []
    assert placement_mismatches(cfg, {**PLACEMENT, "bypass": {"corridor_half_width_m": 1.5}}) == [
        "bypass corridor half-width W: config 1.0, preregistered 1.5"]


def test_position_by_index_known_answer():
    from pfbnav.world.set_plan import position_by_index
    scenarios = [{"path_length_m": 10.0, "unmapped_arcs_m": [2.0, 8.0]},
                 {"path_length_m": 20.0, "unmapped_arcs_m": [10.0]},
                 {"path_length_m": 5.0, "unmapped_arcs_m": []},
                 {"path_length_m": 0.0, "unmapped_arcs_m": [1.0]}]
    got = position_by_index(scenarios)
    assert got["capped"] == {"n": 2, "mean_arc_fraction": pytest.approx(0.35)}
    assert got["others"] == {"n": 1, "mean_arc_fraction": pytest.approx(0.8)}
    assert position_by_index([])["capped"]["mean_arc_fraction"] is None


def test_until_a14_only_tune_and_the_150_scenario_pilot_are_planned():
    """A13 preregisters tune and pilot; the test size is A14, after G2 and the power analysis."""
    a13 = {"tune": {"per_stratum": 20},
           "pilot": {"size": 150, "condition": "unmapped_static"}, "test": "pending_A14"}
    plan = _script().plan({"layout_sets": {"unmapped": RATES}, "scenario_sets": a13})
    assert sorted(r.name for r in plan) == ["pilot_narrow", "tune_medium", "tune_narrow",
                                            "tune_wide"]
    pilot = next(r for r in plan if r.name == "pilot_narrow")
    assert pilot.size == 150 and pilot.episode_rate == 1.0 and not pilot.lock


def test_the_training_set_is_planned_in_thirds_once_preregistered():
    """Ruled 2026-10-03: equal thirds by stratum, ~2,000 scenarios, the train seed range and rate.
    Its blocks come after nothing else in `train`, so adding it moves no other set's seeds."""
    sizes = {"tune": {"per_stratum": 20}, "pilot": {"size": 150, "condition": "unmapped_static"},
             "train": {"per_stratum": 667}, "test": "pending_A14"}
    plan = _script().plan({"layout_sets": {"unmapped": RATES}, "scenario_sets": sizes})
    train = [r for r in plan if r.split == "train"]
    assert [r.name for r in train] == ["train_narrow", "train_medium", "train_wide"]
    assert all(r.size == 667 and r.episode_rate == 0.25 and not r.lock for r in train)
    ranges = {**RANGES, "train": (1_000_000, 1_999_999)}
    blocks = allocate_blocks(ranges, plan, headroom=1)
    assert blocks["train_narrow"] == (1_000_000, 1_000_666)
    assert blocks["train_wide"] == (1_001_334, 1_002_000)
    without = allocate_blocks(ranges, [r for r in plan if r.split != "train"], headroom=1)
    assert all(blocks[k] == v for k, v in without.items())


def test_the_tune_unmapped_set_follows_the_tune_sets_and_moves_none_of_them():
    """Step 0 (ruled 2026-10-03): 20 per stratum, tune seeds, rate 1.0 -- the H1 population's."""
    base = {"tune": {"per_stratum": 20}, "pilot": {"size": 150, "condition": "unmapped_static"}}
    with_set = {**base, "tune_unmapped": {"per_stratum": 20}}
    plan = _script().plan({"layout_sets": {"unmapped": RATES}, "scenario_sets": with_set})
    extra = [r for r in plan if r.name.startswith("tune_unmapped_")]
    assert [r.size for r in extra] == [20, 20, 20] and all(r.episode_rate == 1.0 for r in extra)
    blocks = allocate_blocks(RANGES, plan, headroom=1)
    before = allocate_blocks(RANGES, _script().plan(
        {"layout_sets": {"unmapped": RATES}, "scenario_sets": base}), headroom=1)
    assert all(blocks[k] == v for k, v in before.items())
    assert blocks["tune_unmapped_narrow"][0] == blocks["tune_wide"][1] + 1

