"""Known-answer tests for the layout generator (hard rule 6). Pure; no simulator.

A generated layout is the world an experiment is run in, so three things must hold before
any result rests on one: the same seed and generator give the same world; the aisles match
the stratum the layout claims; and the splits cannot overlap, by seed or by geometry
(supervisor, 2026-09-23).
"""
import itertools
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.world.baking import Obstacle, bake, planner_radii, polygon_distance
from pfbnav.world.layouts import (
    aisle_width,
    distance_to_path,
    generate,
    generator_sha256,
    layout_sha256,
    perimeter,
    ranges_are_disjoint,
    rectangle,
    split_of_seed,
    stratum_of,
)
from pfbnav.world.strata import strata_from_config

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
HALF_WIDTH = float(CFG["robot"]["half_width"])
STRATA = strata_from_config(CFG)
# The ranges proposed to preregistration (2026-09-23); the test uses them as data, and
# `test_the_preregistered_ranges_are_disjoint` checks the committed ones once they land.
RANGES = {"train": (1_000_000, 1_999_999), "tune": (2_000_000, 2_999_999),
          "test": (3_000_000, 3_999_999)}


# ----------------------------------------------------------------------
# Geometry from the strata
# ----------------------------------------------------------------------
def test_aisle_width_is_the_d2_definition():
    assert aisle_width(0.2086, 0.12) == pytest.approx(0.6572)
    assert aisle_width(0.2086, 0.40) == pytest.approx(1.2172)


def test_stratum_of_known_answers():
    # A21 (2026-10-19): narrow [0.1987, 0.2687], medium [0.2687, 0.40], wide [0.40, 0.80]
    assert stratum_of(0.1987, STRATA) == "narrow"
    assert stratum_of(0.24, STRATA) == "narrow"
    assert stratum_of(0.2687, STRATA) == "medium"     # a band owns its lower edge
    assert stratum_of(0.40, STRATA) == "wide"
    assert stratum_of(0.80, STRATA) == "wide"         # the widest owns its top edge
    with pytest.raises(ValueError):
        stratum_of(0.19, STRATA)                      # below the narrow floor
    with pytest.raises(ValueError):
        stratum_of(1.5, STRATA)


@pytest.mark.parametrize("name", [s.name for s in STRATA])
def test_a_layout_aisle_matches_the_stratum_it_claims(name):
    layout = generate(1_000_010, CFG, stratum=name)
    band = next(s for s in STRATA if s.name == name)
    clearance = layout.metadata["clearance_m"]
    assert band.min_clearance <= clearance < band.max_clearance
    assert layout.metadata["stratum"] == name
    assert layout.metadata["aisle_width_m"] == pytest.approx(
        aisle_width(HALF_WIDTH, clearance))


def test_the_measured_gap_between_rows_is_the_claimed_aisle():
    """Not the claim in the metadata: the distance between the shelves as generated."""
    layout = generate(1_000_011, CFG, stratum="medium")
    shelves = [o for o in layout.mapped if o.name.startswith("shelf_")]
    if len(shelves) < 2:
        pytest.skip("this seed produced a single row")
    gaps = []
    for lower, upper in itertools.pairwise(shelves):
        gaps.append(float(upper.polygon[:, 1].min() - lower.polygon[:, 1].max()))
    assert gaps == pytest.approx([layout.metadata["aisle_width_m"]] * len(gaps), abs=1e-9)


def test_the_robot_fits_every_aisle_it_generates():
    for seed in range(1_000_020, 1_000_040):
        layout = generate(seed, CFG)
        assert layout.metadata["aisle_width_m"] > 2 * HALF_WIDTH


def test_shelves_stay_inside_the_walls():
    layout = generate(1_000_041, CFG)
    inner = layout.free_space()
    for shelf in (o for o in layout.mapped if o.name.startswith("shelf_")):
        assert shelf.polygon[:, 0].min() >= inner[0] - 1e-9
        assert shelf.polygon[:, 0].max() <= inner[2] + 1e-9
        assert shelf.polygon[:, 1].min() >= inner[1] - 1e-9
        assert shelf.polygon[:, 1].max() <= inner[3] + 1e-9


def test_the_perimeter_is_four_walls_the_planner_knows_about():
    walls = perimeter((0.0, 0.0, 10.0, 6.0), 0.2)
    assert len(walls) == 4 and all(w.mapped for w in walls)
    assert {w.name for w in walls} == {"wall_south", "wall_north", "wall_west", "wall_east"}


# ----------------------------------------------------------------------
# Identity
# ----------------------------------------------------------------------
def test_the_same_seed_gives_the_same_world():
    a = generate(1_000_050, CFG, unmapped_count=2)
    b = generate(1_000_050, CFG, unmapped_count=2)
    assert a.metadata["layout_sha256"] == b.metadata["layout_sha256"]
    for x, y in zip(a.obstacles, b.obstacles, strict=True):
        assert np.array_equal(x.polygon, y.polygon) and x.name == y.name


def test_different_seeds_give_different_worlds():
    hashes = {generate(seed, CFG).metadata["layout_sha256"]
              for seed in range(1_000_060, 1_000_080)}
    assert len(hashes) > 15, "the generator is barely varying with the seed"


def test_the_layout_hash_is_over_the_geometry_not_the_object():
    layout = generate(1_000_081, CFG, unmapped_count=1)
    rebuilt = [Obstacle(np.array(o.polygon, copy=True), o.mapped, o.name)
               for o in layout.obstacles]
    assert layout_sha256(rebuilt, layout.bounds) == layout.metadata["layout_sha256"]


def test_the_layout_hash_moves_when_the_geometry_moves():
    layout = generate(1_000_082, CFG)
    nudged = list(layout.obstacles)
    nudged[-1] = Obstacle(nudged[-1].polygon + np.array([0.01, 0.0]), True, nudged[-1].name)
    assert layout_sha256(nudged, layout.bounds) != layout.metadata["layout_sha256"]


def test_the_mapped_flag_is_part_of_the_identity():
    layout = generate(1_000_083, CFG, unmapped_count=1)
    flipped = [Obstacle(o.polygon, True, o.name) for o in layout.obstacles]
    assert layout_sha256(flipped, layout.bounds) != layout.metadata["layout_sha256"]


def test_the_generator_hash_travels_with_every_layout():
    layout = generate(1_000_084, CFG)
    assert layout.metadata["generator_sha256"] == generator_sha256()
    assert len(layout.metadata["generator_sha256"]) == 64


# ----------------------------------------------------------------------
# Splits
# ----------------------------------------------------------------------
def test_the_proposed_seed_ranges_are_disjoint():
    assert ranges_are_disjoint(RANGES)
    assert not ranges_are_disjoint({"a": (0, 10), "b": (5, 20)})


def test_split_of_seed_names_exactly_one_split():
    assert split_of_seed(1_500_000, RANGES) == "train"
    assert split_of_seed(2_000_000, RANGES) == "tune"
    assert split_of_seed(3_999_999, RANGES) == "test"
    with pytest.raises(ValueError):
        split_of_seed(500, RANGES)
    with pytest.raises(ValueError):
        split_of_seed(7, {"a": (0, 10), "b": (5, 20)})


def test_layouts_from_different_splits_are_geometrically_disjoint():
    """Not just the seeds: the worlds themselves must not coincide."""
    by_split = {name: {generate(seed, CFG, split=name).metadata["layout_sha256"]
                       for seed in range(lo, lo + 15)}
                for name, (lo, _hi) in RANGES.items()}
    assert not (by_split["train"] & by_split["tune"])
    assert not (by_split["train"] & by_split["test"])
    assert not (by_split["tune"] & by_split["test"])


def test_the_preregistered_ranges_are_disjoint_once_they_are_committed():
    import yaml
    prereg = REPO / "preregistration" / "gates.yaml"
    data = yaml.safe_load(prereg.read_text()) or {}
    sets = data.get("layout_sets")
    if not sets or "seed_ranges" not in sets:
        pytest.skip("layout_sets.seed_ranges is not preregistered yet (WP1 open item)")
    ranges = {k: tuple(v) for k, v in sets["seed_ranges"].items()}
    assert ranges_are_disjoint(ranges)


# ----------------------------------------------------------------------
# Unmapped obstacles
# ----------------------------------------------------------------------
def test_unmapped_obstacles_are_placed_clear_of_everything_mapped():
    layout = generate(1_000_090, CFG, unmapped_count=2)
    margin = float(CFG["world"]["layout"]["unmapped"]["margin"])
    assert len(layout.unmapped) == 2
    for hidden in layout.unmapped:
        centre = hidden.polygon.mean(axis=0)[None, :]
        for known in layout.mapped:
            assert float(polygon_distance(centre, known.polygon)[0]) >= margin


def test_none_are_placed_when_none_are_asked_for():
    layout = generate(1_000_091, CFG, unmapped_count=0)
    assert layout.unmapped == [] and layout.metadata["unmapped_count"] == 0


def test_an_unmapped_obstacle_is_absent_from_the_baked_map():
    """The property the whole design rests on, end to end through the bake."""
    layout = generate(1_000_092, CFG, unmapped_count=1)
    radii = planner_radii(CFG, circumscribed=0.33742)
    baked = bake(layout.obstacles, layout.bounds, 0.05, radii)
    hidden = layout.unmapped[0]
    centre = hidden.polygon.mean(axis=0)
    row, col = baked.cell_of([centre[0]], [centre[1]])
    assert not baked.occupied[row, col][0], "the planner was told about it after all"
    assert baked.unmapped == [hidden.name]


def test_distance_to_path_known_answers():
    path = np.array([[0.0, 0.0], [10.0, 0.0]])
    near = Obstacle(rectangle(4.0, 0.5, 4.4, 0.9), mapped=False, name="near")
    far = Obstacle(rectangle(4.0, 3.0, 4.4, 3.4), mapped=False, name="far")
    on_path = Obstacle(rectangle(4.0, -0.2, 4.4, 0.2), mapped=False, name="on")
    assert distance_to_path(near, path) == pytest.approx(0.5)
    assert distance_to_path(far, path) == pytest.approx(3.0)
    assert distance_to_path(on_path, path) == pytest.approx(0.0)


def test_a_hall_too_shallow_for_its_aisles_is_an_error_not_a_silent_layout():
    import copy
    cfg = copy.deepcopy(CFG)
    cfg["world"]["layout"]["bounds"] = [0.0, 0.0, 14.0, 1.5]
    with pytest.raises(ValueError, match="no shelf row fits"):
        generate(1_000_093, cfg, stratum="wide")
