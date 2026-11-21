"""The validation set's seed rules and truncation: known answers, plus a small real build (hard rule 6)."""
import json
from pathlib import Path

import pytest

from pfbnav.config import load_config
from pfbnav.world.scenarios import build_set, verify
from pfbnav.world.validation_set import (
    BLOCK,
    CELLS,
    FIRST_SEED,
    TUNE_RANGE,
    cell_name,
    cell_seeds,
    overlaps,
    truncate,
    used_seeds,
)

ROOT = Path(__file__).resolve().parents[1]


def test_cells_are_the_test_sets_six_cells():
    assert len(CELLS) == 6
    assert {(s, c) for s, c, _ in CELLS} == {(s, c) for s in ("narrow", "medium", "wide")
                                             for c in ("mapped_only", "unmapped")}
    assert {c: r for _, c, r in CELLS} == {"mapped_only": 0.0, "unmapped": 1.0}
    assert cell_name("narrow", "unmapped") == "validation_unmapped_narrow"


def test_blocks_are_disjoint_and_inside_the_tune_range():
    ranges = [cell_seeds(i, 30, 15) for i in range(len(CELLS))]
    flat = [s for r in ranges for s in r]
    assert len(flat) == len(set(flat))
    assert min(flat) >= TUNE_RANGE[0] and max(flat) <= TUNE_RANGE[1]
    assert ranges[1][0] - ranges[0][0] == BLOCK and ranges[0][0] == FIRST_SEED
    with pytest.raises(ValueError):
        cell_seeds(0, BLOCK, 1)


def test_no_validation_seed_is_used_by_any_existing_set():
    used = used_seeds(ROOT / "scenarios")
    assert used, "the existing sets must be found, or this check is empty"
    for i in range(len(CELLS)):
        assert overlaps(cell_seeds(i, 30, 15), used) == {}


def test_overlap_detection_finds_a_planted_collision():
    """Positive control: a seed that IS in another set is reported."""
    assert overlaps(range(10, 13), {11: "tune/x.json", 99: "y.json"}) == {11: "tune/x.json"}


def test_truncate_keeps_the_first_built_scenarios_in_seed_order_and_recounts_the_rate():
    manifest = {"scenarios": [{"seed": 5, "unmapped_count": 1}, {"seed": 3, "unmapped_count": 0},
                              {"seed": 4, "unmapped_count": 2}],
                "built": 3, "requested": 9}
    out = truncate(manifest, 2)
    assert [s["seed"] for s in out["scenarios"]] == [3, 4]
    assert out["built"] == out["requested"] == 2 and out["episode_rate_achieved"] == 0.5
    assert out["kind"] == "validation" and len(manifest["scenarios"]) == 3
    with pytest.raises(ValueError, match="only 3"):
        truncate(manifest, 4)


def test_a_small_real_build_verifies_and_its_seeds_are_the_blocks_first_built():
    cfg = load_config(ROOT / "configs" / "base.yaml")
    seeds = cell_seeds(2, 2, 3)                      # wide, mapped only
    manifest = truncate(build_set(seeds, cfg, "tune", episode_rate=0.0, stratum="wide"), 2)
    assert manifest["built"] == 2 and verify(manifest, cfg)["ok"]
    assert all(s["unmapped_count"] == 0 and s["stratum"] == "wide" for s in manifest["scenarios"])
    assert all(s["seed"] in seeds for s in manifest["scenarios"])
    json.dumps(manifest, default=float)
