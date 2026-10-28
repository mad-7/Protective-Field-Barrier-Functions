"""The narrowest aisles plan at every sub-cell position (supervisor, 2026-10-16: permanent).

The lethal margin 0.10 sat at the edge of plannability while the narrow floor was 0.13: a centreline band
of 2 x (0.13 - 0.10) = 0.06 m between the lethal inflations, against a 0.05 m grid cell. Since A20
(2026-10-18) the floor is 0.1735 and the band 0.147 m, so the edge (band = one cell) is at a lethal
margin of F - 0.025; the positive control below sits just past THAT edge. These tests
pin that the configured planner still plans there, densely across the cell (hard rule 14). The
positive control (hard rule 15) is the known failure rate just past the edge: a straight aisle plans
only where a cell centre falls inside the band, so the share of sub-cell positions that fail is
1 - band / cell.
"""
import copy
from pathlib import Path

import pytest

from pfbnav.config import load_config
from pfbnav.planning.plannability import aisle_plannability
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.world.strata import strata_from_config

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
CIRCUMSCRIBED = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"]).circumscribed
OFFSETS = 20            # sub-cell positions per axis: every 2.5 mm across a 0.05 m cell
LENGTH = 6.0            # m


def _narrow_floor() -> float:
    return min(s.min_clearance for s in strata_from_config(CFG))


def _with_lethal(margin: float) -> dict:
    """The planner at another lethal margin. Since A21 (F = 0.1987) a lethal margin past the edge
    puts the lethal radius beyond the configured soft radius, which the planner refuses; the soft
    margin is raised with it here (it only adds cost, never forbids a cell), keeping the soft
    radius 0.03 m beyond the lethal one."""
    cfg = copy.deepcopy(CFG)
    cfg["planner"]["lethal_margin"] = margin
    lethal = float(cfg["robot"]["half_width"]) + margin
    soft = CIRCUMSCRIBED + float(cfg["planner"]["soft_margin"])
    if soft <= lethal:
        cfg["planner"]["soft_margin"] = lethal + 0.03 - CIRCUMSCRIBED
    return cfg


def test_every_stratum_plans_at_every_sub_cell_position_under_the_configured_margins():
    for entry in aisle_plannability(CFG, CIRCUMSCRIBED, offsets=OFFSETS, aisle_length=LENGTH):
        assert entry["positions"] == 2 * OFFSETS
        assert entry["failures"] == [], (entry["stratum"], entry["failures"])


def test_the_configured_margin_leaves_the_narrow_centreline_band_wider_than_a_cell():
    band = 2.0 * (_narrow_floor() - float(CFG["planner"]["lethal_margin"]))
    assert band > float(CFG["planner"]["resolution"]), band


@pytest.mark.parametrize("past_edge", [0.0025, 0.005, 0.0075, 0.01])
def test_past_the_edge_the_narrow_aisle_fails_at_the_known_share_of_positions(past_edge):
    """The positive control: the check can fail, and fails where the geometry says it must. The
    lethal margin is set `past_edge` beyond the edge, F - cell / 2 (A20: relative to the floor, not
    to the old 0.13)."""
    res = float(CFG["planner"]["resolution"])
    margin = _narrow_floor() - res / 2.0 + past_edge
    band = 2.0 * (_narrow_floor() - margin)
    expected = 2 * OFFSETS * max(0.0, 1.0 - band / res)
    narrow = next(e for e in aisle_plannability(_with_lethal(margin), CIRCUMSCRIBED,
                                                offsets=OFFSETS, aisle_length=LENGTH)
                  if e["min_clearance_m"] == _narrow_floor())
    # A position exactly on the boundary may go either way in floating point: one per axis.
    assert abs(len(narrow["failures"]) - expected) <= 2, (margin, len(narrow["failures"]), expected)
    if past_edge >= 0.005:
        assert narrow["failures"], "past the edge the check must fail somewhere"
