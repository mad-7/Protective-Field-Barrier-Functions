"""Known-answer tests for the 82-D observation (hard rule 6 and hard rule 7).

The layout was ruled on 2026-09-23. These tests pin it: the channel count, the order, the
scaling, and above all that validity is its own block rather than a sentinel range -- the
distinction hard rule 5 exists to preserve.
"""
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.envs.observation import (
    BLOCKS,
    FULL_DIM,
    NO_PLANNER_DIM,
    DropoutSpec,
    ObservationSpec,
    assemble,
    bounds,
    dropout,
    sample_blocked_arc,
)

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
SPEC = ObservationSpec.from_config(CFG)


def obs(**over):
    base = {"ranges": np.full(36, 2.5), "valid": np.ones(36, dtype=bool),
            "velocity": [0.0, 0.0, 0.0], "lookahead_body": (0.8, 0.0, 1.0),
            "goal_body": (9.0, 0.0, 1.0), "stop_active": False}
    return assemble(SPEC, **{**base, **over})


def test_the_layout_is_the_ruled_one():
    assert FULL_DIM == 82 and NO_PLANNER_DIM == 79
    assert [(name, width) for name, width in BLOCKS] == [
        ("scan", 36), ("validity", 36), ("velocity", 3), ("lookahead", 3), ("goal", 3),
        ("certified", 1)]


def test_the_blocks_sit_where_the_slices_say():
    s = SPEC.slices
    assert (s["scan"].start, s["scan"].stop) == (0, 36)
    assert (s["validity"].start, s["validity"].stop) == (36, 72)
    assert (s["velocity"].start, s["velocity"].stop) == (72, 75)
    assert (s["lookahead"].start, s["lookahead"].stop) == (75, 78)
    assert (s["goal"].start, s["goal"].stop) == (78, 81)
    assert (s["certified"].start, s["certified"].stop) == (81, 82)


def test_the_no_planner_ablation_drops_only_the_lookahead():
    spec = ObservationSpec.from_config(CFG, include_lookahead=False)
    assert spec.dim == 79 and "lookahead" not in spec.slices
    assert (spec.slices["goal"].start, spec.slices["goal"].stop) == (75, 78)


def test_ranges_are_scaled_by_range_max_and_clipped():
    o = obs(ranges=np.array([0.0, 2.5, 5.0, 7.0] + [1.0] * 32))
    assert o[0] == pytest.approx(0.0)
    assert o[1] == pytest.approx(0.5)
    assert o[2] == pytest.approx(1.0)
    assert o[3] == pytest.approx(1.0), "a range beyond range_max clips, never exceeds 1"


def test_validity_is_its_own_block_not_a_sentinel_range():
    """Hard rule 5: 'unobserved' and 'an obstacle at that range' must not be one input."""
    valid = np.ones(36, dtype=bool)
    valid[5] = False
    o = obs(valid=valid)
    assert o[SPEC.slices["validity"]][5] == 0.0
    assert o[SPEC.slices["validity"]].sum() == 35.0
    # The range channel is untouched by the flag: the two carry different information.
    assert o[5] == pytest.approx(0.5)


def test_velocity_is_scaled_by_its_own_axis_ceilings():
    o = obs(velocity=[CFG["actuation"]["v_max"], CFG["actuation"]["vy_max"],
                      CFG["actuation"]["w_max"]])
    assert list(o[SPEC.slices["velocity"]]) == pytest.approx([1.0, 1.0, 1.0])
    o = obs(velocity=[-CFG["actuation"]["v_max"], 0.0, 0.0])
    assert o[SPEC.slices["velocity"]][0] == pytest.approx(-1.0)


def test_velocity_beyond_the_box_clips_rather_than_exceeding_one():
    o = obs(velocity=[10.0, -10.0, 10.0])
    assert list(o[SPEC.slices["velocity"]]) == pytest.approx([1.0, -1.0, 1.0])


def test_the_lookahead_distance_is_scaled_by_the_lookahead_itself():
    o = obs(lookahead_body=(CFG["planner"]["lookahead"], 0.0, 1.0))
    assert o[SPEC.slices["lookahead"]][0] == pytest.approx(1.0)
    o = obs(lookahead_body=(0.4, 0.6, 0.8))
    block = o[SPEC.slices["lookahead"]]
    assert block[0] == pytest.approx(0.4 / CFG["planner"]["lookahead"])
    assert (block[1], block[2]) == pytest.approx((0.6, 0.8))


def test_the_goal_distance_is_scaled_and_clipped():
    scale = CFG["env"]["observation"]["goal_scale"]
    o = obs(goal_body=(scale / 2, 0.0, 1.0))
    assert o[SPEC.slices["goal"]][0] == pytest.approx(0.5)
    o = obs(goal_body=(scale * 3, 0.0, 1.0))
    assert o[SPEC.slices["goal"]][0] == pytest.approx(1.0)


def test_the_certified_channel_is_stop_active_only():
    assert obs(stop_active=False)[-1] == 0.0
    assert obs(stop_active=True)[-1] == 1.0


def test_the_observation_is_float32_and_within_its_declared_bounds():
    low, high = bounds(SPEC)
    for kwargs in ({}, {"stop_active": True}, {"velocity": [-9.0, 9.0, -9.0]},
                   {"ranges": np.zeros(36)}, {"goal_body": (99.0, -0.7, 0.7)}):
        o = obs(**kwargs)
        assert o.dtype == np.float32
        assert np.all(o >= low) and np.all(o <= high)


def test_a_wrong_sized_scan_is_refused():
    with pytest.raises(ValueError, match="policy stream"):
        obs(ranges=np.ones(35))
    with pytest.raises(ValueError, match="policy stream"):
        obs(valid=np.ones(12, dtype=bool))


def test_a_wrong_sized_velocity_is_refused():
    with pytest.raises(ValueError, match="vx, vy, wz"):
        obs(velocity=[0.1, 0.2])


# ----------------------------------------------------------------------
# The declared dropout augmentation (mechanism ruled 2026-09-26)
# ----------------------------------------------------------------------
FLAT = DropoutSpec(arc_probability=0.0, arc_bins=(1, 6), per_step_rate=0.5)
ARCS = DropoutSpec(arc_probability=1.0, arc_bins=(3, 3), per_step_rate=0.0)
OFF = DropoutSpec(arc_probability=0.0, arc_bins=(1, 6), per_step_rate=0.0)


def test_a_dropped_bin_reads_zero_not_range_max():
    """Corrected 2026-09-26. range_max reads as "nothing out there", which is unobserved
    space presented as free -- exactly what hard rule 5 forbids. 0.0 reads as "something
    right here", the safe direction, and the validity channel beside it says it is not a
    real return."""
    rng = np.random.default_rng(0)
    r, v = dropout(np.full(36, 1.23), np.ones(36, dtype=bool), rng, FLAT)
    assert (~v).any(), "nothing was dropped at a per-step rate of 0.5"
    assert np.all(r[~v] == 0.0)
    assert np.all(r[v] == 1.23)


def test_an_inactive_spec_changes_nothing_and_does_not_consume_the_rng():
    rng = np.random.default_rng(0)
    before = rng.bit_generator.state
    r, v = dropout(np.full(36, 2.0), np.ones(36, dtype=bool), rng, OFF)
    assert np.all(r == 2.0) and v.all()
    assert rng.bit_generator.state == before
    assert not OFF.active and FLAT.active and ARCS.active


def test_a_per_step_rate_of_one_drops_everything():
    spec = DropoutSpec(arc_probability=0.0, arc_bins=(1, 6), per_step_rate=1.0)
    _r, v = dropout(np.full(36, 2.0), np.ones(36, dtype=bool),
                    np.random.default_rng(0), spec)
    assert not v.any()


def test_the_per_step_rate_is_hit_on_average():
    rng = np.random.default_rng(1)
    spec = DropoutSpec(arc_probability=0.0, arc_bins=(1, 6), per_step_rate=0.1)
    dropped = sum(int((~dropout(np.full(36, 2.0), np.ones(36, dtype=bool), rng, spec)[1]).sum())
                  for _ in range(200))
    assert 0.08 < dropped / (200 * 36) < 0.12


def test_dropout_does_not_write_through_its_inputs():
    rng = np.random.default_rng(0)
    ranges = np.full(36, 2.0)
    valid = np.ones(36, dtype=bool)
    spec = DropoutSpec(arc_probability=0.0, arc_bins=(1, 6), per_step_rate=1.0)
    dropout(ranges, valid, rng, spec)
    assert np.all(ranges == 2.0) and valid.all()


def test_an_impossible_spec_is_refused():
    with pytest.raises(ValueError, match="per_step_rate"):
        DropoutSpec(arc_probability=0.0, arc_bins=(1, 6), per_step_rate=1.2)
    with pytest.raises(ValueError, match="arc_probability"):
        DropoutSpec(arc_probability=-0.1, arc_bins=(1, 6), per_step_rate=0.0)
    with pytest.raises(ValueError, match="arc_bins"):
        DropoutSpec(arc_probability=0.1, arc_bins=(0, 6), per_step_rate=0.0)
    with pytest.raises(ValueError, match="arc_bins"):
        DropoutSpec(arc_probability=0.1, arc_bins=(6, 3), per_step_rate=0.0)


# ------------------------------------------------- the blocked arc, held for a whole episode
def test_the_blocked_arc_is_contiguous_and_the_requested_width():
    mask = sample_blocked_arc(ARCS, 36, np.random.default_rng(3))
    assert mask.sum() == 3
    idx = np.flatnonzero(mask)
    # Contiguous on a RING: either consecutive, or consecutive once rolled to the start.
    rolled = np.flatnonzero(np.roll(mask, -int(idx[0])))
    assert np.array_equal(rolled, np.arange(3))


def test_the_arc_wraps_around_the_ring():
    """Bin 0 is not a physical boundary, so an arc must be able to straddle it."""
    spec = DropoutSpec(arc_probability=1.0, arc_bins=(5, 5), per_step_rate=0.0)
    wrapped = False
    for seed in range(200):
        mask = sample_blocked_arc(spec, 36, np.random.default_rng(seed))
        assert mask.sum() == 5
        if mask[0] and mask[-1]:
            wrapped = True
    assert wrapped, "no arc ever straddled bin 0 in 200 draws"


def test_an_arc_probability_of_zero_still_draws_so_the_stream_does_not_branch():
    """The failure layouts.generate hit on 2026-09-23: a draw skipped on one branch makes
    two runs with the same seed visit different worlds. The mask is empty, the draw is not."""
    never = DropoutSpec(arc_probability=0.0, arc_bins=(1, 6), per_step_rate=0.0)
    always = DropoutSpec(arc_probability=1.0, arc_bins=(1, 6), per_step_rate=0.0)
    a, b = np.random.default_rng(7), np.random.default_rng(7)
    assert not sample_blocked_arc(never, 36, a).any()
    assert sample_blocked_arc(always, 36, b).any()
    assert a.bit_generator.state == b.bit_generator.state


def test_the_arc_probability_is_hit_on_average():
    rng = np.random.default_rng(11)
    spec = DropoutSpec(arc_probability=0.10, arc_bins=(1, 6), per_step_rate=0.0)
    drawn = sum(int(sample_blocked_arc(spec, 36, rng).any()) for _ in range(2000))
    assert 0.08 < drawn / 2000 < 0.12


def test_the_same_arc_applies_on_every_step_of_the_episode():
    """The point of the mechanism: a persistent blind sector, not per-step noise a policy can
    average away. The arc is passed in, so the caller holds it for the episode."""
    arc = sample_blocked_arc(ARCS, 36, np.random.default_rng(5))
    rng = np.random.default_rng(0)
    for _ in range(10):
        _r, v = dropout(np.full(36, 2.0), np.ones(36, dtype=bool), rng, ARCS, arc)
        assert np.array_equal(~v, arc)


def test_an_arc_of_the_wrong_length_is_refused():
    with pytest.raises(ValueError, match="blocked arc covers"):
        dropout(np.full(36, 2.0), np.ones(36, dtype=bool), np.random.default_rng(0),
                ARCS, np.zeros(20, dtype=bool))


def test_the_arc_and_the_per_step_rate_compose():
    both = DropoutSpec(arc_probability=1.0, arc_bins=(3, 3), per_step_rate=0.5)
    arc = sample_blocked_arc(both, 36, np.random.default_rng(5))
    _r, v = dropout(np.full(36, 2.0), np.ones(36, dtype=bool),
                    np.random.default_rng(0), both, arc)
    assert np.all(~v[arc]), "the episode's arc must stay blind"
    assert (~v).sum() > arc.sum(), "the per-step rate added nothing"


def test_the_config_carries_the_ruled_mechanism():
    """The preregistered numbers, so a change to them fails here rather than passing quietly."""
    spec = DropoutSpec.from_config(load_config(REPO / "configs" / "base.yaml"))
    assert spec.arc_probability == pytest.approx(0.10)
    assert spec.arc_bins == (1, 6)
    assert spec.per_step_rate == pytest.approx(0.01)
