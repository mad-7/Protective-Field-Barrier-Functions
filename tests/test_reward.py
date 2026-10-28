"""Known-answer tests for the reward (hard rule 6 and hard rule 7).

The weights live in `configs/base.yaml`; these pin how they combine, and in particular that
a protective stop is charged ONCE on its trigger -- charging per stopped step would pay a
policy to break a latch it cannot break, and would make the penalty depend on the restart
delay rather than on the stop.
"""
from pathlib import Path

import pytest

from pfbnav.config import load_config
from pfbnav.envs.reward import (
    RewardWeights,
    episode_outcome,
    intervention_cost,
    proximity_cost,
    step_reward,
)

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
W = RewardWeights.from_config(CFG)
CEILINGS = (CFG["actuation"]["v_max"], CFG["actuation"]["vy_max"], CFG["actuation"]["w_max"])


def test_the_weights_come_from_the_config():
    assert W.k_progress == CFG["reward"]["k_progress"]
    assert W.goal_bonus == CFG["reward"]["goal_bonus"]
    assert W.contact_penalty == CFG["reward"]["contact_penalty"]
    assert W.max_steps == CFG["reward"]["max_steps"]


def test_a_quiet_step_costs_only_time():
    total, terms = step_reward(W, progress_m=0.0, stop_triggered=False, intervention=0.0)
    assert total == pytest.approx(-W.time_cost)
    assert terms["time"] == pytest.approx(-W.time_cost)
    assert all(terms[k] == 0.0 for k in ("progress", "stop", "intervention", "proximity",
                                         "goal", "contact"))


def test_progress_is_paid_per_metre_along_the_plan():
    total, terms = step_reward(W, progress_m=0.02, stop_triggered=False, intervention=0.0)
    assert terms["progress"] == pytest.approx(W.k_progress * 0.02)
    assert total == pytest.approx(W.k_progress * 0.02 - W.time_cost)


def test_the_breakdown_sums_to_the_reward_exactly():
    total, terms = step_reward(W, progress_m=0.05, stop_triggered=True, intervention=0.3,
                               proximity=0.4, reached_goal=True, in_contact=True)
    assert total == pytest.approx(sum(terms.values()))
    assert set(terms) == {"progress", "time", "stop", "intervention", "proximity", "goal",
                          "contact"}


def test_a_stop_is_charged_once_on_the_trigger():
    triggered, _ = step_reward(W, progress_m=0.0, stop_triggered=True, intervention=0.0)
    held, _ = step_reward(W, progress_m=0.0, stop_triggered=False, intervention=0.0)
    assert triggered == pytest.approx(held - W.k_stop)


def test_the_goal_bonus_and_the_contact_penalty_are_terminal_and_large():
    goal, _ = step_reward(W, progress_m=0.0, stop_triggered=False, intervention=0.0,
                          reached_goal=True)
    crash, _ = step_reward(W, progress_m=0.0, stop_triggered=False, intervention=0.0,
                           in_contact=True)
    assert goal > 0.0 and crash < 0.0
    assert goal - crash == pytest.approx(W.goal_bonus + W.contact_penalty)


# ----------------------------------------------------------------------
def test_intervention_is_zero_when_the_pfb_passes_the_command():
    assert intervention_cost([0.3, 0.0, 0.5], [0.3, 0.0, 0.5], CEILINGS) == 0.0


def test_intervention_is_exactly_zero_without_the_filter():
    """Ruled 2026-10-24: a method without the PFB has no intervention, whatever the certified layer,
    the projection or the limiter did to its command."""
    assert intervention_cost(list(CEILINGS), None, CEILINGS) == 0.0


def test_the_step_term_is_zero_without_the_filter_and_under_the_latch():
    """Ruled 2026-10-26: no PFB report (filter off) -> 0 whatever happened, latched or not (L1's
    pilot condition); a latched step -> 0 even though the PFB zeroed the request; otherwise the
    PFB's change, as `intervention_cost`."""
    from pfbnav.envs.reward import pfb_intervention

    request = [CEILINGS[0], 0.0, 0.0]
    assert pfb_intervention(request, None, CEILINGS) == 0.0
    latched = {"mode": "latched", "u": [0.0, 0.0, 0.0]}
    assert pfb_intervention(request, latched, CEILINGS) == 0.0
    filtered = {"mode": "filtered", "u": [0.0, 0.0, 0.0]}
    assert pfb_intervention(request, filtered, CEILINGS) == pytest.approx(1.0)
    with pytest.raises(ValueError):
        pfb_intervention(request, None, (0.0, 0.5, 1.5))         # ceilings still validated


def test_intervention_is_one_for_a_full_ceiling_on_one_axis():
    # The configured ceilings (v_max = vy_max = 0.50 since 2026-10-19; 0.40 / 0.50 pre-A20)
    assert intervention_cost([CEILINGS[0], 0.0, 0.0], [0.0, 0.0, 0.0],
                             CEILINGS) == pytest.approx(1.0)
    assert intervention_cost([0.0, CEILINGS[1], 0.0], [0.0, 0.0, 0.0],
                             CEILINGS) == pytest.approx(1.0)


def test_intervention_is_the_squared_norm_on_each_axis_own_ceiling():
    """||(u_request - u_PFB) / ceilings||^2: on an asymmetric box (the pre-A20 0.40 / 0.50), 0.1 m/s
    forward is (0.25)^2 and 0.1 m/s lateral (0.20)^2; both together add."""
    asymmetric = (0.40, 0.50, 1.5)
    forward = intervention_cost([0.1, 0.0, 0.0], [0.0, 0.0, 0.0], asymmetric)
    lateral = intervention_cost([0.0, 0.1, 0.0], [0.0, 0.0, 0.0], asymmetric)
    both = intervention_cost([0.1, 0.1, 0.0], [0.0, 0.0, 0.0], asymmetric)
    assert forward == pytest.approx(0.0625) and lateral == pytest.approx(0.04)
    assert both == pytest.approx(0.1025)


def test_intervention_refuses_mismatched_or_zero_ceilings():
    with pytest.raises(ValueError):
        intervention_cost([0.1, 0.0], [0.0, 0.0, 0.0], CEILINGS)
    with pytest.raises(ValueError):
        intervention_cost([0.1, 0.0, 0.0], [0.0, 0.0, 0.0], (0.0, 0.5, 1.5))


# ----------------------------------------------------------------------
def test_proximity_cost_known_answers():
    assert proximity_cost(0.0, 0.5) == pytest.approx(1.0)
    assert proximity_cost(0.25, 0.5) == pytest.approx(0.5)
    assert proximity_cost(0.5, 0.5) == pytest.approx(0.0)
    assert proximity_cost(3.0, 0.5) == pytest.approx(0.0)


def test_proximity_is_off_by_default():
    assert W.k_proximity == 0.0
    total, terms = step_reward(W, progress_m=0.0, stop_triggered=False, intervention=0.0,
                               proximity=1.0)
    assert terms["proximity"] == 0.0
    assert total == pytest.approx(-W.time_cost)


# ----------------------------------------------------------------------
def test_episode_outcomes_are_named():
    assert episode_outcome(reached_goal=True, in_contact=False, steps=10,
                           weights=W) == (True, False, "success")
    assert episode_outcome(reached_goal=False, in_contact=True, steps=10,
                           weights=W) == (True, False, "collision")
    assert episode_outcome(reached_goal=False, in_contact=False, steps=W.max_steps,
                           weights=W) == (False, True, "timeout")
    assert episode_outcome(reached_goal=False, in_contact=False, steps=3,
                           weights=W) == (False, False, "running")


def test_arriving_and_colliding_on_the_same_step_is_a_collision():
    """A run that ends in contact is not a success, whatever else happened that step."""
    assert episode_outcome(reached_goal=True, in_contact=True, steps=10,
                           weights=W) == (True, False, "collision")


def test_a_timeout_under_a_latched_stop_is_its_own_outcome():
    """Ruling 4 (supervisor 2026-09-27): distinct from timeout, because they are different
    failures. A timeout is a controller that was too slow; an unrecoverable stop is a robot
    that could not legally move again. Pooling them hides the one the safety layers cause."""
    w = RewardWeights.from_config(CFG)
    out = lambda **kw: episode_outcome(reached_goal=False, in_contact=False,
                                       steps=w.max_steps, weights=w, **kw)
    assert out()[2] == "timeout"
    assert out(stop_active=True, restart_blocked=False)[2] == "timeout"
    assert out(stop_active=False, restart_blocked=True)[2] == "timeout"
    assert out(stop_active=True, restart_blocked=True)[2] == "unrecoverable_stop"
    # Both are truncations, not terminations: the episode ran out of steps either way.
    assert out(stop_active=True, restart_blocked=True)[:2] == (False, True)


def test_an_intrusion_inside_the_minimum_margin_at_rest_is_stuck_too():
    """Ruled 2026-10-16. The emulator computes `restart_blocked` only while the robot's own field
    is clear, so a robot at rest with that field breached -- which no direction can release --
    reported restart_blocked False and was labelled `timeout` (two of the 2026-10-13 traces)."""
    w = RewardWeights.from_config(CFG)
    out = lambda **kw: episode_outcome(reached_goal=False, in_contact=False,
                                       steps=w.max_steps, weights=w, **kw)
    # Both stuck forms.
    assert out(stop_active=True, restart_blocked=True, breached=False, at_rest=True)[2]         == "unrecoverable_stop"
    assert out(stop_active=True, restart_blocked=False, breached=True, at_rest=True)[2]         == "unrecoverable_stop"
    # Genuine timeouts: a stop that has just fired with the robot still moving, a breach with no
    # stop latched, and a latched stop that is about to release.
    assert out(stop_active=True, restart_blocked=False, breached=True, at_rest=False)[2]         == "timeout"
    assert out(stop_active=False, restart_blocked=False, breached=True, at_rest=True)[2]         == "timeout"
    assert out(stop_active=True, restart_blocked=False, breached=False, at_rest=True)[2]         == "timeout"
    # Before the step limit nothing is an outcome yet.
    assert episode_outcome(reached_goal=False, in_contact=False, steps=w.max_steps - 1,
                           weights=w, stop_active=True, breached=True, at_rest=True)[2]         == "running"


def test_contact_and_the_goal_still_outrank_a_latched_stop():
    """A collision is a collision even if the layer had also latched, and an episode that
    reaches the goal has succeeded whatever the layer was doing on the way."""
    w = RewardWeights.from_config(CFG)
    assert episode_outcome(reached_goal=False, in_contact=True, steps=w.max_steps, weights=w,
                           stop_active=True, restart_blocked=True)[2] == "collision"
    assert episode_outcome(reached_goal=True, in_contact=False, steps=w.max_steps, weights=w,
                           stop_active=True, restart_blocked=True)[2] == "success"
