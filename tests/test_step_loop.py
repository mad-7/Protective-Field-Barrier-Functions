"""Known-answer tests for the step loop's bookkeeping (hard rule 6). No simulator.

`tests/test_step_loop_sim.py` drives the loop against a real instance; this pins the
arithmetic the metrics rest on, above all `stops_per_100m` -- the primary metric, so a
miscount here would be a miscount of the headline result.
"""
import numpy as np
import pytest

from pfbnav.envs.step_loop import LoopCounters, StepState


def state(**over):
    base = {"step": 1, "sim_time": 0.05, "pose": np.zeros(3), "velocity": np.zeros(3),
            "requested": np.zeros(3), "applied": np.zeros(3), "scan": None,
            "in_contact": False, "contact_names": (),
            "certified": {"stop_active": False, "stop_triggered": False, "field_id": 0,
                          "warning": False, "breached": False, "at_rest": True,
                          "speed": 0.0},
            "timings_s": {}, "over_budget": False}
    return StepState(**{**base, **over})


def test_intervened_compares_applied_against_requested():
    assert not state(requested=np.array([0.2, 0.0, 0.0]),
                     applied=np.array([0.2, 0.0, 0.0])).intervened
    assert state(requested=np.array([0.2, 0.0, 0.0]),
                 applied=np.array([0.0, 0.0, 0.0])).intervened


def test_stop_active_reads_the_emulators_report():
    s = state(certified={"stop_active": True, "stop_triggered": False, "field_id": 1,
                         "warning": False, "breached": True, "at_rest": False,
                         "speed": 0.2})
    assert s.stop_active


def test_stops_per_100m_is_none_before_the_robot_has_moved():
    c = LoopCounters()
    assert c.stops_per_100m() is None
    c.protective_stops = 3
    assert c.stops_per_100m() is None, "0 stops over 0 m is not a rate"


def test_stops_per_100m_known_answer():
    c = LoopCounters(protective_stops=3, distance_m=50.0)
    assert c.stops_per_100m() == pytest.approx(6.0)
    c = LoopCounters(protective_stops=1, distance_m=200.0)
    assert c.stops_per_100m() == pytest.approx(0.5)


def test_zero_stops_over_a_real_distance_is_zero_not_none():
    assert LoopCounters(protective_stops=0, distance_m=12.0).stops_per_100m() == 0.0


def test_warnings_per_100m_is_entries_not_steps():
    """Known answer for the secondary near-miss metric (hard rule 6).

    Three entries over 2 m is 150 per 100 m, whatever the dwell. None before the robot
    moves, for the same reason the primary metric is: 0/0 is not zero near misses.
    """
    c = LoopCounters()
    assert c.warnings_per_100m() is None
    c.warning_entries, c.warning_steps, c.distance_m = 3, 47, 2.0
    assert c.warnings_per_100m() == pytest.approx(150.0)
    assert c.stops_per_100m() == pytest.approx(0.0)


# ---------------------------------------------------------------- the PFB's slot (frozen env)
def test_the_pfb_slot_passes_the_same_array_and_never_calls_the_filter_while_off():
    """Ruled 2026-10-03: filter-off behaviour must not change when WP2 fills the slot. Off, the
    slot hands on the command OBJECT itself -- no copy, no arithmetic -- so nothing WP2 writes
    behind the flag can reach a filter-off run."""
    from pfbnav.envs.step_loop import pfb_stage

    def boom(_):
        raise AssertionError("the filter was called while disabled")

    requested = np.array([0.1, -0.2, 0.3])
    assert pfb_stage(requested, False, boom) is requested
    assert np.array_equal(pfb_stage(requested, True, lambda r: 0.5 * r), 0.5 * requested)


def test_the_loop_builds_the_pfb_with_the_filter_on_and_nothing_with_it_off():
    """WP2 step 4 (2026-10-24): `filter.enabled` builds the PFB in the slot; off, nothing is built,
    and neither constructor touches the simulator."""
    from pathlib import Path

    from pfbnav.config import load_config
    from pfbnav.envs.step_loop import StepLoop
    from pfbnav.safety.pfb import PFB

    root = Path(__file__).resolve().parents[1]
    on = load_config(root / "configs" / "experiments" / "L5_sac_pfb_unshaped.yaml")
    assert on["filter"]["enabled"]
    assert isinstance(StepLoop(_OfflineClient(), on).pfb, PFB)
    off = load_config(root / "configs" / "base.yaml")
    assert not off["filter"]["enabled"] and StepLoop(_OfflineClient(), off).pfb is None


class _OfflineClient:
    """Enough of a client for the loop's constructor, which does no I/O."""

    running = False

    def __getattr__(self, name):
        raise AssertionError(f"the constructor touched the simulator ({name})")
