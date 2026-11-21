"""TensorBoard episode statistics: known answers on synthetic episodes (hard rule 6)."""
from types import SimpleNamespace

import pytest

from pfbnav.config import load_config
from pfbnav.training.record import RECORD_KEY
from pfbnav.training.tb_stats import EpisodeStats, window_stats


def ep(outcome="success", stops=0, dist=10.0, plan=10.0, ret=100.0, length=500, mod=None, infeas=None,
       stage="full"):
    r = {"outcome": outcome, "protective_stops": stops, "distance_m": dist, "plan_length_m": plan,
         "episode_return": ret, "episode_length": length, "curriculum_stage": stage}
    if mod is not None:
        r.update(pfb_modified=mod, pfb_infeasible=infeas or 0)
    return r


def test_known_answers_on_a_small_window():
    stages = ["a", "b", "full"]
    records = [ep("success", stops=1, dist=10, plan=8, ret=100, length=400, mod=100, infeas=4, stage="b"),
               ep("timeout", stops=3, dist=20, plan=12, ret=-50, length=600, mod=200, infeas=6, stage="full"),
               ep("success", stops=0, dist=10, plan=10, ret=60, length=1000, mod=0, infeas=0, stage="full"),
               ep("stall_truncated", stops=0, dist=0.0, plan=10, ret=-10, length=1000, mod=0, infeas=0, stage="full")]
    s = window_stats(records, stages)
    assert s["episodes/window"] == 4
    assert s["episodes/success_rate"] == 0.5
    assert s["episodes/outcome_share/timeout"] == 0.25 and s["episodes/outcome_share/stall_truncated"] == 0.25
    assert s["episodes/stops_per_100m_driven"] == pytest.approx(100 * 4 / 40)
    assert s["episodes/stops_per_100m_planned"] == pytest.approx(100 * 4 / 40)
    assert s["episodes/mean_return"] == pytest.approx((100 - 50 + 60 - 10) / 4)
    assert s["episodes/mean_length"] == pytest.approx(750)
    assert s["episodes/pfb_modified_share"] == pytest.approx(300 / 3000)
    assert s["episodes/pfb_infeasible_share"] == pytest.approx(10 / 3000)
    assert s["curriculum/stage"] == 2.0                       # the LATEST episode's stage


def test_a_quantity_without_a_denominator_is_left_out_not_zero():
    s = window_stats([ep("timeout", stops=2, dist=0.0, plan=0.0, length=0)], ["full"])
    assert "episodes/stops_per_100m_driven" not in s and "episodes/stops_per_100m_planned" not in s
    s = window_stats([ep("success")], ["full"])                  # an arm without the PFB has no PFB share
    assert "episodes/pfb_modified_share" not in s
    assert window_stats([]) == {}


def test_an_unknown_stage_is_left_out():
    assert "curriculum/stage" not in window_stats([ep(stage="mystery")], ["a", "full"])


class FakeLogger:
    def __init__(self):
        self.values: dict[str, float] = {}

    def record(self, name, value, **kwargs):
        self.values[name] = value


def _step(cb, *records):
    cb.locals = {"infos": [({RECORD_KEY: r} if r is not None else {}) for r in records]}
    return cb._on_step()


def test_the_callback_logs_when_an_episode_ends_and_keeps_only_the_window():
    cb = EpisodeStats(window=2, stages=["full"])
    cb.model = SimpleNamespace(logger=FakeLogger())
    assert _step(cb, None) is True and cb.model.logger.values == {}          # no episode finished: nothing logged
    _step(cb, ep("success"))
    assert cb.model.logger.values["episodes/success_rate"] == 1.0
    _step(cb, ep("timeout"))
    assert cb.model.logger.values["episodes/success_rate"] == 0.5
    _step(cb, ep("timeout"), None)                                          # the oldest success leaves the window
    assert cb.model.logger.values["episodes/success_rate"] == 0.0
    assert cb.model.logger.values["episodes/window"] == 2.0


def test_the_window_must_hold_an_episode_and_the_config_carries_it():
    with pytest.raises(ValueError):
        EpisodeStats(window=0)
    from pathlib import Path
    cfg = load_config(Path(__file__).resolve().parents[1] / "configs" / "experiments" / "L1_sac_shaped.yaml")
    assert int(cfg["training"]["episode_stats_window"]) >= 1
