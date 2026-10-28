"""Known answers for the declared real-time measurement (`scripts/measure_realtime.py`; ruled
2026-10-27) and for the per-step compute accounting every evaluation reports
(`pfbnav/eval/compute.py`). No simulator."""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.eval.compute import BIN_S, ComputeHistogram, summarise
from pfbnav.sim.robot import ActuationLimits

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")


def _script():
    spec = importlib.util.spec_from_file_location("measure_realtime",
                                                  REPO / "scripts" / "measure_realtime.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ------------------------------------------------------------------ the selection rule
def test_the_largest_passing_pair_is_selected_ties_to_the_longer_horizon():
    m = _script()
    p95 = {(256, 15): 0.010, (256, 20): 0.012, (256, 30): 0.018, (512, 15): 0.020,
           (512, 20): 0.026, (512, 30): 0.040, (1024, 15): 0.041, (1024, 20): 0.055,
           (1024, 30): 0.080}
    # 512 x 30 and 1024 x 15 both do 15,360 rollout-steps and both pass: the longer horizon wins.
    assert m.select_pair(p95, 0.050) == (512, 30)
    assert m.select_pair({**p95, (512, 30): 0.051}, 0.050) == (1024, 15)
    # 512 x 15 and 256 x 30 tie at 7,680 and both pass at 0.020: the longer horizon again.
    assert m.select_pair(p95, 0.020) == (256, 30)
    assert m.select_pair(p95, 0.015) == (256, 20)
    assert m.select_pair(p95, 0.005) is None                # nothing passes: a finding
    assert m.select_pair({(256, 15): 0.050}, 0.050) == (256, 15)    # the limit itself passes
    assert m.PAIRS == [(s, h) for s in (256, 512, 1024) for h in (15, 20, 30)]


# ------------------------------------------------------------------ the synthetic aisle
def test_the_ray_cast_known_answers():
    m = _script()
    segs = m.aisle_segments(0.8, block_at=2.0)
    b = np.array([0.0, math.pi / 2, -math.pi / 2, math.pi / 4, math.pi])
    r = m.cast(np.array([0.0, 0.0, 0.0]), b, segs, 5.0)
    assert r[:3].tolist() == pytest.approx([2.0, 0.4, 0.4])
    assert r[3] == pytest.approx(0.4 * math.sqrt(2.0))
    assert r[4] == 5.0                                      # behind: open, range_max
    # Turned 90 deg left and moved 0.1 m left: the left wall is dead ahead at 0.3 m.
    r = m.cast(np.array([0.0, 0.1, math.pi / 2]), np.array([0.0, math.pi]), segs, 5.0)
    assert r.tolist() == pytest.approx([0.3, 0.5])
    open_aisle = m.cast(np.zeros(3), np.array([0.0]), m.aisle_segments(0.8), 5.0)
    assert open_aisle.tolist() == [5.0]                     # nothing ahead: range_max


def test_the_kinematic_aisle_applies_commands_as_the_base_does():
    """From rest, 0.4 m/s forward: the speed rises by a_max dt a step (the limiter), the pose by
    the applied speed x dt, and the estimate follows by the estimator's filter."""
    m = _script()
    a = m.Aisle(CFG, 0.8)
    lim = ActuationLimits.from_config(CFG)
    dt = float(CFG["sim"]["dt"])
    a.apply(np.array([0.4, 0.0, 0.0]))
    assert a.applied[0] == pytest.approx(lim.a_max * dt)
    assert a.pose[0] == pytest.approx(lim.a_max * dt * dt)
    assert a.estimate[0] == pytest.approx(a.alpha * lim.a_max * dt)
    inp = a.input()
    assert inp.route.points[0].tolist() == pytest.approx([0.0, 0.0], abs=1e-12)
    assert inp.velocity.tolist() == pytest.approx(a.estimate.tolist())
    assert len(inp.scan.ranges) == 120 and inp.scan.valid.all()


def test_the_pfb_part_moves_then_holds_the_robot(monkeypatch):
    """A shortened pfb part: the robot moves, then is held before the box (stalled steps exist and
    the robot never reaches the box: it settles short of it)."""
    m = _script()
    monkeypatch.setattr(m, "PFB_STEPS", 300)
    monkeypatch.setattr(m, "AISLES", (0.8,))
    out = m.pfb_part(CFG, 10.0)
    row = out["0.8"]
    assert row["moving_steps"] > 0 and row["stalled_steps"] > 0
    assert 0.0 < row["stopped_at_x_m"] < m.BLOCK_AT
    assert out["finding"] is None


# ------------------------------------------------------------------ the compute histogram
def test_the_histogram_quantiles_are_upper_bin_edges():
    h = ComputeHistogram()
    for ms in (1.00, 1.04, 2.0, 2.0, 50.0):
        h.add(ms / 1e3)
    assert h.steps == 5
    assert h.quantile(0.2) == pytest.approx(1.1e-3)        # 1.00 and 1.04 ms share [1.0, 1.1)
    assert h.quantile(0.5) == pytest.approx(2.1e-3)
    assert h.quantile(0.95) == h.quantile(1.0) == pytest.approx(50.1e-3)
    assert ComputeHistogram().quantile(0.95) is None
    with pytest.raises(ValueError):
        h.add(-1e-6)
    with pytest.raises(ValueError):
        h.quantile(0.0)


def test_histograms_merge_and_round_trip_through_a_record():
    a, b = ComputeHistogram(), ComputeHistogram()
    for _ in range(95):
        a.add(0.010)
    for _ in range(5):
        b.add(0.090)
    back = ComputeHistogram.from_record(a.to_record())
    assert back.counts == a.counts
    s = summarise([a, b], limit_s=0.050)
    assert s["steps"] == 100 and s["p95_ms"] == pytest.approx(10.0 + 1e3 * BIN_S)
    assert s["meets_limit"] is True and s["max_ms"] == pytest.approx(90.0 + 1e3 * BIN_S)
    b.add(0.090)                                           # 6 % now over: p95 in the slow bin
    assert summarise([a, b], limit_s=0.050)["meets_limit"] is False
    assert summarise([]) == {"steps": 0, "p50_ms": None, "p95_ms": None, "max_ms": None}
