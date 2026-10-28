"""The scan-fault fix (ruled 2026-10-23): known answers, the deterministic fault injection through the
step loop's read path, and the two false-alarm regressions.

The recorded scans are the 200 steps before the stationary trigger of the traced rerun
(`tests/data/scan_fault_20261023.json`): steps 1336 and 1337 are the corrupted renders, the other
198 good.
"""
import itertools
import json
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.envs.step_loop import LoopCounters, StepLoop
from pfbnav.safety.certified import CertifiedLayer
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.scan_guard import ScanGuard, all_unobserved, out_of_bounds
from pfbnav.sim.scan_packet import ScanPacket, bin_bearings
from pfbnav.sim.velocity_estimate import PoseDifferenceEstimator, VelocityEstimateConfig
from pfbnav.world.strata import certified_params_from_config

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
DATA = json.loads((REPO / "tests" / "data" / "scan_fault_20261023.json").read_text(encoding="utf-8"))
BY_STEP = {s["step"]: s for s in DATA["steps"]}
RANGE_MAX = float(CFG["scanners"]["range_max"])


def _packet(step, t=0.0):
    s = BY_STEP[step]
    r = np.array([np.nan if x is None else x / 1000.0 for x in s["ranges_mm"]])
    valid = np.ones(r.size, bool)
    valid[s["invalid_bins"]] = False
    r = np.where(valid, r, 0.0)
    pol = r.reshape(36, -1).min(axis=1) if r.size % 36 == 0 else np.full(36, RANGE_MAX)
    return ScanPacket(t, pol, np.ones(36, bool), bin_bearings(r.size), r, valid,
                      float(CFG["scanners"]["range_min"]), RANGE_MAX)


def _guard():
    return ScanGuard.from_config(CFG)


# --------------------------------------------------------------------------- known answers
def test_out_of_bounds_known_answers():
    prev = np.full(12, 2.0)
    prev[5] = 1.0
    ok = np.ones(12, bool)
    same = out_of_bounds(prev, ok, prev, ok, neighbour_bins=2, range_tolerance=0.06)
    assert same == 0
    # The near return moved two bins (an occlusion edge sweeping): inside the neighbourhood.
    moved = np.full(12, 2.0)
    moved[7] = 1.0
    assert out_of_bounds(prev, ok, moved, ok, neighbour_bins=2, range_tolerance=0.06) == 0
    # Everything 'vanishing' to 5 m: every bin outside.
    assert out_of_bounds(prev, ok, np.full(12, 5.0), ok, neighbour_bins=2,
                         range_tolerance=0.06) == 12
    # A bin newly unobserved is not tested; one with no valid neighbour before is not either.
    gone = ok.copy()
    gone[0] = False
    assert out_of_bounds(prev, ok, np.full(12, 5.0), gone, neighbour_bins=2,
                         range_tolerance=0.06) == 11


def test_the_fail_safe_marks_every_bin_unobserved():
    p = all_unobserved(_packet(1336))
    assert not p.filter_valid.any() and not p.policy_valid.any()
    assert (p.filter_ranges == 0.0).all() and (p.policy_ranges == 0.0).all()


# --------------------------------------------------------------------------- the recorded fault
def test_both_recorded_corrupted_packets_are_flagged():
    g = _guard()
    g.reset(_packet(1335))
    assert not g.plausible(_packet(1336))
    g.reject()
    assert not g.plausible(_packet(1337))       # against the last ACCEPTED scan, two steps old


# --------------------------------------------------------------------------- false alarms
def test_no_false_alarm_on_the_198_recorded_good_steps():
    steps = [s for s in sorted(BY_STEP) if s not in DATA["corrupted_steps"]]
    assert len(steps) == 198
    g = _guard()
    g.reset(_packet(steps[0]))
    for s in steps[1:]:
        assert g.plausible(_packet(s)), s
        g.accept(_packet(s))


def test_no_false_alarm_on_3196_emulated_moving_steps():
    """B1 driving four tune routes in the emulator (the 2026-10-23 prototype's sample): consecutive
    scans of a robot at speed, through the guard."""
    from pfbnav.control.pure_pursuit import PurePursuit
    from pfbnav.eval.recovery_control import emulate_layout
    from pfbnav.planning.astar import plan_from_config
    from pfbnav.world.baking import bake, planner_radii
    from pfbnav.world.scenarios import _circumscribed, load_set, world_of

    g = _guard()
    total, worst = 0, 0
    for name in ("tune_narrow", "tune_wide"):
        for i in (0, 2):
            rec = load_set(REPO / "scenarios" / "tune" / f"{name}.json")["scenarios"][i]
            world = world_of(rec, CFG)
            baked = bake(world.obstacles, world.bounds, float(CFG["planner"]["resolution"]),
                         planner_radii(CFG, circumscribed=_circumscribed(CFG)))
            plan = plan_from_config(baked, tuple(rec["start"][:2]), tuple(rec["goal"]), CFG)
            records = emulate_layout(CFG, [o.polygon for o in world.obstacles],
                                     np.asarray(plan.path_world), rec["start"],
                                     PurePursuit.from_config(CFG), 1.0, 800)
            for a, b in itertools.pairwise(records):
                n = out_of_bounds(a["scan"].ranges, a["scan"].valid, b["scan"].ranges,
                                  b["scan"].valid, neighbour_bins=g.neighbour_bins,
                                  range_tolerance=g.range_tolerance)
                total += 1
                worst = max(worst, n)
    assert total == 3196
    assert worst < g.corrupt_bins, worst


# --------------------------------------------------------------------------- fault injection
class _Client:
    def __init__(self, dt):
        self.t, self.dt = 0.0, dt

    def time(self):
        return self.t

    def step(self):
        self.t += self.dt


class _Scanner:
    """Replays packets: `reads` for each tick's first read, `rescans` for every re-render."""

    def __init__(self, reads, rescans):
        self.reads, self.rescans = list(reads), list(rescans)
        self.rescan_calls = 0

    def read(self, *, sim_time=None):
        return self.reads.pop(0)

    def rescan(self, *, sim_time=None):
        self.rescan_calls += 1
        return self.rescans.pop(0)


class _Base:
    applied_command = np.zeros(3)

    def command_body_velocity(self, cmd):
        return np.asarray(cmd, dtype=float)

    def pose(self):
        return (0.0, 0.0, 0.0)


class _Reading:
    in_contact, names = False, ()


class _Contact:
    def check(self):
        return _Reading()


def _loop(scanner):
    lp = StepLoop.__new__(StepLoop)
    lp.cfg, lp.dt = CFG, float(CFG["sim"]["dt"])
    lp.client, lp.scanner, lp.base, lp.contact = _Client(lp.dt), scanner, _Base(), _Contact()
    table = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
    lp.certified = CertifiedLayer(certified_params_from_config(CFG),
                                  float(CFG["robot"]["half_length"]),
                                  float(CFG["robot"]["half_width"]), table.extent)
    lp.velocity = PoseDifferenceEstimator(VelocityEstimateConfig.from_config(CFG))
    lp.filter_enabled, lp.certified_enabled = False, True
    lp.budget_s, lp.over_budget_is_fatal = 10.0, False
    lp.counters, lp.guard = LoopCounters(), ScanGuard.from_config(CFG)
    lp._step, lp._last_xy, lp._velocity = 0, None, np.zeros(3)
    lp.packet = _packet(1335)
    lp.guard.reset(lp.packet)
    lp._packet_fault = False
    return lp


def test_fault_injection_flagged_rerendered_failsafed_and_no_stationary_trigger():
    """The two recorded corrupted packets through the read path, each re-render corrupted too: both
    flagged, re-rendered twice, replaced by the fail-safe. The fail-safe stops the standing robot,
    and that stop is a scan-fault stop, never a stationary trigger (the construction gap)."""
    corrupt = [_packet(1336), _packet(1337)]
    scanner = _Scanner(reads=[corrupt[0], corrupt[1], _packet(1335), _packet(1335)],
                       rescans=[corrupt[0], corrupt[0], corrupt[1], corrupt[1]])
    lp = _loop(scanner)
    states = [lp.step(np.zeros(3)) for _ in range(4)]
    assert [s.scan_fault for s in states] == [True, True, False, False]
    assert [s.scan_rerenders for s in states] == [2, 2, 0, 0] and scanner.rescan_calls == 4
    assert not states[0].scan.filter_valid.any() and not states[1].scan.filter_valid.any()
    assert states[2].scan.filter_valid.all()
    c = lp.counters
    assert c.scan_faults == 2 and c.scan_rerenders == 4
    assert c.stationary_triggers == 0 and c.at_rest_triggers == 0
    assert c.protective_stops == c.scan_fault_stops == 1      # decided on the fail-safe, step 2
    assert c.stops_per_100m() is None or c.stops_per_100m() == 0.0
    report = lp.report()
    assert report["scan_faults"] == 2 and report["scan_fault_stops"] == 1


def test_a_corrupted_read_that_rerenders_clean_is_used_and_not_a_fault():
    scanner = _Scanner(reads=[_packet(1336)], rescans=[_packet(1335)])
    lp = _loop(scanner)
    s = lp.step(np.zeros(3))
    assert not s.scan_fault and s.scan_rerenders == 1 and s.scan.filter_valid.all()
    assert lp.counters.scan_faults == 0 and lp.counters.scan_rerenders == 1


def test_without_a_corrupted_render_the_read_path_is_the_plain_read():
    scanner = _Scanner(reads=[_packet(s) for s in (1300, 1301, 1302)], rescans=[])
    lp = _loop(scanner)
    states = [lp.step(np.zeros(3)) for _ in range(3)]
    assert scanner.rescan_calls == 0 and not any(s.scan_fault for s in states)
    assert [s.scan.filter_ranges.tolist() for s in states] == [
        _packet(x).filter_ranges.tolist() for x in (1300, 1301, 1302)]


@pytest.mark.parametrize("key", ["neighbour_bins", "range_tolerance_m", "corrupt_bins",
                                 "rerenders"])
def test_the_thresholds_are_the_ruled_ones(key):
    assert CFG["scanners"]["plausibility"][key] == {"neighbour_bins": 2, "range_tolerance_m": 0.06,
                                                    "corrupt_bins": 6, "rerenders": 2}[key]
