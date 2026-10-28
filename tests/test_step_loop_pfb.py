"""The PFB in the step loop's slot (WP2 step 4, 2026-10-24), offline: what the filter is handed,
where its output goes, and what is recorded and counted. `tests/test_step_loop_sim.py` covers the
filter-off bit identity against a real instance."""
import math
from pathlib import Path

import numpy as np

from pfbnav.config import load_config
from pfbnav.envs.step_loop import LoopCounters, StepLoop
from pfbnav.safety import pfb_field as pf
from pfbnav.safety.certified import CertifiedLayer
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.safety.pfb import PFB, PFBConfig
from pfbnav.sim.scan_guard import ScanGuard
from pfbnav.sim.scan_packet import ScanPacket, bin_bearings
from pfbnav.sim.velocity_estimate import PoseDifferenceEstimator, VelocityEstimateConfig
from pfbnav.world.strata import certified_params_from_config

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
TABLE = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
N = int(CFG["scanners"]["filter_bins"])
RANGE_MAX = float(CFG["scanners"]["range_max"])


def _packet(distance):
    """A wall `distance` ahead, as the filter stream sends it: the bearings are NOT the bin centres
    (each is a quarter bin off), so the test sees whether the exact bearings reach the filter."""
    b = bin_bearings(N) + 0.25 * 2 * math.pi / N
    c = np.cos(b)
    with np.errstate(divide="ignore"):
        r = np.minimum(np.where(c > 1e-6, distance / c, RANGE_MAX), RANGE_MAX)
    pol = np.full(36, RANGE_MAX)
    return ScanPacket(0.0, pol, np.ones(36, bool), b, r, np.ones(N, bool),
                      float(CFG["scanners"]["range_min"]), RANGE_MAX)


class _Client:
    def __init__(self, dt):
        self.t, self.dt = 0.0, dt

    def time(self):
        return self.t

    def step(self):
        self.t += self.dt


class _Scanner:
    def __init__(self, packet):
        self.packet = packet

    def read(self, *, sim_time=None):
        return self.packet


class _Base:
    """Applies what it is given and remembers it (the base's own limiter is not under test)."""

    def __init__(self):
        self.applied_command = np.zeros(3)
        self.received = []

    def command_body_velocity(self, cmd):
        self.received.append(np.asarray(cmd, dtype=float).copy())
        self.applied_command = np.asarray(cmd, dtype=float).copy()
        return self.applied_command

    def pose(self):
        return (0.0, 0.0, 0.0)


class _Contact:
    class _R:
        in_contact, names = False, ()

    def check(self):
        return self._R()


def _loop(packet):
    lp = StepLoop.__new__(StepLoop)
    lp.cfg, lp.dt = CFG, float(CFG["sim"]["dt"])
    lp.client, lp.scanner, lp.base, lp.contact = (_Client(lp.dt), _Scanner(packet), _Base(),
                                                  _Contact())
    lp.certified = CertifiedLayer(certified_params_from_config(CFG),
                                  float(CFG["robot"]["half_length"]),
                                  float(CFG["robot"]["half_width"]), TABLE.extent)
    lp.velocity = PoseDifferenceEstimator(VelocityEstimateConfig.from_config(CFG))
    field = pf.PFBField(TABLE.extent, pf.field_params_from_config(CFG))
    lp.filter_enabled, lp.certified_enabled = True, True
    lp.pfb = PFB(field, PFBConfig.from_config(CFG, field))
    lp._pfb_result, lp._stop_active = None, False
    lp.budget_s, lp.over_budget_is_fatal = 10.0, False
    lp.counters, lp.guard = LoopCounters(), ScanGuard.from_config(CFG)
    lp._step, lp._last_xy, lp._velocity = 0, None, np.zeros(3)
    lp.packet = packet
    lp.guard.reset(packet)
    lp._packet_fault = False
    return lp


def test_the_filter_is_handed_what_the_certified_layer_decides_on():
    """The scan in hand with its EXACT bearings and validity, the velocity estimate, the command the
    base is executing, and the latch the layer left after the previous step."""
    lp = _loop(_packet(0.9))
    seen = []
    real = lp.pfb.filter

    def spy(u_nom, v_hat, u_prev, ranges, bearings, valid, *, latched=False):
        seen.append((np.asarray(v_hat).copy(), np.asarray(u_prev).copy(), np.asarray(bearings),
                     np.asarray(ranges), latched))
        return real(u_nom, v_hat, u_prev, ranges, bearings, valid, latched=latched)

    lp.pfb.filter = spy
    lp._velocity = np.array([0.11, 0.0, 0.0])
    lp.base.applied_command = np.array([0.12, 0.0, 0.0])
    lp._stop_active = True
    lp.step(np.array([0.2, 0.0, 0.0]))
    v_hat, u_prev, bearings, ranges, latched = seen[0]
    assert v_hat.tolist() == [0.11, 0.0, 0.0] and u_prev.tolist() == [0.12, 0.0, 0.0]
    assert np.array_equal(bearings, lp.scanner.packet.filter_bearings)
    assert not np.allclose(bearings, bin_bearings(N))
    assert np.array_equal(ranges, lp.scanner.packet.filter_ranges) and latched


def test_the_filtered_command_is_what_the_layer_judges_and_it_is_recorded_and_counted():
    """A wall 0.6 m ahead and a request for 0.5 m/s straight at it, from rest: the base receives the
    PFB's command (the layer, not stopping, passes it on), the state carries the PFB's decision and
    the nominal, and the counters count the change."""
    lp = _loop(_packet(0.6))
    states = [lp.step(np.array([0.5, 0.0, 0.0])) for _ in range(5)]
    for st, got in zip(states, lp.base.received, strict=True):
        assert st.pfb is not None and st.pfb["nominal"] == [0.5, 0.0, 0.0]
        if not st.certified["stop_active"]:
            assert got.tolist() == st.pfb["u"]
    assert all(st.pfb["u"] != st.pfb["nominal"] for st in states)
    assert lp.counters.pfb_modified == 5 and lp.counters.pfb_infeasible == 0
    report = lp.report()
    assert report["pfb_modified"] == 5 and report["pfb_infeasible"] == 0
    assert "pfb" in report["timings"]
