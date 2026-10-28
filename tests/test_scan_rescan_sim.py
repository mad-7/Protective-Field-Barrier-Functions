"""The scanner's same-tick re-render, on the simulator (the scan-fault fix, ruled 2026-10-23).
`pytest -m sim`.

The fault-injection test (`tests/test_scan_guard.py`) replays recorded packets through a stand-in
scanner, so it cannot show that the REAL re-render works. This does: after a step, `rescan` renders
the same tick again; with nothing changed the render is deterministic, so its packet must equal the
first exactly -- same stamp (it passes the staleness check), same ranges, same validity.
"""
from __future__ import annotations

import numpy as np
import pytest
from test_containment_sim import loop  # noqa: F401  (the module-scoped loop fixture)

pytestmark = pytest.mark.sim


def test_a_rescan_of_the_same_tick_reproduces_the_packet(loop):  # noqa: F811
    loop.reset(settle_steps=10)
    for _ in range(5):
        loop.step(np.zeros(3))
        now = loop.client.time()
        first = loop.scanner.read(sim_time=now)
        again = loop.scanner.rescan(sim_time=now)
        assert again.sim_time == first.sim_time
        assert np.array_equal(again.filter_ranges, first.filter_ranges)
        assert np.array_equal(again.filter_valid, first.filter_valid)
        assert np.array_equal(again.policy_ranges, first.policy_ranges)
