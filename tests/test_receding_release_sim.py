"""The receding release (A24) in the loop, on the simulator (ruled 2026-10-23). `pytest -m sim`.

Scripted: a wall across the way ahead, placed in memory (removed afterwards; the scene is never
saved); the robot creeps toward it at 0.015 m/s (below the 0.02 standstill exit, so at rest by
classification) until the at-rest field trips; it is held; then it commands a receding creep.

Required, as ruled: the release occurs, and the robot leaves the at-rest field without a new
trigger. And the precondition that makes this a test of A24 rather than of the ordinary restart
(its positive control): when held, the at-rest field IS breached -- the intrusion lies inside it, the
state A22 alone could never leave.
"""
from __future__ import annotations

import pytest
from test_containment_sim import _place, loop  # noqa: F401  (the module-scoped loop fixture)

pytestmark = pytest.mark.sim

CREEP = 0.015                 # m/s
WALL_CENTRE = (0.45, 0.0)     # m, body frame: the face 0.44 m ahead, outside the at-rest field
MAX_STEPS = 2000
SETTLE = 10


def test_creep_in_then_recede_out(loop, cfg, sim_client):  # noqa: F811
    loop.reset(settle_steps=SETTLE)
    wall = _place(sim_client, cfg, WALL_CENTRE, 0.0)
    try:
        # 1. Creep in until the at-rest field trips.
        trigger = None
        for _ in range(MAX_STEPS):
            st = loop.step((CREEP, 0.0, 0.0))
            if st.certified["stop_triggered"]:
                trigger = st
                break
        assert trigger is not None, "the creep never tripped the at-rest field"
        assert trigger.certified["triggered_at_rest"] and not trigger.certified[
            "triggered_stationary"], trigger.certified
        stops = loop.certified.stops

        # 2. Held: ask to recede. The positive control: held at rest WITH the at-rest field breached.
        held_breached = released = False
        for _ in range(MAX_STEPS):
            st = loop.step((-CREEP, 0.0, 0.0))
            c = st.certified
            if c["stop_active"] and c["at_rest"] and c["breached"]:
                held_breached = True
            if not c["stop_active"] and c["receding"]:
                released = True
                break
        assert held_breached, "the intrusion never lay inside the at-rest field: not an A24 case"
        assert released, "the receding creep was not released"

        # 3. Recede until the at-rest field is clear, with no new trigger.
        cleared = False
        for _ in range(MAX_STEPS):
            st = loop.step((-CREEP, 0.0, 0.0))
            assert not st.certified["stop_triggered"], "a new stop while receding"
            if not st.certified["breached"]:
                cleared = True
                break
        assert cleared, "the robot never left the at-rest field"
        assert loop.certified.stops == stops and loop.certified.receding_releases == 1
    finally:
        sim_client.sim.removeObjects([wall])
