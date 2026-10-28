"""The synthetic recovery control's route check, and the emulator's positive control. (The
control itself was closed on 2026-10-16 as a recorded negative result; see below.)

Ruled 2026-10-16: the constructed scene must produce the intended stop state -- at rest after the
first stop, the at-rest field clear and B1's restart blocked -- which is checked here by emulating
the step loop's own chain over the declared slip spread, before any simulator run. The emulator's
own positive control (hard rule 15): replaying the 2026-10-13 corner fixture reproduces the
simulator's bare-B1 stop (step 126, s = 1.447 m; at rest, s = 1.483 m).
"""
import copy
import json
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.control.pure_pursuit import PurePursuit
from pfbnav.eval.recovery_control import (
    ControlSpec,
    emulate_layout,
    plan_route,
    route_clearance,
)

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
SPEC = ControlSpec.from_file(REPO / "configs" / "controls" / "recovery_synthetic.yaml")


@pytest.fixture(scope="module")
def route():
    return plan_route(CFG, SPEC)


def test_the_route_is_a_genuine_lethal_0_10_route(route):
    lethal = CFG["robot"]["half_width"] + CFG["planner"]["lethal_margin"]
    assert SPEC.min_route_clearance == pytest.approx(lethal)
    assert route_clearance(route, SPEC) >= SPEC.min_route_clearance


# The synthetic control was CLOSED on 2026-10-16 as a recorded negative result (ruling (b)): under
# the corrected fields no declared scene reproduces a restart-blocked corner stop robustly across
# slips, so the premise test it carried is withdrawn rather than left failing. The route test above
# and the emulator's positive control below stay: the emulation remains a known-answer tool.


@pytest.mark.skip(reason="A20 (2026-10-18): the fixture was recorded under the pre-A20 field and limits (a_max 1.0, alpha_max 3.0), which no longer exist in code; the emulator's positive control is re-established on a fresh A20 simulator trace of the same scenario (scripts/diag_recovery_corner.py)")
def test_the_emulator_reproduces_the_simulators_2026_10_13_stop():
    fx = json.loads((REPO / "tests" / "data" / "corner_stop_20261013.json")
                    .read_text(encoding="utf-8"))
    cfg = copy.deepcopy(CFG)
    cfg["planner"]["lethal_margin"] = fx["planner_lethal_margin"]
    from pfbnav.planning.astar import plan_from_config
    from pfbnav.world.baking import bake, planner_radii
    from pfbnav.world.scenarios import _circumscribed, world_of

    world = world_of(fx["scenario"], cfg)
    baked = bake(world.mapped, world.bounds, float(cfg["planner"]["resolution"]),
                 planner_radii(cfg, circumscribed=_circumscribed(cfg)))
    path = np.asarray(plan_from_config(baked, tuple(fx["scenario"]["start"][:2]),
                                       tuple(fx["scenario"]["goal"]), cfg).path_world)
    # The simulator ran this fixture on 2026-10-16 under the rotation-blind field, so the
    # emulator is checked under that field too (the field is a parameter; the machinery is what
    # this control tests).
    records = emulate_layout(cfg, [o.polygon for o in world.mapped], path,
                             fx["scenario"]["start"], PurePursuit.from_config(cfg), 0.95, 300,
                             rotation_blind=True)
    first = next(r for r in records if r["cert"]["stop_triggered"])
    assert abs(first["step"] - 126) <= 10 and first["s"] == pytest.approx(1.447, abs=0.03)
    assert records[-1]["s"] == pytest.approx(1.483, abs=0.01)
    assert records[-1]["cert"]["stop_active"] and records[-1]["cert"]["restart_blocked"]


def test_the_emulator_reproduces_the_simulators_a22_corner_run():
    """The emulator's positive control, re-pinned under A21 + A22 (2026-10-20). The simulator ran
    bare B1 on the 2026-10-13 corner scenario under the A21 field and the whole-field restart
    (`tests/data/corner_stop_A22_20261020.json`, run RECOVERY_CORNER_20261020T152314Z: ONE stop
    at step 86, s = 0.946 m, then held, ending stopped and restart-blocked at s = 0.983 m -- the
    A20 run's six stop-restart cycles are gone). The emulator, at the slip this control has always
    used (0.95, declared before the A20 run), must reproduce the first stop (step +-10, s +-0.03
    m), the number of stops (+-1), the final latch, and the BRAKING TRAVEL from the trigger's
    decision pose to rest (+-5 mm; ruled 2026-10-20). The final position (+-10 mm) was the A20
    criterion: it failed here by 15.8 mm because the emulator triggered one step later (one step is
    11.9 mm at cruise), while the braking itself agreed to about a millimetre (simulator 48.8 mm,
    emulator 47.5 mm). The braking travel is what the emulator models; the trigger's step falls
    either side of a boundary the scan crosses ~10 mm per step.

    The emulator's `s` is taken at each step's DECISION pose (before the motion), the simulator's
    `s_m` after it: the simulator's decision pose for its trigger at step n is step n-1's record
    (`trigger_decision` in the fixture)."""
    fx = json.loads((REPO / "tests" / "data" / "corner_stop_20261013.json")
                    .read_text(encoding="utf-8"))
    ref = json.loads((REPO / "tests" / "data" / "corner_stop_A22_20261020.json")
                     .read_text(encoding="utf-8"))
    cfg = copy.deepcopy(CFG)
    cfg["planner"]["lethal_margin"] = fx["planner_lethal_margin"]
    from pfbnav.planning.astar import plan_from_config
    from pfbnav.world.baking import bake, planner_radii
    from pfbnav.world.scenarios import _circumscribed, world_of

    world = world_of(fx["scenario"], cfg)
    baked = bake(world.mapped, world.bounds, float(cfg["planner"]["resolution"]),
                 planner_radii(cfg, circumscribed=_circumscribed(cfg)))
    path = np.asarray(plan_from_config(baked, tuple(fx["scenario"]["start"][:2]),
                                       tuple(fx["scenario"]["goal"]), cfg).path_world)
    records = emulate_layout(cfg, [o.polygon for o in world.mapped], path,
                             fx["scenario"]["start"], PurePursuit.from_config(cfg), 0.95,
                             ref["steps"])
    stops = [r for r in records if r["cert"]["stop_triggered"]]
    assert abs(stops[0]["step"] - ref["stops"][0]["step"]) <= 10
    assert stops[0]["s"] == pytest.approx(ref["stops"][0]["s_m"], abs=0.03)
    assert abs(len(stops) - len(ref["stops"])) <= 1
    assert records[-1]["s"] - stops[0]["s"] == pytest.approx(ref["braking_travel_m"], abs=0.005)
    assert records[-1]["cert"]["stop_active"] == ref["final"]["stop_active"]
    assert records[-1]["cert"]["restart_blocked"] == ref["final"]["restart_blocked"]

