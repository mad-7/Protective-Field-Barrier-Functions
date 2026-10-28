"""The step loop against a real simulator (`pytest -m sim`). (WP1, slice 2)

What only an instance can answer: that the order in `pfbnav/envs/step_loop.py` holds
together -- one scan packet per step, stamped for the tick it describes; the applied
command reported as applied and not as requested; the ground-truth contact check in the
loop (which is what `preregistration/gates.yaml` calls the `env_step_loop` context); the
emulator stopping the robot when something enters its field, and the command being zeroed
while it does.

The obstacle exists in memory only and is removed afterwards; the scene is never saved.
"""
from __future__ import annotations

import numpy as np
import pytest

from pfbnav.envs.step_loop import StepLoop
from pfbnav.sim.robot import body_point_in_ref

pytestmark = pytest.mark.sim

WALL_SIZE_M = (0.02, 2.0, 0.60)     # thin, across the aisle, crossing the scan plane
SETTLE_STEPS = 10
CREEP = (0.05, 0.0, 0.0)            # m/s, slow enough that a stop is not a crash


@pytest.fixture(scope="module")
def loop(cfg, sim_client):
    lp = StepLoop(sim_client, cfg)
    sim_client.stop()                # the scanner script initialises at start
    lp.install()
    try:
        yield lp
    finally:
        lp.remove()
        sim_client.stop()
        sim_client.start(settle_steps=20)


def place_wall(sim_client, cfg, ahead_m: float) -> int:
    """A wall `ahead_m` in front of the body origin, as a handle to remove afterwards."""
    sim = sim_client.sim
    ref = sim_client.object(cfg["sim"]["objects"]["base_ref"])
    ref_z = float(sim.getObjectPosition(ref, sim.handle_world)[2])
    wall = int(sim.createPrimitiveShape(sim.primitiveshape_cuboid, list(WALL_SIZE_M), 0))
    yaw = float(cfg["robot"]["yaw_offset"])
    x, y, z = body_point_in_ref([ahead_m, 0.0, 0.5 * WALL_SIZE_M[2] - ref_z], yaw)
    sim.setObjectOrientation(wall, [0.0, 0.0, yaw], ref)
    sim.setObjectPosition(wall, [float(x), float(y), float(z)], ref)
    return wall


def test_reset_gives_a_first_state_with_a_scan_and_no_stop(loop):
    state = loop.reset(settle_steps=SETTLE_STEPS)
    assert state.step == 0
    assert state.scan is not None
    assert len(state.scan.filter_ranges) == int(loop.cfg["scanners"]["filter_bins"])
    assert len(state.scan.policy_ranges) == int(loop.cfg["scanners"]["policy_bins"])
    assert not state.stop_active and not state.in_contact
    assert np.allclose(state.velocity, 0.0)


def test_each_step_brings_one_fresh_scan_stamped_for_its_own_tick(loop):
    loop.reset(settle_steps=SETTLE_STEPS)
    times = []
    for _ in range(5):
        state = loop.step((0.0, 0.0, 0.0))
        times.append(state.scan.sim_time)
        assert state.scan.sim_time == pytest.approx(state.sim_time, abs=1e-6)
    dt = float(loop.cfg["sim"]["dt"])
    assert np.allclose(np.diff(times), dt, atol=1e-6), f"scan times {times}"


def test_the_applied_command_is_what_the_base_applied_not_what_was_asked(loop):
    loop.reset(settle_steps=SETTLE_STEPS)
    # Far beyond the box and the wheel budget: the base must report less than was asked.
    state = loop.step((5.0, 5.0, 5.0))
    assert state.intervened
    assert np.linalg.norm(state.applied[:2]) < 5.0
    limits = loop.base.limits
    assert abs(state.applied[0]) <= limits.v_max + 1e-9
    assert abs(state.applied[1]) <= limits.vy_max + 1e-9


def test_the_velocity_estimate_follows_the_motion_not_the_command(loop):
    loop.reset(settle_steps=SETTLE_STEPS)
    state = loop.reset(settle_steps=SETTLE_STEPS)
    for _ in range(12):
        state = loop.step(CREEP)
    assert state.velocity[0] > 0.01, f"estimated {state.velocity}"
    # The estimator is a filter over poses, so it lags the command rather than matching it.
    assert state.velocity[0] <= CREEP[0] + 0.02


def test_contact_is_checked_inside_the_loop_and_reads_clear_in_the_open(loop):
    state = loop.reset(settle_steps=SETTLE_STEPS)
    assert not state.in_contact
    for _ in range(3):
        assert not loop.step((0.0, 0.0, 0.0)).in_contact


def test_the_emulator_stops_the_robot_when_a_wall_enters_its_field(loop, cfg, sim_client):
    """The whole point of the layer: it sits beneath the controller, and the controller
    cannot drive through it."""
    # Where to put it: creeping at 0.05 m/s the robot advances 2.5 mm a step, and in the
    # lowest band (top 0.10 m/s) the field reaches e(0) + D(0.10) + margin = 0.2863 +
    # 0.015 + 0.10 = 0.401 m ahead. A wall whose near face sits at 0.44 m is therefore
    # ~40 mm beyond the field at the start and inside it within ~16 steps, while the
    # footprint (0.2854 m) stays 0.15 m clear of it throughout.
    loop.reset(settle_steps=SETTLE_STEPS)
    wall = place_wall(sim_client, cfg, ahead_m=0.45)
    try:
        states = [loop.step(CREEP) for _ in range(30)]
    finally:
        if bool(sim_client.sim.isHandle(wall)):
            sim_client.sim.removeObjects([wall])
    triggered = [s for s in states if s.certified["stop_triggered"]]
    stopped = [s for s in states if s.stop_active]
    assert len(triggered) == 1, "a stop is counted once, not once per step"
    assert stopped, "the stop never became active"
    assert np.allclose(stopped[-1].applied, 0.0, atol=1e-9), "commanded while stopped"
    assert loop.counters.protective_stops == 1
    assert not any(s.in_contact for s in states), "the robot reached the wall"


def test_the_report_carries_the_primary_metric_and_the_budget(loop):
    loop.reset(settle_steps=SETTLE_STEPS)
    for _ in range(10):
        loop.step(CREEP)
    report = loop.report()
    assert report["steps"] == 10
    assert report["distance_m"] > 0.0
    assert report["stops_per_100m"] == pytest.approx(
        100.0 * report["protective_stops"] / report["distance_m"])
    assert report["budget_ms"] == pytest.approx(1e3 * loop.budget_s)
    assert set(report["timings"]) >= {"whole", "scan", "step", "contact"}
    timings = ", ".join(f"{k} p95 {v['p95_ms']:.1f}" for k, v in report["timings"].items())
    print(f"\nstep timings (ms): {timings}")
    print(f"budget {report['budget_ms']:.0f} ms, over-budget steps "
          f"{report['over_budget_steps']}/{report['steps']}")


def _trace(loop, commands):
    loop.reset(settle_steps=SETTLE_STEPS)
    out = []
    for command in commands:
        s = loop.step(command)
        out.append((s.pose.copy(), s.velocity.copy(), s.applied.copy(),
                    np.asarray(s.scan.filter_ranges, dtype=float).copy(),
                    np.asarray(s.scan.policy_ranges, dtype=float).copy(),
                    bool(s.in_contact), bool(s.certified["stop_active"]),
                    bool(s.certified["stop_triggered"])))
    return out


def _identical(a, b):
    return len(a) == len(b) and all(
        all((np.array_equal(x, y) if isinstance(x, np.ndarray) else x == y)
            for x, y in zip(sa, sb, strict=True))
        for sa, sb in zip(a, b, strict=True))


def test_filter_off_is_bit_identical_with_and_without_the_pfb_slot(loop, monkeypatch):
    """Ruled 2026-10-03: L1 trains on a FROZEN environment, and WP2 may add the filter only behind
    its flag. Two runs in one process (determinism is claimed within one process): the loop as
    built, slot present and filter off, and the loop with the slot bypassed entirely -- the
    pre-WP2 path. Every recorded output of every step must match bit for bit. After WP2 this test
    keeps guarding: any filter-off change fails it and invalidates prior L1 runs.

    Positive control: a sequence differing in ONE command must not match, so the comparison is
    shown able to see a difference."""
    import pfbnav.envs.step_loop as step_loop_module

    rng = np.random.default_rng(7)
    commands = [tuple(v) for v in rng.uniform([-0.1, -0.1, -0.3], [0.2, 0.1, 0.3], (60, 3))]
    as_built = _trace(loop, commands)
    with monkeypatch.context() as m:
        m.setattr(step_loop_module, "pfb_stage", lambda requested, enabled, fn: requested)
        bypassed = _trace(loop, commands)
    assert _identical(as_built, bypassed)

    changed = list(commands)
    changed[30] = (0.0, 0.0, 0.0)
    assert not _identical(as_built, _trace(loop, changed))




# The invariant "the field is never sized below true motion" (2026-10-16/17, with its 10 um tolerance
# and its null test) was REPLACED on 2026-10-18 (A20): the supervisor ruled it tested the wrong
# quantity. Containment of every stop is tested in tests/test_containment_sim.py.
