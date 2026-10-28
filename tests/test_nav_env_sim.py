"""The environment, driven against a real instance (`pytest -m sim`). (WP1)

An episode end to end: a generated scenario's world built in the scene, the ruled 82-D
observation coming back inside its own space, the safety layers beneath the policy, the
reward's terms adding up, and the episode ending with a named outcome. Also the two things
the supervisor asked to be watched from the first episode: the reset's wall clock, and that
nothing accumulates across resets (2026-09-23, items 4 and 5).

The scene is the pinned one and is left as it was found.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from pfbnav.envs.nav_env import NavEnv
from pfbnav.world.scenarios import make_scenario

pytestmark = pytest.mark.sim

REPO = Path(__file__).resolve().parents[1]
SEEDS = (1_000_005, 1_000_007)
SETTLE = 10


@pytest.fixture(scope="module")
def scenarios(cfg):
    out = []
    for seed in SEEDS:
        status, scenario, _layout = make_scenario(seed, cfg, split="tune", episode_rate=1.0)
        assert scenario is not None, f"seed {seed}: {status}"
        out.append(scenario)
    return out


@pytest.fixture
def env(cfg, sim_client, scenarios):
    environment = NavEnv(sim_client, cfg, scenarios, settle_steps=SETTLE, seed=0)
    try:
        yield environment
    finally:
        environment.close()
        sim_client.stop()
        sim_client.start(settle_steps=SETTLE)


def test_reset_returns_an_observation_inside_its_own_space(env):
    obs, info = env.reset()
    assert env.observation_space.contains(obs), "the observation left its declared space"
    assert obs.shape == (82,) and obs.dtype == np.float32
    assert info["stratum"] in {"narrow", "medium", "wide"}
    assert info["reset_seconds"] > 0.0
    print(f"\nreset {info['reset_seconds'] * 1e3:.0f} ms, stratum {info['stratum']}, "
          f"plan {info['plan_length_m']:.2f} m over {info['plan_waypoints']} waypoints, "
          f"{info['unmapped_count']} unmapped at {np.round(info['unmapped_offsets_m'], 2)} m")


def test_the_episode_info_carries_what_the_rulings_asked_for(env):
    """Item 3: the count and each unmapped obstacle's offset from the plan. Item 5: reset."""
    _obs, info = env.reset()
    assert len(info["unmapped_offsets_m"]) == info["unmapped_count"]
    assert info["unmapped_count"] >= 1, "these scenarios were built with episode_rate 1.0"
    assert info["layout_sha256"] and info["scenario_sha256"]
    assert info["shapes"]["alive"] == info["shapes"]["shapes"]
    assert "reset_seconds" in info


def test_a_step_returns_a_reward_whose_terms_add_up(env):
    env.reset()
    obs, reward, terminated, truncated, info = env.step([0.25, 0.0, 0.0])
    assert env.observation_space.contains(obs)
    assert reward == pytest.approx(sum(info["reward_terms"].values()))
    assert not terminated and not truncated
    assert info["outcome"] == "running"
    assert info["step_seconds"] > 0.0


def test_the_action_scales_by_each_axis_own_ceiling(env):
    env.reset()
    _obs, _r, _t, _tr, info = env.step([1.0, 1.0, 0.0])
    requested = np.asarray(info["requested"])
    assert requested[0] == pytest.approx(env.cfg["actuation"]["v_max"])
    assert requested[1] == pytest.approx(env.cfg["actuation"]["vy_max"])
    # What reaches the wheels is not what was asked: the box and the wheel budget bind.
    assert not np.allclose(requested, info["applied"])


def test_driving_along_the_plan_earns_progress(env):
    env.reset()
    gained = 0.0
    for _ in range(25):
        _obs, _reward, terminated, truncated, info = env.step([0.4, 0.0, 0.0])
        gained += info["progress_m"]
        if terminated or truncated:
            break
    print(f"\nprogress over 25 steps: {gained:.3f} m, cross-track {info['cross_track_m']:.3f} m")
    assert gained > 0.0, "the robot never advanced along its plan"


def follow(obs, env, speed=1.0):
    """A minimal tracker: translate toward the lookahead point, reading the OBSERVATION.

    The real controllers are WP3; this exists so a test can drive the plan at all. It uses
    the lookahead block (sin, cos of the bearing) exactly as a policy would, and never peeks
    at the simulator.
    """
    block = env.obs_spec.slices["lookahead"]
    _distance, sin_b, cos_b = obs[block]
    return np.array([speed * float(cos_b), speed * float(sin_b), 0.0])


def test_following_the_plan_advances_along_it(env):
    obs, _info = env.reset()
    gained = 0.0
    for _ in range(60):
        obs, _reward, terminated, truncated, info = env.step(follow(obs, env, 0.6))
        gained += info["progress_m"]
        if terminated or truncated:
            break
    print(f"\nfollowing the plan: {gained:.2f} m gained, cross-track "
          f"{info['cross_track_m']:.3f} m")
    assert gained > 0.5, "the tracker did not advance along the plan"
    assert info["cross_track_m"] < 0.5, "it wandered off its own route"


def test_a_reset_after_driving_builds_the_next_hall_around_the_robot(env):
    """Found 2026-10-10: the pose the hall was placed around was read before the restart, so
    every episode after the first was built around the previous episode's END pose. The reset
    now checks the step-0 pose itself and raises; this drives far enough that the old code would
    have been metres out. Positive control: the robot moved ten tolerances before the second
    reset, so a stale pose would show as an error ten times the limit. (Not "more than 0.5 m":
    on this scenario the certified layer stops the tracker at a shelf corner after 0.45 m,
    2026-10-13, and how far it gets is not what this test is about.)"""
    obs, _info = env.reset(options={"index": 0})
    begin = np.asarray(env._robot_world_pose()[:2])
    for _ in range(60):
        obs, _reward, terminated, truncated, _info = env.step(follow(obs, env, 0.6))
        if terminated or truncated:
            break
    moved = float(np.linalg.norm(np.asarray(env._robot_world_pose()[:2]) - begin))
    assert moved > 10 * env.start_tolerance_m, (
        f"the robot moved only {moved:.3f} m, so this cannot see a stale pose")
    _obs, info = env.reset(options={"index": 1})
    print(f"\nmoved {moved:.2f} m; next start error "
          f"{info['start_position_error_m'] * 1e3:.2f} mm, {info['start_yaw_error_rad']:.5f} rad")
    assert info["start_position_error_m"] <= env.start_tolerance_m
    assert info["start_yaw_error_rad"] <= env.start_tolerance_rad


def test_the_contact_target_set_is_the_hall_and_its_unmapped_obstacles(env, sim_client):
    """Ruled 2026-10-13: the contact metric concerns obstacles only. What the contact check can
    register against is every COLLIDABLE shape outside the robot (a collection honours the flag),
    and that must be exactly this episode's layout -- hall geometry and unmapped obstacles, all
    of them -- with no floor tile and nothing else from the scene in it."""
    from pfbnav.sim.scene_geometry import shapes_outside

    _obs, info = env.reset(options={"index": 0})
    sim = sim_client.sim
    robot = sim_client.object(env.cfg["sim"]["objects"]["robot_tree"])
    floor = sim_client.object(env.cfg["sim"]["objects"]["floor"])
    targets = {h for h in shapes_outside(sim, robot).values()
               if sim_client.special_flags(h)["collidable"]}
    layout = set(env.shapes.handles.values())
    assert len(layout) == len(env.layout.obstacles), "a layout obstacle has no shape"
    assert info["unmapped_count"] >= 1, "the set must be shown to contain unmapped obstacles"
    floor_shapes = {int(h) for h in sim.getObjectsInTree(floor, sim.sceneobject_shape, 0)}
    print(f"\n{len(targets)} contact targets; layout {len(layout)} "
          f"({info['unmapped_count']} unmapped); floor shapes {len(floor_shapes)}")
    assert not (targets & floor_shapes), "a floor tile is a contact target"
    assert targets == layout, (
        f"not in the layout: {[sim.getObjectAlias(h, 2) for h in targets - layout]}; "
        f"layout shapes the check cannot see: {[sim.getObjectAlias(h, 2) for h in layout - targets]}")


def test_the_certified_layer_sits_beneath_the_policy(env):
    """Following the plan into the unmapped obstacle must end in a stop, not a collision.

    Scenario 0's unmapped obstacle sits 0.03 m off the plan, 2.2 m along it: exactly the
    case the emulator exists for -- the planner routed through something it was never told
    about.
    """
    obs, info = env.reset(options={"index": 0})
    assert info["unmapped_count"] >= 1
    speeds, stopped, terminated, truncated = [], False, False, False
    for _ in range(200):
        obs, _reward, terminated, truncated, info = env.step(follow(obs, env, 0.6))
        if info["certified"]["stop_active"]:
            stopped = True
            speeds.append(float(np.linalg.norm(info["applied"][:2])))
            if len(speeds) >= 15:
                break
        if terminated or truncated:
            break
    print(f"\nstopped={stopped} outcome={info['outcome']} "
          f"s={info['s_m']:.2f} m certified={info['certified']}")
    assert stopped, f"the robot never stopped; it ended {info['outcome']}"
    assert not info["in_contact"], "it reached the obstacle before the layer acted"
    # The stop zeroes the COMMAND; the base then decelerates under its own acceleration
    # limit, as a real one does, so what must hold is that it slows and comes to rest.
    print(f"applied speed while stopped: {[round(s, 3) for s in speeds]}")
    assert speeds[-1] <= speeds[0], "it did not slow while stopped"
    assert speeds[-1] < 0.02, f"still moving at {speeds[-1]:.3f} m/s after {len(speeds)} steps"


def test_an_episode_ends_with_a_named_outcome(env):
    env.reset()
    outcome = None
    for _ in range(40):
        _obs, _reward, terminated, truncated, info = env.step([0.0, 0.0, 0.0])
        outcome = info["outcome"]
        if terminated or truncated:
            break
    assert outcome in {"running", "success", "collision", "timeout"}


def test_resets_do_not_accumulate_shapes_or_handles(env, sim_client):
    """Item 5, in the environment rather than in isolation."""
    sim = sim_client.sim
    counts: dict[int, list[int]] = {}
    for index in (0, 1, 0, 1):
        _obs, info = env.reset(options={"index": index})
        env.step([0.1, 0.0, 0.0])
        alive = len([h for h in sim.getObjectsInTree(sim.handle_scene,
                                                     sim.sceneobject_shape, 0)
                     if str(sim.getObjectAlias(h)).startswith("layout_")])
        # Scenarios differ in how many obstacles they carry, so each index is compared with
        # ITSELF across resets: a count that grows is accumulation, a count that differs
        # between scenarios is just two different worlds.
        counts.setdefault(index, []).append(alive)
        assert alive == info["shapes"]["shapes"], "the scene holds more than this layout"
    print(f"\nlayout shapes per scenario across resets: {counts}")
    for index, seen in counts.items():
        assert len(set(seen)) == 1, f"scenario {index} accumulated shapes: {seen}"


def test_two_scenarios_give_two_different_worlds(env):
    _obs, first = env.reset(options={"index": 0})
    _obs, second = env.reset(options={"index": 1})
    assert first["layout_sha256"] != second["layout_sha256"]
    assert first["scenario_seed"] != second["scenario_seed"]


def test_the_no_planner_ablation_runs_at_79_channels(cfg, sim_client, scenarios):
    environment = NavEnv(sim_client, cfg, scenarios, settle_steps=SETTLE,
                         include_lookahead=False)
    try:
        obs, _info = environment.reset()
        assert obs.shape == (79,)
        assert environment.observation_space.contains(obs)
        obs, _r, _t, _tr, _info = environment.step([0.2, 0.0, 0.0])
        assert obs.shape == (79,)
    finally:
        environment.close()
        sim_client.stop()
        sim_client.start(settle_steps=SETTLE)


def test_the_classical_input_agrees_with_the_observation(env):
    """Ruled 2026-10-16: classical controllers get a typed input built beside the observation.
    Its lookahead and goal are the observation's lookahead and goal channels in another form,
    its stop flag the observation's, and its scan the certified layer's 120-bin stream."""
    from pfbnav.envs.controller_input import ClassicalInput
    from pfbnav.envs.observation import BLOCKS, CERTIFIED, GOAL, LOOKAHEAD

    at = {}
    start = 0
    for name, width in BLOCKS:
        at[name] = start
        start += width
    obs, info = env.reset(options={"index": 0})
    for _ in range(5):
        inp = info["controller_input"]
        assert isinstance(inp, ClassicalInput)
        # The observation's distance channel is clipped at 1 (one lookahead) and float32; the
        # robot off the path sees the lookahead point slightly farther than the lookahead.
        scale = float(env.cfg["planner"]["lookahead"])
        dn, s, c = (float(v) for v in obs[at[LOOKAHEAD]:at[LOOKAHEAD] + 3])
        look = np.asarray(inp.route.lookahead)
        assert min(np.linalg.norm(look) / scale, 1.0) == pytest.approx(dn, abs=1e-6)
        assert np.allclose(look / np.linalg.norm(look), [c, s], atol=1e-6)
        _gd, gs, gc = obs[at[GOAL]:at[GOAL] + 3]
        assert np.allclose(inp.route.goal / np.linalg.norm(inp.route.goal), [gc, gs], atol=1e-6)
        assert inp.stop_active == bool(obs[at[CERTIFIED]])
        assert len(inp.scan.ranges) == int(env.cfg["scanners"]["filter_bins"])
        assert np.allclose(inp.route.points[0], 0.0, atol=0.06)   # starts near the robot
        obs, _r, _t, _tr, info = env.step(np.array([0.3, 0.0, 0.0]))
