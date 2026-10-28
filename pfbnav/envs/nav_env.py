"""The Gymnasium environment: one scenario per episode, the safety layers beneath. (WP1)

Everything it needs already exists and is tested on its own; this is the wiring, and it
holds no geometry, no reward arithmetic and no observation layout of its own:

    scenario       `pfbnav/world/scenarios.py`   the world and the task, with its identity
    layout         `pfbnav/world/layouts.py`     polygons, regenerated and verified per reset
    map            `pfbnav/world/baking.py`      what the planner is told (unmapped excluded)
    plan           `pfbnav/planning/astar.py`    a route, or a named failure
    tracking       `pfbnav/planning/lookahead.py` arc-length progress and the lookahead point
    scene          `pfbnav/sim/layout_shapes.py` the same polygons as shapes, checked
    step           `pfbnav/envs/step_loop.py`    sense, decide, act, measure
    observation    `pfbnav/envs/observation.py`  the ruled 82-D layout
    reward         `pfbnav/envs/reward.py`       the `reward:` block, with a breakdown

**An episode's reset**: stop, start (the robot returns to its scene pose), instantiate the
layout placed so the scenario's start lands where the robot already is, verify every shape
against its polygon (item 4), read the first scan, and check that the robot stands at the
scenario's start in the hall it was given (the step-0 check, added 2026-10-10). Shapes
created while running vanish at the next stop, so nothing accumulates (item 5). The reset's
wall clock is recorded from the first episode (item 5), and the unmapped obstacles' offsets
from the plan travel in `info` (item 3).

**The certified layer runs beneath the policy**, inside the step loop: the action is what
the policy asked for, and what reaches the wheels is what survived the PFB (when on), the
emulator, the box and the wheel budget. The reward's intervention term charges only the PFB's
change to the request, and nothing without the filter or on a step under the certified latch
(ruled 2026-10-24 and 2026-10-26); a protective stop is charged once, on its trigger.

**Frames.** The layout, the plan and the tracker work in the layout's own coordinates; the
simulator works in world coordinates. `self.transform` maps one to the other and is built
per episode by `transform_placing`.
"""
from __future__ import annotations

import time
from pathlib import Path
from typing import Any, ClassVar

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from pfbnav.envs.controller_input import ClassicalInput, RouteView, ScanStream
from pfbnav.envs.observation import (
    DropoutSpec,
    ObservationSpec,
    assemble,
    bounds,
    dropout,
    sample_blocked_arc,
)
from pfbnav.envs.reward import RewardWeights, episode_outcome, pfb_intervention, step_reward
from pfbnav.envs.route_view import remaining_route, world_to_body
from pfbnav.envs.step_loop import StepLoop
from pfbnav.planning.astar import OK, plan_from_config
from pfbnav.planning.lookahead import PathTracker
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.layout_shapes import LayoutShapes, start_pose_error, transform_placing, wrap
from pfbnav.world.baking import bake, planner_radii
from pfbnav.world.placement import surface_offset
from pfbnav.world.scenarios import Scenario, world_of

REPO_ROOT = Path(__file__).resolve().parents[2]


class ScenarioUnusable(RuntimeError):
    """A scenario cannot be run, and the reason is named rather than worked around."""


class NavEnv(gym.Env):
    """One episode = one scenario. Requires a connected `SimClient`."""

    metadata: ClassVar[dict[str, Any]] = {"render_modes": []}

    def __init__(self, client, cfg: dict[str, Any], scenarios, *,
                 include_lookahead: bool = True, dropout_spec: DropoutSpec | None = None,
                 training: bool = False, settle_steps: int = 20,
                 seed: int | None = None):
        if not scenarios:
            raise ValueError("an environment needs at least one scenario")
        self.client = client
        self.cfg = cfg
        self.scenarios = [s if isinstance(s, Scenario) else Scenario(**s) for s in scenarios]
        self.settle_steps = int(settle_steps)
        self.loop = StepLoop(client, cfg)
        self.shapes = LayoutShapes(client, cfg)
        self.obs_spec = ObservationSpec.from_config(cfg, include_lookahead=include_lookahead)
        self.weights = RewardWeights.from_config(cfg)
        obs_cfg = cfg["env"]["observation"]
        # The declared augmentation applies in TRAINING ONLY (supervisor 2026-09-26), and
        # `training` defaults to False so an evaluation or a gate cannot augment by omission.
        # Opting in is a decision; opting out must not require one.
        self.training = bool(training)
        self.dropout_spec = (DropoutSpec.from_config(cfg) if dropout_spec is None
                             else dropout_spec)
        self._blocked_arc = None
        self.proximity_threshold = float(obs_cfg["proximity_threshold"])
        self.ceilings = np.array([cfg["actuation"]["v_max"], cfg["actuation"]["vy_max"],
                                  cfg["actuation"]["w_max"]], dtype=float)
        self.dt = float(cfg["sim"]["dt"])       # s, one environment step
        self.circumscribed = float(FootprintTable.from_csv(
            REPO_ROOT / cfg["robot"]["footprint_table"]).circumscribed)
        self.start_tolerance_m = float(cfg["env"]["start_pose_tolerance_m"])
        self.start_tolerance_rad = float(cfg["env"]["start_pose_tolerance_rad"])

        low, high = bounds(self.obs_spec)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        self.seed_used = seed
        self._rng = np.random.default_rng(seed)
        self._installed = False
        self._episode = -1
        self.scenario: Scenario | None = None
        self.tracker: PathTracker | None = None
        self.transform = None
        self.reset_seconds = 0.0
        self.episode_info: dict[str, Any] = {}

    # ------------------------------------------------------------------
    def install(self) -> None:
        """Put the scanner script in the scene. Idempotent; needs the simulation stopped."""
        if not self._installed:
            if self.client.running:
                self.client.stop()
            self.loop.install()
            self._installed = True

    def close(self) -> None:
        self.shapes.remove()
        if self._installed:
            self.loop.remove()
            self._installed = False

    # ------------------------------------------------------------------
    def reset(self, *, seed: int | None = None, options: dict | None = None):
        """Build the next scenario's world and return the first observation."""
        if seed is not None:
            self._rng = np.random.default_rng(seed)
            self.seed_used = seed
        started = time.perf_counter()
        self.install()
        self._episode += 1
        # The episode's blocked arc is drawn here and held for the whole episode. The draw is
        # unconditional so the random stream does not depend on whether augmentation is on:
        # a run with training=False must produce the same scenarios as one with it on, or the
        # augmentation would silently change which worlds are visited (the failure
        # `world.layouts.generate` hit on 2026-09-23).
        self._blocked_arc = sample_blocked_arc(
            self.dropout_spec, self.obs_spec.policy_bins, self._rng)
        index = int(options["index"]) if options and "index" in options else (
            self._episode % len(self.scenarios))
        scenario = self.scenarios[index]

        # The mapped layout regenerated, plus the unmapped obstacles the scenario RECORDS: since
        # A13 they are placed relative to the route, so they travel with the scenario rather
        # than being redrawn from the seed. `world_of` checks the whole world's hash.
        try:
            layout = world_of(scenario, self.cfg)
        except ValueError as exc:
            raise ScenarioUnusable(str(exc)) from exc

        radii = planner_radii(self.cfg, circumscribed=self.circumscribed)
        baked = bake(layout.obstacles, layout.bounds,
                     float(self.cfg["planner"]["resolution"]), radii)
        plan = plan_from_config(baked, scenario.start[:2], scenario.goal, self.cfg)
        if plan.status != OK:
            raise ScenarioUnusable(f"scenario {scenario.seed}: the plan is {plan.status}")

        self.scenario, self.baked, self.layout, self.plan = scenario, baked, layout, plan
        self.tracker = PathTracker(plan.path_world, float(self.cfg["planner"]["lookahead"]),
                                   window=float(self.cfg["planner"]["track_window"]))

        # The layout is placed where the robot already stands, so nothing is teleported. The
        # pose is read AFTER the restart, inside `after_start`: read before it, it is the end
        # pose of the episode just finished, and every hall after the first was built around a
        # pose the robot no longer occupied (found 2026-10-10, docs/decision_log.md).
        self.shapes.remove()
        geometry_problems: list[dict[str, Any]] = []

        def build() -> None:
            self.transform = transform_placing(scenario.start, self._robot_world_pose())
            self.shapes.instantiate(layout.obstacles, self.transform)
            geometry_problems.extend(self.shapes.verify(layout.obstacles))

        state = self.loop.reset(settle_steps=self.settle_steps, after_start=build)
        if geometry_problems:
            raise ScenarioUnusable(
                f"scenario {scenario.seed}: the scene does not match the layout: "
                f"{geometry_problems[:3]}")
        # The step-0 check: the robot stands at the scenario's start, in the hall it was given.
        # A failure is the environment's, not the scenario's, so it raises.
        position_error, yaw_error = start_pose_error(self.transform, scenario.start, state.pose)
        if position_error > self.start_tolerance_m or yaw_error > self.start_tolerance_rad:
            raise RuntimeError(
                f"scenario {scenario.seed}: at step 0 the robot is {position_error:.4f} m and "
                f"{yaw_error:.4f} rad from the scenario's start (tolerance "
                f"{self.start_tolerance_m} m, {self.start_tolerance_rad} rad): the hall was not "
                "built around the pose the robot occupies")

        # To the nearest SURFACE, exactly (A13), as the scenario recorded them.
        offsets = [surface_offset(o.polygon, plan.path_world) for o in layout.unmapped]
        self.reset_seconds = time.perf_counter() - started
        self.episode_info = {
            "scenario_seed": scenario.seed, "split": scenario.split,
            "stratum": scenario.stratum, "clearance_m": scenario.clearance_m,
            "layout_sha256": scenario.layout_sha256,
            "scenario_sha256": scenario.scenario_sha256,
            "unmapped_count": len(layout.unmapped), "unmapped_offsets_m": offsets,
            "plan_length_m": plan.length_m, "plan_waypoints": len(plan.path_cells),
            "reset_seconds": self.reset_seconds, "shapes": self.shapes.report(),
            "start_position_error_m": position_error, "start_yaw_error_rad": yaw_error,
        }
        tracker_state = self.tracker.update(self._pose_in_layout(state.pose))
        info = {**self.episode_info,
                "controller_input": self._controller_input(state, tracker_state)}
        return self._observation(state, tracker_state), info

    # ------------------------------------------------------------------
    def step(self, action):
        """One environment step: the policy's action, scaled, filtered, applied, measured."""
        if self.scenario is None or self.tracker is None:
            raise RuntimeError("call reset() before step()")
        a = np.clip(np.asarray(action, dtype=float).ravel(), -1.0, 1.0)
        if a.size != 3:
            raise ValueError(f"an action is (a_vx, a_vy, a_wz), got {a.size} values")
        # Each axis scales by its OWN ceiling (the ruling of 2026-09-23): one shared speed
        # would let the same normalised magnitude ask for more lateral than forward motion.
        command = a * self.ceilings

        state = self.loop.step(command)
        tracker_state = self.tracker.update(self._pose_in_layout(state.pose))
        goal_distance = float(np.linalg.norm(
            np.asarray(self._pose_in_layout(state.pose))[:2] - np.asarray(self.scenario.goal)))
        reached = goal_distance <= self.weights.goal_tolerance
        terminated, truncated, outcome = episode_outcome(
            reached_goal=reached, in_contact=state.in_contact, steps=self.loop.counters.steps,
            weights=self.weights, stop_active=bool(state.certified["stop_active"]),
            restart_blocked=bool(state.certified["restart_blocked"]),
            breached=bool(state.certified["breached"]), at_rest=bool(state.certified["at_rest"]))
        reward, terms = step_reward(
            self.weights, progress_m=tracker_state.progress,
            stop_triggered=bool(state.certified["stop_triggered"]),
            # The PFB's change to the request only (ruled 2026-10-24); 0 with the filter off and on
            # latched steps (ruled 2026-10-26).
            intervention=pfb_intervention(command, state.pfb, self.ceilings),
            proximity=self._proximity(state), reached_goal=reached,
            in_contact=state.in_contact)

        info = {
            "outcome": outcome, "reward_terms": terms,
            "goal_distance_m": goal_distance, "cross_track_m": tracker_state.cross_track,
            "progress_m": tracker_state.progress, "s_m": tracker_state.s,
            "certified": dict(state.certified), "applied": state.applied.tolist(),
            "requested": state.requested.tolist(), "in_contact": state.in_contact,
            "contact_names": list(state.contact_names),
            "over_budget": state.over_budget, "step_seconds": state.timings_s["whole"],
            # The scan-fault fix (ruled 2026-10-23): this step's packet was fail-safed, and whether
            # this step's decision used one (the ring buffer dumps on both).
            "scan_fault": state.scan_fault, "decided_on_fault": state.decided_on_fault,
            # The PFB's decision this step (WP2); None with the filter off. Its compute time, s,
            # for the real-time criterion (ruled 2026-10-27: p95 per step <= the control period).
            "pfb": state.pfb, "pfb_seconds": state.timings_s.get("pfb"),

            "controller_input": self._controller_input(state, tracker_state),
        }
        if terminated or truncated:
            info["episode"] = {**self.episode_info, **self.loop.report(), "outcome": outcome}
        return self._observation(state, tracker_state), reward, terminated, truncated, info

    # ------------------------------------------------------------------
    def _observation(self, state, tracker_state) -> np.ndarray:
        ranges = np.asarray(state.scan.policy_ranges, dtype=float)
        valid = np.asarray(state.scan.policy_valid, dtype=bool)
        if self.training and self.dropout_spec.active:
            ranges, valid = dropout(ranges, valid, self._rng, self.dropout_spec,
                                    self._blocked_arc)
        return assemble(self.obs_spec, ranges=ranges, valid=valid, velocity=state.velocity,
                        lookahead_body=tracker_state.lookahead_body,
                        goal_body=tracker_state.goal_body,
                        stop_active=bool(state.certified["stop_active"]))

    def _controller_input(self, state, tracker_state) -> ClassicalInput:
        """What a classical controller may see this step (ruled 2026-10-16): the stop flag, the
        velocity estimate, the certified layer's 120-bin stream and the route ahead in the body
        frame. Built here because the route's body frame needs the pose, which the controller
        never gets (`pfbnav/envs/controller_input.py`)."""
        pose = self._pose_in_layout(state.pose)
        route = world_to_body(remaining_route(self.tracker.path, tracker_state.s), pose)
        ahead = world_to_body(np.vstack([tracker_state.lookahead_world,
                                         tracker_state.goal_world]), pose)
        scan = ScanStream.of(state.scan.filter_ranges, state.scan.filter_bearings,
                             state.scan.filter_valid)
        return ClassicalInput.of(
            stop_active=bool(state.certified["stop_active"]), velocity=state.velocity,
            scan=scan, route=RouteView.of(route, ahead[0], ahead[1], tracker_state.remaining),
            dt=self.dt)

    def _proximity(self, state) -> float:
        """Closeness to the nearest valid return, for the shaped variants (0 by default)."""
        if self.weights.k_proximity == 0.0:
            return 0.0
        from pfbnav.envs.reward import proximity_cost

        ranges = np.asarray(state.scan.filter_ranges, dtype=float)
        valid = np.asarray(state.scan.filter_valid, dtype=bool)
        nearest = float(ranges[valid].min()) if valid.any() else 0.0
        return proximity_cost(nearest, self.proximity_threshold)

    def _pose_in_layout(self, world_pose) -> tuple[float, float, float]:
        """A world (x, y, yaw) in layout coordinates: the frame the plan lives in."""
        inverse = self.transform.invert()
        xy = inverse.apply([[float(world_pose[0]), float(world_pose[1])]])[0]
        return float(xy[0]), float(xy[1]), inverse.apply_yaw(float(world_pose[2]))

    def _robot_world_pose(self) -> tuple[float, float, float]:
        """Where the robot stands now, in the BODY convention. Called once the simulation runs.

        `base_ref`'s own yaw is a quarter turn from chassis-forward (`robot.yaw_offset`), and
        `YouBotBase.pose()` returns the body yaw. Placing the layout against the raw
        base_ref yaw aligned the plan a quarter turn off, so the robot drove sideways
        relative to its own route: 0.000 m of progress and 0.201 m of cross-track over 25
        steps (measured 2026-09-25).
        """
        sim = self.client.sim
        ref = self.client.object(self.cfg["sim"]["objects"]["base_ref"])
        position = [float(v) for v in sim.getObjectPosition(ref, sim.handle_world)]
        yaw = float(sim.getObjectOrientation(ref, sim.handle_world)[2])
        return position[0], position[1], wrap(yaw + float(self.cfg["robot"]["yaw_offset"]))
