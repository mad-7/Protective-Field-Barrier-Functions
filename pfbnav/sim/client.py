"""CoppeliaSim ZeroMQ Remote API client.

All simulator I/O for the project passes through this module. Failures are
raised, never swallowed: a missing object or a stepping-mode mismatch is a
scientific error, not something to paper over with a default value.

Body-frame and unit conventions are documented in pfbnav/sim/robot.py.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

# Tolerance when comparing the scene's physics step against configs dt.
_DT_TOL = 1e-9

# How long to wait for an instance to report that it has stopped, in seconds.
_STOP_TIMEOUT = 10.0

# CoppeliaSim's dynamics-engine identifiers. Engine dependence is a reported finding, so
# every summary, table and figure names the engine (ruling 6 on the D7/G1b report).
ENGINE_NAMES = {0: "Bullet", 1: "ODE", 2: "Vortex", 3: "Newton", 4: "MuJoCo"}


class SimError(RuntimeError):
    """Base class for simulator-side failures."""


class SimObjectNotFound(SimError):
    """A scene object named in the configuration does not exist."""


class SimTimestepMismatch(SimError):
    """The scene's physics step does not match `sim.dt` in the configuration."""


def host_from_endpoint(endpoint: str) -> str:
    """Extract the host from a ZeroMQ endpoint such as ``tcp://127.0.0.1``.

    Only the ``tcp`` transport is supported; anything else is an error rather
    than a silent default, because a mistyped endpoint would otherwise connect
    to the local default instance and produce plausible-looking results.
    """
    scheme, sep, rest = endpoint.partition("://")
    if not sep:
        raise ValueError(f"endpoint {endpoint!r} has no transport (expected 'tcp://<host>')")
    if scheme != "tcp":
        raise ValueError(f"unsupported transport {scheme!r} in endpoint {endpoint!r}")
    host = rest.split("/")[0]
    if ":" in host:
        raise ValueError(
            f"endpoint {endpoint!r} carries a port; the port comes from `sim.port` "
            "so that parallel instances can offset it"
        )
    if not host:
        raise ValueError(f"endpoint {endpoint!r} has an empty host")
    return host


def port_for_instance(base_port: int, instance_id: int, stride: int = 1) -> int:
    """Port of the CoppeliaSim instance `instance_id`: base + stride * id (configs/base.yaml).

    A stride of 2 leaves the port after each instance's own free, for ZeroMQ add-on
    versions that open a second port next to the one they are given.
    """
    if instance_id < 0:
        raise ValueError(f"instance_id must be >= 0, got {instance_id}")
    if stride < 1:
        raise ValueError(f"port stride must be >= 1, got {stride}")
    return base_port + stride * instance_id


# Attempts at the display setting before a start is recorded as a refusal
# (supervisor 2026-09-26: retry rather than hunt the cause).
DISPLAY_ATTEMPTS = 3


@dataclass(frozen=True)
class SimConnection:
    """Everything needed to reach one CoppeliaSim instance."""

    host: str
    port: int
    dt: float
    engine: int              # dynamics engine, frozen by D7 (see configs/base.yaml)
    dynamic_step: float      # s, dynamics substep, frozen by D7
    display_enabled: bool | None = None   # None leaves the instance's own setting alone
    # s, the longest any single remote call may wait for its reply before it raises. None waits
    # forever -- only for unit tests that never reach an instance; `from_config` always sets it.
    reply_timeout: float | None = None

    @classmethod
    def from_config(cls, cfg: dict[str, Any], instance_id: int = 0) -> SimConnection:
        sim_cfg = cfg["sim"]
        display = sim_cfg.get("display_enabled")
        return cls(
            host=host_from_endpoint(sim_cfg["endpoint"]),
            port=port_for_instance(int(sim_cfg["port"]), instance_id,
                                   int(sim_cfg["port_stride"])),
            dt=float(sim_cfg["dt"]),
            engine=int(sim_cfg["physics"]["engine"]),
            dynamic_step=float(sim_cfg["physics"]["dynamic_step"]),
            display_enabled=None if display is None else bool(display),
            reply_timeout=float(sim_cfg["reply_timeout_s"]),
        )


class SimClient:
    """Stepped connection to one CoppeliaSim instance.

    The client owns the simulation lifecycle (start / step / stop) and handle
    resolution. It deliberately exposes the raw ``sim`` remote object so that
    the robot, scanner, and contact wrappers can call the API directly; those
    wrappers are the only other places allowed to do so.
    """

    def __init__(self, connection: SimConnection, *, verbose: bool = False):
        self.connection = connection
        self.verbose = verbose
        self._client = None
        self.sim: Any = None
        self._running = False
        # What rendering is actually doing, once start() has applied the configuration,
        # how often the instance refused the setting, and whether it held throughout.
        self.display_in_force: bool | None = None
        self.display_refusals = 0
        self.display_retries = 0
        self.display_consistent = True
        self.display_starts = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def connect(self) -> SimClient:
        """Open the ZeroMQ connection and switch the instance to stepping mode."""
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient

        self._client = RemoteAPIClient(host=self.connection.host, port=self.connection.port)
        if self.connection.reply_timeout is not None:
            self._guard_replies(self._client, self.connection.reply_timeout)
        self.sim = self._client.require("sim")
        self.sim.setStepping(True)
        return self

    def _guard_replies(self, client, timeout: float) -> None:
        """Make every remote call raise `SimError` when the instance does not reply in time.

        The ZeroMQ client waits on `socket.recv()` with no timeout, so an instance that stops
        answering hangs the caller forever, silently: the B1 evaluation of 2026-10-17 stalled
        after episode 51 while Windows ran out of commit memory, with no error and no summary
        (hard rule 9). Every call goes through `client.call` (the `sim` object's functions look
        it up at call time), so wrapping it covers them all.
        """
        import zmq

        if timeout <= 0.0:
            raise ValueError(f"reply_timeout must be positive, got {timeout}")
        ms = round(timeout * 1000.0)
        client.socket.setsockopt(zmq.RCVTIMEO, ms)
        client.socket.setsockopt(zmq.SNDTIMEO, ms)
        client.socket.setsockopt(zmq.LINGER, 0)
        call = client.call
        where = f"{self.connection.host}:{self.connection.port}"

        def guarded(func, args):
            try:
                return call(func, args)
            except zmq.error.Again as exc:
                raise SimError(
                    f"the instance on {where} did not reply to {func!r} within {timeout} s "
                    "(not responding, crashed, or the host out of memory); the connection is "
                    "unusable from here") from exc

        client.call = guarded

    def load_scene(self, scene_path: str) -> None:
        """Load a scene file. The simulation must be stopped."""
        if self._running:
            raise SimError("stop the simulation before loading a scene")
        self.sim.loadScene(str(scene_path))

    def start(self, *, settle_steps: int = 0) -> None:
        """Stop any stale run, apply the configured physics, and start.

        The engine and substep come from the configuration, not the scene: D7 froze them,
        and the scene file still carries its original Bullet setting, so a script that
        trusted the scene would silently run on the wrong engine.

        `settle_steps` steps are taken with no command applied, which lets the
        chassis rest on its suspension before the first measurement.
        """
        self.sim.stopSimulation()
        self._await_stopped()
        self._running = False
        self.set_physics(engine=self.connection.engine,
                         dynamic_step=self.connection.dynamic_step)
        self.sim.setFloatParam(self.sim.floatparam_simulation_time_step, self.connection.dt)
        if self.connection.display_enabled is not None:
            self.apply_display(self.connection.display_enabled)
        self.sim.setStepping(True)
        self.sim.startSimulation()
        self._running = True
        self.assert_timestep()
        for _ in range(settle_steps):
            self.step()

    def apply_display(self, enabled: bool) -> bool:
        """Switch the instance's rendering on or off; returns what is in force.

        Rendering costs ~7 ms of a ~27 ms step and changes no physics (measured
        2026-09-19; `scripts/regress_display_off.py` checks the trajectories are
        bit-identical), so training runs with it off and a debugging session turns it
        back on through `sim.display_enabled`.

        A headless instance draws nothing and has no display to switch, so the request is
        recorded as "off" rather than pretended to have been applied. Otherwise the
        setting is read back: a write that silently fails is not a setting (hard rule 13).

        **A refusal is retried, then counted and reported, never raised.** Rendering costs
        time and must cost nothing else, so a run that renders when it was asked not to is a
        slower run, not a wrong one -- and raising would throw away an hour-long gate over a
        display setting, as it did on 2026-09-20 at run 53 of 66.

        The supervisor ruled on 2026-09-26 not to hunt the cause but to retry: the refusal is
        intermittent (5, 3 and 4 starts of 66 across three instances on 2026-09-26), so a
        second attempt usually takes. `display_retries` counts attempts beyond the first and
        `display_refusals` counts the starts that were still wrong after the last attempt --
        so a retried-and-fixed start costs a retry, not a refusal, and the two numbers say
        which happened. `display_in_force` says what actually held, and a session whose
        display state changed part-way is flagged by `display_consistent`.
        """
        self.display_starts += 1
        headless = bool(self.sim.getBoolParam(self.sim.boolparam_headless))
        if headless:
            self._record_display(False)
            return False
        want = bool(enabled)
        applied = want
        for attempt in range(DISPLAY_ATTEMPTS):
            if attempt:
                self.display_retries += 1
            self.sim.setBoolParam(self.sim.boolparam_display_enabled, want)
            applied = bool(self.sim.getBoolParam(self.sim.boolparam_display_enabled))
            if applied == want:
                break
            print(f"WARNING: asked for display_enabled={want} on {self.connection.host}:"
                  f"{self.connection.port}, the instance reports {applied} "
                  f"(attempt {attempt + 1} of {DISPLAY_ATTEMPTS})")
        if applied != want:
            self.display_refusals += 1
            print(f"WARNING: rendering stayed {'on' if applied else 'off'} after "
                  f"{DISPLAY_ATTEMPTS} attempts (refusal {self.display_refusals}). "
                  "The run records what was in force.")
        self._record_display(applied)
        return applied

    def display_status(self) -> dict[str, object]:
        """What rendering did over this session, for the run's provenance.

        `starts` is how many times the setting was applied, so a refusal count can be
        read against the number of chances to refuse: the trigger for diagnosing the
        unexplained mid-run flips is a count that rises with run length, or one that
        becomes fatal (supervisor, 2026-09-20). `retries` separates "refused once and took
        on the retry" from "refused for good" (supervisor, 2026-09-26); escalation to a
        one-day diagnosis is triggered by refusals above 10 % of starts, not by retries.
        """
        starts = self.display_starts
        return {"in_force": self.display_in_force, "refusals": self.display_refusals,
                "consistent": self.display_consistent, "starts": starts,
                "retries": self.display_retries,
                "refusal_fraction": (self.display_refusals / starts) if starts else None}

    def _record_display(self, in_force: bool) -> None:
        """Track the display state and whether it has held for the whole session."""
        if self.display_in_force is not None and in_force != self.display_in_force:
            self.display_consistent = False
        self.display_in_force = in_force

    def assert_timestep(self) -> float:
        """Raise unless the running scene steps at the configured `sim.dt`."""
        dt = float(self.sim.getSimulationTimeStep())
        if abs(dt - self.connection.dt) > _DT_TOL:
            raise SimTimestepMismatch(
                f"scene physics step is {dt} s but configs set sim.dt = {self.connection.dt} s; "
                "the acceleration limiter and every time-based metric assume they are equal"
            )
        return dt

    def step(self) -> None:
        self.sim.step()

    def stop(self) -> None:
        if self._running:
            self.sim.stopSimulation()
            self._await_stopped()
            self._running = False

    def _await_stopped(self) -> None:
        """Block until the instance reports `simulation_stopped`, or raise.

        A simulation that never stops would otherwise hang a training run
        silently; the timeout turns it into a failure that gets logged.
        """
        deadline = time.monotonic() + _STOP_TIMEOUT
        while self.sim.getSimulationState() != self.sim.simulation_stopped:
            if time.monotonic() > deadline:
                raise SimError(
                    f"instance on {self.connection.host}:{self.connection.port} did not "
                    f"reach the stopped state within {_STOP_TIMEOUT} s"
                )
            time.sleep(0.01)

    @property
    def running(self) -> bool:
        return self._running

    def time(self) -> float:
        """Simulation time in seconds."""
        return float(self.sim.getSimulationTime())

    def __enter__(self) -> SimClient:  # noqa: PYI034 - typing.Self needs Python 3.11
        return self.connect()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    # ------------------------------------------------------------------
    # Handles
    # ------------------------------------------------------------------
    def object(self, path: str) -> int:
        """Resolve a scene object path, or raise SimObjectNotFound.

        The remote API raises a generic exception for a missing object; it is
        translated here so that a scene/config mismatch names the path that is
        wrong instead of surfacing as an opaque RuntimeError deep in a rollout.
        """
        try:
            return int(self.sim.getObject(path))
        except Exception as exc:
            raise SimObjectNotFound(
                f"scene object {path!r} not found on {self.connection.host}:"
                f"{self.connection.port}; check the `sim.objects` entries in the config "
                "against the loaded scene"
            ) from exc

    def collection_of_everything_except(self, *tree_roots: int) -> int:
        """Collection holding every object except the subtrees under `tree_roots`.

        Used as the obstacle side of the ground-truth contact check: it keeps the
        scene furniture in and takes the robot's own subtree out, so self-contact
        cannot be mistaken for a collision. The floor is excluded only for the
        wheels, which rest on it. Collections created while a simulation runs are
        destroyed when it stops, so this must be called after `start()`.
        """
        collection = int(self.sim.createCollection(0))
        self.sim.addItemToCollection(collection, self.sim.handle_all, -1, 0)
        for root in tree_roots:
            self.sim.addItemToCollection(collection, self.sim.handle_tree, int(root), 1)
        return collection

    def entity_for(self, handles) -> int:
        """A `checkCollision` entity for a set of objects; the handle itself if one.

        A collection holding exactly one object does not behave like that object: on
        2026-09-17 a one-item collection wrapping the chassis reported no contact at
        poses where the same handle, passed directly, reported contact, and where the
        chassis was a member of a larger collection that did report it. Multi-item
        collections are unaffected. Passing the handle through is also the plainer
        call, so the case is simply avoided rather than relied upon.
        """
        handles = [int(h) for h in handles]
        if not handles:
            raise ValueError("cannot build a collision entity from no objects")
        if len(handles) == 1:
            return handles[0]
        collection = int(self.sim.createCollection(0))
        for handle in handles:
            self.sim.addItemToCollection(collection, self.sim.handle_single, handle, 0)
        return collection

    def shapes_in_tree(self, root: int) -> list[int]:
        """Handles of every shape in the subtree under `root`, including `root`."""
        return [int(h) for h in
                self.sim.getObjectsInTree(int(root), self.sim.object_shape_type, 0)]

    def physics_settings(self) -> dict[str, float | int]:
        """The simulator's physics configuration, for recording in a run's provenance.

        D7 freezes a physics configuration before gate G1b and forbids tuning afterwards.
        A frozen setting that is not recorded is not frozen, and these live in the scene
        rather than in `configs/`, so every run captures what was actually in force.
        """
        engine = int(self.sim.getInt32Param(self.sim.intparam_dynamic_engine))
        return {
            "engine": engine,
            "engine_name": ENGINE_NAMES.get(engine, f"unknown({engine})"),
            "simulation_time_step": float(
                self.sim.getFloatParam(self.sim.floatparam_simulation_time_step)),
            "dynamic_step_size": float(
                self.sim.getFloatParam(self.sim.floatparam_dynamic_step_size)),
        }

    def set_physics(self, *, engine: int | None = None,
                    dynamic_step: float | None = None) -> None:
        """Set the dynamics engine and substep size. The simulation must be stopped.

        Exists so the D7 probe can sweep configurations from a script rather than by hand
        in the GUI: a swept axis that has to be set manually between runs is one that
        quietly does not get swept. Friction and mass are *not* settable here -- CoppeliaSim
        4.10 moved those to a properties API whose names are not established in this
        project -- so those axes remain manual and must be recorded by hand.

        Engine identifiers are CoppeliaSim's own: 0 Bullet, 1 ODE, 2 Vortex, 3 Newton,
        4 MuJoCo. Not every build ships every engine; an unavailable one fails here rather
        than silently running on the previous engine and producing a result attributed to
        the wrong one.
        """
        if self._running:
            raise SimError("stop the simulation before changing the physics configuration")
        if engine is not None:
            self.sim.setInt32Param(self.sim.intparam_dynamic_engine, int(engine))
            applied = int(self.sim.getInt32Param(self.sim.intparam_dynamic_engine))
            if applied != int(engine):
                raise SimError(
                    f"asked for dynamics engine {engine} but the instance reports {applied}; "
                    "this build may not provide it, and a run attributed to the wrong "
                    "engine is worse than no run"
                )
        if dynamic_step is not None:
            self.sim.setFloatParam(self.sim.floatparam_dynamic_step_size, float(dynamic_step))
            applied_step = float(self.sim.getFloatParam(self.sim.floatparam_dynamic_step_size))
            if abs(applied_step - float(dynamic_step)) > 1e-9:
                raise SimError(
                    f"asked for a {dynamic_step} s dynamics step but the instance reports "
                    f"{applied_step} s"
                )

    def special_flags(self, handle: int) -> dict[str, bool]:
        """An object's collidable, measurable and detectable flags.

        These decide what `checkCollision` and `checkDistance` see *through a
        collection*: measured on 2026-09-18 (tests/test_collection_semantics.py), a
        collection skips an object whose collidable flag is off, while the same object
        passed as a single handle is checked regardless. What counts as a collision is
        therefore partly set in the scene, and every G0 run records it.
        """
        props = int(self.sim.getObjectSpecialProperty(int(handle)))
        return {name: bool(props & int(getattr(self.sim, f"objectspecialproperty_{name}")))
                for name in ("collidable", "measurable", "detectable")}

    def set_collidable(self, handle: int, value: bool) -> None:
        """Set an object's collidable flag, and verify it took. Simulation must be stopped."""
        if self._running:
            raise SimError("stop the simulation before changing an object's flags")
        bit = int(self.sim.objectspecialproperty_collidable)
        props = int(self.sim.getObjectSpecialProperty(int(handle)))
        self.sim.setObjectSpecialProperty(int(handle), (props | bit) if value else (props & ~bit))
        if self.special_flags(handle)["collidable"] != value:
            raise SimError(f"collidable flag of object {handle} did not change to {value}")

    def object_bbox(self, handle: int) -> tuple[list[float], list[float]]:
        """An object's bounding-box corners in its own frame, as (min_xyz, max_xyz) in m.

        The object's own geometry, not the model's: `objbbox` rather than `modelbbox`, so
        asking the robot's base shape returns the chassis and not the whole vehicle.
        """
        axes = ("x", "y", "z")
        lo = [float(self.sim.getObjectFloatParam(
            int(handle), getattr(self.sim, f"objfloatparam_objbbox_min_{a}"))) for a in axes]
        hi = [float(self.sim.getObjectFloatParam(
            int(handle), getattr(self.sim, f"objfloatparam_objbbox_max_{a}"))) for a in axes]
        return lo, hi
