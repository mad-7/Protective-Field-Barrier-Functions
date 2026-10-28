"""One environment step against the simulator: sense, decide, act, measure. (WP1, slice 2)

This is the loop the Gymnasium environment will drive and the context `preregistration/
gates.yaml` calls `env_step_loop`. It owns the simulator-facing objects and the order they
run in; it owns no task, no reward and no observation, so it can be exercised -- and G0 can
run inside it -- before either exists.

**The order, and why it is this order**

  0. The **PFB** (WP2) filters the controller's command, beneath the policy and above the
     certified layer. Its slot exists from 2026-10-03, before WP2, so that the environment L1
     trains on is frozen: off, the slot returns the command array ITSELF -- no copy, no
     arithmetic -- and WP2 fills only the "on" branch (`pfb_stage`; supervisor's ruling 3 of
     2026-10-03). On (WP2 step 4, 2026-10-24): `pfbnav.safety.pfb.PFB` on the scan in hand (the
     filter stream, exact bearings and validity), the velocity ESTIMATE, the command the base
     last applied and the layer's latch -- the same inputs the certified layer decides on.
  1. The **certified layer decides on the scan already in hand** -- the world as last
     sensed, which is all a real controller has. Sensing happens after a step's physics, so
     the packet held at the top of step k is the world after step k-1.
  2. A **stop overrides the command** (`CertifiedLayer.override`) before anything is
     applied, so the emulator sits beneath every controller, learned or not, exactly as the
     project's contribution requires.
  3. The base **clamps, projects into the wheel budget and rate-limits** what remains
     (`YouBotBase.command_body_velocity`), and reports what it actually applied. (The reward's
     intervention term compares the request with the PFB's output instead; ruled 2026-10-24.)
  4. One **physics step**.
  5. **Sensing**: the scan packet in one round trip (36 + 120 bins with bearings and
     validity, `lua/scanner_v2.lua`), the pose, and the ground-truth contact check. Contact
     is `sim.checkCollision` and nothing else (hard rule 4).
  6. The **velocity estimate** is updated from the new pose (`PoseDifferenceEstimator` over
     `youBot_ref`; never the command, never wheel odometry), and is what the next step's
     band selection uses.

**What it measures.** Wall-clock for the whole step and for each part, against the budget
derived from `env.step_budget` (`pfbnav/eval/step_budget.py`). A step over budget is
counted and carried in the state, never raised: the budget is a compute requirement on the
training host, and this machine is not it (`env.step_budget.over_budget_is_fatal`, false).

**What it does not do.** No episode, no goal, no reward, no reset of the robot's pose: those
belong to the task layer, which needs the scenarios and the map that WP1 still owes. `reset`
here means "the loop's own state, and the simulator running and settled".
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.eval.step_budget import per_step_budget_s
from pfbnav.safety import pfb_field
from pfbnav.safety.certified import CertifiedLayer
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.safety.pfb import PFB, PFBConfig, PFBResult
from pfbnav.sim.client import SimClient
from pfbnav.sim.contact import RobotContact
from pfbnav.sim.robot import YouBotBase
from pfbnav.sim.scan_guard import ScanGuard, all_unobserved
from pfbnav.sim.scan_packet import ScanPacket
from pfbnav.sim.scanner import Scanner
from pfbnav.sim.velocity_estimate import PoseDifferenceEstimator, VelocityEstimateConfig

REPO_ROOT = Path(__file__).resolve().parents[2]


def pfb_stage(requested: np.ndarray, enabled: bool, filter_fn) -> np.ndarray:
    """The command the certified layer receives. Disabled: `requested` itself, the same object,
    and `filter_fn` is never called -- which is what keeps filter-off behaviour bit-identical
    whatever WP2 puts behind the flag. Enabled: whatever the filter returns."""
    if not enabled:
        return requested
    return np.asarray(filter_fn(requested), dtype=float)


@dataclass
class StepState:
    """Everything one step produced. Frames: body; SI units; bearings in (-pi, pi]."""

    step: int
    sim_time: float                 # s, the simulator's clock after this step
    pose: np.ndarray                # (x, y, yaw) of youBot_ref, world frame
    velocity: np.ndarray            # (vx, vy, wz) body frame, ESTIMATED
    requested: np.ndarray           # (vx, vy, wz) asked for by the controller
    applied: np.ndarray             # (vx, vy, wz) actually written to the wheels
    scan: ScanPacket                # the world as sensed AFTER this step's physics
    in_contact: bool
    contact_names: tuple[str, ...]
    certified: dict[str, Any]       # CertifiedLayer.step's report for this step
    timings_s: dict[str, float]
    over_budget: bool
    # The scan-fault fix (ruled 2026-10-23): this step's packet failed the plausibility guard
    # through every re-render and was replaced by the fail-safe (all unobserved); how many
    # re-renders it took; and whether THIS step's certified decision used such a packet (the
    # previous step's), so a stop it triggered is a scan-fault stop.
    scan_fault: bool = False
    scan_rerenders: int = 0
    decided_on_fault: bool = False
    # The PFB's decision this step (WP2): mode, regime, infeasible, reason, QP solves, the field
    # heading and |yaw rate| its rows assumed, and the nominal it filtered. None with the filter off.
    pfb: dict[str, Any] | None = None

    @property
    def stop_active(self) -> bool:
        return bool(self.certified["stop_active"])

    @property
    def intervened(self) -> bool:
        """Whether what was applied differs from what was asked for, for any reason."""
        return not np.allclose(self.requested, self.applied)


@dataclass
class LoopCounters:
    """What the loop counts over an episode; the env turns these into metrics."""

    steps: int = 0
    protective_stops: int = 0
    at_rest_triggers: int = 0       # stops triggered at rest (creep included)
    stationary_triggers: int = 0    # ... with the robot stationary: a construction gap (static world)
    stopped_steps: int = 0
    contact_steps: int = 0
    over_budget_steps: int = 0
    warning_steps: int = 0
    # Entries into the warning field, the SECONDARY near-miss metric (supervisor
    # 2026-09-26). Entries, not steps: `warning_steps` measures how long the robot dwelt
    # near something, which is a different quantity and not a count of near misses.
    warning_entries: int = 0
    invalid_bin_steps: int = 0
    # The scan-fault fix (ruled 2026-10-23): packets replaced by the fail-safe, re-renders made,
    # and stops decided on a fail-safed packet -- excluded from H1's primary stop count.
    scan_faults: int = 0
    scan_rerenders: int = 0
    scan_fault_stops: int = 0
    # The PFB (WP2): steps on which it fell back to maximum braking with no command satisfying its
    # CBF (reported per method and stratum, Q3), and steps on which it changed the command.
    pfb_infeasible: int = 0
    pfb_modified: int = 0
    distance_m: float = 0.0
    timings_s: dict[str, list[float]] = field(default_factory=dict)

    def stops_per_100m(self) -> float | None:
        """The primary metric. None until the robot has moved: 0/0 is not zero stops. Stops
        decided on a scan-faulted packet are excluded (ruled 2026-10-23) and reported apart."""
        if self.distance_m <= 0.0:
            return None
        return 100.0 * (self.protective_stops - self.scan_fault_stops) / self.distance_m

    def warnings_per_100m(self) -> float | None:
        """The secondary near-miss metric, on the same footing as the primary one."""
        if self.distance_m <= 0.0:
            return None
        return 100.0 * self.warning_entries / self.distance_m


class StepLoop:
    """The simulator-facing half of the environment: one step in, one `StepState` out."""

    def __init__(self, client: SimClient, cfg: dict[str, Any]):
        # The PFB's slot (WP2). Off, nothing is built and `_filter` is never called.
        self.filter_enabled = bool(cfg["filter"]["enabled"])
        self.client = client
        self.cfg = cfg
        self.dt = float(cfg["sim"]["dt"])
        self.scanner = Scanner(client, cfg)
        self.base = YouBotBase.from_config(client, cfg)
        self.contact = RobotContact.from_config(client, cfg)
        table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
        self.pfb: PFB | None = None
        if self.filter_enabled:
            field_ = pfb_field.PFBField(table.extent, pfb_field.field_params_from_config(cfg))
            self.pfb = PFB(field_, PFBConfig.from_config(cfg, field_))
        self._pfb_result: PFBResult | None = None
        self._stop_active = False          # the layer's latch after the previous step
        from pfbnav.world.strata import certified_params_from_config
        self.certified = CertifiedLayer(
            certified_params_from_config(cfg), float(cfg["robot"]["half_length"]),
            float(cfg["robot"]["half_width"]), table.extent)
        self.velocity = PoseDifferenceEstimator(VelocityEstimateConfig.from_config(cfg))
        # `certified.enabled` decides whether the emulator may OVERRIDE the command. It is
        # always evaluated and always reported: G0 has to drive a case into contact to
        # validate the contact check, and a layer that stopped the robot first would make
        # the gate untestable -- but a run must still say what the layer would have done
        # (2026-09-25). Off is for that gate and for a declared ablation, never for a result.
        self.certified_enabled = bool(cfg["certified"]["enabled"])
        budget = cfg["env"]["step_budget"]
        self.budget_s = per_step_budget_s(float(budget["aggregate_steps_per_s"]),
                                          int(budget["instances"]))
        self.over_budget_is_fatal = bool(budget["over_budget_is_fatal"])
        self.counters = LoopCounters()
        self.packet: ScanPacket | None = None
        self.guard = ScanGuard.from_config(cfg)
        self._packet_fault = False
        self._step = 0
        self._last_xy: np.ndarray | None = None
        # The estimator returns its estimate from update(); the loop keeps the latest so
        # the emulator selects its band from the same value the state reports.
        self._velocity = np.zeros(3)

    # ------------------------------------------------------------------
    def install(self) -> None:
        """Put the scanner script in place. Before `client.start()`: it initialises there."""
        self.scanner.install()

    def remove(self) -> None:
        """Take the scanner out again, leaving the instance as it was found."""
        self.scanner.remove()

    def reset(self, *, settle_steps: int = 20, after_start=None) -> StepState:
        """Start (or restart) the simulation, settle the base, and take the first scan.

        `after_start` runs once the simulation is running and before the first scan is read:
        the environment builds its layout there, so the first observation already contains
        it. Shapes created while running vanish at the next stop, which is the episode's
        cleanup. The robot's pose is not touched: placing it is the task layer's business.
        """
        if self.client.running:
            self.client.stop()
        self.client.start(settle_steps=settle_steps)
        if after_start is not None:
            after_start()
        self.base.bind()
        self.base.reset_command_state()
        self.contact.bind()
        self.velocity.reset()
        self._velocity = np.zeros(3)
        self.certified.reset()
        if self.pfb is not None:
            self.pfb.reset()
        self._pfb_result = None
        self._stop_active = False
        self.counters = LoopCounters()
        self._step = 0
        self._last_xy = None
        self.client.step()
        self.packet = self.scanner.read(sim_time=self.client.time())
        self.guard.reset(self.packet)
        self._packet_fault = False
        pose = np.asarray(self.base.pose(), dtype=float)
        self._last_xy = pose[:2].copy()
        reading = self.contact.check()
        return StepState(
            step=0, sim_time=self.client.time(), pose=pose, velocity=np.zeros(3),
            requested=np.zeros(3), applied=np.zeros(3), scan=self.packet,
            in_contact=bool(reading.in_contact), contact_names=tuple(reading.names),
            certified={"stop_active": False, "stop_triggered": False, "field_id": 0,
                       "warning": False, "breached": False, "at_rest": True, "speed": 0.0,
                       "enabled": self.certified_enabled},
            timings_s={}, over_budget=False)

    # ------------------------------------------------------------------
    def step(self, command) -> StepState:
        """Run one environment step. `command` is a body-frame (vx, vy, wz) request."""
        if self.packet is None:
            raise RuntimeError("call reset() before step()")
        requested = np.asarray(command, dtype=float).ravel()
        if requested.shape != (3,):
            raise ValueError(f"command must be (vx, vy, wz), got shape {requested.shape}")
        timings: dict[str, float] = {}
        whole = time.perf_counter()

        # 0. The PFB's slot. Off: `nominal` IS `requested` (see `pfb_stage`).
        if self.filter_enabled:
            mark = time.perf_counter()
        nominal = pfb_stage(requested, self.filter_enabled, self._filter)
        if self.filter_enabled:
            timings["pfb"] = time.perf_counter() - mark

        # 1-2. The emulator decides on the world as last sensed, and overrides the command.
        mark = time.perf_counter()
        # The requested command carries the INTENDED travel direction, which the restart
        # condition needs (supervisor 2026-09-27): a stopped robot may be approached, but may
        # not drive toward an intrusion. The command is used for direction only -- the field
        # and the band still come from the estimated velocity.
        # The field is never sized below true motion (ruled 2026-10-16): the band and the
        # rotation sweep take the larger of the estimate and the command the base is executing
        # (applied after the limiter on the previous step).
        decided_on_fault = self._packet_fault
        certified = self.certified.step(
            self.client.time(), self._velocity, self.packet.filter_ranges,
            self.packet.filter_bearings, self.packet.filter_valid,
            intended_direction=nominal, applied=self.base.applied_command)
        allowed = self.certified.override(nominal) if self.certified_enabled else nominal
        timings["certified"] = time.perf_counter() - mark
        self._stop_active = bool(certified["stop_active"])

        # 3-4. Apply what survives, then one physics step.
        mark = time.perf_counter()
        applied = np.asarray(self.base.command_body_velocity(allowed), dtype=float)
        timings["command"] = time.perf_counter() - mark
        mark = time.perf_counter()
        self.client.step()
        timings["step"] = time.perf_counter() - mark

        # 5. Sense: one scan packet, the pose, the ground-truth contact check.
        mark = time.perf_counter()
        self.packet, scan_fault, rerenders = self._sense()
        timings["scan"] = time.perf_counter() - mark
        mark = time.perf_counter()
        pose = np.asarray(self.base.pose(), dtype=float)
        timings["pose"] = time.perf_counter() - mark
        mark = time.perf_counter()
        reading = self.contact.check()
        timings["contact"] = time.perf_counter() - mark

        # 6. The velocity the NEXT step's band selection will use.
        mark = time.perf_counter()
        velocity = np.asarray(self.velocity.update(tuple(pose)), dtype=float)
        self._velocity = velocity
        timings["velocity"] = time.perf_counter() - mark

        timings["whole"] = time.perf_counter() - whole
        over_budget = timings["whole"] > self.budget_s
        if over_budget and self.over_budget_is_fatal:
            raise RuntimeError(
                f"step {self._step} took {timings['whole'] * 1e3:.1f} ms, over the "
                f"{self.budget_s * 1e3:.0f} ms budget")

        self._step += 1
        state = StepState(
            step=self._step, sim_time=self.client.time(), pose=pose, velocity=velocity,
            requested=requested, applied=applied, scan=self.packet,
            in_contact=bool(reading.in_contact), contact_names=tuple(reading.names),
            certified={**certified, "enabled": self.certified_enabled},
            timings_s=timings, over_budget=over_budget, scan_fault=scan_fault,
            scan_rerenders=rerenders, decided_on_fault=decided_on_fault,
            pfb=self._pfb_report(requested) if self.filter_enabled else None)
        self._count(state)
        return state

    def _sense(self) -> tuple[ScanPacket, bool, int]:
        """This tick's packet through the plausibility guard (ruled 2026-10-23): re-rendered up to
        `rerenders` times while implausible; still implausible, the fail-safe (every bin
        unobserved). Without a corrupted render this is exactly `scanner.read`."""
        now = self.client.time()
        packet = self.scanner.read(sim_time=now)
        rerenders = 0
        plausible = self.guard.plausible(packet)
        while not plausible and rerenders < self.guard.rerenders:
            rerenders += 1
            packet = self.scanner.rescan(sim_time=now)
            plausible = self.guard.plausible(packet)
        if plausible:
            self.guard.accept(packet)
            self._packet_fault = False
            return packet, False, rerenders
        self.guard.reject()
        self._packet_fault = True
        return all_unobserved(packet), True, rerenders

    def _filter(self, requested: np.ndarray) -> np.ndarray:
        """The PFB (WP2) on what the certified layer is about to decide on: the scan in hand,
        the velocity estimate, the command the base is executing, and the latch the layer left
        after the previous step (Q7: under it, only releasable commands pass)."""
        packet = self.packet
        res = self.pfb.filter(requested, self._velocity, self.base.applied_command,
                              packet.filter_ranges, packet.filter_bearings, packet.filter_valid,
                              latched=self._stop_active)
        self._pfb_result = res
        return res.u

    def _pfb_report(self, requested: np.ndarray) -> dict[str, Any] | None:
        res = self._pfb_result
        if res is None:
            return None
        return {"mode": res.mode, "regime": res.regime, "infeasible": res.infeasible,
                "reason": res.reason, "solves": res.solves, "heading": res.heading,
                "yaw_field": res.yaw_field, "u": res.u.tolist(), "nominal": requested.tolist(),
                "unobserved": res.unobserved}


    # ------------------------------------------------------------------
    def _count(self, state: StepState) -> None:
        c = self.counters
        c.steps += 1
        c.protective_stops += int(state.certified["stop_triggered"])
        # A stop decided on a fail-safed packet is a scan-fault stop, not a construction gap
        # (ruled 2026-10-23): it fires with the robot standing, so it is kept out of these.
        genuine = not state.decided_on_fault
        c.at_rest_triggers += int(state.certified["triggered_at_rest"] and genuine)
        c.stationary_triggers += int(state.certified["triggered_stationary"] and genuine)
        c.stopped_steps += int(state.certified["stop_active"])
        c.contact_steps += int(state.in_contact)
        c.over_budget_steps += int(state.over_budget)
        c.warning_steps += int(state.certified["warning"])
        c.warning_entries += int(state.certified["warning_entered"])
        c.invalid_bin_steps += int(not bool(np.all(state.scan.filter_valid)))
        c.scan_faults += int(state.scan_fault)
        c.scan_rerenders += int(state.scan_rerenders)
        c.scan_fault_stops += int(bool(state.certified["stop_triggered"]) and state.decided_on_fault)
        if state.pfb is not None:
            c.pfb_infeasible += int(state.pfb["infeasible"])
            c.pfb_modified += int(state.pfb["u"] != state.pfb["nominal"])
        if self._last_xy is not None:
            c.distance_m += float(np.linalg.norm(state.pose[:2] - self._last_xy))
        self._last_xy = state.pose[:2].copy()
        for name, seconds in state.timings_s.items():
            c.timings_s.setdefault(name, []).append(seconds)

    def report(self) -> dict[str, Any]:
        """The loop's own account of an episode: counts, distance, timings, budget."""
        c = self.counters
        timings = {name: {"mean_ms": float(np.mean(v) * 1e3),
                          "p95_ms": float(np.percentile(v, 95) * 1e3),
                          "max_ms": float(np.max(v) * 1e3)}
                   for name, v in c.timings_s.items() if v}
        return {"steps": c.steps, "distance_m": c.distance_m,
                "protective_stops": c.protective_stops,
                "at_rest_triggers": c.at_rest_triggers,
                "stationary_triggers": c.stationary_triggers,
                "stops_per_100m": c.stops_per_100m(),
                "stopped_steps": c.stopped_steps, "contact_steps": c.contact_steps,
                "warning_steps": c.warning_steps,
                "warning_entries": c.warning_entries,
                "warnings_per_100m": c.warnings_per_100m(),
                "invalid_bin_steps": c.invalid_bin_steps,
                "scan_faults": c.scan_faults, "scan_rerenders": c.scan_rerenders,
                "scan_fault_stops": c.scan_fault_stops,
                "pfb_infeasible": c.pfb_infeasible, "pfb_modified": c.pfb_modified,
                "over_budget_steps": c.over_budget_steps,
                "budget_ms": self.budget_s * 1e3, "timings": timings}
