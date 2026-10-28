"""The declared classical recovery behaviour (ruled 2026-10-13 (3b) and 2026-10-16).

One wrapper, applied identically to every classical baseline -- B1, B2, B3, B6 and B7 -- and to the
zero-action control. Learned policies get no scripted recovery; the methods state the asymmetry.
Specification: `docs/pending/recovery_spec.md`; numbers in `configs/base.yaml`,
`baselines.recovery`.

**Where it sits.** Between the base controller and the environment: controller -> recovery ->
(B7 only: the PFB) -> certified layer. Its commands pass through B7's filter and the certified
layer like any other; the emulator knows nothing about recovery. It reads only the
`ClassicalInput` (stop flag, velocity estimate, 120-bin stream, route); it counts steps itself.

**Trigger.** All of:
  * the stop flag is set, and the robot's own field has been clear (conservatively: the at-rest
    field AND the field of the band its estimated speed selects) for `restart_delay + wait`. The
    emulator times its restart delay from the moment the field comes clear, so counting from the
    same moment, and never earlier, means a permitted command is released at once;
  * the base controller's own requested direction is NOT restart-permitted: recovery never
    overrides a command the emulator would let through, and a controller that requests nothing
    is never blocked;
  * fewer than `max_attempts` attempts this episode.
At rest with the at-rest field breached, no direction can release the latch: a deadlock
(`inside_min_margin`). Recovery records it and stops intervening.

**Manoeuvre.** Of `directions` evenly spaced body-frame directions, those the restart condition
permits -- the emulator's own test, `CertifiedLayer._restart_allowed`, called on a layer built
exactly as the step loop builds it (the code is shared, not copied); of those, the one with the
largest clearance -- the minimum distance from the footprint, translated the full `max_distance`
along that direction, to every scan point (implemented continuously, ruled 2026-10-16; it was
first implemented as a field margin, which tied across directions because the nearest point lay
abeam or behind every candidate). Ties within `tie_tolerance` go to the direction closest to the
route's (the lookahead point's bearing), then to the lower index. It translates at `speed` with
zero yaw rate -- translation only (ruled 2026-10-16) -- in that fixed direction.

**An attempt ends** at the first of: `cleared` (the base controller's direction is permitted
again; control returns to it), `distance` (`max_distance` travelled, integrated from the velocity
estimate), `new_stop` (a protective stop; it counts in H1 like any other), or `not_released` (the
emulator did not release the stop on the first commanded step, which would mean recovery and the
emulator disagree; counted, never silent). No permitted direction is a deadlock
(`no_permitted_direction`), counted as a failed attempt. After a deadlock, or once the cap is
reached and a further trigger arrives, recovery stops intervening for the rest of the episode; the
episode still runs to the step limit (ruled 2026-10-16).
"""
from __future__ import annotations

import math
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.envs.controller_input import ClassicalInput
from pfbnav.safety.certified import CertifiedLayer, band_speed_limit, field_breached
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.safety.unobserved import resolve_unobserved
from pfbnav.world.strata import certified_params_from_config

REPO_ROOT = Path(__file__).resolve().parents[2]
ENDINGS = ("cleared", "distance", "new_stop", "not_released", "no_permitted_direction")


class RecoveryGeometry:
    """The band-0 field geometry and the restart test: the certified layer's own, from a layer
    built as the step loop builds it. Only its geometry and its restart test are used; its state
    (latch, timers) is never read."""

    def __init__(self, layer: CertifiedLayer, standstill: float, footprint_extent,
                 travel: float, boundary_points: int = 720):
        self.layer = layer
        self.fields = layer.fields
        self.band_top_speeds = tuple(float(v) for v in layer.params.band_top_speeds)
        self.band0 = self.band_top_speeds[0]
        self.standstill = float(standstill)
        self.travel = float(travel)              # m, the full manoeuvre (max_distance)
        self._extent = footprint_extent          # also where an unobserved bin's point is put
        psi = np.linspace(-math.pi, math.pi, boundary_points, endpoint=False)
        e = np.asarray(footprint_extent(psi), dtype=float)
        self._fp_a = np.column_stack([e * np.cos(psi), e * np.sin(psi)])
        self._fp_b = np.roll(self._fp_a, -1, axis=0)

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> RecoveryGeometry:
        table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
        layer = CertifiedLayer(certified_params_from_config(cfg),
                               float(cfg["robot"]["half_length"]),
                               float(cfg["robot"]["half_width"]), table.extent)
        return cls(layer, float(cfg["certified"]["standstill_enter"]), table.extent,
                   travel=float(cfg["baselines"]["recovery"]["max_distance"]))

    def permitted(self, scan, direction) -> bool:
        return self.layer._restart_allowed(scan.ranges, scan.bearings, scan.valid, direction)

    def at_rest_breached(self, scan, yaw_rate: float = 0.0) -> bool:
        """The at-rest field, swept by the yaw rate as the emulator sweeps it (that field is
        continuous in the yaw rate, so a noise-level estimate changes nothing measurable)."""
        field = self.fields.select(scan.bearings, 0.0, 0.0, at_rest=True, yaw_rate=yaw_rate)
        return field_breached(scan.ranges, scan.valid, field)

    def own_field_clear(self, scan, velocity) -> bool:
        """Conservatively: the at-rest field AND the field of the band the estimated speed
        selects are both clear, so whichever the emulator is using is clear."""
        speed = float(math.hypot(velocity[0], velocity[1]))
        if self.at_rest_breached(scan, float(velocity[2]) if len(velocity) > 2 else 0.0):
            return False
        if speed <= self.standstill:
            return True
        heading = float(math.atan2(velocity[1], velocity[0]))
        yaw_rate = float(velocity[2]) if len(velocity) > 2 else 0.0
        field = self.fields.select(scan.bearings, speed, heading, at_rest=False,
                                   yaw_rate=yaw_rate)
        return not field_breached(scan.ranges, scan.valid, field)

    def footprint_distance(self, points) -> np.ndarray:
        """Signed distance, m, from each body-frame point to the footprint: positive outside,
        negative inside. The footprint is star-shaped about the origin, so inside is
        |q| < e(bearing of q); the distance is to its boundary polygon."""
        q = np.atleast_2d(np.asarray(points, dtype=float))
        a, b = self._fp_a[None, :, :], self._fp_b[None, :, :]
        ab = b - a
        t = np.clip(np.sum((q[:, None, :] - a) * ab, axis=2) / np.sum(ab * ab, axis=2), 0.0, 1.0)
        nearest = a + t[:, :, None] * ab
        d = np.min(np.linalg.norm(q[:, None, :] - nearest, axis=2), axis=1)
        inside = np.hypot(q[:, 0], q[:, 1]) < np.asarray(
            self._extent(np.arctan2(q[:, 1], q[:, 0])), dtype=float)
        return np.where(inside, -d, d)

    def clearance(self, scan, heading: float) -> float:
        """The ruled clearance of a recovery direction (2026-10-16): the minimum distance, m,
        from the footprint AFTER the full manoeuvre -- translated `travel` along `heading`,
        without rotating -- to every scan point. An unobserved bin is a point at the footprint's
        boundary on its bearing (hard rule 5; the shared convention ruled 2026-10-27,
        `pfbnav/safety/unobserved.py`; it was a point at range_min until then)."""
        r, _ = resolve_unobserved(scan.ranges, scan.bearings, scan.valid, self._extent)
        b = np.asarray(scan.bearings, dtype=float)

        shift = self.travel * np.array([math.cos(heading), math.sin(heading)])
        pts = np.column_stack([r * np.cos(b), r * np.sin(b)]) - shift
        return float(np.min(self.footprint_distance(pts)))


def choose_direction(geometry: RecoveryGeometry, scan, route_bearing: float, directions: int,
                     tie_tolerance: float, speed: float) -> float | None:
    """The recovery heading, rad (body frame), or None if no direction is permitted. Each
    candidate is judged as the twist the recovery will command, `speed` (m/s) along it: since
    2026-10-19 the restart test clears the whole field that twist selects, and at the creep speed
    (at or below the standstill exit) that is the at-rest field."""
    headings = [math.remainder(2.0 * math.pi * k / directions, 2.0 * math.pi)
                for k in range(directions)]
    candidates = [(h, geometry.clearance(scan, h)) for h in headings
                  if geometry.permitted(scan, (speed * math.cos(h), speed * math.sin(h)))]
    if not candidates:
        return None
    best = max(c for _h, c in candidates)
    tied = [h for h, c in candidates if c >= best - tie_tolerance]
    return min(tied, key=lambda h: abs(math.remainder(h - route_bearing, 2.0 * math.pi)))


class Recovery:
    """The recovery wrapper around a classical controller. Same name and ceilings as its base."""

    input_kind = "classical"

    def __init__(self, base, geometry: RecoveryGeometry, *, restart_delay: float, wait: float,
                 speed: float, max_distance: float, max_attempts: int, directions: int,
                 tie_tolerance: float, dt: float):
        if getattr(base, "input_kind", None) != "classical":
            raise TypeError(f"{getattr(base, 'name', base)} is not a classical controller")
        # A20 (2026-10-18): the speed REACHABLE from the recovery's own creep must stay in band
        # 0, so the creep is at most band 0's speed limit (top - a_max T) -- and at most the
        # standstill exit, so the at-rest field contains its reach (both 0.02 m/s as configured).
        params = geometry.layer.params
        if not 0.0 < speed <= min(band_speed_limit(0, params), params.rest_speed) + 1e-12:
            raise ValueError("recovery speed must be positive, its reachable speed within band 0, "
                             "and no faster than the standstill exit")
        if min(wait, max_distance, dt, tie_tolerance) <= 0.0 or max_attempts < 1 \
                or directions < 4:
            raise ValueError("impossible recovery parameters")
        self.base, self.geometry = base, geometry
        self.name = base.name
        self.ceilings = np.asarray(base.ceilings, dtype=float)
        self.trigger_steps = math.ceil((restart_delay + wait) / dt - 1e-9)
        self.speed, self.max_distance = float(speed), float(max_distance)
        self.max_attempts, self.directions = int(max_attempts), int(directions)
        self.tie_tolerance, self.dt = float(tie_tolerance), float(dt)
        self.reset()

    @classmethod
    def from_config(cls, base, cfg: dict[str, Any]) -> Recovery:
        r = cfg["baselines"]["recovery"]
        return cls(base, RecoveryGeometry.from_config(cfg),
                   restart_delay=float(cfg["certified"]["restart_delay"]),
                   wait=float(r["wait"]), speed=float(r["speed"]),
                   max_distance=float(r["max_distance"]), max_attempts=int(r["max_attempts"]),
                   directions=int(r["directions"]), tie_tolerance=float(r["tie_tolerance"]),
                   dt=float(cfg["sim"]["dt"]))

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self.base.reset()
        self.mode = "pass"                 # "pass", "recover" or "off"
        self.stopped_steps = 0             # consecutive steps with the stop flag set
        self.clear_steps = 0               # ... and the robot's own field clear
        self.attempts = 0
        self.endings: Counter = Counter()
        self.gave_up: str | None = None
        self.distance_m = 0.0
        self.steps_recovering = 0
        self.attempt_log: list[dict[str, Any]] = []   # one entry per attempt (ruled 2026-10-16)
        self._step = 0
        self._heading = 0.0
        self._released = False
        self._attempt_steps = 0
        self._attempt_distance = 0.0

    def begin_episode(self, episode: dict[str, Any]) -> None:
        """Passed to the base controller, which may need the episode's route profile (the
        declared route cap, ruled 2026-10-20)."""
        hook = getattr(self.base, "begin_episode", None)
        if hook is not None:
            hook(episode)

    def episode_report(self) -> dict[str, Any]:
        """What the episode record carries (per stratum in every report), with the base
        controller's own report (the route cap's) when it has one."""
        deadlock = self.gave_up if self.gave_up in ("inside_min_margin",
                                                     "no_permitted_direction") else None
        base = getattr(self.base, "episode_report", None)
        return {**(base() if base is not None else {}),
                "recovery_attempts": self.attempts,
                "recovery_endings": {e: int(self.endings.get(e, 0)) for e in ENDINGS},
                "recovery_attempt_log": [dict(a) for a in self.attempt_log],
                "recovery_deadlock": deadlock,
                "recovery_gave_up": self.gave_up,
                "recovery_distance_m": self.distance_m,
                "recovery_steps": self.steps_recovering,
                "recovery_time_s": self.steps_recovering * self.dt}

    def command(self, inp: ClassicalInput) -> np.ndarray:
        if not isinstance(inp, ClassicalInput):
            raise TypeError(f"recovery takes a ClassicalInput, got {type(inp).__name__}")
        base = np.asarray(self.base.command(inp), dtype=float)
        self._step += 1
        if inp.stop_active:
            self.stopped_steps += 1
            own_clear = self.geometry.own_field_clear(inp.scan, inp.velocity)
            self.clear_steps = self.clear_steps + 1 if own_clear else 0
        else:
            self.stopped_steps = self.clear_steps = 0
        if self.mode == "recover":
            moving = self._continue(inp, base)
            if moving is not None:
                return moving
        if self.mode != "pass" or not inp.stop_active:
            return base
        if self.stopped_steps >= self.trigger_steps and self.clear_steps == 0 \
                and float(np.hypot(*inp.velocity[:2])) <= self.geometry.standstill \
                and self.geometry.at_rest_breached(inp.scan, float(inp.velocity[2])):
            self._give_up("inside_min_margin")
            return base
        if self.clear_steps < self.trigger_steps:
            return base
        if self.geometry.permitted(inp.scan, base):
            return base                    # the base controller can restart on its own
        if self.attempts >= self.max_attempts:
            self._give_up("cap")
            return base
        self.attempts += 1
        lx, ly = (float(v) for v in inp.route.lookahead)
        route_bearing = math.atan2(ly, lx) if math.hypot(lx, ly) > 1e-12 else 0.0
        heading = choose_direction(self.geometry, inp.scan, route_bearing, self.directions,
                                   self.tie_tolerance, self.speed)
        if heading is None:
            self.endings["no_permitted_direction"] += 1
            self.attempt_log.append({"start_step": self._step, "heading_deg": None,
                                     "ending": "no_permitted_direction", "distance_m": 0.0,
                                     "steps": 0, "time_s": 0.0})
            self._give_up("no_permitted_direction")
            return base
        self.mode, self._heading = "recover", heading
        self._released, self._attempt_steps, self._attempt_distance = False, 0, 0.0
        self.attempt_log.append({"start_step": self._step,
                                 "heading_deg": math.degrees(heading), "ending": None,
                                 "distance_m": 0.0, "steps": 0, "time_s": 0.0})
        return self._move()

    def act(self, inp: ClassicalInput) -> np.ndarray:
        return np.clip(self.command(inp) / self.ceilings, -1.0, 1.0)

    # ------------------------------------------------------------------
    def _move(self) -> np.ndarray:
        self._attempt_steps += 1
        self.steps_recovering += 1
        # Kept live, so an attempt the episode's end cuts short still reports its time (ruled
        # 2026-10-19: at 0.018 m/s an attempt to 0.30 m takes ~17 s; G4 reports attempt times).
        self.attempt_log[-1].update(steps=self._attempt_steps,
                                    time_s=self._attempt_steps * self.dt)
        return np.array([self.speed * math.cos(self._heading),
                         self.speed * math.sin(self._heading), 0.0])

    def _end(self, ending: str) -> None:
        self.endings[ending] += 1
        self.attempt_log[-1].update(ending=ending, distance_m=self._attempt_distance,
                                    steps=self._attempt_steps,
                                    time_s=self._attempt_steps * self.dt)
        self.mode = "pass"

    def _give_up(self, reason: str) -> None:
        self.gave_up, self.mode = reason, "off"

    def _continue(self, inp: ClassicalInput, base: np.ndarray) -> np.ndarray | None:
        """One step of an attempt in progress: the recovery command, or None if it ended."""
        if not self._released:
            if inp.stop_active:
                self._end("not_released")
                return None
            self._released = True
        elif inp.stop_active:
            self._end("new_stop")
            return None
        step = float(np.hypot(*inp.velocity[:2])) * self.dt
        self._attempt_distance += step
        self.distance_m += step
        if self.geometry.permitted(inp.scan, base):
            self._end("cleared")
            return None
        if self._attempt_distance >= self.max_distance:
            self._end("distance")
            return None
        return self._move()
