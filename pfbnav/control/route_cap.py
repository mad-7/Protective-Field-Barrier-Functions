"""The declared route cap every classical baseline carries (ruled 2026-10-20), and the privileged
oracle tracker, G4's positive control (ruled 2026-10-20, pre-declared 2026-10-17).

**`RouteCapped`** wraps a classical controller (B1, B2, B3, B6, B7). Each step it reads where the
robot is along its route from the classical input alone -- arc length = the route's length minus
`route.remaining_m` -- and caps the base controller's command at the profile's speed and yaw rate
there (`pfbnav/planning/route_profile.py`, computed offline from the MAPPED world before the
episode). Translation is scaled, keeping its direction; the yaw rate is clipped. A controller may go
slower, never faster. Learned arms do not get it; the asymmetry is stated in the methods. The
recovery (`pfbnav/control/recovery.py`) sits OVER it, so the direction the recovery judges is the
capped one, and the recovery's own creep is below every cap.

**`OracleTracker`** is privileged: its schedule is computed over every obstacle, mapped and
unmapped, and fixes a body yaw and a speed at each point of the route. It tracks the route with
feedback (cross-track toward the route, yaw toward the schedule's), never faster than the
schedule except at the creep speed (below the standstill exit, so in the at-rest regime) where the
schedule's speed tends to zero, rotating in place where the schedule does. It is a positive control -- it shows the
pipeline can deliver successes under the corrected fields -- and never a comparator.

Both take the profile per episode through `begin_episode`, which the evaluation runner calls after
each reset with the episode's identity; the profile's route length must equal the environment's
plan length, or the profile was computed for a different route and the episode is refused.
"""
from __future__ import annotations

import math
from collections.abc import Callable
from typing import Any

import numpy as np

from pfbnav.envs.controller_input import ClassicalInput
from pfbnav.planning.route_profile import RouteProfile
from pfbnav.world.strata import certified_params_from_config

# A route profile's length is the planned route's arc length; the environment's plan_length_m is the
# same polyline measured by the planner. They agree to rounding.
LENGTH_TOLERANCE_M = 1e-6


class _Profiled:
    """The per-episode profile, looked up by the episode's scenario."""

    def __init__(self, profiles: Callable[[dict[str, Any]], RouteProfile]):
        self._profiles = profiles
        self.profile: RouteProfile | None = None

    def begin_episode(self, episode: dict[str, Any]) -> None:
        profile = self._profiles(episode)
        plan = float(episode["plan_length_m"])
        if abs(profile.length - plan) > LENGTH_TOLERANCE_M:
            raise ValueError(f"scenario {episode.get('scenario_seed')}: the route profile is "
                             f"{profile.length:.6f} m long, the environment's plan "
                             f"{plan:.6f} m: computed for a different route")
        self.profile = profile

    def _need(self) -> RouteProfile:
        if self.profile is None:
            raise RuntimeError("no route profile: begin_episode was not called")
        return self.profile

    def _s(self, inp: ClassicalInput) -> float:
        return self._need().length - float(inp.route.remaining_m)


class RouteCapped(_Profiled):
    """A classical controller under the declared route cap. Same name and ceilings as its base."""

    input_kind = "classical"

    def __init__(self, base, profiles: Callable[[dict[str, Any]], RouteProfile]):
        if getattr(base, "input_kind", None) != "classical":
            raise TypeError(f"{getattr(base, 'name', base)} is not a classical controller")
        super().__init__(profiles)
        self.base = base
        self.name = base.name
        self.ceilings = np.asarray(base.ceilings, dtype=float)
        self.capped_steps = 0
        self.steps = 0

    def reset(self) -> None:
        self.base.reset()
        self.profile = None
        self.capped_steps = self.steps = 0

    def begin_episode(self, episode: dict[str, Any]) -> None:
        """Look up the profile, then pass the episode's identity on to the base controller when it
        takes one (B6 seeds its sampler from the scenario, 2026-10-27)."""
        super().begin_episode(episode)
        hook = getattr(self.base, "begin_episode", None)
        if hook is not None:
            hook(episode)

    def command(self, inp: ClassicalInput) -> np.ndarray:
        cmd = np.asarray(self.base.command(inp), dtype=float)
        v_cap, w_cap = self._need().caps(self._s(inp))
        out = cap_twist(cmd, v_cap, w_cap)
        self.steps += 1
        self.capped_steps += int(not np.array_equal(out, cmd))
        return out

    def act(self, inp: ClassicalInput) -> np.ndarray:
        return np.clip(self.command(inp) / self.ceilings, -1.0, 1.0)

    def episode_report(self) -> dict[str, Any]:
        """The cap's counts, with the base controller's own report when it has one (B3's blocked
        steps)."""
        base = getattr(self.base, "episode_report", None)
        return {**(base() if base is not None else {}),
                "route_cap_steps": self.steps, "route_cap_capped_steps": self.capped_steps}


def cap_twist(cmd: np.ndarray, v_cap: float, w_cap: float) -> np.ndarray:
    """The command with its translation scaled down to at most `v_cap` (m/s, direction kept) and
    its yaw rate clipped to +-`w_cap` (rad/s). Never faster: a command within both is unchanged."""
    out = np.asarray(cmd, dtype=float).copy()
    v = float(math.hypot(out[0], out[1]))
    if v > v_cap:
        out[:2] *= (v_cap / v) if v > 0.0 else 0.0
    out[2] = float(np.clip(out[2], -w_cap, w_cap))
    return out


class OracleTracker(_Profiled):
    """The privileged oracle tracker (G4's positive control). Gains and tolerances from
    `baselines.oracle_tracker` in `configs/base.yaml`."""

    name = "oracle_tracker_privileged"
    input_kind = "classical"

    def __init__(self, profiles: Callable[[dict[str, Any]], RouteProfile], *, ceilings,
                 k_cross: float, k_yaw: float, yaw_tolerance: float, min_speed: float,
                 creep_yaw_rate: float):
        super().__init__(profiles)
        self.ceilings = np.asarray(ceilings, dtype=float)
        self.k_cross, self.k_yaw = float(k_cross), float(k_yaw)
        self.yaw_tolerance, self.min_speed = float(yaw_tolerance), float(min_speed)
        self.creep_yaw_rate = float(creep_yaw_rate)
        self.rotating_steps = self.stopped_steps = 0

    @classmethod
    def from_config(cls, cfg: dict[str, Any], profiles) -> OracleTracker:
        o = cfg["baselines"]["oracle_tracker"]
        a = cfg["actuation"]
        params = certified_params_from_config(cfg)
        return cls(profiles, ceilings=[a["v_max"], a["vy_max"], a["w_max"]],
                   k_cross=float(o["k_cross"]), k_yaw=float(o["k_yaw"]),
                   yaw_tolerance=float(o["yaw_tolerance_rad"]),
                   min_speed=float(o["min_speed"]),
                   creep_yaw_rate=float(cfg["baselines"]["route_cap"]["creep_fraction"])
                   * params.standstill_yaw_exit)

    def reset(self) -> None:
        self.profile = None
        self.rotating_steps = self.stopped_steps = 0

    def command(self, inp: ClassicalInput) -> np.ndarray:
        s = self._s(inp)
        p = self._need()
        v_cap, w_cap = p.caps(s)
        pts = np.asarray(inp.route.points, dtype=float)
        foot = pts[0]                                    # the robot's projection on the route
        ahead = pts[1:][np.linalg.norm(pts[1:] - foot, axis=1) > 1e-9]
        if ahead.size == 0:
            return np.zeros(3)
        t = (ahead[0] - foot) / float(np.linalg.norm(ahead[0] - foot))   # tangent, body frame
        yaw_rel = -math.atan2(t[1], t[0])                # body yaw minus the tangent's
        target = float(p.yaw_offset[p.index(s)]) if p.yaw_offset is not None else 0.0
        err = math.atan2(math.sin(target - yaw_rel), math.cos(target - yaw_rel))
        w = float(np.clip(self.k_yaw * err, -w_cap, w_cap))
        if inp.stop_active:
            # Stopped (found 2026-10-20: a stop mid-rotation, then a 0.2 rad/s spin whose own field
            # held the intrusion, blocked forever): ask for the CREEP regime -- translation at the
            # creep speed, yaw within the creep rate -- which keeps the at-rest field, so the restart
            # is judged on the field nesting keeps clear (A22).
            self.stopped_steps += 1
            w = float(np.clip(w, -self.creep_yaw_rate, self.creep_yaw_rate))
            if v_cap < self.min_speed and abs(err) > self.yaw_tolerance:
                return np.array([0.0, 0.0, w])
            v = self.min_speed * t + self.k_cross * foot
            n = float(np.linalg.norm(v))
            if n > self.min_speed:
                v *= self.min_speed / n
            return np.array([float(v[0]), float(v[1]), w])
        if v_cap < self.min_speed and abs(err) > self.yaw_tolerance:
            self.rotating_steps += 1                     # the schedule rotates in place here
            return np.array([0.0, 0.0, w])
        # Below `min_speed` (the creep speed, in the at-rest regime) the schedule's speed tends to
        # zero toward a rotation point or the start; the tracker creeps there instead of stalling.
        speed = max(v_cap, self.min_speed)
        v = speed * t + self.k_cross * foot
        n = float(np.linalg.norm(v))
        if n > speed:
            v *= speed / n
        return np.array([float(v[0]), float(v[1]), w])

    def act(self, inp: ClassicalInput) -> np.ndarray:
        return np.clip(self.command(inp) / self.ceilings, -1.0, 1.0)

    def episode_report(self) -> dict[str, Any]:
        return {"oracle_rotating_steps": self.rotating_steps,
                "oracle_stopped_steps": self.stopped_steps,
                "oracle_schedule_traversable": None if self.profile is None
                else self.profile.traversable}
