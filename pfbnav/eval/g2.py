"""Gate G2 -- training-host throughput: the measuring wrapper and the pure verdict. No simulator imports.

`scripts/gate_g2_throughput.py` runs N worker processes, one per simulator instance, for each N in
`G2_compute_budget.parallel_instances`. Each worker steps the training environment in two phases of
`measure_seconds`: random actions, then SAC learning with an update every step (the environment AND
the learner, docs/requirements.md). This module holds what a worker measures (`Meter`) and what the
gate concludes from it (`summarise_n`, `verdict_n`, `select_instances`, `projection`).

Every threshold is read by the caller from `preregistration/gates.yaml` (`G2_compute_budget`), except
the numeric reading of "essentially no over-budget steps", which is G2's own criterion in
`configs/gates/g2.yaml` (hard rule 16: an overlay carries only its own criteria), and the scan-fault
flag, which is the runner's (`runner.SCAN_FAULT_FLAG`, ruled 2026-10-23).

**Per N:**
  * the aggregate rate is the SUM over workers of each worker's steps / its window's wall seconds;
    resets are inside the window, so an episode costs its resets (requirements);
  * throughput is also reported split by rendered and non-rendered episodes, never pooled (ruled
    2026-09-26), per instance: steps / seconds of the episodes that started inside the window;
  * the rendered-episode fraction pools every worker's episodes;
  * the over-budget and scan-fault shares pool every worker's steps in the SAC window.

**Verdict for an N** (SAC phase, the one that matters): the aggregate reaches
`min_steps_per_second`; no more than `max_rendered_episode_fraction` of episodes rendered; the
over-budget share at most the ruled maximum; the scan-fault share at most the flag. **The approved
instance count** is the largest N that passes; G2 passes if any N does. The projection gives the wall
clock of `n_training_runs` x `total_timesteps` at that N's aggregate against `wallclock_budget_weeks`.
"""
from __future__ import annotations

import math
import time
from collections.abc import Callable, Mapping, Sequence
from typing import Any

import gymnasium as gym
import numpy as np

SECONDS_PER_WEEK = 7 * 24 * 3600.0


class Meter(gym.Wrapper):
    """Counts steps, over-budget steps, scan faults and episodes (with their reset time and whether
    they rendered) inside a measurement window. Passive: it changes nothing the environment returns.

    `rendered` is read after each reset, from `rendered()` -- the client's `display_in_force`, set by
    the start of the episode's simulation."""

    def __init__(self, env: gym.Env, rendered: Callable[[], bool | None],
                 clock: Callable[[], float] = time.perf_counter):
        super().__init__(env)
        self._rendered = rendered
        self._clock = clock
        self._open: dict[str, Any] | None = None
        self._t_last = 0.0
        self.window_start: float | None = None
        self.window_seconds = 0.0
        self.steps = self.over_budget = self.scan_faults = 0
        self.episodes: list[dict[str, Any]] = []

    @property
    def measuring(self) -> bool:
        return self.window_start is not None

    def start_window(self) -> None:
        self.window_start = self._clock()
        self.steps = self.over_budget = self.scan_faults = 0
        self.episodes = []
        self._open = None                       # only episodes that START inside the window count

    def elapsed(self) -> float:
        return 0.0 if self.window_start is None else self._clock() - self.window_start

    def stop_window(self) -> dict[str, Any]:
        if self.window_start is None:
            raise RuntimeError("no measurement window is open")
        self.window_seconds = self._clock() - self.window_start
        self.window_start = None
        self._open = None                       # an episode cut by the window's end is not counted
        return self.result()

    def result(self) -> dict[str, Any]:
        return {"seconds": self.window_seconds, "steps": self.steps,
                "over_budget_steps": self.over_budget, "scan_fault_steps": self.scan_faults,
                "episodes": list(self.episodes)}

    def reset(self, **kwargs):
        t0 = self._clock()
        out = self.env.reset(**kwargs)
        reset_s = self._clock() - t0
        if self.measuring:
            self._open = {"reset_s": reset_s, "rendered": bool(self._rendered()), "steps": 0,
                          "seconds": reset_s}
            self.episodes.append(self._open)
            self._t_last = self._clock()
        return out

    def step(self, action):
        out = self.env.step(action)
        if self.measuring:
            info = out[4]
            self.steps += 1
            self.over_budget += int(bool(info.get("over_budget")))
            self.scan_faults += int(bool(info.get("scan_fault")))
            if self._open is not None:
                now = self._clock()
                self._open["steps"] += 1
                self._open["seconds"] += now - self._t_last
                self._t_last = now
        return out


def _rate(steps: float, seconds: float) -> float | None:
    return steps / seconds if seconds > 0 else None


def _split(episodes: Sequence[Mapping[str, Any]], rendered: bool) -> dict[str, Any]:
    eps = [e for e in episodes if bool(e["rendered"]) == rendered]
    steps = sum(int(e["steps"]) for e in eps)
    seconds = sum(float(e["seconds"]) for e in eps)
    return {"episodes": len(eps), "steps": steps, "per_instance_steps_per_s": _rate(steps, seconds)}


def summarise_phase(workers: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """One phase over the N workers' `Meter.result()`s."""
    episodes = [e for w in workers for e in w["episodes"]]
    steps = sum(int(w["steps"]) for w in workers)
    resets = np.array([float(e["reset_s"]) for e in episodes]) if episodes else np.zeros(0)
    rates = [_rate(int(w["steps"]), float(w["seconds"])) for w in workers]
    return {
        "instances": len(workers),
        "aggregate_steps_per_s": (float(sum(r for r in rates if r is not None))
                                  if rates and None not in rates else None),
        "per_instance_steps_per_s": rates,
        "steps": steps,
        "episodes": len(episodes),
        "rendered_episode_fraction": (sum(1 for e in episodes if e["rendered"]) / len(episodes)
                                      if episodes else None),
        "rendered": _split(episodes, True), "not_rendered": _split(episodes, False),
        "over_budget_share": (sum(int(w["over_budget_steps"]) for w in workers) / steps
                              if steps else None),
        "scan_fault_share": (sum(int(w["scan_fault_steps"]) for w in workers) / steps
                             if steps else None),
        "reset_s": ({"median": float(np.median(resets)), "p95": float(np.quantile(resets, 0.95))}
                    if len(resets) else None),
    }


def verdict_n(sac: Mapping[str, Any], *, min_steps_per_second: float,
              max_rendered_episode_fraction: float, max_over_budget_share: float,
              max_scan_fault_share: float) -> dict[str, Any]:
    """PASS for this N, or the reasons it fails. A missing figure fails (it was not measured)."""
    checks = {
        "throughput": (sac["aggregate_steps_per_s"], ">=", min_steps_per_second),
        "rendered": (sac["rendered_episode_fraction"], "<=", max_rendered_episode_fraction),
        "over_budget": (sac["over_budget_share"], "<=", max_over_budget_share),
        "scan_faults": (sac["scan_fault_share"], "<=", max_scan_fault_share),
    }
    failed = []
    for name, (value, op, bound) in checks.items():
        ok = value is not None and (value >= bound if op == ">=" else value <= bound)
        if not ok:
            failed.append(f"{name}: {value} not {op} {bound}")
    return {"pass": not failed, "failed": failed}


def select_instances(verdicts: Mapping[int, Mapping[str, Any]]) -> int | None:
    """The approved instance count: the largest N whose verdict passes, or None."""
    passing = [n for n, v in verdicts.items() if v["pass"]]
    return max(passing) if passing else None


def projection(aggregate_steps_per_s: float, n_training_runs: int, total_timesteps: int,
               wallclock_budget_weeks: float) -> dict[str, Any]:
    """Wall clock of the training matrix at this aggregate, against the window."""
    if not aggregate_steps_per_s > 0:
        return {"seconds": math.inf, "weeks": math.inf, "fits": False}
    seconds = n_training_runs * total_timesteps / aggregate_steps_per_s
    return {"seconds": seconds, "days": seconds / 86400.0, "weeks": seconds / SECONDS_PER_WEEK,
            "fits": seconds <= wallclock_budget_weeks * SECONDS_PER_WEEK}
