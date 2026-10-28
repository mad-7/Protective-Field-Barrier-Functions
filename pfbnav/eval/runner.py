"""Run a controller over a scenario set and summarise what happened. (WP3-WP5; first use: G4)

The loop is environment- and controller-agnostic: any Gymnasium environment that ends an episode
with `info["episode"]` (as `NavEnv` does) and any `Controller` (`pfbnav/control/base.py`). Each
scenario is run once, by index, in the order given, on ONE environment and so within one scene
load -- determinism is claimed within one scene load only (ruled 2026-10-14: a reload of the same
scene in one process moved trajectories by up to 1.8 mm), and paired comparisons run their members
together (`docs/reproducibility.md`). Nothing here loads a scene; `run_controllers` runs every
controller of a scenario's set back to back on the same environment.

The primary metric is pooled, never averaged per episode: protective stops per 100 m over the set
is total stops / total distance x 100, which is the rate-ratio estimand's numerator and
denominator (ruled 2026-10-02). An average of per-episode rates would weight a 2 m episode like a
20 m one.
"""
from __future__ import annotations

import time
from collections.abc import Callable, Iterable, Sequence
from typing import Any

import numpy as np

from pfbnav.control.base import input_kind
from pfbnav.envs.controller_input import ClassicalInput
from pfbnav.eval.compute import ComputeHistogram, summarise
from pfbnav.eval.ring import StepRing

OUTCOMES = ("success", "collision", "timeout", "unrecoverable_stop")
SCAN_FAULT_FLAG = 0.001        # a run above 0.1 % of steps scan-faulted is flagged (2026-10-23)
# What `begin_episode` is handed: which scenario, and the plan's length to check a profile against.
EPISODE_IDENTITY = ("scenario_seed", "scenario_sha256", "split", "stratum", "plan_length_m")
# The controllers' own counted statuses, reported per method and stratum beside the PFB's infeasible
# steps (ruled 2026-10-27); 0 for a method without the status. invalid_bin_steps is the environment's
# count of steps with any unobserved bin, the same for every method (the shared convention).
STATUS_COUNTS = ("pfb_infeasible", "dwa_blocked_steps", "mppi_all_rejected", "invalid_bin_steps")


def _given(kind: str, obs, info: dict[str, Any]):
    """What the controller is handed: the observation, or ONLY the typed classical input --
    never the `info` dict, which carries the pose, the emulator's timers and other privileged
    keys (ruled 2026-10-16)."""
    if kind == "observation":
        return obs
    inp = info.get("controller_input")
    if not isinstance(inp, ClassicalInput):
        raise TypeError("a classical controller needs info['controller_input'], a "
                           f"ClassicalInput; the environment gave {type(inp).__name__}")
    return inp


def run_episodes(env, controller, indices: Iterable[int],
                 on_episode: Callable[[dict[str, Any]], None] | None = None,
                 *, ring) -> list[dict[str, Any]]:
    """Run `controller` once on each scenario index; return one record per episode. A controller
    with an `episode_report` (the classical recovery) adds its counts to the record. A controller
    with a `begin_episode` (one under the declared route cap, or the privileged oracle tracker,
    ruled 2026-10-20) is handed the episode's IDENTITY after each reset -- the scenario and the
    plan's length, from which it finds its offline route profile -- never the pose or any other
    step-level key.

    `ring` (a `pfbnav.eval.ring.StepRing`, ruled 2026-10-23) records every step passively and dumps
    the recent steps on stationary triggers, contacts and unrecoverable stops; the episode record
    lists the dumps. It is REQUIRED (ruled 2026-10-26): a run without it is refused before the first
    reset."""
    if not isinstance(ring, StepRing):
        raise TypeError("run_episodes needs a StepRing (ruled 2026-10-26: the ring buffer is on in "
                         "every evaluation)")
    kind = input_kind(controller)
    records = []
    for index in indices:
        controller.reset()
        obs, info = env.reset(options={"index": int(index)})
        begin = getattr(controller, "begin_episode", None)
        if begin is not None:
            begin({k: info[k] for k in EPISODE_IDENTITY})
        ring.begin(int(index))
        total, steps, done = 0.0, 0, False
        modes: dict[str, int] = {}
        # Per-step compute (ruled 2026-10-27): the controller's act, the PFB's slot, their sum.
        hist = {"controller": ComputeHistogram(), "pfb": ComputeHistogram(),
                "decision": ComputeHistogram()}
        while not done:
            given = _given(kind, obs, info)
            mark = time.perf_counter()
            action = controller.act(given)
            spent = time.perf_counter() - mark
            obs, reward, terminated, truncated, info = env.step(action)
            hist["controller"].add(spent)
            pfb_s = info.get("pfb_seconds")
            if pfb_s is not None:
                hist["pfb"].add(float(pfb_s))
                spent += float(pfb_s)
            hist["decision"].add(spent)
            total += float(reward)
            steps += 1
            done = bool(terminated or truncated)
            ring.record(steps, info)
            pfb = info.get("pfb")
            if pfb is not None:                 # the PFB's decision mode, counted per episode
                modes[pfb["mode"]] = modes.get(pfb["mode"], 0) + 1
        episode = dict(info.get("episode") or {})
        if "outcome" not in episode:
            raise RuntimeError(f"scenario index {index} ended without an outcome in info['episode']")
        record = {**episode, "index": int(index), "controller": controller.name,
                  "episode_return": total, "episode_steps": steps}
        if modes:
            record["pfb_modes"] = modes
        record["compute_hist"] = {k: h.to_record() for k, h in hist.items() if h.steps}
        report = getattr(controller, "episode_report", None)
        if report is not None:
            record.update(report())
        record["ring_dumps"] = ring.end(record["outcome"])
        records.append(record)
        if on_episode is not None:
            on_episode(record)
    return records



def run_controllers(env, controllers: Sequence[Any], indices: Iterable[int],
                    on_episode: Callable[[dict[str, Any]], None] | None = None,
                    *, rings: dict[str, StepRing]) -> list[dict[str, Any]]:
    """Every controller on each scenario, the scenario's set back to back, on ONE environment.

    The WP5 pairing requirement (ruled 2026-10-14): all controllers for a scenario run within one
    scene load. Scenario-major order -- scenario i under every controller, then scenario i+1 -- so
    a pair's members are never separated by anything but resets of the same loaded scene. No
    scene is loaded here, and the environment never loads one (`tests/test_runner.py`). `rings`
    holds one ring per controller name, so two controllers' dumps never share a name.
    """
    records = []
    for index in indices:
        for controller in controllers:
            records += run_episodes(env, controller, [index], on_episode=on_episode,
                                    ring=rings[controller.name])
    return records

def outcome_summary(records: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Counts and shares of each outcome, the contact share, and the pooled stop rate."""
    n = len(records)
    counts = {o: sum(1 for r in records if r["outcome"] == o) for o in OUTCOMES}
    unknown = sorted({r["outcome"] for r in records} - set(OUTCOMES))
    if unknown:
        raise ValueError(f"outcomes outside the environment's vocabulary: {unknown}")
    # Contact is any ground-truth contact in the episode (hard rule 4), not only a "collision"
    # ending: the outcome records the first ending, the counter every contacted step.
    contact = sum(1 for r in records
                  if r["outcome"] == "collision" or int(r.get("contact_steps", 0)) > 0)
    distance = float(sum(float(r.get("distance_m", 0.0)) for r in records))
    # Ruled 2026-10-23: stops decided on a scan-faulted packet are EXCLUDED from the primary count
    # and reported alongside it; a run with more than 0.1 % of its steps scan-faulted is flagged.
    fault_stops = int(sum(int(r.get("scan_fault_stops", 0)) for r in records))
    stops = int(sum(int(r.get("protective_stops", 0)) for r in records)) - fault_stops
    steps_total = int(sum(int(r.get("steps", r.get("episode_steps", 0))) for r in records))
    scan_faults = int(sum(int(r.get("scan_faults", 0)) for r in records))
    # The two secondary versions A14 declares (ruled 2026-10-16). The primary divides by the
    # distance actually driven, recovery motion included. (1) Excluding recovery distance (zero
    # for methods without recovery). (2) Per mission: stops per 100 m of PLANNED route, which no
    # controller's motion can change. A disagreement in direction between the primary and (2) on
    # any H1 comparison is reported prominently.
    recovery = float(sum(float(r.get("recovery_distance_m", 0.0)) for r in records))
    planned = float(sum(float(r.get("plan_length_m", 0.0)) for r in records))
    if recovery > distance + 1e-9:
        raise ValueError(f"recovery distance {recovery} exceeds the distance driven {distance}")
    return {
        "episodes": n,
        "counts": counts,
        "shares": {o: (c / n if n else None) for o, c in counts.items()},
        "success_rate": counts["success"] / n if n else None,
        "contact_rate": contact / n if n else None,
        "largest_outcome_share": max(counts.values()) / n if n else None,
        "protective_stops": stops,
        "scan_faults": scan_faults,
        "scan_fault_stops": fault_stops,
        "scan_rerenders": int(sum(int(r.get("scan_rerenders", 0)) for r in records)),
        "scan_fault_step_share": scan_faults / steps_total if steps_total else None,
        "scan_fault_flag": bool(steps_total and scan_faults / steps_total > SCAN_FAULT_FLAG),
        # G2's criterion (ruled 2026-10-23): over-budget steps recorded per run.
        "over_budget_steps": int(sum(int(r.get("over_budget_steps", 0)) for r in records)),
        # Ruled 2026-10-19: expected zero in a static world; any is a construction gap.
        "at_rest_triggers": int(sum(int(r.get("at_rest_triggers", 0)) for r in records)),
        # Of which stationary -- the construction gap; the rest are creep stops (at-rest stops by
        # ruling), whose episodes ending unrecoverable_stop are counted below and must be reported.
        "stationary_triggers": int(sum(int(r.get("stationary_triggers", 0)) for r in records)),
        "creep_stops_ending_unrecoverable": int(sum(
            1 for r in records if int(r.get("at_rest_triggers", 0)) > int(r.get(
                "stationary_triggers", 0)) and r["outcome"] == "unrecoverable_stop")),
        "distance_m": distance,
        "stops_per_100m": 100.0 * stops / distance if distance > 0 else None,
        "recovery_distance_m": recovery,
        "stops_per_100m_excluding_recovery": (100.0 * stops / (distance - recovery)
                                              if distance - recovery > 0 else None),
        "planned_length_m": planned,
        "stops_per_100m_planned": 100.0 * stops / planned if planned > 0 else None,
        "mean_return": float(np.mean([r["episode_return"] for r in records])) if n else None,
        # Ruled 2026-10-27: the controllers' counted statuses, per method and stratum.
        **{k: int(sum(int(r.get(k, 0)) for r in records)) for k in STATUS_COUNTS},
        # Ruled 2026-10-27: per-step compute for every controller (pfbnav/eval/compute.py).
        "compute": compute_summary(records),
    }


def compute_summary(records: Sequence[dict[str, Any]], limit_s: float | None = None
                    ) -> dict[str, Any]:
    """Pooled per-step compute of the controller, the PFB and their sum (ms; p50, p95, max), from the
    episodes' histograms; a part no episode recorded is absent."""
    out = {}
    for part in ("controller", "pfb", "decision"):
        hists = [ComputeHistogram.from_record(r["compute_hist"][part]) for r in records
                 if part in r.get("compute_hist", {})]
        if hists:
            out[part] = summarise(hists, limit_s)
    return out

