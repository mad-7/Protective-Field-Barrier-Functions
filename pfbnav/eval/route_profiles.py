"""Route profiles for a scenario set, computed OFFLINE before its episodes (ruled 2026-10-20).

Two kinds, by scenario (`pfbnav/planning/route_profile.py`):

  * `"cap"` -- the declared route cap every classical baseline carries: the MAPPED world only;
  * `"schedule"` -- the privileged oracle tracker's schedule: every obstacle, mapped and unmapped.

The route is planned exactly as `NavEnv.reset` plans it (the same layout, bake, radii and planner),
so the profile's length must equal the environment's `plan_length_m`; `RouteCapped` and
`OracleTracker` refuse an episode where it does not. Profiles are keyed by `scenario_sha256` and
written into the run directory beside the episodes, so a run records the caps it was held to.
"""
from __future__ import annotations

import json
import math
from collections.abc import Callable, Sequence
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.planning.astar import plan_from_config
from pfbnav.planning.route_profile import ProfileSpec, RouteProfile, oracle_schedule, route_cap
from pfbnav.safety.certified import FieldGeometry
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.world.baking import bake, planner_radii
from pfbnav.world.oracle import Limits
from pfbnav.world.scenarios import Scenario, world_of
from pfbnav.world.strata import certified_params_from_config

KINDS = ("cap", "schedule")
REPO_ROOT = Path(__file__).resolve().parents[2]


def limits_from_config(cfg: dict[str, Any]) -> Limits:
    """The oracle's speed and turning limits, as `scripts/quantify_oracle_free.py` builds them."""
    a = cfg["actuation"]
    b = a["wheel_budget"]
    return Limits(v_max=float(a["v_max"]), vy_max=float(a["vy_max"]), w_max=float(a["w_max"]),
                  alpha_max=float(a["alpha_max"]),
                  budget=float(b["nominal_fraction"]) * float(b["v_rim"]),
                  lever_arm=float(cfg["robot"]["half_wheelbase"])
                  + float(cfg["robot"]["half_track"]),
                  decel=float(a["a_decel_max"]))


def profile_for(record: dict[str, Any], cfg: dict[str, Any], kind: str) -> dict[str, Any]:
    """One scenario's profile of `kind`, with the identity it is checked against."""
    if kind not in KINDS:
        raise ValueError(f"unknown profile kind {kind!r}; one of {KINDS}")
    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    params = certified_params_from_config(cfg)
    fields = FieldGeometry(table.extent, params)
    spec = ProfileSpec.from_config(cfg, params)
    scenario = Scenario(**record)
    layout = world_of(scenario, cfg)
    baked = bake(layout.obstacles, layout.bounds, float(cfg["planner"]["resolution"]),
                 planner_radii(cfg, circumscribed=float(table.circumscribed)))
    plan = plan_from_config(baked, scenario.start[:2], scenario.goal, cfg)
    if not plan.ok:
        raise ValueError(f"scenario {scenario.seed}: the plan is {plan.status}")
    path = np.asarray(plan.path_world)
    if kind == "cap":
        profile = route_cap(path, [o.polygon for o in layout.mapped], fields, params, spec)
    else:
        v = cfg["world"]["layout"]["validity"]
        profile = oracle_schedule(path, [o.polygon for o in layout.obstacles],
                                  float(scenario.start[2]), fields, params,
                                  limits_from_config(cfg), spec,
                                  yaw_step=math.radians(float(v["yaw_step_deg"])),
                                  heading_round=math.radians(float(v["heading_round_deg"])),
                                  yaw_rates=[float(r) for r in
                                             cfg["baselines"]["oracle_tracker"]["yaw_rate_ladder"]])
    return {"scenario_seed": scenario.seed, "scenario_sha256": scenario.scenario_sha256,
            "kind": kind, "plan_length_m": float(plan.length_m), "profile": profile.to_dict()}


def _job(args: tuple[dict[str, Any], dict[str, Any], str]) -> dict[str, Any]:
    return profile_for(*args)


def build_profiles(records: Sequence[dict[str, Any]], cfg: dict[str, Any], kind: str,
                   workers: int = 1,
                   on_done: Callable[[int, int], None] | None = None) -> list[dict[str, Any]]:
    """Every scenario's profile of `kind`, in order, over `workers` processes."""
    jobs = [(dict(r), cfg, kind) for r in records]
    out: list[dict[str, Any]] = []
    if workers <= 1:
        for i, job in enumerate(jobs, start=1):
            out.append(_job(job))
            if on_done is not None:
                on_done(i, len(jobs))
        return out
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for i, result in enumerate(pool.map(_job, jobs, chunksize=1), start=1):
            out.append(result)
            if on_done is not None:
                on_done(i, len(jobs))
    return out


def write_profiles(path: Path, entries: Sequence[dict[str, Any]]) -> None:
    path.write_text(json.dumps(list(entries)), encoding="utf-8")


def lookup(entries: Sequence[dict[str, Any]], kind: str) -> Callable[[dict[str, Any]], RouteProfile]:
    """`begin_episode`'s source: the profile of `kind` for an episode's scenario. A scenario with
    none, or one whose plan length differs, is refused, never run uncapped."""
    by = {e["scenario_sha256"]: e for e in entries if e["kind"] == kind}

    def get(episode: dict[str, Any]) -> RouteProfile:
        entry = by.get(episode["scenario_sha256"])
        if entry is None:
            raise KeyError(f"scenario {episode.get('scenario_seed')}: no {kind} profile")
        return RouteProfile.from_dict(entry["profile"])
    return get
