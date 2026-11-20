"""Scenarios for an empty hall: the perimeter walls and no shelves. Pure; no simulator.

`make_scenario` places endpoints inside aisles, so a hall with no shelf rows has no endpoint to give.
This builds the same `Scenario` with the endpoints in the open floor, under the same rules that apply
elsewhere: the route is planned on the mapped map, and it must pass the free-orientation validity check
(`oracle.route_validity`), so an empty-hall scenario is exactly as traversable as any other.

Configuration (`world.free_hall`, `configs/worlds/empty_hall.yaml`) names the margins; the layout must
come from a config with `world.layout.rows: [0, 0]`.

The scenario's `stratum` is "wide": `generate` needs a stratum name to sample a clearance, and `world_of`
passes the recorded name back to it. There are no aisles here, so the label carries no meaning and a
per-stratum summary of an empty-hall set is a single row called "wide".
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from pfbnav.planning.astar import OK, plan_from_config
from pfbnav.world.baking import bake, planner_radii
from pfbnav.world.layouts import Layout, generate
from pfbnav.world.oracle import ROUTE_OK
from pfbnav.world.scenarios import (
    NO_PLAN,
    NO_VALID_ROUTE,
    SCENARIO_OK,
    Scenario,
    ValidityCheck,
    _circumscribed,
    _clearance_along,
    scenario_sha256,
    scenarios_sha256,
)

STRATUM = "wide"                      # a label only; see the module docstring
NOT_EMPTY = "the_layout_has_shelves"


def make_free_scenario(seed: int, cfg: dict[str, Any], *, split: str = "",
                       validity: ValidityCheck | None = None
                       ) -> tuple[str, Scenario | None, Layout | None]:
    """One empty-hall scenario: `(status, scenario, layout)`, as `make_scenario` returns."""
    fh = cfg["world"]["free_hall"]
    layout = generate(seed, cfg, stratum=STRATUM, unmapped_count=0, split=split)
    walls = {"wall_south", "wall_north", "wall_west", "wall_east"}
    if {o.name for o in layout.obstacles} != walls:
        return NOT_EMPTY, None, layout
    rng = np.random.default_rng(seed)
    x0, y0, x1, y1 = layout.bounds
    t = float(cfg["world"]["layout"]["wall_thickness"])
    m = float(fh["wall_margin_m"])
    lo_x, hi_x = x0 + t + m, x1 - t - m
    lo_y, hi_y = y0 + t + m, y1 - t - m
    span = hi_x - lo_x
    end = float(fh["end_fraction"])
    radii = planner_radii(cfg, circumscribed=_circumscribed(cfg))
    baked = bake(layout.obstacles, layout.bounds, float(cfg["planner"]["resolution"]), radii)
    check = (validity if validity is not None else ValidityCheck(cfg)).for_layout(
        [o.polygon for o in layout.mapped])
    invalid = False
    for _ in range(int(fh["attempts"])):
        start = (float(rng.uniform(lo_x, lo_x + end * span)), float(rng.uniform(lo_y, hi_y)))
        goal = (float(rng.uniform(hi_x - end * span, hi_x)), float(rng.uniform(lo_y, hi_y)))
        result = plan_from_config(baked, start, goal, cfg)
        if result.status != OK or result.length_m < float(fh["min_route_m"]):
            continue
        if check(result.path_world, start) != ROUTE_OK:
            invalid = True
            continue
        yaw = math.atan2(result.path_world[1][1] - start[1], result.path_world[1][0] - start[0])
        digest = str(layout.metadata["layout_sha256"])
        scenario = Scenario(
            seed=int(seed), split=split, stratum=STRATUM, start=(start[0], start[1], yaw),
            goal=goal, layout_sha256=digest,
            generator_sha256=str(layout.metadata["generator_sha256"]),
            scenarios_sha256=scenarios_sha256(),
            scenario_sha256=scenario_sha256(digest, (start[0], start[1], yaw), goal),
            unmapped_count=0, clearance_m=float(layout.metadata["clearance_m"]),
            path_length_m=result.length_m,
            path_min_clearance_m=_clearance_along(result.path_world, layout),
            aisle_travel_m=0.0)
        return SCENARIO_OK, scenario, layout
    return (NO_VALID_ROUTE if invalid else NO_PLAN), None, layout


def build_free_set(seeds, cfg: dict[str, Any], split: str) -> dict[str, Any]:
    """A set manifest in the shape `verify()` and `evaluate.py` read."""
    check = ValidityCheck(cfg)
    scenarios, failures = [], []
    for seed in (int(s) for s in seeds):
        status, scenario, _ = make_free_scenario(seed, cfg, split=split, validity=check)
        if scenario is None:
            failures.append({"seed": seed, "status": status})
        else:
            scenarios.append(scenario.as_dict())
    return {
        "split": split, "kind": "empty_hall", "requested": len(list(seeds)),
        "built": len(scenarios), "failures": failures,
        "episode_rate_requested": 0.0, "episode_rate_achieved": 0.0,
        "generator_sha256": scenarios[0]["generator_sha256"] if scenarios else None,
        "scenarios_sha256": scenarios_sha256(), "locked": False, "scenarios": scenarios,
    }
