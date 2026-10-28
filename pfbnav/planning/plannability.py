"""Sub-cell plannability of the narrowest aisles (supervisor, 2026-10-16: a permanent regression).

At lethal margin 0.10 the narrow floor leaves a centreline band of about 0.06 m between the lethal
inflations, about one 0.05 m grid cell: 2 x (half_width + 0.13 - (half_width + 0.10)). Whether an
aisle that narrow plans then depends on where its walls fall relative to the grid. So the check
places a straight aisle at each stratum's minimum clearance at `offsets` sub-cell positions, along
both grid axes, and plans along it. Every position must plan.

Moved from `scripts/quantify_margins.py` unchanged in what it computes, so the margin
quantifications and the regression test run the same check.
"""
from __future__ import annotations

from typing import Any

import numpy as np

from pfbnav.planning.astar import plan_from_config
from pfbnav.world.baking import Obstacle, bake, planner_radii
from pfbnav.world.strata import strata_from_config


def aisle_plannability(cfg: dict[str, Any], circumscribed: float, *, offsets: int,
                       aisle_length: float) -> list[dict[str, Any]]:
    """For each stratum: does a straight aisle at its minimum clearance per side plan at every one
    of `offsets` sub-cell positions along x and along y, under `cfg`'s planner margins?

    `circumscribed` is the footprint's circumscribed radius, m; `aisle_length` is in m. Returns one
    entry per stratum: {"stratum", "min_clearance_m", "positions", "failures"}. Each failure is
    {"axis", "offset_cells", "status"}.
    """
    res = float(cfg["planner"]["resolution"])
    hw = float(cfg["robot"]["half_width"])
    radii = planner_radii(cfg, circumscribed=circumscribed)
    out = []
    for s in strata_from_config(cfg):
        width = 2.0 * (hw + s.min_clearance)
        failures = []
        for axis in ("x", "y"):
            for k in range(offsets):
                y0 = 1.0 + k * res / offsets
                top = y0 + width
                height = top + 1.0
                walls = [np.array([[-1, -1], [aisle_length + 1, -1], [aisle_length + 1, y0],
                                   [-1, y0]]),
                         np.array([[-1, top], [aisle_length + 1, top],
                                   [aisle_length + 1, height + 1], [-1, height + 1]])]
                start, goal = (0.5, y0 + width / 2), (aisle_length - 0.5, y0 + width / 2)
                bounds = (0.0, 0.0, aisle_length, height)
                if axis == "y":       # the same aisle, transposed
                    walls = [w[:, ::-1] for w in walls]
                    start, goal = start[::-1], goal[::-1]
                    bounds = (0.0, 0.0, height, aisle_length)
                obstacles = [Obstacle(np.asarray(w, dtype=float), mapped=True, name=f"wall_{i}")
                             for i, w in enumerate(walls)]
                baked = bake(obstacles, bounds, res, radii)
                plan = plan_from_config(baked, start, goal, cfg)
                if not plan.ok:
                    failures.append({"axis": axis, "offset_cells": k / offsets,
                                     "status": plan.status})
        out.append({"stratum": s.name, "min_clearance_m": s.min_clearance,
                    "positions": 2 * offsets, "failures": failures})
    return out
