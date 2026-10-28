"""Figure 4: example generated layouts, one per clearance stratum, from the TUNING set (hard rule 2:
test layouts are never plotted during development). Each is rebuilt from its recorded scenario through
the project's own generator (`world_of`), baking and planner, exactly as the environment builds it:
shelves and walls (mapped), unmapped obstacles (sensed, never in the planner's map), the planned
route, the start pose and the goal.
"""
from __future__ import annotations

import math

import matplotlib.pyplot as plt
import numpy as np
import style
from matplotlib.patches import Polygon as Patch

from pfbnav.config import load_config
from pfbnav.planning.astar import plan_from_config
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.world.baking import bake, planner_radii
from pfbnav.world.scenarios import Scenario, load_set, world_of

# (set, index): the first scenario of each stratum's tuning set with two unmapped obstacles, or one
# where none has two.
CHOICE = (("scenarios/tune/tune_narrow.json", "narrow"), ("scenarios/tune/tune_medium.json", "medium"),
          ("scenarios/tune/tune_wide.json", "wide"))

style.apply()
cfg = load_config(style.CONFIG)
table = FootprintTable.from_csv(style.REPO / cfg["robot"]["footprint_table"])
radii = planner_radii(cfg, circumscribed=float(table.circumscribed))
strata = {s["name"]: s for s in cfg["world"]["aisle_strata"]}


def pick(records):
    two = [r for r in records if r["unmapped_count"] == 2]
    one = [r for r in records if r["unmapped_count"] == 1]
    return (two or one)[0]


fig, axes = plt.subplots(3, 1, figsize=(style.TEXT_WIDTH_IN, 8.4))
for (path, name), ax in zip(CHOICE, axes, strict=True):
    record = pick(load_set(style.REPO / path)["scenarios"])
    scenario = Scenario(**record)
    layout = world_of(scenario, cfg)
    baked = bake(layout.obstacles, layout.bounds, float(cfg["planner"]["resolution"]), radii)
    plan = plan_from_config(baked, scenario.start[:2], scenario.goal, cfg)
    for o in layout.obstacles:
        poly = np.asarray(o.polygon, dtype=float)[:, :2]
        ax.add_patch(Patch(poly, closed=True, fc=style.GREY if o.mapped else style.VERMILLION,
                           ec="none", alpha=1.0 if o.mapped else 0.9))
    route = np.asarray(plan.path_world, dtype=float)
    ax.plot(route[:, 0], route[:, 1], color=style.BLUE, lw=1.2, label="planned route")
    x0, y0, yaw0 = scenario.start
    ax.annotate("", xy=(x0 + 0.6 * math.cos(yaw0), y0 + 0.6 * math.sin(yaw0)), xytext=(x0, y0),
                arrowprops={"arrowstyle": "->", "lw": 1.0, "color": style.GREEN})
    ax.plot(x0, y0, marker="o", ms=4, color=style.GREEN, label="start")
    ax.plot(*scenario.goal, marker="*", ms=8, color=style.BLACK, label="goal")
    ax.plot([], [], marker="s", ls="none", color=style.GREY, label="mapped shelves and walls")
    ax.plot([], [], marker="s", ls="none", color=style.VERMILLION, label="unmapped obstacles")
    s = strata[name]
    ax.set_title(f"{name} stratum [{s['min_clearance']:.4g}, {s['max_clearance']:.4g}] m per side; "
                 f"this layout {record['clearance_m']:.3f} m; {record['unmapped_count']} unmapped; "
                 f"tuning seed {record['seed']}", fontsize=7)
    x_min, y_min, x_max, y_max = layout.bounds
    ax.set_xlim(x_min - 0.3, x_max + 0.3)
    ax.set_ylim(y_min - 0.3, y_max + 0.3)
    ax.set_aspect("equal")
    ax.set_ylabel("$y$ (m)")
axes[-1].set_xlabel("$x$ (m)")
axes[0].legend(loc="upper center", bbox_to_anchor=(0.5, 1.32), ncol=3)
fig.subplots_adjust(hspace=0.3)
style.save(fig, "fig4_layout.pdf")
