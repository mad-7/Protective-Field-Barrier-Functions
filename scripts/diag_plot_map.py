"""Draw a baked map and a plan over it. Offline; no simulator.

What the global planner believes, made visible: the obstacles as given, the cells the bake
marks occupied, the lethal tier the plan may never enter, the soft tier it pays to cross,
and the path before and after line-of-sight simplification. Unmapped obstacles are drawn as
outlines only, because that is exactly what the planner is not told about.

Until `pfbnav/world/layouts.py` exists (WP1), the layout drawn here is a small aisle
arrangement built in this script, marked as such in the figure's title so no one mistakes
it for a generated one.

Usage:
    python scripts/diag_plot_map.py --config configs/base.yaml --seed 0 \
        --out docs/figures/map_and_plan.png
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from pfbnav.config import load_config
from pfbnav.planning.astar import plan_from_config
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.world.baking import Obstacle, bake, planner_radii
from pfbnav.world.layouts import generate

REPO_ROOT = Path(__file__).resolve().parents[1]


def rectangle(x0: float, y0: float, x1: float, y1: float) -> np.ndarray:
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=float)


def rotated_rectangle(cx: float, cy: float, length: float, width: float,
                      angle_deg: float) -> np.ndarray:
    """A shelf at an angle: the case exact footprints exist for."""
    hl, hw = length / 2, width / 2
    corners = np.array([[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]])
    a = math.radians(angle_deg)
    rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    return corners @ rot.T + np.array([cx, cy])


def demo_layout() -> tuple[list[Obstacle], tuple[float, float, float, float]]:
    """A hand-built aisle arrangement, standing in for the generator (WP1)."""
    bounds = (0.0, 0.0, 12.0, 8.0)
    obstacles = [
        Obstacle(rectangle(0.0, 0.0, 12.0, 0.2), name="wall_south"),
        Obstacle(rectangle(0.0, 7.8, 12.0, 8.0), name="wall_north"),
        Obstacle(rectangle(0.0, 0.0, 0.2, 8.0), name="wall_west"),
        Obstacle(rectangle(11.8, 0.0, 12.0, 8.0), name="wall_east"),
        Obstacle(rectangle(2.0, 1.0, 2.8, 4.2), name="shelf_a"),
        Obstacle(rectangle(4.6, 3.6, 5.4, 7.0), name="shelf_b"),
        Obstacle(rectangle(7.2, 1.0, 8.0, 4.2), name="shelf_c"),
        Obstacle(rotated_rectangle(9.6, 5.6, 2.6, 0.8, 35.0), name="shelf_d_angled"),
        # The planner is not told about this one: the local planner and the safety layers
        # are what stand between the robot and it.
        Obstacle(rectangle(5.9, 0.4, 6.5, 1.0), mapped=False, name="pallet_unmapped"),
    ]
    return obstacles, bounds


def draw(baked, obstacles, result, start, goal, title: str, out: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap

    x0, y0 = baked.origin
    rows, cols = baked.shape
    extent = (x0, x0 + cols * baked.resolution, y0, y0 + rows * baked.resolution)

    fig, ax = plt.subplots(figsize=(12.0, 8.4))
    soft = LinearSegmentedColormap.from_list("soft", ["#FFFFFF00", "#F3C969CC"])
    ax.imshow(np.where(baked.lethal, np.nan, baked.cost), origin="lower", extent=extent,
              cmap=soft, vmin=0.0, vmax=1.0, interpolation="nearest")
    lethal_only = np.where(baked.lethal & ~baked.occupied, 1.0, np.nan)
    ax.imshow(lethal_only, origin="lower", extent=extent, interpolation="nearest",
              cmap=LinearSegmentedColormap.from_list("lethal", ["#E24B4A66", "#E24B4A66"]))
    ax.imshow(np.where(baked.occupied, 1.0, np.nan), origin="lower", extent=extent,
              interpolation="nearest",
              cmap=LinearSegmentedColormap.from_list("occ", ["#2C2C2A", "#2C2C2A"]))

    for obstacle in obstacles:
        poly = np.vstack([obstacle.polygon, obstacle.polygon[:1]])
        if obstacle.mapped:
            ax.plot(poly[:, 0], poly[:, 1], color="#1A1A18", lw=1.2, zorder=4)
        else:
            ax.plot(poly[:, 0], poly[:, 1], color="#8E44AD", lw=2.0, ls="--", zorder=5,
                    label="unmapped obstacle" if obstacle is obstacles[-1] else None)

    if result.ok:
        ax.plot(result.path_world[:, 0], result.path_world[:, 1], color="#1D9E75", lw=2.6,
                marker="o", ms=5, zorder=6,
                label=f"plan, {len(result.path_cells)} waypoints, {result.length_m:.2f} m")
    ax.plot(*start, marker="o", ms=11, color="#2C6FBB", zorder=7, label="start")
    ax.plot(*goal, marker="*", ms=17, color="#D4622A", zorder=7, label="goal")

    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title, fontsize=10)
    ax.legend(loc="upper left", fontsize=8, framealpha=0.95)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=170)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", default="docs/figures/map_and_plan.png")
    parser.add_argument("--layout-seed", type=int,
                        help="generate this layout instead of the hand-built demo")
    parser.add_argument("--stratum", help="aisle stratum for a generated layout")
    parser.add_argument("--unmapped", type=int, default=2,
                        help="unmapped obstacles in a generated layout")
    # Chosen so the plan runs down the aisle the unmapped pallet stands in: the planner
    # routes straight through it, because it was never told. That is the case the local
    # planner and the safety layers exist for, and the figure should show it.
    parser.add_argument("--start", type=float, nargs=2, default=(1.0, 2.0))
    parser.add_argument("--goal", type=float, nargs=2, default=(10.8, 2.0))
    args = parser.parse_args()

    cfg = load_config(args.config)
    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    radii = planner_radii(cfg, circumscribed=table.circumscribed)
    if args.layout_seed is None:
        obstacles, bounds = demo_layout()
        origin = "hand-built layout; the generator is WP1"
    else:
        layout = generate(args.layout_seed, cfg, stratum=args.stratum,
                          unmapped_count=args.unmapped)
        obstacles, bounds = layout.obstacles, layout.bounds
        m = layout.metadata
        origin = (f"generated: seed {m['seed']}, {m['stratum']} aisles, "
                  f"{m['clearance_m']:.3f} m per side / {m['aisle_width_m']:.3f} m wide, "
                  f"{m['rows']} rows, {m['unmapped_count']} unmapped, "
                  f"layout {m['layout_sha256'][:10]}")
        print(f"layout: {origin}")
    baked = bake(obstacles, bounds, float(cfg["planner"]["resolution"]), radii)
    result = plan_from_config(baked, tuple(args.start), tuple(args.goal), cfg)

    print(f"map {baked.shape[0]} x {baked.shape[1]} cells at "
          f"{baked.resolution} m; occupied {int(baked.occupied.sum())}, "
          f"lethal {int(baked.lethal.sum())}, unmapped {baked.unmapped}")
    print(f"radii: lethal {radii['lethal']:.4f} m, soft {radii['soft']:.4f} m")
    print(f"plan: {result.status}, {result.expansions} expansions, "
          f"{result.waypoints_before_simplification} waypoints -> "
          f"{len(result.path_cells)}, {result.length_m:.3f} m, cost {result.cost:.3f}")

    out = REPO_ROOT / args.out
    title = (f"Baked map and plan\n{origin}\n"
             f"{baked.resolution * 100:.0f} cm cells; lethal {radii['lethal']:.3f} m, "
             f"soft to {radii['soft']:.3f} m; plan {result.status}, "
             f"{result.length_m:.2f} m over {len(result.path_cells)} waypoints "
             f"(from {result.waypoints_before_simplification})")
    draw(baked, obstacles, result, tuple(args.start), tuple(args.goal), title, out)
    print(f"-> {out}")
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
