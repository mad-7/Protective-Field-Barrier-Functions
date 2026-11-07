"""Journal draft, Figure 1: the system architecture, drawn schematically (vector, no photographs).

Every quantity in a label is the configured or ruled value quoted in the manuscript (configs/base.yaml,
pfbnav/safety/pfb.py, certified.py). Run from the repository root:

    python paper/journal/scripts/fig_architecture_journal.py
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
REPO = HERE.parents[3]
os.environ.setdefault("FIG_OUT", "paper/journal/figures")
sys.path.insert(0, str(REPO / "paper" / "figures" / "scripts"))

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import (Arc, Circle, FancyArrowPatch, FancyBboxPatch, Polygon,  # noqa: E402
                                Rectangle, Wedge)
import style  # noqa: E402

NAVY = "#1F4E79"
BLUE_FILL = "#EEF3FA"
PFB_EDGE = "#2E86DE"
PFB_FILL = "#DCEBFB"
GREEN_EDGE = "#2E7D32"
GREEN_FILL = "#EEF6E8"
PURPLE_EDGE = "#5B3F8C"
PURPLE_FILL = "#EEEAF7"
LABEL_FILL = "#EAF3DF"
LABEL_EDGE = "#8DB36B"
TXT = "#111111"
BODY = 5.7
TITLE = 7.8


def rbox(ax, x, y, w, h, fc, ec, lw=1.2, r=1.2, z=1):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={r}",
                                fc=fc, ec=ec, lw=lw, zorder=z))


def header_box(ax, x, y, w, h, title, fc=BLUE_FILL, ec=NAVY, band=NAVY, band_h=4.6, lw=1.2, tsize=TITLE):
    rbox(ax, x, y, w, h, fc, ec, lw=lw)
    ax.add_patch(FancyBboxPatch((x, y), w, band_h, boxstyle="round,pad=0,rounding_size=1.2",
                                fc=band, ec=band, lw=0, zorder=2))
    ax.add_patch(Rectangle((x, y + band_h - 1.3), w, 1.3, fc=band, ec=band, lw=0, zorder=2))
    ax.text(x + w / 2, y + band_h / 2, title, ha="center", va="center", color="white",
            fontsize=tsize, fontweight="bold", zorder=3, linespacing=1.0)


def plain_box(ax, x, y, w, h, title, fc, ec):
    rbox(ax, x, y, w, h, fc, ec, lw=1.3)
    ax.text(x + w / 2, y + 2.6, title, ha="center", va="center", color=TXT, fontsize=7.2,
            fontweight="bold", linespacing=1.0, zorder=3)


def bullets(ax, x, y, lines, dy=1.9, size=BODY, gap=0.5):
    """Bullet list; a line starting with two spaces continues the previous bullet."""
    for line in lines:
        if line.startswith("  "):
            ax.text(x + 1.6, y, line.strip(), fontsize=size, va="top", color=TXT, zorder=3)
            y += dy
        else:
            y += gap
            ax.text(x, y, "•", fontsize=size, va="top", color=TXT, zorder=3)
            ax.text(x + 1.6, y, line, fontsize=size, va="top", color=TXT, zorder=3)
            y += dy
    return y


def rule(ax, x0, x1, y, color="#A9B4C2"):
    ax.plot([x0, x1], [y, y], color=color, lw=0.6, zorder=3)


def arrow(ax, pts, text=None, tpos=None, dashed=False, lw=1.4, color=TXT, size=7.5, ha="center"):
    for (xa, ya), (xb, yb) in zip(pts[:-2], pts[1:-1]):
        ax.plot([xa, xb], [ya, yb], color=color, lw=lw, ls=(0, (4, 2)) if dashed else "-",
                zorder=4, solid_capstyle="butt")
    ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle="-|>,head_length=4,head_width=2.4",
                                 color=color, lw=lw, ls=(0, (4, 2)) if dashed else "-",
                                 shrinkA=0, shrinkB=0, zorder=4))
    if text:
        ax.text(*tpos, text, fontsize=size, style="italic", ha=ha, va="center", color=TXT,
                zorder=5, linespacing=1.1)


def tag(ax, x, y, w, h, text):
    rbox(ax, x, y, w, h, LABEL_FILL, LABEL_EDGE, lw=0.9, r=0.8, z=5)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=5.2,
            fontweight="bold", color=TXT, zorder=6, linespacing=1.1)


# --- icons ---------------------------------------------------------------------------------------
def icon_grid_map(ax, x, y, w=21, h=11.5):
    n_x, n_y = 12, 6
    cw, ch = w / n_x, h / n_y
    dark = {(1, 0), (1, 1), (5, 0), (5, 1), (3, 2), (2, 2), (8, 2), (8, 3), (5, 4), (6, 4), (5, 5),
            (6, 5)}
    for i in range(n_x):
        for j in range(n_y):
            ax.add_patch(Rectangle((x + i * cw, y + j * ch), cw, ch,
                                   fc="#4A4F57" if (i, j) in dark else "white",
                                   ec="#C9CED6", lw=0.3, zorder=3))
    px = [x + 1.2, x + 5.5, x + 8.0, x + 11.5, x + 14.0, x + 15.6, x + 19.8]
    py = [y + 9.6, y + 9.6, y + 8.4, y + 6.4, y + 3.9, y + 1.4, y + 1.4]
    ax.plot(px, py, color="#1E78D7", lw=1.3, zorder=4)
    ax.add_patch(Circle((px[0], py[0]), 0.65, fc="#E53935", ec="none", zorder=5))
    ax.plot(px[-1], py[-1], marker="*", ms=7, color="#2E9E44", zorder=5)


def icon_fields(ax, cx, cy):
    for r, c in [(7.3, "#3A8DDE"), (5.5, "#2CB67D"), (3.9, "#F2C744"), (2.5, "#F28C28")]:
        ax.add_patch(Arc((cx, cy), 2 * r, 2 * r * 1.25, theta1=180, theta2=360, color=c, lw=2.1,
                         zorder=3))
    ax.add_patch(FancyBboxPatch((cx - 1.1, cy - 1.0), 2.2, 2.0,
                                boxstyle="round,pad=0,rounding_size=0.4", fc="#2B3440",
                                ec="none", zorder=4))


def icon_robot(ax, cx, cy, s=1.0):
    bw, bh = 10 * s, 13 * s
    for dx in (-1, 1):
        for dy in (-1, 1):
            wx = cx + dx * (bw / 2 + 0.2) - 1.3 * s
            wy = cy + dy * (bh / 2 - 2.6 * s) - 1.8 * s
            ax.add_patch(Rectangle((wx, wy), 2.6 * s, 3.6 * s, fc="#3B3F45", ec="#202326",
                                   lw=0.5, zorder=3))
            for k in range(4):
                ax.plot([wx + 0.3 * s, wx + 2.3 * s],
                        [wy + (0.6 + 0.8 * k) * s, wy + (1.2 + 0.8 * k) * s],
                        color="#9AA0A8", lw=0.5, zorder=4)
    ax.add_patch(FancyBboxPatch((cx - bw / 2, cy - bh / 2), bw, bh,
                                boxstyle=f"round,pad=0,rounding_size={1.0 * s}", fc="#F2A900",
                                ec="#7A5600", lw=0.7, zorder=4))
    for sx, sy in [(-1, -1), (1, 1)]:
        ax.add_patch(Circle((cx + sx * (bw / 2 - 1.3 * s), cy + sy * (bh / 2 - 1.3 * s)), 1.1 * s,
                            fc="#2B3440", ec="none", zorder=5))
    ax.annotate("", xy=(cx, cy - bh / 2 + 1.0 * s), xytext=(cx, cy + 1.5 * s),
                arrowprops=dict(arrowstyle="-|>", color="#202326", lw=0.8), zorder=6)


def icon_warehouse(ax, x, y, w=17.5, h=17.5):
    ax.add_patch(Rectangle((x, y), w, h, fc="#F4F6F9", ec="#B8C0CC", lw=0.6, zorder=3))
    for row in range(3):
        ry = y + 1.5 + row * 5.6
        ax.add_patch(Rectangle((x + 1.2, ry), w - 6.5, 1.8, fc="#5C7FB0", ec="none", zorder=4))
        for k in range(5):
            ax.add_patch(Rectangle((x + 1.6 + k * 2.1, ry - 1.6), 1.7, 1.6, fc="#C99A62",
                                   ec="#8A6533", lw=0.3, zorder=4))
    ax.plot([x + w - 3.2, x + w - 3.2, x + 2.0], [y + 2.0, y + h - 2.2, y + h - 2.2],
            color="#1E78D7", lw=1.0, ls=(0, (3, 1.5)), zorder=4)
    ax.add_patch(Rectangle((x + w - 5.0, y + h - 7.4), 2.2, 2.2, fc="#7A4E2D", ec="none", zorder=5))
    ax.add_patch(FancyBboxPatch((x + w - 4.3, y + 3.0), 2.4, 3.0,
                                boxstyle="round,pad=0,rounding_size=0.4", fc="#F2A900",
                                ec="#7A5600", lw=0.5, zorder=5))


def icon_scanner(ax, cx, cy, flip=False):
    """A scanner housing with its 270-degree fan, seen from above (y axis inverted)."""
    start, end = (90, 360) if not flip else (270, 540)
    ax.add_patch(Wedge((cx, cy), 3.2, start, end, fc="#9CC9F2", ec="#5DA3E0", lw=0.4,
                       alpha=0.75, zorder=3))
    ax.add_patch(Circle((cx, cy), 1.5, fc="#2B3440", ec="none", zorder=4))
    ax.add_patch(Circle((cx, cy), 0.7, fc="#6BAA3A", ec="none", zorder=5))


def main() -> int:
    style.apply()
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["mathtext.fontset"] = "dejavuserif"
    fig = plt.figure(figsize=(7.4, 4.95))
    ax = fig.add_axes((0.0, 0.0, 1.0, 1.0))
    ax.set_xlim(0, 158)
    ax.set_ylim(103, 0)
    ax.set_aspect("equal")
    ax.axis("off")

    # shared configuration
    rbox(ax, 92.5, 0.8, 24, 7.0, "#E6EFFA", "#7FA7D6", lw=1.0)
    ax.text(104.5, 4.3, "Same field definition\n(shared configuration,\nindependent code)",
            ha="center", va="center", fontsize=6.3, linespacing=1.05, zorder=3)
    arrow(ax, [(92.5, 4.3), (84.5, 4.3), (84.5, 10.6)], dashed=True, lw=1.0)
    arrow(ax, [(116.5, 4.3), (124.5, 4.3), (124.5, 11.0)], dashed=True, lw=1.0)

    # 1 global planner
    x, y, w, h = 1.0, 11.5, 28.5, 38.5
    header_box(ax, x, y, w, h, "Global Planner")
    icon_grid_map(ax, x + 3.7, y + 6.3)
    yy = bullets(ax, x + 1.6, y + 19.0, ["A* on the baked occupancy", "  map (0.05 m grid)",
                                         "unmapped obstacles are", "  absent from the map",
                                         "two-tier inflation", "  (lethal and soft margins)"])
    rule(ax, x + 1.5, x + w - 1.5, yy + 0.6)
    ax.text(x + 1.6, yy + 1.4, r"$\bf{Output:}$ route; lookahead" "\npoint 0.8 m ahead, goal",
            fontsize=BODY, va="top", linespacing=1.15, zorder=3)

    # 2 local planner
    x, y, w, h = 38.0, 11.5, 25.0, 38.5
    header_box(ax, x, y, w, h, "Local Planner")
    ax.text(x + w / 2, y + 6.4, "classical", fontsize=5.8, style="italic", ha="center",
            va="center", color="#4A5568", zorder=3)
    for k, name in enumerate(["B1 Pure Pursuit", "B2 Regulated PP", "B3 DWA",
                              "B6 Field-aware MPPI"]):
        by = y + 7.6 + k * 2.75
        rbox(ax, x + 2.0, by, w - 4.0, 2.35, "#DCE6F2", "#9DB3CF", lw=0.6, r=0.5, z=3)
        ax.text(x + w / 2, by + 1.18, name, ha="center", va="center", fontsize=6.0,
                fontweight="bold", zorder=4)
    ax.text(x + w / 2, y + 19.3, "learned", fontsize=5.8, style="italic", ha="center",
            va="center", color="#4A5568", zorder=3)
    rbox(ax, x + 2.0, y + 20.4, w - 4.0, 2.35, "#E3DDF2", "#A795CC", lw=0.6, r=0.5, z=3)
    ax.text(x + w / 2, y + 21.6, "SAC policy (L1–L5)", ha="center", va="center", fontsize=6.0,
            fontweight="bold", zorder=4)
    yy = bullets(ax, x + 1.6, y + 23.2, ["classical: route cap,", "  recovery after a stop"],
                 dy=2.0)
    rule(ax, x + 1.5, x + w - 1.5, yy + 0.4)
    ax.text(x + w / 2, yy + 1.1, r"$\bf{Output:}$ $u_{\mathrm{nom}}=(v_x,v_y,\omega)$",
            fontsize=BODY, ha="center", va="top", zorder=3)
    ax.text(x + w / 2, yy + 3.6, "(classical: 120-sector scan;\npolicy: 36 sectors + stop flag)",
            fontsize=5.0, ha="center", va="top", color="#4A5568", linespacing=1.1, zorder=3)

    # 3 PFB
    x, y, w, h = 70.5, 10.6, 29.0, 39.8
    rbox(ax, x - 0.7, y - 0.7, w + 1.4, h + 1.4, "none", "#8CC4F5", lw=2.4)
    header_box(ax, x, y, w, h, "PFB\n(Protective-Field Barrier)", fc=PFB_FILL, ec=PFB_EDGE, tsize=7.0,
               band="#14407A", band_h=6.4, lw=1.6)
    rbox(ax, x + 2.0, y + 8.0, w - 4.0, 6.6, "#F3F8FE", "#9DB3CF", lw=0.6, r=0.6, z=3)
    ax.text(x + w / 2, y + 9.9, r"$g_k(u)\;\geq\;(1-\alpha\Delta t)\,\max(h_k,0)$",
            fontsize=6.6, ha="center", va="center", zorder=4)
    ax.text(x + w / 2, y + 12.6, r"$\alpha = 1\ \mathrm{s}^{-1}$ (frozen)", fontsize=6.4,
            ha="center", va="center", zorder=4)
    yy = bullets(ax, x + 1.8, y + 15.2, ["one QP per regime", "  (at rest + 3 speed bands)",
                                         r"minimal change to $u_{\mathrm{nom}}$ within",
                                         "  wheel, box, acceleration limits",
                                         "maximal braking if no regime", "  is feasible",
                                         "under a latch: only releasable", "  commands pass"],
                 dy=2.0, gap=0.45)
    rule(ax, x + 1.5, x + w - 1.5, yy + 0.4)
    ax.text(x + w / 2, yy + 1.1, r"$\bf{Output:}$ $u_{\mathrm{PFB}}$", fontsize=BODY,
            ha="center", va="top", zorder=3)
    ax.text(x + w / 2, y + h - 1.6, "identity when off (B1–B3, B6, L1, L2)",
            fontsize=5.0, style="italic", ha="center", va="center", color="#4A5568", zorder=3)

    # 4 certified layer
    x, y, w, h = 108.0, 11.0, 33.5, 39.0
    header_box(ax, x, y, w, h, "Certified Safety Layer", fc="#FDF3E7", ec="#B26A00",
               band="#8A4B00")
    icon_fields(ax, x + 10.0, y + 14.3)
    for k, (lab, c) in enumerate([("0.51 m/s", "#3A8DDE"), ("0.30 m/s", "#2CB67D"),
                                  ("0.10 m/s", "#F2C744"), ("at rest", "#F28C28")]):
        ly = y + 6.4 + k * 2.3
        ax.add_patch(Rectangle((x + 20.0, ly - 0.7), 3.0, 1.4, fc=c, ec="none", zorder=3))
        ax.text(x + 23.8, ly, lab, fontsize=6.0, va="center", zorder=3)
    yy = bullets(ax, x + 1.6, y + 16.4, ["speed-banded protective fields,",
                                         "  selected by reachable speed",
                                         "reach time 0.15 s (response 0.10 s)",
                                         "latched stop when a valid return",
                                         "  (or any invalid sector) is inside",
                                         "restart: field clear + 2 s delay"],
                 dy=2.0, gap=0.45)
    rule(ax, x + 1.5, x + w - 1.5, yy + 0.4)
    ax.text(x + w / 2, yy + 1.1, r"$\bf{Output:}$ $u_{\mathrm{PFB}}$, or zero while latched;"
            "\nstop flag", fontsize=BODY, ha="center", va="top", linespacing=1.15, zorder=3)
    ax.text(x + w / 2, y + h - 1.6, "emulated; active beneath every controller",
            fontsize=5.0, style="italic", ha="center", va="center", color="#4A5568", zorder=3)

    # 5 robot base
    x, y, w, h = 121.0, 58.0, 33.0, 33.0
    plain_box(ax, x, y, w, h, "Robot Base\n(KUKA youBot, mecanum)", GREEN_FILL, GREEN_EDGE)
    ax.texts[-1].set_position((x + w / 2, y + 3.4))
    icon_robot(ax, x + 7.6, y + 15.8, s=0.62)
    bullets(ax, x + 14.0, y + 7.7, [r"$|v_x|,|v_y|\leq 0.5$ m/s", r"$|\omega|\leq 1.5$ rad/s",
                                    "wheel budget", r"  $0.95\times 0.617$ m/s",
                                    "acc. limiter", "  0.5 m/s², 1.4 rad/s²"],
            dy=2.0, gap=0.45, size=6.0)
    rule(ax, x + 1.5, x + w - 1.5, y + 25.2)
    ax.text(x + 1.8, y + 26.2, r"$\bf{Output:}$ applied command $u_{\mathrm{app}}$",
            fontsize=BODY, va="top", zorder=3)

    # 6 simulator
    x, y, w, h = 60.5, 66.0, 38.0, 27.0
    plain_box(ax, x, y, w, h, "Simulator (CoppeliaSim, ODE)", PURPLE_FILL, PURPLE_EDGE)
    icon_warehouse(ax, x + 1.6, y + 6.2)
    bullets(ax, x + 20.6, y + 5.6, ["control period", "  0.05 s (20 Hz)", "dynamics step",
                                    "  0.005 s", "warehouse halls,", "  unmapped obstacles",
                                    "ground-truth", "  contact detection"],
            dy=2.0, gap=0.45, size=6.1)

    # 7 sensing
    x, y, w, h = 4.0, 67.0, 31.0, 31.5
    plain_box(ax, x, y, w, h, "Sensing", GREEN_FILL, GREEN_EDGE)
    rx, ry = x + 12.5, y + 7.0
    icon_scanner(ax, rx, ry)
    icon_scanner(ax, rx + 6.0, ry + 4.0, flip=True)
    ax.add_patch(Rectangle((rx, ry), 6.0, 4.0, fc="#F2A900", ec="#7A5600", lw=0.6, zorder=4))
    bullets(ax, x + 1.8, y + 13.3, ["two S300-type scanners,", "  270° each, diagonal corners",
                                    "filter stream 120 × 3°; policy",
                                    "  stream 36 × 10°; 0.05–5 m",
                                    "validity flags; plausibility", "  check (fail-safe)",
                                    r"velocity estimate $\hat{v}$:", "  low-pass 0.1 s"],
            dy=1.95, gap=0.35, size=6.1)

    # --- command path -----------------------------------------------------------------------------
    arrow(ax, [(29.5, 29.0), (38.0, 29.0)], "route", (33.7, 26.4))
    arrow(ax, [(63.0, 29.0), (69.8, 29.0)], r"$u_{\mathrm{nom}}$", (66.4, 26.4))
    arrow(ax, [(100.2, 29.0), (108.0, 29.0)], r"$u_{\mathrm{PFB}}$", (104.1, 26.4))
    arrow(ax, [(141.5, 29.0), (146.0, 29.0), (146.0, 58.0)],
          r"$u_{\mathrm{cert}}$" "\n(zero while\nlatched)", (151.5, 39.0), size=6.2)
    arrow(ax, [(121.0, 78.5), (98.5, 78.5)], "applied\ncommand", (110.0, 74.6), size=6.8)
    arrow(ax, [(60.5, 78.5), (35.0, 78.5)], "robot state,\nenvironment", (47.8, 74.6), size=6.8)

    # --- sensing bus --------------------------------------------------------------------------------
    bus_y = 59.0
    ax.plot([17.0, 17.0], [67.0, bus_y], color=TXT, lw=1.4, zorder=4)
    ax.plot([17.0, 124.5], [bus_y, bus_y], color=TXT, lw=1.4, zorder=4)
    arrow(ax, [(50.5, bus_y), (50.5, 50.0)])
    arrow(ax, [(85.0, bus_y), (85.0, 51.1)])
    arrow(ax, [(124.5, bus_y), (124.5, 50.0)])
    tag(ax, 28.5, 52.4, 21.0, 4.4, "scans (120 / 36 sectors),\nvelocity estimate")
    tag(ax, 63.0, 52.4, 20.8, 4.4, "scan (120 sectors),\nvelocity estimate")
    tag(ax, 99.5, 52.4, 23.5, 4.4, "scan (120 sectors),\nvelocity, applied command")

    style.save(fig, "fig_architecture.pdf")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
