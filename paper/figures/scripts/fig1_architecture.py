"""Figure 1: the system architecture. Every quantity in a label is read from configs/base.yaml (or
derived from it by the project's own functions); the block order is the step loop's
(`pfbnav/envs/step_loop.py`): PFB slot, certified layer (decides on the scan in hand, overrides),
base (box, wheel budget, limiter), physics, sensing, velocity estimate.
"""
from __future__ import annotations

import matplotlib.pyplot as plt
import style
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

from pfbnav.config import load_config
from pfbnav.safety import pfb_field as pf

style.apply()
cfg = load_config(style.CONFIG)
p = pf.field_params_from_config(cfg)
act, sc, cert = cfg["actuation"], cfg["scanners"], cfg["certified"]
budget = act["wheel_budget"]
NL = "\n"

fig, ax = plt.subplots(figsize=(style.TEXT_WIDTH_IN, 2.9))
ax.set_xlim(0, 10.2)
ax.set_ylim(1.45, 6.1)
ax.axis("off")


def box(x, y, w, h, title, lines, colour):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.04,rounding_size=0.12",
                                fc=colour, ec=style.BLACK, lw=0.6))
    ax.text(x + w / 2, y + h - 0.14, title, ha="center", va="top", fontweight="bold", fontsize=6.5)
    ax.text(x + w / 2, y + h - 0.48, NL.join(lines), ha="center", va="top", fontsize=5.4,
            linespacing=1.3)


def arrow(a, b, text=None, offset=(0.0, 0.06), rad=0.0, va="bottom"):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=7, lw=0.7,
                                 color=style.BLACK, connectionstyle=f"arc3,rad={rad}"))
    if text:
        ax.text((a[0] + b[0]) / 2 + offset[0], (a[1] + b[1]) / 2 + offset[1], text, ha="center",
                va=va, fontsize=5.4)


tint = {"plan": "#E8F1FA", "ctrl": "#FFF4D6", "pfb": "#DDEFE8", "cert": "#FBE3D6",
        "base": "#EEEEEE", "sense": "#EFE6F3"}
bands = ", ".join(f"{v:g}" for v in p.band_tops)
box(0.1, 4.3, 2.1, 1.7, "Global planner",
    ["baked map", "(unmapped excluded)", "A*, two-tier inflation",
     f"lookahead {cfg['planner']['lookahead']:g} m"], tint["plan"])
box(2.75, 4.3, 2.2, 1.7, "Controller",
    ["learned policy (82-D)", "or classical baseline", "(route cap, recovery)"], tint["ctrl"])
box(5.5, 4.3, 1.9, 1.7, "PFB", ["one QP per regime:", "rest and 3 bands", "identity when off"],
    tint["pfb"])
box(7.95, 4.3, 2.15, 1.7, "Certified layer",
    [f"bands {bands} m/s", f"reach time {p.reach_time:.2f} s", "latched stop",
     f"restart delay {cert['restart_delay']:g} s"], tint["cert"])
box(7.8, 2.0, 2.35, 1.6, "Base",
    [f"box {act['v_max']:g}, {act['vy_max']:g} m/s; {act['w_max']:g} rad/s",
     f"budget {budget['filter_fraction']:g} x {budget['v_rim']:g} m/s",
     f"limiter {act['a_max']:g} m/s$^2$, {act['alpha_max']:g} rad/s$^2$"], tint["base"])
box(4.75, 2.0, 2.45, 1.6, "Simulator (ODE)",
    [f"control period {cfg['sim']['dt']:g} s",
     f"dynamics step {cfg['sim']['physics']['dynamic_step']:g} s", "ground-truth contact"],
    tint["base"])
box(0.1, 1.55, 4.15, 2.05, "Sensing",
    ["two 270$^\\circ$ S300-type corner scanners",
     f"policy stream: {sc['policy_bins']} sectors and validity",
     f"filter stream: {sc['filter_bins']} sectors with measured",
     "bearing and validity",
     f"range {sc['range_min']:g}-{sc['range_max']:g} m; plausibility check, fail-safe",
     f"velocity estimate (low-pass {cfg['estimation']['velocity']['filter_tau']:g} s)"],
    tint["sense"])

arrow((2.2, 5.15), (2.75, 5.15), "route")
arrow((4.95, 5.15), (5.5, 5.15), "$\\mathbf{u}^{\\mathrm{req}}$")
arrow((7.4, 5.15), (7.95, 5.15), "$\\mathbf{u}^{\\mathrm{PFB}}$")
arrow((9.0, 4.3), (9.0, 3.6), "zero while latched", offset=(0.62, -0.05), va="center")
arrow((7.8, 2.8), (7.2, 2.8), "applied", offset=(0.0, 0.05))
arrow((4.75, 2.8), (4.25, 2.8))
arrow((2.6, 3.6), (3.6, 4.3), "36 sectors", offset=(-0.55, -0.02), va="center")
arrow((3.6, 3.6), (6.2, 4.3), "120 sectors", offset=(0.05, -0.2), va="center")
arrow((4.0, 3.6), (8.4, 4.3), rad=-0.05)
style.save(fig, "fig1_architecture.pdf")
