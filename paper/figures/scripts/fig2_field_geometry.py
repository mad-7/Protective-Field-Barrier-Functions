"""Figure 2: the protective fields on the measured footprint (Equations (field)-(extent)).

(a) the at-rest field and the three band fields, forward travel, no rotation;
(b) the band-1 field for travel forward, diagonally and laterally;
(c) the band-1 field for forward travel at three yaw rates.
Fields from `pfbnav/safety/pfb_field.py` (cross-checked equal to the certified layer's in
`tests/test_pfb_field.py`), footprint from `robot.footprint_table`, parameters from configs/base.yaml.
"""
from __future__ import annotations

import math

import matplotlib.pyplot as plt
import numpy as np
import style

from pfbnav.config import load_config
from pfbnav.safety import pfb_field as pf
from pfbnav.safety.footprint_table import FootprintTable

style.apply()
cfg = load_config(style.CONFIG)
table = FootprintTable.from_csv(style.REPO / cfg["robot"]["footprint_table"])
params = pf.field_params_from_config(cfg)
field = pf.PFBField(table.extent, params)
psi = np.linspace(-math.pi, math.pi, 1441)


def outline(rho):
    return rho * np.cos(psi), rho * np.sin(psi)


def footprint(ax):
    e = np.asarray(table.extent(psi), dtype=float)
    x, y = outline(e)
    ax.fill(x, y, color=style.LIGHT, lw=0)
    ax.plot(x, y, color=style.BLACK, lw=0.6)


def arrow(ax, heading, length=0.55, colour=style.BLACK):
    ax.annotate("", xy=(length * math.cos(heading), length * math.sin(heading)), xytext=(0, 0),
                arrowprops={"arrowstyle": "->", "lw": 0.8, "color": colour})


tops = params.band_tops
fig, axes = plt.subplots(1, 3, figsize=(style.TEXT_WIDTH_IN, 2.9))

ax = axes[0]
footprint(ax)
ax.plot(*outline(field.extent(psi, 0.0, 0.0, 0.0, at_rest=True)), color=style.BLACK, ls="--",
        label="at rest")
for top, colour in zip(tops, (style.SKY, style.GREEN, style.VERMILLION), strict=True):
    ax.plot(*outline(field.extent(psi, top, 0.0, 0.0)), color=colour,
            label=f"band top {top:.2f} m/s")
arrow(ax, 0.0)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=1)
style.panel_label(ax, "a")

ax = axes[1]
footprint(ax)
for heading, colour, name in ((0.0, style.GREEN, "forward"), (math.pi / 4, style.BLUE, "diagonal"),
                              (math.pi / 2, style.ORANGE, "lateral")):
    ax.plot(*outline(field.extent(psi, tops[1], heading, 0.0)), color=colour, label=name)
    arrow(ax, heading, colour=colour)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=1, title=f"band top {tops[1]:.2f} m/s")
style.panel_label(ax, "b")

ax = axes[2]
footprint(ax)
for w, colour in zip((0.0, 0.5, 1.0), (style.GREEN, style.PURPLE, style.BLACK), strict=True):
    ax.plot(*outline(field.extent(psi, tops[1], 0.0, w)), color=colour,
            label=f"$|\\omega|$ = {w:.1f} rad/s")
arrow(ax, 0.0)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=1, title=f"band top {tops[1]:.2f} m/s")
style.panel_label(ax, "c")

lim = 0.72
for ax in axes:
    ax.set_aspect("equal")
    ax.set_xlim(-lim + 0.1, lim + 0.1)
    ax.set_ylim(-lim, lim)
    ax.set_xlabel("$x$ (m)")
    ax.axhline(0, color=style.GREY, lw=0.3)
    ax.axvline(0, color=style.GREY, lw=0.3)
axes[0].set_ylabel("$y$ (m)")
for ax in axes[1:]:
    ax.set_yticklabels([])
fig.subplots_adjust(wspace=0.08)
style.save(fig, "fig2_field_geometry.pdf")
