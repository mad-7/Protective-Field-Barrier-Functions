"""Control chain of the proposed system (classical controllers only), drawn as a formal block diagram (vector, monochrome with one emphasised block).

Every quantity in a label is a configured or ruled value quoted in the manuscript. Run from the repository root:

    FIG_OUT=paper/latex_classical/figures python paper/latex_classical/figure_scripts/fig1_architecture_classical.py

(PowerShell: $env:FIG_OUT="paper/latex_classical/figures"; python paper/latex_classical/figure_scripts/fig1_architecture_classical.py).
Imports style.py from paper/draft/figures/scripts, unchanged.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "draft" / "figures" / "scripts"))

import matplotlib.pyplot as plt  # noqa: E402
import style  # noqa: E402
from matplotlib.patches import FancyArrowPatch, Rectangle  # noqa: E402

style.apply()
plt.rcParams["font.family"] = "serif"
plt.rcParams["mathtext.fontset"] = "cm"

W, H = 160.0, 66.0
fig = plt.figure(figsize=(style.TEXT_WIDTH_IN, style.TEXT_WIDTH_IN * H / W))
ax = fig.add_axes((0, 0, 1, 1))
ax.set_xlim(0, W)
ax.set_ylim(H, 0)
ax.axis("off")

INK = "#000000"
GREY = "#E6E6E6"
BOX_W, BOX_H, GAP = 22.0, 20.0, 5.0
ROW_Y = 22.0                      # centre of the command-path row
X0 = 2.0


def block(i, title, lines, fill="white", lw=0.9, bold_lw=None):
    x = X0 + i * (BOX_W + GAP)
    ax.add_patch(Rectangle((x, ROW_Y - BOX_H / 2), BOX_W, BOX_H, fc=fill, ec=INK, lw=bold_lw or lw, zorder=2))
    ax.text(x + BOX_W / 2, ROW_Y - BOX_H / 2 + 3.4, title, ha="center", va="center", fontsize=6.6,
            fontweight="bold", zorder=3)
    ax.text(x + BOX_W / 2, ROW_Y + 2.6, "\n".join(lines), ha="center", va="center", fontsize=5.9, zorder=3,
            linespacing=1.35)
    return x


def arrow(p, q, lw=0.9, dashed=False):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>,head_length=3.2,head_width=1.8", color=INK, lw=lw,
                                 ls=(0, (3, 2)) if dashed else "-", shrinkA=0, shrinkB=0, zorder=4))


def seg(p, q, lw=0.9):
    ax.plot([p[0], q[0]], [p[1], q[1]], color=INK, lw=lw, zorder=4, solid_capstyle="butt")


def label(x, y, text, size=6.4, ha="center", va="bottom"):
    ax.text(x, y, text, ha=ha, va=va, fontsize=size, zorder=5)


xs = {}
xs["plan"] = block(0, "Global planner", ["A$^{*}$ on the map,", "which may omit", "obstacles"])
xs["ctrl"] = block(1, "Controller", ["B1 pure pursuit,", "B2 regulated PP,", "B3 DWA"])
xs["pfb"] = block(2, "PFB filter", [r"$\mathcal{P}$: one QP per", "speed regime;", "identity when off"],
                  fill=GREY, bold_lw=1.8)
xs["cert"] = block(3, "Certified layer", [r"$\mathcal{C}$: speed-banded", "fields, latched stop,", "restart condition"])
xs["act"] = block(4, "Actuation limits", ["axis box,", "wheel-speed budget,", "acceleration limiter"])
xs["robot"] = block(5, "Robot base", ["mecanum base,", "CoppeliaSim"])

# command path
labels = ["route", r"$u_{\mathrm{nom}}$", r"$u_{\mathrm{PFB}}$", r"$u$", r"$u_{\mathrm{app}}$"]
order = ["plan", "ctrl", "pfb", "cert", "act", "robot"]
for k, text in enumerate(labels):
    xa = xs[order[k]] + BOX_W
    xb = xs[order[k + 1]]
    arrow((xa, ROW_Y), (xb, ROW_Y))
    label(0.5 * (xa + xb), ROW_Y - 1.2, text, size=(5.0 if k == 0 else 6.2))

# shared field parameters (dashed link above the two blocks)
x_pfb_c = xs["pfb"] + BOX_W / 2
x_cert_c = xs["cert"] + BOX_W / 2
top = ROW_Y - BOX_H / 2
seg((x_pfb_c, top), (x_pfb_c, 5.0))
seg((x_cert_c, top), (x_cert_c, 5.0))
ax.plot([x_pfb_c, x_cert_c], [5.0, 5.0], color=INK, lw=0.9, ls=(0, (3, 2)), zorder=4)
label(0.5 * (x_pfb_c + x_cert_c), 4.0, "same field parameters, independent code", size=6.0)

# sensing and the measurement bus
SENS_Y = 52.0
sx0, sx1 = xs["plan"], xs["act"] + BOX_W
ax.add_patch(Rectangle((xs["ctrl"], SENS_Y - 7.0), xs["cert"] + BOX_W - xs["ctrl"], 14.0, fc="white", ec=INK,
                       lw=0.9, zorder=2))
ax.text(0.5 * (xs["ctrl"] + xs["cert"] + BOX_W), SENS_Y - 3.2, "Sensing", ha="center", va="center", fontsize=6.6,
        fontweight="bold", zorder=3)
ax.text(0.5 * (xs["ctrl"] + xs["cert"] + BOX_W), SENS_Y + 2.6,
        "two scanners, 270" + r"$^\circ$" + " each; scan streams of 36 and 120\nsectors with validity flags; velocity estimate " + r"$\hat v$",
        ha="center", va="center", fontsize=5.9, zorder=3)
bottom = ROW_Y + BOX_H / 2
for key in ("ctrl", "pfb", "cert"):
    xc = xs[key] + BOX_W / 2
    arrow((xc, SENS_Y - 7.0), (xc, bottom))
label(xs["ctrl"] + BOX_W / 2 + 1.0, 39.0, r"scan, $\hat v$", size=6.0, ha="left", va="center")
label(xs["pfb"] + BOX_W / 2 + 1.0, 39.0, r"scan, $\hat v$", size=6.0, ha="left", va="center")
label(xs["cert"] + BOX_W / 2 + 1.0, 39.0, r"scan, $\hat v$", size=6.0, ha="left", va="center")

# feedback from the robot to the sensing block
xr = xs["robot"] + BOX_W / 2
seg((xr, bottom), (xr, SENS_Y))
arrow((xr, SENS_Y), (xs["cert"] + BOX_W, SENS_Y))
label(0.5 * (xr + xs["cert"] + BOX_W), SENS_Y - 1.0, "robot state, environment", size=6.0)

style.save(fig, "fig1_architecture.pdf")
