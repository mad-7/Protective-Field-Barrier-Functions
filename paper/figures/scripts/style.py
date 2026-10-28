"""Shared style for the manuscript's figures: one look, a colour-blind-safe palette (Okabe-Ito),
sizes for the Sensors text width, and the paths every script writes to.

Every figure is generated from the project's own geometry, configuration and run directories; no
coordinate is typed by hand. Run each script from the repository root:

    python paper/figures/scripts/fig2_field_geometry.py
"""
from __future__ import annotations

import os
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[3]
OUT = REPO / "paper" / "MDPI_template" / "figures"
CONFIG = REPO / "configs" / "base.yaml"

# Okabe-Ito (Wong, Nature Methods 2011): distinguishable under the common colour-vision deficiencies.
BLACK = "#000000"
ORANGE = "#E69F00"
SKY = "#56B4E9"
GREEN = "#009E73"
YELLOW = "#F0E442"
BLUE = "#0072B2"
VERMILLION = "#D55E00"
PURPLE = "#CC79A7"
GREY = "#999999"
LIGHT = "#DDDDDD"

TEXT_WIDTH_IN = 5.35     # the Sensors template's text width, about 13.6 cm


def apply() -> None:
    plt.rcParams.update({
        "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8, "legend.fontsize": 7,
        "xtick.labelsize": 7, "ytick.labelsize": 7, "lines.linewidth": 1.0,
        "axes.linewidth": 0.6, "xtick.major.width": 0.6, "ytick.major.width": 0.6,
        "legend.frameon": False, "savefig.bbox": "tight", "savefig.pad_inches": 0.02,
        "pdf.fonttype": 42, "font.family": "sans-serif",
    })


def save(fig, name: str) -> Path:
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / name
    fig.savefig(path)
    preview = os.environ.get("FIG_PREVIEW_DIR")       # a PNG copy for checking, outside the paper
    if preview:
        fig.savefig(Path(preview) / (Path(name).stem + ".png"), dpi=200)
    plt.close(fig)
    print(f"-> {path.relative_to(REPO).as_posix()}")
    return path


def panel_label(ax, letter: str) -> None:
    ax.text(0.02, 0.98, f"({letter})", transform=ax.transAxes, ha="left", va="top",
            fontweight="bold")
