"""Journal draft, held-out figure: the classical planners on both halves of the sealed test set.

Reads paper/journal/results_test.json (values from docs/decision_log.md, 2026-11-08). Colour encodes the
test condition; hatching repeats it, so that identity survives greyscale printing. Run from the repository
root:

    python paper/journal/scripts/fig_journal_heldout.py
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
REPO = HERE.parents[3]
os.environ.setdefault("FIG_OUT", "paper/journal/figures")
os.environ.setdefault("FIG_TEXT_WIDTH", "6.3")
sys.path.insert(0, str(REPO / "paper" / "figures" / "scripts"))

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import style  # noqa: E402

DATA = REPO / "paper" / "journal" / "results_test.json"
ORDER = ["B1", "B2", "B3", "B6", "B7"]
HALVES = [("unmapped_static", "unmapped obstacle on every route", style.ORANGE, "////"),
          ("mapped_only", "no unmapped obstacle", style.BLUE, "")]
GAP = 0.03


def bars(ax, data, key):
    w = 0.38
    x = np.arange(len(ORDER))
    for sign, (half, label, colour, hatch) in zip((-1, 1), HALVES):
        values = [data[half]["arms"][a][key] for a in ORDER]
        ax.bar(x + sign * (w / 2 + GAP), values, w, color=colour, hatch=hatch, edgecolor="white",
               linewidth=0.3, label=label)
    ax.set_xticks(x, ORDER)
    ax.spines[["top", "right"]].set_visible(False)
    ax.tick_params(axis="x", length=0)


def main() -> int:
    data = json.loads(DATA.read_text(encoding="utf-8"))
    style.apply()
    fig, axes = plt.subplots(1, 3, figsize=(style.TEXT_WIDTH_IN, 2.35))

    ax = axes[0]
    bars(ax, data, "planned")
    ax.set_yscale("log")
    ax.set_ylim(0.05, 200)
    ax.set_ylabel("stops per 100 m planned")
    style.panel_label(ax, "a")

    ax = axes[1]
    bars(ax, data, "success")
    ax.set_ylim(0, 1.08)
    ax.set_ylabel("success rate (descriptive)")
    style.panel_label(ax, "b")

    ax = axes[2]
    bars(ax, data, "fraction")
    ax.axhline(1.0, color=style.GREY, lw=0.6, ls=(0, (3, 2)), zorder=0)
    ax.set_ylim(0, 1.3)
    ax.set_ylabel("distance driven / planned")
    style.panel_label(ax, "c")

    handles, labels = axes[0].get_legend_handles_labels()
    fig.tight_layout(w_pad=0.9, rect=(0, 0.07, 1, 1))
    fig.legend(handles, labels, loc="lower center", ncol=2, frameon=False)
    style.save(fig, "fig_journal_heldout.pdf")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
