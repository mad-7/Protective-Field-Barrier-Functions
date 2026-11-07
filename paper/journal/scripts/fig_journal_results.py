"""Journal draft, results figure: every arm on the DEVELOPMENT (tune) set, and B7 against the gain alpha.

Reads paper/journal/results_tune.json (values copied from docs/decision_log.md, 2026-11-07). L5 is a
PILOT checkpoint (seed 9000002); it is drawn hatched so that it is never read as a confirmatory result.
Run from the repository root:

    python paper/journal/scripts/fig_journal_results.py
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

DATA = REPO / "paper" / "journal" / "results_tune.json"
ORDER = ["B1", "B2", "B3", "B6", "B7", "L5", "oracle"]
SHORT = {"B1": "B1", "B2": "B2", "B3": "B3", "B6": "B6", "B7": "B7", "L5": "L5*", "oracle": "PT"}
COLOUR = {"B1": style.GREY, "B2": style.SKY, "B3": style.ORANGE, "B6": style.PURPLE,
          "B7": style.GREEN, "L5": style.BLUE, "oracle": style.LIGHT}


def main() -> int:
    data = json.loads(DATA.read_text(encoding="utf-8"))
    arms, alpha = data["arms"], data["alpha"]
    style.apply()
    fig, axes = plt.subplots(1, 3, figsize=(style.TEXT_WIDTH_IN, 2.3),
                             gridspec_kw={"width_ratios": [1.25, 1.25, 1.0]})
    x = np.arange(len(ORDER))
    hatch = ["////" if k == "L5" else "" for k in ORDER]
    edge = [style.BLACK if k in ("L5", "oracle") else "none" for k in ORDER]

    ax = axes[0]
    w = 0.38
    planned = [arms[k]["planned"] for k in ORDER]
    driven = [arms[k]["driven"] for k in ORDER]
    for i, k in enumerate(ORDER):
        ax.bar(x[i] - w / 2, planned[i], w, color=COLOUR[k], hatch=hatch[i], edgecolor=edge[i],
               linewidth=0.4)
        ax.bar(x[i] + w / 2, driven[i], w, color=COLOUR[k], alpha=0.45, hatch=hatch[i],
               edgecolor=edge[i], linewidth=0.4)
    ax.set_yscale("log")
    ax.set_ylim(0.2, 400)
    ax.set_xticks(x, [SHORT[k] for k in ORDER], fontsize=7)
    ax.set_ylabel("protective stops per 100 m")
    ax.text(0.98, 0.98, "solid: planned\npale: driven", transform=ax.transAxes, ha="right",
            va="top", fontsize=6)
    style.panel_label(ax, "a")

    ax = axes[1]
    succ = [arms[k]["success"] for k in ORDER]
    lat = [arms[k]["latched"] for k in ORDER]
    for i, k in enumerate(ORDER):
        ax.bar(x[i] - w / 2, succ[i], w, color=COLOUR[k], hatch=hatch[i], edgecolor=edge[i],
               linewidth=0.4)
        ax.bar(x[i] + w / 2, lat[i], w, color="white", hatch="....", edgecolor=style.BLACK,
               linewidth=0.4)
    ax.set_ylim(0, 1.05)
    ax.set_xticks(x, [SHORT[k] for k in ORDER], fontsize=7)
    ax.set_ylabel("fraction")
    ax.text(0.98, 0.98, "filled: success rate\ndotted: time latched", transform=ax.transAxes,
            ha="right", va="top", fontsize=6)
    style.panel_label(ax, "b")

    ax = axes[2]
    a = np.array(alpha["values"], dtype=float)
    ax.plot(a, alpha["stops_driven"], "o-", color=style.GREEN, ms=3, label="stops / 100 m driven")
    ax.set_xscale("log")
    ax.set_xticks(a, [f"{v:g}" for v in a])
    ax.set_xlabel(r"PFB gain $\alpha$ (1/s)")
    ax.set_ylabel("stops / 100 m driven", color=style.GREEN)
    ax2 = ax.twinx()
    ax2.plot(a, alpha["fallback_steps"], "s--", color=style.VERMILLION, ms=3)
    ax2.set_yscale("log")
    ax2.set_ylabel("fallback steps", color=style.VERMILLION)
    style.panel_label(ax, "c")

    fig.tight_layout(w_pad=0.8)
    style.save(fig, "fig_journal_results.pdf")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
