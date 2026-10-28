"""Figure 3: sensor coverage, measured by the dense post sweep, filter stream.

(a) the original layout, two 240 deg pods on the centreline (scene 340dbc5d);
(b) the adopted layout, two 270 deg S300-type scanners on diagonal corners (scene 0ae35a50).
Cells, classes, sensor poses and fields of view are read from the two coverage-map runs. Each panel
draws the footprint table in force when its run was made: the current table records the scene it was
built on, and a run on another scene gets the table of its own commit, read from git.
"""
from __future__ import annotations

import csv
import json
import math
import subprocess
import tempfile
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import style
from matplotlib.patches import Wedge

from pfbnav.config import load_config
from pfbnav.safety.footprint_table import FootprintTable

RUNS = (("runs/diag/COVMAP-centreline_20260922T163600Z_seed0", "a", "centreline 240$^\\circ$ pods"),
        ("runs/diag/COVMAP-s300_corners_20260922T214324Z_seed0", "b",
         "diagonal-corner 270$^\\circ$ scanners"))
COLOURS = {"observed": style.SKY, "invalid": style.ORANGE, "violation": style.VERMILLION,
           "confounded": style.PURPLE}

style.apply()
cfg = load_config(style.CONFIG)
TABLE = cfg["robot"]["footprint_table"]


def table_scene() -> str:
    """The scene SHA-256 the current footprint table was built on (its header)."""
    for line in (style.REPO / TABLE).read_text(encoding="utf-8").splitlines():
        if line.startswith("# scene_sha256:"):
            return line.split(":", 1)[1].strip()
    raise ValueError("the footprint table does not record its scene")


def footprint_for(scene_sha256: str, commit: str) -> FootprintTable:
    if scene_sha256 == table_scene():
        return FootprintTable.from_csv(style.REPO / TABLE)
    text = subprocess.run(["git", "show", f"{commit}:{TABLE}"], cwd=style.REPO, check=True,
                          capture_output=True, text=True).stdout
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "table.csv"
        path.write_text(text, encoding="utf-8")
        return FootprintTable.from_csv(path)


fig, axes = plt.subplots(1, 2, figsize=(style.TEXT_WIDTH_IN, 3.0), sharey=True)
psi = np.linspace(-math.pi, math.pi, 721)
for (run, letter, name), ax in zip(RUNS, axes, strict=True):
    d = style.REPO / run
    summary = json.loads((d / "summary.json").read_text(encoding="utf-8"))
    provenance = json.loads((d / "provenance.json").read_text(encoding="utf-8"))
    table = footprint_for(summary["scene_sha256"], provenance["commit"])
    with (d / "cells.csv").open(newline="") as f:
        cells = list(csv.DictReader(f))
    xs = np.array([float(c["x"]) for c in cells])
    ys = np.array([float(c["y"]) for c in cells])
    cls = np.array([c["filter_class"] for c in cells])
    for label, colour in COLOURS.items():
        sel = cls == label
        if sel.any():
            ax.scatter(xs[sel], ys[sel], s=1.4, marker="s", color=colour, lw=0,
                       label=f"{label} ({int(sel.sum())})")
    e = np.asarray(table.extent(psi), dtype=float)
    ax.fill(e * np.cos(psi), e * np.sin(psi), color=style.LIGHT, lw=0)
    ax.plot(e * np.cos(psi), e * np.sin(psi), color=style.BLACK, lw=0.6)
    for sensor in summary["coverage_geometry"]["sensors"]:
        half = float(sensor["fov_deg"]) / 2.0
        h = float(sensor["heading_deg"])
        ax.add_patch(Wedge((sensor["x"], sensor["y"]), 0.22, h - half, h + half, fill=False,
                           ec=style.BLACK, lw=0.5, ls=":"))
        ax.plot(sensor["x"], sensor["y"], marker="o", ms=2.5, color=style.BLACK)
    counts = summary["filter_counts"]
    ax.set_title(f"{name}\n{summary['coverage_geometry']['filter_valid']}/120 filter sectors valid; "
                 f"{counts['violation']} violations", fontsize=7)
    ax.set_aspect("equal")
    ax.set_xlim(-1.45, 1.45)
    ax.set_ylim(-1.3, 1.3)
    ax.set_xlabel("$x$ (m)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.18), ncol=2, markerscale=4)
    style.panel_label(ax, letter)
axes[0].set_ylabel("$y$ (m)")
fig.subplots_adjust(wspace=0.15)
style.save(fig, "fig3_coverage.pdf")
