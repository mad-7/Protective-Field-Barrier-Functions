"""CoDIT figure: the classical planners on the tuning set, and B7 against the PFB gain alpha.

(a) protective stops per 100 m driven (log scale) and (b) the share of steps latched, per planner;
(c) B7's stops per 100 m and the PFB's fallback steps against alpha. Tune-set, preliminary.

By default the values are read from `paper/codit2026/results_tune.json` (each with its run directory and
its decision-log source). On the training host, `--from-runs` recomputes (a)-(b) from the runs' episode
logs with the project's own summary (`pfbnav.eval.runner.outcome_summary`) and (c) from the alpha
tuning's `summary.json`, so the figure can be checked against the runs themselves.

    FIG_OUT=paper/codit2026/figures python paper/figures/scripts/fig_codit_results.py [--from-runs]
"""
from __future__ import annotations

import argparse
import json

import matplotlib.pyplot as plt
import numpy as np
import style

from pfbnav.config import load_config
from pfbnav.eval.runner import outcome_summary

DATA = style.REPO / "paper" / "codit2026" / "results_tune.json"
ORDER = ["B1", "B2", "B3", "B6", "B7"]


def from_runs(data: dict) -> dict:
    dt = float(load_config(style.CONFIG)["sim"]["dt"])
    for key in ORDER:
        p = data["planners"][key]
        recs = [json.loads(line) for line in (style.REPO / p["run"] / "episodes.jsonl")
                .read_text(encoding="utf-8").splitlines() if line]
        s = outcome_summary(recs, dt)
        p.update(success=s["success_rate"], stops_driven=s["stops_per_100m"],
                 latched=s["latched_step_share"], moving=s["moving_speed_mps"])
    summary = json.loads((style.REPO / "runs/tuning/B7_alpha_20261102T195759Z_seed0/summary.json")
                         .read_text(encoding="utf-8"))
    a = data["alpha"]
    a["values"] = summary["grid_per_s"]
    a["success"] = [c["success_rate"] for c in summary["scores"]]
    a["stops_driven"] = [c["stops_per_100m"] for c in summary["scores"]]
    a["fallback_steps"] = [c["pfb_fallback_steps"] for c in summary["scores"]]
    return data


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--from-runs", action="store_true")
    args = parser.parse_args()
    data = json.loads(DATA.read_text(encoding="utf-8"))
    if args.from_runs:
        data = from_runs(data)
    style.apply()
    pl = data["planners"]
    short = {"B1": "PP", "B2": "RPP", "B3": "DWA", "B6": "MPPI", "B7": "RPP\n+PFB"}
    labels = [f"{k}\n{short[k]}" for k in ORDER]
    colours = [style.GREY, style.SKY, style.ORANGE, style.PURPLE, style.GREEN]
    fig, axes = plt.subplots(1, 3, figsize=(style.TEXT_WIDTH_IN, 2.1),
                             gridspec_kw={"width_ratios": [1.15, 1.15, 1.0]})
    x = np.arange(len(ORDER))

    ax = axes[0]
    stops = [pl[k]["stops_driven"] for k in ORDER]
    ax.bar(x, stops, color=colours, edgecolor=style.BLACK, lw=0.4)
    ax.set_yscale("log")
    ax.set_ylim(0.1, 2000)
    for i, v in enumerate(stops):
        ax.text(i, v * 1.25, f"{v:.2g}" if v < 10 else f"{v:.0f}", ha="center", va="bottom",
                fontsize=6)
    ax.set_xticks(x, labels, fontsize=6)
    ax.set_ylabel("protective stops / 100 m")
    style.panel_label(ax, "a")

    ax = axes[1]
    latched = [100 * pl[k]["latched"] for k in ORDER]
    succ = [100 * pl[k]["success"] for k in ORDER]
    w = 0.38
    ax.bar(x - w / 2, latched, w, color=colours, edgecolor=style.BLACK, lw=0.4, label="time latched")
    ax.bar(x + w / 2, succ, w, color="white", edgecolor=colours, lw=0.9, hatch="////",
           label="success")
    ax.set_ylim(0, 100)
    ax.set_xticks(x, labels, fontsize=6)
    ax.set_ylabel("% of steps / % of routes")
    ax.set_ylim(0, 115)
    ax.legend(loc="upper right", fontsize=5.5, ncol=1, handlelength=1.2)
    style.panel_label(ax, "b")

    ax = axes[2]
    a = data["alpha"]
    ax.plot(a["values"], a["stops_driven"], "o-", color=style.GREEN, ms=3, label="stops / 100 m")
    ax.set_xscale("log")
    ax.set_xticks(a["values"], [f"{v:g}" for v in a["values"]])
    ax.set_xlabel(r"PFB gain $\alpha$ (s$^{-1}$)")
    ax.set_ylabel("B7 stops / 100 m", color=style.GREEN)
    twin = ax.twinx()
    twin.plot(a["values"], a["fallback_steps"], "s--", color=style.VERMILLION, ms=3)
    twin.set_yscale("log")
    twin.set_ylabel("fallback steps", color=style.VERMILLION)
    for spine, c in (("left", style.GREEN), ("right", style.VERMILLION)):
        twin.spines[spine].set_color(c)
    style.panel_label(ax, "c")
    fig.tight_layout(w_pad=0.6)
    style.save(fig, "fig_codit_results.pdf")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
