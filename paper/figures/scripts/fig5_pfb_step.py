"""Figure 5: one PFB step, from the emulator (sector scan, as every new emulator analysis must use).

The scan-blind ideal route tracker of the acceptance step drives a TUNING route through the PFB with
the certified layer beneath (`emulate_layout`). The step shown is the one where the filter changed
the nominal command most (weighted by the QP's own metric) while moving faster than 0.1 m/s; it is
re-solved from a copy of the filter's state taken just before that step, so the figure shows exactly
what `pfbnav/safety/pfb.py` decided there.

(a) Body frame: the scan returns, the footprint; the field of the regime the NOMINAL command selects,
    with the returns predicted one horizon ahead under it (those inside it are the rows the nominal
    violates); the field of the regime the FILTERED command uses, with its active rows (least slack).
(b) The plane of translation: previous applied command, nominal and filtered commands, the limiter's
    one-step envelope about the previous command, and the speed limit of the filtered command's regime.
"""
from __future__ import annotations

import math

import matplotlib.pyplot as plt
import numpy as np
import style
from matplotlib.patches import Circle

from pfbnav.config import load_config
from pfbnav.control.ideal_tracker import IdealTracker
from pfbnav.eval.recovery_control import emulate_layout
from pfbnav.planning.astar import plan_from_config
from pfbnav.safety import pfb_field as pf
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.safety.pfb import PFB, REST, PFBConfig
from pfbnav.world.baking import bake, planner_radii
from pfbnav.world.scenarios import Scenario, load_set, world_of

SET, INDEX = "scenarios/tune/tune_narrow.json", 0
SLIP = 0.95             # the emulator's declared slip (as scripts/pfb_acceptance.py)
STEPS = 600

style.apply()
cfg = load_config(style.CONFIG)
table = FootprintTable.from_csv(style.REPO / cfg["robot"]["footprint_table"])
field = pf.PFBField(table.extent, pf.field_params_from_config(cfg))
pfb = PFB(field, PFBConfig.from_config(cfg, field))
s = cfg["scanners"]
rays = max(1, round(float(s["rays_per_degree"]) * 360.0 / int(s["filter_bins"])))

scenario = Scenario(**load_set(style.REPO / SET)["scenarios"][INDEX])
layout = world_of(scenario, cfg)
baked = bake(layout.obstacles, layout.bounds, float(cfg["planner"]["resolution"]),
             planner_radii(cfg, circumscribed=float(table.circumscribed)))
plan = plan_from_config(baked, scenario.start[:2], scenario.goal, cfg)

# The filter carries two pieces of state between steps (`PFB._heading`, `PFB._at_rest`); each step's
# inputs and that state are recorded, so the chosen step can be re-solved exactly on a fresh filter.
weights = np.array([1.0, 1.0, pfb.c.weight_yaw])
best, best_cost = None, -1.0
real = pfb.filter


def spy(u_nom, v_hat, u_prev, ranges, bearings, valid, *, latched=False):
    global best, best_cost
    before = (pfb._heading, pfb._at_rest)
    res = real(u_nom, v_hat, u_prev, ranges, bearings, valid, latched=latched)
    u_nom = np.array(u_nom, float)
    if res.mode == "filtered" and not latched and math.hypot(*np.asarray(u_prev)[:2]) >= 0.1:
        cost = float(((res.u - u_nom) ** 2 * weights).sum())
        if cost > best_cost:
            best_cost = cost
            best = (before, u_nom, np.array(v_hat, float), np.array(u_prev, float),
                    np.array(ranges, float), np.array(bearings, float), np.array(valid, bool))
    return res


pfb.filter = spy
emulate_layout(cfg, [o.polygon for o in layout.obstacles], np.asarray(plan.path_world),
               scenario.start, IdealTracker.from_config(cfg), SLIP, STEPS, pfb=pfb,
               goal=tuple(scenario.goal), goal_tolerance=float(cfg["reward"]["goal_tolerance"]),
               sector_rays=rays)

(heading0, at_rest0), u_nom, v_hat, u_prev, r, b, ok = best
solver = PFB(field, PFBConfig.from_config(cfg, field))
solver._heading, solver._at_rest = heading0, at_rest0
res = solver.filter(u_nom, v_hat, u_prev, r, b, ok)

pts = r[:, None] * np.column_stack([np.cos(b), np.sin(b)])
speed_hat, w_hat, heading_hat = math.hypot(*v_hat[:2]), float(v_hat[2]), solver._heading
tau = solver.c.dt + solver.c.lag


def predicted(u):
    phi = -u[2] * tau
    rot = np.array([[math.cos(phi), -math.sin(phi)], [math.sin(phi), math.cos(phi)]])
    return (pts - tau * u[:2]) @ rot.T


def field_outline(regime, heading, w):
    psi = np.linspace(-math.pi, math.pi, 721)
    if regime == REST:
        rho = field.extent(psi, 0.0, 0.0, w, at_rest=True)
    else:
        rho = field.band_extent(psi, regime, heading, w)
    return rho * np.cos(psi), rho * np.sin(psi)


def assumed(u):
    return solver._assumed(u, speed_hat, heading_hat, w_hat)


nom_regime = pf.band_of(math.hypot(*u_nom[:2]), field.params)
nom_heading, nom_w = assumed(u_nom)
g_nom, _ = solver._rows(u_nom, pts, nom_regime, speed_hat, heading_hat, w_hat)
violating = g_nom < 0.0
g_res, _ = solver._rows(res.u, pts, res.regime, speed_hat, heading_hat, w_hat)
active = g_res <= g_res.min() + 0.002
res_heading, res_w = res.heading, res.yaw_field

fig, (ax, bx) = plt.subplots(1, 2, figsize=(style.TEXT_WIDTH_IN, 2.9),
                             gridspec_kw={"width_ratios": [1.35, 1.0]})
psi = np.linspace(-math.pi, math.pi, 721)
e = np.asarray(table.extent(psi), dtype=float)
ax.fill(e * np.cos(psi), e * np.sin(psi), color=style.LIGHT, lw=0)
ax.plot(e * np.cos(psi), e * np.sin(psi), color=style.BLACK, lw=0.6)
ax.scatter(pts[:, 0], pts[:, 1], s=3, color=style.GREY, lw=0, label="scan returns")
ax.plot(*field_outline(nom_regime, nom_heading, nom_w), color=style.VERMILLION, ls="--",
        label=f"field, nominal (band {nom_regime})")
p_nom = predicted(u_nom)
ax.scatter(p_nom[violating, 0], p_nom[violating, 1], s=10, marker="x", color=style.VERMILLION,
           lw=0.8, label="predicted, inside (nominal)")
name = "rest" if res.regime == REST else f"band {res.regime}"
ax.plot(*field_outline(res.regime, res_heading, res_w), color=style.BLUE,
        label=f"field, filtered ({name})")
p_res = predicted(res.u)
ax.scatter(p_res[active, 0], p_res[active, 1], s=12, marker="o", facecolor="none",
           edgecolor=style.BLUE, lw=0.8, label="active rows (filtered)")
ax.set_aspect("equal")
lim = 0.9
ax.set_xlim(-lim, lim)
ax.set_ylim(-lim, lim)
ax.set_xlabel("$x$ (m)")
ax.set_ylabel("$y$ (m)")
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2)
style.panel_label(ax, "a")

c = solver.c
bx.add_patch(Circle(u_prev[:2], c.a_max * c.dt, fill=False, ec=style.BLACK, lw=0.6, ls=":",
                    label="envelope $a_{\\max}\\Delta t$"))
bx.add_patch(Circle((0, 0), solver.speed_limit(res.regime), fill=False, ec=style.BLUE, lw=0.6,
                    ls="--", label=f"speed limit ({name})"))
bx.plot(*u_prev[:2], marker="s", ms=4, color=style.BLACK, ls="none", label="previous applied")
bx.plot(*u_nom[:2], marker="x", ms=6, color=style.VERMILLION, ls="none", label="nominal")
bx.plot(*res.u[:2], marker="o", ms=4, color=style.BLUE, ls="none", label="filtered")
bx.annotate("", xy=res.u[:2], xytext=u_nom[:2],
            arrowprops={"arrowstyle": "->", "lw": 0.7, "color": style.GREY})
pts_b = np.vstack([u_nom[:2], res.u[:2], u_prev[:2], [0.0, 0.0]])
pad = 0.08
bx.set_xlim(pts_b[:, 0].min() - pad, pts_b[:, 0].max() + pad)
bx.set_ylim(pts_b[:, 1].min() - pad, pts_b[:, 1].max() + pad)
bx.set_aspect("equal")
bx.set_xlabel("$v_x$ (m/s)")
bx.set_ylabel("$v_y$ (m/s)")
bx.set_title(f"$\\omega$: nominal {u_nom[2]:.2f}, filtered {res.u[2]:.2f} rad/s", fontsize=7)
bx.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2)
style.panel_label(bx, "b")
fig.subplots_adjust(wspace=0.3)
print(f"step chosen: regime {res.regime}, nominal {np.round(u_nom, 3)}, filtered {np.round(res.u, 3)}, "
      f"rows violated by the nominal {int(violating.sum())}, active {int(active.sum())}")
style.save(fig, "fig5_pfb_step.pdf")
