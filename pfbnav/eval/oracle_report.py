"""The oracle criterion's results: per soft margin and stratum, and the declared decisions. Pure.

Ruled 2026-10-15 (supervisor). With the lethal margin at 0.07, each built route is re-planned at
each candidate soft margin and followed by the ORACLE tracker (`pfbnav/world/route_check.py`):
fastest clear band at every point, under the deceleration limit. A route is traversable without a
stop iff the band-0 field is clear everywhere along it; the criterion is zero routes with a band-0
breach.

How each route is classed:
  * **start-invalid** -- band-0 field breached at the start, along the route's initial direction;
  * **goal-invalid** -- breached on arrival, along the final approach direction. Both are caught
    by the generator's validity rules, which re-sample the goal (never the layout) and log the
    rejection per stratum;
  * **residual** -- valid at start and goal, band-0 breached somewhere between.

**Declared before the run (the residual rule):** residual <= 2 % of routes in EVERY stratum ->
those routes are rejected and re-sampled, and the rate is reported; any stratum above 2 % ->
stop and report. **The soft margin** is the one with the fewest routes breaching band 0 at a bend
(ties to the smaller margin). The 50 mm inside-bend figure is information only.
"""
from __future__ import annotations

from collections.abc import Sequence
from typing import Any

RESIDUAL_LIMIT = 0.02          # share of a stratum's routes (ruled 2026-10-15)

INT_FIELDS = ("samples", "band0_breached_samples", "unavoidable_stops", "start_valid",
              "goal_valid", "band0_breach_start", "band0_breach_goal", "band0_breach_bend",
              "band0_breach_straight", "offset_band0_breached_samples")
FLOAT_FIELDS = ("lethal_margin", "soft_margin", "path_length_m", "band0_min_clearance_m",
                "oracle_time_s", "constant_time_s")


def parse_row(raw: dict[str, str]) -> dict[str, Any]:
    row: dict[str, Any] = dict(raw)
    for k in INT_FIELDS:
        if k in row:
            row[k] = int(row[k]) if row[k] not in ("", None) else None
    for k in FLOAT_FIELDS:
        if k in row:
            row[k] = float(row[k]) if row[k] not in ("", None) else None
    return row


def classify(row: dict[str, Any]) -> str:
    """'unplannable', 'start_invalid', 'goal_invalid', 'residual' or 'clear'."""
    if row["status"] != "ok":
        return "unplannable"
    if not row["start_valid"]:
        return "start_invalid"
    if not row["goal_valid"]:
        return "goal_invalid"
    return "residual" if row["band0_breached_samples"] > 0 else "clear"


def _median(values: Sequence[float]) -> float | None:
    v = sorted(values)
    if not v:
        return None
    m = len(v) // 2
    return v[m] if len(v) % 2 else 0.5 * (v[m - 1] + v[m])


def summarise_soft(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """One soft margin's routes, per stratum and over all."""
    def block(rs: Sequence[dict[str, Any]]) -> dict[str, Any]:
        classes = [classify(r) for r in rs]
        ok = [r for r in rs if r["status"] == "ok"]
        clear = [r for r, c in zip(rs, classes) if c == "clear"]
        ratios = [r["oracle_time_s"] / r["constant_time_s"] for r in clear
                  if r.get("oracle_time_s") is not None and r.get("constant_time_s")]
        length = sum(r["path_length_m"] for r in ok)
        n = len(rs)
        residual = classes.count("residual")
        return {
            "routes": n, "plannable": len(ok),
            "start_invalid": classes.count("start_invalid"),
            "goal_invalid": classes.count("goal_invalid"),
            "residual": residual, "residual_share": residual / n if n else None,
            "clear": classes.count("clear"),
            "bend_breach_routes": sum(r["band0_breach_bend"] > 0 for r in ok),
            "unavoidable_stops": sum(r["unavoidable_stops"] for r in ok),
            "unavoidable_stops_per_100m": (100.0 * sum(r["unavoidable_stops"] for r in ok) / length
                                           if length else None),
            "mission_time_ratio_median": _median(ratios),
            "mission_time_ratio_mean": sum(ratios) / len(ratios) if ratios else None,
            "offset_band0_breach_routes": sum(r["offset_band0_breached_samples"] > 0 for r in ok),
        }

    strata = sorted({r["stratum"] for r in rows})
    out = {"all": block(rows), "per_stratum": {s: block([r for r in rows if r["stratum"] == s])
                                               for s in strata}}
    out["plannable_everywhere"] = out["all"]["plannable"] == out["all"]["routes"]
    out["residual_rule"] = ("re-sample" if all(b["residual_share"] <= RESIDUAL_LIMIT
                                               for b in out["per_stratum"].values())
                            else "stop")
    return out


def summarise(rows: Sequence[dict[str, Any]], soft_margins: Sequence[float]
              ) -> dict[str, Any]:
    """Every soft margin, the declared choice among them, and the residual rule's verdict for it.

    The choice is made among soft margins that keep every route plannable (none may lose
    plannability); a margin that loses any route is listed but never chosen.
    """
    table = {float(s): summarise_soft([r for r in rows if float(r["soft_margin"]) == float(s)])
             for s in soft_margins}
    eligible = [s for s, t in table.items() if t["plannable_everywhere"]]
    chosen = (min(eligible, key=lambda s: (table[s]["all"]["bend_breach_routes"], s))
              if eligible else None)
    return {"by_soft": table, "chosen_soft": chosen,
            "residual_rule": None if chosen is None else table[chosen]["residual_rule"]}


# --- the free-orientation oracle (ruled 2026-10-15, second ruling) --------------------------------
#
# Same decisions, same residual rule, on the oracle that may choose its body orientation within the
# yaw-continuity, yaw-rate, yaw-acceleration and wheel-budget limits. A route's columns:
# `traversable`, `unavoidable_stops` and `stops_<location>` (where each stop's stretch begins), the
# validity flags, and the TANGENT-LOCKED oracle's floor (`tangent_stops`) -- the unicycle A1's own
# floor, reported beside it (ruled). The floor is platform-specific -- holonomic, full route
# knowledge -- and every report says so.

FLOOR_NOTE = ("The unavoidable-stop floor is platform-specific: a holonomic base with full "
              "knowledge of its route (the oracle). A1's floor is the tangent-locked oracle's.")
FREE_INT_FIELDS = ("samples", "start_valid", "goal_valid", "traversable", "unavoidable_stops",
                   "stops_start", "stops_goal", "stops_bend", "stops_straight", "tangent_stops",
                   "offset_traversable")
FREE_FLOAT_FIELDS = ("lethal_margin", "soft_margin", "path_length_m", "oracle_time_s",
                     "constant_time_s")


def parse_free_row(raw: dict[str, str]) -> dict[str, Any]:
    row: dict[str, Any] = dict(raw)
    for k in FREE_INT_FIELDS:
        if k in row:
            row[k] = int(row[k]) if row[k] not in ("", None) else None
    for k in FREE_FLOAT_FIELDS:
        if k in row:
            row[k] = float(row[k]) if row[k] not in ("", None) else None
    return row


def classify_free(row: dict[str, Any]) -> str:
    """'unplannable', 'start_invalid', 'goal_invalid', 'residual' or 'clear'."""
    if row["status"] != "ok":
        return "unplannable"
    if not row["start_valid"]:
        return "start_invalid"
    if not row["goal_valid"]:
        return "goal_invalid"
    return "clear" if row["traversable"] else "residual"


def summarise_free_soft(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    def block(rs: Sequence[dict[str, Any]]) -> dict[str, Any]:
        classes = [classify_free(r) for r in rs]
        ok = [r for r in rs if r["status"] == "ok"]
        length = sum(r["path_length_m"] for r in ok)
        n = len(rs)
        residual = classes.count("residual")
        return {
            "routes": n, "plannable": len(ok),
            "start_invalid": classes.count("start_invalid"),
            "goal_invalid": classes.count("goal_invalid"),
            "residual": residual, "residual_share": residual / n if n else None,
            "clear": classes.count("clear"),
            "bend_stop_routes": sum(r["stops_bend"] > 0 for r in ok),
            "unavoidable_stops_per_100m": (100.0 * sum(r["unavoidable_stops"] for r in ok) / length
                                           if length else None),
            "a1_tangent_stops_per_100m": (100.0 * sum(r["tangent_stops"] for r in ok) / length
                                          if length else None),
        }

    strata = sorted({r["stratum"] for r in rows})
    out = {"all": block(rows), "per_stratum": {s: block([r for r in rows if r["stratum"] == s])
                                               for s in strata}}
    out["plannable_everywhere"] = out["all"]["plannable"] == out["all"]["routes"]
    out["residual_rule"] = ("re-sample" if all(b["residual_share"] <= RESIDUAL_LIMIT
                                               for b in out["per_stratum"].values())
                            else "stop")
    return out


def summarise_free(rows: Sequence[dict[str, Any]], soft_margins: Sequence[float]
                   ) -> dict[str, Any]:
    """Every soft margin under the free oracle, the declared choice (fewest routes with a stop at a
    bend, among margins keeping every route plannable; ties to the smaller) and its residual rule."""
    table = {float(s): summarise_free_soft([r for r in rows if float(r["soft_margin"]) == float(s)])
             for s in soft_margins}
    eligible = [s for s, t in table.items() if t["plannable_everywhere"]]
    chosen = (min(eligible, key=lambda s: (table[s]["all"]["bend_stop_routes"], s))
              if eligible else None)
    return {"by_soft": table, "chosen_soft": chosen,
            "residual_rule": None if chosen is None else table[chosen]["residual_rule"],
            "floor_note": FLOOR_NOTE}


def summarise_free_info(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Information at the chosen soft margin, per lethal margin and stratum: plannability, the
    oracle's mission time against constant top speed (traversable routes), and how many routes the
    oracle can no longer traverse when displaced 50 mm toward the inside of bends."""
    out: dict[str, Any] = {}
    for lethal in sorted({float(r["lethal_margin"]) for r in rows}):
        mine = [r for r in rows if float(r["lethal_margin"]) == lethal]
        per = {}
        for s in sorted({r["stratum"] for r in mine}):
            rs = [r for r in mine if r["stratum"] == s]
            ok = [r for r in rs if r["status"] == "ok"]
            ratios = [r["oracle_time_s"] / r["constant_time_s"] for r in ok
                      if r.get("oracle_time_s") is not None and r.get("constant_time_s")]
            per[s] = {"routes": len(rs), "plannable": len(ok),
                      "offset_breach_routes": sum(not r["offset_traversable"] for r in ok),
                      "offset_breach_share": (sum(not r["offset_traversable"] for r in ok) / len(ok)
                                              if ok else None),
                      "mission_time_ratio_median": _median(ratios) if ratios else None}
        out[str(lethal)] = per
    return out
