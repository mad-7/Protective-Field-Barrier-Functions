"""Compare two logged trajectories for a regression check. Pure functions.

Ruling 1 on the WP0 results report: turning on the collidable flag of the four wheel
spheres must not change the dynamics, so one G1b profile is run before and after the
scene edit and the trajectories must be bit-identical. "Bit-identical" is tested
literally -- the CSV fields compared as strings -- and the numeric differences are
reported alongside, so a failure says how far apart the runs are, not only that they
differ.
"""
from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

import numpy as np


def read_rows(path: str | Path) -> tuple[list[str], list[dict[str, str]]]:
    with Path(path).open(newline="") as f:
        reader = csv.DictReader(f)
        return list(reader.fieldnames or []), list(reader)


def compare_trajectories(a: str | Path, b: str | Path,
                         columns: tuple[str, ...] | None = None) -> dict[str, Any]:
    """Compare two trajectory CSVs field by field.

    Returns `identical` (every compared field equal as text, and the same number of
    rows), `rows` for each, `first_difference` (row index and column of the first
    mismatch, or None), and `max_abs_difference` per numeric column over the rows both
    files have. `columns` defaults to every column the two files share.
    """
    fields_a, rows_a = read_rows(a)
    fields_b, rows_b = read_rows(b)
    shared = [c for c in fields_a if c in fields_b]
    cols = list(columns) if columns is not None else shared
    missing = [c for c in cols if c not in shared]
    if missing:
        raise ValueError(f"columns {missing} are not in both files")

    first = None
    max_diff: dict[str, float] = {}
    for i, (ra, rb) in enumerate(zip(rows_a, rows_b)):
        for c in cols:
            if ra[c] != rb[c] and first is None:
                first = {"row": i, "column": c, "a": ra[c], "b": rb[c]}
            try:
                d = abs(float(ra[c]) - float(rb[c]))
            except ValueError:
                continue
            max_diff[c] = max(max_diff.get(c, 0.0), d)
    same_length = len(rows_a) == len(rows_b)
    if first is None and not same_length:
        first = {"row": min(len(rows_a), len(rows_b)), "column": "<row count>",
                 "a": str(len(rows_a)), "b": str(len(rows_b))}
    return {
        "identical": first is None,
        "rows": (len(rows_a), len(rows_b)),
        "columns_compared": cols,
        "first_difference": first,
        "max_abs_difference": max_diff,
    }


# --- scan packets across a scene edit that removes geometry (ruled 2026-10-13) -------------------
#
# Removing /Wall may change a scan only where the scan saw the wall. So the bins that may differ
# are PREDICTED from the old scene alone -- the bins whose return lies on the old wall -- and the
# regression requires the bins that actually differ to be exactly those: a bin differing that did
# not view the wall, or a bin that viewed the wall and did not change, both fail it.

def distance_to_triangles_2d(points, triangles) -> np.ndarray:
    """Plan-view distance (m) from each point to the union of triangles; 0 inside one.

    `points` (N, 2+) and `triangles` a sequence of (3, 2+) arrays, world frame; z is ignored.
    """
    p = np.atleast_2d(np.asarray(points, dtype=float))[:, :2]
    best = np.full(len(p), np.inf)
    for tri in triangles:
        t = np.asarray(tri, dtype=float)[:, :2]
        a, b, c = t
        # Inside test by the signs of the three edge cross products (either winding).
        def side(u, v):
            return (v[0] - u[0]) * (p[:, 1] - u[1]) - (v[1] - u[1]) * (p[:, 0] - u[0])
        s1, s2, s3 = side(a, b), side(b, c), side(c, a)
        inside = ((s1 >= 0) & (s2 >= 0) & (s3 >= 0)) | ((s1 <= 0) & (s2 <= 0) & (s3 <= 0))
        d = np.where(inside, 0.0, np.inf)
        for u, v in ((a, b), (b, c), (c, a)):
            e = v - u
            denom = float(e @ e)
            s = np.clip(((p - u) @ e) / denom, 0.0, 1.0) if denom > 0 else np.zeros(len(p))
            d = np.minimum(d, np.linalg.norm(p - (u + s[:, None] * e), axis=1))
        best = np.minimum(best, d)
    return best


def bins_on_shape(pose, ranges, bearings, valid, triangles, *, range_max: float, tol: float,
                  sector_half_width: float = 0.0, arc_samples: int = 9) -> np.ndarray:
    """Which bins' returns lie on the shape (within `tol` m, plan view).

    `pose` is the body's world (x, y, yaw); ranges from the body origin along body-frame
    `bearings`. A bin with no return (invalid, or at `range_max`) views nothing. A bin that is
    the minimum over a sector (`sector_half_width` > 0, rad) counts when any point of its arc at
    that range is on the shape, since its range may come from any ray in the sector.
    """
    x, y, yaw = (float(v) for v in pose)
    r = np.asarray(ranges, dtype=float)
    b = np.asarray(bearings, dtype=float)
    returned = np.asarray(valid, dtype=bool) & (r < range_max)
    offsets = (np.linspace(-sector_half_width, sector_half_width, arc_samples)
               if sector_half_width > 0 else np.zeros(1))
    on = np.zeros(r.size, dtype=bool)
    for o in offsets:
        a = yaw + b + o
        pts = np.column_stack([x + r * np.cos(a), y + r * np.sin(a)])
        on |= distance_to_triangles_2d(pts, triangles) <= tol
    return on & returned


def packet_bin_differences(a: dict, c: dict) -> dict[str, np.ndarray]:
    """Per stream, which bins differ between two packets (range, validity or bearing), bitwise."""
    out = {}
    for stream, fields in (("filter", ("filter_ranges", "filter_valid", "filter_bearings")),
                           ("policy", ("policy_ranges", "policy_valid"))):
        diff = np.zeros(np.asarray(a[fields[0]]).size, dtype=bool)
        for f in fields:
            diff |= np.asarray(a[f]) != np.asarray(c[f])
        out[stream] = diff
    return out


def wall_bin_check(packets_a, packets_c, triangles, *, range_max: float, tol: float) -> dict:
    """Whether the bins differing between A (old scene) and C (new) are exactly those that viewed
    the removed shape in A, packet by packet.

    Packets are dicts with `pose` and the ScanPacket arrays. Returns `exact` (every packet's
    differing bins equal its predicted bins, in both streams, and the packet counts match),
    `packets`, `packets_viewing` (packets with any predicted bin), `bins_predicted`,
    `bins_differing`, and the first few `mismatches`.
    """
    report = {"packets": (len(packets_a), len(packets_c)), "packets_viewing": 0,
              "bins_predicted": 0, "bins_differing": 0, "mismatches": []}
    if len(packets_a) != len(packets_c):
        report["exact"] = False
        report["mismatches"].append({"packet": None, "reason": "packet counts differ"})
        return report
    for i, (a, c) in enumerate(zip(packets_a, packets_c)):
        n_policy = np.asarray(a["policy_ranges"]).size
        predicted = {
            "filter": bins_on_shape(a["pose"], a["filter_ranges"], a["filter_bearings"],
                                    a["filter_valid"], triangles, range_max=range_max, tol=tol),
            "policy": bins_on_shape(a["pose"], a["policy_ranges"],
                                    -np.pi + 2 * np.pi * (np.arange(n_policy) + 0.5) / n_policy,
                                    a["policy_valid"], triangles, range_max=range_max, tol=tol,
                                    sector_half_width=np.pi / n_policy),
        }
        differing = packet_bin_differences(a, c)
        viewing = False
        for stream in ("filter", "policy"):
            p, d = predicted[stream], differing[stream]
            viewing |= bool(p.any())
            report["bins_predicted"] += int(p.sum())
            report["bins_differing"] += int(d.sum())
            if not np.array_equal(p, d) and len(report["mismatches"]) < 10:
                report["mismatches"].append({
                    "packet": i, "stream": stream,
                    "differ_not_predicted": np.flatnonzero(d & ~p).tolist(),
                    "predicted_not_differing": np.flatnonzero(p & ~d).tolist()})
        report["packets_viewing"] += int(viewing)
    report["exact"] = not report["mismatches"]
    return report


# --- the saved-file leg: a reload envelope (ruled 2026-10-14, for the floor regression only) -----
#
# A scene reload is not deterministic (~6 um settled start, up to 1.8 mm over 5 m), so a run on a
# freshly loaded file cannot be bit-identical to one on the in-memory edit. The ruling: load the old
# scene three times, take the largest pairwise deviation per file and column as the reload
# envelope, and require the saved new scene's run to deviate from the in-memory run by no more.

# Columns that log wall-clock time, not the trajectory: they differ on every run and say nothing
# about the physics (`*_seconds` in G0's step-loop log, `*_ms` anywhere).
WALL_CLOCK_SUFFIXES = ("_seconds", "_ms")


def trajectory_columns(fields) -> tuple[str, ...]:
    """The columns a bit-identity comparison covers: every one but the wall-clock timings."""
    return tuple(f for f in fields if not f.endswith(WALL_CLOCK_SUFFIXES))


def compare_runs(a: str | Path, b: str | Path) -> dict[str, Any]:
    """Every CSV of run directory `a` against the same-named CSV of `b`, trajectory columns only."""
    a, b = Path(a), Path(b)
    names = sorted(p.name for p in a.glob("*.csv"))
    missing = sorted(set(names) ^ {p.name for p in b.glob("*.csv")})
    files = {}
    for name in names:
        if (b / name).is_file():
            fields, _ = read_rows(a / name)
            files[name] = compare_trajectories(a / name, b / name, trajectory_columns(fields))
    return {"identical": not missing and all(r["identical"] for r in files.values()),
            "missing": missing, "files": files}


def reload_envelope(run_dirs) -> dict[str, dict[str, float]]:
    """Per CSV and trajectory column, the largest absolute difference between any two runs, and
    under the key '<rows>' the largest difference in row count. Non-numeric columns get 0.0 when
    equal everywhere and inf when they ever differ (they must then match to stay inside)."""
    run_dirs = [Path(d) for d in run_dirs]
    envelope: dict[str, dict[str, float]] = {}
    for i, a in enumerate(run_dirs):
        for b in run_dirs[i + 1:]:
            for name, r in compare_runs(a, b)["files"].items():
                env = envelope.setdefault(name, {"<rows>": 0.0})
                env["<rows>"] = max(env["<rows>"], float(abs(r["rows"][0] - r["rows"][1])))
                for col in r["columns_compared"]:
                    d = r["max_abs_difference"].get(col)
                    if d is None:          # non-numeric: equal or not
                        _, ra = read_rows(a / name)
                        _, rb = read_rows(b / name)
                        same = all(x[col] == y[col] for x, y in zip(ra, rb))
                        d = 0.0 if same else float("inf")
                    env[col] = max(env.get(col, 0.0), float(d))
    return envelope


def within_envelope(c: str | Path, d: str | Path, envelope: dict[str, dict[str, float]]
                    ) -> dict[str, Any]:
    """Whether run `d` deviates from run `c` by no more than `envelope`, file by file and column
    by column. Returns `within` and the first few `excess` entries (file, column, deviation,
    envelope)."""
    excess = []
    comparison = compare_runs(c, d)
    for name in comparison["missing"]:
        excess.append({"file": name, "column": "<file>", "deviation": "missing", "envelope": None})
    for name, r in comparison["files"].items():
        env = envelope.get(name)
        if env is None:
            excess.append({"file": name, "column": "<file>", "deviation": "no envelope",
                           "envelope": None})
            continue
        rows = float(abs(r["rows"][0] - r["rows"][1]))
        if rows > env["<rows>"]:
            excess.append({"file": name, "column": "<rows>", "deviation": rows,
                           "envelope": env["<rows>"]})
        for col in r["columns_compared"]:
            dev = r["max_abs_difference"].get(col)
            if dev is None:
                _, rc = read_rows(Path(c) / name)
                _, rd = read_rows(Path(d) / name)
                dev = 0.0 if all(x[col] == y[col] for x, y in zip(rc, rd)) else float("inf")
            if dev > env.get(col, 0.0):
                excess.append({"file": name, "column": col, "deviation": dev,
                               "envelope": env.get(col, 0.0)})
    return {"within": not excess, "excess": excess[:20], "excess_count": len(excess)}
