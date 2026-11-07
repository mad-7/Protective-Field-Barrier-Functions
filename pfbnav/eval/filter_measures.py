"""How much the PFB corrects a planner's command: descriptive measures for every FILTERED arm (B7, L3,
L4, L5; ruled 2026-11-07, item 5). Pure.

Per step, from the PFB's own record (`info["pfb"]`: the nominal command it received and the command `u`
it passed on) and the applied command:
  * the translational correction ||(vx, vy)_PFB - (vx, vy)_nom||, m/s, and the yaw correction
    |w_PFB - w_nom|, rad/s;
  * the requested planar speed ||(vx, vy)_nom|| and the executed planar speed ||(vx, vy)_applied||, m/s.
Each is kept per episode as a histogram of `BIN` wide bins (the pooled quantiles are then exact to one
bin), and the summary reports the median, the 95th percentile and the share of steps above each of
`THRESHOLDS`. Descriptive only: no reward term, no gate and no hypothesis uses them, and
`k_intervention` is unchanged (the finding "the policy leans on the filter" is recorded, not acted on).
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

BIN = 0.005                        # m/s (or rad/s for yaw) per histogram bin
THRESHOLDS = (0.05, 0.10, 0.20)    # m/s: shares of steps whose translational correction exceeds these
KEYS = ("correction_mps", "correction_yaw_rps", "requested_mps", "executed_mps")


class ValueHistogram:
    """Counts of non-negative values in `BIN` bins (bin k holds [k BIN, (k + 1) BIN))."""

    def __init__(self, counts: Mapping[int, int] | None = None):
        self.counts: dict[int, int] = {int(k): int(n) for k, n in (counts or {}).items() if n}

    def add(self, value: float) -> None:
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"a measure is finite and non-negative, got {value}")
        k = math.floor(round(value / BIN, 9))
        self.counts[k] = self.counts.get(k, 0) + 1

    def merge(self, other: ValueHistogram) -> ValueHistogram:
        out = ValueHistogram(self.counts)
        for k, n in other.counts.items():
            out.counts[k] = out.counts.get(k, 0) + n
        return out

    @property
    def n(self) -> int:
        return sum(self.counts.values())

    def quantile(self, q: float) -> float | None:
        """The q-quantile as the UPPER edge of the bin holding the ceil(q n)-th smallest value."""
        if not self.n:
            return None
        rank, seen = max(1, math.ceil(q * self.n)), 0
        for k in sorted(self.counts):
            seen += self.counts[k]
            if seen >= rank:
                return (k + 1) * BIN
        raise AssertionError("unreachable")

    def share_above(self, threshold: float) -> float | None:
        """The share of values in bins wholly at or above `threshold` (a threshold on a bin edge)."""
        if not self.n:
            return None
        edge = round(threshold / BIN)
        return sum(n for k, n in self.counts.items() if k >= edge) / self.n

    def to_record(self) -> dict[str, int]:
        return {str(k): n for k, n in sorted(self.counts.items())}


def step_values(pfb: Mapping[str, Any], applied: Sequence[float]) -> dict[str, float]:
    """One step's four values from the PFB's record and the applied command."""
    u, nom = [float(x) for x in pfb["u"]], [float(x) for x in pfb["nominal"]]
    return {"correction_mps": math.hypot(u[0] - nom[0], u[1] - nom[1]),
            "correction_yaw_rps": abs(u[2] - nom[2]),
            "requested_mps": math.hypot(nom[0], nom[1]),
            "executed_mps": math.hypot(float(applied[0]), float(applied[1]))}


def summarise(records: Sequence[Mapping[str, Any]]) -> dict[str, Any] | None:
    """Pooled over the episodes that carry `pfb_measures`; None for an arm without the filter."""
    pooled = {k: ValueHistogram() for k in KEYS}
    found = False
    for r in records:
        m = r.get("pfb_measures")
        if not m:
            continue
        found = True
        for k in KEYS:
            pooled[k] = pooled[k].merge(ValueHistogram({int(b): n for b, n in m[k].items()}))
    if not found:
        return None
    out: dict[str, Any] = {"steps": pooled["correction_mps"].n, "bin": BIN}
    for k in KEYS:
        out[k] = {"median": pooled[k].quantile(0.5), "p95": pooled[k].quantile(0.95)}
    out["correction_share_above_mps"] = {f"{t:g}": pooled["correction_mps"].share_above(t)
                                         for t in THRESHOLDS}
    return out
