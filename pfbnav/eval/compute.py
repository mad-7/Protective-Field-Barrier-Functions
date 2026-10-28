"""Per-step compute, reported for every controller (ruled 2026-10-27).

The real-time criterion is p95 compute per step <= 50 ms, the control period, on the TRAINING host
(`preregistration/gates.yaml`, `realtime_compute`, A28). Every evaluation records, per episode, a
histogram of the controller's compute per step (the wall-clock time of `controller.act`), of the
PFB's where it runs (`info["pfb_seconds"]`, the step loop's own timing of its slot), and of their sum
-- the method's whole decision, B7's RPP plus its filter. A pooled quantile over a run needs every
step, not the episodes' own quantiles, so the record keeps the histogram itself: `BIN_S` wide bins,
sparse (only the bins used), merged across episodes by adding counts. A quantile is read as the
UPPER edge of the bin holding it, so a reported p95 never under-states the measured one by more than
nothing and over-states it by at most one bin (0.1 ms).

No figure from a laptop is a property of the training host; the summary records the host.
"""
from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from typing import Any

BIN_S = 1e-4                     # s, histogram bin width (0.1 ms)
PARTS = ("controller", "pfb", "decision")


class ComputeHistogram:
    """Counts of per-step compute times in `BIN_S` bins (bin k holds [k BIN_S, (k + 1) BIN_S))."""

    def __init__(self, counts: Mapping[int, int] | None = None):
        self.counts: dict[int, int] = {}
        for k, n in (counts or {}).items():
            self._add_count(int(k), int(n))

    def _add_count(self, k: int, n: int) -> None:
        if k < 0 or n < 0:
            raise ValueError("bins and counts are non-negative")
        if n:
            self.counts[k] = self.counts.get(k, 0) + n

    def add(self, seconds: float) -> None:
        if not math.isfinite(seconds) or seconds < 0.0:
            raise ValueError(f"a compute time is finite and non-negative, got {seconds}")
        self._add_count(math.floor(round(seconds / BIN_S, 6)), 1)   # an edge is its upper bin

    def merge(self, other: ComputeHistogram) -> ComputeHistogram:
        out = ComputeHistogram(self.counts)
        for k, n in other.counts.items():
            out._add_count(k, n)
        return out

    @property
    def steps(self) -> int:
        return sum(self.counts.values())

    def quantile(self, q: float) -> float | None:
        """The q-quantile, s, as the upper edge of the bin holding the ceil(q n)-th smallest time;
        None for an empty histogram."""
        if not 0.0 < q <= 1.0:
            raise ValueError("q is in (0, 1]")
        n = self.steps
        if n == 0:
            return None
        rank = max(1, math.ceil(q * n - 1e-9))
        seen = 0
        for k in sorted(self.counts):
            seen += self.counts[k]
            if seen >= rank:
                return (k + 1) * BIN_S
        raise AssertionError("unreachable")

    def to_record(self) -> dict[str, int]:
        return {str(k): n for k, n in sorted(self.counts.items())}

    @classmethod
    def from_record(cls, record: Mapping[str, int]) -> ComputeHistogram:
        return cls({int(k): int(n) for k, n in record.items()})


def summarise(hists: Iterable[ComputeHistogram], limit_s: float | None = None
              ) -> dict[str, Any]:
    """Pooled steps, p50, p95 and max (upper bin edges), ms, and -- with `limit_s` -- whether the p95
    meets it."""
    pooled = ComputeHistogram()
    for h in hists:
        pooled = pooled.merge(h)
    if pooled.steps == 0:
        return {"steps": 0, "p50_ms": None, "p95_ms": None, "max_ms": None}
    p50, p95, top = (pooled.quantile(q) for q in (0.50, 0.95, 1.0))
    assert p50 is not None and p95 is not None and top is not None      # steps > 0
    out: dict[str, Any] = {"steps": pooled.steps, "p50_ms": 1e3 * p50, "p95_ms": 1e3 * p95,
                           "max_ms": 1e3 * top}
    if limit_s is not None:
        out["limit_ms"] = 1e3 * limit_s
        out["meets_limit"] = bool(p95 <= limit_s + 1e-12)
    return out
