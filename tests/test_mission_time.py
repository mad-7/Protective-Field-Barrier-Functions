"""Known answers for the old-against-new mission-time comparison (`pfbnav/eval/mission_time.py`)."""
import csv

import pytest

from pfbnav.eval.mission_time import matched, per_stratum, read_routes

FIELDS = ["set", "seed", "stratum", "lethal_margin", "soft_margin", "status", "path_length_m",
          "samples", "traversable", "oracle_time_s", "constant_time_s", "offset_traversable"]


def _write(path, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in FIELDS})


def _row(set_, seed, stratum, length, t, lethal=0.10, trav=1):
    return {"set": set_, "seed": seed, "stratum": stratum, "lethal_margin": lethal,
            "soft_margin": 0.03, "status": "ok", "path_length_m": length,
            "traversable": trav, "oracle_time_s": t}


def test_per_stratum_and_matched_ratios(tmp_path):
    old, new = tmp_path / "old.csv", tmp_path / "new.csv"
    _write(old, [_row("tune_wide", 1, "wide", 10.0, 25.0), _row("tune_wide", 2, "wide", 8.0, 20.0),
                 _row("tune_wide", 2, "wide", 8.0, 99.0, lethal=0.07),       # other margin: out
                 _row("tune_narrow", 3, "narrow", 10.0, 40.0)])
    _write(new, [_row("tune_wide", 1, "wide", 10.0, 30.0), _row("tune_wide", 2, "wide", 8.0, 30.0),
                 _row("tune_narrow", 3, "narrow", 12.0, "", trav=0)])
    o, n = read_routes(old, 0.10, 0.03), read_routes(new, 0.10, 0.03)
    assert len(o) == 3 and len(n) == 3
    ps = per_stratum(n)
    assert ps["wide"]["median_time_s"] == pytest.approx(30.0)
    assert ps["wide"]["median_s_per_m"] == pytest.approx((3.0 + 3.75) / 2)
    assert ps["narrow"] == {"routes": 1, "traversable": 0, "median_time_s": None,
                            "median_s_per_m": None}
    m = matched(o, n)
    assert m["wide"]["pairs"] == 2
    assert m["wide"]["median_ratio"] == pytest.approx((1.2 + 1.5) / 2)
    assert m["wide"]["min_ratio"] == pytest.approx(1.2) and m["wide"]["max_ratio"] == pytest.approx(1.5)
    assert "narrow" not in m                                                   # not timed in new
