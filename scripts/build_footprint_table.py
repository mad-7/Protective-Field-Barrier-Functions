"""Build the 1-degree footprint table from a footprint-probe run (ruling 3, WP0 results).

Reads the collidable envelope polygon from `<run>/footprint_probe.json`, builds the table
with `footprint_table_from_polygon` (a periodic spline that never falls below the polygon),
and writes it with a header naming its source. Offline and fast; no simulator.

Refuses a probe run whose scene differs from the configured scene file (by SHA-256), or
that was made from an uncommitted working tree: the table becomes the footprint every
safety computation uses, so it must be traceable to one committed scene.

Usage:
    python scripts/build_footprint_table.py --config configs/base.yaml \
        --probe-run runs/gates/FOOTPRINT_<stamp>_seed0 \
        --out configs/footprint/youbot_envelope_1deg.csv
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from pfbnav.config import load_config
from pfbnav.sim.footprint import footprint_table_from_polygon
from pfbnav.utils.runio import REPO_ROOT, scene_provenance


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--probe-run", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    run = Path(args.probe_run)
    provenance = json.loads((run / "provenance.json").read_text())
    probe = json.loads((run / "footprint_probe.json").read_text())

    current = scene_provenance(cfg)
    if provenance.get("scene_sha256") != current["scene_sha256"]:
        raise SystemExit(f"the probe ran on scene {provenance.get('scene_sha256')} but the "
                         f"configured scene is {current['scene_sha256']}; re-run the probe")
    if provenance.get("dirty", True):
        raise SystemExit("the probe run was made from an uncommitted working tree")

    polygon = np.asarray(probe["envelope_collidable"]["polygon"], dtype=float)
    table, margin = footprint_table_from_polygon(polygon)
    out = REPO_ROOT / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    table.to_csv(out, {
        "what": "robot footprint extent e(psi), m, body frame, per whole degree",
        "source_run": run.name,
        "source_commit": provenance["commit"],
        "scene": current["scene"],
        "scene_sha256": current["scene_sha256"],
        "engine": probe["physics"]["engine_name"],
        "probe_tolerance_m": repr(probe["tolerance_m"]),
        "spline_margin_added_m": repr(margin),
        "inscribed_m": repr(table.inscribed),
        "circumscribed_m": repr(table.circumscribed),
    })
    print(f"wrote {out}")
    print(f"inscribed {table.inscribed:.4f} m, circumscribed {table.circumscribed:.4f} m, "
          f"spline margin {margin * 1000:.3f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
