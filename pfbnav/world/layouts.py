"""Generate warehouse layouts: shelf rows at a sampled aisle clearance. Pure; no simulator.

Layouts are **generated, not authored in scenes** (decision, 2026-09-23). This module emits
polygons; `pfbnav/world/baking.py` turns the mapped ones into the planner's grid, and the
environment instantiates all of them -- mapped or not -- into the fixed, hash-pinned scene
at episode reset. One source of truth, here.

**Aisles are built from the D2 strata.** A stratum is a band of *per-side clearance* -- the
gap between the robot's side and the shelf when it is centred -- not an aisle width, so the
generator samples a clearance from the stratum and spaces the rows at
`2 * (half_width + clearance)`. A layout therefore states which stratum it belongs to by
construction, and the safety analysis and the geometry cannot drift apart.

**Identity.** A layout set is identified by `(seed, layout hash, generator hash)`
(supervisor, 2026-09-23). The layout hash is over the geometry as generated; the generator
hash is over this file's source, so a change to the generator cannot silently produce a
different world under the same seed. Both travel in the metadata and are checked by runs.

**Unmapped obstacles** are flagged `mapped=False` and are otherwise ordinary obstacles: in
the scene, absent from the map. Their rate is preregistered and shared across methods; this
module places them when asked, and never decides how often.

Frames: world metres, axis-aligned unless a row is given a yaw.
"""
from __future__ import annotations

import hashlib
import itertools
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.utils.hashing import text_sha256
from pfbnav.world.baking import Obstacle

_SOURCE = Path(__file__)


@dataclass(frozen=True)
class Layout:
    """One generated world: its obstacles, its bounds, and what identifies it."""

    obstacles: list[Obstacle]
    bounds: tuple[float, float, float, float]
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def mapped(self) -> list[Obstacle]:
        return [o for o in self.obstacles if o.mapped]

    @property
    def unmapped(self) -> list[Obstacle]:
        return [o for o in self.obstacles if not o.mapped]

    def free_space(self) -> tuple[float, float, float, float]:
        """The hall inside its walls, (x_min, y_min, x_max, y_max) in m."""
        t = float(self.metadata["wall_thickness"])
        x0, y0, x1, y1 = self.bounds
        return (x0 + t, y0 + t, x1 - t, y1 - t)


def generator_sha256() -> str:
    """Hash of this generator's source: part of a layout set's identity.

    Line endings are normalised before hashing (`pfbnav.utils.hashing`): the raw bytes differ
    between a CRLF checkout here and an LF one on the training host, which would make a set
    locked on one machine fail verification on the other (found 2026-09-28).
    """
    return text_sha256(_SOURCE)


def layout_sha256(obstacles: list[Obstacle], bounds) -> str:
    """Hash of the geometry itself, independent of how it was produced.

    Rounded to a micrometre before hashing so a layout that is geometrically the same
    hashes the same, whatever floating-point path produced it.
    """
    h = hashlib.sha256()
    h.update(np.round(np.asarray(bounds, dtype=float), 6).tobytes())
    for o in sorted(obstacles, key=lambda o: (o.name, o.mapped)):
        h.update(o.name.encode("utf-8"))
        h.update(b"1" if o.mapped else b"0")
        h.update(np.round(np.asarray(o.polygon, dtype=float), 6).tobytes())
    return h.hexdigest()


def rectangle(x0: float, y0: float, x1: float, y1: float) -> np.ndarray:
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=float)


def aisle_width(half_width: float, clearance: float) -> float:
    """D2: an aisle is `2 * (half_width + per-side clearance)` wide, m."""
    return 2.0 * (float(half_width) + float(clearance))


def stratum_of(clearance: float, strata) -> str:
    """Which stratum a per-side clearance falls in; the widest stratum owns its top edge."""
    for i, s in enumerate(strata):
        last = i == len(strata) - 1
        if s.min_clearance <= clearance < s.max_clearance or (last
                                                              and clearance == s.max_clearance):
            return s.name
    raise ValueError(f"clearance {clearance} m is outside every stratum")


def perimeter(bounds, thickness: float) -> list[Obstacle]:
    """The four walls of the hall, as obstacles the planner knows about."""
    x0, y0, x1, y1 = (float(v) for v in bounds)
    t = float(thickness)
    return [
        Obstacle(rectangle(x0, y0, x1, y0 + t), name="wall_south"),
        Obstacle(rectangle(x0, y1 - t, x1, y1), name="wall_north"),
        Obstacle(rectangle(x0, y0, x0 + t, y1), name="wall_west"),
        Obstacle(rectangle(x1 - t, y0, x1, y1), name="wall_east"),
    ]


def generate(seed: int, cfg: dict[str, Any], *, stratum: str | None = None,
             unmapped_count: int = 0, split: str = "") -> Layout:
    """One layout from one seed. Same seed and same generator, same world.

    `stratum` picks the aisle band by name; None samples one uniformly. `unmapped_count` is
    how many obstacles the planner is not told about -- the caller decides, because the rate
    is preregistered and shared across methods (2026-09-23).
    """
    from pfbnav.world.strata import strata_from_config

    rng = np.random.default_rng(seed)
    layout_cfg = cfg["world"]["layout"]
    half_width = float(cfg["robot"]["half_width"])
    strata = strata_from_config(cfg)
    names = [s.name for s in strata]
    # The draw happens either way, so `generate(seed, cfg)` and the same call with the
    # stratum it would have sampled give the SAME world. Skipping the draw when the stratum
    # is given shifts every later draw, and a set verified by regeneration then fails
    # against itself -- which is how this was found (tests/test_scenarios.py, 2026-09-23).
    sampled = str(rng.choice(names))
    stratum = sampled if stratum is None else str(stratum)
    if stratum not in names:
        raise ValueError(f"unknown stratum {stratum!r}; known: {names}")
    band = next(s for s in strata if s.name == stratum)
    # Sampled strictly inside the band: its top belongs to the next stratum up.
    clearance = float(rng.uniform(band.min_clearance, band.max_clearance))

    b = [float(v) for v in layout_cfg["bounds"]]
    bounds = (b[0], b[1], b[2], b[3])
    thickness = float(layout_cfg["wall_thickness"])
    depth = float(layout_cfg["shelf_depth"])
    frac_lo, frac_hi = (float(v) for v in layout_cfg["shelf_length_fraction"])
    rows_lo, rows_hi = (int(v) for v in layout_cfg["rows"])
    end_clear = float(layout_cfg["end_clearance"])
    width = aisle_width(half_width, clearance)

    x0, y0, x1, y1 = bounds
    inner = (x0 + thickness, y0 + thickness, x1 - thickness, y1 - thickness)
    span_y = inner[3] - inner[1]
    # Rows run along x, separated by aisles of the sampled width, with an aisle above and
    # below the block so no row is pinned against a wall.
    max_rows = int((span_y - width) // (depth + width))
    if max_rows < 1:
        raise ValueError(
            f"the hall is {span_y:.2f} m deep: no shelf row fits with {width:.2f} m aisles")
    rows = int(rng.integers(rows_lo, min(rows_hi, max_rows) + 1))
    block = rows * depth + (rows + 1) * width
    bottom = inner[1] + 0.5 * (span_y - block) + width

    obstacles = perimeter(bounds, thickness)
    shelves: list[Obstacle] = []
    usable = (inner[2] - inner[0]) - 2 * end_clear
    if usable <= 0.0:
        raise ValueError("end_clearance leaves no room for a shelf row")
    for r in range(rows):
        row_y0 = bottom + r * (depth + width)
        length = usable * float(rng.uniform(frac_lo, frac_hi))
        # Rows alternate which end they leave open, so travelling from one end of the hall
        # to the other threads the aisles instead of crossing open floor beside them.
        flush_left = r % 2 == 0
        row_x0 = inner[0] + end_clear if flush_left else inner[2] - end_clear - length
        shelves.append(Obstacle(rectangle(row_x0, row_y0, row_x0 + length, row_y0 + depth),
                                name=f"shelf_{r}"))
    obstacles += shelves

    hidden = _place_unmapped(rng, unmapped_count, obstacles, inner, layout_cfg)
    obstacles += hidden

    metadata = {
        "seed": int(seed), "split": split, "stratum": stratum,
        "clearance_m": clearance, "aisle_width_m": width, "rows": rows,
        "shelf_depth_m": depth, "wall_thickness": thickness,
        "unmapped_count": len(hidden),
        "generator_sha256": generator_sha256(),
    }
    metadata["layout_sha256"] = layout_sha256(obstacles, bounds)
    return Layout(obstacles=obstacles, bounds=bounds, metadata=metadata)


def _place_unmapped(rng, count: int, existing: list[Obstacle], inner,
                    layout_cfg) -> list[Obstacle]:
    """Square obstacles in the open, clear of everything the planner already knows about.

    Rejection sampling with a stated attempt limit: if the layout has no room, the caller is
    told how many were placed rather than being handed a silently smaller world.
    """
    if count <= 0:
        return []
    cfg_u = layout_cfg["unmapped"]
    size_lo, size_hi = (float(v) for v in cfg_u["size"])
    margin = float(cfg_u["margin"])
    placed: list[Obstacle] = []
    attempts = 0
    limit = 200 * count
    while len(placed) < count and attempts < limit:
        attempts += 1
        size = float(rng.uniform(size_lo, size_hi))
        cx = float(rng.uniform(inner[0] + size, inner[2] - size))
        cy = float(rng.uniform(inner[1] + size, inner[3] - size))
        box = rectangle(cx - size / 2, cy - size / 2, cx + size / 2, cy + size / 2)
        keep_out = margin + size / 2
        if _too_close(box, existing + placed, keep_out):
            continue
        placed.append(Obstacle(box, mapped=False, name=f"unmapped_{len(placed)}"))
    return placed


def _too_close(box: np.ndarray, others: list[Obstacle], keep_out: float) -> bool:
    from pfbnav.world.baking import polygon_distance

    centre = box.mean(axis=0)[None, :]
    for other in others:
        if float(polygon_distance(centre, other.polygon)[0]) < keep_out:
            return True
    return False


def split_of_seed(seed: int, ranges: dict[str, tuple[int, int]]) -> str:
    """Which split a seed belongs to. A seed in no range, or in two, is an error."""
    hits = [name for name, (lo, hi) in ranges.items() if int(lo) <= seed <= int(hi)]
    if len(hits) != 1:
        raise ValueError(f"seed {seed} falls in {len(hits)} split ranges: {hits or 'none'}")
    return hits[0]


def ranges_are_disjoint(ranges: dict[str, tuple[int, int]]) -> bool:
    """Whether the split seed ranges overlap anywhere."""
    spans = sorted((int(lo), int(hi)) for lo, hi in ranges.values())
    return all(a[1] < b[0] for a, b in itertools.pairwise(spans))


def distance_to_path(obstacle: Obstacle, path: np.ndarray) -> float:
    """Shortest distance from an obstacle to a planned polyline, m.

    Logged per episode for every unmapped obstacle (supervisor, 2026-09-23): H1 is then
    reportable against how far the map understates the world, not only as a stratum mean.
    """
    from pfbnav.planning.lookahead import project

    poly = np.asarray(obstacle.polygon, dtype=float)
    samples = np.vstack([poly, poly.mean(axis=0)[None, :],
                         0.5 * (poly + np.roll(poly, -1, axis=0))])
    return float(min(project(path, p).distance for p in samples))
