"""Scenarios: a layout, a start and a goal that make its stratum bind, and a set manifest.

Pure; no simulator. A scenario is what one episode runs: a generated layout
(`pfbnav/world/layouts.py`), a start pose, a goal, and the identity that lets a run prove
which world it used.

**Endpoints are chosen so the stratum binds.** A layout with narrow aisles proves nothing if
the route goes round the shelf block instead of through it -- the plan is then a wide-open
traverse and the layout's stratum does not describe the episode that happened (found by
drawing a generated layout, 2026-09-23). So a candidate pair is **planned**, and the plan is
**measured**: how much of it lies between two shelf rows (`aisle_travel`). A scenario is kept
only when that reaches `world.layout.min_aisle_travel`. Clearance alone was not enough -- a
plan that merely clips a shelf corner has a small minimum clearance while never entering an
aisle. A pair that cannot be found is a named failure, never a silently easier scenario.

**The unmapped count is drawn, not decided here.** The rate is preregistered and shared
across methods (supervisor, 2026-09-23); `episode_unmapped_count` applies whatever rate it
is given, and the set records the rate it used and the count it achieved.

**Identity.** A scenario carries the layout's `(seed, layout_sha256, generator_sha256)` plus
this module's own hash and the scenario hash (endpoints included), so a run can verify that
the world and the endpoints are the ones the set claims. `verify` regenerates and compares:
a set whose generator has changed is refused rather than quietly re-created.
"""
from __future__ import annotations

import hashlib
import itertools
import json
import math
from collections import Counter
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.planning.astar import OK, plan_from_config
from pfbnav.utils.hashing import text_sha256
from pfbnav.world.baking import Obstacle, bake, planner_radii, polygon_distance
from pfbnav.world.bypass import BypassRule, Corridor
from pfbnav.world.layouts import Layout, generate, layout_sha256
from pfbnav.world.oracle import ROUTE_OK, VALIDITY_FAILURES, route_validity
from pfbnav.world.placement import PlacementRule, place_on_route

_SOURCE = Path(__file__)

# Outcomes of asking for a scenario. Strings, so they travel into a report.
SCENARIO_OK = "ok"
NO_ENDPOINTS = "no_endpoint_pair_binds_the_stratum"
NO_PLAN = "no_plan_between_any_candidate_pair"
# A13: every endpoint pair's route was tried and none could hold its unmapped obstacles within
# the attempt limit while keeping the goal reachable. The layout is never re-sampled.
NO_PLACEMENT = "no_route_admits_its_unmapped_obstacles"
# Revised 2026-10-02: every endpoint pair's route failed the bypass test's POSITIVE CONTROL --
# its corridor could not be passed even before any unmapped obstacle was placed. The test cannot
# judge such a route; expected never to happen, and counted if it does.
BYPASS_CONTROL_FAILED = "route_corridor_fails_the_bypass_control"
# Ruled 2026-10-15: every candidate route, and every re-sampled goal, failed the validity rules
# (start, goal or residual under the free-orientation oracle). The layout is never re-sampled.
NO_VALID_ROUTE = "no_route_passes_the_validity_rules"
# A re-sampled goal whose plan fails or does not thread an aisle: drawn again, not a validity cause.
_UNUSABLE_GOAL = "unusable_goal"


@dataclass(frozen=True)
class Scenario:
    """One episode's world and task."""

    seed: int
    split: str
    stratum: str
    start: tuple[float, float, float]      # x, y, yaw (world)
    goal: tuple[float, float]              # x, y (world)
    layout_sha256: str
    generator_sha256: str
    scenarios_sha256: str
    scenario_sha256: str
    unmapped_count: int
    clearance_m: float
    path_length_m: float
    path_min_clearance_m: float
    aisle_travel_m: float           # how much of the plan lies between two shelf rows
    unmapped_offsets_m: list[float] = field(default_factory=list)
    # Where along the plan each unmapped obstacle sits, m. Recorded beside the offsets so H1
    # can be read against WHERE the map understates the world as well as by how much.
    unmapped_arcs_m: list[float] = field(default_factory=list)
    # A13 (2026-09-29): unmapped obstacles are placed relative to the ROUTE, after it is planned,
    # so they are no longer a function of the seed alone. Their geometry travels with the
    # scenario, counter-clockwise polygons in world metres, and `layout_sha256` covers it.
    unmapped_polygons: list[list[list[float]]] = field(default_factory=list)
    # How hard placement was: draws made, and draws refused by cause, over every endpoint pair
    # tried for this scenario. The rejection rate per stratum is reported from these.
    placement_attempts: int = 0
    placement_rejections: dict[str, int] = field(default_factory=dict)
    # The bypass rule this scenario was placed under (revised 2026-10-02): the corridor
    # half-width W, and how many routes were set aside because their corridor failed the
    # positive control (expected 0).
    bypass_corridor_m: float = 0.0
    bypass_control_failures: int = 0
    # The validity rules (ruled 2026-10-15): candidate routes rejected by cause (start_invalid,
    # goal_invalid, residual) over every endpoint pair tried, and the goal re-draws they cost.
    validity_rejections: dict[str, int] = field(default_factory=dict)
    goal_resamples: int = 0

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def scenarios_sha256() -> str:
    """Hash of the code that decides WHICH scenarios exist: this module and every `pfbnav` module
    it imports (its import closure). Part of a set's identity, as the generator's is.

    It was this file alone until 2026-10-15. When the validity rules made the scenarios depend on
    the oracle (`oracle.py`, `route_check.py`) and the certified fields, a change there could have
    altered which routes are built with no hash moving -- as `placement.py` and `bypass.py` (A13)
    already could. The closure is computed as the environment freeze computes its own
    (`pfbnav.envs.freeze.import_closure`).

    Line endings are normalised before hashing (`pfbnav.utils.hashing`): the raw bytes differ
    between a CRLF checkout here and an LF one on the training host, which would make a set
    locked on one machine fail verification on the other (found 2026-09-28).
    """
    from pfbnav.envs.freeze import import_closure

    root = _SOURCE.resolve().parents[2]
    h = hashlib.sha256()
    for path in import_closure("pfbnav.world.scenarios", root):
        h.update(f"{path.relative_to(root).as_posix()}\0{text_sha256(path)}\n".encode())
    return h.hexdigest()


def scenario_sha256(layout_hash: str, start, goal) -> str:
    """Hash of the world and the task together: endpoints are part of what a run used."""
    h = hashlib.sha256()
    h.update(layout_hash.encode("utf-8"))
    h.update(np.round(np.asarray(list(start) + list(goal), dtype=float), 6).tobytes())
    return h.hexdigest()


def episode_unmapped_count(rng, episode_rate: float, count_range) -> int:
    """How many unmapped obstacles this episode carries, at the preregistered rate.

    `episode_rate` is the share of episodes carrying at least one; `count_range` is the
    inclusive range drawn when it does. The rate is a shared environment hyperparameter and
    is never chosen here.
    """
    if not 0.0 <= episode_rate <= 1.0:
        raise ValueError(f"the episode rate must be in [0, 1], got {episode_rate}")
    if float(rng.random()) >= episode_rate:
        return 0
    lo, hi = (int(v) for v in count_range)
    return int(rng.integers(lo, hi + 1))


def aisle_centres(layout: Layout) -> list[float]:
    """The y of every aisle centre in a layout, south to north, including the end aisles."""
    shelves = sorted((o for o in layout.mapped if o.name.startswith("shelf_")),
                     key=lambda o: float(o.polygon[:, 1].min()))
    inner = layout.free_space()
    edges = [inner[1]]
    for shelf in shelves:
        edges.append(float(shelf.polygon[:, 1].min()))
        edges.append(float(shelf.polygon[:, 1].max()))
    edges.append(inner[3])
    return [0.5 * (edges[i] + edges[i + 1]) for i in range(0, len(edges) - 1, 2)]


def _densify(path: np.ndarray, step: float = 0.05) -> np.ndarray:
    dense = []
    for a, b in itertools.pairwise(path):
        n = max(2, int(np.linalg.norm(b - a) / step))
        dense.append(a + np.linspace(0.0, 1.0, n)[:, None] * (b - a))
    return np.vstack(dense)


def _clearance_along(path: np.ndarray, layout: Layout) -> float:
    """The tightest distance from a path to any mapped obstacle, m (sampled along it)."""
    points = _densify(path)
    return float(min(polygon_distance(points, o.polygon).min() for o in layout.mapped))


def aisle_rectangles(layout: Layout) -> list[tuple[float, float, float, float]]:
    """Each aisle BETWEEN two shelf rows, as (x_min, y_min, x_max, y_max) in m.

    Only where the two rows overlap in x: that is the stretch a robot must thread rather
    than walk around. End aisles (between a row and a wall) are not included -- travelling
    along one of those is going round the block, which is what this exists to detect.
    """
    shelves = sorted((o for o in layout.mapped if o.name.startswith("shelf_")),
                     key=lambda o: float(o.polygon[:, 1].min()))
    out = []
    for lower, upper in itertools.pairwise(shelves):
        x0 = max(float(lower.polygon[:, 0].min()), float(upper.polygon[:, 0].min()))
        x1 = min(float(lower.polygon[:, 0].max()), float(upper.polygon[:, 0].max()))
        if x1 > x0:
            out.append((x0, float(lower.polygon[:, 1].max()),
                        x1, float(upper.polygon[:, 1].min())))
    return out


def aisle_travel(path: np.ndarray, rectangles) -> float:
    """How much of a path lies inside any of those aisles, m.

    The measure that decides whether a layout's stratum describes the episode: a plan that
    goes round the block spends nothing here, however narrow the aisles were.
    """
    if len(path) < 2 or not rectangles:
        return 0.0
    points = _densify(path)
    mid = 0.5 * (points[:-1] + points[1:])
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    inside = np.zeros(len(mid), dtype=bool)
    for x0, y0, x1, y1 in rectangles:
        inside |= ((mid[:, 0] >= x0) & (mid[:, 0] <= x1)
                   & (mid[:, 1] >= y0) & (mid[:, 1] <= y1))
    return float(seg[inside].sum())


class ValidityCheck:
    """The generator's validity rules as a callable (ruled 2026-10-15): `check(path, start_xy)`
    gives `oracle.ROUTE_OK` or the failure's cause. Builds the band-0 field geometry once and keeps
    the field shapes across routes, so one instance serves a whole set."""

    def __init__(self, cfg: dict[str, Any]):
        from pfbnav.safety.certified import FieldGeometry, band_speed_limit
        from pfbnav.safety.footprint_table import FootprintTable
        from pfbnav.world.route_check import FieldShape
        from pfbnav.world.strata import certified_params_from_config

        repo = Path(__file__).resolve().parents[2]
        params = certified_params_from_config(cfg)
        fields = FieldGeometry(FootprintTable.from_csv(repo / cfg["robot"]["footprint_table"])
                               .extent, params)
        band0 = float(params.band_top_speeds[0])
        v = cfg["world"]["layout"]["validity"]
        self.sample_spacing = float(v["sample_spacing_m"])
        self.yaw_step = math.radians(float(v["yaw_step_deg"]))
        self.heading_round = math.radians(float(v["heading_round_deg"]))
        self.coarse_stride = int(v["coarse_stride"])
        # The rotation sweep of tangent tracking at band 0's speed, the tangent's turn taken over
        # one lookahead (ruled 2026-10-16), rounded up to `yaw_rate_step`. Under A20 (2026-10-18)
        # band 0's speed is the highest whose REACHABLE speed stays in band 0, top - a_max T.
        self.tracking_window = float(cfg["planner"]["lookahead"])
        self.tracking_speed = band_speed_limit(0, params)
        self.rate_step = float(v["yaw_rate_step"])
        cache: dict[tuple[float, float], Any] = {}

        def shape_for(heading: float, yaw_rate: float = 0.0):
            key = (round(heading, 9), round(yaw_rate, 9))
            if key not in cache:
                cache[key] = FieldShape.moving(fields, band0, heading=heading,
                                               yaw_rate=yaw_rate)
            return cache[key]

        self.shape_for = shape_for

    def for_layout(self, mapped_polygons):
        """The check for one layout's mapped obstacles: `(path, start_xy) -> verdict`."""
        def check(path, start_xy) -> str:
            p = np.asarray(path, dtype=float)
            yaw = math.atan2(p[1][1] - start_xy[1], p[1][0] - start_xy[0])
            return route_validity(p, mapped_polygons, yaw, self.shape_for,
                                  sample_spacing=self.sample_spacing, yaw_step=self.yaw_step,
                                  heading_round=self.heading_round,
                                  coarse_stride=self.coarse_stride,
                                  tracking_window=self.tracking_window,
                                  tracking_speed=self.tracking_speed, rate_step=self.rate_step)
        return check


def make_scenario(seed: int, cfg: dict[str, Any], *, split: str = "",
                  stratum: str | None = None, episode_rate: float = 0.0,
                  attempts: int = 40, validity: ValidityCheck | None = None
                  ) -> tuple[str, Scenario | None, Layout | None]:
    """A scenario whose plan threads an aisle of the layout's own width and passes the validity
    rules (ruled 2026-10-15). `validity` lets a set share one `ValidityCheck`.

    Returns `(status, scenario, layout)`; the scenario is None unless the status is `ok`. On a
    failure the layout's metadata carries `validity_rejections`, so a set can log them per stratum.
    """
    rng = np.random.default_rng(seed)
    layout_cfg = cfg["world"]["layout"]
    unmapped = episode_unmapped_count(rng, episode_rate, layout_cfg["unmapped"]["count"])
    # The MAPPED world only. Unmapped obstacles are placed below, relative to the route (A13);
    # the generator's own uniform placement is no longer used.
    layout = generate(seed, cfg, stratum=stratum, unmapped_count=0, split=split)
    clearance = float(layout.metadata["clearance_m"])

    radii = planner_radii(cfg, circumscribed=_circumscribed(cfg))
    baked = bake(layout.obstacles, layout.bounds, float(cfg["planner"]["resolution"]), radii)
    lanes = aisle_centres(layout)
    rectangles = aisle_rectangles(layout)
    required = float(layout_cfg["min_aisle_travel"])
    # Endpoints go INSIDE aisles, along the shelves' own span -- not in the open cross-aisle
    # at the hall's end, from which a plan can simply walk round the block.
    planned = False
    placement_failed = False
    rule = PlacementRule.from_config(cfg)
    bypass = BypassRule.from_config(cfg)
    mapped_world = layout
    hall = mapped_world.free_space()
    mapped_polygons = [o.polygon for o in mapped_world.mapped]
    total_attempts, total_rejections = 0, Counter()
    control_failures = 0
    validity_rejections: Counter = Counter()
    total_goal_resamples, validity_failed = 0, False
    goal_resamples = int(layout_cfg["validity"]["goal_resamples"])
    check = (validity if validity is not None else ValidityCheck(cfg)).for_layout(mapped_polygons)
    for _ in range(attempts):
        if len(lanes) < 2 or not rectangles:
            break
        i, j = rng.choice(len(lanes), size=2, replace=False)
        x_lo = min(r[0] for r in rectangles)
        x_hi = max(r[2] for r in rectangles)
        start_xy = (float(rng.uniform(x_lo, x_lo + 0.25 * (x_hi - x_lo))), lanes[int(i)])
        goal_xy = (float(rng.uniform(x_hi - 0.25 * (x_hi - x_lo), x_hi)), lanes[int(j)])
        result = plan_from_config(baked, start_xy, goal_xy, cfg)
        if result.status != OK:
            continue
        planned = True
        threaded = aisle_travel(result.path_world, rectangles)
        if threaded < required:
            continue                # it went round: the layout's stratum would not bind
        # The validity rules (ruled 2026-10-15): the route must be traversable by the
        # free-orientation oracle -- start valid, goal valid, no residual band-0 breach. On any
        # failure the GOAL is re-drawn (this start kept), never the layout; each failure is
        # logged by cause. A re-drawn goal whose plan fails or misses the aisles is drawn again.
        verdict = check(result.path_world, start_xy)
        redraws = 0
        while verdict != ROUTE_OK:
            if verdict in VALIDITY_FAILURES:
                validity_rejections[verdict] += 1
            if redraws >= goal_resamples:
                break
            redraws += 1
            others = [k for k in range(len(lanes)) if k != int(i)]
            j = others[int(rng.integers(len(others)))]
            goal_xy = (float(rng.uniform(x_hi - 0.25 * (x_hi - x_lo), x_hi)), lanes[int(j)])
            candidate = plan_from_config(baked, start_xy, goal_xy, cfg)
            if (candidate.status != OK
                    or aisle_travel(candidate.path_world, rectangles) < required):
                verdict = _UNUSABLE_GOAL
                continue
            result = candidate
            verdict = check(result.path_world, start_xy)
        total_goal_resamples += redraws
        if verdict != ROUTE_OK:
            validity_failed = True
            continue
        tightest = _clearance_along(result.path_world, layout)
        threaded = aisle_travel(result.path_world, rectangles)
        # A13: place the unmapped obstacles relative to THIS route. Revised 2026-10-02: each is
        # kept only if the robot can get past ALL of them at once without leaving a corridor of
        # half-width W around this route, keeping the narrow floor beyond its footprint
        # (`bypass.py`). The stack never replans, so a detour through another aisle is no help.
        # A refused obstacle is re-sampled, never the layout; if the route cannot hold them,
        # the next endpoint pair is tried.
        if unmapped > 0:
            corridor = Corridor(result.path_world, mapped_polygons, start_xy, goal_xy, bypass)
            if not corridor.passable([]):           # the positive control (hard rule 15)
                control_failures += 1
                continue

            def feasible(extra, corridor=corridor):
                return corridor.passable([o.polygon for o in extra])
        else:
            def feasible(extra):
                return True

        placed = place_on_route(rng, result.path_world, goal_xy, mapped_world.mapped, hall,
                                unmapped, rule, feasible)
        total_attempts += placed.attempts
        total_rejections.update(placed.rejections)
        if not placed.complete:
            placement_failed = True
            continue
        world = Layout(obstacles=mapped_world.mapped + placed.obstacles,
                       bounds=mapped_world.bounds, metadata=dict(mapped_world.metadata))
        world.metadata["unmapped_count"] = len(placed.obstacles)
        world.metadata["layout_sha256"] = layout_sha256(world.obstacles, world.bounds)
        yaw = math.atan2(result.path_world[1][1] - start_xy[1],
                         result.path_world[1][0] - start_xy[0])
        scenario = Scenario(
            seed=int(seed), split=split, stratum=str(layout.metadata["stratum"]),
            start=(start_xy[0], start_xy[1], yaw), goal=goal_xy,
            layout_sha256=str(world.metadata["layout_sha256"]),
            generator_sha256=str(layout.metadata["generator_sha256"]),
            scenarios_sha256=scenarios_sha256(),
            scenario_sha256=scenario_sha256(str(world.metadata["layout_sha256"]),
                                            (start_xy[0], start_xy[1], yaw), goal_xy),
            unmapped_count=len(world.unmapped), clearance_m=clearance,
            path_length_m=result.length_m, path_min_clearance_m=tightest,
            aisle_travel_m=threaded, unmapped_offsets_m=list(placed.offsets_m),
            unmapped_arcs_m=list(placed.arcs_m),
            unmapped_polygons=[np.asarray(o.polygon, dtype=float).tolist()
                               for o in placed.obstacles],
            placement_attempts=total_attempts,
            placement_rejections={k: int(v) for k, v in sorted(total_rejections.items())},
            bypass_corridor_m=bypass.corridor if unmapped > 0 else 0.0,
            bypass_control_failures=control_failures,
            validity_rejections={k: int(validity_rejections.get(k, 0))
                                 for k in VALIDITY_FAILURES},
            goal_resamples=total_goal_resamples)
        return SCENARIO_OK, scenario, world
    layout.metadata["validity_rejections"] = {k: int(validity_rejections.get(k, 0))
                                              for k in VALIDITY_FAILURES}
    if placement_failed:
        return NO_PLACEMENT, None, layout
    if control_failures:
        return BYPASS_CONTROL_FAILED, None, layout
    if validity_failed:
        return NO_VALID_ROUTE, None, layout
    return (NO_ENDPOINTS if planned else NO_PLAN), None, layout


def world_of(scenario: Scenario | dict[str, Any], cfg: dict[str, Any]) -> Layout:
    """The full world a scenario ran in: the regenerated mapped layout plus its recorded unmapped
    obstacles, checked against the scenario's `layout_sha256`. Raises if they disagree.

    Used by the environment at every reset and by `verify`, so a locked set is rebuilt from what
    it records rather than by re-running the placement.
    """
    s = scenario if isinstance(scenario, dict) else scenario.as_dict()
    layout = generate(int(s["seed"]), cfg, stratum=s["stratum"], unmapped_count=0,
                      split=s.get("split", ""))
    hidden = [Obstacle(np.asarray(poly, dtype=float), mapped=False, name=f"unmapped_{i}")
              for i, poly in enumerate(s.get("unmapped_polygons", []))]
    world = Layout(obstacles=layout.mapped + hidden, bounds=layout.bounds,
                   metadata=dict(layout.metadata))
    world.metadata["unmapped_count"] = len(hidden)
    world.metadata["layout_sha256"] = layout_sha256(world.obstacles, world.bounds)
    if world.metadata["layout_sha256"] != s["layout_sha256"]:
        raise ValueError(
            f"scenario {s['seed']}: the world rebuilds to {world.metadata['layout_sha256'][:10]}, "
            f"not the recorded {str(s['layout_sha256'])[:10]}; the generator, the config or the "
            "recorded obstacles have moved")
    return world


def _circumscribed(cfg: dict[str, Any]) -> float:
    from pfbnav.safety.footprint_table import FootprintTable

    repo = Path(__file__).resolve().parents[2]
    return float(FootprintTable.from_csv(repo / cfg["robot"]["footprint_table"]).circumscribed)


# ---------------------------------------------------------------------------------------
# Sets: a manifest, a lock, and verification
# ---------------------------------------------------------------------------------------
_BUILD: dict[str, Any] = {}      # per-process state for parallel set building


def _build_init(cfg: dict[str, Any], split: str, stratum: str | None,
                episode_rate: float) -> None:
    _BUILD.update(cfg=cfg, split=split, stratum=stratum, episode_rate=episode_rate,
                  check=ValidityCheck(cfg))


def _build_one(seed: int) -> tuple[int, str, dict[str, Any] | None, str | None, dict[str, int]]:
    """One seed: (seed, status, scenario dict or None, its stratum, its validity rejections)."""
    status, scenario, layout = make_scenario(
        seed, _BUILD["cfg"], split=_BUILD["split"], stratum=_BUILD["stratum"],
        episode_rate=_BUILD["episode_rate"], validity=_BUILD["check"])
    if scenario is not None:
        return (int(seed), status, scenario.as_dict(), scenario.stratum,
                dict(scenario.validity_rejections))
    meta = layout.metadata if layout is not None else {}
    return (int(seed), status, None, meta.get("stratum"), dict(meta.get("validity_rejections", {})))


def build_set(seeds, cfg: dict[str, Any], split: str, *, episode_rate: float,
              stratum: str | None = None, workers: int = 1) -> dict[str, Any]:
    """Build a scenario set, recording what was asked for and what was achieved -- including,
    per stratum, the candidate routes the validity rules rejected (ruled 2026-10-15).

    `workers` > 1 builds in parallel processes. Every scenario draws from its own seed's random
    stream, so the set is identical to a serial build, in the same order
    (`tests/test_scenarios.py`)."""
    seeds = [int(s) for s in seeds]
    if workers > 1:
        from concurrent.futures import ProcessPoolExecutor

        with ProcessPoolExecutor(max_workers=workers, initializer=_build_init,
                                 initargs=(cfg, split, stratum, episode_rate)) as pool:
            outcomes = list(pool.map(_build_one, seeds, chunksize=2))
    else:
        _build_init(cfg, split, stratum, episode_rate)
        outcomes = [_build_one(s) for s in seeds]
    scenarios, failures = [], []
    by_stratum: dict[str, Counter] = {}
    for seed, status, scenario, s_stratum, rejections in outcomes:
        tally = by_stratum.setdefault(str(s_stratum), Counter(
            {**{v: 0 for v in VALIDITY_FAILURES}, "goal_resamples": 0, "built": 0,
             "failed_seeds": 0}))
        tally.update(rejections)
        if scenario is None:
            failures.append({"seed": seed, "status": status})
            tally["failed_seeds"] += 1
        else:
            scenarios.append(scenario)
            tally["goal_resamples"] += int(scenario["goal_resamples"])
            tally["built"] += 1
    with_unmapped = sum(1 for s in scenarios if s["unmapped_count"] > 0)
    return {
        "split": split, "requested": len(seeds), "built": len(scenarios),
        "failures": failures,
        "validity_rejections_by_stratum": {s: dict(sorted(c.items()))
                                           for s, c in sorted(by_stratum.items())},
        "episode_rate_requested": float(episode_rate),
        # What the set actually carries, which is what a paper reports -- never the nominal.
        "episode_rate_achieved": (with_unmapped / len(scenarios)) if scenarios else None,
        "generator_sha256": scenarios[0]["generator_sha256"] if scenarios else None,
        "scenarios_sha256": scenarios_sha256(),
        "locked": False,
        "scenarios": scenarios,
    }


def write_set(path: str | Path, manifest: dict[str, Any], *, lock: bool = False) -> Path:
    """Write a set. A locked set refuses to be overwritten (test layouts, 2026-09-23)."""
    out = Path(path)
    if out.exists():
        existing = json.loads(out.read_text())
        if existing.get("locked"):
            raise PermissionError(
                f"{out} is locked: regenerating it needs a recorded amendment, not a rerun")
    manifest = dict(manifest, locked=bool(lock))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(manifest, indent=2, default=float))
    return out


def load_set(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text())


def verify(manifest: dict[str, Any], cfg: dict[str, Any]) -> dict[str, Any]:
    """Regenerate each scenario's layout and check the identity the set claims.

    A run calls this before it uses a set. Mismatches are listed rather than raised, so a
    report can say exactly which scenarios moved and why.
    """
    problems = []
    if manifest.get("scenarios_sha256") != scenarios_sha256():
        problems.append({"scope": "set", "field": "scenarios_sha256",
                         "claimed": manifest.get("scenarios_sha256"),
                         "found": scenarios_sha256()})
    for entry in manifest.get("scenarios", []):
        try:
            layout = world_of(entry, cfg)
        except ValueError as exc:
            problems.append({"scope": "scenario", "seed": entry["seed"],
                             "field": "layout_sha256", "claimed": entry.get("layout_sha256"),
                             "found": str(exc)})
            continue
        found_task = scenario_sha256(layout.metadata["layout_sha256"], entry["start"],
                                     entry["goal"])
        for field_name, found in (("generator_sha256", layout.metadata["generator_sha256"]),
                                  ("scenario_sha256", found_task)):
            if entry.get(field_name) != found:
                problems.append({"scope": "scenario", "seed": entry["seed"],
                                 "field": field_name, "claimed": entry.get(field_name),
                                 "found": found})
    return {"ok": not problems, "problems": problems,
            "checked": len(manifest.get("scenarios", []))}
