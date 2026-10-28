"""Build the tune, pilot and test scenario sets, and lock the test sets.

Run by the RESEARCHER (hard rule 1: `scenarios/test/` is never written by Claude), after the
freeze, and only once the set sizes are preregistered:

    python scripts/build_scenario_sets.py --config configs/base.yaml --seed 0 --dry-run
    python scripts/build_scenario_sets.py --config configs/base.yaml --seed 0

Everything a set depends on is read from the frozen preregistration and never restated here:
the seed range of each split (`layout_sets.seed_ranges`), the unmapped-obstacle rate of each set
(`layout_sets.unmapped.episode_rate`), the sizes and the pilot's construction (`scenario_sets`,
A13), and the route-relative placement rule and its validity check (`unmapped_placement`, A13).
The TEST size is A14, after G2 and the H1 power analysis: until it is preregistered, only the
tune and pilot sets are built.

What it refuses, and why:
  * **No preregistered sizes.** A test set's size fixes the power of H1 and H2; choosing it here
    would decide a design parameter outside the preregistration.
  * **An unfrozen preregistration, or a generator that is not the one the lock pinned.** Test sets
    must be made by the frozen generator, or `verify()` would reject every one of them later.
  * **A dirty tree**, for the same reason as every run.
  * **A set that fails `verify()`** straight after being built.
  * **A config that disagrees with the preregistered placement rule**: the generator reads the
    config, so a drifted config would build sets nobody ruled.
  * **Locking an `unmapped_static` test set that fails the validity check** (A13: at least 90 %
    of its scenarios with an obstacle surface within 0.3 m of the route). A dry run reports it.

`--dry-run` builds everything and writes the sets under `runs/diag/` instead of `scenarios/`,
unlocked, so a plan can be inspected -- sizes, yield, and engagement -- before anything is locked.
A dry run tolerates a scenario module that differs from the lock (recorded in the summary): a
revision is measured BEFORE its amendment is written, and nothing a dry run builds is ever used.

Two options exist for dry runs only (2026-10-02, measuring the bypass rule's cost before A13's
text records it):
  * `--bypass-width W` builds with the corridor half-width W instead of the config's;
  * `--probe` adds 150 `unmapped_static` scenarios for medium and for wide from the pilot seed
    range, after `pilot_narrow`'s block, so the cost per stratum rests on more than the 5-7
    tune scenarios that carry obstacles.
Every set reports the bypass control (expected 0), whether it was built in full -- the ruled
fallback to W = 1.5 m applies only if a tune or pilot set is not -- and where its obstacles lie:
inside an aisle, at an aisle's mouth, or on open floor.

**Engagement is reported for every set** (2026-09-29): the share of scenarios whose nearest
unmapped obstacle sits within `ENGAGED_OFFSET_M` of the route. On pilot seeds it was only 8-15 %
for `unmapped_static`, so a set's name says less than its engagement does.
"""
from __future__ import annotations

import argparse
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from pfbnav.config import load_config
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.layouts import generate, generator_sha256
from pfbnav.world.scenarios import (
    BYPASS_CONTROL_FAILED,
    NO_PLACEMENT,
    aisle_rectangles,
    build_set,
    scenarios_sha256,
    verify,
    write_set,
)
from pfbnav.world.set_plan import (
    SetRequest,
    allocate_blocks,
    engagement,
    location_of,
    location_summary,
    placement_mismatches,
    position_by_index,
    rejection_summary,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
PREREG = REPO_ROOT / "preregistration"
STRATA = ("narrow", "medium", "wide")


def preregistration() -> dict[str, Any]:
    """Every YAML file in `preregistration/`, merged; a key defined twice is an error."""
    merged: dict[str, Any] = {}
    for path in sorted(PREREG.glob("*.yaml")):
        for key, value in (yaml.safe_load(path.read_text(encoding="utf-8")) or {}).items():
            if key in merged:
                raise SystemExit(f"{key!r} is defined in two preregistration files")
            merged[key] = value
    return merged


PROBE_SIZE = 150      # dry runs only: as many as the pilot, so strata compare like for like


def plan(prereg: dict[str, Any], probe: bool = False) -> list[SetRequest]:
    """The sets to build, from the preregistration alone. Test sets only once A14 sizes them."""
    sizes = prereg.get("scenario_sets")
    if not sizes:
        raise SystemExit(
            "The set sizes are not preregistered (no `scenario_sets` block). A test set's size "
            "fixes the power of H1 and H2, so it is ruled and preregistered, not chosen here. See "
            "docs/reports/decision_request_2026-09-29_scenario_sets.md.")
    rates = prereg["layout_sets"]["unmapped"]["episode_rate"]
    requests = [SetRequest(f"tune_{s}", "tune", s, float(rates["tune"]),
                           int(sizes["tune"]["per_stratum"])) for s in STRATA]
    pilot = sizes["pilot"]
    requests.append(SetRequest("pilot_narrow", "pilot", "narrow",
                               float(rates["test"][pilot["condition"]]), int(pilot["size"])))
    # The training set (ruled 2026-10-03): equal thirds by stratum from the train seed range, at
    # the preregistered train rate. Unlocked; each training seed visits it in its own order.
    # The disposable tune-unmapped set for the Step-0 measurement (ruled 2026-10-03): tune seeds,
    # at the H1 population's rate (unmapped_static), allocated after the tune sets above.
    tune_unmapped = sizes.get("tune_unmapped")
    if isinstance(tune_unmapped, dict) and "per_stratum" in tune_unmapped:
        requests += [SetRequest(f"tune_unmapped_{s}", "tune", s,
                                float(rates["test"]["unmapped_static"]),
                                int(tune_unmapped["per_stratum"])) for s in STRATA]
    train = sizes.get("train")
    if isinstance(train, dict) and "per_stratum" in train:
        requests += [SetRequest(f"train_{s}", "train", s, float(rates["train"]),
                                int(train["per_stratum"])) for s in STRATA]
    test = sizes.get("test")
    if isinstance(test, dict) and "per_cell" in test:
        for condition, rate in rates["test"].items():
            requests += [SetRequest(f"test_{condition}_{s}", "test", s, float(rate),
                                    int(test["per_cell"]), lock=True) for s in STRATA]
    if probe:
        requests += [SetRequest(f"probe_{s}", "pilot", s, float(rates["test"][pilot["condition"]]),
                                PROBE_SIZE) for s in STRATA if s != "narrow"]
    return requests


def sum_centre(polygon: list[list[float]]) -> tuple[float, float]:
    """The mean of a polygon's vertices; for the squares placed here, its centre."""
    xs = [float(p[0]) for p in polygon]
    ys = [float(p[1]) for p in polygon]
    return sum(xs) / len(xs), sum(ys) / len(ys)


def check_frozen(dry_run: bool = False) -> bool:
    """Refuse an unfrozen preregistration, a moved generator or a dirty tree. A dry run may use a
    scenario module the lock did not pin; returns whether the module matches."""
    lock = PREREG / "LOCK.json"
    if not lock.exists():
        raise SystemExit("The preregistration is not frozen; test sets are made after the freeze.")
    pinned = json.loads(lock.read_text(encoding="utf-8"))
    if pinned["generator_sha256"] != generator_sha256():
        raise SystemExit("The layout generator is not the one the lock pinned.")
    module_matches = pinned["scenarios_sha256"] == scenarios_sha256()
    if not module_matches and not dry_run:
        raise SystemExit("The scenario module is not the one the lock pinned.")
    dirty = subprocess.run(["git", "status", "--porcelain"], cwd=REPO_ROOT,
                           capture_output=True, text=True, check=True).stdout.strip()
    if dirty:
        raise SystemExit("The working tree is not clean; commit first.")
    if not module_matches:
        print("NOTE: the scenario module differs from the lock; allowed for a dry run only.")
    return module_matches


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--dry-run", action="store_true",
                        help="build everything but write under runs/diag/, unlocked")
    parser.add_argument("--sets", nargs="*", default=None, help="build only these sets")
    parser.add_argument("--bypass-width", type=float, default=None,
                        help="dry run only: the bypass corridor half-width W, m")
    parser.add_argument("--probe", action="store_true",
                        help="dry run only: add 150 unmapped_static scenarios for medium and wide")
    parser.add_argument("--workers", type=int, default=1,
                        help="scenario-building processes; the sets are identical for any value")
    args = parser.parse_args()
    if (args.bypass_width is not None or args.probe) and not args.dry_run:
        raise SystemExit("--bypass-width and --probe are for dry runs only.")

    cfg = load_config(args.config)
    seed_everything(args.seed)
    module_matches = check_frozen(args.dry_run)
    prereg = preregistration()
    requests = plan(prereg, probe=args.probe)
    placement = prereg.get("unmapped_placement")
    if not placement:
        raise SystemExit("The route-relative placement rule is not preregistered (A13).")
    drift = placement_mismatches(cfg, placement)
    if drift:
        raise SystemExit("configs/base.yaml disagrees with the preregistered placement: "
                         + "; ".join(drift))
    if "bypass" not in placement and not args.dry_run:
        raise SystemExit("The bypass rule (revised 2026-10-02) is not preregistered yet.")
    bypass_cfg = cfg["world"]["layout"]["unmapped"]["route_relative"]["bypass"]
    if args.bypass_width is not None:
        # Applied AFTER the drift check and BEFORE the run directory, so the resolved config
        # recorded in the run is the one the sets were built with.
        bypass_cfg["corridor_half_width"] = float(args.bypass_width)
    mouth_depth = float(cfg["world"]["layout"]["end_clearance"])
    validity = placement["validity"]
    if not any(r.split == "test" for r in requests):
        print("Test sets are not built: their size is amendment A14, after G2 and the H1 power "
              "analysis.")
    if args.sets:
        unknown = sorted(set(args.sets) - {r.name for r in requests})
        if unknown:
            raise SystemExit(f"unknown sets {unknown}")
    ranges = {k: tuple(v) for k, v in prereg["layout_sets"]["seed_ranges"].items()}
    # Blocks are allocated over the WHOLE plan, before any narrowing, so a set's seeds do not
    # depend on which other sets are built in the same invocation.
    blocks = allocate_blocks(ranges, requests, headroom=1)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"SCENARIO_SETS_{stamp}", args.seed, cfg)
    report: dict[str, Any] = {"dry_run": args.dry_run,
                              "scenario_module_matches_lock": module_matches,
                              "bypass_corridor_half_width_m": float(
                                  bypass_cfg["corridor_half_width"]),
                              "sets": {}}
    for r in requests:
        if args.sets and r.name not in args.sets:
            continue
        lo, hi = blocks[r.name]
        print(f"-- {r.name}: {r.split}, {r.stratum}, rate {r.episode_rate}, seeds {lo}-{hi}")
        manifest = build_set(range(lo, hi + 1), cfg, r.split, episode_rate=r.episode_rate,
                             stratum=r.stratum, workers=args.workers)
        checked = verify(manifest, cfg)
        if not checked["ok"]:
            raise SystemExit(f"{r.name} failed verification right after building: "
                             f"{checked['problems'][:3]}")
        nearest = [min(s["unmapped_offsets_m"]) if s["unmapped_offsets_m"] else None
                   for s in manifest["scenarios"]]
        eng = engagement(nearest, float(validity["engaged_offset_m"]))
        rejections = rejection_summary(manifest["scenarios"], manifest["failures"], NO_PLACEMENT)
        positions = position_by_index(manifest["scenarios"])
        control = {
            "routes_set_aside": sum(int(s.get("bypass_control_failures", 0))
                                    for s in manifest["scenarios"]),
            "scenarios_lost": sum(1 for f in manifest["failures"]
                                  if f.get("status") == BYPASS_CONTROL_FAILED)}
        places = []
        for s in manifest["scenarios"]:
            if not s["unmapped_polygons"]:
                continue
            aisles = aisle_rectangles(generate(int(s["seed"]), cfg, stratum=s["stratum"],
                                               unmapped_count=0, split=s.get("split", "")))
            places += [location_of(sum_centre(p), aisles, mouth_depth)
                       for p in s["unmapped_polygons"]]
        locations = location_summary(places)
        in_full = manifest["built"] == manifest["requested"]
        # A13's validity check, on every unmapped_static set: reported always, and refusing the
        # LOCK of a test set that fails it.
        valid = None
        if r.episode_rate == 1.0:
            share = eng["engaged_share"]
            valid = share is not None and share >= float(validity["min_engaged_share"])
            if r.lock and not args.dry_run and not valid:
                raise SystemExit(
                    f"{r.name}: only {share} of scenarios have an obstacle within "
                    f"{validity['engaged_offset_m']} m of the route, below the preregistered "
                    f"{validity['min_engaged_share']}; not locked.")
        target = (run_dir if args.dry_run else REPO_ROOT / "scenarios" / r.split) / f"{r.name}.json"
        write_set(target, manifest, lock=r.lock and not args.dry_run)
        report["sets"][r.name] = {
            "split": r.split, "stratum": r.stratum, "episode_rate": r.episode_rate,
            "seeds": [lo, hi], "requested": manifest["requested"], "built": manifest["built"],
            "failures": len(manifest["failures"]),
            "episode_rate_achieved": manifest["episode_rate_achieved"],
            "engagement": eng, "validity_check": valid, "placement": rejections,
            "route_validity": manifest["validity_rejections_by_stratum"],
            "position_by_index": positions, "bypass_control": control,
            "built_in_full": in_full, "locations": locations, "locked": r.lock and not args.dry_run,
            "path": target.relative_to(REPO_ROOT).as_posix()}
        share = eng["engaged_share"]
        rate = rejections["rejection_rate"]
        print(f"   built {manifest['built']}/{manifest['requested']}, "
              f"rate achieved {manifest['episode_rate_achieved']}, engaged "
              f"{'n/a' if share is None else f'{share:.0%}'}"
              f"{'' if valid is None else (' (valid)' if valid else ' (FAILS the 90 % check)')}, "
              f"placement rejection {'n/a' if rate is None else f'{rate:.0%}'} "
              f"{rejections['by_cause']}, lost to placement "
              f"{rejections['scenarios_lost_to_placement']} -> {target.name}")
        for stratum, counts in manifest["validity_rejections_by_stratum"].items():
            print(f"   route validity [{stratum}]: start-invalid {counts['start_invalid']}, "
                  f"goal-invalid {counts['goal_invalid']}, residual {counts['residual']}, "
                  f"goal re-draws {counts['goal_resamples']}, failed seeds "
                  f"{counts['failed_seeds']}")
        shares = locations["shares"]
        if locations["obstacles"]:
            full = "yes" if in_full else "NO"
            print(f"   obstacles {locations['obstacles']}: aisle {shares['aisle']:.0%}, mouth "
                  f"{shares['aisle_mouth']:.0%}, open floor {shares['open_floor']:.0%}; bypass "
                  f"control set aside {control['routes_set_aside']} routes, lost "
                  f"{control['scenarios_lost']}; built in full: {full}")
        if positions["capped"]["n"]:
            capped, others = (positions[g]["mean_arc_fraction"] for g in ("capped", "others"))
            print(f"   mean position along the route: capped {capped:.2f} "
                  f"(n {positions['capped']['n']}), others "
                  f"{'n/a' if others is None else f'{others:.2f}'} "
                  f"(n {positions['others']['n']})")
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float),
                                          encoding="utf-8")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
