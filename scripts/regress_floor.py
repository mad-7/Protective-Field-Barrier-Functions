"""Regression: does the scene edit (wall removed, floor enlarged) change what G0, G1b and the
traversability probe measured?

Ruled 2026-10-10 and 2026-10-13: before G0, G1b and traversability carry over to the new scene,
run G1b `translate_while_rotating` at 0.40 m/s, G0 `side_wall_lateral` (both contexts) and one
traversability probe run (every stratum, both offsets) on the old scene and the new.
Trajectories and contact outcomes bit-identical; scan packets differing exactly in the bins that
viewed the old wall.

**Redesigned 2026-10-14 (ruled), because a scene reload is not deterministic.** The first run
(`REGRESS_FLOOR_20261013T232522Z`) loaded the old scene twice in one process, and the two loads
differed from row 0 (~6 um settled start, up to 1.8 mm over 5 m). Determinism holds within one
scene LOAD, so the decisive comparison now happens inside one:

  **Leg 1, one load of the old scene (decisive).**
    A   the three checks
    B   the three checks again, no reload            -> A vs B: the determinism control
        the edit applied IN MEMORY, by the edit script's own code
        (`scene_extend_floor.remove_wall`, `add_tiles`)
    C   the three checks on the edited scene         -> A vs C: the edit
  CSVs bit-identical across A, B and C (trajectory columns: wall-clock timing columns are set
  aside, `pfbnav.eval.regression.trajectory_columns`). Scan packets: A vs B identical; A vs C
  differing exactly in the bins predicted, from A alone, to have viewed the old wall
  (`wall_bin_check`).

  **Leg 2, the saved file (secondary; a tolerance ruled for this purpose only).**
    L1, L2, L3   the old scene loaded three times, the three checks after each
                 -> the reload envelope: per CSV and column, the largest pairwise deviation
    D            the saved new scene (537f1bbb) loaded, the three checks
  D's deviation from C must lie within the envelope (`within_envelope`). The saved file's
  inventory must also equal the in-memory edit's, object for object.

`PASS` needs leg 1 exact and leg 2 within. A vs B differing is `CONTROL FAILED` (A vs C then cannot
be read). Otherwise any failure is `STOP`. How many packets viewed the wall is reported, so a pass
that held only vacuously is visible.

The old scene comes from git (`--old-rev`, any revision before the edit, checked against its
pinned SHA-256); the new one is the configured scene, hash-checked as every run's is:

    python scripts/regress_floor.py --config configs/base.yaml --seed 0 --old-rev 220ceac

Writes `runs/gates/REGRESS_FLOOR_<timestamp>_seed<seed>/`: A/, B/, C/, L1/, L2/, L3/, D/ (every CSV
the checks write; scan_packets.npz in A, B, C), scene_old.ttt, wall.npz, envelope.json,
summary.json.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import subprocess
import sys
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pfbnav.config import load_config
from pfbnav.eval.regression import (
    compare_runs,
    reload_envelope,
    wall_bin_check,
    within_envelope,
)
from pfbnav.safety.certified import FieldGeometry
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.robot import ChassisGeometry, YouBotBase
from pfbnav.sim.scanner import Scanner
from pfbnav.sim.scene_geometry import floor_tiles, shapes_outside, world_triangles
from pfbnav.utils.runio import create_run_dir, stamp_display
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.strata import (
    certified_params_from_config,
    max_traversal_speed,
    strata_from_config,
    usable_band_top,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
PREREG = REPO_ROOT / "preregistration" / "gates.yaml"
# The scene every gate and probe so far ran on (d936360, the camera edit), pinned until the
# floor edit. The old scene read from git must be exactly this.
OLD_SCENE_SHA256 = "20974276a354e7f12e72c52670a09a8f032e7c93ff3b890e752dbeeffdad2b81"
G1B_PROFILE, G1B_SPEED = "translate_while_rotating", 0.40
G0_CASE = "side_wall_lateral"
G0_CONTEXTS = ("gate_script", "env_step_loop")
RELOADS = 3                       # old-scene loads for the envelope (ruled 2026-10-14)
# How near the old wall's plan footprint a return must lie to count as viewing it, m. The scan
# measures surfaces to ~1 mm (2026-09-27); 10 mm is ten times that and far below the 0.1 m
# between the wall and anything else a check builds.
WALL_BIN_TOL_M = 0.01
PACKET_FIELDS = ("filter_ranges", "filter_bearings", "filter_valid", "policy_ranges",
                 "policy_valid")


class PacketRecorder:
    """Every scan packet read in this process, with the body's world pose when it was read."""

    def __init__(self, cfg: dict[str, Any]):
        self.cfg = cfg
        self.packets: list[dict[str, Any]] | None = None

    def install(self) -> None:
        original = Scanner.read
        recorder = self

        def read(self, *, sim_time=None):
            packet = original(self, sim_time=sim_time)
            if recorder.packets is not None:
                recorder.packets.append(recorder.record(self.client, packet))
            return packet

        Scanner.read = read

    def record(self, client: SimClient, packet) -> dict[str, Any]:
        sim = client.sim
        ref = client.object(self.cfg["sim"]["objects"]["base_ref"])
        x, y, _z = (float(v) for v in sim.getObjectPosition(ref, sim.handle_world))
        yaw = float(sim.getObjectOrientation(ref, sim.handle_world)[2])
        yaw += float(self.cfg["robot"]["yaw_offset"])
        pose = (x, y, math.atan2(math.sin(yaw), math.cos(yaw)))
        return {"pose": pose, **{f: np.array(getattr(packet, f)) for f in PACKET_FIELDS}}


def save_packets(path: Path, packets: list[dict[str, Any]]) -> None:
    if not packets:
        np.savez(path, pose=np.zeros((0, 3)))
        return
    np.savez(path, pose=np.array([p["pose"] for p in packets]),
             **{f: np.stack([p[f] for p in packets]) for f in PACKET_FIELDS})


def _script(name: str):
    spec = importlib.util.spec_from_file_location(name, REPO_ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def old_scene_bytes(rev: str, scene: str) -> bytes:
    data = subprocess.run(["git", "show", f"{rev}:{scene}"], cwd=REPO_ROOT, check=True,
                          capture_output=True).stdout
    digest = hashlib.sha256(data).hexdigest()
    if digest != OLD_SCENE_SHA256:
        raise SystemExit(f"{rev}:{scene} has SHA-256 {digest}, not the pre-edit scene "
                         f"{OLD_SCENE_SHA256}; pass a revision from before the floor edit")
    return data


def foreign_triangles(client: SimClient, cfg: dict[str, Any]) -> list[np.ndarray]:
    """World triangles of every shape outside the robot and the floor in the loaded scene: in the
    old scene `/Wall/Cuboid` alone (2026-10-13 dry run), after the edit nothing."""
    sim = client.sim
    floor = client.object(cfg["sim"]["objects"]["floor"])
    robot = client.object(cfg["sim"]["objects"]["robot_tree"])
    return [tri for h in shapes_outside(sim, floor, robot).values()
            for tri in world_triangles(sim, h)]


def run_checks(client: SimClient, cfg: dict[str, Any], out: Path, g0, g1b, probe,
               recorder: PacketRecorder, record_packets: bool) -> dict[str, Any]:
    """The three checks on the scene as it is loaded now -- nothing is loaded here."""
    out.mkdir()
    criteria = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    report: dict[str, Any] = {}
    recorder.packets = [] if record_packets else None

    print(f"  G1b {G1B_PROFILE} at {G1B_SPEED}")
    report["g1b"] = g1b.run_once(client, cfg, G1B_PROFILE, G1B_SPEED,
                                 float(criteria["G1b_closed_loop_tracking"]["distance"]), out, 0)

    # G0's case is placed from the robot as measured now, exactly as the gate does.
    client.start(settle_steps=5)
    boxes = g0.shape_points(client, cfg)
    client.stop()
    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    psi = g0.outermost_bearing(table.extent)
    reach = float(table.extent(psi))
    corner = (reach * math.cos(psi), reach * math.sin(psi))
    cases = g0.expected_shapes(g0.build_cases(ChassisGeometry.from_config(cfg), corner, psi),
                               boxes)
    case = next(c for c in cases if c.name == G0_CASE)
    g0_criteria = criteria["G0_contact_validation"]
    speed = float(g0_criteria["approach_speed"])
    required = (g0_criteria.get("case_required_shapes") or {}).get(G0_CASE)
    report["g0"] = {}
    for context in G0_CONTEXTS:
        print(f"  G0 {G0_CASE} ({context})")
        runner = g0.run_case if context == "gate_script" else g0.run_case_in_env_loop
        report["g0"][context] = runner(client, cfg, case, speed, out, context,
                                       required_prefix=None if required is None else str(required))

    # One traversability probe run: every stratum at both offsets, as the probe runs it.
    offset = float(criteria["G1b_closed_loop_tracking"]["max_cross_track"])
    scanner = Scanner(client, cfg)
    base = YouBotBase.from_config(client, cfg)
    params = certified_params_from_config(cfg)
    fields = FieldGeometry(table.extent, params)
    report["traversability"] = []
    for stratum in strata_from_config(cfg):
        derived = max_traversal_speed(fields, float(cfg["robot"]["half_width"]),
                                      stratum.min_clearance, offset=offset)
        band = usable_band_top(derived, params)
        for off in (0.0, offset):
            print(f"  traverse {stratum.name}, offset {off * 1000:.0f} mm")
            result = probe.traverse(client, cfg, scanner, base, stratum.name,
                                    stratum.min_clearance, band, off, out)
            report["traversability"].append(asdict(result))
    client.stop()
    packets, recorder.packets = recorder.packets, None
    if record_packets:
        save_packets(out / "scan_packets.npz", packets)
        report["packets"] = len(packets)
    return {"report": report, "packets": packets}


def load(client: SimClient, scene: Path) -> None:
    client.stop()
    client.load_scene(str(scene))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--old-rev", required=True,
                        help="a git revision holding the pre-edit scene (e.g. 220ceac)")
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    # Hash-checks the configured (new) scene against sim.scene_sha256, as every run does.
    run_dir = create_run_dir(REPO_ROOT / "runs" / "gates", f"REGRESS_FLOOR_{stamp}", args.seed,
                             cfg)
    new_scene = REPO_ROOT / cfg["sim"]["scene"]
    if hashlib.sha256(new_scene.read_bytes()).hexdigest() == OLD_SCENE_SHA256:
        raise SystemExit("the configured scene is still the pre-edit one; commit the edit first")
    old_scene = run_dir / "scene_old.ttt"
    old_scene.write_bytes(old_scene_bytes(args.old_rev, cfg["sim"]["scene"]))

    g0, g1b, probe = (_script("gate_g0_contact"), _script("gate_g1b_tracking"),
                      _script("probe_traversability"))
    edit, inventory = _script("scene_extend_floor"), _script("diag_scene_inventory")
    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    recorder = PacketRecorder(cfg)
    recorder.install()
    runs: dict[str, Any] = {}
    try:
        # Leg 1: ONE load of the old scene.
        print(f"-- leg 1: one load of {old_scene.name}")
        load(client, old_scene)
        wall = foreign_triangles(client, cfg)
        for label in ("A", "B"):
            print(f"-- {label}")
            runs[label] = run_checks(client, cfg, run_dir / label, g0, g1b, probe, recorder, True)
        sim = client.sim
        floor = client.object(cfg["sim"]["objects"]["floor"])
        robot = client.object(cfg["sim"]["objects"]["robot_tree"])
        centre, _top = edit.check_old_floor(floor_tiles(sim, floor))
        _props, removed = edit.remove_wall(client, cfg, edit.foreign_report(sim, floor, robot))
        copies, _t, _v = edit.add_tiles(sim, floor, centre)
        print(f"-- edit applied in memory: removed {removed}, added {len(copies)} objects")
        in_memory = inventory.inventory(client)
        leftover = foreign_triangles(client, cfg)
        print("-- C")
        runs["C"] = run_checks(client, cfg, run_dir / "C", g0, g1b, probe, recorder, True)

        # Leg 2: the reload envelope, then the saved file.
        for i in range(1, RELOADS + 1):
            print(f"-- leg 2: L{i}, {old_scene.name} loaded again")
            load(client, old_scene)
            runs[f"L{i}"] = run_checks(client, cfg, run_dir / f"L{i}", g0, g1b, probe, recorder,
                                       False)
        print(f"-- leg 2: D, {new_scene.name} loaded")
        load(client, new_scene)
        saved = inventory.inventory(client)
        runs["D"] = run_checks(client, cfg, run_dir / "D", g0, g1b, probe, recorder, False)
    finally:
        client.stop()

    np.savez(run_dir / "wall.npz", triangles=np.array(wall))
    range_max = float(cfg["scanners"]["range_max"])
    control = compare_runs(run_dir / "A", run_dir / "B")
    control["packets"] = wall_bin_check(runs["A"]["packets"], runs["B"]["packets"], [],
                                        range_max=range_max, tol=WALL_BIN_TOL_M)
    edit_cmp = compare_runs(run_dir / "A", run_dir / "C")
    edit_cmp["packets"] = wall_bin_check(runs["A"]["packets"], runs["C"]["packets"], wall,
                                         range_max=range_max, tol=WALL_BIN_TOL_M)
    envelope = reload_envelope([run_dir / f"L{i}" for i in range(1, RELOADS + 1)])
    (run_dir / "envelope.json").write_text(json.dumps(envelope, indent=2), encoding="utf-8")
    saved_leg = within_envelope(run_dir / "C", run_dir / "D", envelope)
    saved_leg["inventory_vs_in_memory"] = inventory.diff(in_memory, saved)

    problems = []
    if not wall:
        problems.append("the old scene has no wall: this is not the scene ruled on")
    if leftover:
        problems.append("the in-memory edit left a shape outside the robot and the floor")
    if not saved_leg["inventory_vs_in_memory"]["identical"]:
        problems.append("the saved scene's inventory differs from the in-memory edit's")
    control_ok = control["identical"] and control["packets"]["exact"]
    leg1_ok = edit_cmp["identical"] and edit_cmp["packets"]["exact"]
    result = ("CONTROL FAILED" if not control_ok
              else "PASS" if leg1_ok and saved_leg["within"] and not problems else "STOP")
    summary = {"result": result, "old_scene_sha256": OLD_SCENE_SHA256,
               "new_scene_sha256": hashlib.sha256(new_scene.read_bytes()).hexdigest(),
               "wall_bin_tol_m": WALL_BIN_TOL_M, "problems": problems,
               "leg1": {"control_A_vs_B": control, "edit_A_vs_C": edit_cmp},
               "leg2": saved_leg,
               "runs": {k: v["report"] for k, v in runs.items()}}
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=str),
                                          encoding="utf-8")
    stamp_display(run_dir, client.display_status())

    for label, report in (("leg 1, determinism control, A vs B", control),
                          ("leg 1, the edit in memory, A vs C", edit_cmp)):
        print(f"\n{label}: CSVs identical={report['identical']}")
        for name in report["missing"]:
            print(f"   {name}: present in one run only")
        for name, r in report["files"].items():
            if not r["identical"]:
                worst = {k: v for k, v in r["max_abs_difference"].items() if v}
                print(f"   {name}: first difference {r['first_difference']}; max |diff| {worst}")
        p = report["packets"]
        print(f"   scan packets {p['packets']}: {p['packets_viewing']} viewed the wall, "
              f"{p['bins_predicted']} bins predicted to differ, {p['bins_differing']} differ; "
              f"exactly as predicted: {p['exact']}")
        for m in p["mismatches"][:5]:
            print(f"   mismatch: {m}")
    if edit_cmp["packets"]["packets_viewing"] == 0:
        print("   NOTE: no packet viewed the wall, so 'exactly those bins differ' held vacuously")
    inv = saved_leg["inventory_vs_in_memory"]
    print(f"\nleg 2, saved file vs in-memory edit: D within the {RELOADS}-load envelope: "
          f"{saved_leg['within']} ({saved_leg['excess_count']} excesses); inventory identical: "
          f"{inv['identical']} (added {len(inv['added'])}, removed {len(inv['removed'])}, moved "
          f"{len(inv['moved'])}, resized {len(inv['resized'])})")
    for e in saved_leg["excess"][:10]:
        print(f"   outside: {e}")
    for problem in problems:
        print(f"   {problem}")
    print(f"\n{result}  ->  {run_dir.relative_to(REPO_ROOT).as_posix()}")
    if result == "PASS":
        print("Ruled 2026-10-14: G0, G1b and the traversability result carry over.")
    else:
        print("Stop and report. Do not carry G0, G1b or the traversability result over.")
    return 0 if result == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
