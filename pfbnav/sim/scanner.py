"""The client side of the scanner: load `lua/scanner_v2.lua`, configure it, read its packet.

The Lua does the binning inside the simulator and publishes one packet per tick
(`pfbnav/sim/scan_packet.py` is the contract); this module installs it, hands it its
configuration, and decodes what it publishes.

**The script is loaded from `lua/` at run time and never saved into the scene.** The scene
stays pinned by its SHA-256 (hard rule 12) while the scanner is versioned as source in
git, so a run's provenance says which scanner produced its scans -- the commit does.
`sim.createScript` puts the text into the running instance; `remove()` takes it out again.

**Which bins count as observed is decided here, from where each sensor actually is**
(`pfbnav/sim/coverage.py`), and handed to the Lua as a mask. The first scanner judged
coverage by angle from the body origin, as if every sensor sat there, and reported a
blind zone beside the robot as free space (2026-09-21; hard rule 5). The mask depends only
on the sensors' poses, their fields of view and the footprint, all fixed relative to the
robot, so it is computed once per installation.

**The robot is rendered, except each sensor's own scanner model, and its returns are
filtered by the footprint.** Each sensor sees the chassis and the other scanners, so it is
occluded exactly as the renderer occludes it; a return that lands inside the footprint
(plus `scanners.self_filter_margin`) is the robot and is dropped. The own model is left
out because it is the sensor's casing: rendering it (d7da848) put every pixel on the casing
at 0.0094 m and blinded all four sensors (SELFOCC_20260922T163041Z).

**Scanner models ride in the robot tree with scripts of their own** (the v1 Hokuyo pods'
binning scripts, the S300 model's). They would run beside this scanner on the same
sensors, so each is read through the live `scriptDisabled` property -- the deprecated
route once reported every script disabled and that reading was withdrawn (hard rule 13) --
disabled if live, read back, and re-enabled by `remove()`. The scene file is never touched.

Everything that talks to the simulator lives here; the packet's format, the coverage
geometry and their checks are pure and live next door.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.client import SimClient, SimError
from pfbnav.sim.coverage import SensorView, bin_masks, unobserved_extent
from pfbnav.sim.scan_packet import ScanPacket, assert_current, decode_scan
from pfbnav.sim.scanning import resolution_for_bins

REPO_ROOT = Path(__file__).resolve().parents[2]
LUA_PATH = REPO_ROOT / "lua" / "scanner_v2.lua"

CONFIG_SIGNAL = "signal.pfbnav.scanner.config"
SCAN_SIGNAL = "signal.pfbnav.scan"
STATUS_SIGNAL = "signal.pfbnav.scanner.status"

# Rays per bin the streams need at the least: a floor the configured density must clear.
OVERSAMPLE = 2.0
# The footprint is sent to the Lua sampled at this spacing, for the self-return filter.
FOOTPRINT_STEP_DEG = 1


def rays_for(fov_deg: float, rays_per_degree: float) -> int:
    """Rays one sensor renders: its field of view times the configured density.

    Rounded to four decimals before the ceiling, so single-precision noise in a field of
    view read back from the scene (120.0000033 deg) cannot add a ray (2026-09-21).
    """
    if fov_deg <= 0.0 or rays_per_degree <= 0.0:
        raise ValueError("field of view and ray density must be positive")
    return max(1, math.ceil(round(fov_deg * rays_per_degree, 4)))


def sensor_view_from_matrix(matrix: list[float], fov_rad: float, yaw_offset: float,
                            range_max: float) -> SensorView:
    """A sensor's pose in the body frame, from its CoppeliaSim matrix relative to base_ref.

    The matrix is row-major 3x4; the sensor looks along its +z, so column 2 is its optical
    axis. The body frame is base_ref rotated by `yaw_offset`, so both the position and the
    axis are rotated by -yaw_offset.
    """
    m = np.asarray(matrix, dtype=float).reshape(3, 4)
    c, s = math.cos(-yaw_offset), math.sin(-yaw_offset)
    px, py = m[0, 3], m[1, 3]
    ax, ay = m[0, 2], m[1, 2]
    return SensorView(x=c * px - s * py, y=s * px + c * py,
                      heading=math.atan2(s * ax + c * ay, c * ax - s * ay),
                      fov=float(fov_rad), range_max=float(range_max))


class Scanner:
    """Installs `lua/scanner_v2.lua` in a running instance and reads its packets."""

    def __init__(self, client: SimClient, cfg: dict[str, Any]):
        sc = cfg["scanners"]
        self.client = client
        self.policy_bins = int(sc["policy_bins"])
        self.filter_bins = int(sc["filter_bins"])
        self.range_min = float(sc["range_min"])
        self.range_max = float(sc["range_max"])
        self.dt = float(cfg["sim"]["dt"])
        self.objects = cfg["sim"]["objects"]
        # The body frame is robot_ref rotated by this (pi/2 on this platform); the Lua
        # reports in the body frame, so it must be told (found 2026-09-21).
        self.yaw_offset = float(cfg["robot"]["yaw_offset"])
        self.pixel_mirror = bool(sc["pixel_mirror"])
        self.depth_is_euclidean = bool(sc["depth_is_euclidean"])
        # Set by measured range accuracy, not by the bins (configs/base.yaml derivation).
        self.rays_per_degree = float(sc["rays_per_degree"])
        self.coverage_depth = float(sc["coverage_depth"])
        self.self_filter_margin = float(sc["self_filter_margin"])
        self.footprint = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
        self.script: int | None = None
        self.status: dict[str, Any] | None = None
        self.sensors: list[dict[str, Any]] = []
        self.masks: dict[str, np.ndarray] = {}
        # Scripts found inside scanner models: {handle: was_disabled_when_found}.
        self.legacy: dict[int, bool] = {}

    # ------------------------------------------------------------------
    # Installation
    # ------------------------------------------------------------------
    def discover(self) -> list[dict[str, Any]]:
        """Every vision sensor in the robot, with its field of view, ray count and pose."""
        sim = self.client.sim
        tree = self.client.object(self.objects["robot_tree"])
        ref = self.client.object(self.objects["base_ref"])
        handles = [int(h) for h in sim.getObjectsInTree(tree, sim.sceneobject_visionsensor, 0)]
        if not handles:
            raise SimError(f"no vision sensors under {self.objects['robot_tree']}")
        found = []
        for h in handles:
            fov = float(sim.getObjectFloatParam(h, sim.visionfloatparam_perspective_angle))
            fov_deg = math.degrees(fov)
            rays = rays_for(fov_deg, self.rays_per_degree)
            floor = resolution_for_bins(fov_deg, self.filter_bins, oversample=OVERSAMPLE)
            if rays < floor:
                raise ValueError(f"sensor {h}: {rays} rays at {self.rays_per_degree} per "
                                 f"degree is below the {floor} the filter bins need")
            view = sensor_view_from_matrix(
                [float(v) for v in sim.getObjectMatrix(h, ref)], fov, self.yaw_offset,
                self.range_max)
            found.append({"handle": h, "fov_deg": fov_deg, "rays": rays, "view": view})
        self.sensors = found
        return found

    def compute_masks(self) -> dict[str, np.ndarray]:
        """Position-aware validity for both streams (pfbnav/sim/coverage.py)."""
        views = [s["view"] for s in self.sensors]
        both = bin_masks(views, [self.policy_bins, self.filter_bins], self.footprint.extent,
                         self.coverage_depth)
        self.masks = {"policy": both[self.policy_bins], "filter": both[self.filter_bins]}
        return self.masks

    def coverage_report(self) -> dict[str, Any]:
        """What the layout leaves unseen, and which bins are therefore invalid."""
        views = [s["view"] for s in self.sensors]
        blind = unobserved_extent(views, self.footprint.extent, self.coverage_depth)
        return {"unobserved_area_m2": blind["area_m2"],
                "unobserved_reach_m": blind["reach_m"],
                "policy_valid": int(self.masks["policy"].sum()),
                "filter_valid": int(self.masks["filter"].sum()),
                "sensors": [{"x": s["view"].x, "y": s["view"].y,
                             "heading_deg": math.degrees(s["view"].heading),
                             "fov_deg": s["fov_deg"], "rays": s["rays"]}
                            for s in self.sensors]}

    def configuration(self) -> dict[str, Any]:
        """What the Lua is told: bins, limits, frames, ray density, masks, footprint."""
        extents = self.footprint.extent(
            np.radians(np.arange(-180, 180, FOOTPRINT_STEP_DEG, dtype=float)))
        return {
            "policy_bins": self.policy_bins, "filter_bins": self.filter_bins,
            "range_min": self.range_min, "range_max": self.range_max,
            "robot_ref": self.objects["base_ref"], "robot_tree": self.objects["robot_tree"],
            # Per-sensor ray count = field of view x density (the error is pixel sampling,
            # proportional to pixel width; configs/base.yaml derivation).
            "rays_per_degree": self.rays_per_degree,
            # How a depth pixel becomes a point (pfbnav/sim/depth_geometry.py).
            "pixel_mirror": self.pixel_mirror,
            "depth_is_euclidean": self.depth_is_euclidean,
            "yaw_offset": self.yaw_offset,
            # Validity from where the sensors are, not from bearings (coverage.py).
            "policy_valid_mask": [1 if v else 0 for v in self.masks["policy"]],
            "filter_valid_mask": [1 if v else 0 for v in self.masks["filter"]],
            # The robot's own returns: inside the footprint + margin, dropped.
            "footprint_extent": [float(e) for e in np.asarray(extents)],
            "footprint_step_deg": FOOTPRINT_STEP_DEG,
            "self_filter_margin": self.self_filter_margin,
        }

    def install(self, *, resolution_x: int | None = None) -> int:
        """Create the script in the instance. The simulation must be stopped.

        The configuration is published before the script exists, because the script reads
        it in `sysCall_init` when the simulation starts. `resolution_x` forces one ray
        count on every sensor, for diagnostics that vary it
        (scripts/diag_scanner_resolution.py); the environment never passes it.
        """
        if self.script is not None:
            raise SimError("the scanner script is already installed")
        sim = self.client.sim
        self.discover()
        self.compute_masks()
        self._silence_scanner_model_scripts()
        config = self.configuration()
        if resolution_x is not None:
            if resolution_x < 1:
                raise ValueError(f"resolution_x must be at least 1, got {resolution_x}")
            config["resolution_x"] = int(resolution_x)
        sim.setBufferProperty(sim.handle_scene, CONFIG_SIGNAL, sim.packTable(config))
        self.script = int(sim.createScript(sim.scripttype_simulation,
                                           LUA_PATH.read_text(encoding="utf-8"), 0, "lua"))
        return self.script

    def remove(self) -> None:
        """Take the script out again, leaving the instance as it was found.

        Scripts this scanner disabled are re-enabled; ones that were already disabled when
        found are left as they were.
        """
        sim = self.client.sim
        if self.script is not None:
            sim.removeObjects([self.script])
            self.script = None
        for handle, was_disabled in self.legacy.items():
            if not was_disabled:
                sim.setBoolProperty(handle, "scriptDisabled", False)
        self.legacy = {}

    def scanner_models(self) -> list[int]:
        """The model each vision sensor belongs to: its ancestor just below the robot root.

        Works for any scanner model mounted on the robot -- the v1 Hokuyo pods (aliased
        `LiDAR`) and the S300s alike -- rather than recognising one by name.
        """
        sim = self.client.sim
        root = self.client.object(self.objects["robot_tree"])
        models: list[int] = []
        for s in self.sensors or self.discover():
            h = s["handle"]
            parent = int(sim.getObjectParent(h))
            while parent not in (root, -1):
                h, parent = parent, int(sim.getObjectParent(parent))
            if parent == -1:
                raise SimError(f"sensor {s['handle']} is not inside {self.objects['robot_tree']}")
            if h not in models:
                models.append(h)
        return models

    def _silence_scanner_model_scripts(self) -> None:
        """Disable every live script inside a scanner model, confirmed by read-back.

        A write that does not take raises: a model script left running would process the
        same sensors a second time, and a vision callback can rewrite a sensor's image, so
        the scan could not be vouched for.
        """
        sim = self.client.sim
        for model in self.scanner_models():
            for h in (int(x) for x in sim.getObjectsInTree(model, sim.sceneobject_script, 0)):
                was_disabled = bool(sim.getBoolProperty(h, "scriptDisabled"))
                self.legacy[h] = was_disabled
                if was_disabled:
                    continue
                sim.setBoolProperty(h, "scriptDisabled", True)
                if not bool(sim.getBoolProperty(h, "scriptDisabled")):
                    raise SimError(f"scanner-model script {h} did not disable (read back live)")

    def legacy_report(self) -> dict[str, Any]:
        """What was found in the scanner models and what was done about it."""
        found = len(self.legacy)
        live = sum(not d for d in self.legacy.values())
        return {"legacy_scripts_found": found, "found_live_and_disabled": live,
                "already_disabled": found - live}

    # ------------------------------------------------------------------
    # Reading
    # ------------------------------------------------------------------
    def read_status(self) -> dict[str, Any]:
        """What the script reports about itself after `sysCall_init`, or raise."""
        sim = self.client.sim
        raw = sim.getBufferProperty(sim.handle_scene, STATUS_SIGNAL, {"noError": True})
        if not raw:
            raise SimError("the scanner script published no status: it did not initialise "
                           "(check the simulator log for its error)")
        self.status = dict(sim.unpackTable(raw))
        return self.status

    def read(self, *, sim_time: float | None = None) -> ScanPacket:
        """Decode this tick's packet, and refuse one from another tick.

        `sim_time` is the time the step belongs to; when given, a packet from a different
        tick raises rather than being used, because a stale scan describes where the robot
        was (`scan_packet.assert_current`).
        """
        sim = self.client.sim
        raw = sim.getBufferProperty(sim.handle_scene, SCAN_SIGNAL, {"noError": True})
        if not raw:
            raise SimError("no scan packet this tick: the scanner script is not running")
        packet = decode_scan(sim.unpackFloatTable(raw), policy_bins=self.policy_bins,
                             filter_bins=self.filter_bins, range_min=self.range_min,
                             range_max=self.range_max)
        if sim_time is not None:
            assert_current(packet, sim_time, self.dt)
        return packet

    def rescan(self, *, sim_time: float | None = None) -> ScanPacket:
        """Re-render the SAME tick and decode it (the scan-fault fix, ruled 2026-10-23). The script
        renders every sensor again without stepping physics and keeps the tick's stamp, so the
        packet passes `assert_current` exactly as the first did."""
        if self.script is None:
            raise SimError("rescan: the scanner script is not installed")
        self.client.sim.callScriptFunction("rescan", self.script)
        return self.read(sim_time=sim_time)
