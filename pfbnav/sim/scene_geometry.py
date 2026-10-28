"""Read the scene's fixed geometry back: floor tiles, shapes as world triangles, properties.

Simulator I/O for the floor's extent checks (`pfbnav/world/extent.py` holds the pure geometry;
ruled 2026-10-10). Used by `tests/test_floor_sim.py` and `scripts/scene_extend_floor.py`, so
the test and the edit read the scene the same way.
"""
from __future__ import annotations

import hashlib
from typing import Any

import numpy as np

# Tiles are axis-aligned with their tops at z = 0, to single precision.
FLAT_TOL = 1e-6


def floor_tiles(sim, floor: int) -> dict[int, tuple[float, float, float, float, float]]:
    """Each shape directly under `floor`: (cx, cy, half_x, half_y, top_z), world frame, m.

    Raises if a tile is not axis-aligned: the cover check assumes it.
    """
    out = {}
    for h in (int(x) for x in sim.getObjectsInTree(floor, sim.sceneobject_shape, 1 | 2)):
        pos = [float(v) for v in sim.getObjectPosition(h, sim.handle_world)]
        ori = [float(v) for v in sim.getObjectOrientation(h, sim.handle_world)]
        half = [float(v) for v in sim.getVector3Property(h, "bbHSize")]
        if max(abs(a) for a in ori) >= FLAT_TOL:
            raise RuntimeError(f"floor tile {sim.getObjectAlias(h, 2)} is not axis-aligned: {ori}")
        out[h] = (pos[0], pos[1], half[0], half[1], pos[2] + half[2])
    return out


def world_triangles(sim, handle: int) -> list[np.ndarray]:
    """A shape's mesh as triangles in world coordinates, each a (3, 3) array in m."""
    vertices, indices, _normals = sim.getShapeMesh(handle)
    matrix = sim.getObjectMatrix(handle, sim.handle_world)
    world = np.asarray(sim.multiplyVector(matrix, vertices), dtype=float).reshape(-1, 3)
    return [world[list(indices[i:i + 3])] for i in range(0, len(indices), 3)]


def shapes_outside(sim, *roots: int) -> dict[str, int]:
    """Every shape in the scene not under one of `roots`, by alias path."""
    ours = {int(h) for root in roots for h in sim.getObjectsInTree(root, sim.handle_all, 0)}
    return {str(sim.getObjectAlias(int(h), 2)): int(h)
            for h in sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_shape, 0)
            if int(h) not in ours}


def _plain(value: Any) -> Any:
    """A property value as JSON: buffers by their SHA-256, everything else as it came."""
    if isinstance(value, (bytes, bytearray)):
        return {"sha256": hashlib.sha256(bytes(value)).hexdigest(), "bytes": len(value)}
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    return value


def property_dump(sim, handle: int) -> dict[str, Any]:
    """Every readable property of an object (`sim.getProperties`), plus its mesh's hash if a
    shape. Large properties are skipped by the API and the mesh stands in for them."""
    props = {str(k): _plain(v) for k, v in sim.getProperties(handle, {"skipLarge": True}).items()}
    if int(sim.getObjectType(handle)) == int(sim.sceneobject_shape):
        vertices, indices, _normals = sim.getShapeMesh(handle)
        digest = hashlib.sha256(np.asarray(vertices, dtype=np.float64).tobytes()
                                + np.asarray(indices, dtype=np.int64).tobytes()).hexdigest()
        props["mesh_sha256"] = digest
    return props


def property_differences(a: dict[str, Any], b: dict[str, Any],
                         ignore: tuple[str, ...] = ()) -> list[str]:
    """Property names whose values differ (or exist on one side only), skipping any name that
    contains one of the `ignore` substrings (case-insensitive)."""
    def ignored(name: str) -> bool:
        return any(s.lower() in name.lower() for s in ignore)
    return sorted(k for k in set(a) | set(b) if not ignored(k) and a.get(k) != b.get(k))
