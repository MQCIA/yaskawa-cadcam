"""
CAD seam detection.

Loads a mesh (STL/OBJ/PLY natively; STEP if a converter such as ``cascadio``
or ``gmsh`` is installed) and looks for candidate fillet-weld seams: internal
edges whose dihedral angle is close to 90 degrees AND concave -- the classic
signature of a T / lap fillet joint.

This is a HEURISTIC on tessellated geometry. Real weld-seam recognition on
B-Rep STEP data (true face adjacency, edge continuity, groove type) needs a
kernel like OpenCASCADE (pythonocc-core). Treat the returned edges as
suggestions for a human to confirm, never as an authoritative weld map.
"""

from __future__ import annotations

import math
import numpy as np
import trimesh


def _as_single_mesh(loaded) -> trimesh.Trimesh:
    """Collapse a Scene (multi-body STEP/GLB) into one Trimesh."""
    if isinstance(loaded, trimesh.Scene):
        if len(loaded.geometry) == 0:
            raise ValueError("No geometry found in file")
        return trimesh.util.concatenate(
            [g for g in loaded.geometry.values() if isinstance(g, trimesh.Trimesh)]
        )
    return loaded


def detect_seams(
    file_path: str,
    angle_tol_deg: float = 20.0,
    target_angle_deg: float = 90.0,
    min_length: float = 1e-6,
) -> dict:
    """Return candidate weld seams as 3D line segments.

    A face-adjacency edge is kept when its dihedral angle is within
    ``angle_tol_deg`` of ``target_angle_deg`` and the joint is concave.
    Consecutive kept edges are merged into polylines.
    """
    mesh = _as_single_mesh(trimesh.load(file_path, force="mesh"))

    angles = mesh.face_adjacency_angles          # radians, unsigned
    edges = mesh.face_adjacency_edges            # vertex-index pairs
    convex = mesh.face_adjacency_convex          # True=convex, False=concave

    target = math.radians(target_angle_deg)
    tol = math.radians(angle_tol_deg)

    verts = mesh.vertices
    segments: list[dict] = []

    for (a_ang, (vi, vj), is_convex) in zip(angles, edges, convex):
        if abs(a_ang - target) > tol:
            continue
        if is_convex:  # fillet welds sit in concave corners
            continue
        p0, p1 = verts[vi], verts[vj]
        length = float(np.linalg.norm(p1 - p0))
        if length < min_length:
            continue
        segments.append(
            {
                "start": [float(p0[0]), float(p0[1]), float(p0[2])],
                "end": [float(p1[0]), float(p1[1]), float(p1[2])],
                "length": length,
                "dihedral_deg": float(math.degrees(a_ang)),
            }
        )

    merged = merge_collinear_seams(segments)
    total_len = float(sum(s["length"] for s in merged))
    return {
        "seam_count": len(merged),
        "raw_edge_count": len(segments),
        "total_length": total_len,
        "units": "model units (as stored in the file)",
        "segments": merged,
        "note": (
            "Heuristic result from tessellated geometry. Collinear fillet edges "
            "are merged into weld seams. Confirm visually and against the B-Rep "
            "before generating a weld path."
        ),
    }


def merge_collinear_seams(
    segments: list[dict],
    turn_deg: float = 25.0,
) -> list[dict]:
    """Chain triangle edges that share a vertex and keep a similar direction.

    A tessellated fillet is many short edges. ArcNC-style extraction wants one
    weld seam per straight run, and stops at a corner or a 3-way junction.
    """
    if len(segments) < 2:
        return segments

    coords = np.array(
        [p for s in segments for p in (s["start"], s["end"])], dtype=float
    )
    scale = float(np.max(np.abs(coords))) or 1.0
    tol = max(scale * 1e-6, 1e-9)

    def q(p) -> tuple[int, int, int]:
        return tuple(int(round(float(c) / tol)) for c in p)

    def unit(v: np.ndarray) -> np.ndarray:
        n = float(np.linalg.norm(v))
        return v / n if n else v

    incident: dict[tuple[int, int, int], list[int]] = {}
    for i, s in enumerate(segments):
        incident.setdefault(q(s["start"]), []).append(i)
        incident.setdefault(q(s["end"]), []).append(i)

    used: set[int] = set()

    def extend(from_key, incoming: np.ndarray):
        cands = [j for j in incident.get(from_key, []) if j not in used]
        # A weld junction (three fillets meeting) is not one seam.
        if len(incident.get(from_key, [])) > 2 and len(cands) != 1:
            return None
        best = None
        best_ang = 1e9
        for j in cands:
            s = segments[j]
            if q(s["start"]) == from_key:
                d = unit(np.asarray(s["end"], dtype=float) - np.asarray(s["start"], dtype=float))
                nxt_key, nxt_pt = q(s["end"]), s["end"]
            else:
                d = unit(np.asarray(s["start"], dtype=float) - np.asarray(s["end"], dtype=float))
                nxt_key, nxt_pt = q(s["start"]), s["start"]
            ang = math.degrees(math.acos(float(np.clip(np.dot(incoming, d), -1.0, 1.0))))
            if ang < best_ang:
                best_ang = ang
                best = (j, nxt_key, nxt_pt, d, ang)
        if best is None or best[4] > turn_deg:
            return None
        return best

    merged: list[dict] = []
    for i, seed in enumerate(segments):
        if i in used:
            continue
        used.add(i)
        chain = [i]
        pts = [list(seed["start"]), list(seed["end"])]

        direction = unit(np.asarray(seed["end"], dtype=float) - np.asarray(seed["start"], dtype=float))
        key_end = q(seed["end"])
        while True:
            nxt = extend(key_end, direction)
            if nxt is None:
                break
            j, key_end, nxt_pt, direction, _ang = nxt
            used.add(j)
            chain.append(j)
            pts.append(list(nxt_pt))

        direction = unit(np.asarray(seed["start"], dtype=float) - np.asarray(seed["end"], dtype=float))
        key_end = q(seed["start"])
        while True:
            nxt = extend(key_end, direction)
            if nxt is None:
                break
            j, key_end, nxt_pt, direction, _ang = nxt
            used.add(j)
            chain.append(j)
            pts.insert(0, list(nxt_pt))

        p0 = np.asarray(pts[0], dtype=float)
        p1 = np.asarray(pts[-1], dtype=float)
        dihedral = float(np.mean([segments[k]["dihedral_deg"] for k in chain]))
        merged.append(
            {
                "start": [float(p0[0]), float(p0[1]), float(p0[2])],
                "end": [float(p1[0]), float(p1[1]), float(p1[2])],
                "length": float(np.linalg.norm(p1 - p0)),
                "dihedral_deg": dihedral,
            }
        )

    merged.sort(key=lambda s: s["length"], reverse=True)
    return merged
