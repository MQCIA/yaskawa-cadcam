"""
CAD seam detection.

Loads a mesh (STL/OBJ/PLY natively; STEP if a converter such as ``cascadio``
or ``gmsh`` is installed) and looks for candidate fillet-weld seams: internal
edges whose dihedral angle is close to 90 degrees AND concave -- the classic
signature of a T / lap fillet joint.

For each kept edge we also return the two adjacent face normals and their
normalized sum (fillet bisector pointing *out* of the joint). Path planning
uses that bisector for torch work-angle and approach/retract offset.

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


def _normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.zeros(3, dtype=float)
    return v / n


def _tolist3(v: np.ndarray) -> list[float]:
    return [float(v[0]), float(v[1]), float(v[2])]


def detect_seams(
    file_path: str,
    angle_tol_deg: float = 20.0,
    target_angle_deg: float = 90.0,
    min_length: float = 1e-6,
) -> dict:
    """Return candidate weld seams as 3D line segments.

    A face-adjacency edge is kept when its dihedral angle is within
    ``angle_tol_deg`` of ``target_angle_deg`` and the joint is concave.
    Each segment includes averaged adjacent face normals ``n1`` / ``n2`` and
    ``bisector = normalize(n1 + n2)`` (outward from the fillet).
    """
    mesh = _as_single_mesh(trimesh.load(file_path, force="mesh"))

    angles = mesh.face_adjacency_angles  # radians, unsigned
    edges = mesh.face_adjacency_edges  # vertex-index pairs
    convex = mesh.face_adjacency_convex  # True=convex, False=concave
    face_pairs = mesh.face_adjacency  # face-index pairs
    normals = mesh.face_normals

    target = math.radians(target_angle_deg)
    tol = math.radians(angle_tol_deg)

    verts = mesh.vertices
    segments: list[dict] = []

    for a_ang, (vi, vj), is_convex, (f0, f1) in zip(
        angles, edges, convex, face_pairs
    ):
        if abs(a_ang - target) > tol:
            continue
        if is_convex:  # fillet welds sit in concave corners
            continue
        p0, p1 = verts[vi], verts[vj]
        length = float(np.linalg.norm(p1 - p0))
        if length < min_length:
            continue

        n1 = _normalize(np.asarray(normals[f0], dtype=float))
        n2 = _normalize(np.asarray(normals[f1], dtype=float))
        bis = _normalize(n1 + n2)
        # Degenerate (nearly parallel opposite normals): fall back to world-up.
        if float(np.linalg.norm(bis)) < 1e-9:
            bis = np.array([0.0, 0.0, 1.0])

        segments.append(
            {
                "start": _tolist3(p0),
                "end": _tolist3(p1),
                "length": length,
                "dihedral_deg": float(math.degrees(a_ang)),
                "n1": _tolist3(n1),
                "n2": _tolist3(n2),
                "bisector": _tolist3(bis),
                "face_indices": [int(f0), int(f1)],
            }
        )

    total_len = float(sum(s["length"] for s in segments))
    return {
        "seam_count": len(segments),
        "total_length": total_len,
        "units": "model units (as stored in the file)",
        "segments": segments,
        "note": (
            "Heuristic result from tessellated geometry. Confirm visually and "
            "against the B-Rep before generating a weld path. "
            "bisector = normalize(n1+n2) points out of the fillet; torch wire "
            "aims along −bisector with an optional travel (push/pull) tilt."
        ),
    }
