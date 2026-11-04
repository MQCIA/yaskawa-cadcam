"""
CAD seam detection.

Loads a mesh (STL/OBJ/PLY natively; STEP if a converter such as ``cascadio``
or ``gmsh`` is installed) and looks for candidate fillet-weld seams: internal
edges whose dihedral angle is close to 90 degrees AND concave -- the classic
signature of a T / lap fillet joint.

Post-processing:
* ``merge_collinear_seams`` — glue consecutive straight edges into one linear seam
* ``group_constant_radius_arcs`` — chain short edges that share a constant
  curvature radius into a single ``type="arc"`` seam (start / via / end +
  sampled ``points``), matching production pipe welds taught as ``MOVC``
  (e.g. ``12.JBI`` / ``123.JBI`` around Ø70 mm tube).

This is a HEURISTIC on tessellated geometry. Real weld-seam recognition on
B-Rep STEP data (true face adjacency, edge continuity, groove type) needs a
kernel like OpenCASCADE (pythonocc-core). Treat the returned edges as
suggestions for a human to confirm, never as an authoritative weld map.
"""

from __future__ import annotations

import math
from collections import defaultdict

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


def _as_vec(p) -> np.ndarray:
    return np.asarray(p, dtype=float).reshape(3)


def _tolist3(v: np.ndarray) -> list[float]:
    return [float(v[0]), float(v[1]), float(v[2])]


def _normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.zeros(3, dtype=float)
    return v / n


def _point_key(p: np.ndarray, ndigits: int = 5) -> tuple[float, float, float]:
    return (round(float(p[0]), ndigits), round(float(p[1]), ndigits), round(float(p[2]), ndigits))


def _segment_dir(seg: dict) -> np.ndarray:
    return _normalize(_as_vec(seg["end"]) - _as_vec(seg["start"]))


def _polyline_length(points: list[np.ndarray]) -> float:
    total = 0.0
    for i in range(len(points) - 1):
        total += float(np.linalg.norm(points[i + 1] - points[i]))
    return total


def merge_collinear_seams(
    segments: list[dict],
    *,
    angle_tol_deg: float = 8.0,
    gap_tol: float = 1e-4,
) -> list[dict]:
    """Merge consecutive collinear edge segments into longer linear seams.

    Two segments merge when an endpoint coincides (within ``gap_tol``) and
    their direction vectors are within ``angle_tol_deg``.
    """
    if len(segments) <= 1:
        return [dict(s, type=s.get("type", "line")) for s in segments]

    cos_tol = math.cos(math.radians(angle_tol_deg))
    used = [False] * len(segments)
    merged: list[dict] = []

    # Endpoint → segment indices for chaining.
    ends: dict[tuple[float, float, float], list[tuple[int, str]]] = defaultdict(list)
    for i, seg in enumerate(segments):
        ends[_point_key(_as_vec(seg["start"]))].append((i, "start"))
        ends[_point_key(_as_vec(seg["end"]))].append((i, "end"))

    def try_extend(chain_pts: list[np.ndarray], direction: np.ndarray) -> bool:
        tip = _point_key(chain_pts[-1])
        for idx, which in ends.get(tip, []):
            if used[idx]:
                continue
            seg = segments[idx]
            a = _as_vec(seg["start"])
            b = _as_vec(seg["end"])
            # Orient so we leave the tip.
            if which == "end":
                a, b = b, a
            d = _normalize(b - a)
            if float(np.dot(direction, d)) < cos_tol:
                continue
            # Collinear: new point must stay on the same line.
            if len(chain_pts) >= 2:
                lateral = np.linalg.norm(np.cross(direction, b - chain_pts[0]))
                if lateral > max(gap_tol * 10.0, 1e-3 * max(1.0, float(np.linalg.norm(b - chain_pts[0])))):
                    continue
            used[idx] = True
            chain_pts.append(b)
            return True
        return False

    for i, seg in enumerate(segments):
        if used[i]:
            continue
        used[i] = True
        pts = [_as_vec(seg["start"]), _as_vec(seg["end"])]
        direction = _normalize(pts[1] - pts[0])
        if float(np.linalg.norm(direction)) < 1e-12:
            merged.append(
                {
                    "start": _tolist3(pts[0]),
                    "end": _tolist3(pts[1]),
                    "length": float(seg.get("length", 0.0)),
                    "dihedral_deg": seg.get("dihedral_deg"),
                    "type": "line",
                }
            )
            continue
        # Grow forward then backward.
        while try_extend(pts, direction):
            direction = _normalize(pts[-1] - pts[0])
        pts_rev = list(reversed(pts))
        direction_rev = _normalize(pts_rev[-1] - pts_rev[0])
        while try_extend(pts_rev, direction_rev):
            direction_rev = _normalize(pts_rev[-1] - pts_rev[0])
        pts = list(reversed(pts_rev))
        length = _polyline_length(pts)
        out = {
            "start": _tolist3(pts[0]),
            "end": _tolist3(pts[-1]),
            "length": length,
            "dihedral_deg": seg.get("dihedral_deg"),
            "type": "line",
        }
        if len(pts) > 2:
            out["points"] = [_tolist3(p) for p in pts]
        merged.append(out)
    return merged


def _fit_circle_3d(
    points: list[np.ndarray],
) -> tuple[np.ndarray, np.ndarray, float] | None:
    """Fit a circle through ≥3 points. Returns (center, normal, radius) or None."""
    if len(points) < 3:
        return None
    pts = np.stack(points, axis=0)
    centroid = pts.mean(axis=0)
    centered = pts - centroid
    # Plane normal from SVD.
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = _normalize(vh[-1])
    if float(np.linalg.norm(normal)) < 1e-9:
        return None
    # Orthonormal basis in the plane.
    helper = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = _normalize(np.cross(normal, helper))
    v = _normalize(np.cross(normal, u))
    xy = np.column_stack((centered @ u, centered @ v))
    # Algebraic circle fit: x²+y² + D x + E y + F = 0.
    A = np.column_stack((xy[:, 0], xy[:, 1], np.ones(len(xy))))
    b = -(xy[:, 0] ** 2 + xy[:, 1] ** 2)
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    d, e, f = sol
    cx = -0.5 * d
    cy = -0.5 * e
    r2 = cx * cx + cy * cy - f
    if r2 <= 1e-12:
        return None
    radius = float(math.sqrt(r2))
    center = centroid + cx * u + cy * v
    return center, normal, radius


def _arc_via_point(
    start: np.ndarray,
    end: np.ndarray,
    center: np.ndarray,
    normal: np.ndarray,
    points: list[np.ndarray],
) -> np.ndarray:
    """Pick a via point ≈ mid-arc (prefer an interior sample)."""
    if len(points) >= 3:
        mid = points[len(points) // 2]
        # Ensure mid is not coincident with ends.
        if (
            float(np.linalg.norm(mid - start)) > 1e-9
            and float(np.linalg.norm(mid - end)) > 1e-9
        ):
            return mid
    # Geometric midpoint on the shorter arc.
    v0 = _normalize(start - center)
    v1 = _normalize(end - center)
    mid_dir = _normalize(v0 + v1)
    if float(np.linalg.norm(mid_dir)) < 1e-9:
        mid_dir = _normalize(np.cross(normal, v0))
    radius = float(np.linalg.norm(start - center))
    return center + mid_dir * radius


def group_constant_radius_arcs(
    segments: list[dict],
    *,
    min_edges: int = 3,
    radius_tol_frac: float = 0.12,
    max_edge_length: float | None = None,
    gap_tol: float = 1e-4,
) -> list[dict]:
    """Group neighbouring short edges that share a constant curvature radius.

    Production pipe welds (Ø70 mm → R≈35 mm in ``12.JBI`` / ``123.JBI``) are
    taught as a run of ``MOVC`` with ≥3 points. Tessellated fillet edges around
    a cylinder look like many short chords of nearly equal radius — those become
    a single ``type="arc"`` seam with ``start`` / ``via`` / ``end`` and ``points``.
    """
    if not segments:
        return []

    # Only consider short edges as arc candidates; long ones stay linear.
    lengths = [float(s.get("length", np.linalg.norm(_as_vec(s["end"]) - _as_vec(s["start"])))) for s in segments]
    median_len = float(np.median(lengths)) if lengths else 0.0
    edge_cap = max_edge_length if max_edge_length is not None else max(median_len * 3.0, 1e-6)

    candidates = [i for i, L in enumerate(lengths) if L <= edge_cap + 1e-12]
    if len(candidates) < min_edges:
        return [dict(s, type=s.get("type", "line")) for s in segments]

    ends: dict[tuple[float, float, float], list[tuple[int, str]]] = defaultdict(list)
    for i in candidates:
        seg = segments[i]
        ends[_point_key(_as_vec(seg["start"]))].append((i, "start"))
        ends[_point_key(_as_vec(seg["end"]))].append((i, "end"))

    used: set[int] = set()
    out: list[dict] = []
    leftover_idx = set(range(len(segments)))

    def grow_chain(seed: int) -> list[np.ndarray] | None:
        seg = segments[seed]
        pts = [_as_vec(seg["start"]), _as_vec(seg["end"])]
        used_local = {seed}

        def extend(tip_side: str) -> bool:
            tip = _point_key(pts[0] if tip_side == "start" else pts[-1])
            for idx, which in ends.get(tip, []):
                if idx in used or idx in used_local:
                    continue
                a = _as_vec(segments[idx]["start"])
                b = _as_vec(segments[idx]["end"])
                if tip_side == "end":
                    if which == "start":
                        nxt = b
                    elif which == "end":
                        nxt = a
                    else:
                        continue
                    if float(np.linalg.norm(pts[-1] - (a if which == "start" else b))) > gap_tol:
                        # Snap: tip must match the attached endpoint.
                        attach = a if which == "start" else b
                        if float(np.linalg.norm(pts[-1] - attach)) > gap_tol:
                            continue
                    pts.append(nxt)
                else:
                    if which == "end":
                        nxt = a
                    elif which == "start":
                        nxt = b
                    else:
                        continue
                    attach = b if which == "end" else a
                    if float(np.linalg.norm(pts[0] - attach)) > gap_tol:
                        continue
                    pts.insert(0, nxt)
                used_local.add(idx)
                return True
            return False

        grew = True
        while grew:
            grew = extend("end") or extend("start")
        if len(used_local) < min_edges or len(pts) < min_edges + 1:
            return None
        fit = _fit_circle_3d(pts)
        if fit is None:
            return None
        center, _normal, radius = fit
        if radius < 1e-6:
            return None
        # Radius consistency check at every sample.
        for p in pts:
            err = abs(float(np.linalg.norm(p - center)) - radius) / radius
            if err > radius_tol_frac:
                return None
        used.update(used_local)
        leftover_idx.difference_update(used_local)
        return pts

    for i in candidates:
        if i in used:
            continue
        chain = grow_chain(i)
        if chain is None:
            continue
        fit = _fit_circle_3d(chain)
        assert fit is not None
        center, normal, radius = fit
        start, end = chain[0], chain[-1]
        via = _arc_via_point(start, end, center, normal, chain)
        out.append(
            {
                "start": _tolist3(start),
                "via": _tolist3(via),
                "end": _tolist3(end),
                "points": [_tolist3(p) for p in chain],
                "length": _polyline_length(chain),
                "radius": float(radius),
                "center": _tolist3(center),
                "normal": _tolist3(normal),
                "type": "arc",
                "move_type": "MOVC",
                "dihedral_deg": segments[i].get("dihedral_deg"),
            }
        )

    # Remaining (unconsumed) segments stay as lines — still run collinear merge.
    remain = [segments[i] for i in sorted(leftover_idx)]
    out.extend(merge_collinear_seams(remain))
    return out


def detect_seams(
    file_path: str,
    angle_tol_deg: float = 20.0,
    target_angle_deg: float = 90.0,
    min_length: float = 1e-6,
    *,
    merge_collinear: bool = True,
    group_arcs: bool = True,
) -> dict:
    """Return candidate weld seams as 3D line / arc segments.

    A face-adjacency edge is kept when its dihedral angle is within
    ``angle_tol_deg`` of ``target_angle_deg`` and the joint is concave.
    Consecutive kept edges are merged into collinear polylines; short edges
    that share a constant curvature radius become ``type="arc"`` seams for
    ``MOVC`` postprocessing.
    """
    mesh = _as_single_mesh(trimesh.load(file_path, force="mesh"))

    angles = mesh.face_adjacency_angles  # radians, unsigned
    edges = mesh.face_adjacency_edges  # vertex-index pairs
    convex = mesh.face_adjacency_convex  # True=convex, False=concave

    target = math.radians(target_angle_deg)
    tol = math.radians(angle_tol_deg)

    verts = mesh.vertices
    segments: list[dict] = []

    for a_ang, (vi, vj), is_convex in zip(angles, edges, convex):
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
                "type": "line",
            }
        )

    if group_arcs and len(segments) >= 3:
        segments = group_constant_radius_arcs(segments)
    elif merge_collinear and segments:
        segments = merge_collinear_seams(segments)
    elif merge_collinear is False and group_arcs is False:
        pass
    elif merge_collinear and not group_arcs:
        segments = merge_collinear_seams(segments)

    total_len = float(sum(float(s.get("length", 0.0)) for s in segments))
    return {
        "seam_count": len(segments),
        "total_length": total_len,
        "units": "model units (as stored in the file)",
        "segments": segments,
        "note": (
            "Heuristic result from tessellated geometry. Confirm visually and "
            "against the B-Rep before generating a weld path. "
            "type=arc seams (constant radius) map to INFORM MOVC (≥3 points), "
            "as in production 12.JBI / 123.JBI pipe welds."
        ),
    }
