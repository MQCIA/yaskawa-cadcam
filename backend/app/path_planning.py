"""
Turn detected weld seams into an ordered TCP path for the robot.

This is a deliberately simple planner: for every seam segment it emits an
approach point, sampled points along the seam (arc on), and a retract point
(arc off). Torch orientation is a fixed placeholder pointing down into the
joint.

``type="arc"`` seams (from ``cad_analysis.group_constant_radius_arcs``) emit
≥3 weld samples tagged ``move_type="MOVC"`` so the postprocessor can match
production pipe welds (``12.JBI`` / ``123.JBI``).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dx200_postprocessor import WeldSegment


@dataclass
class PlannedPath:
    points: list[dict]  # ordered TCP poses {x,y,z,rx,ry,rz[,move_type]}
    weld_segments: list[WeldSegment]  # index ranges to wrap with ARCON/ARCOF


def _sample_arc_points(seam: dict, samples: int) -> list[np.ndarray]:
    """Sample an arc seam: prefer explicit points, else start/via/end circle."""
    raw = seam.get("points")
    if isinstance(raw, (list, tuple)) and len(raw) >= 3:
        pts = [np.asarray(p, dtype=float) for p in raw]
        if samples <= len(pts):
            # Evenly pick from the polyline.
            idx = np.linspace(0, len(pts) - 1, max(3, samples))
            return [pts[int(round(i))] for i in idx]
        return pts

    start = np.asarray(seam["start"], dtype=float)
    end = np.asarray(seam["end"], dtype=float)
    via = seam.get("via")
    if via is not None:
        via_p = np.asarray(via, dtype=float)
        # Quadratic Bezier through start/via/end as a stable ≥3-point path.
        n = max(3, samples)
        out: list[np.ndarray] = []
        for t in np.linspace(0.0, 1.0, n):
            p = (1 - t) ** 2 * start + 2 * (1 - t) * t * via_p + t**2 * end
            out.append(p)
        return out

    n = max(2, samples)
    return [start + t * (end - start) for t in np.linspace(0.0, 1.0, n)]


def plan_from_seams(
    seams: list[dict],
    *,
    samples_per_seam: int = 5,
    approach_height: float = 0.05,  # m above the seam start/end
    torch_rpy_deg: tuple[float, float, float] = (180.0, 0.0, 0.0),
    weld_speed: float = 10.0,
    arc_file: int = 1,
) -> PlannedPath:
    """Build a weld path from seam segments (each with 'start'/'end' 3D lists).

    Arc seams (``type="arc"`` / ``move_type="MOVC"``) get circular samples
    tagged for INFORM ``MOVC``; linear seams use ``MOVL``.
    """
    rx, ry, rz = torch_rpy_deg
    points: list[dict] = []
    weld_segments: list[WeldSegment] = []

    def pose(p, *, move_type: str | None = None) -> dict:
        out = {
            "x": float(p[0]),
            "y": float(p[1]),
            "z": float(p[2]),
            "rx": rx,
            "ry": ry,
            "rz": rz,
        }
        if move_type:
            out["move_type"] = move_type
        return out

    for seam in seams:
        is_arc = (
            str(seam.get("type", "")).lower() == "arc"
            or str(seam.get("move_type", "")).upper() == "MOVC"
        )
        move_type = "MOVC" if is_arc else "MOVL"
        if is_arc:
            weld_pts = _sample_arc_points(seam, max(3, samples_per_seam))
        else:
            start = np.asarray(seam["start"], dtype=float)
            end = np.asarray(seam["end"], dtype=float)
            n = max(2, samples_per_seam)
            weld_pts = [start + t * (end - start) for t in np.linspace(0.0, 1.0, n)]

        start = weld_pts[0]
        end = weld_pts[-1]
        up = np.array([0.0, 0.0, approach_height])

        points.append(pose(start + up, move_type="MOVJ"))  # approach (air)
        weld_start_idx = len(points)

        for p in weld_pts:
            points.append(pose(p, move_type=move_type))

        weld_end_idx = len(points) - 1
        points.append(pose(end + up, move_type="MOVJ"))  # retract

        weld_segments.append(
            WeldSegment(
                start_index=weld_start_idx,
                end_index=weld_end_idx,
                weld_speed=weld_speed,
                arc_file=arc_file,
                move_type=move_type,
            )
        )

    return PlannedPath(points=points, weld_segments=weld_segments)
