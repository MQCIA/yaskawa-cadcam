"""
Turn detected weld seams into an ordered TCP path for the robot.

For fillet seams that carry face normals / a bisector (from ``cad_analysis``),
torch orientation and approach/retract are derived from geometry:

* Work angle: wire aims along ``−bisector`` (into the ~90° fillet, typically 45°
  to each wall).
* Travel / push-pull angle: wire is tilted about the axis ⊥ to (wire × travel)
  by ``travel_angle_deg`` (default 12°).
* Approach / retract: offset along ``+bisector`` (out of the joint), not world +Z,
  so vertical / side seams do not drive the torch through the part.

Without a bisector the planner falls back to world-up lift and a fixed RPY.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dx200_postprocessor import WeldSegment


@dataclass
class PlannedPath:
    points: list[dict]  # ordered TCP poses {x,y,z,rx,ry,rz}
    weld_segments: list[WeldSegment]  # index ranges to wrap with ARCON/ARCOF


def _normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.zeros(3, dtype=float)
    return v / n


def seam_bisector(seam: dict) -> np.ndarray:
    """Outward fillet bisector from a CAD segment (or inferred defaults)."""
    if "bisector" in seam and seam["bisector"] is not None:
        return _normalize(np.asarray(seam["bisector"], dtype=float))
    if seam.get("n1") is not None and seam.get("n2") is not None:
        n1 = _normalize(np.asarray(seam["n1"], dtype=float))
        n2 = _normalize(np.asarray(seam["n2"], dtype=float))
        b = _normalize(n1 + n2)
        if float(np.linalg.norm(b)) > 1e-9:
            return b
    return np.array([0.0, 0.0, 1.0])


def _rotation_matrix(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    """Rodrigues rotation of ``angle_rad`` about unit ``axis``."""
    ax = _normalize(axis)
    if float(np.linalg.norm(ax)) < 1e-9 or abs(angle_rad) < 1e-12:
        return np.eye(3)
    x, y, z = ax
    c = float(np.cos(angle_rad))
    s = float(np.sin(angle_rad))
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ],
        dtype=float,
    )


def torch_wire_direction(
    bisector: np.ndarray,
    travel: np.ndarray,
    *,
    travel_angle_deg: float = 12.0,
    push: bool = True,
) -> np.ndarray:
    """Unit wire / tool approach direction (into the joint).

    ``bisector`` points out of the fillet; wire aims along ``−bisector``, then
    tilted by ``travel_angle_deg`` about an axis ⊥ to wire and travel.
    ``push=True`` tilts the tip to trail the motion (common fillet push).
    """
    wire = _normalize(-np.asarray(bisector, dtype=float))
    if float(np.linalg.norm(wire)) < 1e-9:
        wire = np.array([0.0, -1.0, 0.0])
    trav = _normalize(np.asarray(travel, dtype=float))
    if abs(travel_angle_deg) < 1e-9 or float(np.linalg.norm(trav)) < 1e-9:
        return wire

    axis = np.cross(wire, trav)
    if float(np.linalg.norm(axis)) < 1e-9:
        # Travel nearly parallel to wire — pick a stable perpendicular.
        helper = np.array([0.0, 0.0, 1.0]) if abs(wire[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        axis = np.cross(wire, helper)
    axis = _normalize(axis)
    signed = float(np.deg2rad(travel_angle_deg))
    if not push:
        signed = -signed
    return _normalize(_rotation_matrix(axis, signed) @ wire)


def torch_dir_to_rpy_deg(direction: np.ndarray) -> tuple[float, float, float]:
    """Approximate Motoman RECTAN Rx,Ry,Rz [deg] from tool −Z / wire direction.

    Mirrors ``frontend/lib/weldProgram.ts`` ``torchDirToRpy``.
    """
    d = _normalize(np.asarray(direction, dtype=float))
    if float(np.linalg.norm(d)) < 1e-9:
        return 180.0, 0.0, 0.0
    rx = float(np.degrees(np.arctan2(d[1], -d[2])))
    ry = float(np.degrees(np.arcsin(float(np.clip(-d[0], -1.0, 1.0)))))
    return rx, ry, 0.0


def plan_from_seams(
    seams: list[dict],
    *,
    samples_per_seam: int = 5,
    approach_height: float = 0.05,  # m along bisector (safe standoff)
    torch_rpy_deg: tuple[float, float, float] | None = None,
    travel_angle_deg: float = 12.0,
    push: bool = True,
    weld_speed: float = 10.0,
    arc_file: int = 1,
) -> PlannedPath:
    """Build a weld path from seam segments (each with 'start'/'end' 3D lists).

    When a segment provides ``bisector`` / ``n1``+``n2``, approach and retract
    are offset along the bisector and torch RPY follows work + travel angles.
    Pass ``torch_rpy_deg`` to force a fixed orientation (legacy / override).
    """
    points: list[dict] = []
    weld_segments: list[WeldSegment] = []

    for seam in seams:
        start = np.asarray(seam["start"], dtype=float)
        end = np.asarray(seam["end"], dtype=float)
        travel = end - start
        bis = seam_bisector(seam)
        wire = torch_wire_direction(
            bis, travel, travel_angle_deg=travel_angle_deg, push=push
        )
        if torch_rpy_deg is not None:
            rx, ry, rz = torch_rpy_deg
        else:
            rx, ry, rz = torch_dir_to_rpy_deg(wire)

        # Outward offset for air moves — never default to world +Z alone.
        offset = bis * float(approach_height)
        if float(np.linalg.norm(offset)) < 1e-9:
            offset = np.array([0.0, 0.0, approach_height])

        def pose(p: np.ndarray) -> dict:
            return {
                "x": float(p[0]),
                "y": float(p[1]),
                "z": float(p[2]),
                "rx": rx,
                "ry": ry,
                "rz": rz,
            }

        points.append(pose(start + offset))  # approach
        weld_start_idx = len(points)

        n = max(2, samples_per_seam)
        for t in np.linspace(0.0, 1.0, n):
            points.append(pose(start + t * (end - start)))

        weld_end_idx = len(points) - 1
        points.append(pose(end + offset))  # retract

        weld_segments.append(
            WeldSegment(
                start_index=weld_start_idx,
                end_index=weld_end_idx,
                weld_speed=weld_speed,
                arc_file=arc_file,
            )
        )

    return PlannedPath(points=points, weld_segments=weld_segments)
