"""
Turn detected weld seams into an ordered TCP path for the robot.

This is a deliberately simple planner: for every seam segment it emits an
approach point, sampled points along the seam (arc on), and a retract point
(arc off). Torch orientation is a fixed placeholder pointing down into the
joint. Real planning needs work-angle/travel-angle control, collision
avoidance, weave, lead-in/out and reachability checks -- see the limitations
in the README.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dx200_postprocessor import WeldSegment


@dataclass
class PlannedPath:
    points: list[dict]                # ordered TCP poses {x,y,z,rx,ry,rz}
    weld_segments: list[WeldSegment]  # index ranges to wrap with ARCON/ARCOF


def plan_from_seams(
    seams: list[dict],
    *,
    samples_per_seam: int = 5,
    approach_height: float = 0.05,     # m above the seam start/end
    torch_rpy_deg: tuple[float, float, float] = (180.0, 0.0, 0.0),
    weld_speed: float = 10.0,
    arc_file: int = 1,
) -> PlannedPath:
    """Build a weld path from seam segments (each with 'start'/'end' 3D lists)."""
    rx, ry, rz = torch_rpy_deg
    points: list[dict] = []
    weld_segments: list[WeldSegment] = []

    def pose(p) -> dict:
        return {"x": float(p[0]), "y": float(p[1]), "z": float(p[2]),
                "rx": rx, "ry": ry, "rz": rz}

    for seam in seams:
        start = np.asarray(seam["start"], dtype=float)
        end = np.asarray(seam["end"], dtype=float)
        up = np.array([0.0, 0.0, approach_height])

        points.append(pose(start + up))              # approach (air move)
        weld_start_idx = len(points)                 # first welded point

        n = max(2, samples_per_seam)
        for t in np.linspace(0.0, 1.0, n):
            points.append(pose(start + t * (end - start)))

        weld_end_idx = len(points) - 1               # last welded point
        points.append(pose(end + up))                # retract (air move)

        weld_segments.append(
            WeldSegment(
                start_index=weld_start_idx,
                end_index=weld_end_idx,
                weld_speed=weld_speed,
                arc_file=arc_file,
            )
        )

    return PlannedPath(points=points, weld_segments=weld_segments)
