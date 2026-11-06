"""
Turn detected weld seams into an ordered TCP path for the robot.

M2.6: seams are ordered by nearest-neighbour (with optional start↔end flip)
to shorten air moves. Each seam emits approach → weld samples → retract;
between retract(i) and approach(i+1) the postprocessor emits MOVJ DEC=20.
Optional home bookends match production C00000 / BC=570125.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dx200_postprocessor import WeldSegment
from .kinematics_engine import forward_kinematics
from .robot_config import cell_home_deg


@dataclass
class PlannedPath:
    points: list[dict]                # ordered TCP poses {x,y,z,rx,ry,rz}
    weld_segments: list[WeldSegment]  # index ranges to wrap with ARCON/ARCOF
    seam_order: list[int] | None = None  # original seam indices after NN
    seam_flipped: list[bool] | None = None


def _dist(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b))


def order_seams_nearest_neighbor(
    seams: list[dict],
) -> tuple[list[dict], list[int], list[bool]]:
    """Greedy NN tour over seams; may flip start↔end to cut air travel.

    Returns (ordered_seams, original_indices, flipped_flags).
    """
    if not seams:
        return [], [], []
    if len(seams) == 1:
        s0 = {
            "start": list(seams[0]["start"]),
            "end": list(seams[0]["end"]),
            **{k: v for k, v in seams[0].items() if k not in ("start", "end")},
        }
        return [s0], [0], [False]

    remaining = list(range(len(seams)))
    # Start at the seam endpoint closest to cell-home TCP (APP_ESTIMATE).
    try:
        hp = _home_pose((180.0, 0.0, 0.0))
        seed = np.array([hp["x"], hp["y"], hp["z"]], dtype=float)
    except Exception:
        seed = np.zeros(3, dtype=float)

    best_i = remaining[0]
    best_d = float("inf")
    best_flip = False
    for i in remaining:
        a = np.asarray(seams[i]["start"], dtype=float)
        b = np.asarray(seams[i]["end"], dtype=float)
        da, db = _dist(seed, a), _dist(seed, b)
        if da <= db and da < best_d:
            best_d, best_i, best_flip = da, i, False
        elif db < best_d:
            best_d, best_i, best_flip = db, i, True

    order = [best_i]
    flips = [best_flip]
    remaining.remove(best_i)
    cur_end = np.asarray(
        seams[best_i]["start"] if best_flip else seams[best_i]["end"],
        dtype=float,
    )

    while remaining:
        pick = remaining[0]
        pick_flip = False
        pick_d = float("inf")
        for i in remaining:
            a = np.asarray(seams[i]["start"], dtype=float)
            b = np.asarray(seams[i]["end"], dtype=float)
            da, db = _dist(cur_end, a), _dist(cur_end, b)
            if da <= db and da < pick_d:
                pick_d, pick, pick_flip = da, i, False
            elif db < pick_d:
                pick_d, pick, pick_flip = db, i, True
        order.append(pick)
        flips.append(pick_flip)
        remaining.remove(pick)
        cur_end = np.asarray(
            seams[pick]["start"] if pick_flip else seams[pick]["end"],
            dtype=float,
        )

    ordered: list[dict] = []
    for i, flip in zip(order, flips):
        s = seams[i]
        start = list(s["end"] if flip else s["start"])
        end = list(s["start"] if flip else s["end"])
        ordered.append(
            {
                **{k: v for k, v in s.items() if k not in ("start", "end")},
                "start": start,
                "end": end,
            }
        )
    return ordered, order, flips


def _home_pose(torch_rpy_deg: tuple[float, float, float]) -> dict:
    """TCP at taught cell home (APP_ESTIMATE FK) — bookend for safe travel."""
    fk = forward_kinematics(cell_home_deg(), frame="tcp")
    rx, ry, rz = torch_rpy_deg
    return {
        "x": float(fk["x"]),
        "y": float(fk["y"]),
        "z": float(fk["z"]),
        "rx": rx,
        "ry": ry,
        "rz": rz,
        "role": "home",
    }


def plan_from_seams(
    seams: list[dict],
    *,
    samples_per_seam: int = 5,
    approach_height: float = 0.05,     # m above the seam start/end
    torch_rpy_deg: tuple[float, float, float] = (180.0, 0.0, 0.0),
    weld_speed: float = 10.0,
    arc_file: int = 1,
    home_bookends: bool = True,
    order_seams: bool = True,
) -> PlannedPath:
    """Build a weld path from seam segments (each with 'start'/'end' 3D lists)."""
    rx, ry, rz = torch_rpy_deg
    points: list[dict] = []
    weld_segments: list[WeldSegment] = []

    if order_seams and len(seams) > 1:
        ordered, orig_idx, flipped = order_seams_nearest_neighbor(seams)
    else:
        ordered = [
            {
                **{k: v for k, v in s.items() if k not in ("start", "end")},
                "start": list(s["start"]),
                "end": list(s["end"]),
            }
            for s in seams
        ]
        orig_idx = list(range(len(seams)))
        flipped = [False] * len(seams)

    def pose(p, *, role: str | None = None) -> dict:
        out = {
            "x": float(p[0]),
            "y": float(p[1]),
            "z": float(p[2]),
            "rx": rx,
            "ry": ry,
            "rz": rz,
        }
        if role:
            out["role"] = role
        return out

    if home_bookends:
        points.append(_home_pose(torch_rpy_deg))

    for seam in ordered:
        start = np.asarray(seam["start"], dtype=float)
        end = np.asarray(seam["end"], dtype=float)
        up = np.array([0.0, 0.0, approach_height])

        # Approach (air) — after previous retract this is the inter-seam MOVJ.
        points.append(pose(start + up, role="approach"))
        weld_start_idx = len(points)  # first welded point

        n = max(2, samples_per_seam)
        for t in np.linspace(0.0, 1.0, n):
            points.append(pose(start + t * (end - start), role="weld"))

        weld_end_idx = len(points) - 1
        points.append(pose(end + up, role="retract"))

        weld_segments.append(
            WeldSegment(
                start_index=weld_start_idx,
                end_index=weld_end_idx,
                weld_speed=weld_speed,
                arc_file=arc_file,
            )
        )

    if home_bookends:
        points.append(_home_pose(torch_rpy_deg))

    return PlannedPath(
        points=points,
        weld_segments=weld_segments,
        seam_order=orig_idx,
        seam_flipped=flipped,
    )
