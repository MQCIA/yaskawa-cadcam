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

Multi-seam jobs (M2):
* Order + direction via greedy nearest-neighbour, then 2-opt (air length =
  retract → next approach).
* Clearance waypoint between seams when bisectors differ a lot (wall change)
  or always as a raised mid-air MOVJ so the torch clears the part.
* Per-seam ``arc_file`` → MACRO1 ARGF changes in the postprocessor (same pattern
  as production ``02025_KOSZ_…``).

Without a bisector the planner falls back to world-up lift and a fixed RPY.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .cell_frames import DEFAULT_TOUCH_SENSE, all_cube_aabbs_m
from .dx200_postprocessor import WeldSegment

# Production basket job ``02025_KOSZ_DO_RURY_70-PLUS-RAMKI.JBI`` has 200+ C#.
# Keep a soft cap so a noisy CAD detect cannot explode the path.
DEFAULT_MAX_SEAMS: int = 50
DEFAULT_CLEARANCE_HEIGHT: float = 0.08  # m extra lift for inter-seam air moves
# Insert a raised clearance when consecutive bisectors are this dissimilar
# (dot product below threshold ⇒ large orientation change).
DEFAULT_BISECTOR_CLEARANCE_DOT: float = 0.35


@dataclass
class PlannedPath:
    points: list[dict]  # ordered TCP poses {x,y,z,rx,ry,rz}
    weld_segments: list[WeldSegment]  # index ranges to wrap with ARCON/ARCOF
    # Oriented seam order after optimisation: (original_index, reversed).
    seam_order: list[tuple[int, bool]] = field(default_factory=list)
    air_length_m: float = 0.0


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


def _seam_endpoints(
    seam: dict, reversed_: bool
) -> tuple[np.ndarray, np.ndarray]:
    start = np.asarray(seam["start"], dtype=float)
    end = np.asarray(seam["end"], dtype=float)
    if reversed_:
        return end.copy(), start.copy()
    return start.copy(), end.copy()


def _approach_offset(seam: dict, approach_height: float) -> np.ndarray:
    bis = seam_bisector(seam)
    offset = bis * float(approach_height)
    if float(np.linalg.norm(offset)) < 1e-9:
        offset = np.array([0.0, 0.0, float(approach_height)])
    return offset


def seam_approach_point(
    seam: dict, *, reversed_: bool = False, approach_height: float = 0.05
) -> np.ndarray:
    start, _ = _seam_endpoints(seam, reversed_)
    return start + _approach_offset(seam, approach_height)


def seam_retract_point(
    seam: dict, *, reversed_: bool = False, approach_height: float = 0.05
) -> np.ndarray:
    _, end = _seam_endpoints(seam, reversed_)
    return end + _approach_offset(seam, approach_height)


def air_transition_length(
    from_seam: dict,
    from_reversed: bool,
    to_seam: dict,
    to_reversed: bool,
    *,
    approach_height: float = 0.05,
) -> float:
    """Straight-line air length retract(from) → approach(to)."""
    a = seam_retract_point(
        from_seam, reversed_=from_reversed, approach_height=approach_height
    )
    b = seam_approach_point(
        to_seam, reversed_=to_reversed, approach_height=approach_height
    )
    return float(np.linalg.norm(b - a))


def total_air_length(
    seams: list[dict],
    order: list[tuple[int, bool]],
    *,
    approach_height: float = 0.05,
    start_point: np.ndarray | None = None,
) -> float:
    """Sum of air moves: optional start→first approach + retract→approach links."""
    if not order:
        return 0.0
    total = 0.0
    first_i, first_rev = order[0]
    first_ap = seam_approach_point(
        seams[first_i], reversed_=first_rev, approach_height=approach_height
    )
    if start_point is not None:
        total += float(np.linalg.norm(first_ap - np.asarray(start_point, dtype=float)))
    for k in range(len(order) - 1):
        i0, r0 = order[k]
        i1, r1 = order[k + 1]
        total += air_transition_length(
            seams[i0], r0, seams[i1], r1, approach_height=approach_height
        )
    return total


def greedy_nearest_neighbor_order(
    seams: list[dict],
    *,
    approach_height: float = 0.05,
    start_point: np.ndarray | None = None,
) -> list[tuple[int, bool]]:
    """Greedy NN over seams × {forward, reversed} minimising air length."""
    n = len(seams)
    if n == 0:
        return []
    remaining = set(range(n))
    order: list[tuple[int, bool]] = []

    # Seed: seam+direction closest to start_point (or shortest approach from origin).
    origin = (
        np.asarray(start_point, dtype=float)
        if start_point is not None
        else np.zeros(3, dtype=float)
    )
    best: tuple[float, int, bool] | None = None
    for i in remaining:
        for rev in (False, True):
            ap = seam_approach_point(
                seams[i], reversed_=rev, approach_height=approach_height
            )
            d = float(np.linalg.norm(ap - origin))
            if best is None or d < best[0]:
                best = (d, i, rev)
    assert best is not None
    _, i0, r0 = best
    order.append((i0, r0))
    remaining.remove(i0)

    while remaining:
        prev_i, prev_r = order[-1]
        best_next: tuple[float, int, bool] | None = None
        for i in remaining:
            for rev in (False, True):
                d = air_transition_length(
                    seams[prev_i],
                    prev_r,
                    seams[i],
                    rev,
                    approach_height=approach_height,
                )
                if best_next is None or d < best_next[0]:
                    best_next = (d, i, rev)
        assert best_next is not None
        _, i_n, r_n = best_next
        order.append((i_n, r_n))
        remaining.remove(i_n)
    return order


def _two_opt_reverse(
    order: list[tuple[int, bool]], i: int, j: int
) -> list[tuple[int, bool]]:
    """Reverse subsequence order[i:j+1], flipping each seam direction."""
    mid = [(idx, not rev) for idx, rev in reversed(order[i : j + 1])]
    return order[:i] + mid + order[j + 1 :]


def two_opt_improve(
    seams: list[dict],
    order: list[tuple[int, bool]],
    *,
    approach_height: float = 0.05,
    start_point: np.ndarray | None = None,
    max_passes: int = 32,
) -> list[tuple[int, bool]]:
    """2-opt (+ single-seam direction flips) until local air-length minimum."""
    if len(order) < 2:
        return list(order)
    current = list(order)
    best_len = total_air_length(
        seams, current, approach_height=approach_height, start_point=start_point
    )
    n = len(current)
    for _ in range(max_passes):
        improved = False
        # Direction flip of one seam.
        for k in range(n):
            trial = list(current)
            idx, rev = trial[k]
            trial[k] = (idx, not rev)
            L = total_air_length(
                seams, trial, approach_height=approach_height, start_point=start_point
            )
            if L + 1e-12 < best_len:
                current, best_len, improved = trial, L, True
                break
        if improved:
            continue
        # Classic 2-opt edge swap via subsequence reverse.
        for i in range(n - 1):
            for j in range(i + 1, n):
                trial = _two_opt_reverse(current, i, j)
                L = total_air_length(
                    seams,
                    trial,
                    approach_height=approach_height,
                    start_point=start_point,
                )
                if L + 1e-12 < best_len:
                    current, best_len, improved = trial, L, True
                    break
            if improved:
                break
        if not improved:
            break
    return current


def optimize_seam_sequence(
    seams: list[dict],
    *,
    approach_height: float = 0.05,
    start_point: np.ndarray | None = None,
) -> list[tuple[int, bool]]:
    """Greedy nearest-neighbour seed + 2-opt polish (order and direction)."""
    if len(seams) <= 1:
        return [(0, False)] if seams else []
    seed = greedy_nearest_neighbor_order(
        seams, approach_height=approach_height, start_point=start_point
    )
    return two_opt_improve(
        seams,
        seed,
        approach_height=approach_height,
        start_point=start_point,
    )


def needs_clearance(
    from_seam: dict,
    to_seam: dict,
    *,
    bisector_dot_threshold: float = DEFAULT_BISECTOR_CLEARANCE_DOT,
) -> bool:
    """True when consecutive seams sit on dissimilar walls (bisector change)."""
    b0 = seam_bisector(from_seam)
    b1 = seam_bisector(to_seam)
    return float(np.dot(b0, b1)) < float(bisector_dot_threshold)


def clearance_waypoint(
    from_seam: dict,
    from_reversed: bool,
    to_seam: dict,
    to_reversed: bool,
    *,
    approach_height: float = 0.05,
    clearance_height: float = DEFAULT_CLEARANCE_HEIGHT,
) -> tuple[np.ndarray, np.ndarray]:
    """Raised mid-air pose between retract(from) and approach(to).

    Returns ``(xyz, bisector_for_rpy)``. Lift uses the average outward bisector
    (plus a +Z component) so the torch clears the part while reorienting.
    """
    retract = seam_retract_point(
        from_seam, reversed_=from_reversed, approach_height=approach_height
    )
    approach = seam_approach_point(
        to_seam, reversed_=to_reversed, approach_height=approach_height
    )
    mid = 0.5 * (retract + approach)
    b0 = seam_bisector(from_seam)
    b1 = seam_bisector(to_seam)
    lift_dir = _normalize(b0 + b1)
    if float(np.linalg.norm(lift_dir)) < 1e-9:
        lift_dir = np.array([0.0, 0.0, 1.0])
    # Always keep some world-up so we do not dive into a horizontal table.
    lift_dir = _normalize(lift_dir + np.array([0.0, 0.0, 0.35]))
    pos = mid + lift_dir * float(clearance_height)
    # Prefer a higher Z than both air points when the lift still dips.
    z_floor = max(float(retract[2]), float(approach[2])) + 0.5 * float(clearance_height)
    if pos[2] < z_floor:
        pos = pos.copy()
        pos[2] = z_floor
    return pos, lift_dir


def touch_sense_offsets_m(
    *,
    pattern_distance_mm: float | None = None,
    pattern_distance2_mm: float | None = None,
) -> tuple[float, float]:
    """SRCH DIS values (mm) → metres along +bisector for sense poses."""
    d1 = (
        float(pattern_distance_mm)
        if pattern_distance_mm is not None
        else float(DEFAULT_TOUCH_SENSE.distance_mm)
    )
    d2 = (
        float(pattern_distance2_mm)
        if pattern_distance2_mm is not None
        else float(
            DEFAULT_TOUCH_SENSE.distance2_mm
            if DEFAULT_TOUCH_SENSE.distance2_mm is not None
            else d1 * 0.5
        )
    )
    return d1 / 1000.0, d2 / 1000.0


def insert_touch_sense_points(
    points: list[dict],
    weld_segments: list[WeldSegment],
    *,
    bisector: np.ndarray | None = None,
    seam_start: np.ndarray | None = None,
) -> tuple[list[dict], list[WeldSegment]]:
    """Insert two SRCH-style sense poses before the first weld (SZUKAJ1 / SRCH-2D1).

    Sense points sit along +bisector at DIS=20 mm and DIS=10 mm from the first
    seam start (production ``SRCH-2D1`` distances). Weld segment indices shift.
    """
    if not weld_segments or not points:
        return points, weld_segments
    first = weld_segments[0]
    # First weld sample is at start_index; approach is start_index - 1 when present.
    weld0 = points[first.start_index]
    if seam_start is None:
        seam_start = np.array(
            [weld0["x"], weld0["y"], weld0["z"]], dtype=float
        )
    bis = (
        _normalize(np.asarray(bisector, dtype=float))
        if bisector is not None
        else np.array([0.0, 0.0, 1.0])
    )
    if float(np.linalg.norm(bis)) < 1e-9:
        bis = np.array([0.0, 0.0, 1.0])
    d20, d10 = touch_sense_offsets_m()
    # Lateral offset for the second search (SRCH-2D1 second direction).
    helper = np.array([1.0, 0.0, 0.0]) if abs(bis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    lateral = _normalize(np.cross(bis, helper)) * 0.03

    def sense_pose(p: np.ndarray) -> dict:
        return {
            "x": float(p[0]),
            "y": float(p[1]),
            "z": float(p[2]),
            "rx": float(weld0.get("rx", 180.0)),
            "ry": float(weld0.get("ry", 0.0)),
            "rz": float(weld0.get("rz", 0.0)),
            "kind": "sense",
            "tag": "TOUCH",
        }

    s1 = sense_pose(np.asarray(seam_start, dtype=float) + bis * d20)
    s2 = sense_pose(np.asarray(seam_start, dtype=float) + bis * d10 + lateral)
    insert_at = max(0, first.start_index)
    new_points = points[:insert_at] + [s1, s2] + points[insert_at:]
    shift = 2
    new_segs = [
        WeldSegment(
            start_index=s.start_index + (shift if s.start_index >= insert_at else 0),
            end_index=s.end_index + (shift if s.end_index >= insert_at else 0),
            weld_speed=s.weld_speed,
            arc_file=s.arc_file,
            weave_no=s.weave_no,
        )
        for s in weld_segments
    ]
    return new_points, new_segs


def cube_interference_aabbs_m() -> list[
    tuple[tuple[float, float, float], tuple[float, float, float]]
]:
    """CUBEINTF soft cubes as (min, max) metres for collision checks."""
    return [(mn, mx) for _name, mn, mx in all_cube_aabbs_m()]


def _point_in_aabb(
    p: np.ndarray,
    mn: tuple[float, float, float],
    mx: tuple[float, float, float],
    *,
    margin: float = 0.02,
) -> bool:
    return bool(
        p[0] >= mn[0] - margin
        and p[0] <= mx[0] + margin
        and p[1] >= mn[1] - margin
        and p[1] <= mx[1] + margin
        and p[2] >= mn[2] - margin
        and p[2] <= mx[2] + margin
    )


def segment_hits_aabb(
    a: np.ndarray,
    b: np.ndarray,
    mn: tuple[float, float, float],
    mx: tuple[float, float, float],
    *,
    steps: int = 8,
    margin: float = 0.02,
) -> bool:
    """Sample the segment a→b; True if any sample lies in the AABB (+margin)."""
    aa = np.asarray(a, dtype=float)
    bb = np.asarray(b, dtype=float)
    for i in range(steps + 1):
        t = i / steps
        p = aa + t * (bb - aa)
        if _point_in_aabb(p, mn, mx, margin=margin):
            return True
    return False


def path_hits_cube_interference(
    points: list[dict],
    *,
    skip_kinds: frozenset[str] | set[str] | None = None,
) -> list[str]:
    """Return names of CUBEINTF zones that air-move segments intersect.

    Weld and sense segments are skipped by default (torch is meant to work
    near the part / search into the joint). Soft cubes are coarse keep-outs
    for travel/approach/retract air moves.
    """
    skip = skip_kinds if skip_kinds is not None else frozenset({"weld", "sense"})
    cubes = all_cube_aabbs_m()
    hits: list[str] = []
    for i in range(len(points) - 1):
        a, b = points[i], points[i + 1]
        ka = str(a.get("kind", ""))
        kb = str(b.get("kind", ""))
        if ka in skip and kb in skip:
            continue
        pa = np.array([a["x"], a["y"], a["z"]], dtype=float)
        pb = np.array([b["x"], b["y"], b["z"]], dtype=float)
        for name, mn, mx in cubes:
            if name not in hits and segment_hits_aabb(pa, pb, mn, mx):
                hits.append(name)
    return hits


def check_collision_free(
    points: list[dict],
    *,
    skip_kinds: frozenset[str] | set[str] | None = None,
) -> bool:
    """True when air-moves clear all CUBEINTF soft cubes."""
    return not path_hits_cube_interference(points, skip_kinds=skip_kinds)


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
    max_seams: int = DEFAULT_MAX_SEAMS,
    optimize: bool = True,
    clearance_height: float = DEFAULT_CLEARANCE_HEIGHT,
    bisector_clearance_dot: float = DEFAULT_BISECTOR_CLEARANCE_DOT,
    start_point: list[float] | tuple[float, float, float] | None = None,
    touch_sense: bool = False,
) -> PlannedPath:
    """Build a weld path from seam segments (each with 'start'/'end' 3D lists).

    When a segment provides ``bisector`` / ``n1``+``n2``, approach and retract
    are offset along the bisector and torch RPY follows work + travel angles.
    Pass ``torch_rpy_deg`` to force a fixed orientation (legacy / override).

    ``max_seams`` caps how many input seams are planned (default 50, sized for
    production multi-seam jobs such as ``02025_KOSZ_…``). Set ``optimize=False``
    to keep input order and forward direction only.

    ``touch_sense=True`` inserts SRCH-2D1-distance sense poses before the first
    ARCON (pair with ``emit_touch_sense_call`` in the postprocessor).
    """
    if max_seams < 1:
        raise ValueError("max_seams must be >= 1")
    seams = list(seams[: int(max_seams)])
    if not seams:
        return PlannedPath(points=[], weld_segments=[], seam_order=[], air_length_m=0.0)

    start_pt = np.asarray(start_point, dtype=float) if start_point is not None else None
    if optimize and len(seams) > 1:
        order = optimize_seam_sequence(
            seams, approach_height=approach_height, start_point=start_pt
        )
    else:
        order = [(i, False) for i in range(len(seams))]

    points: list[dict] = []
    weld_segments: list[WeldSegment] = []
    air_len = total_air_length(
        seams, order, approach_height=approach_height, start_point=start_pt
    )
    first_bis: np.ndarray | None = None
    first_seam_start: np.ndarray | None = None

    def pose_at(p: np.ndarray, rx: float, ry: float, rz: float) -> dict:
        return {
            "x": float(p[0]),
            "y": float(p[1]),
            "z": float(p[2]),
            "rx": rx,
            "ry": ry,
            "rz": rz,
        }

    for k, (si, reversed_) in enumerate(order):
        seam = seams[si]
        start, end = _seam_endpoints(seam, reversed_)
        travel = end - start
        bis = seam_bisector(seam)
        wire = torch_wire_direction(
            bis, travel, travel_angle_deg=travel_angle_deg, push=push
        )
        if torch_rpy_deg is not None:
            rx, ry, rz = torch_rpy_deg
        else:
            rx, ry, rz = torch_dir_to_rpy_deg(wire)

        offset = _approach_offset(seam, approach_height)
        raw_arc = seam.get("arc_file", None)
        seg_arc = int(arc_file if raw_arc is None else raw_arc)
        raw_speed = seam.get("weld_speed", None)
        seg_speed = float(weld_speed if raw_speed is None else raw_speed)

        if k == 0:
            first_bis = bis
            first_seam_start = start.copy()

        # Clearance air move between previous retract and this approach so the
        # torch clears the part while reorienting (taller when walls change).
        if k > 0:
            prev_i, prev_rev = order[k - 1]
            prev_seam = seams[prev_i]
            extra = float(clearance_height)
            if needs_clearance(
                prev_seam,
                seam,
                bisector_dot_threshold=bisector_clearance_dot,
            ):
                extra = max(extra, float(clearance_height) * 1.5)
            clr_xyz, clr_bis = clearance_waypoint(
                prev_seam,
                prev_rev,
                seam,
                reversed_,
                approach_height=approach_height,
                clearance_height=extra,
            )
            if torch_rpy_deg is not None:
                crx, cry, crz = torch_rpy_deg
            else:
                # Keep tool roughly along the previous wire until re-approach.
                crx, cry, crz = torch_dir_to_rpy_deg(-clr_bis)
            points.append(pose_at(clr_xyz, crx, cry, crz))

        points.append(pose_at(start + offset, rx, ry, rz))  # approach
        weld_start_idx = len(points)

        n = max(2, samples_per_seam)
        for t in np.linspace(0.0, 1.0, n):
            points.append(pose_at(start + t * (end - start), rx, ry, rz))

        weld_end_idx = len(points) - 1
        points.append(pose_at(end + offset, rx, ry, rz))  # retract

        weld_segments.append(
            WeldSegment(
                start_index=weld_start_idx,
                end_index=weld_end_idx,
                weld_speed=seg_speed,
                arc_file=seg_arc,
            )
        )

    if touch_sense:
        points, weld_segments = insert_touch_sense_points(
            points,
            weld_segments,
            bisector=first_bis,
            seam_start=first_seam_start,
        )

    return PlannedPath(
        points=points,
        weld_segments=weld_segments,
        seam_order=order,
        air_length_m=air_len,
    )
