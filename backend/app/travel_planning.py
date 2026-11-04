"""
Safe inter-seam travel for the MA2010 + RECT-X (BS1) + TURN cell.

Mirrors production patterns mined from
``02025_KOSZ_DO_RURY_70-PLUS-RAMKI.JBI`` and ``1-DEFRO-5-KOSZ.JBI``:

* BS1 ``BC#`` works in stable *zones* (left / mid / right plateaus) — the
  carriage stays put while neighbouring seams weld and only indexes on MOVJ.
* Between faces, air moves use clearance via-points (``MOVJ … VJ=80 DEC=20``)
  instead of a straight line through the part.
* When ``EC#`` jumps more than ~5°, a 3-step safe-index keeps the torch out of
  the table swing (``C00163→C00164→C00165``: old EC → mid EC → new EC).

Speed tags follow the cell teach pendant:
  far / home          VJ=80  DEC=20
  near / reorient     VJ=30 or 50  DEC=20
  weld + weave        V=4.2 or 5.0  DEC=20
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .pulse_config import (
    BASE_HOME_TRACK_PULSE,
    BASE_PARK_PULSE,
    STATION_PULSES_PER_DEGREE,
    station_deg_to_ec,
)


# Production BC plateaus (02025_KOSZ) — left / mid / right of the fixture.
BC_ZONE_ANCHORS: tuple[int, ...] = (
    124171,  # left basket wall — most common
    227797,  # mid-left
    250331,  # mid
    360505,  # right
    375361,  # far right
)
BC_QUANTIZE_PULSE = 18_000  # cluster seams whose ideal BC is within this
BC_REACH_TARGET_M = 1.10  # aim seam mid ~1.1 m from robot S (window 0.9–1.35)
BC_REACH_WINDOW_M = (0.90, 1.35)

# Approx. pulses/mm inferred from 02025 left→right span (~0.55 m fixture / ΔBC).
# Used only when CELL_PULSE.base_pulses_per_mm is None — zone planning heuristic.
BC_ESTIMATED_PULSES_PER_MM = 251190.0 / 550.0  # ≈ 456.7

EC_SAFE_INDEX_DEG = 5.0  # index when |ΔEC| exceeds this
CLEARANCE_NORMAL_DEG = 60.0
CLEARANCE_MARGIN_M = 0.06  # expanded AABB / hull margin
CLEARANCE_Z_LIFT_M = 0.15  # Z_safe = max_z + 150 mm
SAFE_INDEX_CLEARANCE_M = 0.25  # pull-back before table swing

VJ_FAR = 80.0
VJ_NEAR = 50.0
VJ_TIGHT = 30.0
V_WELD_WEAVE = 4.2
V_WELD_DEFAULT = 5.0
DEC_DEFAULT = 20


def _normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.zeros(3, dtype=float)
    return v / n


def _xyz(p: dict) -> np.ndarray:
    return np.array([float(p["x"]), float(p["y"]), float(p["z"])], dtype=float)


def _rpy(p: dict) -> tuple[float, float, float]:
    return float(p.get("rx", 0.0)), float(p.get("ry", 0.0)), float(p.get("rz", 0.0))


def _with_motion(
    p: dict,
    *,
    move_type: str = "MOVJ",
    kind: str = "travel",
    vj: float = VJ_FAR,
    ec_vj: float | None = None,
    v: float | None = None,
    dec: int = DEC_DEFAULT,
    bc: int | None = None,
    ec: tuple[int, int] | None = None,
) -> dict:
    out = dict(p)
    out["move_type"] = move_type
    out["kind"] = kind
    out["vj"] = float(vj)
    out["ec_vj"] = float(ec_vj if ec_vj is not None else vj)
    out["dec"] = int(dec)
    if v is not None:
        out["v"] = float(v)
    if bc is not None:
        out["bc"] = int(bc)
    if ec is not None:
        out["ec"] = [int(ec[0]), int(ec[1])]
    return out


def seam_midpoint(seam: dict) -> np.ndarray:
    if "points" in seam and isinstance(seam["points"], (list, tuple)) and seam["points"]:
        pts = np.asarray(seam["points"], dtype=float)
        return pts.mean(axis=0)
    a = np.asarray(seam["start"], dtype=float)
    b = np.asarray(seam["end"], dtype=float)
    return 0.5 * (a + b)


def ideal_bc_for_seam(
    seam: dict,
    *,
    rail_axis: int = 0,
    pulses_per_mm: float | None = None,
    robot_base_along_rail_m: float = 0.0,
    rail_span_m: tuple[float, float] | None = None,
) -> int:
    """BC pulse that puts the seam mid in the MA2010 reach window.

    Production RECT-X has no certified lead (``base_pulses_per_mm`` is None),
    so the default path linearly maps the seam's rail coordinate into the
    ``02025_KOSZ`` left→right BC plateaus (124171 … 375361). When a measured
    ``pulses_per_mm`` *is* supplied, BC is solved so the seam sits ~1.1 m from
    the robot S axis.
    """
    mid = seam_midpoint(seam)
    seam_along = float(mid[rail_axis])

    if pulses_per_mm is not None and pulses_per_mm > 0:
        # Want: (seam_along_rail - robot_along_rail) ≈ BC_REACH_TARGET_M
        # robot_along_rail ≈ base + (bc − home_track) / ppm / 1000
        # ⇒ bc = home_track + (seam − base − target) * 1000 * ppm
        delta_m = seam_along - robot_base_along_rail_m - BC_REACH_TARGET_M
        raw = int(round(BASE_HOME_TRACK_PULSE + delta_m * 1000.0 * pulses_per_mm))
        lo = BC_ZONE_ANCHORS[0]
        hi = max(BC_ZONE_ANCHORS[-1], BASE_PARK_PULSE)
        return int(np.clip(raw, lo, hi))

    # Uncalibrated lead: map rail position into the production zone span.
    # Default fixture envelope covers the basket work volume (~0.55–1.55 m).
    lo_m, hi_m = rail_span_m if rail_span_m is not None else (0.55, 1.55)
    if hi_m <= lo_m:
        t = 0.5
    else:
        t = float(np.clip((seam_along - lo_m) / (hi_m - lo_m), 0.0, 1.0))
    bc_lo, bc_hi = BC_ZONE_ANCHORS[0], BC_ZONE_ANCHORS[-1]
    return int(round(bc_lo + t * (bc_hi - bc_lo)))


def quantize_bc_zones(
    ideal_bcs: list[int],
    *,
    tol: int = BC_QUANTIZE_PULSE,
    anchors: tuple[int, ...] = BC_ZONE_ANCHORS,
) -> list[int]:
    """Cluster nearby ideal BCs onto shared plateaus (carriage stays put)."""
    if not ideal_bcs:
        return []
    # Snap each ideal to nearest production anchor, then merge neighbours.
    snapped = [min(anchors, key=lambda a: abs(a - b)) for b in ideal_bcs]
    out = list(snapped)
    # Greedy merge along seam order: keep previous zone if within tol of ideal.
    for i in range(1, len(out)):
        if abs(ideal_bcs[i] - out[i - 1]) <= tol:
            out[i] = out[i - 1]
        elif abs(snapped[i] - out[i - 1]) <= tol:
            out[i] = out[i - 1]
    return out


def angle_between_deg(a: np.ndarray, b: np.ndarray) -> float:
    aa, bb = _normalize(a), _normalize(b)
    if float(np.linalg.norm(aa)) < 1e-9 or float(np.linalg.norm(bb)) < 1e-9:
        return 0.0
    c = float(np.clip(np.dot(aa, bb), -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))


def expanded_bounds_from_points(
    points: list[np.ndarray],
    *,
    margin: float = CLEARANCE_MARGIN_M,
) -> tuple[np.ndarray, np.ndarray]:
    if not points:
        z = np.zeros(3)
        return z - margin, z + margin
    stack = np.stack(points, axis=0)
    return stack.min(axis=0) - margin, stack.max(axis=0) + margin


def expanded_bounds_from_mesh(mesh, *, margin: float = CLEARANCE_MARGIN_M):
    if mesh is None:
        return None
    bmin = np.asarray(mesh.bounds[0], dtype=float) - margin
    bmax = np.asarray(mesh.bounds[1], dtype=float) + margin
    return bmin, bmax


def segment_hits_aabb(
    a: np.ndarray,
    b: np.ndarray,
    bmin: np.ndarray,
    bmax: np.ndarray,
    *,
    samples: int = 24,
) -> bool:
    """Conservative segment vs AABB test (sampled + endpoint checks)."""
    for t in np.linspace(0.0, 1.0, max(2, samples)):
        p = a + t * (b - a)
        if np.all(p >= bmin) and np.all(p <= bmax):
            return True
    return False


def needs_clearance_via(
    retract: dict,
    approach: dict,
    bis_a: np.ndarray,
    bis_b: np.ndarray,
    bounds: tuple[np.ndarray, np.ndarray] | None,
) -> bool:
    if angle_between_deg(bis_a, bis_b) > CLEARANCE_NORMAL_DEG:
        return True
    if bounds is None:
        return False
    return segment_hits_aabb(_xyz(retract), _xyz(approach), bounds[0], bounds[1])


def clearance_via_points(
    retract: dict,
    approach: dict,
    *,
    z_safe: float,
    bc: int,
    ec: tuple[int, int],
) -> list[dict]:
    """1–2 MOVJ vias: climb to Z_safe, optionally shift toward robot, then drop."""
    a = _xyz(retract)
    b = _xyz(approach)
    rpy = _rpy(retract)
    vias: list[dict] = []

    high_a = a.copy()
    high_a[2] = max(float(z_safe), float(a[2]) + CLEARANCE_Z_LIFT_M)
    # Pull slightly toward world origin (robot side) while high.
    pull = _normalize(np.array([-high_a[0], -high_a[1], 0.0]))
    if float(np.linalg.norm(pull)) < 1e-6:
        pull = np.array([0.0, -1.0, 0.0])
    mid = 0.5 * (a + b)
    mid[2] = high_a[2]
    mid = mid + pull * 0.08

    high_b = b.copy()
    high_b[2] = high_a[2]

    vias.append(
        _with_motion(
            {"x": high_a[0], "y": high_a[1], "z": high_a[2], "rx": rpy[0], "ry": rpy[1], "rz": rpy[2]},
            kind="clearance",
            vj=VJ_FAR,
            bc=bc,
            ec=ec,
        )
    )
    # Second via when lateral travel is long (>150 mm).
    if float(np.linalg.norm((b - a)[:2])) > 0.15:
        vias.append(
            _with_motion(
                {"x": mid[0], "y": mid[1], "z": mid[2], "rx": rpy[0], "ry": rpy[1], "rz": rpy[2]},
                kind="clearance",
                vj=VJ_FAR,
                bc=bc,
                ec=ec,
            )
        )
    vias.append(
        _with_motion(
            {"x": high_b[0], "y": high_b[1], "z": high_b[2], "rx": rpy[0], "ry": rpy[1], "rz": rpy[2]},
            kind="clearance",
            vj=VJ_NEAR,
            bc=bc,
            ec=ec,
        )
    )
    return vias


def ec_delta_deg(ec_a: tuple[int, int], ec_b: tuple[int, int]) -> float:
    """Max |Δ| across EC components, in degrees."""
    ppd = STATION_PULSES_PER_DEGREE
    d1 = abs(ec_b[0] - ec_a[0]) / ppd
    d2 = abs(ec_b[1] - ec_a[1]) / ppd
    return float(max(d1, d2))


def lerp_ec(ec_a: tuple[int, int], ec_b: tuple[int, int], t: float = 0.5) -> tuple[int, int]:
    return (
        int(round(ec_a[0] + t * (ec_b[0] - ec_a[0]))),
        int(round(ec_a[1] + t * (ec_b[1] - ec_a[1]))),
    )


def safe_index_pose(
    retract: dict,
    *,
    bc: int,
    ec: tuple[int, int],
    clearance_m: float = SAFE_INDEX_CLEARANCE_M,
) -> dict:
    """Pull torch outside the table swing radius before EC indexes."""
    p = _xyz(retract)
    rpy = _rpy(retract)
    radial = _normalize(np.array([-p[0], -p[1], 0.0]))
    if float(np.linalg.norm(radial)) < 1e-6:
        radial = np.array([0.0, -1.0, 0.0])
    safe = p + radial * clearance_m + np.array([0.0, 0.0, CLEARANCE_Z_LIFT_M])
    return _with_motion(
        {"x": safe[0], "y": safe[1], "z": safe[2], "rx": rpy[0], "ry": rpy[1], "rz": rpy[2]},
        kind="index_safe",
        vj=VJ_NEAR,
        bc=bc,
        ec=ec,
    )


def ec_safe_index_maneuver(
    retract: dict,
    approach_next: dict,
    *,
    ec_old: tuple[int, int],
    ec_new: tuple[int, int],
    bc_old: int,
    bc_new: int,
) -> list[dict]:
    """3-step EC index: safe@old → mid-EC@safe → approach-prep@new.

    Matches ``02025`` ``C00163→C00164→C00165`` (EC −933 → 38487 → 120166).
    """
    safe_old = safe_index_pose(retract, bc=bc_old, ec=ec_old)
    mid_ec = lerp_ec(ec_old, ec_new, 0.35)
    # Keep Cartesian pose while EC slews (table turns under a parked torch).
    mid = _with_motion(
        {k: safe_old[k] for k in ("x", "y", "z", "rx", "ry", "rz")},
        kind="index_safe",
        vj=VJ_NEAR,
        ec_vj=VJ_NEAR,
        bc=bc_new if abs(bc_new - bc_old) > BC_QUANTIZE_PULSE // 2 else bc_old,
        ec=mid_ec,
    )
    # Arrive at new EC still clear of the fixture, then caller adds approach.
    safe_new = _with_motion(
        {k: safe_old[k] for k in ("x", "y", "z", "rx", "ry", "rz")},
        kind="index_safe",
        vj=VJ_FAR,
        bc=bc_new,
        ec=ec_new,
    )
    # Nudge toward next approach while high.
    ap = _xyz(approach_next)
    safe_new["x"] = float(0.5 * (safe_old["x"] + ap[0]))
    safe_new["y"] = float(0.5 * (safe_old["y"] + ap[1]))
    safe_new["z"] = float(max(safe_old["z"], ap[2] + CLEARANCE_Z_LIFT_M))
    return [safe_old, mid, safe_new]


def resolve_seam_ec(seam: dict) -> tuple[int, int]:
    """EC pair for a seam — explicit pulses or station_deg → pulses."""
    raw = seam.get("ec")
    if isinstance(raw, (list, tuple)) and len(raw) >= 2:
        return int(raw[0]), int(raw[1])
    if "station_deg" in seam and seam["station_deg"] is not None:
        return station_deg_to_ec(float(seam["station_deg"]))
    if "ec1" in seam or "ec2" in seam:
        return int(seam.get("ec1", 0)), int(seam.get("ec2", 0))
    return (0, 0)


@dataclass
class SeamBlock:
    """One seam's local path before inter-seam stitching."""

    points: list[dict]
    weld_start: int
    weld_end: int
    bisector: np.ndarray
    mid: np.ndarray
    spin_deg: float
    dense_points: list[dict] = field(default_factory=list)
    dense_joints: list[list[float]] = field(default_factory=list)
    bc: int = BASE_PARK_PULSE
    ec: tuple[int, int] = (0, 0)
    move_type: str = "MOVL"
    weld_speed: float = V_WELD_DEFAULT
    arc_file: int = 1
    weave_no: int = 21


def annotate_seam_speeds(block: SeamBlock) -> None:
    """Stamp VJ/V/DEC on approach / weld / retract of one seam."""
    for i, p in enumerate(block.points):
        if i < block.weld_start:
            kind = "approach"
            mt = "MOVJ"
            vj = VJ_FAR if i == 0 else VJ_NEAR
            block.points[i] = _with_motion(
                p, move_type=mt, kind=kind, vj=vj, bc=block.bc, ec=block.ec
            )
        elif i > block.weld_end:
            block.points[i] = _with_motion(
                p,
                move_type="MOVJ",
                kind="retract",
                vj=VJ_NEAR,
                bc=block.bc,
                ec=block.ec,
            )
        else:
            # Weld samples — first point often coincides with ARCON.
            speed = block.weld_speed
            if abs(speed - 5.0) < 1e-6:
                speed = V_WELD_DEFAULT
            if abs(speed - 4.2) < 1e-6 or speed < 4.5:
                speed = V_WELD_WEAVE if speed <= 4.5 else speed
            block.points[i] = _with_motion(
                p,
                move_type=block.move_type,
                kind="weld",
                vj=VJ_FAR,
                v=float(speed),
                bc=block.bc,
                ec=block.ec,
            )


def stitch_seam_blocks(
    blocks: list[SeamBlock],
    *,
    mesh=None,
    part_points: list[np.ndarray] | None = None,
) -> tuple[list[dict], list[tuple[int, int, float, int, str, int]]]:
    """Insert clearance vias + EC safe-index between seams.

    Returns
    -------
    points : stitched path
    weld_meta : list of (start, end, weld_speed, arc_file, move_type, weave_no)
    """
    if not blocks:
        return [], []

    # Bounds for clearance tests.
    bounds = expanded_bounds_from_mesh(mesh) if mesh is not None else None
    if bounds is None and part_points:
        bounds = expanded_bounds_from_points(part_points)
    z_safe = float(bounds[1][2]) if bounds is not None else 0.6
    if bounds is None:
        # Fall back from seam geometry.
        zs = [float(b.mid[2]) for b in blocks]
        z_safe = max(zs) + CLEARANCE_Z_LIFT_M if zs else 0.6
    else:
        z_safe = float(bounds[1][2])  # already includes margin; add lift
        z_safe = z_safe - CLEARANCE_MARGIN_M + CLEARANCE_Z_LIFT_M

    out: list[dict] = []
    weld_meta: list[tuple[int, int, float, int, str, int]] = []

    for i, block in enumerate(blocks):
        annotate_seam_speeds(block)
        if i > 0:
            prev = blocks[i - 1]
            retract = out[-1]
            approach = block.points[0]
            ec_old, ec_new = prev.ec, block.ec
            bc_old, bc_new = prev.bc, block.bc

            if ec_delta_deg(ec_old, ec_new) > EC_SAFE_INDEX_DEG:
                for vp in ec_safe_index_maneuver(
                    retract,
                    approach,
                    ec_old=ec_old,
                    ec_new=ec_new,
                    bc_old=bc_old,
                    bc_new=bc_new,
                ):
                    out.append(vp)
            elif needs_clearance_via(
                retract, approach, prev.bisector, block.bisector, bounds
            ) or abs(bc_new - bc_old) > BC_QUANTIZE_PULSE // 2:
                # Carriage move or face change → clearance vias; BC switches on MOVJ.
                for vp in clearance_via_points(
                    retract,
                    approach,
                    z_safe=z_safe,
                    bc=bc_new if abs(bc_new - bc_old) > BC_QUANTIZE_PULSE // 2 else bc_old,
                    ec=ec_new if abs(bc_new - bc_old) <= BC_QUANTIZE_PULSE // 2 else ec_old,
                ):
                    # During pure BC index keep old EC; face-change vias already set.
                    if abs(bc_new - bc_old) > BC_QUANTIZE_PULSE // 2:
                        vp = _with_motion(
                            vp,
                            kind="clearance",
                            vj=float(vp.get("vj", VJ_FAR)),
                            bc=bc_new,
                            ec=ec_old,
                        )
                    out.append(vp)

        base = len(out)
        out.extend(block.points)
        weld_meta.append(
            (
                base + block.weld_start,
                base + block.weld_end,
                block.weld_speed,
                block.arc_file,
                block.move_type,
                block.weave_no,
            )
        )

    return out, weld_meta
