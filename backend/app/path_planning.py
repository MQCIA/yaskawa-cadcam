"""
Turn detected weld seams into an ordered TCP path for the robot.

For fillet seams that carry face normals / a bisector (from ``cad_analysis``),
torch orientation and approach/retract are derived from geometry:

* Work angle: wire aims along ``−bisector`` (into the ~90° fillet, typically 45°
  to each wall).
* Travel / push-pull angle: wire is tilted about the axis ⊥ to (wire × travel)
  by ``travel_angle_deg`` (default 12°).
* Spin / roll ``ψ`` about the wire axis: free for MIG/MAG, but critical with a
  bent torch (TOOL 0 Ry ≈ −42.62°, X ≈ −54.28 mm). The planner samples ``ψ``,
  rejects IK failures / wrist singularities / joint jumps / torch–CAD hits, and
  picks the spin that stays closest to home (or the previous seam) while
  clearing joint limits.
* Approach / retract: offset along ``+bisector`` (out of the joint), not world +Z.

Without a bisector the planner falls back to world-up lift and a fixed RPY.

``type="arc"`` seams (from ``cad_analysis.group_constant_radius_arcs``) emit
≥3 weld samples tagged ``move_type="MOVC"`` so the postprocessor can match
production pipe welds (``12.JBI`` / ``123.JBI``).

Dense Cartesian→joint verification (every ``dense_step_m``, default 25 mm)
returns a full ``dense_joint_angles_deg`` array for smooth URDF animation even
when the INFORM ``.JBI`` only keeps Start/End ``MOVL`` points.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from spatialmath import SE3

from .dx200_postprocessor import WeldSegment
from .kinematics_engine import solve_ik_path, tool_file_se3
from .pulse_config import CELL_HOME_PULSES, CELL_PULSE, joint_pulses_to_deg
from .robot_config import ACTIVE_MODEL, TOOL0_XYZ_M
from .travel_planning import (
    SeamBlock,
    ideal_bc_for_seam,
    quantize_bc_zones,
    resolve_seam_ec,
    seam_midpoint,
    stitch_seam_blocks,
)


# ---------------------------------------------------------------------------
# Tunables
# ---------------------------------------------------------------------------
SPIN_STEP_DEG_DEFAULT = 30.0
DENSE_STEP_M_DEFAULT = 0.025  # 25 mm IK / animation samples along the seam
MAX_JOINT_STEP_DEG = 45.0  # reject ψ if R or T jumps more than this
B_AXIS_INDEX = 4  # wrist pitch (B / θ5)
R_AXIS_INDEX = 3
T_AXIS_INDEX = 5
LIMIT_MARGIN_DEG = 10.0  # soft penalty band near qlim


@dataclass
class SeamSpinResult:
    """Per-seam spin optimisation outcome."""

    spin_deg: float
    cost: float
    rejected: bool
    reason: str = ""
    candidate_count: int = 0
    feasible_count: int = 0


@dataclass
class PlannedPath:
    points: list[dict]  # ordered TCP poses {x,y,z,rx,ry,rz[,move_type,bc,ec,vj]}
    weld_segments: list[WeldSegment]  # index ranges to wrap with ARCON/ARCOF
    # Dense Cartesian samples + IK (animation / continuity); may be longer than
    # ``points`` when JBI keeps a sparse Start/End weld.
    dense_points: list[dict] = field(default_factory=list)
    dense_joint_angles_deg: list[list[float]] = field(default_factory=list)
    spin_angles_deg: list[float] = field(default_factory=list)
    spin_results: list[SeamSpinResult] = field(default_factory=list)
    max_joint_step_deg: float = 0.0
    bc_zones: list[int] = field(default_factory=list)


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
        helper = np.array([0.0, 0.0, 1.0]) if abs(wire[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        axis = np.cross(wire, helper)
    axis = _normalize(axis)
    signed = float(np.deg2rad(travel_angle_deg))
    if not push:
        signed = -signed
    return _normalize(_rotation_matrix(axis, signed) @ wire)


def torch_dir_to_rpy_deg(direction: np.ndarray) -> tuple[float, float, float]:
    """Approximate Motoman RECTAN Rx,Ry,Rz [deg] from tool −Z / wire direction."""
    d = _normalize(np.asarray(direction, dtype=float))
    if float(np.linalg.norm(d)) < 1e-9:
        return 180.0, 0.0, 0.0
    rx = float(np.degrees(np.arctan2(d[1], -d[2])))
    ry = float(np.degrees(np.arcsin(float(np.clip(-d[0], -1.0, 1.0)))))
    return rx, ry, 0.0


def _wire_reference_axes(
    wire: np.ndarray,
    travel: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Orthonormal tool axes: ``z`` = wire (approach), ``x`` from travel."""
    z = _normalize(wire)
    trav = _normalize(travel)
    x = np.cross(trav, z)
    if float(np.linalg.norm(x)) < 1e-9:
        helper = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        x = np.cross(helper, z)
    x = _normalize(x)
    y = _normalize(np.cross(z, x))
    return x, y, z


def rpy_from_wire_spin(
    wire: np.ndarray,
    travel: np.ndarray,
    spin_deg: float,
) -> tuple[float, float, float]:
    """Motoman xyz-RPY (deg) for wire approach + spin ``ψ`` about the wire.

    Columns of ``R`` are tool axes in world; tool +Z follows the wire (into the
    joint). ``ψ`` rotates the gooseneck / cable plane around that axis.
    """
    x0, y0, z = _wire_reference_axes(wire, travel)
    c = float(np.cos(np.deg2rad(spin_deg)))
    s = float(np.sin(np.deg2rad(spin_deg)))
    x = c * x0 + s * y0
    y = -s * x0 + c * y0
    R = np.column_stack((x, y, z))
    T = SE3.Rt(R, [0.0, 0.0, 0.0])
    rpy = T.rpy(unit="deg", order="xyz")
    return float(rpy[0]), float(rpy[1]), float(rpy[2])


def preferred_spin_deg(
    wire: np.ndarray,
    travel: np.ndarray,
    bisector: np.ndarray,
    *,
    step_deg: float = SPIN_STEP_DEG_DEFAULT,
) -> float:
    """Spin that swings the bent-torch body outward (along +bisector).

    Uses TOOL 0 stick-out geometry: ``tool0 → tcp_0`` with the candidate tip
    orientation; the component of that vector ⊥ to the wire should point out
    of the fillet so the gooseneck clears both walls.
    """
    bis = _normalize(bisector)
    wire_u = _normalize(wire)
    T_tool = tool_file_se3()
    best_psi = 0.0
    best_score = -1e18
    for psi in np.arange(-180.0, 180.0, step_deg):
        rx, ry, rz = rpy_from_wire_spin(wire_u, travel, float(psi))
        T_tip = SE3(0, 0, 0) * SE3.RPY([rx, ry, rz], unit="deg", order="xyz")
        # Treat tip as tcp_0 → recover tool0; body = tcp − tool0.
        T_tool0 = T_tip * T_tool.inv()
        body = np.asarray(T_tip.t - T_tool0.t, dtype=float)
        # Body direction from tip back to mount, world.
        mount_dir = _normalize(-body) if float(np.linalg.norm(body)) > 1e-12 else -wire_u
        lateral = mount_dir - float(np.dot(mount_dir, wire_u)) * wire_u
        if float(np.linalg.norm(lateral)) < 1e-9:
            score = 0.0
        else:
            score = float(np.dot(_normalize(lateral), bis))
        # Prefer gooseneck X-offset (−54 mm) swinging clear of the joint.
        if score > best_score:
            best_score = score
            best_psi = float(psi)
    return best_psi


def _angle_delta_deg(a: float, b: float) -> float:
    """Smallest signed delta b−a in (−180, 180]."""
    d = (b - a + 180.0) % 360.0 - 180.0
    return float(d)


def _max_joint_step(path_q: list[list[float]]) -> float:
    if len(path_q) < 2:
        return 0.0
    worst = 0.0
    for i in range(1, len(path_q)):
        for j in range(len(path_q[i])):
            worst = max(worst, abs(_angle_delta_deg(path_q[i - 1][j], path_q[i][j])))
    return float(worst)


def _crosses_wrist_singularity(path_q: list[list[float]]) -> bool:
    """True if axis B (θ5) changes sign through 0° between consecutive samples."""
    for i in range(1, len(path_q)):
        b0 = path_q[i - 1][B_AXIS_INDEX]
        b1 = path_q[i][B_AXIS_INDEX]
        if abs(b0) < 1e-9 or abs(b1) < 1e-9:
            continue
        if b0 * b1 < 0.0 and abs(_angle_delta_deg(b0, b1)) < 170.0:
            return True
    return False


def _rt_jump_too_large(path_q: list[list[float]], limit_deg: float = MAX_JOINT_STEP_DEG) -> bool:
    for i in range(1, len(path_q)):
        if abs(_angle_delta_deg(path_q[i - 1][R_AXIS_INDEX], path_q[i][R_AXIS_INDEX])) > limit_deg:
            return True
        if abs(_angle_delta_deg(path_q[i - 1][T_AXIS_INDEX], path_q[i][T_AXIS_INDEX])) > limit_deg:
            return True
    return False


def _joint_limit_penalty(q_deg: list[float]) -> float:
    """Higher when joints sit near catalogue soft-limits."""
    pen = 0.0
    for qi, joint in zip(q_deg, ACTIVE_MODEL.joints):
        lo, hi = np.rad2deg(joint.qlim[0]), np.rad2deg(joint.qlim[1])
        span = max(hi - lo, 1e-6)
        # Normalised distance to nearest limit in [0, 0.5].
        dist = min(qi - lo, hi - qi) / span
        if dist * span < LIMIT_MARGIN_DEG:
            pen += (LIMIT_MARGIN_DEG - dist * span) ** 2
    return float(pen)


def _home_distance(q_deg: list[float], q_ref: list[float]) -> float:
    return float(
        sum(_angle_delta_deg(a, b) ** 2 for a, b in zip(q_deg, q_ref)) ** 0.5
    )


def _torch_body_hits_mesh(
    poses: list[dict],
    mesh,
) -> bool:
    """Ray/segment check: tool0→tcp_0 body vs CAD mesh faces."""
    if mesh is None:
        return False
    T_tool = tool_file_se3()
    for p in poses:
        T_tip = SE3(p["x"], p["y"], p["z"]) * SE3.RPY(
            [p.get("rx", 0.0), p.get("ry", 0.0), p.get("rz", 0.0)],
            unit="deg",
            order="xyz",
        )
        # Planner xyz is the tip on the seam → treat as tcp_0.
        T_tool0 = T_tip * T_tool.inv()
        a = np.asarray(T_tool0.t, dtype=float)
        b = np.asarray(T_tip.t, dtype=float)
        direction = b - a
        length = float(np.linalg.norm(direction))
        if length < 1e-6:
            continue
        # Shrink slightly so the tip contact with the seam is not a hit.
        a2 = a + 0.02 * direction
        b2 = b - 0.005 * direction
        locations, _index_ray, _index_tri = mesh.ray.intersects_location(
            ray_origins=np.asarray([a2]),
            ray_directions=np.asarray([b2 - a2]),
        )
        if len(locations) == 0:
            continue
        # Count only intersections that lie on the finite segment.
        seg = b2 - a2
        seg_len2 = float(np.dot(seg, seg))
        for loc in locations:
            t = float(np.dot(loc - a2, seg) / seg_len2)
            if 0.0 <= t <= 1.0:
                return True
    return False


def _sample_polyline(points: list[np.ndarray], step_m: float) -> list[np.ndarray]:
    """Resample a polyline at ≤ ``step_m`` spacing (keeps endpoints)."""
    if len(points) == 0:
        return []
    if len(points) == 1:
        return [points[0].copy()]
    out: list[np.ndarray] = [points[0].copy()]
    for i in range(len(points) - 1):
        a = points[i]
        b = points[i + 1]
        seg = b - a
        length = float(np.linalg.norm(seg))
        if length < 1e-12:
            continue
        n = max(1, int(np.ceil(length / max(step_m, 1e-6))))
        for k in range(1, n + 1):
            out.append(a + (k / n) * seg)
    return out


def _sample_arc_points(seam: dict, samples: int) -> list[np.ndarray]:
    """Sample an arc seam: prefer explicit points, else start/via/end circle."""
    raw = seam.get("points")
    if isinstance(raw, (list, tuple)) and len(raw) >= 3:
        pts = [np.asarray(p, dtype=float) for p in raw]
        if samples <= len(pts):
            idx = np.linspace(0, len(pts) - 1, max(3, samples))
            return [pts[int(round(i))] for i in idx]
        return pts

    start = np.asarray(seam["start"], dtype=float)
    end = np.asarray(seam["end"], dtype=float)
    via = seam.get("via")
    if via is not None:
        via_p = np.asarray(via, dtype=float)
        n = max(3, samples)
        out: list[np.ndarray] = []
        for t in np.linspace(0.0, 1.0, n):
            p = (1 - t) ** 2 * start + 2 * (1 - t) * t * via_p + t**2 * end
            out.append(p)
        return out

    n = max(2, samples)
    return [start + t * (end - start) for t in np.linspace(0.0, 1.0, n)]


def _pose_dict(
    p: np.ndarray,
    rpy: tuple[float, float, float],
    *,
    mt: str | None = None,
    spin_deg: float | None = None,
) -> dict:
    out = {
        "x": float(p[0]),
        "y": float(p[1]),
        "z": float(p[2]),
        "rx": float(rpy[0]),
        "ry": float(rpy[1]),
        "rz": float(rpy[2]),
    }
    if mt:
        out["move_type"] = mt
    if spin_deg is not None:
        out["spin_deg"] = float(spin_deg)
    return out


def _key_sample_indices(n: int) -> list[int]:
    """Start / mid / end indices for a dense sample list."""
    if n <= 0:
        return []
    if n == 1:
        return [0]
    mid = n // 2
    return sorted({0, mid, n - 1})


def evaluate_spin_along_seam(
    weld_pts: list[np.ndarray],
    approach: np.ndarray,
    retract: np.ndarray,
    wire: np.ndarray,
    travel: np.ndarray,
    spin_deg: float,
    *,
    q_seed_deg: list[float],
    q_ref_deg: list[float],
    mesh=None,
    move_type: str = "MOVL",
    ik_frame: str = "tool0",
) -> tuple[bool, float, str, list[dict], list[list[float]]]:
    """Return (ok, cost, reason, poses, joints) for one ``ψ`` candidate."""
    rpy = rpy_from_wire_spin(wire, travel, spin_deg)
    poses: list[dict] = [
        _pose_dict(approach, rpy, mt="MOVJ", spin_deg=spin_deg),
    ]
    for p in weld_pts:
        poses.append(_pose_dict(p, rpy, mt=move_type, spin_deg=spin_deg))
    poses.append(_pose_dict(retract, rpy, mt="MOVJ", spin_deg=spin_deg))

    # Key poses: Approach, Start, Mid, End, Retract (for early reject).
    key_idx = [0] + [1 + i for i in _key_sample_indices(len(weld_pts))] + [len(poses) - 1]
    key_poses = [poses[i] for i in key_idx]
    ik_key = solve_ik_path(key_poses, q_seed_deg=q_seed_deg, frame=ik_frame)  # type: ignore[arg-type]
    if not ik_key["all_reachable"]:
        return False, 1e9, "ik_key_fail", poses, []

    if _torch_body_hits_mesh(key_poses, mesh):
        return False, 1e9, "torch_cad_collision", poses, []

    ik = solve_ik_path(poses, q_seed_deg=q_seed_deg, frame=ik_frame)  # type: ignore[arg-type]
    if not ik["all_reachable"]:
        return False, 1e9, "ik_dense_fail", poses, []

    joints: list[list[float]] = ik["joint_angles_deg"]
    # Continuity only along the weld (MOVL/MOVC), not the air approach jump.
    weld_joints = joints[1:-1]
    if _crosses_wrist_singularity(weld_joints):
        return False, 1e9, "wrist_singularity_B", poses, joints
    if _rt_jump_too_large(weld_joints):
        return False, 1e9, "joint_jump_RT", poses, joints

    # Cost: distance from reference (home / previous seam) + limit proximity.
    cost = 0.0
    for q in joints:
        cost += _home_distance(q, q_ref_deg)
        cost += 0.15 * _joint_limit_penalty(q)
    cost += 0.5 * _max_joint_step(weld_joints)
    return True, float(cost), "ok", poses, joints


def optimize_seam_spin(
    weld_pts: list[np.ndarray],
    approach: np.ndarray,
    retract: np.ndarray,
    wire: np.ndarray,
    travel: np.ndarray,
    bisector: np.ndarray,
    *,
    q_seed_deg: list[float],
    q_ref_deg: list[float],
    mesh=None,
    spin_step_deg: float = SPIN_STEP_DEG_DEFAULT,
    move_type: str = "MOVL",
    ik_frame: str = "tool0",
    fixed_rpy: tuple[float, float, float] | None = None,
) -> tuple[SeamSpinResult, list[dict], list[list[float]]]:
    """Discrete ``ψ`` search; returns best spin + dense poses/joints."""
    if fixed_rpy is not None:
        poses = [
            _pose_dict(approach, fixed_rpy, mt="MOVJ"),
            *[_pose_dict(p, fixed_rpy, mt=move_type) for p in weld_pts],
            _pose_dict(retract, fixed_rpy, mt="MOVJ"),
        ]
        ik = solve_ik_path(poses, q_seed_deg=q_seed_deg, frame=ik_frame)  # type: ignore[arg-type]
        ok = bool(ik["all_reachable"])
        joints = ik["joint_angles_deg"] if ok else []
        res = SeamSpinResult(
            spin_deg=0.0,
            cost=0.0 if ok else 1e9,
            rejected=not ok,
            reason="fixed_rpy" if ok else "ik_fixed_rpy_fail",
            candidate_count=1,
            feasible_count=1 if ok else 0,
        )
        return res, poses, joints

    base = preferred_spin_deg(wire, travel, bisector, step_deg=spin_step_deg)
    # Sample full circle, but evaluate base first for a good seed.
    candidates = list(np.arange(-180.0, 180.0, spin_step_deg))
    if base not in candidates:
        candidates.append(base)
    candidates = sorted(set(float(c) for c in candidates), key=lambda p: (abs(p - base), p))

    best: SeamSpinResult | None = None
    best_poses: list[dict] = []
    best_joints: list[list[float]] = []
    feasible = 0
    seed = list(q_seed_deg)

    for psi in candidates:
        ok, cost, reason, poses, joints = evaluate_spin_along_seam(
            weld_pts,
            approach,
            retract,
            wire,
            travel,
            psi,
            q_seed_deg=seed,
            q_ref_deg=q_ref_deg,
            mesh=mesh,
            move_type=move_type,
            ik_frame=ik_frame,
        )
        if ok:
            feasible += 1
            seed = joints[-1]  # warm-start next ψ from a reachable config
            if best is None or cost < best.cost:
                best = SeamSpinResult(
                    spin_deg=float(psi),
                    cost=cost,
                    rejected=False,
                    reason=reason,
                    candidate_count=len(candidates),
                    feasible_count=0,  # filled below
                )
                best_poses = poses
                best_joints = joints

    if best is None:
        # Fallback: preferred spin even if imperfect (keeps planner usable).
        psi = float(base)
        rpy = rpy_from_wire_spin(wire, travel, psi)
        poses = [
            _pose_dict(approach, rpy, mt="MOVJ", spin_deg=psi),
            *[_pose_dict(p, rpy, mt=move_type, spin_deg=psi) for p in weld_pts],
            _pose_dict(retract, rpy, mt="MOVJ", spin_deg=psi),
        ]
        ik = solve_ik_path(poses, q_seed_deg=q_seed_deg, frame=ik_frame)  # type: ignore[arg-type]
        joints = ik["joint_angles_deg"]
        best = SeamSpinResult(
            spin_deg=psi,
            cost=1e9,
            rejected=True,
            reason="no_feasible_spin",
            candidate_count=len(candidates),
            feasible_count=0,
        )
        return best, poses, joints

    best.feasible_count = feasible
    best.candidate_count = len(candidates)
    return best, best_poses, best_joints


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
    optimize_spin: bool = True,
    spin_step_deg: float = SPIN_STEP_DEG_DEFAULT,
    dense_step_m: float = DENSE_STEP_M_DEFAULT,
    mesh=None,
    q_home_deg: list[float] | None = None,
    ik_frame: str = "tool0",
    safe_travel: bool = True,
    rail_axis: int = 0,
) -> PlannedPath:
    """Build a weld path from seam segments (each with 'start'/'end' 3D lists).

    When a segment provides ``bisector`` / ``n1``+``n2``, approach and retract
    are offset along the bisector and torch RPY follows work + travel angles.
    Pass ``torch_rpy_deg`` to force a fixed orientation (legacy / override).

    With ``optimize_spin=True`` (default) the planner samples roll about the
    wire, rejects singular / colliding / unreachable spins, and densifies the
    weld to ≤ ``dense_step_m`` for Cartesian→joint continuity.

    With ``safe_travel=True`` (default) inter-seam stitching adds:
    * quantized BS1 ``BC#`` zones (carriage held during neighbouring welds),
    * clearance ``MOVJ`` vias when retract→approach crosses the part / faces,
    * 3-step EC safe-index when the positioner turns more than 5°.
    """
    dense_points: list[dict] = []
    dense_joints: list[list[float]] = []
    spin_angles: list[float] = []
    spin_results: list[SeamSpinResult] = []
    blocks: list[SeamBlock] = []

    if q_home_deg is None:
        q_home_deg = joint_pulses_to_deg(list(CELL_HOME_PULSES))
    q_ref = list(q_home_deg)
    q_seed = list(q_home_deg)
    worst_step = 0.0

    ppm = CELL_PULSE.base_pulses_per_mm
    # When lead is uncalibrated, map the batch's rail extent into 02025 zones.
    rail_span: tuple[float, float] | None = None
    if seams and ppm is None:
        mids = [float(seam_midpoint(s)[rail_axis]) for s in seams]
        lo_m, hi_m = min(mids), max(mids)
        # Keep a minimum span so a single-seam job still lands mid-zone.
        if hi_m - lo_m < 0.15:
            mid = 0.5 * (lo_m + hi_m)
            lo_m, hi_m = mid - 0.25, mid + 0.25
        rail_span = (lo_m, hi_m)
    ideal_bcs = [
        ideal_bc_for_seam(
            s, rail_axis=rail_axis, pulses_per_mm=ppm, rail_span_m=rail_span
        )
        for s in seams
    ]
    bc_zones = quantize_bc_zones(ideal_bcs) if seams else []

    for si, seam in enumerate(seams):
        is_arc = (
            str(seam.get("type", "")).lower() == "arc"
            or str(seam.get("move_type", "")).upper() == "MOVC"
        )
        move_type = "MOVC" if is_arc else "MOVL"
        if is_arc:
            coarse = _sample_arc_points(seam, max(3, samples_per_seam))
        else:
            start = np.asarray(seam["start"], dtype=float)
            end = np.asarray(seam["end"], dtype=float)
            n = max(2, samples_per_seam)
            coarse = [start + t * (end - start) for t in np.linspace(0.0, 1.0, n)]

        # Dense resampling for IK continuity / animation (≤ dense_step_m).
        weld_dense = _sample_polyline(coarse, dense_step_m)
        if is_arc and len(weld_dense) < 3:
            weld_dense = list(coarse)

        start = weld_dense[0]
        end = weld_dense[-1]
        travel = end - start
        if float(np.linalg.norm(travel)) < 1e-9 and len(weld_dense) >= 2:
            travel = weld_dense[1] - weld_dense[0]
        bis = seam_bisector(seam)
        wire = torch_wire_direction(
            bis, travel, travel_angle_deg=travel_angle_deg, push=push
        )

        offset = bis * float(approach_height)
        if float(np.linalg.norm(offset)) < 1e-9:
            offset = np.array([0.0, 0.0, approach_height])
        approach = start + offset
        retract = end + offset

        if optimize_spin and torch_rpy_deg is None:
            spin_res, dense_poses, joints = optimize_seam_spin(
                weld_dense,
                approach,
                retract,
                wire,
                travel,
                bis,
                q_seed_deg=q_seed,
                q_ref_deg=q_ref,
                mesh=mesh,
                spin_step_deg=spin_step_deg,
                move_type=move_type,
                ik_frame=ik_frame,
            )
        else:
            # Legacy fixed RPY (or disabled spin).
            if torch_rpy_deg is not None:
                rpy = torch_rpy_deg
                psi = 0.0
            else:
                psi = preferred_spin_deg(wire, travel, bis, step_deg=spin_step_deg)
                rpy = rpy_from_wire_spin(wire, travel, psi)
            dense_poses = [
                _pose_dict(approach, rpy, mt="MOVJ", spin_deg=psi),
                *[_pose_dict(p, rpy, mt=move_type, spin_deg=psi) for p in weld_dense],
                _pose_dict(retract, rpy, mt="MOVJ", spin_deg=psi),
            ]
            ik = solve_ik_path(dense_poses, q_seed_deg=q_seed, frame=ik_frame)  # type: ignore[arg-type]
            joints = ik["joint_angles_deg"]
            spin_res = SeamSpinResult(
                spin_deg=float(psi),
                cost=0.0,
                rejected=not ik["all_reachable"],
                reason="legacy" if ik["all_reachable"] else "ik_fail",
                candidate_count=1,
                feasible_count=1 if ik["all_reachable"] else 0,
            )

        spin_angles.append(spin_res.spin_deg)
        spin_results.append(spin_res)

        # Sparse JBI path for this seam: approach + coarse weld + retract.
        rpy_s = (
            dense_poses[1]["rx"],
            dense_poses[1]["ry"],
            dense_poses[1]["rz"],
        )
        psi = spin_res.spin_deg
        local: list[dict] = [_pose_dict(approach, rpy_s, mt="MOVJ", spin_deg=psi)]
        weld_start = 1
        for p in coarse:
            local.append(_pose_dict(p, rpy_s, mt=move_type, spin_deg=psi))
        weld_end = len(local) - 1
        local.append(_pose_dict(retract, rpy_s, mt="MOVJ", spin_deg=psi))

        bc = int(seam.get("bc", bc_zones[si] if si < len(bc_zones) else ideal_bcs[si]))
        ec = resolve_seam_ec(seam)
        blocks.append(
            SeamBlock(
                points=local,
                weld_start=weld_start,
                weld_end=weld_end,
                bisector=bis,
                mid=seam_midpoint(seam),
                spin_deg=float(psi),
                dense_points=list(dense_poses),
                dense_joints=list(joints),
                bc=bc,
                ec=ec,
                move_type=move_type,
                weld_speed=float(seam.get("weld_speed", weld_speed)),
                arc_file=int(seam.get("arc_file", arc_file)),
                weave_no=int(seam.get("weave_no", 21)),
            )
        )

        dense_points.extend(dense_poses)
        dense_joints.extend(joints)
        if len(joints) >= 2:
            worst_step = max(worst_step, _max_joint_step(joints[1:-1]))

        if joints:
            q_seed = list(joints[-1])
            q_ref = list(joints[-1])

    if safe_travel and blocks:
        part_pts: list[np.ndarray] = [b.mid for b in blocks]
        for b in blocks:
            part_pts.append(_pose_xyz(b.points[0]))
            part_pts.append(_pose_xyz(b.points[-1]))
        points, weld_meta = stitch_seam_blocks(
            blocks, mesh=mesh, part_points=part_pts
        )
        weld_segments = [
            WeldSegment(
                start_index=ws,
                end_index=we,
                weld_speed=spd,
                arc_file=af,
                move_type=mt,
                weave_no=wv,
            )
            for ws, we, spd, af, mt, wv in weld_meta
        ]
    else:
        # Legacy concatenation without inter-seam vias.
        from .travel_planning import annotate_seam_speeds

        points = []
        weld_segments = []
        for block in blocks:
            annotate_seam_speeds(block)
            base = len(points)
            points.extend(block.points)
            weld_segments.append(
                WeldSegment(
                    start_index=base + block.weld_start,
                    end_index=base + block.weld_end,
                    weld_speed=block.weld_speed,
                    arc_file=block.arc_file,
                    move_type=block.move_type,
                    weave_no=block.weave_no,
                )
            )

    return PlannedPath(
        points=points,
        weld_segments=weld_segments,
        dense_points=dense_points,
        dense_joint_angles_deg=dense_joints,
        spin_angles_deg=spin_angles,
        spin_results=spin_results,
        max_joint_step_deg=worst_step,
        bc_zones=bc_zones,
    )


def _pose_xyz(p: dict) -> np.ndarray:
    return np.array([float(p["x"]), float(p["y"]), float(p["z"])], dtype=float)


# Re-export for callers / tests that build stick-out geometry.
__all__ = [
    "PlannedPath",
    "SeamSpinResult",
    "plan_from_seams",
    "seam_bisector",
    "torch_wire_direction",
    "torch_dir_to_rpy_deg",
    "rpy_from_wire_spin",
    "preferred_spin_deg",
    "optimize_seam_spin",
    "DENSE_STEP_M_DEFAULT",
    "SPIN_STEP_DEG_DEFAULT",
    "TOOL0_XYZ_M",
    "ideal_bc_for_seam",
    "quantize_bc_zones",
]
