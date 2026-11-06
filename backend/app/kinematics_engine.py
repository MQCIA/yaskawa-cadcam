"""
Kinematics engine for a 6-DOF Yaskawa-class manipulator.

MA2010 uses an ``ERobot`` whose elementary transforms match the ROS-Industrial
URDF (``frontend/public/models/ma2010/ma2010.urdf``): same origins, same axis
directions (including the Motoman sign flips on U/R/B/T). Other catalogue
models keep a classic DH chain.

TCP poses:

* ``flange`` — URDF ``tool0`` (REP-103, X+ out of flange face)
* ``tool0``  — Motoman Z+ out (``FLANGE_TO_TOOL0_R`` only; no TOOL.CND)
* ``tcp``    — URDF flange × TOOL.CND (dump tip). Motoman R×TOOL.CND is
  available via ``motoman_tcp_se3`` for experiments; default ``tcp`` matches
  the URDF bake used in compare/golden until pendant XYZ confirms which
  convention matches the cell.

Link lengths match the URDF. That is **not** the same as “certified on the
physical DX200” — until ``golden_poses.json`` has ``verified_on_pendant: true``,
treat Cartesian results as APP_ESTIMATE only.

Read the safety notice in ``robot_config.py`` before trusting any output.
"""

from __future__ import annotations

from typing import Literal

import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import ET
from spatialmath import SE3

from .robot_config import (
    ACTIVE_MODEL,
    AXIS_NAMES,
    FLANGE_TO_TOOL0_R,
    MA2010_URDF,
    TOOL0_RPY_DEG,
    TOOL0_XYZ_M,
    RobotModel,
    cell_home_deg,
)

FrameName = Literal["flange", "tool0", "tcp"]

# Wrist axes (R,B,T) — reject LM solutions that flip > this vs the seed.
_WRIST_FLIP_DEG = 90.0


def flange_to_tool0_se3() -> SE3:
    """Static Motoman ``T_flange_to_tool0`` (REP-103 flange → Motoman Z+ out)."""
    R = np.array(FLANGE_TO_TOOL0_R, dtype=float)
    T = np.eye(4)
    T[:3, :3] = R
    return SE3(T)


def tool_file_se3() -> SE3:
    """``T_tool0_to_tcp0`` from TOOL.CND (metres + Motoman Rx,Ry,Rz)."""
    return SE3(*TOOL0_XYZ_M) * SE3.RPY(
        list(TOOL0_RPY_DEG), unit="deg", order="xyz"
    )


def tool0_se3() -> SE3:
    """Default APP_ESTIMATE tip: TOOL.CND applied in the URDF flange frame.

    Pendant/MotoSim may prefer Motoman Z-out composition — see
    ``motoman_tcp_se3``. Do not treat either as DX200-certified until golden
    poses are ``verified_on_pendant``.
    """
    return tool_file_se3()


def motoman_tcp_se3() -> SE3:
    """Experimental ``flange → Motoman tool0 → TOOL.CND`` (MotoROS2 order)."""
    return flange_to_tool0_se3() * tool_file_se3()


def build_ma2010_erobot(model: RobotModel) -> rtb.ERobot:
    """URDF-faithful elementary-transform chain for MA2010.

    Axes (from URDF)::

        S  +Z   L  +Y   U  -Y   R  -X   B  -Y   T  -X
        then fixed Tx(d6) to the flange (URDF tool0).
    """
    g = MA2010_URDF
    ets = (
        ET.tz(g.d1)
        * ET.Rz()
        * ET.tx(g.a1)
        * ET.Ry()
        * ET.tz(g.a2)
        * ET.Ry(flip=True)
        * ET.tz(g.a3)
        * ET.Rx(flip=True)
        * ET.tx(g.d4)
        * ET.Ry(flip=True)
        * ET.Rx(flip=True)
        * ET.tx(g.d6)
    )
    robot = rtb.ERobot(ets, name=model.name)
    lo = [j.qlim[0] for j in model.joints]
    hi = [j.qlim[1] for j in model.joints]
    robot.qlim = np.array([lo, hi], dtype=float)
    return robot


def urdf_flange_se3_reference(q_deg: list[float]) -> SE3:
    """Independent SE3 product matching the URDF (cross-check vs ERobot)."""
    g = MA2010_URDF
    q = np.deg2rad(np.asarray(q_deg, dtype=float))
    T = SE3()
    T = T * SE3.Tz(g.d1) * SE3.Rz(q[0])
    T = T * SE3.Tx(g.a1) * SE3.Ry(q[1])
    T = T * SE3.Tz(g.a2) * SE3.Ry(-q[2])
    T = T * SE3.Tz(g.a3) * SE3.Rx(-q[3])
    T = T * SE3.Tx(g.d4) * SE3.Ry(-q[4])
    T = T * SE3.Rx(-q[5])
    T = T * SE3.Tx(g.d6)
    return T


def build_dh_robot(model: RobotModel) -> rtb.DHRobot:
    """Classic DH robot for AR-series catalogue placeholders."""
    links = [
        rtb.RevoluteDH(a=j.a, alpha=j.alpha, d=j.d, offset=j.offset, qlim=list(j.qlim))
        for j in model.joints
    ]
    return rtb.DHRobot(links, name=model.name)


def build_robot(model: RobotModel = ACTIVE_MODEL):
    """Return an ERobot (MA2010) or DHRobot (catalogue AR models)."""
    if model.urdf_chain:
        return build_ma2010_erobot(model)
    return build_dh_robot(model)


_ROBOT = build_robot()


def get_robot():
    return _ROBOT


def _pose_to_se3(pose: dict) -> SE3:
    """Convert a {x,y,z,rx,ry,rz} pose into an SE3 (metres, deg, RPY XYZ)."""
    T = SE3(pose["x"], pose["y"], pose["z"])
    T = T * SE3.RPY(
        [pose.get("rx", 0.0), pose.get("ry", 0.0), pose.get("rz", 0.0)],
        unit="deg",
        order="xyz",
    )
    return T


def _se3_to_pose(T: SE3) -> dict:
    rpy = T.rpy(unit="deg", order="xyz")
    t = T.t
    return {
        "x": float(t[0]),
        "y": float(t[1]),
        "z": float(t[2]),
        "rx": float(rpy[0]),
        "ry": float(rpy[1]),
        "rz": float(rpy[2]),
    }


def flange_se3(q_deg: list[float]) -> SE3:
    """Flange pose (URDF tool0) for joint angles in degrees."""
    q = np.deg2rad(np.asarray(q_deg, dtype=float))
    return _ROBOT.fkine(q)


def tcp_se3(q_deg: list[float]) -> SE3:
    """Torch TCP (default APP_ESTIMATE) = flange * TOOL.CND."""
    return flange_se3(q_deg) * tool0_se3()


def forward_kinematics(
    q_deg: list[float],
    frame: FrameName = "tcp",
) -> dict:
    """FK: joint angles (deg) → pose.

    ``frame="flange"`` — URDF tool0.
    ``frame="tool0"`` — Motoman tool0 (Z+ out).
    ``frame="tcp"`` (default) — TOOL.CND on URDF flange.
    """
    T_flange = flange_se3(q_deg)
    if frame == "flange":
        T = T_flange
    elif frame == "tool0":
        T = T_flange * flange_to_tool0_se3()
    else:
        T = T_flange * tool0_se3()
    return _se3_to_pose(T)


def _default_seed_deg() -> list[float]:
    if ACTIVE_MODEL.home_deg is not None:
        return list(ACTIVE_MODEL.home_deg)
    return cell_home_deg()


def _wrist_flipped(q_new: np.ndarray, q_seed: np.ndarray) -> bool:
    delta = np.abs(np.rad2deg(q_new - q_seed))
    return bool(np.any(delta[3:6] > _WRIST_FLIP_DEG))


def solve_ik_path(
    points: list[dict],
    q_seed_deg: list[float] | None = None,
    frame: FrameName = "tcp",
) -> dict:
    """Solve IK for an ordered list of poses.

    Poses are interpreted as ``frame`` (default torch TCP). The first seed is
    the cell home (taught C00000 pulses) when none is given; each later point
    seeds from the previous *accepted* solution. Wrist flips (>90° on R/B/T
    vs the seed) are rejected so straight weld segments stay continuous.
    """
    robot = _ROBOT

    if q_seed_deg is not None:
        q_seed = np.deg2rad(np.asarray(q_seed_deg, dtype=float))
    else:
        q_seed = np.deg2rad(np.asarray(_default_seed_deg(), dtype=float))

    tool = tool0_se3()
    flange_to_tool0 = flange_to_tool0_se3()
    solutions: list[list[float]] = []
    diagnostics: list[dict] = []
    q_prev = q_seed

    mid = np.array(
        [0.5 * (j.qlim[0] + j.qlim[1]) for j in ACTIVE_MODEL.joints],
        dtype=float,
    )

    for idx, p in enumerate(points):
        T_target = _pose_to_se3(p)
        if frame == "tcp":
            T_flange = T_target * tool.inv()
        elif frame == "tool0":
            T_flange = T_target * flange_to_tool0.inv()
        else:
            T_flange = T_target

        sol = robot.ikine_LM(
            T_flange, q0=q_prev, ilimit=300, slimit=80, joint_limits=True
        )
        reached = bool(sol.success)
        flipped = False
        # Continuity: reject wrist flips only after the first point (weld chords).
        if reached and idx > 0 and _wrist_flipped(sol.q, q_prev):
            flipped = True
            reached = False
        # First point: if home seed is a bad basin, retry mid-range once.
        if not reached and idx == 0:
            sol2 = robot.ikine_LM(
                T_flange, q0=mid, ilimit=300, slimit=80, joint_limits=True
            )
            if sol2.success:
                sol = sol2
                reached = True
                flipped = False

        if reached:
            q_prev = sol.q
            q_out = sol.q
        else:
            q_out = q_prev
        solutions.append([float(a) for a in np.rad2deg(q_out)])
        residual = float(getattr(sol, "residual", 0.0) or 0.0)
        diagnostics.append(
            {
                "index": idx,
                "success": reached,
                "wrist_flip_rejected": flipped,
                "iterations": int(getattr(sol, "iterations", 0)),
                "residual": residual,
            }
        )

    all_ok = all(d["success"] for d in diagnostics)
    return {
        "axis_names": AXIS_NAMES,
        "joint_angles_deg": solutions,
        "diagnostics": diagnostics,
        "all_reachable": all_ok,
        "frame": frame,
        "seed_deg": [float(a) for a in np.rad2deg(q_seed)],
    }


# Alias used by some call sites / docs.
calculate_ik = solve_ik_path
