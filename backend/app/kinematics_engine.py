"""
Kinematics engine for a 6-DOF Yaskawa-class manipulator.

Built on ``roboticstoolbox-python``. Wraps a DHRobot from
``robot_config.ACTIVE_MODEL`` and exposes FK / IK with the MotoROS2 frame
chain (see ``Yaskawa-Global/motoros2`` ``doc/ros_api.md``):

* ``flange`` — mechanical interface / ROS-Industrial REP-103 (X+ out of face)
* ``tool0``  — Yaskawa all-zeros toolframe (Z+ out of face); static ``flange→tool0``
* ``tcp`` / ``tcp_0`` — active Tool No. 0 from ``TOOL.CND``, relative to ``tool0``

Read the safety notice in ``robot_config.py`` before trusting any output.
"""

from __future__ import annotations

from typing import Literal

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

from .robot_config import (
    ACTIVE_MODEL,
    AXIS_NAMES,
    TOOL0_RPY_DEG,
    TOOL0_XYZ_MM,
    RobotModel,
)

FrameName = Literal["flange", "tool0", "tcp"]


def build_robot(model: RobotModel = ACTIVE_MODEL) -> rtb.DHRobot:
    """Construct a roboticstoolbox DHRobot from a RobotModel definition."""
    links = [
        rtb.RevoluteDH(a=j.a, alpha=j.alpha, d=j.d, offset=j.offset, qlim=list(j.qlim))
        for j in model.joints
    ]
    return rtb.DHRobot(links, name=model.name)


_ROBOT = build_robot()


def get_robot() -> rtb.DHRobot:
    return _ROBOT


def flange_to_tool0_se3() -> SE3:
    """Static ROS-Industrial ``flange → tool0`` (Motoman / MotoROS2).

    Matches AR2010 URDF ``flange-tool0`` (columns = tool0 basis in flange):

    * Motoman +Z (out of flange) = flange +X (REP-103 forward)
    * Motoman +X = flange +Z (up)
    * Motoman +Y = flange −Y
    """
    # Explicit matrix — avoids Euler-order ambiguity across libraries.
    return SE3(
        np.array(
            [
                [0.0, 0.0, 1.0, 0.0],
                [0.0, -1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
    )


def tool_file_se3() -> SE3:
    """Yaskawa Tool No. 0 from ``TOOL.CND``, expressed in Motoman ``tool0``.

    Translation metres; RPY degrees as Motoman Rx,Ry,Rz. Order matches the
    frontend ``TOOL_RPY_ORDER = ZYX`` (Three.js Euler → R = Rx·Ry·Rz ≡
    spatialmath ``order='xyz'``).
    """
    xyz_m = [v / 1000.0 for v in TOOL0_XYZ_MM]
    return SE3(xyz_m[0], xyz_m[1], xyz_m[2]) * SE3.RPY(
        list(TOOL0_RPY_DEG), unit="deg", order="xyz"
    )


def tool0_se3() -> SE3:
    """Legacy name: full ``flange → tcp_0`` = ``flange→tool0`` · ``TOOL.CND``.

    Prefer ``flange_to_tool0_se3()`` / ``tool_file_se3()`` for clarity.
    """
    return flange_to_tool0_se3() * tool_file_se3()


def _pose_to_se3(pose: dict) -> SE3:
    """Convert a {x,y,z,rx,ry,rz} pose into an SE3.

    Translation metres, rotation degrees, intrinsic RPY XYZ.
    """
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


def _fk_tip_se3(q_deg: list[float]) -> SE3:
    """Raw toolbox FK tip (orientation depends on the active chain)."""
    q = np.deg2rad(np.asarray(q_deg, dtype=float))
    return _ROBOT.fkine(q)


def _tip_is_motoman_z() -> bool:
    """Catalogue DH tips use Motoman Z+ along the approach.

    ROS-Industrial MA2010/AR2010 URDF tips use REP-103 flange (X+ approach).
    Placeholder DH models set ``approximate=True`` and keep Motoman tip axes.
    """
    return bool(getattr(ACTIVE_MODEL, "approximate", True))


def flange_se3(q_deg: list[float]) -> SE3:
    """REP-103 flange pose (X+ out of face) from joint angles (degrees)."""
    tip = _fk_tip_se3(q_deg)
    if _tip_is_motoman_z():
        # DH tip ≡ Motoman tool0 → recover REP-103 flange.
        return tip * flange_to_tool0_se3().inv()
    return tip


def motoman_tool0_se3(q_deg: list[float]) -> SE3:
    """Yaskawa ``tool0`` (Z+ out of face)."""
    tip = _fk_tip_se3(q_deg)
    if _tip_is_motoman_z():
        return tip
    return tip * flange_to_tool0_se3()


def tcp_se3(q_deg: list[float]) -> SE3:
    """Active ``tcp_0`` = Motoman tool0 · TOOL.CND."""
    return motoman_tool0_se3(q_deg) * tool_file_se3()


def forward_kinematics(
    q_deg: list[float],
    frame: FrameName = "tcp",
) -> dict:
    """FK: joint angles (deg) → pose.

    ``frame="flange"`` — REP-103 mechanical interface (X+ out).
    ``frame="tool0"`` — Motoman all-zeros toolframe (Z+ out).
    ``frame="tcp"`` (default) — Tool No. 0 tip from ``TOOL.CND``.
    """
    if frame == "flange":
        T = flange_se3(q_deg)
    elif frame == "tool0":
        T = motoman_tool0_se3(q_deg)
    else:
        T = tcp_se3(q_deg)
    return _se3_to_pose(T)


def solve_ik_path(
    points: list[dict],
    q_seed_deg: list[float] | None = None,
    frame: FrameName = "tcp",
) -> dict:
    """Solve IK for an ordered list of poses.

    Default ``frame="tcp"`` matches ``forward_kinematics`` (Tool No. 0 /
    ``TOOL.CND``). Path-planner samples that omit the taught tool RPY should
    pass ``frame="tool0"``. Each solution seeds the next call for continuity.
    """
    robot = _ROBOT
    n = robot.n

    if q_seed_deg is not None:
        q_seed = np.deg2rad(np.asarray(q_seed_deg, dtype=float))
    else:
        q_seed = np.array(
            [0.5 * (j.qlim[0] + j.qlim[1]) for j in ACTIVE_MODEL.joints]
        )

    # Map requested frame → toolbox tip.
    if _tip_is_motoman_z():
        # tip = tool0; tip→tcp = TOOL.CND; tip→flange = inv(flange→tool0)
        tip_from_flange = flange_to_tool0_se3().inv()
        tip_from_tool0 = SE3()
        tip_from_tcp = tool_file_se3().inv()
    else:
        # tip = flange (REP-103 URDF)
        tip_from_flange = SE3()
        tip_from_tool0 = flange_to_tool0_se3().inv()
        tip_from_tcp = tool0_se3().inv()

    solutions: list[list[float]] = []
    diagnostics: list[dict] = []
    q_prev = q_seed

    for idx, p in enumerate(points):
        T_target = _pose_to_se3(p)
        if frame == "tcp":
            T_tip = T_target * tip_from_tcp
        elif frame == "tool0":
            T_tip = T_target * tip_from_tool0
        else:
            T_tip = T_target * tip_from_flange
        sol = robot.ikine_LM(
            T_tip, q0=q_prev, ilimit=200, slimit=50, joint_limits=True
        )
        reached = bool(sol.success)
        q_prev = sol.q if reached else q_prev
        solutions.append([float(a) for a in np.rad2deg(sol.q)])
        diagnostics.append(
            {
                "index": idx,
                "success": reached,
                "iterations": int(getattr(sol, "iterations", 0)),
                "residual": float(getattr(sol, "residual", 0.0) or 0.0),
            }
        )

    all_ok = all(d["success"] for d in diagnostics)
    return {
        "axis_names": AXIS_NAMES,
        "joint_angles_deg": solutions,
        "diagnostics": diagnostics,
        "all_reachable": all_ok,
        "frame": frame,
        "n_joints": n,
    }
