"""
Kinematics engine for a 6-DOF Yaskawa-class manipulator.

MA2010 uses an ``ERobot`` whose elementary transforms match the ROS-Industrial
URDF (``frontend/public/models/ma2010/ma2010.urdf``): same origins, same axis
directions (including the Motoman sign flips on U/R/B/T). Other catalogue
models keep a classic DH chain.

TCP poses apply the cell TOOL 0 frame from ``TOOL.CND``. Callers can ask for
the flange (URDF ``tool0``) or the torch tip (``tcp``).

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
    MA2010_URDF,
    TOOL0_RPY_DEG,
    TOOL0_XYZ_MM,
    RobotModel,
    cell_home_deg,
)

FrameName = Literal["flange", "tcp"]


def tool0_se3() -> SE3:
    """Cell TOOL 0 relative to the flange (URDF tool0). Metres + deg RPY XYZ."""
    xyz_m = [v / 1000.0 for v in TOOL0_XYZ_MM]
    return SE3(xyz_m[0], xyz_m[1], xyz_m[2]) * SE3.RPY(
        list(TOOL0_RPY_DEG), unit="deg", order="xyz"
    )


def build_ma2010_erobot(model: RobotModel) -> rtb.ERobot:
    """URDF-faithful elementary-transform chain for MA2010.

    Axes (from URDF)::

        S  +Z   L  +Y   U  -Y   R  -X   B  -Y   T  -X
        then fixed Tx(d6) to the flange (tool0).
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
    """Convert a {x,y,z,rx,ry,rz} pose into an SE3.

    Translation is in metres, rotation in degrees, intrinsic RPY XYZ.
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


def flange_se3(q_deg: list[float]) -> SE3:
    """Flange pose (URDF tool0) for joint angles in degrees."""
    q = np.deg2rad(np.asarray(q_deg, dtype=float))
    return _ROBOT.fkine(q)


def tcp_se3(q_deg: list[float]) -> SE3:
    """Torch TCP = flange * TOOL 0."""
    return flange_se3(q_deg) * tool0_se3()


def forward_kinematics(
    q_deg: list[float],
    frame: FrameName = "tcp",
) -> dict:
    """FK: joint angles (deg) → pose.

    ``frame="flange"`` is URDF tool0 (mechanical interface).
    ``frame="tcp"`` (default) multiplies cell TOOL 0 onto the flange.
    """
    T = flange_se3(q_deg) if frame == "flange" else tcp_se3(q_deg)
    return _se3_to_pose(T)


def _default_seed_deg() -> list[float]:
    if ACTIVE_MODEL.home_deg is not None:
        return list(ACTIVE_MODEL.home_deg)
    return cell_home_deg()


def solve_ik_path(
    points: list[dict],
    q_seed_deg: list[float] | None = None,
    frame: FrameName = "tcp",
) -> dict:
    """Solve IK for an ordered list of poses.

    Poses are interpreted as ``frame`` (default torch TCP). The first seed is
    the cell home when none is given; each later point seeds from the previous
    solution. Joint limits from the active model are enforced.
    """
    robot = _ROBOT

    if q_seed_deg is not None:
        q_seed = np.deg2rad(np.asarray(q_seed_deg, dtype=float))
    else:
        q_seed = np.deg2rad(np.asarray(_default_seed_deg(), dtype=float))

    tool = tool0_se3()
    solutions: list[list[float]] = []
    diagnostics: list[dict] = []
    q_prev = q_seed

    for idx, p in enumerate(points):
        T_target = _pose_to_se3(p)
        if frame == "tcp":
            T_flange = T_target * tool.inv()
        else:
            T_flange = T_target
        sol = robot.ikine_LM(
            T_flange, q0=q_prev, ilimit=200, slimit=50, joint_limits=True
        )
        reached = bool(sol.success)
        q_prev = sol.q if reached else q_prev
        solutions.append([float(a) for a in np.rad2deg(sol.q)])
        residual = float(getattr(sol, "residual", 0.0) or 0.0)
        diagnostics.append(
            {
                "index": idx,
                "success": reached,
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
