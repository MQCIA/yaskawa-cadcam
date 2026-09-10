"""
Kinematics engine for a 6-DOF Yaskawa-class manipulator.

Built on `roboticstoolbox-python`. This wraps a DHRobot built from
``robot_config.ACTIVE_MODEL`` and exposes forward/inverse kinematics plus a
path solver that keeps solutions continuous (seeds each IK call with the
previous joint solution to avoid configuration jumps).

Read the safety notice in ``robot_config.py`` before trusting any output.
"""

from __future__ import annotations

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

from .robot_config import ACTIVE_MODEL, AXIS_NAMES, RobotModel


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


def _pose_to_se3(pose: dict) -> SE3:
    """Convert a {x,y,z,rx,ry,rz} TCP pose into an SE3.

    Translation is in metres, rotation in degrees, interpreted as an
    intrinsic RPY (roll about X, pitch about Y, yaw about Z). Adjust the
    convention to match how your CAD/teach data is expressed.
    """
    T = SE3(pose["x"], pose["y"], pose["z"])
    T = T * SE3.RPY(
        [pose.get("rx", 0.0), pose.get("ry", 0.0), pose.get("rz", 0.0)],
        unit="deg",
        order="xyz",
    )
    return T


def forward_kinematics(q_deg: list[float]) -> dict:
    """FK: joint angles (deg) -> TCP pose."""
    q = np.deg2rad(np.asarray(q_deg, dtype=float))
    T = _ROBOT.fkine(q)
    rpy = T.rpy(unit="deg", order="xyz")
    t = T.t
    return {
        "x": float(t[0]), "y": float(t[1]), "z": float(t[2]),
        "rx": float(rpy[0]), "ry": float(rpy[1]), "rz": float(rpy[2]),
    }


def solve_ik_path(
    points: list[dict],
    q_seed_deg: list[float] | None = None,
) -> dict:
    """Solve IK for an ordered list of TCP poses.

    Each solution seeds the next call so the arm follows a continuous
    configuration. Returns per-point joint angles (deg) and a success flag.
    """
    robot = _ROBOT
    n = robot.n

    if q_seed_deg is not None:
        q_seed = np.deg2rad(np.asarray(q_seed_deg, dtype=float))
    else:
        # Mid-range seed keeps the numeric solver away from limits.
        q_seed = np.array(
            [0.5 * (j.qlim[0] + j.qlim[1]) for j in ACTIVE_MODEL.joints]
        )

    solutions: list[list[float]] = []
    diagnostics: list[dict] = []
    q_prev = q_seed

    for idx, p in enumerate(points):
        T = _pose_to_se3(p)
        # Levenberg-Marquardt numeric IK, seeded with the previous solution.
        sol = robot.ikine_LM(T, q0=q_prev, ilimit=200, slimit=50, joint_limits=True)
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
    }
