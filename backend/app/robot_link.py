"""How this cell is actually driven — INFORM .JBI vs optional MotoROS2.

Primary execution path (ArcNC / this CAD-CAM app):
  native DX200 INFORM ``.JBI`` with Motoweld ``MACRO1`` / ``WVON`` / ``ARCON``.

Optional live link (Yaskawa, not ArcNC):
  MotoROS2 v0.2.1 on DX200 — see ``Yaskawa-Global/motoros2`` ``README.md`` and
  ``doc/ros_api.md``. Suitable for joint telemetry, TF, and air moves after
  ``start_traj_mode`` / ``INIT_ROS``. Welding cycles stay in ``.JBI``.

Set ``MOTOROS_AGENT_HOST`` (+ optional ``MOTOROS_AGENT_PORT``, default 8888)
to probe a micro-ROS Agent. This module never commands motion.
"""

from __future__ import annotations

import os
import re
import socket
from typing import Any


# ---------------------------------------------------------------------------
# Cell dump facts (reference/robot-cell-dx200 / SYSTEM.SYS)
# ---------------------------------------------------------------------------
CELL_FIRMWARE = "DN2.27.00A(US/PL)-00"
MOTOROS2_MIN_FIRMWARE = "DN2.44.00-00"
MOTOROS2_VERSION = "0.2.1"

MOTOROS2_BINARIES = {
    "humble": {
        "file": "mr2_dx2_h.out",
        "ros_distro": "Humble",
        "md5": "6672e218b60c42f00ce335dc4a977ee9",
    },
    "jazzy": {
        "file": "mr2_dx2_j.out",
        "ros_distro": "Jazzy",
        "md5": "f31673a8559e331719f8b25f174419a3",
    },
}

# README.md — PARAMETER → S2C (DX200)
S2C_REQUIRED: dict[str, int] = {
    "S2C541": 0,
    "S2C542": 0,
    "S2C1102": 2,
    "S2C1104": 2,
    "S2C1117": 1,  # DX200 Speed FB — alarm 8001[10] then reboot
    "S2C1250": 1,
    "S2C1402": 3,
}

# INFORM group ↔ MotoROS2 ctrl_groups (this multi-group cell)
GROUP_MAP = [
    {
        "inform": "RB1",
        "yaskawa": "R1",
        "ros_group": "r1",
        "topic": "ctrl_groups/r1/joint_states",
        "unit": "rad",
        "role": "MA2010 manipulator",
    },
    {
        "inform": "BS1",
        "yaskawa": "B1",
        "ros_group": "b1",
        "topic": "ctrl_groups/b1/joint_states",
        "unit": "m",
        "role": "RECT-X linear base",
    },
    {
        "inform": "ST1",
        "yaskawa": "S1",
        "ros_group": "s1",
        "topic": "ctrl_groups/s1/joint_states",
        "unit": "rad",
        "role": "TURN station 1",
    },
    {
        "inform": "ST2",
        "yaskawa": "S2",
        "ros_group": "s2",
        "topic": "ctrl_groups/s2/joint_states",
        "unit": "rad",
        "role": "TURN station 2",
    },
    {
        "inform": "ST3",
        "yaskawa": "S3",
        "ros_group": "s3",
        "topic": "ctrl_groups/s3/joint_states",
        "unit": "rad",
        "role": "TURN station 3",
    },
    {
        "inform": "ST4",
        "yaskawa": "S4",
        "ros_group": "s4",
        "topic": "ctrl_groups/s4/joint_states",
        "unit": "rad",
        "role": "TURN station 4",
    },
]

# Common FollowJointTrajectory result fragments (motoros2_interfaces MotionReadyEnum)
_MOTOROS2_RESULT_HINTS: dict[int, str] = {
    0: "OK / ready",
    101: "active controller alarm / not ready for motion",
    102: "not in remote / trajectory mode",
    103: "servo power off",
    104: "HOLD / e-stop active",
    105: "invalid goal / trajectory rejected",
}


def decode_motoros2_error(error_code: int) -> dict[str, Any]:
    """Decode MotoROS2 ``FollowJointTrajectory`` ``error_code`` (−ECCCCC).

    MotoROS2 extends the ROS action result: values look like ``-ECCCCC`` where
    ``E`` is the ROS ``FollowJointTrajectory`` error digit and ``CCCCC`` is a
    MotoROS2 ``MotionReadyEnum``-style code.

    Example: ``-100101`` → ROS ``-1`` (invalid goal), MotoROS2 ``101``
    (active controller alarm).
    """
    raw = int(error_code)
    ros_error: int | None = None
    motoros_code: int | None = None
    parsed = False

    if raw < 0:
        digits = str(abs(raw))
        # Prefer 1 + 5 split when |code| has ≥6 digits (−100101).
        if len(digits) >= 6:
            ros_error = -int(digits[0])
            motoros_code = int(digits[1:])
            parsed = True
        elif len(digits) >= 2:
            # Fallback: last 5 digits Motoman, leading digit ROS.
            motoros_code = int(digits[-5:]) if len(digits) > 5 else int(digits[1:])
            ros_error = -int(digits[0])
            parsed = True
    elif raw == 0:
        ros_error = 0
        motoros_code = 0
        parsed = True

    ros_names = {
        0: "SUCCESSFUL",
        -1: "INVALID_GOAL",
        -2: "INVALID_JOINTS",
        -3: "OLD_HEADER_TIMESTAMP",
        -4: "PATH_TOLERANCE_VIOLATED",
        -5: "GOAL_TOLERANCE_VIOLATED",
    }
    hint = _MOTOROS2_RESULT_HINTS.get(motoros_code or -1, "see MotionReadyEnum / robot_status")
    return {
        "raw": raw,
        "parsed": parsed,
        "format": "-ECCCCC",
        "ros_error": ros_error,
        "ros_error_name": ros_names.get(ros_error) if ros_error is not None else None,
        "motoros2_code": motoros_code,
        "motoros2_hint": hint if motoros_code is not None else None,
        "example": "-100101 → ros=-1 (INVALID_GOAL), motoros2=101 (active alarm)",
    }


def firmware_compatible(cell_fw: str = CELL_FIRMWARE, minimum: str = MOTOROS2_MIN_FIRMWARE) -> dict[str, Any]:
    """Compare dump firmware string against MotoROS2 minimum (DN2.44.00-00)."""
    def _nums(s: str) -> tuple[int, ...]:
        m = re.search(r"DN(\d+)\.(\d+)\.(\d+)", s, re.I)
        if not m:
            return (0, 0, 0)
        return tuple(int(x) for x in m.groups())

    cell = _nums(cell_fw)
    need = _nums(minimum)
    ok = cell >= need
    return {
        "cell_firmware": cell_fw,
        "motoros2_minimum": minimum,
        "compatible": ok,
        "action": (
            None
            if ok
            else (
                f"Upgrade DX200 system software from {cell_fw} to {minimum} "
                "or newer before installing MotoROS2."
            )
        ),
    }


def describe_robot_link() -> dict[str, Any]:
    host = os.environ.get("MOTOROS_AGENT_HOST", "").strip()
    port_raw = os.environ.get("MOTOROS_AGENT_PORT", "8888").strip() or "8888"
    try:
        port = int(port_raw)
    except ValueError:
        port = 8888

    connected = False
    detail = "not configured"
    if host:
        connected, detail = _tcp_open(host, port)

    fw = firmware_compatible()

    return {
        "execution": "inform_jbi",
        "controller": "DX200",
        "robot": "MA2010",
        "arcnc": {
            "public_api": False,
            "model": "offline_program",
            "note": (
                "ArcNC has no public robot API. It downloads a native job "
                "the controller already understands."
            ),
        },
        "native_program": {
            "language": "INFORM",
            "extension": ".JBI",
            "groups": ["RB1", "BS1", "ST1", "ST2", "ST3", "ST4"],
            "arc": "ARCON/ARCOF + MACRO1 ARGF + WVON/WVOF (Motoweld)",
            "note": (
                "Spawanie z ARCON / WVON / MACRO1 zawsze idzie przez natywny "
                "plik .JBI — nie przez FollowJointTrajectory."
            ),
        },
        "group_map": GROUP_MAP,
        "ros": {
            "recommended": "MotoROS2",
            "package": "Yaskawa-Global/motoros2",
            "version": MOTOROS2_VERSION,
            "docs": ["README.md", "doc/ros_api.md"],
            "agent": "micro-ROS Agent",
            "rmw": "FastDDS only (Galactic must force FastDDS)",
            "firmware": fw,
            "binaries": MOTOROS2_BINARIES,
            "hardware": {
                "usb_config_port": "CN106",
                "usb_config_file": "motoros2_config.yaml",
                "usb_must_remain": True,
                "missing_cfg_alarm": "8013 [0] Missing MotoROS2 cfg file",
                "speed_fb_alarm": "8001[10] Speed FB enabled, reboot now",
            },
            "maintenance": {
                "motoplus_func": "USED",
                "motoman_driver": "USED",
                "incompatible_apps": ["Simple Connect"],
                "fsu_crc_check": (
                    "Temporarily set SAVE DATA CRC CHECK FUNC (FSU) = INVALID "
                    "during install; restore VALID afterwards"
                ),
            },
            "s2c_parameters": S2C_REQUIRED,
            "s2c_notes": {
                "S2C1117": (
                    "DX200 only — enables Speed FB; expect alarm 8001[10] then reboot"
                ),
            },
            "motion_action": "control_msgs/action/FollowJointTrajectory",
            "motion_prerequisites": [
                "start_traj_mode (servos + trajectory mode + INIT_ROS)",
                f"system software {MOTOROS2_MIN_FIRMWARE} or newer",
                "FastDDS RMW on the ROS 2 PC",
            ],
            "motion_role": (
                "Telemetry and air moves / jog only. follow_joint_trajectory "
                f"requires INIT_ROS and {MOTOROS2_MIN_FIRMWARE}+; it must not "
                "carry Motoweld WVON / MACRO1 weld cycles — those stay in "
                "native .JBI."
            ),
            "known_limitations": [
                "No Cartesian motion interfaces",
                "No Absolute Accuracy Compensation benefit (Cartesian unsupported)",
                "No asynchronous motion",
                "No partial FollowJointTrajectory goals",
                "FastDDS is the only supported RMW",
                "Welding (ARCON / WVON / MACRO1) must use native .JBI",
            ],
            "published_topics": {
                "joint_states": "sensor_msgs/msg/JointState (all groups)",
                "ctrl_groups/r1/joint_states": "rad — robot RB1 / R1",
                "ctrl_groups/b1/joint_states": "m — base RECT-X / BS1",
                "ctrl_groups/s1/joint_states": "rad — station TURN / ST1",
                "ctrl_groups/s2/joint_states": "rad — station TURN / ST2",
                "ctrl_groups/s3/joint_states": "rad — station TURN / ST3",
                "ctrl_groups/s4/joint_states": "rad — station TURN / ST4",
                "robot_status": "industrial_msgs/msg/RobotStatus",
                "tf": (
                    "tf2_msgs/msg/TFMessage — world→base→flange→tool0 and "
                    "flange→tcp_N; this cell uses tcp_0 from TOOL.CND"
                ),
            },
            "tf_frames": {
                "world → base": "Yaskawa BF → ROS-I base (RF)",
                "base → flange": (
                    "REP-103 flange: X+ forward (out of face), Y+ left, Z+ up"
                ),
                "flange → tool0": (
                    "Yaskawa all-zeros toolframe: Z+ forward (out of flange); static"
                ),
                "flange → tcp_0": (
                    "Active Tool No. 0 = flange→tool0 · TOOL.CND "
                    "(xyz=[-54.280,1.052,415.702] mm, "
                    "RPY=[1.3119,-42.6215,-6.8387] deg in tool0)"
                ),
            },
            "services": [
                "start_traj_mode",
                "stop_traj_mode",
                "start_point_queue_mode",
                "queue_traj_point",
                "reset_error",
                "read_single_io",
                "write_single_io",
                "read_group_io",
                "write_group_io",
                "read_mregister",
                "write_mregister",
            ],
            "error_code_format": {
                "description": (
                    "FollowJointTrajectory result error_code uses -ECCCCC "
                    "(ROS digit E + Motoman CCCCC). Use decode_motoros2_error()."
                ),
                "example": decode_motoros2_error(-100101),
            },
            "legacy_ros1": (
                "ros-industrial/motoman motoman_driver "
                "(robot_interface_streaming_dx200.launch)"
            ),
            "connected": connected,
            "agent_host": host or None,
            "agent_port": port if host else None,
            "detail": detail,
            "role": (
                "Optional live joint / TF stream. Welding jobs for this cell are "
                "INFORM .JBI (WVON + MACRO1), not ROS trajectories."
            ),
        },
    }


def _tcp_open(host: str, port: int, timeout: float = 0.4) -> tuple[bool, str]:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True, "tcp open"
    except OSError as exc:
        return False, str(exc)
