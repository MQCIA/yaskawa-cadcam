"""How this cell is actually driven.

ArcNC (https://arcnc.com) does not publish a robot API or a ROS driver.
It writes a native controller program. On this Yaskawa DX200 cell that file
is INFORM ``.JBI`` (groups RB1/BS1 + ST1…ST4, ARCON, Motoweld ``MACRO1`` /
``WVON``). The pendant or MotoSim runs that job.

Live joint streaming is a different stack, shipped by Yaskawa rather than by
ArcNC (see ``Yaskawa-Global/motoros2`` ``doc/ros_api.md``):

* ROS 2: MotoROS2 (``Yaskawa-Global/motoros2``), binary ``mr2_dx2_*.out`` on
  DX200 (system software ``DN2.44.00-00`` or newer) plus a micro-ROS Agent.
  Motion is ``control_msgs/action/FollowJointTrajectory`` after
  ``start_traj_mode`` / ``INIT_ROS``.
* ROS 1: ``motoman_driver`` + the older MotoROS MotoPlus app
  (``robot_interface_streaming_dx200.launch``).

Set ``MOTOROS_AGENT_HOST`` (and optional ``MOTOROS_AGENT_PORT``, default 8888)
to probe whether a micro-ROS Agent is listening. Welding execution in this
app stays the INFORM download; the probe only reports the optional live link.
"""

from __future__ import annotations

import os
import socket


def describe_robot_link() -> dict:
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
            "groups": ["RB1", "BS1", "ST1"],
            "arc": "ARCON/ARCOF + MACRO1 ARGF + WVON/WVOF (Motoweld)",
            "note": (
                "Spawanie z WVON i MACRO1 zawsze idzie przez natywny plik .JBI — "
                "nie przez FollowJointTrajectory."
            ),
        },
        "ros": {
            "recommended": "MotoROS2",
            "package": "Yaskawa-Global/motoros2",
            "docs": "doc/ros_api.md",
            "controller_binary": "mr2_dx2_*.out",
            "min_system": "DN2.44.00-00",
            "agent": "micro-ROS Agent",
            "motion_action": "control_msgs/action/FollowJointTrajectory",
            "motion_prerequisites": [
                "start_traj_mode (enables servos + trajectory mode + INIT_ROS)",
                "system software DN2.44.00-00 or newer",
            ],
            "motion_role": (
                "Telemetry and air moves / jog only. follow_joint_trajectory "
                "requires INIT_ROS and DN2.44.00-00+; it must not carry Motoweld "
                "WVON / MACRO1 weld cycles — those stay in native .JBI."
            ),
            "published_topics": {
                "joint_states": "sensor_msgs/msg/JointState (all groups)",
                "ctrl_groups/r1/joint_states": (
                    "sensor_msgs/msg/JointState — robot RB1 / R1"
                ),
                "ctrl_groups/b1/joint_states": (
                    "sensor_msgs/msg/JointState — base RECT-X / BS1"
                ),
                "ctrl_groups/s1/joint_states": (
                    "sensor_msgs/msg/JointState — station TURN / ST1"
                ),
                "robot_status": "industrial_msgs/msg/RobotStatus",
                "tf": (
                    "tf2_msgs/msg/TFMessage — world→base→flange→tool0 and "
                    "flange→tcp_N (active TOOL); this cell uses tcp_0 from TOOL.CND"
                ),
            },
            "tf_frames": {
                "flange": (
                    "REP-103 relative to the link: X+ forward (out of flange), "
                    "Y+ left, Z+ up"
                ),
                "tool0": (
                    "Yaskawa all-zeros toolframe: Z+ forward (out of flange); "
                    "static flange→tool0"
                ),
                "tcp_0": (
                    "Active Tool No. 0 from TOOL.CND; xyz=[-54.280,1.052,415.702] mm, "
                    "RPY=[1.3119,-42.6215,-6.8387] deg in Motoman tool0"
                ),
            },
            "services": [
                "read_mregister",
                "write_mregister",
                "read_single_io",
                "write_single_io",
                "read_group_io",
                "write_group_io",
                "start_traj_mode",
                "stop_traj_mode",
                "start_point_queue_mode",
                "queue_traj_point",
                "reset_error",
            ],
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
