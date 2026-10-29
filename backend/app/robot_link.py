"""How this cell is actually driven.

ArcNC (https://arcnc.com) does not publish a robot API or a ROS driver.
It writes a native controller program. On this Yaskawa DX200 cell that file
is INFORM ``.JBI`` (groups RB1/BS1 + ST1, ARCON, Motoweld). The pendant or
MotoSim runs that job.

Live joint streaming is a different stack, shipped by Yaskawa rather than by
ArcNC:

* ROS 2: MotoROS2 (``Yaskawa-Global/motoros2``), binary ``mr2_dx2_*.out`` on
  DX200 (system software ``DN2.44.00-00`` or newer) plus a micro-ROS Agent.
  Motion is ``control_msgs/action/FollowJointTrajectory``.
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
            "arc": "ARCON/ARCOF + Motoweld (no ASF#)",
        },
        "ros": {
            "recommended": "MotoROS2",
            "package": "Yaskawa-Global/motoros2",
            "controller_binary": "mr2_dx2_*.out",
            "min_system": "DN2.44.00-00",
            "agent": "micro-ROS Agent",
            "motion_action": "control_msgs/action/FollowJointTrajectory",
            "legacy_ros1": (
                "ros-industrial/motoman motoman_driver "
                "(robot_interface_streaming_dx200.launch)"
            ),
            "connected": connected,
            "agent_host": host or None,
            "agent_port": port if host else None,
            "detail": detail,
            "role": (
                "Optional live joint stream. Welding jobs for this cell are "
                "INFORM, not ROS trajectories."
            ),
        },
    }


def _tcp_open(host: str, port: int, timeout: float = 0.4) -> tuple[bool, str]:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True, "tcp open"
    except OSError as exc:
        return False, str(exc)
