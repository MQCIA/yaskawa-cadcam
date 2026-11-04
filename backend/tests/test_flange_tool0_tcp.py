"""MotoROS2 flange → tool0 → tcp_0 chain (ros_api.md + TOOL.CND)."""

from __future__ import annotations

import numpy as np
import pytest

from app.kinematics_engine import (
    flange_se3,
    flange_to_tool0_se3,
    forward_kinematics,
    tool_file_se3,
    tool0_se3,
)
from app.robot_config import TOOL0_RPY_DEG, TOOL0_XYZ_MM
from app.robot_link import describe_robot_link


def test_flange_to_tool0_maps_motoman_z_to_flange_x():
    """Motoman +Z (out of flange) coincides with REP-103 flange +X."""
    R = flange_to_tool0_se3().R
    # Columns of R are tool0 basis vectors expressed in flange.
    assert R[:, 2] == pytest.approx([1.0, 0.0, 0.0], abs=1e-9)  # tool0 Z
    assert R[:, 0] == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)  # tool0 X
    assert R[:, 1] == pytest.approx([0.0, -1.0, 0.0], abs=1e-9)  # tool0 Y


def test_tool_file_translation_matches_tool_cnd():
    t = tool_file_se3().t
    assert t[0] == pytest.approx(TOOL0_XYZ_MM[0] / 1000.0, abs=1e-12)
    assert t[1] == pytest.approx(TOOL0_XYZ_MM[1] / 1000.0, abs=1e-12)
    assert t[2] == pytest.approx(TOOL0_XYZ_MM[2] / 1000.0, abs=1e-12)


def test_tcp_is_flange_times_flange_to_tool0_times_tool_file():
    q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    composed = flange_se3(q) * flange_to_tool0_se3() * tool_file_se3()
    tcp = forward_kinematics(q, frame="tcp")
    assert tcp["x"] == pytest.approx(float(composed.t[0]), abs=1e-9)
    assert tcp["y"] == pytest.approx(float(composed.t[1]), abs=1e-9)
    assert tcp["z"] == pytest.approx(float(composed.t[2]), abs=1e-9)
    # Motoman +Z (415.7 mm) aligns with flange +X, not flange +Z.
    flange = forward_kinematics(q, frame="flange")
    assert not np.isclose(tcp["z"], flange["z"] + TOOL0_XYZ_MM[2] / 1000.0)
    tool0 = forward_kinematics(q, frame="tool0")
    assert tool0["x"] == pytest.approx(flange["x"], abs=1e-9)
    assert tool0["y"] == pytest.approx(flange["y"], abs=1e-9)
    assert tool0["z"] == pytest.approx(flange["z"], abs=1e-9)


def test_tool0_legacy_helper_equals_composed_chain():
    assert tool0_se3() == flange_to_tool0_se3() * tool_file_se3()
    assert TOOL0_RPY_DEG[1] == pytest.approx(-42.6215)


def test_robot_link_exposes_motoros2_topics_and_jbi_weld_path():
    info = describe_robot_link()
    assert info["execution"] == "inform_jbi"
    ros = info["ros"]
    assert ros["published_topics"]["ctrl_groups/r1/joint_states"]
    assert ros["published_topics"]["ctrl_groups/b1/joint_states"]
    assert ros["published_topics"]["ctrl_groups/s1/joint_states"]
    assert "flange→tcp" in ros["published_topics"]["tf"] or "tcp" in ros["published_topics"]["tf"]
    assert "read_mregister" in ros["services"]
    assert "start_traj_mode" in ros["services"]
    assert "reset_error" in ros["services"]
    assert "INIT_ROS" in ros["motion_role"] or "INIT_ROS" in str(ros["motion_prerequisites"])
    assert ".JBI" in info["native_program"]["note"]
    assert "WVON" in info["native_program"]["note"] or "WVON" in info["native_program"]["arc"]
