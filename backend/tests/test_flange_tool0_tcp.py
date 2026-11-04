"""MotoROS2 flange → tool0 → tcp_0 chain (ros_api.md + TOOL.CND)."""

from __future__ import annotations

import numpy as np
import pytest

from app.kinematics_engine import (
    flange_se3,
    flange_to_tool0_se3,
    forward_kinematics,
    forward_kinematics_frames,
    tool_file_se3,
    tool0_se3,
)
from app.robot_config import TOOL0_RPY_DEG, TOOL0_XYZ_MM
from app.robot_link import (
    CELL_FIRMWARE,
    decode_motoros2_error,
    describe_robot_link,
    firmware_compatible,
)


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


def test_forward_kinematics_frames_bundle():
    q = [10.0, -20.0, 15.0, 5.0, 30.0, 0.0]
    bundle = forward_kinematics_frames(q)
    assert bundle["tf_chain"].startswith("world")
    for key in ("flange", "tool0", "tcp_0", "tcp"):
        assert set(bundle[key]) >= {"x", "y", "z", "rx", "ry", "rz"}
    assert bundle["tcp_0"] == bundle["tcp"]
    assert bundle["tool_file_mm"]["xyz"][2] == pytest.approx(415.702)


def test_decode_motoros2_error_minus_e_ccccc():
    d = decode_motoros2_error(-100101)
    assert d["parsed"]
    assert d["ros_error"] == -1
    assert d["ros_error_name"] == "INVALID_GOAL"
    assert d["motoros2_code"] == 101
    assert "alarm" in (d["motoros2_hint"] or "").lower()


def test_firmware_dump_requires_upgrade():
    fw = firmware_compatible()
    assert CELL_FIRMWARE.startswith("DN2.27")
    assert fw["compatible"] is False
    assert "DN2.44" in (fw["action"] or "")


def test_robot_link_exposes_motoros2_topics_and_jbi_weld_path():
    info = describe_robot_link()
    assert info["execution"] == "inform_jbi"
    ros = info["ros"]
    assert ros["version"] == "0.2.1"
    assert ros["binaries"]["humble"]["md5"].startswith("6672")
    assert ros["hardware"]["usb_config_port"] == "CN106"
    assert ros["s2c_parameters"]["S2C1117"] == 1
    assert ros["published_topics"]["ctrl_groups/r1/joint_states"]
    assert ros["published_topics"]["ctrl_groups/b1/joint_states"]
    assert ros["published_topics"]["ctrl_groups/s1/joint_states"]
    assert ros["published_topics"]["ctrl_groups/s4/joint_states"]
    assert "read_mregister" in ros["services"]
    assert "start_traj_mode" in ros["services"]
    assert "reset_error" in ros["services"]
    assert any("Cartesian" in x or "cartesian" in x.lower() for x in ros["known_limitations"])
    assert ros["firmware"]["compatible"] is False
    assert ".JBI" in info["native_program"]["note"]
    assert len(info["group_map"]) == 6
