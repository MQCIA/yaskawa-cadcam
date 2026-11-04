"""MA2010 URDF chain + TOOL 0 + cell-home IK seed."""

from __future__ import annotations

import numpy as np
import pytest

from app.kinematics_engine import (
    forward_kinematics,
    get_robot,
    solve_ik_path,
    tool0_se3,
)
from app.pulse_config import MA2010_LIMITS_DEG, joint_pulses_to_deg
from app.robot_config import (
    ACTIVE_MODEL,
    CELL_HOME_PULSES,
    MA2010_URDF,
    TOOL0_XYZ_MM,
    cell_home_deg,
)


def test_ma2010_is_urdf_chain_not_placeholder():
    assert ACTIVE_MODEL.name == "MA2010"
    assert ACTIVE_MODEL.urdf_chain is True
    assert ACTIVE_MODEL.approximate is False
    assert ACTIVE_MODEL.home_deg is not None
    g = MA2010_URDF
    assert (g.d1, g.a1, g.a2, g.a3, g.d4, g.d6) == (
        0.505,
        0.150,
        0.760,
        0.200,
        1.082,
        0.100,
    )


def test_joint_limits_match_urdf_and_pulse_softlimits():
    for joint, (lo, hi) in zip(ACTIVE_MODEL.joints, MA2010_LIMITS_DEG):
        assert joint.qlim[0] == pytest.approx(np.deg2rad(lo), abs=1e-4)
        assert joint.qlim[1] == pytest.approx(np.deg2rad(hi), abs=1e-4)
    qlim = get_robot().qlim
    assert qlim.shape == (2, 6)
    assert qlim[0, 1] == pytest.approx(np.deg2rad(-105), abs=1e-4)
    assert qlim[1, 4] == pytest.approx(np.deg2rad(90), abs=1e-4)


def test_zero_pose_flange_matches_urdf_tool0():
    # At all zeros: x = a1 + d4 + d6, y = 0, z = d1 + a2 + a3.
    pose = forward_kinematics([0, 0, 0, 0, 0, 0], frame="flange")
    assert pose["x"] == pytest.approx(0.150 + 1.082 + 0.100, abs=1e-9)
    assert pose["y"] == pytest.approx(0.0, abs=1e-9)
    assert pose["z"] == pytest.approx(0.505 + 0.760 + 0.200, abs=1e-9)
    assert pose["rx"] == pytest.approx(0.0, abs=1e-6)
    assert pose["ry"] == pytest.approx(0.0, abs=1e-6)
    assert pose["rz"] == pytest.approx(0.0, abs=1e-6)


def test_tool0_offsets_tcp_from_flange():
    flange = forward_kinematics([0, 0, 0, 0, 0, 0], frame="flange")
    tcp = forward_kinematics([0, 0, 0, 0, 0, 0], frame="tcp")
    # At identity flange orientation, TCP translation = flange + TOOL0 xyz.
    assert tcp["x"] == pytest.approx(flange["x"] + TOOL0_XYZ_MM[0] / 1000.0, abs=1e-9)
    assert tcp["y"] == pytest.approx(flange["y"] + TOOL0_XYZ_MM[1] / 1000.0, abs=1e-9)
    assert tcp["z"] == pytest.approx(flange["z"] + TOOL0_XYZ_MM[2] / 1000.0, abs=1e-9)
    assert tool0_se3().t[2] == pytest.approx(TOOL0_XYZ_MM[2] / 1000.0, abs=1e-12)


def test_cell_home_fk_ik_roundtrip():
    home = cell_home_deg(CELL_HOME_PULSES)
    assert home == pytest.approx(joint_pulses_to_deg(list(CELL_HOME_PULSES)))
    tcp = forward_kinematics(home, frame="tcp")
    res = solve_ik_path([tcp], q_seed_deg=None, frame="tcp")  # default seed = home
    assert res["all_reachable"]
    assert res["seed_deg"] == pytest.approx(home, abs=1e-6)
    assert res["joint_angles_deg"][0] == pytest.approx(home, abs=1e-6)
    back = forward_kinematics(res["joint_angles_deg"][0], frame="tcp")
    err = np.linalg.norm(
        np.array([tcp[k] for k in "xyz"]) - np.array([back[k] for k in "xyz"])
    )
    assert err < 1e-6


def test_ik_default_seed_is_cell_home_not_midrange():
    home = cell_home_deg()
    tcp = forward_kinematics(home, frame="tcp")
    res = solve_ik_path([tcp])
    assert res["seed_deg"] == pytest.approx(home, abs=1e-9)
