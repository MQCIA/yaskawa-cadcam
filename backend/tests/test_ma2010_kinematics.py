"""MA2010 URDF chain + TOOL 0 + cell-home IK seed (M1.4)."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from app.kinematics_engine import (
    flange_se3,
    forward_kinematics,
    get_robot,
    solve_ik_path,
    tool0_se3,
    urdf_flange_se3_reference,
)
from app.pulse_config import MA2010_LIMITS_DEG, deg_to_joint_pulses, joint_pulses_to_deg
from app.robot_config import (
    ACTIVE_MODEL,
    CELL_HOME_PULSES,
    MA2010_URDF,
    cell_home_deg,
)

GOLDEN = (
    Path(__file__).parent / "fixtures" / "robot-cell" / "golden_poses.json"
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


def test_joint_limits_match_urdf_softlimits_table():
    for joint, (lo, hi) in zip(ACTIVE_MODEL.joints, MA2010_LIMITS_DEG):
        assert joint.qlim[0] == pytest.approx(np.deg2rad(lo), abs=1e-4)
        assert joint.qlim[1] == pytest.approx(np.deg2rad(hi), abs=1e-4)
    qlim = get_robot().qlim
    assert qlim.shape == (2, 6)


def test_zero_pose_flange_matches_urdf_tool0():
    pose = forward_kinematics([0, 0, 0, 0, 0, 0], frame="flange")
    assert pose["x"] == pytest.approx(0.150 + 1.082 + 0.100, abs=1e-9)
    assert pose["y"] == pytest.approx(0.0, abs=1e-9)
    assert pose["z"] == pytest.approx(0.505 + 0.760 + 0.200, abs=1e-9)


def test_erobot_matches_independent_urdf_se3():
    for q in ([0, 0, 0, 0, 0, 0], cell_home_deg()):
        err = np.linalg.norm(flange_se3(q).t - urdf_flange_se3_reference(q).t)
        assert err * 1000.0 < 1.0


def test_cell_home_fk_ik_roundtrip_pulses():
    home = cell_home_deg(CELL_HOME_PULSES)
    assert home == pytest.approx(joint_pulses_to_deg(list(CELL_HOME_PULSES)))
    tcp = forward_kinematics(home, frame="tcp")
    res = solve_ik_path([tcp], q_seed_deg=None, frame="tcp")
    assert res["all_reachable"]
    assert res["seed_deg"] == pytest.approx(home, abs=1e-6)
    assert res["joint_angles_deg"][0] == pytest.approx(home, abs=1e-5)
    back = deg_to_joint_pulses(res["joint_angles_deg"][0])
    assert max(abs(a - b) for a, b in zip(CELL_HOME_PULSES, back)) <= 2


def test_golden_poses_urdf_fk_and_roundtrip():
    data = json.loads(GOLDEN.read_text(encoding="utf-8"))
    assert len(data["poses"]) == 8
    for pose in data["poses"]:
        assert pose["verified_on_pendant"] is False
        assert pose["cartesian_status"] == "UNVERIFIED_ON_PENDANT"
        q = joint_pulses_to_deg(pose["pulses"])
        err = np.linalg.norm(flange_se3(q).t - urdf_flange_se3_reference(q).t) * 1000
        assert err < 1.0, pose["id"]
        tcp = forward_kinematics(q, frame="tcp")
        res = solve_ik_path([tcp], q_seed_deg=q, frame="tcp")
        assert res["diagnostics"][0]["success"], pose["id"]
        back = deg_to_joint_pulses(res["joint_angles_deg"][0])
        assert max(abs(a - b) for a, b in zip(pose["pulses"], back)) <= 2, pose["id"]


def test_tool0_se3_not_identity():
    T = tool0_se3()
    assert np.linalg.norm(T.t) > 0.3  # ~419 mm stick-out family
