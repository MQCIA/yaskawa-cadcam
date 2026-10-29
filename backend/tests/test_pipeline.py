"""Pytest suite covering the backend modules and the full pipeline."""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np
import pytest
import trimesh
from shapely.geometry import Polygon

from app.kinematics_engine import forward_kinematics, solve_ik_path, get_robot
from app.dx200_postprocessor import (
    generate_jbi,
    WeldSegment,
    PostprocessorConfig,
    load_reference_jbi,
)
from app.cad_analysis import detect_seams
from app.path_planning import plan_from_seams
from app.robot_config import ACTIVE_MODEL, ACTIVE_POSITIONER, MODELS
from app.welding_config import ACTIVE_POWER_SOURCE, LORCH_S8_SCHEDULES


FIXTURES = Path(__file__).parent / "fixtures"
CELL = FIXTURES / "robot-cell"


@pytest.fixture(scope="module")
def lbracket(tmp_path_factory) -> str:
    poly = Polygon(
        [(0, 0), (0.3, 0), (0.3, 0.05), (0.05, 0.05), (0.05, 0.3), (0, 0.3)]
    )
    solid = trimesh.creation.extrude_polygon(poly, height=0.4)
    solid.apply_translation([0.7, -0.2, 0.2])
    path = str(tmp_path_factory.mktemp("cad") / "lbracket.stl")
    solid.export(path)
    return path


def test_config_loaded():
    assert ACTIVE_MODEL.name in MODELS
    assert ACTIVE_MODEL.name == "MA2010"
    assert ACTIVE_POSITIONER.name == "TURN-ST1"
    assert ACTIVE_MODEL.pulses_per_degree is not None
    assert ACTIVE_MODEL.pulses_per_degree[0] > 1000  # S ≈ 1341 from RC1G
    assert ACTIVE_POSITIONER.axes[0].pulses_per_degree > 1000  # ≈ 1019.7
    assert "MOTOWELD" in ACTIVE_POWER_SOURCE.name.upper()
    assert set(LORCH_S8_SCHEDULES) >= {1, 2, 3}


def test_fk_ik_roundtrip():
    q = [10, -20, 15, 5, 30, 0]
    pose = forward_kinematics(q)
    res = solve_ik_path([pose], q_seed_deg=q)
    assert res["all_reachable"]
    pose2 = forward_kinematics(res["joint_angles_deg"][0])
    err = np.linalg.norm(
        np.array([pose[k] for k in "xyz"]) - np.array([pose2[k] for k in "xyz"])
    )
    assert err < 1e-3


def test_jbi_ma2010_cell_profile():
    from app.pulse_config import BC_PARK_PULSE, track_mm_to_pulses, track_mm_from_world_z

    pts = [
        {"x": 0.8, "y": 0.0, "z": 0.5, "rx": 180},
        {"x": 0.9, "y": 0.1, "z": 0.5, "rx": 180},
        {"x": 1.0, "y": 0.2, "z": 0.5, "rx": 180},
    ]
    segs = [WeldSegment(0, 2, weld_speed=5.0, arc_file=3)]
    jbi = generate_jbi(pts, segs, PostprocessorConfig(), station_deg=[0, 45, 90])
    assert jbi.startswith("/JOB")
    assert "///NPOS 3,3,3,0,0,0" in jbi
    assert "///GROUP1 RB1,BS1" in jbi
    assert "///GROUP2 ST1" in jbi
    assert "ARCON" in jbi and "ARCOF" in jbi
    assert "ASF#(" not in jbi
    assert "WVON WEV#(21)" in jbi and "WVOF" in jbi
    assert "MACRO1 MJ#(0)" in jbi
    assert "+MOVJ EC" in jbi
    # Auto BC from world Z=0.5 m → track mm = −500.
    expect_bc = track_mm_to_pulses(track_mm_from_world_z(0.5))
    assert f"BC00000={expect_bc}" in jbi
    assert f"BC00002={expect_bc}" in jbi
    # Explicit park still available for 000.JBI-style jobs.
    park_jbi = generate_jbi(
        pts, segs, PostprocessorConfig(), station_deg=[0, 45, 90], base_pulse=BC_PARK_PULSE
    )
    assert f"BC00000={BC_PARK_PULSE}" in park_jbi
    assert "EC00001=45886,0" in jbi  # 45 * 1019.6889…
    assert "Motoweld" in jbi
    assert jbi.strip().endswith("END")


def test_jbi_pulse_from_joints():
    from app.pulse_config import deg_to_joint_pulses, station_deg_to_ec

    pts = [{"x": 0.8, "y": 0.0, "z": 0.5, "rx": 180}]
    q = [10.0, -20.0, 15.0, 5.0, 30.0, 0.0]
    jbi = generate_jbi(
        pts,
        config=PostprocessorConfig(job_name="PULSE_TEST"),
        station_deg=[0.0],
        joint_angles_deg=[q],
    )
    pulses = deg_to_joint_pulses(q)
    assert "///POSTYPE PULSE" in jbi
    assert f"C00000={','.join(str(p) for p in pulses)}" in jbi
    assert "///RECTAN" not in jbi
    e1, e2 = station_deg_to_ec(0.0)
    assert f"EC00000={e1},{e2}" in jbi


def test_rect_x_pulses_per_mm_and_bc_sequence():
    """M1.3: BC00000…BCxxxxx from track mm / station / per-point pulses."""
    from app.pulse_config import (
        BASE_PULSES_PER_MM,
        BC_HOME_PULSE,
        BC_PARK_PULSE,
        BC_ST1_ST3_RANGE,
        BC_ST2_RANGE,
        PULSES_PER_MM,
        STATION_BC_MID,
        track_mm_to_pulses,
        track_pulses_to_mm,
        resolve_base_pulses,
    )

    assert PULSES_PER_MM == pytest.approx(257.708125)
    assert BASE_PULSES_PER_MM == PULSES_PER_MM
    assert track_mm_to_pulses(0.0) == BC_HOME_PULSE
    assert track_pulses_to_mm(BC_HOME_PULSE) == pytest.approx(0.0)
    # Park − home ≈ 1195 mm (half of 2400 mm station span).
    assert track_pulses_to_mm(BC_PARK_PULSE) == pytest.approx(1194.805, abs=0.01)
    assert track_mm_to_pulses(track_pulses_to_mm(BC_PARK_PULSE)) == BC_PARK_PULSE

    # Station midpoints sit inside taught envelopes.
    assert BC_ST2_RANGE[0] <= STATION_BC_MID["ST2"] <= BC_ST2_RANGE[1]
    assert BC_ST1_ST3_RANGE[0] <= STATION_BC_MID["ST1"] <= BC_ST1_ST3_RANGE[1]

    pts = [
        {"x": 0.8, "y": 0.0, "z": -1.2, "rx": 180},  # ST1 side
        {"x": 0.9, "y": 0.0, "z": 0.0, "rx": 180},  # home
        {"x": 1.0, "y": 0.0, "z": 1.2, "rx": 180},  # ST2 side
    ]
    # Per-point explicit pulses.
    explicit = [808680, BC_HOME_PULSE, 375361]
    jbi = generate_jbi(
        pts,
        config=PostprocessorConfig(job_name="BC_SEQ"),
        station_deg=[0.0, 0.0, 0.0],
        base_pulse=explicit,
    )
    for i, bc in enumerate(explicit):
        assert f"BC{i:05d}={bc}" in jbi

    # Auto from world Z.
    auto = resolve_base_pulses(3, points=pts)
    assert auto[1] == BC_HOME_PULSE
    assert auto[0] > auto[1] > auto[2]  # −Z → higher BC
    auto_jbi = generate_jbi(pts, config=PostprocessorConfig(job_name="BC_AUTO"))
    for i, bc in enumerate(auto):
        assert f"BC{i:05d}={bc}" in auto_jbi

    # Station override → constant mid for whole job.
    st_jbi = generate_jbi(
        pts, config=PostprocessorConfig(job_name="BC_ST2"), station="ST2"
    )
    assert f"BC00000={STATION_BC_MID['ST2']}" in st_jbi
    assert f"BC00002={STATION_BC_MID['ST2']}" in st_jbi

    # Per-point base_mm.
    mm_list = [0.0, 500.0, -500.0]
    mm_jbi = generate_jbi(
        pts, config=PostprocessorConfig(job_name="BC_MM"), base_mm=mm_list
    )
    for i, mm in enumerate(mm_list):
        assert f"BC{i:05d}={track_mm_to_pulses(mm)}" in mm_jbi


def test_pulse_softlimit_roundtrip():
    from app.pulse_config import (
        MA2010_PULSES_PER_DEGREE,
        RC1G_SOFTLIMIT_POS,
        deg_to_joint_pulses,
        joint_pulses_to_deg,
    )

    # Softlimit + side should map back near joint limit degrees
    q = joint_pulses_to_deg(list(RC1G_SOFTLIMIT_POS))
    assert abs(q[0] - 180.0) < 0.05
    assert abs(q[3] - 150.0) < 0.05
    back = deg_to_joint_pulses(q)
    assert back == list(RC1G_SOFTLIMIT_POS)
    assert len(MA2010_PULSES_PER_DEGREE) == 6


def test_jbi_cartesian_review_and_drop01_grammar():
    ref = (FIXTURES / "DROP01.JBI").read_text(encoding="utf-8")
    assert "///POSTYPE USER" in ref or "///POSTYPE PULSE" in ref
    assert re.search(r"MOVL C\d{5} EC\d{5} V=", ref)

    pts = [
        {"x": 536.88, "y": 595.0, "z": 1428.0, "rx": 180, "ry": 0, "rz": 0},
        {"x": 536.88, "y": 595.0, "z": 360.81, "rx": 180, "ry": 0, "rz": 0},
    ]
    jbi = generate_jbi(
        pts,
        config=PostprocessorConfig(
            job_name="DROP01_LIKE",
            profile="cartesian_review",
            joint_approach=False,
            station_pulses_per_deg=1.0,
        ),
        station_deg=[359, 359],
    )
    for token in (
        "/JOB",
        "///NPOS 2,0,2,0,0,0",
        "///USER 1",
        "///POSTYPE USER",
        "EC00000=359",
        "///ATTR SC,RW,RJ",
        "///GROUP1 RB1,ST1",
        "MOVL C00000 EC00000",
        "END",
    ):
        assert token in jbi, f"missing {token!r}"


def test_real_cell_000_jbi_grammar():
    cell_job = (CELL / "000.JBI").read_text(encoding="utf-8", errors="replace")
    assert "///POSTYPE PULSE" in cell_job
    assert "///GROUP1 RB1,BS1" in cell_job
    assert "///GROUP2 ST1" in cell_job
    assert "ARCON" in cell_job and "ARCOF" in cell_job
    assert "WVON WEV#" in cell_job
    assert "+MOVJ EC" in cell_job
    sysinfo = (CELL / "SYSTEM.SYS").read_text(encoding="utf-8", errors="replace")
    assert "MA02010" in sysinfo
    assert "ARC WELDING" in sysinfo
    tool = (CELL / "TOOL.CND").read_text(encoding="utf-8", errors="replace")
    assert "415.702" in tool
    assert load_reference_jbi("000.JBI").startswith("/JOB")


def test_cad_seam_detection(lbracket):
    res = detect_seams(lbracket, angle_tol_deg=25, target_angle_deg=90)
    assert res["seam_count"] >= 1
    assert res["total_length"] > 0


def test_full_pipeline(lbracket):
    seams = detect_seams(lbracket, angle_tol_deg=25, target_angle_deg=90)
    planned = plan_from_seams(seams["segments"], samples_per_seam=5, arc_file=2)
    assert len(planned.points) > 0
    ik = solve_ik_path(planned.points)
    assert ik["all_reachable"]
    from app.pulse_config import resolve_base_pulses

    jbi = generate_jbi(
        planned.points,
        planned.weld_segments,
        PostprocessorConfig(),
        station_deg=[0.0] * len(planned.points),
        joint_angles_deg=ik["joint_angles_deg"],
    )
    assert "///POSTYPE PULSE" in jbi
    assert "///RECTAN" not in jbi
    assert "C00000=" in jbi
    expect_bc = resolve_base_pulses(len(planned.points), points=planned.points)
    assert f"BC00000={expect_bc[0]}" in jbi
    assert f"BC{len(planned.points) - 1:05d}={expect_bc[-1]}" in jbi
    assert "ARCON" in jbi and jbi.strip().endswith("END")
    assert "RB1,BS1" in jbi


def test_robot_dof():
    assert get_robot().n == 6
