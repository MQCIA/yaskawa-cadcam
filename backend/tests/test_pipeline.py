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
    SAFE_HOME_BC,
    SAFE_HOME_PULSES,
    generate_jbi,
    WeldSegment,
    PostprocessorConfig,
    load_reference_jbi,
)
from app.cad_analysis import detect_seams
from app.path_planning import plan_from_seams
from app.robot_config import ACTIVE_MODEL, ACTIVE_POSITIONER, CELL_HOME_PULSES, MODELS
from app.welding_config import ACTIVE_POWER_SOURCE, MOTOWELD_SCHEDULES


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
    assert "MA2010" in ACTIVE_POWER_SOURCE.cell_label
    assert set(MOTOWELD_SCHEDULES) >= {1, 2, 3}
    assert MOTOWELD_SCHEDULES[1].argf == 1
    assert MOTOWELD_SCHEDULES[2].argf == 4
    assert MOTOWELD_SCHEDULES[3].argf == 5
    assert "ARCSRT" in MOTOWELD_SCHEDULES[1].params_source


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
    # No joints → RECTAN review (no home bookend).
    jbi = generate_jbi(pts, segs, PostprocessorConfig(crlf=False), station_deg=[0, 45, 90])
    assert jbi.startswith("/JOB")
    assert "///NPOS 3,3,3,0,0,0" in jbi
    assert "///GROUP1 RB1,BS1" in jbi
    assert "///GROUP2 ST1" in jbi
    assert "ARCON" in jbi and "ARCOF" in jbi
    assert "ASF#(" not in jbi
    assert "WVON WEV#(21)" in jbi and "WVOF" in jbi
    assert "MACRO1 MJ#(0) ARGF5" in jbi  # schedule 3 → ARGF5
    assert "+MOVJ EC" in jbi
    assert "DEC=20" in jbi
    expect_bc = track_mm_to_pulses(track_mm_from_world_z(0.5))
    assert f"BC00000={expect_bc}" in jbi
    assert f"BC00002={expect_bc}" in jbi
    park_jbi = generate_jbi(
        pts,
        segs,
        PostprocessorConfig(crlf=False),
        station_deg=[0, 45, 90],
        base_pulse=BC_PARK_PULSE,
    )
    assert f"BC00000={BC_PARK_PULSE}" in park_jbi
    assert "EC00001=45886,0" in jbi
    assert "MOTOWELD-E" in jbi
    assert jbi.strip().endswith("END")


def test_jbi_pulse_home_bookends_and_crlf():
    """Production-like PULSE export: home C/BC bookends, CRLF, GROUP2, EC v1,v2."""
    from app.pulse_config import deg_to_joint_pulses

    pts = [
        {"x": 0.8, "y": 0.0, "z": -1.2, "rx": 180},
        {"x": 0.9, "y": 0.0, "z": -1.2, "rx": 180},
    ]
    q = [
        [10.0, -20.0, 15.0, 5.0, 30.0, 0.0],
        [12.0, -18.0, 14.0, 4.0, 28.0, 1.0],
    ]
    segs = [WeldSegment(0, 1, weld_speed=5.0, arc_file=2, weave_no=15)]
    jbi = generate_jbi(
        pts,
        segs,
        PostprocessorConfig(job_name="HOME_WRAP", group2_station="ST2"),
        station_deg=[0.0, 10.0],
        joint_angles_deg=q,
        station="ST2",
    )
    # CRLF line endings (production CF).
    assert "\r\n" in jbi
    assert jbi.split("\r\n")[0] == "/JOB"
    assert "///NPOS 4,4,4,0,0,0" in jbi  # 2 path + 2 home
    assert "///TOOL 0" in jbi
    assert "///POSTYPE PULSE" in jbi
    assert "///PULSE" in jbi
    assert "///ATTR SC,RW" in jbi
    assert "///GROUP1 RB1,BS1" in jbi
    assert "///GROUP2 ST2" in jbi
    home = ",".join(str(p) for p in SAFE_HOME_PULSES)
    assert SAFE_HOME_PULSES == CELL_HOME_PULSES
    assert f"C00000={home}" in jbi
    assert f"C00003={home}" in jbi
    assert f"BC00000={SAFE_HOME_BC}" in jbi
    assert f"BC00003={SAFE_HOME_BC}" in jbi
    path0 = deg_to_joint_pulses(q[0])
    assert f"C00001={','.join(str(p) for p in path0)}" in jbi
    assert "EC00000=192,0" in jbi  # home EC
    assert re.search(r"EC00001=-?\d+,0", jbi)
    assert "MACRO1 MJ#(0) ARGF4" in jbi  # schedule 2
    assert "ARCON" in jbi and "ASF#(" not in jbi
    assert "WVON WEV#(15)" in jbi
    # WVOF before ARCOF
    assert jbi.index("WVOF") < jbi.index("ARCOF")
    assert "MOVJ C00000 BC00000 VJ=80.00 DEC=20  +MOVJ EC00000 VJ=80.00" in jbi
    assert "DEC=20" in jbi


def test_jbi_pulse_from_joints_no_bookend():
    from app.pulse_config import deg_to_joint_pulses, station_deg_to_ec

    pts = [{"x": 0.8, "y": 0.0, "z": 0.5, "rx": 180}]
    q = [10.0, -20.0, 15.0, 5.0, 30.0, 0.0]
    jbi = generate_jbi(
        pts,
        config=PostprocessorConfig(job_name="PULSE_TEST", crlf=False),
        station_deg=[0.0],
        joint_angles_deg=[q],
        bookend_home=False,
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
    assert track_pulses_to_mm(BC_PARK_PULSE) == pytest.approx(1194.805, abs=0.01)
    assert track_mm_to_pulses(track_pulses_to_mm(BC_PARK_PULSE)) == BC_PARK_PULSE

    assert BC_ST2_RANGE[0] <= STATION_BC_MID["ST2"] <= BC_ST2_RANGE[1]
    assert BC_ST1_ST3_RANGE[0] <= STATION_BC_MID["ST1"] <= BC_ST1_ST3_RANGE[1]

    pts = [
        {"x": 0.8, "y": 0.0, "z": -1.2, "rx": 180},
        {"x": 0.9, "y": 0.0, "z": 0.0, "rx": 180},
        {"x": 1.0, "y": 0.0, "z": 1.2, "rx": 180},
    ]
    cfg = PostprocessorConfig(job_name="BC_SEQ", crlf=False, bookend_home=False)
    explicit = [808680, BC_HOME_PULSE, 375361]
    jbi = generate_jbi(
        pts, config=cfg, station_deg=[0.0, 0.0, 0.0], base_pulse=explicit
    )
    for i, bc in enumerate(explicit):
        assert f"BC{i:05d}={bc}" in jbi

    auto = resolve_base_pulses(3, points=pts)
    assert auto[1] == BC_HOME_PULSE
    assert auto[0] > auto[1] > auto[2]
    auto_jbi = generate_jbi(pts, config=PostprocessorConfig(job_name="BC_AUTO", crlf=False))
    for i, bc in enumerate(auto):
        assert f"BC{i:05d}={bc}" in auto_jbi

    st_jbi = generate_jbi(
        pts, config=PostprocessorConfig(job_name="BC_ST2", crlf=False), station="ST2"
    )
    assert f"BC00000={STATION_BC_MID['ST2']}" in st_jbi
    assert "///GROUP2 ST2" in st_jbi

    mm_list = [0.0, 500.0, -500.0]
    mm_jbi = generate_jbi(
        pts, config=PostprocessorConfig(job_name="BC_MM", crlf=False), base_mm=mm_list
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
            crlf=False,
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
    assert "DEC=20" in cell_job
    assert "MACRO1 MJ#(0) ARGF" in cell_job
    assert re.search(r"EC\d{5}=-?\d+,-?\d+", cell_job)
    assert "ASF#(" not in cell_job
    sysinfo = (CELL / "SYSTEM.SYS").read_text(encoding="utf-8", errors="replace")
    assert "MA02010" in sysinfo
    assert "ARC WELDING" in sysinfo
    tool = (CELL / "TOOL.CND").read_text(encoding="utf-8", errors="replace")
    assert "415.702" in tool
    assert load_reference_jbi("000.JBI").startswith("/JOB")

    home = (CELL / "HOME_TRACK.JBI").read_text(encoding="utf-8", errors="replace")
    assert f"C00000={','.join(str(p) for p in CELL_HOME_PULSES)}" in home
    assert "BC00000=570125" in home


def test_generated_matches_production_structure():
    """Generated ma2010_cell PULSE job mirrors 000.JBI / DEFRO header grammar."""
    ref = (CELL / "000.JBI").read_text(encoding="utf-8", errors="replace")
    pts = [
        {"x": 0.8, "y": 0.0, "z": 0.5, "rx": 180},
        {"x": 0.85, "y": 0.0, "z": 0.5, "rx": 180},
        {"x": 0.9, "y": 0.0, "z": 0.5, "rx": 180},
    ]
    q = [[10.0, -20.0, 15.0, 5.0, 30.0, 0.0] for _ in pts]
    segs = [WeldSegment(0, 2, weld_speed=5.0, arc_file=1)]
    jbi = generate_jbi(
        pts,
        segs,
        PostprocessorConfig(job_name="STRUCT"),
        station_deg=[0.0] * 3,
        joint_angles_deg=q,
        macro_arg=1,
        weave_no=21,
    )
    # Shared structural tokens with production 000.JBI.
    for token in (
        "/JOB",
        "//POS",
        "///TOOL 0",
        "///POSTYPE PULSE",
        "///PULSE",
        "//INST",
        "///ATTR SC,RW",
        "///GROUP1 RB1,BS1",
        "///GROUP2 ST1",
        "NOP",
        "MACRO1 MJ#(0) ARGF",
        "ARCON",
        "WVON WEV#(",
        "DEC=20",
        "+MOVJ EC",
        "ARCOF",
        "END",
    ):
        assert token in ref and token in jbi, f"missing shared token {token!r}"

    assert re.search(r"///NPOS \d+,\d+,\d+,0,0,0", jbi)
    assert re.search(r"EC\d{5}=-?\d+,-?\d+", jbi)
    assert "ASF#(" not in jbi
    # Bookended home BC.
    assert f"BC00000={SAFE_HOME_BC}" in jbi
    assert jbi.count("BC00000=") == 1


def test_cad_seam_detection(lbracket):
    res = detect_seams(lbracket, angle_tol_deg=25, target_angle_deg=90)
    assert res["seam_count"] >= 1
    assert res["total_length"] > 0
    # Fillet segments carry face normals + outward bisector.
    seg = res["segments"][0]
    assert "n1" in seg and "n2" in seg and "bisector" in seg
    n1 = np.asarray(seg["n1"], dtype=float)
    n2 = np.asarray(seg["n2"], dtype=float)
    bis = np.asarray(seg["bisector"], dtype=float)
    assert abs(np.linalg.norm(n1) - 1.0) < 1e-6
    assert abs(np.linalg.norm(n2) - 1.0) < 1e-6
    assert abs(np.linalg.norm(bis) - 1.0) < 1e-6
    expected = n1 + n2
    expected = expected / np.linalg.norm(expected)
    assert np.allclose(bis, expected, atol=1e-5)


def test_tfillet_bisector_approach_and_torch(tmp_path):
    """T-fillet (extruded L): approach offset and torch wire follow face bisector."""
    from app.path_planning import (
        plan_from_seams,
        seam_bisector,
        torch_dir_to_rpy_deg,
        torch_wire_direction,
    )

    # Concave 90° corner: faces with normals +Y and +X → bisector ≈ (0.707, 0.707, 0).
    poly = Polygon(
        [(0, 0), (0.3, 0), (0.3, 0.05), (0.05, 0.05), (0.05, 0.3), (0, 0.3)]
    )
    solid = trimesh.creation.extrude_polygon(poly, height=0.4)
    path = str(tmp_path / "tfillet.stl")
    solid.export(path)

    seams = detect_seams(path, angle_tol_deg=25, target_angle_deg=90)
    assert seams["seam_count"] >= 1
    seg = seams["segments"][0]
    bis = np.asarray(seg["bisector"], dtype=float)
    assert abs(np.linalg.norm(bis) - 1.0) < 1e-6
    assert bis[0] > 0.3 and bis[1] > 0.3
    assert abs(bis[2]) < 1e-6
    assert np.allclose(seg["n1"], [0.0, 1.0, 0.0], atol=1e-5) or np.allclose(
        seg["n2"], [0.0, 1.0, 0.0], atol=1e-5
    )

    travel = np.asarray(seg["end"], dtype=float) - np.asarray(seg["start"], dtype=float)
    wire = torch_wire_direction(bis, travel, travel_angle_deg=12.0, push=True)
    # Wire aims into the joint ≈ −bisector (within travel tilt).
    assert float(np.dot(wire, -bis)) > 0.85
    assert float(np.dot(wire, bis)) < 0.0

    standoff = 0.05
    planned = plan_from_seams(
        [seg],
        samples_per_seam=3,
        approach_height=standoff,
        travel_angle_deg=12.0,
        arc_file=1,
    )
    assert len(planned.points) >= 5
    approach = np.array(
        [planned.points[0]["x"], planned.points[0]["y"], planned.points[0]["z"]]
    )
    seam_start = np.asarray(seg["start"], dtype=float)
    delta = approach - seam_start
    # Approach is offset along +bisector, not world +Z alone.
    assert float(np.dot(delta / np.linalg.norm(delta), bis)) > 0.95
    assert abs(np.linalg.norm(delta) - standoff) < 1e-6
    assert abs(delta[2]) < 1e-6  # bisector is in XY → no pure +Z lift

    rx, ry, rz = torch_dir_to_rpy_deg(wire)
    assert planned.points[1]["rx"] == pytest.approx(rx, abs=1e-3)
    assert planned.points[1]["ry"] == pytest.approx(ry, abs=1e-3)
    assert planned.points[1]["rz"] == pytest.approx(rz, abs=1e-3)
    forced = plan_from_seams(
        [seg], samples_per_seam=2, torch_rpy_deg=(180.0, 0.0, 0.0)
    )
    assert forced.points[1]["rx"] == 180.0
    assert np.allclose(seam_bisector(seg), bis)


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
    assert "\r\n" in jbi
    assert "///POSTYPE PULSE" in jbi
    assert "///RECTAN" not in jbi
    # Home bookend is first/last.
    home = ",".join(str(p) for p in SAFE_HOME_PULSES)
    assert f"C00000={home}" in jbi
    assert f"BC00000={SAFE_HOME_BC}" in jbi
    npos = len(planned.points) + 2
    assert f"///NPOS {npos},{npos},{npos},0,0,0" in jbi
    path_bc = resolve_base_pulses(len(planned.points), points=planned.points)
    assert f"BC00001={path_bc[0]}" in jbi
    assert "MACRO1 MJ#(0) ARGF4" in jbi  # arc_file=2
    assert "ARCON" in jbi and jbi.strip().endswith("END")
    assert "RB1,BS1" in jbi


def test_robot_dof():
    assert get_robot().n == 6
