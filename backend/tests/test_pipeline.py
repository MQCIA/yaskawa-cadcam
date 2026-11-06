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
    PulseExportError,
    PostprocessorConfig,
    WeldSegment,
    generate_jbi,
    generate_station_master_jbi,
    load_reference_jbi,
)
from app.cad_analysis import detect_seams
from app.path_planning import plan_from_seams
from app.robot_config import ACTIVE_MODEL, ACTIVE_POSITIONER, MODELS
from app.welding_config import ACTIVE_POWER_SOURCE, LORCH_S8_SCHEDULES
from app.pulse_config import (
    BC_HOME_PULSE,
    BC_PARK_000_PULSE,
    RECT_X_LEAD_VERIFIED,
    RECT_X_PULSES_PER_MM,
    ST1_BC_MAX,
    ST2_BC_RANGE,
    ST3_BC_RANGE,
    base_mm_to_bc,
    bc_to_base_mm,
)


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


def test_cell_stations_registry():
    from app.robot_config import CELL_STATIONS, TOOL0_XYZ_MM, cell_stations_public
    from app.main import cell_config

    assert TOOL0_XYZ_MM == (-54.280, 1.052, 415.702)
    assert set(CELL_STATIONS) == {"ST1", "ST2", "ST3", "ST4"}
    assert CELL_STATIONS["ST1"].ec_axes == 1
    assert CELL_STATIONS["ST2"].ec_axes == 1
    assert CELL_STATIONS["ST3"].ec_axes == 2
    assert CELL_STATIONS["ST4"].ec_axes == 2
    assert CELL_STATIONS["ST1"].default_bc == 927851
    assert CELL_STATIONS["ST4"].default_bc == 190240
    assert len(CELL_STATIONS["ST2"].mount_xyz_m) == 3
    pub = cell_stations_public()
    assert len(pub) == 4
    cfg = cell_config()
    assert cfg["uframe"]["file_present"] is False
    assert cfg["uframe"]["used_in_production_jobs"] is False
    assert {s["id"] for s in cfg["stations"]} == {"ST1", "ST2", "ST3", "ST4"}
    assert cfg["tool0"]["xyz_mm"] == list(TOOL0_XYZ_MM)
    frames = Path(__file__).resolve().parents[2] / "docs" / "CELL_FRAMES.md"
    assert frames.is_file()
    text = frames.read_text(encoding="utf-8")
    assert "TOOL 0" in text and "UFRAME.CND" in text and "ST1" in text


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


def test_rect_x_base_mm_bc_roundtrip():
    """M1.3: base_mm_to_bc / bc_to_base_mm around dump reference pulses."""
    assert RECT_X_LEAD_VERIFIED is False
    assert RECT_X_PULSES_PER_MM == 1000.0  # UNVERIFIED_PLACEHOLDER
    assert base_mm_to_bc(0.0) == BC_HOME_PULSE
    assert bc_to_base_mm(BC_HOME_PULSE) == pytest.approx(0.0)
    assert base_mm_to_bc(bc_to_base_mm(BC_PARK_000_PULSE)) == BC_PARK_000_PULSE
    assert base_mm_to_bc(10.0) == BC_HOME_PULSE + 10_000
    assert ST2_BC_RANGE == (124171, 375361)
    assert ST3_BC_RANGE == (808680, 847209)
    assert ST1_BC_MAX == 927851
    assert BC_PARK_000_PULSE == 878036


def test_jbi_ma2010_cell_profile():
    pts = [
        {"x": 0.8, "y": 0.0, "z": 0.5, "rx": 180},
        {"x": 0.9, "y": 0.1, "z": 0.5, "rx": 180},
        {"x": 1.0, "y": 0.2, "z": 0.5, "rx": 180},
    ]
    segs = [WeldSegment(0, 2, weld_speed=5.0, arc_file=3)]
    # Without IK → diagnostic RECTAN only (no invented PULSE C#).
    jbi = generate_jbi(
        pts,
        segs,
        PostprocessorConfig(allow_preview_rectan=True, home_bookends=False),
        station_deg=[0, 45, 90],
    )
    assert jbi.startswith("/JOB")
    assert "PREVIEW_RECTAN_DO_NOT_LOAD" in jbi
    assert "///RECTAN" in jbi
    assert "///NPOS 3,3,3,0,0,0" in jbi
    assert "///GROUP1 RB1,BS1" in jbi
    assert "///GROUP2 ST1" in jbi
    assert "ARCON" in jbi and "ARCOF" in jbi
    assert "ASF#(" not in jbi
    assert "WVON WEV#(21)" in jbi and "WVOF" in jbi
    assert "MACRO1 MJ#(0)" in jbi
    assert "+MOVJ EC" in jbi
    assert f"BC00000={BC_HOME_PULSE}" in jbi or "BC00000=" in jbi
    assert "EC00001=45886,0" in jbi  # 45 * 1019.6889…
    assert "UNVERIFIED_PLACEHOLDER" in jbi
    assert "Motoweld" in jbi
    assert jbi.strip().endswith("END")


def test_jbi_pulse_requires_ik_no_fake_pulses():
    pts = [{"x": 0.8, "y": 0.0, "z": 0.5, "rx": 180}]
    with pytest.raises(PulseExportError):
        generate_jbi(
            pts,
            config=PostprocessorConfig(prefer_pulse=True, home_bookends=False),
        )
    # Must not silently emit C00000=0,0,0,0,0,0
    preview = generate_jbi(
        pts,
        config=PostprocessorConfig(
            prefer_pulse=True, allow_preview_rectan=True, home_bookends=False
        ),
    )
    assert "///RECTAN" in preview
    assert "C00000=0,0,0,0,0,0" not in preview
    assert "PREVIEW_RECTAN_DO_NOT_LOAD" in preview


def test_jbi_pulse_from_joints():
    from app.pulse_config import deg_to_joint_pulses, station_deg_to_ec

    pts = [{"x": 0.8, "y": 0.0, "z": 0.5, "rx": 180}]
    q = [10.0, -20.0, 15.0, 5.0, 30.0, 0.0]
    jbi = generate_jbi(
        pts,
        config=PostprocessorConfig(job_name="PULSE_TEST", home_bookends=False),
        station_deg=[0.0],
        joint_angles_deg=[q],
    )
    pulses = deg_to_joint_pulses(q)
    assert "///POSTYPE PULSE" in jbi
    assert f"C00000={','.join(str(p) for p in pulses)}" in jbi
    assert "///RECTAN" not in jbi
    e1, e2 = station_deg_to_ec(0.0)
    assert f"EC00000={e1},{e2}" in jbi


def test_jbi_stations_st1_st4_group2_bc_home():
    """M2.2/M2.3: GROUP2 STx, dual EC#, zone BC, home BC=570125 bookends."""
    from app.robot_config import CELL_HOME_PULSES, CELL_STATIONS, cell_home_deg
    from app.pulse_config import deg_to_joint_pulses

    home_q = cell_home_deg(CELL_HOME_PULSES)
    work_q = [10.0, -20.0, 15.0, 5.0, 30.0, 0.0]
    pts = [
        {"x": 0.0, "y": 0.0, "z": 1.0, "rx": 180, "role": "home"},
        {"x": 0.8, "y": 0.0, "z": 0.5, "rx": 180, "role": "weld"},
        {"x": 0.0, "y": 0.0, "z": 1.0, "rx": 180, "role": "home"},
    ]
    segs = [WeldSegment(1, 1, weld_speed=5.0, arc_file=1)]
    joints = [home_q, work_q, home_q]
    for st_id, st in CELL_STATIONS.items():
        jbi = generate_jbi(
            pts,
            segs,
            PostprocessorConfig(
                job_name=f"WELD_{st_id}",
                station=st_id,  # type: ignore[arg-type]
                home_bookends=True,
            ),
            station_deg=[0.0, 10.0, 0.0],
            station_deg2=[0.0, 20.0, 0.0] if st.ec_axes >= 2 else None,
            joint_angles_deg=joints,
        )
        assert f"///GROUP2 {st_id}" in jbi
        assert "///GROUP1 RB1,BS1" in jbi
        assert "///NPOS 3,3,3,0,0,0" in jbi
        assert f"BC00000={BC_HOME_PULSE}" in jbi
        assert f"BC00002={BC_HOME_PULSE}" in jbi
        assert f"BC00001={st.default_bc}" in jbi
        assert re.search(r"EC00001=-?\d+,-?\d+", jbi)
        assert "ASF#(" not in jbi
        assert "+MOVJ EC" in jbi
        home_pulses = deg_to_joint_pulses(home_q)
        assert f"C00000={','.join(str(p) for p in home_pulses)}" in jbi


def test_multi_seam_nearest_neighbor_order():
    """M2.6: NN order + flip shortens air travel; home bookends present."""
    from app.path_planning import order_seams_nearest_neighbor, plan_from_seams

    seams = [
        {"start": [1.0, 0.0, 0.2], "end": [1.2, 0.0, 0.2]},
        {"start": [0.0, 0.0, 0.2], "end": [0.2, 0.0, 0.2]},
    ]
    ordered, idxs, flips = order_seams_nearest_neighbor(seams)
    assert len(ordered) == 2
    # Seed is cell-home TCP (~x=1.04 m) → starts at seam 0, then flips seam 1.
    assert idxs[0] == 0
    assert idxs[1] == 1
    assert flips[1] is True  # approach from previous end near x=1.2 → flip seam1
    # Tour length after NN ≤ naïve order air gap.
    def air(seq, fl):
        total = 0.0
        cur = None
        for s, f in zip(seq, fl):
            a = np.asarray(s["end"] if f else s["start"], float)
            b = np.asarray(s["start"] if f else s["end"], float)
            if cur is not None:
                total += float(np.linalg.norm(cur - a))
            cur = b
        return total

    naive = air(seams, [False, False])
    opt = air(ordered, flips)
    assert opt <= naive + 1e-9
    planned = plan_from_seams(seams, samples_per_seam=2, home_bookends=True)
    assert planned.points[0]["role"] == "home"
    assert planned.points[-1]["role"] == "home"
    assert len(planned.weld_segments) == 2
    # Retract of seam0 then approach of seam1 exist as air roles.
    roles = [p.get("role") for p in planned.points]
    assert roles.count("approach") == 2
    assert roles.count("retract") == 2


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


def test_cubeintf_named_stations_parse():
    from app.cubeintf import cube_contains_pulses, parse_cubeintf, path_cube_hits

    cubes = parse_cubeintf(CELL / "CUBEINTF.CND")
    named = [c for c in cubes if c.named]
    assert {c.name for c in named} >= {"STACJA_1", "STACJA_2"}
    st1 = next(c for c in named if c.name == "STACJA_1")
    mid = [(st1.lo[i] + st1.hi[i]) // 2 for i in range(6)]
    assert cube_contains_pulses(st1, mid)
    outside = list(mid)
    outside[0] = st1.hi[0] + 50_000
    assert not cube_contains_pulses(st1, outside)
    hits = path_cube_hits([mid], [st1])
    assert hits and hits[0]["cube"] == "STACJA_1"


def test_jbi_touchsense_call_before_arcon():
    """M2.4 — optional CALL JOB:SRCH-2D1 before first ARCON."""
    pts = [
        {"x": 0.8, "y": 0.0, "z": 0.5, "rx": 180},
        {"x": 0.9, "y": 0.1, "z": 0.5, "rx": 180},
    ]
    segs = [WeldSegment(0, 1, weld_speed=5.0, arc_file=1)]
    jbi = generate_jbi(
        pts,
        segs,
        PostprocessorConfig(
            allow_preview_rectan=True,
            home_bookends=False,
            enable_touchsense=True,
            touchsense_job="SRCH-2D1",
        ),
        station_deg=[0, 0],
    )
    assert "CALL JOB:SRCH-2D1" in jbi
    assert jbi.index("CALL JOB:SRCH-2D1") < jbi.index("ARCON")
    assert jbi.count("CALL JOB:SRCH-2D1") == 1


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


def test_generate_station_master_jbi():
    """M4.2 — optional STACJA_n wrapper with CALL JOB: child."""
    jbi = generate_station_master_jbi("STACJA_1", "WELD_AUTO", station="ST1")
    for token in (
        "/JOB",
        "//NAME STACJA_1",
        "///NPOS 0,0,0,0,0,0",
        "///ATTR SC,RW,CJ",
        "///GROUP1 RB1,BS1",
        "///GROUP2 ST1",
        "CALL JOB:WELD_AUTO",
        "END",
    ):
        assert token in jbi, f"missing {token!r}"
    assert "DOUT" not in jbi
    assert "SOUT" not in jbi
    # .JBI suffix stripped; invalid station rejected
    jbi2 = generate_station_master_jbi("STACJA_3.JBI", "CHILD.JBI", station="ST3")
    assert "//NAME STACJA_3" in jbi2
    assert "CALL JOB:CHILD" in jbi2
    assert "///GROUP2 ST3" in jbi2
    with pytest.raises(ValueError):
        generate_station_master_jbi("STACJA_X", "WELD", station="ST9")


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
    jbi = generate_jbi(
        planned.points,
        planned.weld_segments,
        PostprocessorConfig(),
        station_deg=[0.0] * len(planned.points),
        joint_angles_deg=ik["joint_angles_deg"],
    )
    assert "///POSTYPE PULSE" in jbi
    assert "///RECTAN" not in jbi
    assert "C00000=" in jbi and f"BC00000={BC_HOME_PULSE}" in jbi
    assert "ARCON" in jbi and jbi.strip().endswith("END")
    assert "RB1,BS1" in jbi
    assert "C00000=0,0,0,0,0,0" not in jbi


def test_robot_dof():
    assert get_robot().n == 6
