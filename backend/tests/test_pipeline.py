"""Pytest suite covering the backend modules and the full pipeline."""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np
import pytest
import trimesh
from shapely.geometry import Polygon

from app.kinematics_engine import (
    forward_kinematics,
    forward_kinematics_frames,
    solve_ik_path,
    get_robot,
)
from app.dx200_postprocessor import (
    generate_jbi,
    WeldSegment,
    PostprocessorConfig,
    load_reference_jbi,
)
from app.cad_analysis import detect_seams
from app.path_planning import plan_from_seams
from app.pulse_config import joint_pulses_to_deg
from app.robot_config import (
    ACTIVE_MODEL,
    ACTIVE_POSITIONER,
    CELL_HOME_PULSES,
    MODELS,
    TOOL0_LENGTH_MM,
    TOOL0_XYZ_MM,
)
from app.welding_config import (
    ACTIVE_POWER_SOURCE,
    ARCSRT_TEMPLATE,
    LORCH_S8_SCHEDULES,
    MOTOWELD_ARGF_SCHEDULES,
    TOUCH_SENSE,
    WEAVE_FILES,
    get_schedule_by_argf,
    get_weave,
    wire_for_current,
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
    assert "LORCH" in ACTIVE_POWER_SOURCE.name.upper()
    assert "MOTOWELD" in ACTIVE_POWER_SOURCE.welder_dat_label.upper()
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
    assert "BC00000=878036" in jbi
    assert "EC00001=45886,0" in jbi  # 45 * 1019.6889…
    assert "Lorch Tiptronic" in jbi or "ARGF" in jbi
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
    # Planner poses are Motoman tip / wire frames (no TOOL.CND RPY).
    ik = solve_ik_path(planned.points, frame="tool0")
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
    assert "C00000=" in jbi and "BC00000=878036" in jbi
    assert "ARCON" in jbi and jbi.strip().endswith("END")
    assert "RB1,BS1" in jbi


def test_merge_collinear_and_arc_grouping():
    """Collinear glue + constant-radius chords → type=arc (MOVC pipe pattern)."""
    from app.cad_analysis import group_constant_radius_arcs, merge_collinear_seams

    # Three collinear 100 mm edges → one line.
    col = [
        {"start": [0.0, 0.0, 0.0], "end": [0.1, 0.0, 0.0], "length": 0.1},
        {"start": [0.1, 0.0, 0.0], "end": [0.2, 0.0, 0.0], "length": 0.1},
        {"start": [0.2, 0.0, 0.0], "end": [0.3, 0.0, 0.0], "length": 0.1},
    ]
    merged = merge_collinear_seams(col)
    assert len(merged) == 1
    assert merged[0]["type"] == "line"
    assert merged[0]["length"] == pytest.approx(0.3, rel=1e-6)

    # Ø70 mm tube (R=35 mm) — short chords like tessellated fillet around pipe.
    r = 0.035
    n = 12
    chords: list[dict] = []
    for i in range(n):
        a0 = 2 * np.pi * i / n
        a1 = 2 * np.pi * (i + 1) / n
        p0 = [r * np.cos(a0), r * np.sin(a0), 0.0]
        p1 = [r * np.cos(a1), r * np.sin(a1), 0.0]
        chords.append(
            {
                "start": p0,
                "end": p1,
                "length": float(np.linalg.norm(np.array(p1) - np.array(p0))),
            }
        )
    grouped = group_constant_radius_arcs(chords, min_edges=3)
    arcs = [s for s in grouped if s.get("type") == "arc"]
    assert len(arcs) >= 1
    arc = arcs[0]
    assert arc["move_type"] == "MOVC"
    assert "via" in arc and "points" in arc
    assert len(arc["points"]) >= 3
    assert arc["radius"] == pytest.approx(r, rel=0.15)


def test_jbi_movc_and_per_point_ec():
    """MOVC for arc welds (12/123.JBI) + variable EC (1-DEFRO-5 ST3 pattern)."""
    # Three circular weld samples → MOVC in //INST.
    pts = [
        {"x": 0.8, "y": 0.0, "z": 0.55, "rx": 180, "move_type": "MOVJ"},
        {"x": 0.835, "y": 0.0, "z": 0.5, "rx": 180, "move_type": "MOVC"},
        {"x": 0.8, "y": 0.035, "z": 0.5, "rx": 180, "move_type": "MOVC"},
        {"x": 0.765, "y": 0.0, "z": 0.5, "rx": 180, "move_type": "MOVC"},
        {"x": 0.8, "y": 0.0, "z": 0.55, "rx": 180, "move_type": "MOVJ"},
    ]
    segs = [WeldSegment(1, 3, weld_speed=5.0, arc_file=1, move_type="MOVC")]
    jbi = generate_jbi(pts, segs, PostprocessorConfig(), station_deg=[0.0] * 5)
    assert "MOVC C00001 BC00001 V=5.0 DEC=20  +MOVJ EC00001" in jbi
    assert "MOVC C00002" in jbi
    assert "MOVC C00003" in jbi
    # Air approach stays MOVJ.
    assert "MOVJ C00000" in jbi

    # ST3 circumferential: identical TCP intent, EC axis-2 rotates
    # (1-DEFRO-5.JBI: EC00004=-2638,-94527 → EC00005=-2638,-726117).
    circ = [
        {"x": 0.9, "y": 0.5, "z": 0.4, "rx": 180},
        {"x": 0.9, "y": 0.5, "z": 0.4, "rx": 180},
    ]
    ec = [(-2638, -94527), (-2638, -726117)]
    jbi_ec = generate_jbi(
        circ,
        [WeldSegment(0, 1, weld_speed=4.2, arc_file=1, weave_no=0)],
        PostprocessorConfig(macro_arg=4),
        station_deg=[0.0, 0.0],
        ec_pulses=ec,
    )
    assert "EC00000=-2638,-94527" in jbi_ec
    assert "EC00001=-2638,-726117" in jbi_ec

    # Point-embedded ec= also works.
    pts_ec = [
        {"x": 0.9, "y": 0.5, "z": 0.4, "rx": 180, "ec": [-2638, -94527]},
        {"x": 0.9, "y": 0.5, "z": 0.4, "rx": 180, "ec": [-2638, -726117]},
    ]
    jbi_pt = generate_jbi(
        pts_ec,
        [WeldSegment(0, 1, weld_speed=4.2, arc_file=1)],
        PostprocessorConfig(),
    )
    assert "EC00000=-2638,-94527" in jbi_pt
    assert "EC00001=-2638,-726117" in jbi_pt

    # Planner tags arc seams as MOVC.
    planned = plan_from_seams(
        [
            {
                "type": "arc",
                "move_type": "MOVC",
                "start": [0.035, 0.0, 0.0],
                "via": [0.0, 0.035, 0.0],
                "end": [-0.035, 0.0, 0.0],
                "points": [
                    [0.035, 0.0, 0.0],
                    [0.0, 0.035, 0.0],
                    [-0.035, 0.0, 0.0],
                ],
                "radius": 0.035,
            }
        ],
        samples_per_seam=3,
    )
    assert planned.weld_segments[0].move_type == "MOVC"
    weld_moves = [
        p.get("move_type")
        for p in planned.points[planned.weld_segments[0].start_index : planned.weld_segments[0].end_index + 1]
    ]
    assert all(m == "MOVC" for m in weld_moves)
    assert len(weld_moves) >= 3
    jbi_plan = generate_jbi(
        planned.points,
        planned.weld_segments,
        PostprocessorConfig(),
        station_deg=[0.0] * len(planned.points),
    )
    movc_lines = [ln for ln in jbi_plan.splitlines() if ln.startswith("MOVC ")]
    assert len(movc_lines) >= 3


def test_production_movc_and_defro_ec_fixtures():
    """Fixtures uploaded from the cell dump encode the taught patterns."""
    j12 = (CELL / "12.JBI").read_text(encoding="utf-8", errors="replace")
    assert "MOVC C00013" in j12
    assert "MOVC C00014" in j12
    assert "MOVC C00015" in j12
    j123 = (CELL / "123.JBI").read_text(encoding="utf-8", errors="replace")
    assert j123.count("MOVC ") >= 3
    defro = (CELL / "1-DEFRO-5.JBI").read_text(encoding="utf-8", errors="replace")
    assert "///GROUP2 ST3" in defro
    assert "C00004=-83555,68320,-11474,26050,-31610,-59171" in defro
    assert "C00005=-83555,68320,-11474,26050,-31610,-59171" in defro
    assert "EC00004=-2638,-94527" in defro
    assert "EC00005=-2638,-726117" in defro


def test_robot_dof():
    assert get_robot().n == 6


def test_motoweld_argf_and_arcsrt_template():
    """M3: ARGF map + shared ARCSRT 200 A / 20 V template from dump."""
    assert ARCSRT_TEMPLATE.current_a == 200.0
    assert ARCSRT_TEMPLATE.voltage_v == 20.0
    assert set(LORCH_S8_SCHEDULES) >= {1, 2, 3}
    assert set(MOTOWELD_ARGF_SCHEDULES) >= {1, 4, 5, 8}
    sch8 = get_schedule_by_argf(8)
    assert sch8.lorch_job == 8
    assert sch8.current_a == ARCSRT_TEMPLATE.current_a
    assert sch8.default_weave == 21
    assert abs(wire_for_current(200) - 7.2) < 0.15
    # Fixture ARCSRT EN blocks are identical templates
    arcsrt = (CELL / "ARCSRT.CND").read_text(encoding="latin-1", errors="replace")
    assert arcsrt.count("//ARCSRT EN ") == 1000
    assert "200,20.0,100" in arcsrt


def test_weave_files_from_weav_cnd():
    """M3: WEV#(1/15/16/21) amplitudes match WEAV.CND."""
    assert get_weave(1).amp_l_mm == 2.0 and get_weave(1).angle_l_deg == 70.0
    assert get_weave(1).dwell_s[0] == 0.1
    assert get_weave(15).amp_l_mm == 3.0 and get_weave(15).frequency_hz == 3.0
    assert get_weave(15).dwell_s[5] == 0.3
    assert get_weave(16).angle_r_deg == 0.0
    assert get_weave(21).amp_l_mm == 4.0 and get_weave(21).amp_r_mm == 0.0
    weav = (CELL / "WEAV.CND").read_text(encoding="latin-1", errors="replace")
    assert "//WEAV 21" in weav
    assert set(WEAVE_FILES) == {1, 15, 16, 21}


def test_touchsense_movs_pattern_from_123123():
    """M3: TouchSense search uses MOVS V=11.0 +MOVJ EC VJ=0.78 (123123.JBI)."""
    job = (CELL / "123123.JBI").read_text(encoding="latin-1", errors="replace")
    assert "MOVS C00001 BC00001 V=11.0  +MOVJ EC00001 VJ=0.78" in job
    assert TOUCH_SENSE.opcode == "MOVS"
    assert TOUCH_SENSE.search_speed_cm_min == 11.0
    assert TOUCH_SENSE.station_vj == 0.78
    macro = (CELL / "MACRO.DAT").read_text(encoding="latin-1", errors="replace")
    assert "//MACRO1 1" in macro and "PROG-NR:" in macro


def test_tool0_frame_chain_consistency():
    """FE↔BE contract: flange→tool0→tcp_0 length is dump TOOL 0, JBI uses ///TOOL 0.

    Golden poses from ``docs/CELL_POSES.md`` (home C00000 + ``000.JBI`` bookends).
    Euclidean ‖tcp_0 − tool0‖ must equal
    √((-54.280)² + 1.052² + 415.702²) ≈ 419.232 mm for every S..T pose.
    """
    expected_len_mm = float(np.linalg.norm(np.asarray(TOOL0_XYZ_MM, dtype=float)))
    assert TOOL0_LENGTH_MM == pytest.approx(expected_len_mm, abs=1e-9)
    assert expected_len_mm == pytest.approx(419.231, abs=0.002)

    home_deg = joint_pulses_to_deg(list(CELL_HOME_PULSES))
    # Slight rounding from user brief (−0.21, −53.03, −52.87, +3.63, −81.80, −4.26).
    assert home_deg[0] == pytest.approx(-0.21, abs=0.01)
    assert home_deg[1] == pytest.approx(-53.03, abs=0.01)
    assert home_deg[4] == pytest.approx(-81.80, abs=0.01)

    # HOME_TRACK fixture + selected 000.JBI points (docs/CELL_POSES.md).
    golden_pulses = [
        list(CELL_HOME_PULSES),
        [95902, -19559, -65353, -68228, -62443, 70439],  # 000 C00000 approach
        [96643, 1036, -73382, -77097, -56952, 78162],  # 000 C00002
        [144285, 13854, -56584, 51164, -49608, 19486],  # 000 C00005
        [116865, -4881, -33684, -3254, -73880, -61311],  # 000 C00019
        [0, 0, 0, 0, 0, 0],
        [10.0, -20.0, 15.0, 5.0, 30.0, 0.0],  # deg directly below
    ]
    # Last entry is already degrees — handle separately.
    poses_deg = [joint_pulses_to_deg(p) for p in golden_pulses[:-1]]
    poses_deg.append(list(golden_pulses[-1]))

    for q in poses_deg:
        frames = forward_kinematics_frames(q)
        for key in ("flange", "tool0", "tcp_0"):
            assert set(frames[key]) >= {"x", "y", "z", "rx", "ry", "rz"}
        t0 = np.array([frames["tool0"][k] for k in "xyz"], dtype=float)
        tcp = np.array([frames["tcp_0"][k] for k in "xyz"], dtype=float)
        dist_mm = float(np.linalg.norm(tcp - t0) * 1000.0)
        assert dist_mm == pytest.approx(expected_len_mm, abs=1e-6), (
            f"tool0→tcp_0 length {dist_mm:.6f} mm ≠ {expected_len_mm:.6f} at q={q}"
        )
        # Origins of flange and tool0 coincide (static rotation only).
        fl = np.array([frames["flange"][k] for k in "xyz"], dtype=float)
        assert np.linalg.norm(fl - t0) == pytest.approx(0.0, abs=1e-9)

    # Generated .JBI always declares ///TOOL 0.
    pts = [
        {"x": 0.8, "y": 0.0, "z": 0.5, "rx": 180.0, "ry": 0.0, "rz": 0.0},
        {"x": 0.9, "y": 0.1, "z": 0.5, "rx": 180.0, "ry": 0.0, "rz": 0.0},
    ]
    jbi = generate_jbi(
        pts,
        [WeldSegment(0, 1)],
        PostprocessorConfig(),
        station_deg=[0.0, 0.0],
        joint_angles_deg=[home_deg, home_deg],
    )
    assert "///TOOL 0" in jbi
    assert f"C00000={','.join(str(p) for p in CELL_HOME_PULSES)}" in jbi

    home_job = (CELL / "HOME_TRACK.JBI").read_text(encoding="utf-8", errors="replace")
    assert "///TOOL 0" in home_job
    assert "C00000=-281,-101163,-84214,3730,-80188,-1936" in home_job
