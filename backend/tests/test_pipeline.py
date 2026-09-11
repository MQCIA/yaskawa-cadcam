"""Pytest suite covering the backend modules and the full pipeline."""

from __future__ import annotations

import numpy as np
import pytest
import trimesh
from shapely.geometry import Polygon

from app.kinematics_engine import forward_kinematics, solve_ik_path, get_robot
from app.dx200_postprocessor import generate_jbi, WeldSegment, PostprocessorConfig
from app.cad_analysis import detect_seams
from app.path_planning import (
    plan_from_seams,
    CTWD_WELD_MM,
    CTWD_AIR_MM,
    SEED_STATES,
    ACTION_HOME,
    ACTION_TOUCH,
    ACTION_WELD,
    TOUCH_APPROACH_STATES,
    MAX_JOINT_CHANGE_DEG,
    UNWRAP_DEG,
)
from app.robot_config import ACTIVE_MODEL, ACTIVE_POSITIONER, MODELS
from app.welding_config import ACTIVE_POWER_SOURCE, LORCH_S8_SCHEDULES


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
    assert ACTIVE_MODEL.name.startswith("AR")
    assert ACTIVE_POSITIONER.name == "H1000D"
    assert ACTIVE_POWER_SOURCE.name == "Lorch S8"
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


def test_jbi_structure_with_station_and_weld():
    pts = [
        {"x": 0.8, "y": 0.0, "z": 0.5, "rx": 180},
        {"x": 0.9, "y": 0.1, "z": 0.5, "rx": 180},
        {"x": 1.0, "y": 0.2, "z": 0.5, "rx": 180},
    ]
    segs = [WeldSegment(0, 2, weld_speed=8.0, arc_file=3)]
    jbi = generate_jbi(pts, segs, PostprocessorConfig(), station_deg=[0, 45, 90])
    assert jbi.startswith("/JOB")
    assert "//NAME WELD_AUTO" in jbi
    assert "ARCON ASF#(3)" in jbi and "ARCOF" in jbi
    assert "SMOVL" in jbi and "ST1" in jbi
    assert "Lorch S8" in jbi
    assert jbi.strip().endswith("END")


def test_cad_seam_detection(lbracket):
    res = detect_seams(lbracket, angle_tol_deg=25, target_angle_deg=90)
    assert res["seam_count"] >= 1
    assert res["total_length"] > 0


def test_full_pipeline(lbracket):
    seams = detect_seams(lbracket, angle_tol_deg=25, target_angle_deg=90)
    planned = plan_from_seams(seams["segments"], samples_per_seam=5, arc_file=2)
    assert len(planned.points) > 0
    # Weld TCP (12 mm CTWD) must be reachable. Home/touch air poses are
    # collision-free placeholders; Verbotics also retries those separately.
    weld_pts = [p for p in planned.points if p.get("kind") == "weld"]
    ik = solve_ik_path(weld_pts or planned.points)
    assert ik["all_reachable"]
    jbi = generate_jbi(planned.points, planned.weld_segments, PostprocessorConfig())
    assert "ARCON" in jbi and jbi.strip().endswith("END")


def test_robot_dof():
    assert get_robot().n == 6


def test_verbotics_planner_from_logs():
    """Planner shape taken from the attached Weld 2026 session logs."""
    assert CTWD_WELD_MM == 12.0
    assert CTWD_AIR_MM == 0.0
    assert SEED_STATES == 5000
    assert (ACTION_HOME, ACTION_TOUCH, ACTION_WELD) == (1, 3, 5)
    assert TOUCH_APPROACH_STATES == 3
    assert MAX_JOINT_CHANGE_DEG == 175.0
    assert UNWRAP_DEG == 360.0

    seams = [
        {"start": [0.8, 0.2, 0.4], "end": [1.0, 0.2, 0.4]},
        {"start": [0.8, 0.25, 0.45], "end": [1.0, 0.25, 0.45]},
    ]
    planned = plan_from_seams(seams, samples_per_seam=4, arc_file=1)
    assert [w.name for w in planned.welds] == ["Weld 1-1", "Weld 1-2"]
    assert planned.ctwd_mm == 12.0
    assert planned.seed_states == 5000
    kinds = [p.get("kind") for p in planned.points]
    assert kinds[0] == "home"
    assert kinds.count("home") >= 3  # enter/leave each weld via home
    assert kinds.count("sense") == 6  # 3 touch states × 2 welds
    weld_pts = [p for p in planned.points if p.get("kind") == "weld"]
    assert weld_pts
    assert all(p.get("ctwd_mm") == 12.0 for p in weld_pts)
    assert all(p.get("action") == ACTION_WELD for p in weld_pts)
    assert any(p.get("tag") == "TOUCH" for p in planned.points)
    log = "\n".join(planned.log)
    assert "Seeded 5000 toolpath states" in log
    assert "Planning weld 'Weld 1-1" in log
    assert "going home" in log
    assert "Generating aligned touch planes" in log
    assert "Planning motion from home to touches" in log
