"""Quick offline smoke test of the backend modules (no server needed)."""
import numpy as np
import trimesh

from app.kinematics_engine import forward_kinematics, solve_ik_path, get_robot
from app.dx200_postprocessor import generate_jbi, WeldSegment, PostprocessorConfig
from app.cad_analysis import detect_seams
from app.pulse_config import CELL_PULSE, deg_to_joint_pulses
from app.robot_config import ACTIVE_MODEL, ACTIVE_POSITIONER, MODELS
from app.welding_config import ACTIVE_POWER_SOURCE, LORCH_S8_SCHEDULES


def test_config():
    print(f"Active robot: {ACTIVE_MODEL.name} "
          f"(reach {ACTIVE_MODEL.reach_mm} mm, payload {ACTIVE_MODEL.payload_kg} kg)")
    print(f"Available models: {', '.join(MODELS)}")
    print(f"Positioner: {ACTIVE_POSITIONER.name}, axes={len(ACTIVE_POSITIONER.axes)}")
    print(f"Power source: {ACTIVE_POWER_SOURCE.name}, "
          f"schedules={sorted(LORCH_S8_SCHEDULES)}")
    print(f"Pulse/deg S..T: {[round(p, 2) for p in CELL_PULSE.robot_pulses_per_degree]}")
    print(f"Station pulse/deg: {CELL_PULSE.station_pulses_per_degree:.4f}")
    assert ACTIVE_MODEL.name == "MA2010"
    assert ACTIVE_POSITIONER.name == "TURN-ST1"
    assert "MOTOWELD" in ACTIVE_POWER_SOURCE.name.upper()


def test_fk_ik_roundtrip():
    robot = get_robot()
    print(f"Robot DOF: {robot.n}")
    q = [10, -20, 15, 5, 30, 0]
    pose = forward_kinematics(q)
    print("FK pose:", {k: round(v, 4) for k, v in pose.items()})

    res = solve_ik_path([pose], q_seed_deg=q)
    print("IK all_reachable:", res["all_reachable"])
    sol = res["joint_angles_deg"][0]
    pose2 = forward_kinematics(sol)
    err = np.linalg.norm(
        np.array([pose[k] for k in ["x", "y", "z"]])
        - np.array([pose2[k] for k in ["x", "y", "z"]])
    )
    print(f"FK(IK(pose)) position error: {err*1000:.4f} mm")
    assert res["all_reachable"], "IK failed to converge"
    assert err < 1e-3, "Round-trip position error too large"


def test_jbi():
    pts = [
        {"x": 0.8, "y": 0.0, "z": 0.5, "rx": 180, "ry": 0, "rz": 0},
        {"x": 0.9, "y": 0.1, "z": 0.5, "rx": 180, "ry": 0, "rz": 0},
        {"x": 1.0, "y": 0.2, "z": 0.5, "rx": 180, "ry": 0, "rz": 0},
    ]
    segs = [WeldSegment(start_index=1, end_index=2, weld_speed=8.0, arc_file=3)]
    # Prefer real IK; if unreachable (placeholder DH), still exercise PULSE path.
    ik = solve_ik_path(pts)
    if ik["all_reachable"]:
        joints = ik["joint_angles_deg"]
    else:
        joints = [[10.0, -20.0, 15.0, 5.0, 30.0, 0.0] for _ in pts]
        print("IK unreachable for demo pts — using synthetic joints for PULSE demo")
    jbi = generate_jbi(
        pts,
        segs,
        PostprocessorConfig(job_name="WELD_AUTO"),
        station_deg=[0.0, 45.0, 90.0],
        joint_angles_deg=joints,
    )
    print("\n--- Generated JBI (MA2010 + TURN ST1 + Motoweld) ---")
    print(jbi)
    assert jbi.startswith("/JOB")
    assert "//NAME WELD_AUTO" in jbi
    assert "ARCON" in jbi and "ARCOF" in jbi
    assert "RB1,BS1" in jbi and "ST1" in jbi
    assert "+MOVJ EC" in jbi
    assert "Motoweld" in jbi
    assert "BC00000=878036" in jbi
    assert "///POSTYPE PULSE" in jbi
    assert "///RECTAN" not in jbi
    pulses = deg_to_joint_pulses(joints[0])
    assert f"C00000={','.join(str(p) for p in pulses)}" in jbi
    assert "EC00001=45886,0" in jbi
    assert jbi.strip().endswith("END")


def test_cad():
    # Single L-section solid -> one concave 90-deg inner edge (a fillet seam).
    from pathlib import Path
    from shapely.geometry import Polygon

    poly = Polygon(
        [(0, 0), (0.3, 0), (0.3, 0.05), (0.05, 0.05), (0.05, 0.3), (0, 0.3)]
    )
    solid = trimesh.creation.extrude_polygon(poly, height=0.4)
    path = Path(__file__).resolve().parent / "tests" / "fixtures" / "_smoke_lbracket.stl"
    path.parent.mkdir(parents=True, exist_ok=True)
    solid.export(str(path))
    res = detect_seams(str(path), angle_tol_deg=25, target_angle_deg=90)
    print("\nCAD seam_count:", res["seam_count"], "total_length:", round(res["total_length"], 4))
    assert res["seam_count"] > 0, "Expected to find fillet seam on the L inner edge"


if __name__ == "__main__":
    test_config()
    test_fk_ik_roundtrip()
    test_jbi()
    test_cad()
    print("\nALL SMOKE TESTS PASSED")
