"""Quick offline smoke test of the backend modules (no server needed)."""
import numpy as np
import trimesh

from app.kinematics_engine import forward_kinematics, solve_ik_path, get_robot
from app.dx200_postprocessor import generate_jbi, WeldSegment, PostprocessorConfig
from app.cad_analysis import detect_seams


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
    jbi = generate_jbi(pts, segs, PostprocessorConfig(job_name="WELD_AUTO"))
    print("\n--- Generated JBI ---")
    print(jbi)
    assert jbi.startswith("/JOB")
    assert "//NAME WELD_AUTO" in jbi
    assert "ARCON" in jbi and "ARCOF" in jbi
    assert jbi.strip().endswith("END")


def test_cad():
    # Single L-section solid -> one concave 90-deg inner edge (a fillet seam).
    from shapely.geometry import Polygon

    poly = Polygon(
        [(0, 0), (0.3, 0), (0.3, 0.05), (0.05, 0.05), (0.05, 0.3), (0, 0.3)]
    )
    solid = trimesh.creation.extrude_polygon(poly, height=0.4)
    path = "/tmp/lbracket.stl"
    solid.export(path)
    res = detect_seams(path, angle_tol_deg=25, target_angle_deg=90)
    print("\nCAD seam_count:", res["seam_count"], "total_length:", round(res["total_length"], 4))
    assert res["seam_count"] > 0, "Expected to find fillet seam on the L inner edge"


if __name__ == "__main__":
    test_fk_ik_roundtrip()
    test_jbi()
    test_cad()
    print("\nALL SMOKE TESTS PASSED")
