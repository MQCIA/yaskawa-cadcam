"""
End-to-end demo of the whole pipeline (no server required):

  CAD mesh  ->  seam detection  ->  path planning  ->  inverse kinematics
            ->  DX200 .JBI job (with Lorch S8 ARCON conditions)

Generates an L-bracket, runs every phase, and writes the resulting job to
``out/WELD_AUTO.JBI``. Run:  python pipeline_example.py
"""

from __future__ import annotations

import os

import trimesh
from shapely.geometry import Polygon

from app.cad_analysis import detect_seams
from app.path_planning import plan_from_seams
from app.kinematics_engine import solve_ik_path
from app.dx200_postprocessor import PostprocessorConfig, generate_jbi


def make_demo_part(path: str) -> None:
    """A single L-section solid -> one concave 90-degree fillet seam."""
    poly = Polygon(
        [(0, 0), (0.3, 0), (0.3, 0.05), (0.05, 0.05), (0.05, 0.3), (0, 0.3)]
    )
    solid = trimesh.creation.extrude_polygon(poly, height=0.4)
    # Place it in front of the robot so the path is roughly reachable.
    solid.apply_translation([0.7, -0.2, 0.2])
    solid.export(path)


def main() -> None:
    os.makedirs("out", exist_ok=True)
    part = "out/demo_lbracket.stl"
    make_demo_part(part)

    # Phase 4: seam detection
    seams = detect_seams(part, angle_tol_deg=25, target_angle_deg=90)
    print(f"[1] Seams detected: {seams['seam_count']} "
          f"(total {seams['total_length']:.3f} m)")

    # Path planning: seams -> ordered TCP path + weld ranges
    planned = plan_from_seams(seams["segments"], samples_per_seam=5, arc_file=2)
    print(f"[2] Path points: {len(planned.points)}, "
          f"weld segments: {len(planned.weld_segments)}, "
          f"welds: {[w.name for w in planned.welds]}")

    # Phase 2: inverse kinematics (weld TCP at 12 mm CTWD)
    weld_pts = [p for p in planned.points if p.get("kind") == "weld"]
    ik = solve_ik_path(weld_pts or planned.points)
    reached = sum(d["success"] for d in ik["diagnostics"])
    print(f"[3] IK solved: {reached}/{len(ik['diagnostics'])} points reachable "
          f"(all_reachable={ik['all_reachable']})")

    # Phase 3: DX200 .JBI (Cartesian RECTAN job with ARCON/ARCOF)
    jbi = generate_jbi(
        planned.points,
        planned.weld_segments,
        PostprocessorConfig(job_name="WELD_AUTO"),
    )
    out = "out/WELD_AUTO.JBI"
    with open(out, "w", encoding="utf-8") as f:
        f.write(jbi)
    print(f"[4] Wrote {out} ({len(jbi.splitlines())} lines)")
    print("\n--- first 20 lines ---")
    print("\n".join(jbi.splitlines()[:20]))
    print("\nNOTE: placeholder kinematics/params -- validate in MotoSim before use.")


if __name__ == "__main__":
    main()
