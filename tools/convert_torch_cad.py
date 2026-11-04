#!/usr/bin/env python3
"""Convert an ABIROB / iCAT torch CAD file to STL + GLB for the viewer.

Usage
-----
  python tools/convert_torch_cad.py path/to/torch.step \\
      --out-dir frontend/public/models/torch

Requires ``cascadio`` (STEP) and ``trimesh``. If only STL/OBJ/GLB is given,
re-exports optimized meshes and prints bbox / suggested tool0 align.

This script does **not** invent a TCP — after conversion, tip must still be
aligned to DX200 TOOL 0 (``robot_config.TOOL0_*``) in ``torchMesh.ts``.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def load_mesh(path: Path):
    import trimesh

    suffix = path.suffix.lower()
    if suffix in {".step", ".stp"}:
        try:
            import cascadio
        except ImportError as exc:  # noqa: BLE001
            raise SystemExit(
                "STEP conversion needs cascadio — pip install cascadio"
            ) from exc
        glb_tmp = path.with_suffix(".cascadio.glb")
        cascadio.step_to_glb(str(path), str(glb_tmp))
        mesh = trimesh.load(glb_tmp, force="mesh")
        try:
            glb_tmp.unlink()
        except OSError:
            pass
    else:
        mesh = trimesh.load(path, force="mesh")

    if isinstance(mesh, trimesh.Scene):
        mesh = mesh.dump(concatenate=True) if hasattr(mesh, "dump") else mesh.to_mesh()
    return mesh


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("cad", type=Path, help=".step/.stp/.stl/.obj/.glb")
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=Path("frontend/public/models/torch"),
    )
    ap.add_argument("--name", default="abirob_a300_eco")
    args = ap.parse_args()

    if not args.cad.is_file():
        print(f"CAD not found: {args.cad}", file=sys.stderr)
        print(
            "Place an ABIROB/iCAT STEP here and re-run.\n"
            "Reference download used in-repo:\n"
            "  https://d1d8a3050v4fu6.cloudfront.net/"
            "6a2e663ea257a80888c08dcb/6a2e663ea257a80888c08dcb.step",
            file=sys.stderr,
        )
        return 2

    mesh = load_mesh(args.cad)
    extents = mesh.extents
    # Heuristic: if max extent < 2 → likely metres (torch ≈ 0.4 m).
    units = "m" if float(extents.max()) < 2.0 else "mm"
    if units == "m":
        mesh.apply_scale(1000.0)
        units = "mm"

    args.out_dir.mkdir(parents=True, exist_ok=True)
    stl_path = args.out_dir / f"{args.name}.stl"
    glb_path = args.out_dir / f"{args.name}.glb"
    mesh.export(stl_path)
    try:
        mesh.export(glb_path)
    except Exception as exc:  # noqa: BLE001
        print(f"GLB export skipped: {exc}", file=sys.stderr)

    extents = mesh.extents
    bounds = mesh.bounds
    meta = {
        "source": str(args.cad),
        "units": units,
        "extents": extents.tolist(),
        "bounds_min": bounds[0].tolist(),
        "bounds_max": bounds[1].tolist(),
        "centroid": mesh.centroid.tolist(),
        "vertex_count": int(len(mesh.vertices)),
        "note": (
            "Align tip + wire to Motoman tcp_0 (TOOL0_XYZ_M / TOOL0_RPY_DEG) "
            "in torchMesh.ts; do not guess offsets."
        ),
        "tool0_tcp_mm": [-54.280, 1.052, 415.702],
        "tool0_rpy_deg": [1.3119, -42.6215, -6.8387],
    }
    (args.out_dir / f"{args.name}_meta.json").write_text(
        json.dumps(meta, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(meta, indent=2))
    print(f"Wrote {stl_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
