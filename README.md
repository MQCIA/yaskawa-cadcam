# Yaskawa CAD/CAM Prototype (DX200 · AR/MA series)

A **prototype** welding CAD/CAM web application for Yaskawa 6-axis welding
robots: a 3D workspace (React-Three-Fiber), a kinematics backend
(FastAPI + roboticstoolbox), a Yaskawa DX200 `.JBI` postprocessor, and a CAD
seam-detection endpoint.

> ## ⚠️ SAFETY — READ BEFORE DOING ANYTHING WITH REAL HARDWARE
>
> This project **cannot** and **does not** guarantee "immediate, error-free"
> operation on a physical robot, and no software honestly can. It drives an
> industrial arc-welding manipulator; incorrect output causes collisions,
> equipment damage, fire and injury.
>
> **The kinematic parameters shipped here are generic PLACEHOLDERS**, not the
> certified values of any specific robot. Before this is anything more than a
> visualiser you MUST:
> 1. Replace the DH parameters, joint limits, home pose and pulse constants in
>    `backend/app/robot_config.py` with the **exact** figures for your robot
>    (from the Yaskawa data sheet / MotoSim model / controller parameter file).
> 2. Validate the DX200 `.JBI` format against a reference job exported from
>    **your** controller (RCONF word, TOOL/USER frames, weld-condition file
>    numbers, speed tags all differ per cell).
> 3. Dry-run **every** generated path in **MotoSim EG-VRC** with collision
>    checking, and have a qualified Yaskawa integrator sign off, before loading
>    onto a live controller.
>
> Treat all generated code as a draft for expert review, never as
> ready-to-run.

---

## Architecture

```
frontend/   Next.js 14 (App Router) + TypeScript + Tailwind
            React-Three-Fiber 3D workspace, axis jog sliders, CAD upload
backend/    FastAPI + roboticstoolbox-python + spatialmath + trimesh
            kinematics engine, DX200 postprocessor, CAD seam detection
```

### Phase 1 — 3D frontend (`frontend/`)
- `components/RobotWorkspace.tsx` — `<Canvas>` with `OrbitControls`, an
  infinite `Grid` floor, and multi-source lighting.
- `components/YaskawaManipulator.tsx` — a **nested** kinematic chain
  (base → S → L → U → R → B → T); each link is a child of the previous so a
  joint rotates everything above it. Procedural primitives stand in for the
  real geometry; swap in `useGLTF` per-link `.glb` files using the documented
  pattern.
- `components/AxisSliders.tsx` — per-axis sliders bound to React state.

### Phase 2 — Kinematics backend (`backend/app/kinematics_engine.py`)
- Builds a `roboticstoolbox` `DHRobot` from `robot_config.ACTIVE_MODEL`.
- `POST /api/calculate-ik` — takes an ordered list of TCP poses
  `{x,y,z,rx,ry,rz}` and returns the 6 joint angles per point, using the
  Levenberg–Marquardt numeric IK solver seeded with the previous solution
  (continuous configuration along the path).

### Phase 3 — DX200 postprocessor (`backend/app/dx200_postprocessor.py`)
- Converts a path into INFORM III `.JBI` with the requested header
  (`/JOB`, `//NAME`, `//POS`, `///NPOS`, `///TOOL`, `///POSTYPE ROBOT`,
  `///RECTAN`, `///RCONF`), a `//INST` + `///DATE` section, `MOVL` moves with
  `V=` speeds, `ARCON`/`ARCOF` around detected seams, ending with `END`.
- `POST /api/generate-jbi`.

### Phase 4 — CAD seam detection (`backend/app/cad_analysis.py`)
- `POST /api/analyze-cad` — loads a mesh and returns candidate fillet seams:
  internal edges with a dihedral angle near 90° that are **concave**. Returns
  3D start/end segments for the frontend to highlight in red.
- STL/OBJ/PLY/GLB load natively; **STEP** needs an extra converter
  (`pip install cascadio`, or a `gmsh` backend). B-Rep-accurate seam
  recognition really needs OpenCASCADE (`pythonocc-core`) — see limitations.

---

## Running locally

### Backend
```bash
cd backend
python -m virtualenv .venv && source .venv/bin/activate   # or python -m venv
pip install -r requirements.txt
uvicorn app.main:app --reload            # http://localhost:8000  (/docs for Swagger)
python smoke_test.py                     # offline verification of all modules
```

### Frontend
```bash
cd frontend
npm install
npm run dev                              # http://localhost:3000
# set NEXT_PUBLIC_API_URL if the backend is not on http://localhost:8000
```

---

## Known limitations / honest status
- **Kinematics are placeholder DH parameters** — approximate a ~1.4 m welding
  arm topology, not any real MA/AR model. Not verified against MotoSim.
- **IK** is unconstrained numeric IK: no collision avoidance, singularity
  handling, reach/limit strategy, or turn-number (RCONF) management.
- **`.JBI`** structure is plausible but unvalidated against a real DX200;
  pulse↔degree scaling, weld schedules and frames are placeholders.
- **STEP** import is optional and not wired by default; seam detection is a
  tessellation heuristic on single solids — multi-body contact detection needs
  a real CAD kernel.
- No authentication, persistence, path smoothing, or reachability planning.

These are the pieces that must be filled in with your real data and validated
in simulation before any production use.
