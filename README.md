# Yaskawa CAD/CAM — komórka MA2010 / DX200

[![CI](https://github.com/MQCIA/yaskawa-cadcam/actions/workflows/ci.yml/badge.svg)](https://github.com/MQCIA/yaskawa-cadcam/actions/workflows/ci.yml)

Offline welding CAD/CAM for the **production MA2010 cell**: React-Three-Fiber
workspace, FastAPI kinematics + DX200 `.JBI` postprocessor, CAD seam detection.

**Roadmap:** [`docs/ROADMAP.md`](docs/ROADMAP.md) · **Cell:** [`docs/CELL.md`](docs/CELL.md) ·
**Go-live:** [`docs/GO_LIVE.md`](docs/GO_LIVE.md) · **Dump catalog:** [`docs/CELL_DUMP_CATALOG.md`](docs/CELL_DUMP_CATALOG.md)

**Configured cell** ([`docs/HARDWARE.md`](docs/HARDWARE.md)):
- Robot: **Yaskawa MA2010** (MA02010-A0*) on DX200 — `ROBOT_MODEL=MA2010`
- Base / stations: **RECT-X (BS1)** + **TURN ST1–ST4** — `POSITIONER=TURN-ST1`
- Power source: **MOTOWELD-E Series 350A** — `MACRO1 ARGF` + bare `ARCON` (no `ASF#`)
- Frames / IO / weld map: [`CELL_FRAMES.md`](docs/CELL_FRAMES.md) · [`CELL_IO.md`](docs/CELL_IO.md) · [`CELL_WELD_MAP.md`](docs/CELL_WELD_MAP.md) (when present)

> ## ⚠️ SAFETY — READ BEFORE REAL HARDWARE
>
> Generated `.JBI` is a **draft**. Incorrect output can cause collisions, fire, or injury.
>
> 1. FK/IK uses **ROS-Industrial MA2010 URDF × TOOL 0** — still **APP_ESTIMATE** until pendant XYZ sign-off ([`CELL_POSES.md`](docs/CELL_POSES.md)).
> 2. RECT-X mm↔pulse lead is **unverified** (`RECT_X_LEAD_VERIFIED=false`).
> 3. Follow **[`docs/GO_LIVE.md`](docs/GO_LIVE.md)**: `compare_pulses --check-golden` → MotoSim EG-VRC → DX200 TEACH+DRY RUN.
> 4. **Never** bypass curtains / FSU; **never** auto-load `.JBI` onto production DX200 from CI.

---

## Architecture

```
frontend/   Next.js 14 + TypeScript + Tailwind + R3F (MA2010 URDF + torch)
backend/    FastAPI + roboticstoolbox + trimesh
            URDF FK/IK, pulse_config, ma2010_cell PULSE postprocessor, seam detect
```

### Frontend
- 3D cell workspace, joint jog, seam highlight, Motoweld ARGF / WEV# controls
- Client-side demo JBI path (static Pages preview) + API export when backend is up

### Backend
- `POST /api/calculate-ik` — path IK seeded from cell home
- `POST /api/generate-jbi` — production **PULSE** when IK joints provided; otherwise opt-in `PREVIEW_RECTAN_DO_NOT_LOAD`
- `generate_station_master_jbi` — optional `STACJA_n` + `CALL JOB:` wrapper
- `POST /api/analyze-cad` — fillet seam heuristics (STL/…)

---

## Live preview (GitHub Pages)

**https://mqcia.github.io/yaskawa-cadcam/**

Static 3D / jog only. IK, CAD analysis, and controller-grade JBI need the FastAPI backend (`NEXT_PUBLIC_API_URL`).

## Quick start

```bash
docker compose up --build
# frontend -> http://localhost:3000   backend -> http://localhost:8000 (/docs)
```

```bash
# backend
cd backend && python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt requirements-dev.txt
uvicorn app.main:app --reload
python -m pytest -q

# frontend
cd frontend && npm install && npm run dev
```

Pipeline without server:

```bash
cd backend && source .venv/bin/activate && python pipeline_example.py
```

Golden pulse check:

```bash
python3 tools/compare_pulses.py --check-golden
```

---

## Honest status

| Area | Status |
|------|--------|
| MA2010 URDF FK/IK + TOOL 0 | In code; pendant XYZ sign-off open |
| PULSE export (IK → C#/BC#/EC#) | Hard-gated; no invented pulses |
| ST1–ST4 GROUP2 + zone BC | Done |
| Motoweld ARGF / WEV UI | See Motoweld PRs / `CELL_WELD_MAP.md` |
| IO / GO_LIVE | [`CELL_IO.md`](docs/CELL_IO.md), [`GO_LIVE.md`](docs/GO_LIVE.md) |
| RECT-X lead mm | Unverified placeholder |
| Auto DX200 load | **Out of scope** |

---

## Docs index

| Doc | Topic |
|-----|--------|
| [`docs/ROADMAP.md`](docs/ROADMAP.md) | Milestones M0–M6 |
| [`docs/GO_LIVE.md`](docs/GO_LIVE.md) | Offline → MotoSim → DX200 checklist |
| [`docs/CELL_IO.md`](docs/CELL_IO.md) | Curtains / stations / YSFLOGIC note |
| [`docs/CELL_TOUCHSENSE.md`](docs/CELL_TOUCHSENSE.md) | `SRCH-*` / `CALL JOB` pattern |
| [`docs/CELL_POSES.md`](docs/CELL_POSES.md) | Golden poses |
| [`docs/MODELS.md`](docs/MODELS.md) | URDF / meshes |
