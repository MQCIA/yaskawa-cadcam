"""
FastAPI application: kinematics, DX200 postprocessing and CAD seam detection.

Run with:  uvicorn app.main:app --reload  (from the ``backend`` directory)
"""

from __future__ import annotations

import os
import tempfile

from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import PlainTextResponse
from pydantic import BaseModel, Field

from .kinematics_engine import forward_kinematics, get_robot, solve_ik_path
from .dx200_postprocessor import (
    PostprocessorConfig,
    PulseExportError,
    WeldSegment,
    generate_jbi,
)
from .path_planning import plan_from_seams
from .robot_config import (
    ACTIVE_MODEL,
    ACTIVE_POSITIONER,
    AXIS_NAMES,
    MODELS,
    TOOL0_RPY_DEG,
    TOOL0_XYZ_MM,
    cell_stations_public,
)
from .welding_config import ACTIVE_POWER_SOURCE, LORCH_S8_SCHEDULES
from .cubeintf import cubes_public, path_cube_hits, parse_cubeintf

app = FastAPI(
    title="Yaskawa CAD/CAM prototype API",
    version="0.1.0-prototype",
    description=(
        "PROTOTYPE ONLY. Kinematic parameters are placeholders. "
        "Never drive real hardware with this output without validation."
    ),
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # tighten for anything beyond local dev
    allow_methods=["*"],
    allow_headers=["*"],
)


# --------------------------------------------------------------------------- #
#  Schemas
# --------------------------------------------------------------------------- #
class TcpPoint(BaseModel):
    x: float
    y: float
    z: float
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0


class IkRequest(BaseModel):
    points: list[TcpPoint]
    q_seed_deg: list[float] | None = Field(
        default=None, description="Optional 6-value seed pose in degrees."
    )


class WeldSegmentIn(BaseModel):
    start_index: int
    end_index: int
    weld_speed: float = 10.0
    arc_file: int = 1


class SeamIn(BaseModel):
    start: list[float]
    end: list[float]


class PlanRequest(BaseModel):
    seams: list[SeamIn]
    samples_per_seam: int = 5
    approach_height: float = 0.05
    weld_speed: float = 10.0
    arc_file: int = 1


class JbiRequest(BaseModel):
    points: list[TcpPoint]
    weld_segments: list[WeldSegmentIn] = []
    job_name: str = "WELD_AUTO"
    tool_no: int = 0
    move_speed: float = 10.0
    station: str = Field(
        default="ST1",
        description="GROUP2 station: ST1 | ST2 | ST3 | ST4.",
    )
    station_deg: list[float] | None = Field(
        default=None, description="Optional per-point TURN axis-1 angle (deg)."
    )
    station_deg2: list[float] | None = Field(
        default=None,
        description="Optional per-point TURN axis-2 angle (deg) for ST3/ST4.",
    )
    joint_angles_deg: list[list[float]] | None = Field(
        default=None,
        description="Per-point [S,L,U,R,B,T] deg from IK — required for PULSE C#.",
    )
    allow_preview_rectan: bool = Field(
        default=False,
        description=(
            "If true and IK joints are missing, emit PREVIEW_RECTAN_DO_NOT_LOAD "
            "instead of failing. Never invents PULSE C#."
        ),
    )
    base_mm: list[float] | float | None = Field(
        default=None,
        description="RECT-X rail mm relative to BC_HOME (via base_mm_to_bc).",
    )
    home_bookends: bool = Field(
        default=True,
        description="Prepend/append cell-home C00000 / BC=570125 when missing.",
    )


# --------------------------------------------------------------------------- #
#  Endpoints
# --------------------------------------------------------------------------- #
@app.get("/api/robot")
def robot_info():
    robot = get_robot()
    return {
        "name": ACTIVE_MODEL.name,
        "reach_mm": ACTIVE_MODEL.reach_mm,
        "payload_kg": ACTIVE_MODEL.payload_kg,
        "dof": robot.n,
        "axis_names": AXIS_NAMES,
        "approximate_params": ACTIVE_MODEL.approximate,
        "joint_limits_deg": [
            [round(j.qlim[0] * 57.29577951308232, 3),
             round(j.qlim[1] * 57.29577951308232, 3)]
            for j in ACTIVE_MODEL.joints
        ],
        "pulses_per_degree": ACTIVE_MODEL.pulses_per_degree,
        "warning": (
            "MA2010 uses ROS-Industrial URDF lengths/axes + TOOL 0 from the cell dump; "
            "pulse scaling is RC1G. Other catalogue models still use placeholder DH. "
            "Validate in MotoSim before controller load."
        ),
        "tool0_xyz_mm": list(TOOL0_XYZ_MM),
        "tool0_rpy_deg": list(TOOL0_RPY_DEG),
        "home_deg": ACTIVE_MODEL.home_deg,
        "urdf_chain": ACTIVE_MODEL.urdf_chain,
    }


@app.get("/api/config")
def cell_config():
    """Full cell configuration: robot catalogue, positioner, power source."""
    return {
        "active_robot": ACTIVE_MODEL.name,
        "available_robots": [
            {"name": m.name, "reach_mm": m.reach_mm, "payload_kg": m.payload_kg}
            for m in MODELS.values()
        ],
        "positioner": {
            "name": ACTIVE_POSITIONER.name,
            "payload_kg": ACTIVE_POSITIONER.payload_kg,
            "axes": [
                {
                    "name": a.name,
                    "kind": a.kind,
                    "label": a.axis_label,
                    "qlim_deg": a.qlim_deg,
                    "pulses_per_degree": a.pulses_per_degree,
                }
                for a in ACTIVE_POSITIONER.axes
            ],
        },
        "power_source": {
            "name": ACTIVE_POWER_SOURCE.name,
            "interface": ACTIVE_POWER_SOURCE.interface,
            "schedules": [
                {"condition_no": s.condition_no, "lorch_job": s.lorch_job,
                 "current_a": s.current_a, "voltage_v": s.voltage_v,
                 "wire_speed": s.wire_speed, "process": s.process, "gas": s.gas}
                for s in LORCH_S8_SCHEDULES.values()
            ],
        },
        "tool0": {
            "xyz_mm": list(TOOL0_XYZ_MM),
            "rpy_deg": list(TOOL0_RPY_DEG),
            "source": "TOOL.CND ///TOOL 0 STANDARD TOOL11",
        },
        "uframe": {
            "file_present": False,
            "used_in_production_jobs": False,
            "note": (
                "UFRAME.CND absent from dump; jobs use POSTYPE PULSE + TOOL 0 "
                "without ///USER n — see docs/CELL_FRAMES.md"
            ),
        },
        "stations": cell_stations_public(),
        "cubeintf": cubes_public(),
        "warning": "Validate pulse jobs in MotoSim before loading the DX200.",
    }


class CubeCheckRequest(BaseModel):
    """Joint pulse rows (6 ints) to flag against named CUBEINTF soft cubes."""

    joint_pulses: list[list[int]] = Field(default_factory=list)
    named_only: bool = True


@app.post("/api/check-cubes")
def check_cubes(req: CubeCheckRequest):
    """Coarse soft-cube hit report (not FSU / not MotoSim)."""
    if not req.joint_pulses:
        raise HTTPException(400, "joint_pulses required")
    hits = path_cube_hits(req.joint_pulses, named_only=req.named_only)
    return {
        "hit_count": len(hits),
        "hits": hits,
        "cubes_loaded": len([c for c in parse_cubeintf() if c.named]),
        "note": "PULSE-space soft interference only — validate in MotoSim EG-VRC",
    }


@app.post("/api/calculate-ik")
def calculate_ik(req: IkRequest):
    if not req.points:
        raise HTTPException(400, "No points supplied")
    points = [p.model_dump() for p in req.points]
    return solve_ik_path(points, q_seed_deg=req.q_seed_deg)


@app.post("/api/plan-path")
def plan_path(req: PlanRequest):
    """Turn detected seams into an ordered TCP path + weld ranges."""
    if not req.seams:
        raise HTTPException(400, "No seams supplied")
    planned = plan_from_seams(
        [s.model_dump() for s in req.seams],
        samples_per_seam=req.samples_per_seam,
        approach_height=req.approach_height,
        weld_speed=req.weld_speed,
        arc_file=req.arc_file,
    )
    return {
        "points": planned.points,
        "weld_segments": [
            {"start_index": w.start_index, "end_index": w.end_index,
             "weld_speed": w.weld_speed, "arc_file": w.arc_file}
            for w in planned.weld_segments
        ],
    }


@app.post("/api/forward-kinematics")
def fk(q_deg: list[float]):
    if len(q_deg) != get_robot().n:
        raise HTTPException(400, f"Expected {get_robot().n} joint values")
    return forward_kinematics(q_deg)


@app.post("/api/generate-jbi", response_class=PlainTextResponse)
def generate_job(req: JbiRequest):
    if not req.points:
        raise HTTPException(400, "No points supplied")
    cfg = PostprocessorConfig(
        job_name=req.job_name,
        tool_no=req.tool_no,
        move_speed=req.move_speed,
        prefer_pulse=True,
        allow_preview_rectan=req.allow_preview_rectan,
        station=req.station,  # type: ignore[arg-type]
        home_bookends=req.home_bookends,
    )
    segments = [
        WeldSegment(
            start_index=s.start_index,
            end_index=s.end_index,
            weld_speed=s.weld_speed,
            arc_file=s.arc_file,
        )
        for s in req.weld_segments
    ]
    try:
        return generate_jbi(
            [p.model_dump() for p in req.points],
            segments,
            cfg,
            station_deg=req.station_deg,
            station_deg2=req.station_deg2,
            joint_angles_deg=req.joint_angles_deg,
            base_mm=req.base_mm,
        )
    except PulseExportError as exc:
        raise HTTPException(
            status_code=409,
            detail={
                "code": "PULSE_REQUIRES_IK",
                "message": str(exc),
                "hint": (
                    "Call /api/calculate-ik first; only export POSTYPE PULSE when "
                    "all_reachable=true. For diagnostics set allow_preview_rectan."
                ),
            },
        ) from exc


@app.post("/api/analyze-cad")
async def analyze_cad(
    file: UploadFile = File(...),
    angle_tol_deg: float = 20.0,
    target_angle_deg: float = 90.0,
):
    # Imported lazily so the rest of the API works even without trimesh present.
    from .cad_analysis import detect_seams

    suffix = os.path.splitext(file.filename or "")[1] or ".stl"
    with tempfile.NamedTemporaryFile(delete=False, suffix=suffix) as tmp:
        tmp.write(await file.read())
        tmp_path = tmp.name
    try:
        return detect_seams(
            tmp_path, angle_tol_deg=angle_tol_deg, target_angle_deg=target_angle_deg
        )
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(422, f"Could not analyze file: {exc}") from exc
    finally:
        os.unlink(tmp_path)


@app.get("/")
def root():
    return {
        "service": "Yaskawa CAD/CAM prototype",
        "status": "ok",
        "disclaimer": "Prototype. Placeholder kinematics. Validate in MotoSim.",
    }
