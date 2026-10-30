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
from .dx200_postprocessor import PostprocessorConfig, WeldSegment, generate_jbi
from .path_planning import plan_from_seams
from .pulse_config import (
    BASE_PULSES_PER_MM,
    BC_HOME_PULSE,
    BC_PARK_PULSE,
    PULSES_PER_MM,
    resolve_base_pulses,
)
from .robot_config import (
    ACTIVE_MODEL,
    ACTIVE_POSITIONER,
    AXIS_NAMES,
    MODELS,
    TOOL0_RPY_DEG,
    TOOL0_XYZ_MM,
)
from .welding_config import ACTIVE_POWER_SOURCE, MOTOWELD_SCHEDULES

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
    station: str | None = Field(
        default=None,
        description="Optional ST1..ST4 — suggests a fixed BC midpoint for all points.",
    )
    base_mm: list[float] | float | None = Field(
        default=None,
        description="Optional RECT-X travel mm from home (scalar or per-point).",
    )
    base_pulse: list[int] | int | None = Field(
        default=None,
        description="Optional explicit BC# pulses (scalar or per-point).",
    )


class WeldSegmentIn(BaseModel):
    start_index: int
    end_index: int
    weld_speed: float = 10.0
    arc_file: int = 1


class SeamIn(BaseModel):
    start: list[float]
    end: list[float]
    # Optional fillet geometry from /api/analyze-cad.
    n1: list[float] | None = None
    n2: list[float] | None = None
    bisector: list[float] | None = None
    dihedral_deg: float | None = None
    # Optional per-seam Motoweld condition (MACRO1 ARGF via postprocessor).
    arc_file: int | None = None
    weld_speed: float | None = None


class PlanRequest(BaseModel):
    seams: list[SeamIn]
    samples_per_seam: int = 5
    approach_height: float = 0.05
    travel_angle_deg: float = 12.0
    push: bool = True
    weld_speed: float = 10.0
    arc_file: int = 1
    max_seams: int = 50
    optimize: bool = True
    clearance_height: float = 0.08


class JbiRequest(BaseModel):
    points: list[TcpPoint]
    weld_segments: list[WeldSegmentIn] = []
    job_name: str = "WELD_AUTO"
    tool_no: int = 0
    move_speed: float = 10.0
    station_deg: list[float] | None = Field(
        default=None, description="Optional per-point TURN ST1 angle (deg)."
    )
    joint_angles_deg: list[list[float]] | None = Field(
        default=None,
        description="Optional per-point [S,L,U,R,B,T] deg — enables PULSE C# export.",
    )
    station: str | None = Field(
        default=None,
        description="Optional ST1..ST4 — use taught BC midpoint for all BC#.",
    )
    base_mm: list[float] | float | None = Field(
        default=None,
        description="RECT-X mm from home (scalar or per-point). Overrides auto Z.",
    )
    base_pulse: list[int] | int | None = Field(
        default=None,
        description="Explicit BC# pulses (scalar or per-point). Highest priority.",
    )
    macro_arg: int | None = Field(
        default=None,
        description="MACRO1 MJ#(0) ARGFn override (production: 1, 4, 5, …).",
    )
    weave_no: int | None = Field(
        default=None,
        description="WVON WEV#(n); omit to use schedule/default. Set disable_weave.",
    )
    disable_weave: bool = False
    bookend_home: bool | None = Field(
        default=None,
        description="Wrap path with safe home C/BC (default on when PULSE joints).",
    )
    group2_station: str | None = Field(
        default=None,
        description="///GROUP2 STn (ST1..ST4). Defaults to station or ST1.",
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
        "rect_x": {
            "pulses_per_mm": PULSES_PER_MM,
            "bc_home": BC_HOME_PULSE,
            "bc_park": BC_PARK_PULSE,
            "note": (
                "No rack lead/gear in RC1G text dump; calibrated from taught BC "
                "station ranges vs 2400 mm cell span. Encoder SV1G=4096."
            ),
        },
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
            "cell_label": ACTIVE_POWER_SOURCE.cell_label,
            "schedules": [
                {
                    "condition_no": s.condition_no,
                    "argf": s.argf,
                    "current_a": s.current_a,
                    "voltage_v": s.voltage_v,
                    "wire_speed": s.wire_speed,
                    "travel_speed": s.travel_speed,
                    "process": s.process,
                    "gas": s.gas,
                    "argf_source": s.argf_source,
                    "params_source": s.params_source,
                }
                for s in MOTOWELD_SCHEDULES.values()
            ],
        },
        "warning": "Validate pulse jobs in MotoSim before loading the DX200.",
    }


@app.post("/api/calculate-ik")
def calculate_ik(req: IkRequest):
    if not req.points:
        raise HTTPException(400, "No points supplied")
    points = [p.model_dump() for p in req.points]
    result = solve_ik_path(points, q_seed_deg=req.q_seed_deg)
    try:
        base_pulses = resolve_base_pulses(
            len(points),
            base_pulse=req.base_pulse,
            base_mm=req.base_mm,
            points=points,
            station=req.station,
        )
    except ValueError as exc:
        raise HTTPException(400, str(exc)) from exc
    result["base_pulses"] = base_pulses
    result["base_pulses_per_mm"] = BASE_PULSES_PER_MM
    result["bc_home"] = BC_HOME_PULSE
    result["bc_park"] = BC_PARK_PULSE
    return result


@app.post("/api/plan-path")
def plan_path(req: PlanRequest):
    """Turn detected seams into an ordered TCP path + weld ranges."""
    if not req.seams:
        raise HTTPException(400, "No seams supplied")
    planned = plan_from_seams(
        [s.model_dump() for s in req.seams],
        samples_per_seam=req.samples_per_seam,
        approach_height=req.approach_height,
        travel_angle_deg=req.travel_angle_deg,
        push=req.push,
        weld_speed=req.weld_speed,
        arc_file=req.arc_file,
        max_seams=req.max_seams,
        optimize=req.optimize,
        clearance_height=req.clearance_height,
    )
    return {
        "points": planned.points,
        "weld_segments": [
            {"start_index": w.start_index, "end_index": w.end_index,
             "weld_speed": w.weld_speed, "arc_file": w.arc_file}
            for w in planned.weld_segments
        ],
        "seam_order": [
            {"index": i, "reversed": rev} for i, rev in planned.seam_order
        ],
        "air_length_m": planned.air_length_m,
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
    group2 = req.group2_station or req.station
    cfg = PostprocessorConfig(
        job_name=req.job_name,
        tool_no=req.tool_no,
        move_speed=req.move_speed,
        group2_station=(group2 or "ST1").upper().replace(" ", ""),
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
            joint_angles_deg=req.joint_angles_deg,
            base_pulse=req.base_pulse,
            base_mm=req.base_mm,
            station=req.station,
            macro_arg=req.macro_arg,
            weave_no=req.weave_no,
            bookend_home=req.bookend_home,
            disable_weave=req.disable_weave,
        )
    except ValueError as exc:
        raise HTTPException(400, str(exc)) from exc


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
