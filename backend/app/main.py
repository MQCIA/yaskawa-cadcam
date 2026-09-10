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
from .robot_config import ACTIVE_MODEL, AXIS_NAMES

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


class JbiRequest(BaseModel):
    points: list[TcpPoint]
    weld_segments: list[WeldSegmentIn] = []
    job_name: str = "WELD_AUTO"
    tool_no: int = 0
    move_speed: float = 10.0


# --------------------------------------------------------------------------- #
#  Endpoints
# --------------------------------------------------------------------------- #
@app.get("/api/robot")
def robot_info():
    robot = get_robot()
    return {
        "name": ACTIVE_MODEL.name,
        "dof": robot.n,
        "axis_names": AXIS_NAMES,
        "joint_limits_deg": [
            [round(j.qlim[0] * 57.29577951308232, 3),
             round(j.qlim[1] * 57.29577951308232, 3)]
            for j in ACTIVE_MODEL.joints
        ],
        "warning": (
            "Placeholder kinematics. Replace robot_config.ACTIVE_MODEL with "
            "certified values for your exact robot before any real use."
        ),
    }


@app.post("/api/calculate-ik")
def calculate_ik(req: IkRequest):
    if not req.points:
        raise HTTPException(400, "No points supplied")
    points = [p.model_dump() for p in req.points]
    return solve_ik_path(points, q_seed_deg=req.q_seed_deg)


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
        job_name=req.job_name, tool_no=req.tool_no, move_speed=req.move_speed
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
    return generate_jbi([p.model_dump() for p in req.points], segments, cfg)


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
