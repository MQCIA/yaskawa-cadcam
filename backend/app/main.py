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

from .kinematics_engine import (
    forward_kinematics,
    forward_kinematics_frames,
    get_robot,
    solve_ik_path,
)
from .dx200_postprocessor import PostprocessorConfig, WeldSegment, generate_jbi
from .path_planning import plan_from_seams
from .robot_config import ACTIVE_MODEL, ACTIVE_POSITIONER, AXIS_NAMES, MODELS
from .robot_link import decode_motoros2_error, describe_robot_link
from .welding_config import (
    ACTIVE_POWER_SOURCE,
    LORCH_S8_SCHEDULES,
    TOUCH_SENSE,
    WEAVE_FILES,
)

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
    # Optional INFORM tags from the travel / spin planner.
    move_type: str | None = None
    kind: str | None = None
    bc: int | None = None
    vj: float | None = None
    ec_vj: float | None = None
    v: float | None = None
    dec: int | None = None
    ec: list[int] | None = None
    ec1: int | None = None
    ec2: int | None = None
    spin_deg: float | None = None


class IkRequest(BaseModel):
    points: list[TcpPoint]
    q_seed_deg: list[float] | None = Field(
        default=None, description="Optional 6-value seed pose in degrees."
    )
    frame: str = Field(
        default="tcp",
        description=(
            "MotoROS2 frame for poses: flange (REP-103), tool0 (Motoman Z+), "
            "or tcp (TOOL.CND tip). Path-planner samples usually use tool0."
        ),
    )


class WeldSegmentIn(BaseModel):
    start_index: int
    end_index: int
    weld_speed: float = 10.0
    arc_file: int = 1


class SeamIn(BaseModel):
    start: list[float]
    end: list[float]
    type: str | None = None
    move_type: str | None = None
    via: list[float] | None = None
    points: list[list[float]] | None = None
    bisector: list[float] | None = None
    n1: list[float] | None = None
    n2: list[float] | None = None
    weld_speed: float | None = None
    arc_file: int | None = None
    weave_no: int | None = None
    bc: int | None = None
    ec: list[int] | None = None
    ec1: int | None = None
    ec2: int | None = None
    station_deg: float | None = None


class PlanRequest(BaseModel):
    seams: list[SeamIn]
    samples_per_seam: int = 5
    approach_height: float = 0.05
    weld_speed: float = 10.0
    arc_file: int = 1
    optimize_spin: bool = True
    spin_step_deg: float = 30.0
    dense_step_m: float = 0.025
    travel_angle_deg: float = 12.0
    safe_travel: bool = True


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
            "DH link lengths are approximate; pulse scaling for MA2010 is from the "
            "DX200 CF dump (RC1G). Validate in MotoSim before controller load."
        ),
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
            "default_argf": ACTIVE_POWER_SOURCE.default_argf,
            "default_weave": ACTIVE_POWER_SOURCE.default_weave,
            "schedules": [
                {
                    "condition_no": s.condition_no,
                    "lorch_job": s.lorch_job,
                    "argf": s.lorch_job,
                    "current_a": s.current_a,
                    "voltage_v": s.voltage_v,
                    "wire_speed": s.wire_speed,
                    "travel_speed": s.travel_speed,
                    "default_weave": s.default_weave,
                    "arcsrt_en": s.arcsrt_en,
                    "process": s.process,
                    "gas": s.gas,
                    "note": s.note,
                }
                for s in LORCH_S8_SCHEDULES.values()
            ],
            "weaves": [
                {
                    "wev_no": w.wev_no,
                    "mode": w.mode,
                    "amp_l_mm": w.amp_l_mm,
                    "amp_r_mm": w.amp_r_mm,
                    "angle_l_deg": w.angle_l_deg,
                    "angle_r_deg": w.angle_r_deg,
                    "frequency_hz": w.frequency_hz,
                    "used_in_jobs": w.used_in_jobs,
                }
                for w in WEAVE_FILES.values()
            ],
            "touch_sense": {
                "opcode": TOUCH_SENSE.opcode,
                "search_speed_cm_min": TOUCH_SENSE.search_speed_cm_min,
                "station_vj": TOUCH_SENSE.station_vj,
                "reference_job": TOUCH_SENSE.reference_job,
            },
        },
        "warning": "Validate pulse jobs in MotoSim before loading the DX200.",
    }


@app.post("/api/calculate-ik")
def calculate_ik(req: IkRequest):
    if not req.points:
        raise HTTPException(400, "No points supplied")
    frame = (req.frame or "tcp").lower()
    if frame not in ("flange", "tool0", "tcp"):
        raise HTTPException(400, "frame must be flange, tool0, or tcp")
    points = [p.model_dump() for p in req.points]
    return solve_ik_path(points, q_seed_deg=req.q_seed_deg, frame=frame)  # type: ignore[arg-type]


@app.post("/api/plan-path")
def plan_path(req: PlanRequest):
    """Turn detected seams into an ordered TCP path + weld ranges.

    Also returns dense Cartesian samples and joint angles for smooth URDF
    animation, plus the optimised torch spin ``ψ`` about the wire axis.
    """
    if not req.seams:
        raise HTTPException(400, "No seams supplied")
    planned = plan_from_seams(
        [s.model_dump(exclude_none=True) for s in req.seams],
        samples_per_seam=req.samples_per_seam,
        approach_height=req.approach_height,
        weld_speed=req.weld_speed,
        arc_file=req.arc_file,
        optimize_spin=req.optimize_spin,
        spin_step_deg=req.spin_step_deg,
        dense_step_m=req.dense_step_m,
        travel_angle_deg=req.travel_angle_deg,
        safe_travel=req.safe_travel,
    )
    return {
        "points": planned.points,
        "weld_segments": [
            {
                "start_index": w.start_index,
                "end_index": w.end_index,
                "weld_speed": w.weld_speed,
                "arc_file": w.arc_file,
                "move_type": w.move_type,
                "weave_no": w.weave_no,
            }
            for w in planned.weld_segments
        ],
        "dense_points": planned.dense_points,
        "dense_joint_angles_deg": planned.dense_joint_angles_deg,
        "spin_angles_deg": planned.spin_angles_deg,
        "spin_results": [
            {
                "spin_deg": s.spin_deg,
                "cost": s.cost,
                "rejected": s.rejected,
                "reason": s.reason,
                "candidate_count": s.candidate_count,
                "feasible_count": s.feasible_count,
            }
            for s in planned.spin_results
        ],
        "max_joint_step_deg": planned.max_joint_step_deg,
        "bc_zones": planned.bc_zones,
        "axis_names": AXIS_NAMES,
    }


@app.post("/api/forward-kinematics")
def fk(q_deg: list[float], all_frames: bool = True):
    """FK for joint angles (deg).

    By default returns MotoROS2 frames ``flange``, ``tool0``, and ``tcp_0``
    (plus ``tcp`` alias) for ``/tf`` cross-checks. Pass ``all_frames=false``
    for a single ``tcp`` pose (legacy flat dict).
    """
    if len(q_deg) != get_robot().n:
        raise HTTPException(400, f"Expected {get_robot().n} joint values")
    if all_frames:
        return forward_kinematics_frames(q_deg)
    return forward_kinematics(q_deg)


@app.get("/api/motoros2-error/{error_code}")
def motoros2_error(error_code: int):
    """Decode FollowJointTrajectory ``error_code`` (−ECCCCC)."""
    return decode_motoros2_error(error_code)


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
    return generate_jbi(
        [p.model_dump(exclude_none=True) for p in req.points],
        segments,
        cfg,
        station_deg=req.station_deg,
        joint_angles_deg=req.joint_angles_deg,
    )


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


@app.get("/api/robot-link")
def robot_link():
    """Execution path for this cell: INFORM .JBI, plus optional MotoROS2 status.

    ArcNC does not expose an API. ROS is reported only when a micro-ROS Agent
    address is configured; this endpoint never commands motion. Motoweld
    ``WVON`` / ``MACRO1`` always go through native ``.JBI``.
    """
    return describe_robot_link()


@app.get("/")
def root():
    return {
        "service": "Yaskawa CAD/CAM prototype",
        "status": "ok",
        "disclaimer": "Prototype. Placeholder kinematics. Validate in MotoSim.",
        "robot_link": "/api/robot-link",
    }
