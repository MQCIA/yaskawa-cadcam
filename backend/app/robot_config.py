"""
Robot + external-axis kinematic configuration.

Production cell (``reference/robot-cell-dx200`` / SYSTEM.SYS):
  * Robot        : Yaskawa **MA2010** (MA02010-A0*) on DX200 ARC WELDING
  * Base         : RECT-X → INFORM **BS1**
  * Station      : TURN S1–S4 → INFORM **ST1** (jobs use GROUP2 ST1)
  * Power source : MOTOWELD-E Series 350A (see welding_config.py)

Pulse↔degree for MA2010 + TURN is mined in ``pulse_config.py`` from RC.PRM /
RBCALIB. MA2010 FK/IK uses the ROS-Industrial URDF chain (not placeholder DH).
Cartesian poses are still **APP_ESTIMATE** until ``golden_poses.json`` has
``verified_on_pendant: true``.

============================================================================
  !!!  UWAGA / WARNING  -  CRITICAL SAFETY NOTICE  !!!
============================================================================
Wrong numbers => wrong joint solutions => crashes, damage, injury.
Validate every path in MotoSim EG-VRC before loading the controller.
============================================================================
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
import numpy as np

from .pulse_config import (
    MA2010_LIMITS_DEG,
    MA2010_PULSES_PER_DEGREE,
    STATION_PULSES_PER_DEGREE,
)


@dataclass
class JointDH:
    """Revolute joint in standard (distal) DH parameters.

    a[m], alpha[rad], d[m], offset[rad] added to theta, qlim=(min,max)[rad].
    """

    a: float
    alpha: float
    d: float
    offset: float = 0.0
    qlim: tuple[float, float] = (-np.pi, np.pi)


@dataclass(frozen=True)
class Ma2010Geometry:
    """ROS-Industrial MA2010 URDF origins (metres).

    From ``frontend/public/models/ma2010/ma2010.urdf`` joint_1_s…joint_6_t
    + fixed ``joint_6_t-tool0`` (datasheet-aligned: d1=505, a1=150, a2=760,
    a3=200, d4=1082, d6=100 mm).
    """

    d1: float = 0.505
    a1: float = 0.150
    a2: float = 0.760
    a3: float = 0.200
    d4: float = 1.082
    d6: float = 0.100


MA2010_URDF = Ma2010Geometry()

# TOOL.CND ///TOOL 0 — Motoman tcp_0 relative to Motoman tool0 (Z+ out).
TOOL0_XYZ_MM: tuple[float, float, float] = (-54.280, 1.052, 415.702)
TOOL0_RPY_DEG: tuple[float, float, float] = (1.3119, -42.6215, -6.8387)
TOOL0_XYZ_M: tuple[float, float, float] = (
    TOOL0_XYZ_MM[0] / 1000.0,
    TOOL0_XYZ_MM[1] / 1000.0,
    TOOL0_XYZ_MM[2] / 1000.0,
)

# ROS-I flange (REP-103 X+ out) → Motoman tool0 (Z+ out). Columns = tool0 basis.
FLANGE_TO_TOOL0_R: tuple[tuple[float, float, float], ...] = (
    (0.0, 0.0, 1.0),
    (0.0, -1.0, 0.0),
    (1.0, 0.0, 0.0),
)

# Safe home bookend — POSE_01 / 02025_KOSZ… C00000 (see golden_poses.json).
CELL_HOME_PULSES: tuple[int, ...] = (-281, -101163, -84214, 3730, -80188, -1936)


def cell_home_deg(
    pulses: tuple[int, ...] | list[int] = CELL_HOME_PULSES,
    pulses_per_degree: list[float] | tuple[float, ...] | None = None,
) -> list[float]:
    """Home joint angles in degrees from taught pulses."""
    ppd = list(pulses_per_degree or MA2010_PULSES_PER_DEGREE)
    return [p / s for p, s in zip(pulses, ppd)]


@dataclass
class RobotModel:
    name: str
    reach_mm: float
    payload_kg: float
    joints: list[JointDH] = field(default_factory=list)  # order: S,L,U,R,B,T
    pulses_per_degree: list[float] | None = None
    # True => catalogue placeholder DH. MA2010 sets False (URDF lengths) but
    # Cartesian is still APP_ESTIMATE until pendant-verified golden poses exist.
    approximate: bool = True
    urdf_chain: bool = False
    home_deg: list[float] | None = None


def _limits_rad(limits_deg: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(np.deg2rad(a), np.deg2rad(b)) for a, b in limits_deg]


def _ar_model(
    name: str,
    reach_mm: float,
    payload_kg: float,
    d1: float,
    a1: float,
    a2: float,
    a3: float,
    d4: float,
    d6: float,
    limits_deg: list[tuple[float, float]],
) -> RobotModel:
    """Build an AR-topology 6R arm from coarse dimensions (placeholders)."""
    lim = _limits_rad(limits_deg)
    return RobotModel(
        name=name,
        reach_mm=reach_mm,
        payload_kg=payload_kg,
        joints=[
            JointDH(a1, -np.pi / 2, d1, 0.0, lim[0]),          # S
            JointDH(a2, 0.0, 0.0, -np.pi / 2, lim[1]),         # L
            JointDH(a3, -np.pi / 2, 0.0, 0.0, lim[2]),         # U
            JointDH(0.0, np.pi / 2, d4, 0.0, lim[3]),          # R
            JointDH(0.0, -np.pi / 2, 0.0, 0.0, lim[4]),        # B
            JointDH(0.0, 0.0, d6, 0.0, lim[5]),                # T
        ],
        pulses_per_degree=[1000.0] * 6,
        approximate=True,
        urdf_chain=False,
    )


def _ma2010_model() -> RobotModel:
    """Production MA2010: URDF lengths/axes, RC1G pulses, cell-home IK seed."""
    lim = _limits_rad(MA2010_LIMITS_DEG)
    g = MA2010_URDF
    joints = [
        JointDH(g.a1, -np.pi / 2, g.d1, 0.0, lim[0]),
        JointDH(g.a2, 0.0, 0.0, -np.pi / 2, lim[1]),
        JointDH(g.a3, -np.pi / 2, 0.0, 0.0, lim[2]),
        JointDH(0.0, np.pi / 2, g.d4, 0.0, lim[3]),
        JointDH(0.0, -np.pi / 2, 0.0, 0.0, lim[4]),
        JointDH(0.0, 0.0, g.d6, 0.0, lim[5]),
    ]
    return RobotModel(
        name="MA2010",
        reach_mm=2010,
        payload_kg=10,
        joints=joints,
        pulses_per_degree=list(MA2010_PULSES_PER_DEGREE),
        # Lengths match URDF; NOT pendant-certified (see golden_poses.json).
        approximate=False,
        urdf_chain=True,
        home_deg=cell_home_deg(),
    )


# ---------------------------------------------------------------------------
# AR-series catalogue. Reach & joint ranges follow the public specs; link
# lengths (d1,a1,a2,a3,d4,d6) are APPROXIMATE except MA2010 (URDF).
# ---------------------------------------------------------------------------
_AR_LIMITS = [
    (-170, 170),   # S
    (-90, 155),    # L
    (-175, 250),   # U
    (-200, 200),   # R
    (-150, 150),   # B
    (-455, 455),   # T
]

MODELS: dict[str, RobotModel] = {
    "AR900": _ar_model("AR900", 927, 6, 0.330, 0.040, 0.445, 0.040, 0.440, 0.080, _AR_LIMITS),
    "AR1440": _ar_model("AR1440", 1440, 6, 0.505, 0.155, 0.614, 0.200, 0.640, 0.100, _AR_LIMITS),
    "AR1730": _ar_model("AR1730", 1730, 8, 0.540, 0.160, 0.760, 0.200, 0.780, 0.100, _AR_LIMITS),
    "AR2010": _ar_model("AR2010", 2010, 12, 0.540, 0.150, 0.760, 0.200, 1.082, 0.100, _AR_LIMITS),
    "MA2010": _ma2010_model(),
    "AR3120": _ar_model("AR3120", 3124, 20, 0.650, 0.155, 1.150, 0.250, 1.412, 0.100, _AR_LIMITS),
}

AXIS_NAMES = ["S", "L", "U", "R", "B", "T"]


# ---------------------------------------------------------------------------
# Cell stations ST1–ST4 (M2.1) — mined from production PULSE jobs.
# Cartesian mount / AABB are APP_ESTIMATE (URDF FK × TOOL 0), not pendant.
# See docs/CELL_FRAMES.md. No UFRAME inventing: dump has no UFRAME.CND and
# jobs never use ///USER n.
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class CellStation:
    """One TURN station on the RECT-X rail (GROUP2 STn)."""

    id: str
    group2: str
    # Active EC# components: 1 → EC=val,0 (TURN-1); 2 → EC=val1,val2 (TURN-2).
    ec_axes: int
    default_bc: int
    bc_range: tuple[int, int]
    ec1_range: tuple[int, int]
    ec2_range: tuple[int, int]
    # Weld-cloud centre (metres, robot/base frame) from ARCON→MOV* TCP FK.
    mount_xyz_m: tuple[float, float, float]
    aabb_xyz_mm: tuple[tuple[float, float, float], tuple[float, float, float]]
    source_jobs: tuple[str, ...]
    weld_point_count: int
    notes: str = ""


CELL_STATIONS: dict[str, CellStation] = {
    "ST1": CellStation(
        id="ST1",
        group2="ST1",
        ec_axes=1,
        default_bc=927851,
        bc_range=(927851, 927851),
        ec1_range=(1163, 1163),
        ec2_range=(0, 0),
        mount_xyz_m=(-0.1499, 1.7606, 1.0101),
        aabb_xyz_mm=((-149.9, 1760.6, 1010.1), (-149.9, 1760.6, 1010.1)),
        source_jobs=("11.JBI",),
        weld_point_count=1,
        notes="TURN-1; single weld MOVL in 11.JBI",
    ),
    "ST2": CellStation(
        id="ST2",
        group2="ST2",
        ec_axes=1,
        default_bc=250331,
        bc_range=(124171, 375361),
        ec1_range=(-933, 120166),
        ec2_range=(0, 0),
        mount_xyz_m=(0.0508, 1.6563, 0.6338),
        aabb_xyz_mm=((-915.8, 817.3, 201.5), (833.7, 2216.6, 1406.9)),
        source_jobs=(
            "12.JBI",
            "02025_KOSZ_DO_RURY_70-PLUS-RAMKI.JBI",
            "123123.JBI",
        ),
        weld_point_count=66,
        notes="TURN-1; 123123.JBI has no ARCON (excluded from weld cloud)",
    ),
    "ST3": CellStation(
        id="ST3",
        group2="ST3",
        ec_axes=2,
        default_bc=808680,
        bc_range=(808679, 808680),
        ec1_range=(-2638, 119164),
        ec2_range=(-726117, 167367),
        mount_xyz_m=(0.3633, -1.5604, 0.4048),
        aabb_xyz_mm=((22.0, -1807.8, 326.7), (776.5, -1207.8, 535.2)),
        source_jobs=("1-DEFRO-5.JBI", "1-DEFRO-5-KOSZ.JBI", "123.JBI"),
        weld_point_count=9,
        notes="TURN-2; 123.JBI has no ARCON (excluded from weld cloud)",
    ),
    "ST4": CellStation(
        id="ST4",
        group2="ST4",
        ec_axes=2,
        default_bc=190240,
        bc_range=(190240, 190240),
        ec1_range=(-2, -2),
        ec2_range=(-1, -1),
        mount_xyz_m=(0.6996, -1.2730, 0.9618),
        aabb_xyz_mm=((699.6, -1273.0, 961.8), (699.6, -1273.0, 961.8)),
        source_jobs=("1KT.JBI",),
        weld_point_count=1,
        notes="TURN-2; EC near zero in 1KT.JBI sample",
    ),
}


def cell_stations_public() -> list[dict]:
    """JSON-friendly CELL_STATIONS for /api/config."""
    out: list[dict] = []
    for st in CELL_STATIONS.values():
        out.append(
            {
                "id": st.id,
                "group2": st.group2,
                "ec_axes": st.ec_axes,
                "default_bc": st.default_bc,
                "bc_range": list(st.bc_range),
                "ec1_range": list(st.ec1_range),
                "ec2_range": list(st.ec2_range),
                "mount_xyz_m": list(st.mount_xyz_m),
                "aabb_xyz_mm": {
                    "min": list(st.aabb_xyz_mm[0]),
                    "max": list(st.aabb_xyz_mm[1]),
                },
                "source_jobs": list(st.source_jobs),
                "weld_point_count": st.weld_point_count,
                "notes": st.notes,
                "cartesian_status": "APP_ESTIMATE_UNVERIFIED_ON_PENDANT",
            }
        )
    return out


# ---------------------------------------------------------------------------
# External axes: production TURN station + optional H1000D placeholder.
# ---------------------------------------------------------------------------
@dataclass
class ExternalAxis:
    name: str
    kind: str            # "rotary" or "linear"
    axis_label: str      # DX200 external/station axis label, e.g. "ST1"
    qlim_deg: tuple[float, float] = (-360.0, 360.0)
    pulses_per_degree: float = 1000.0


@dataclass
class Positioner:
    name: str
    payload_kg: float
    axes: list[ExternalAxis]
    approximate: bool = True


# SYSTEM.SYS: S1/S2 TURN-1, S3/S4 TURN-2. Jobs coordinate via GROUP2 ST1.
POSITIONER_TURN_ST1 = Positioner(
    name="TURN-ST1",
    payload_kg=0.0,
    axes=[
        ExternalAxis(
            name="TURN-1",
            kind="rotary",
            axis_label="ST1",
            qlim_deg=(-360.0, 360.0),
            pulses_per_degree=STATION_PULSES_PER_DEGREE,
        ),
    ],
    approximate=False,  # pulses from RBCALIB; mechanical limits still cell-specific
)

POSITIONER_H1000D = Positioner(
    name="H1000D",
    payload_kg=1000.0,
    axes=[
        ExternalAxis(
            name="H1000D-rotary",
            kind="rotary",
            axis_label="ST1",
            qlim_deg=(-360.0, 360.0),
            pulses_per_degree=STATION_PULSES_PER_DEGREE,
        ),
    ],
)


# ---------------------------------------------------------------------------
# Active selection (override with env vars, e.g. ROBOT_MODEL=AR2010).
# ---------------------------------------------------------------------------
def _select_model() -> RobotModel:
    # Default MA2010 matches the production DX200 dump (SYSTEM.SYS R1: MA02010-A0*).
    key = os.getenv("ROBOT_MODEL", "MA2010").upper()
    if key not in MODELS:
        raise ValueError(
            f"Unknown ROBOT_MODEL '{key}'. Available: {', '.join(MODELS)}"
        )
    return MODELS[key]


def _select_positioner() -> Positioner:
    key = os.getenv("POSITIONER", "TURN-ST1").upper().replace("_", "-")
    if key in ("TURN-ST1", "TURN", "ST1"):
        return POSITIONER_TURN_ST1
    if key in ("H1000D", "H1000"):
        return POSITIONER_H1000D
    raise ValueError(f"Unknown POSITIONER '{key}'. Available: TURN-ST1, H1000D")


ACTIVE_MODEL: RobotModel = _select_model()
ACTIVE_POSITIONER: Positioner = _select_positioner()
