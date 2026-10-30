"""
Robot + external-axis kinematic configuration.

Production cell (``reference/robot-cell-dx200`` / SYSTEM.SYS):
  * Robot        : Yaskawa **MA2010** (MA02010-A0*) on DX200 ARC WELDING
  * Base         : RECT-X → INFORM **BS1**
  * Station      : TURN S1–S4 → INFORM **ST1** (jobs use GROUP2 ST1)
  * Power source : MOTOWELD-E Series 350A (see welding_config.py)

MA2010 link lengths and joint axes match ROS-Industrial
``motoman_ma2010_support`` URDF (vendored at ``frontend/public/models/ma2010``).
Pulse↔degree for MA2010 + TURN is mined in ``pulse_config.py`` from RC.PRM /
RBCALIB.

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

from .cell_frames import TOOL0
from .pulse_config import (
    MA2010_LIMITS_DEG,
    MA2010_PULSES_PER_DEGREE,
    STATION_BC_MID,
    STATION_PULSES_PER_DEGREE,
    track_pulses_to_mm,
)


@dataclass
class JointDH:
    """Revolute joint in standard (distal) DH parameters.

    a[m], alpha[rad], d[m], offset[rad] added to theta, qlim=(min,max)[rad].
    Used by the AR-series catalogue models. MA2010 uses URDF elementary
    transforms instead (see ``Ma2010Geometry``); its ``joints`` list only
    carries correct qlim for the API.
    """

    a: float
    alpha: float
    d: float
    offset: float = 0.0
    qlim: tuple[float, float] = (-np.pi, np.pi)


@dataclass(frozen=True)
class Ma2010Geometry:
    """ROS-Industrial MA2010 URDF origins / axes (metres).

    joint_1_s … joint_6_t + fixed ``joint_6_t-tool0`` from
    ``frontend/public/models/ma2010/ma2010.urdf``.
    """

    # Shoulder height (base_link → S).
    d1: float = 0.505
    # S → L along X.
    a1: float = 0.150
    # L → U along Z (upper-arm length).
    a2: float = 0.760
    # U → R along Z.
    a3: float = 0.200
    # R → B along X (forearm / reach to wrist pitch).
    d4: float = 1.082
    # B/T → flange face (URDF tool0).
    d6: float = 0.100


MA2010_URDF = Ma2010Geometry()

# Cell dump TOOL.CND — torch TCP relative to the flange (URDF tool0).
# Canonical registry: ``cell_frames.TOOL0`` / ``docs/CELL_FRAMES.md``.
TOOL0_XYZ_MM: tuple[float, float, float] = TOOL0.xyz_mm
TOOL0_RPY_DEG: tuple[float, float, float] = TOOL0.rpy_deg

# Safe home / transport over the rail — repeating C00000 in production jobs.
# Pulses: (-283, -101162, -84213, 3710, -80188, -1936). See docs/CELL_POSES.md.
CELL_HOME_PULSES: tuple[int, ...] = (-283, -101162, -84213, 3710, -80188, -1936)


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
    # Pulses-per-degree per axis from the DX200 parameter file.
    pulses_per_degree: list[float] | None = None
    approximate: bool = True  # True => link lengths are placeholders
    # When True, kinematics_engine builds an ERobot from MA2010_URDF axes.
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
    """Production MA2010: URDF lengths / axes, RC1G pulses, cell home seed."""
    lim = _limits_rad(MA2010_LIMITS_DEG)
    g = MA2010_URDF
    # JointDH rows are informational for /api/robot limits; FK/IK use ERobot.
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
        approximate=False,
        urdf_chain=True,
        home_deg=cell_home_deg(),
    )


# ---------------------------------------------------------------------------
# AR-series catalogue. Reach & joint ranges follow the public specs; link
# lengths (d1,a1,a2,a3,d4,d6) are APPROXIMATE and must be confirmed per unit.
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


# ---------------------------------------------------------------------------
# Station workspaces (M2) — weld TCP clouds in the robot-base frame + RECT-X.
#
# Computed by ``python tools/compare_pulses.py --analyze-stations`` over
# production GROUP2 jobs (fixtures ship ``000.JBI`` for ST1; full dump jobs
# ``11`` / ``12`` / ``02025_*`` / ``1-DEFRO-5*`` / ``1KT`` when present under
# ``reference/robot-cell-dx200/jobs/``).
#
# ``center_m`` / ``bbox_*`` are FK(MA2010)×TOOL0 metres in the robot base.
# ``rail_z_m = -track_pulses_to_mm(bc_mean)/1000`` (cell world, rail along Z).
# ``mount_m`` is the CAD / demo part origin for IK (≈ cloud centre, slightly
# below the torch tip so the coupon face sits under the seam).
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class StationFrame:
    name: str
    bc_mean: int
    bc_min: int
    bc_max: int
    center_m: tuple[float, float, float]
    bbox_min_m: tuple[float, float, float]
    bbox_max_m: tuple[float, float, float]
    rail_z_m: float
    mount_m: tuple[float, float, float]
    source_jobs: tuple[str, ...]
    approximate: bool = False


def _rail_z_from_bc(bc: int) -> float:
    return -track_pulses_to_mm(bc) / 1000.0


# ST1 workspace mined from fixture ``000.JBI`` (GROUP2 ST1, park BC).
_ST1_CENTER = (-0.0320, 1.3555, 0.5876)
_ST1_BBOX_MIN = (-0.6833, 0.8571, 0.3307)
_ST1_BBOX_MAX = (0.6880, 1.9002, 0.9089)
_ST1_MOUNT = (-0.0320, 1.3255, 0.5876)  # ~30 mm under seam centroid
_ST1_BC = 878037  # mean of work-point BC# in 000.JBI


def _frame_from_st1_cloud(
    name: str,
    bc_mean: int,
    bc_min: int,
    bc_max: int,
    source_jobs: tuple[str, ...],
    *,
    approximate: bool,
) -> StationFrame:
    """Reuse the ST1 relative weld cloud when a station has no C# in-repo yet."""
    return StationFrame(
        name=name,
        bc_mean=bc_mean,
        bc_min=bc_min,
        bc_max=bc_max,
        center_m=_ST1_CENTER,
        bbox_min_m=_ST1_BBOX_MIN,
        bbox_max_m=_ST1_BBOX_MAX,
        rail_z_m=_rail_z_from_bc(bc_mean),
        mount_m=_ST1_MOUNT,
        source_jobs=source_jobs,
        approximate=approximate,
    )


STATION_FRAMES: dict[str, StationFrame] = {
    "ST1": StationFrame(
        name="ST1",
        bc_mean=_ST1_BC,
        bc_min=878036,
        bc_max=878037,
        center_m=_ST1_CENTER,
        bbox_min_m=_ST1_BBOX_MIN,
        bbox_max_m=_ST1_BBOX_MAX,
        rail_z_m=_rail_z_from_bc(_ST1_BC),
        mount_m=_ST1_MOUNT,
        source_jobs=("000.JBI",),  # production twin: 11.JBI (gitignore dump)
        approximate=False,
    ),
    # ST2 / ST4: low-BC rail end. Cloud pending 12.JBI / 02025_* / 1KT.JBI.
    "ST2": _frame_from_st1_cloud(
        "ST2",
        bc_mean=STATION_BC_MID["ST2"],
        bc_min=STATION_BC_MID["ST2"],
        bc_max=STATION_BC_MID["ST2"],
        source_jobs=("12.JBI", "02025_KOSZ_DO_RURY_70-PLUS-RAMKI.JBI"),
        approximate=True,
    ),
    # ST3: high-BC end with ST1 (TURN-2). Cloud pending 1-DEFRO-5*.JBI.
    "ST3": _frame_from_st1_cloud(
        "ST3",
        bc_mean=STATION_BC_MID["ST3"],
        bc_min=STATION_BC_MID["ST3"],
        bc_max=STATION_BC_MID["ST3"],
        source_jobs=("1-DEFRO-5.JBI", "1-DEFRO-5-KOSZ.JBI"),
        approximate=True,
    ),
    "ST4": _frame_from_st1_cloud(
        "ST4",
        bc_mean=STATION_BC_MID["ST4"],
        bc_min=STATION_BC_MID["ST4"],
        bc_max=STATION_BC_MID["ST4"],
        source_jobs=("1KT.JBI", "BAZA1.JBI"),
        approximate=True,
    ),
}


def station_frame(name: str) -> StationFrame:
    key = name.upper().replace(" ", "")
    if key not in STATION_FRAMES:
        raise ValueError(f"Unknown station '{name}'. Use ST1..ST4.")
    return STATION_FRAMES[key]
