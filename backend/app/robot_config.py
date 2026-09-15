"""
Robot + external-axis kinematic configuration.

Production cell (``reference/robot-cell-dx200`` / SYSTEM.SYS):
  * Robot        : Yaskawa **MA2010** (MA02010-A0*) on DX200 ARC WELDING
  * Base         : RECT-X → INFORM **BS1**
  * Station      : TURN S1–S4 → INFORM **ST1** (jobs use GROUP2 ST1)
  * Power source : MOTOWELD-E Series 350A (see welding_config.py)

Pulse↔degree for MA2010 + TURN is mined in ``pulse_config.py`` from RC.PRM /
RBCALIB. DH link lengths remain approximate until verified against MotoSim.

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


@dataclass
class RobotModel:
    name: str
    reach_mm: float
    payload_kg: float
    joints: list[JointDH] = field(default_factory=list)  # order: S,L,U,R,B,T
    # Pulses-per-degree per axis from the DX200 parameter file. PLACEHOLDER.
    pulses_per_degree: list[float] | None = None
    approximate: bool = True  # True => link lengths are placeholders


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
    """Build an AR-topology 6R arm from coarse dimensions (all placeholders)."""
    lim = [(np.deg2rad(a), np.deg2rad(b)) for a, b in limits_deg]
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
        pulses_per_degree=[1000.0] * 6,  # PLACEHOLDER: read from DX200 params
        approximate=True,
    )


# ---------------------------------------------------------------------------
# AR-series catalogue. Reach & joint ranges follow the public specs; link
# lengths (d1,a1,a2,a3,d4,d6) are APPROXIMATE and must be confirmed per unit.
# Joint-range convention below (S,L,U,R,B,T) is the commonly cited AR range.
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
    # Production cell (SYSTEM.SYS): R1 = MA02010-A0*.
    # Joint qlim stays AR-catalogue (placeholder DH); pulses from RC1G dump.
    "MA2010": _ar_model("MA2010", 2010, 10, 0.540, 0.150, 0.760, 0.200, 1.082, 0.100, _AR_LIMITS),
    "AR3120": _ar_model("AR3120", 3124, 20, 0.650, 0.155, 1.150, 0.250, 1.412, 0.100, _AR_LIMITS),
}

# Override placeholder pulses for the production model (RC1G softlimits).
MODELS["MA2010"].pulses_per_degree = list(MA2010_PULSES_PER_DEGREE)
MODELS["MA2010"].approximate = True

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
