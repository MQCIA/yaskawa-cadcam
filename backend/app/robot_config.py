"""
Robot + external-axis kinematic configuration.

Target cell:
  * Robot        : Yaskawa AR series (arc welding) on a DX200 controller
  * Positioner   : H1000D (external / station axis)
  * Power source : Lorch S8 (see welding_config.py)

============================================================================
  !!!  UWAGA / WARNING  -  CRITICAL SAFETY NOTICE  !!!
============================================================================
The Denavit-Hartenberg link lengths below are APPROXIMATE PLACEHOLDERS. They
reproduce the AR-series topology (shoulder + elbow-offset 6R arm) and use the
published *reach* and *joint ranges* per model, but the exact per-link lengths,
offsets, motor directions, home pose and pulse-per-degree constants are NOT
certified here and differ per unit.

Before this engine may drive anything real you MUST:
  1. Replace the numbers with the exact values from the Yaskawa data sheet /
     MotoSim model / DX200 parameter file for YOUR specific AR model.
  2. Verify FK of several known poses against the pendant read-out.
  3. Validate every path in MotoSim EG-VRC with collision checking, with a
     qualified integrator, before loading onto the controller.

Wrong numbers => wrong joint solutions => crashes, damage, injury.
============================================================================
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
import numpy as np


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
    "AR3120": _ar_model("AR3120", 3124, 20, 0.650, 0.155, 1.150, 0.250, 1.412, 0.100, _AR_LIMITS),
}

AXIS_NAMES = ["S", "L", "U", "R", "B", "T"]


# ---------------------------------------------------------------------------
# External axis: H1000D positioner (station / coordinated axis).
# Configure to match your actual unit: 1-axis (rotary headstock) or 2-axis
# (tilt + rotate). Defaults assume a single rotary axis, ~1000 kg class.
# ---------------------------------------------------------------------------
@dataclass
class ExternalAxis:
    name: str
    kind: str            # "rotary" or "linear"
    axis_label: str      # DX200 external/station axis label, e.g. "S1E"
    qlim_deg: tuple[float, float] = (-360.0, 360.0)
    pulses_per_degree: float = 1000.0  # PLACEHOLDER from DX200 param file


@dataclass
class Positioner:
    name: str
    payload_kg: float
    axes: list[ExternalAxis]
    approximate: bool = True


POSITIONER_H1000D = Positioner(
    name="H1000D",
    payload_kg=1000.0,  # nominal class; confirm against the rating plate
    axes=[
        ExternalAxis(
            name="H1000D-rotary",
            kind="rotary",
            axis_label="S1E",           # station axis 1; confirm on your DX200
            qlim_deg=(-360.0, 360.0),
            pulses_per_degree=1000.0,   # PLACEHOLDER
        ),
    ],
)


# ---------------------------------------------------------------------------
# Active selection (override with env vars, e.g. ROBOT_MODEL=AR2010).
# ---------------------------------------------------------------------------
def _select_model() -> RobotModel:
    key = os.getenv("ROBOT_MODEL", "AR1440").upper()
    if key not in MODELS:
        raise ValueError(
            f"Unknown ROBOT_MODEL '{key}'. Available: {', '.join(MODELS)}"
        )
    return MODELS[key]


ACTIVE_MODEL: RobotModel = _select_model()
ACTIVE_POSITIONER: Positioner = POSITIONER_H1000D
