"""
Robot kinematic configuration.

============================================================================
  !!!  UWAGA / WARNING  -  CRITICAL SAFETY NOTICE  !!!
============================================================================
The Denavit-Hartenberg parameters below are GENERIC PLACEHOLDERS that only
*approximate* the geometry of a Yaskawa AR/MA-class 6-axis manipulator
(shoulder + elbow offset topology). They are NOT the certified parameters of
any specific robot and MUST NOT be used to move real hardware.

Before this engine may be trusted for anything beyond visualisation you MUST:
  1. Replace every value below with the exact figures from the official
     Yaskawa data sheet / MotoSim model for YOUR robot (e.g. MA1440,
     AR1440, AR2010, MA3120, ...). Link lengths, offsets, joint limits,
     joint directions and the zero (home) pose all differ per model.
  2. Verify the resulting model against MotoSim EG-VRC (compare forward
     kinematics of several known poses against the pendant read-out).
  3. Validate every generated path in simulation with collision checking
     BEFORE running it on a physical cell.

Getting any of these numbers wrong will produce wrong joint solutions,
which on a real welding cell means crashes, damaged equipment and injury.
============================================================================
"""

from dataclasses import dataclass, field
import numpy as np


@dataclass
class JointDH:
    """One revolute joint described with standard (distal) DH parameters.

    a      : link length            [m]
    alpha  : link twist             [rad]
    d      : link offset            [m]
    offset : constant added to the joint variable (theta) [rad]
    qlim   : (min, max) joint range [rad]
    """

    a: float
    alpha: float
    d: float
    offset: float = 0.0
    qlim: tuple[float, float] = (-np.pi, np.pi)


@dataclass
class RobotModel:
    name: str
    # Ordered S, L, U, R, B, T (Yaskawa axis naming).
    joints: list[JointDH] = field(default_factory=list)
    # Pulses-per-degree per axis, taken from the controller parameter file
    # (PPD / "pulse" values). PLACEHOLDER: must be read from your DX200.
    pulses_per_degree: list[float] | None = None


# ---------------------------------------------------------------------------
# PLACEHOLDER generic model. Replace with your certified values.
# The numbers loosely resemble a ~1.4 m reach welding arm but are NOT exact.
# ---------------------------------------------------------------------------
PLACEHOLDER_GENERIC_6DOF = RobotModel(
    name="GENERIC_6DOF_PLACEHOLDER",
    joints=[
        # a[m]   alpha[rad]      d[m]    offset      qlim[rad]
        JointDH(0.155, -np.pi / 2, 0.450, 0.0, (np.deg2rad(-170), np.deg2rad(170))),  # S
        JointDH(0.614, 0.0, 0.0, -np.pi / 2, (np.deg2rad(-90), np.deg2rad(155))),      # L
        JointDH(0.200, -np.pi / 2, 0.0, 0.0, (np.deg2rad(-175), np.deg2rad(250))),     # U
        JointDH(0.0, np.pi / 2, 0.640, 0.0, (np.deg2rad(-180), np.deg2rad(180))),      # R
        JointDH(0.0, -np.pi / 2, 0.0, 0.0, (np.deg2rad(-135), np.deg2rad(135))),       # B
        JointDH(0.0, 0.0, 0.100, 0.0, (np.deg2rad(-360), np.deg2rad(360))),            # T
    ],
    # PLACEHOLDER pulse constants. Real values come from the DX200 param file.
    pulses_per_degree=[1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0],
)

AXIS_NAMES = ["S", "L", "U", "R", "B", "T"]

# The model actually served by the API. Swap this out once you have real data.
ACTIVE_MODEL = PLACEHOLDER_GENERIC_6DOF
