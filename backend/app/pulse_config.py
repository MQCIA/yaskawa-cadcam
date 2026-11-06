"""
Pulse ↔ degree / mm conversion mined from the production DX200 CF dump
(`reference/robot-cell-dx200` / `reference/dx200-backup-20260915`, 2026-09-15).

Sources
-------
* Robot softlimits: ``RC.PRM`` / ``RC1G`` indices 924–937 (pulse ±limits)
  paired with MA2010 URDF joint limits (``frontend/public/models/ma2010``).
* Station TURN: ``RBCALIB.DAT`` SPULSE steps (E Δ≈91771 over ~90°) and
  ``RC3G``/``RC4G`` (S1/S2 TURN-1). Production jobs write ``EC#=p,0``.
* Base RECT-X (BS1):
  - Encoder PPR ``SV.PRM`` / ``SV1G`` = **4096** (axes 0…5).
  - Travel softlimit hint ``RC1G[8] = 1082000`` pulses.
  - Reference BC# from production INFORM jobs (home / park / station spans).
  - **Ballscrew / rack lead is NOT present as an explicit mm register in
    RC1G/RC2G in this dump** → ``RECT_X_LEAD_VERIFIED = False`` and the
    pulses/mm coefficient is ``UNVERIFIED_PLACEHOLDER`` until a teach
    measurement or Motoman RECT-X datasheet lead×gear is entered.
* Absolute-encoder ``ABSO.DAT`` offsets are NOT applied here — Motoman INFORM
  PULSE is relative to the mechanical origin used by the pendant.

Validate in MotoSim before any controller load.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

_log = logging.getLogger(__name__)

# RC1G softlimits (pulses) — positive / negative per S,L,U,R,B,T
RC1G_SOFTLIMIT_POS = (241449, 295690, 254863, 153430, 88222, 95499)
RC1G_SOFTLIMIT_NEG = (-241449, -200306, -136989, -153430, -132333, -95499)

# URDF / MA2010 joint limits used to derive pulses_per_degree (deg)
MA2010_LIMITS_DEG: list[tuple[float, float]] = [
    (-180.0, 180.0),   # S
    (-105.0, 155.0),   # L
    (-86.0, 160.0),    # U
    (-150.0, 150.0),   # R
    (-135.0, 90.0),    # B
    (-210.0, 210.0),   # T
]


def _ppd_from_softlimits() -> list[float]:
    """pulses/deg = |softlimit_pulse| / |limit_deg| (avg of + and − sides)."""
    out: list[float] = []
    for i, (lo, hi) in enumerate(MA2010_LIMITS_DEG):
        pos = RC1G_SOFTLIMIT_POS[i] / hi if hi else 0.0
        neg = abs(RC1G_SOFTLIMIT_NEG[i] / lo) if lo else pos
        out.append((pos + neg) / 2.0)
    return out


# Derived: ≈ [1341.38, 1907.68, 1592.89, 1022.87, 980.24, 454.76]
MA2010_PULSES_PER_DEGREE: list[float] = _ppd_from_softlimits()

# RBCALIB SSTC1→SSTC2: E2 0 → −91771 (≈ −90° on TURN axis)
STATION_PULSES_PER_DEGREE: float = 91771.0 / 90.0  # ≈ 1019.6889

# ---------------------------------------------------------------------------
# RECT-X (BS1) — reference pulses from production .JBI (not invented)
# ---------------------------------------------------------------------------
# Safe home / bookend used across 1-DEFRO-5, 02025_KOSZ…, HOME_TRACK, …
BC_HOME_PULSE: int = 570125
# Park pose from jobs/000.JBI
BC_PARK_000_PULSE: int = 878036
# Observed BC spans in production jobs (inclusive hints for validation UI)
ST2_BC_RANGE: tuple[int, int] = (124171, 375361)
ST3_BC_RANGE: tuple[int, int] = (808680, 847209)
ST1_BC_MAX: int = 927851

BASE_SOFTLIMIT_PULSE: int = 1_082_000  # RC1G[8] travel hint
STATION_IDLE_PULSE: int = -1746  # jobs/000.JBI EC# first component at weld

ENCODER_RESOLUTION: int = 4096  # SV1G PPR

# Back-compat alias (park from 000.JBI)
BASE_PARK_PULSE: int = BC_PARK_000_PULSE

# TODO(M1.3): replace when RC2 / RECT-X datasheet lead×gear or rail teach
# (two known mm + BC pairs) is available. Until then this is a round
# UNVERIFIED_PLACEHOLDER — never claim DX200 certification.
RECT_X_PULSES_PER_MM: float = 1000.0  # UNVERIFIED_PLACEHOLDER
RECT_X_LEAD_VERIFIED: bool = False

_RECT_X_WARNED = False


def _warn_rect_x_unverified() -> None:
    global _RECT_X_WARNED
    if RECT_X_LEAD_VERIFIED or _RECT_X_WARNED:
        return
    _RECT_X_WARNED = True
    _log.warning(
        "RECT-X pulses/mm is UNVERIFIED_PLACEHOLDER (%.4f); "
        "RC1G/RC2G dump has no ballscrew lead mm — BC↔mm is APP_ESTIMATE only",
        RECT_X_PULSES_PER_MM,
    )


@dataclass(frozen=True)
class PulseConfig:
    """Cell pulse scaling used by the postprocessor."""

    robot_pulses_per_degree: tuple[float, ...] = tuple(MA2010_PULSES_PER_DEGREE)
    station_pulses_per_degree: float = STATION_PULSES_PER_DEGREE
    base_park_pulse: int = BC_PARK_000_PULSE
    base_home_pulse: int = BC_HOME_PULSE
    station_idle_pulse: int = STATION_IDLE_PULSE
    encoder_resolution: int = ENCODER_RESOLUTION
    rect_x_pulses_per_mm: float = RECT_X_PULSES_PER_MM
    rect_x_lead_verified: bool = RECT_X_LEAD_VERIFIED


CELL_PULSE = PulseConfig()


def deg_to_joint_pulses(
    q_deg: list[float],
    pulses_per_degree: list[float] | tuple[float, ...] | None = None,
) -> list[int]:
    """Convert S,L,U,R,B,T degrees → INFORM C# pulse integers."""
    ppd = list(pulses_per_degree or CELL_PULSE.robot_pulses_per_degree)
    if len(q_deg) != len(ppd):
        raise ValueError(f"Expected {len(ppd)} joint angles, got {len(q_deg)}")
    return [int(round(q * p)) for q, p in zip(q_deg, ppd)]


def joint_pulses_to_deg(
    pulses: list[int],
    pulses_per_degree: list[float] | tuple[float, ...] | None = None,
) -> list[float]:
    ppd = list(pulses_per_degree or CELL_PULSE.robot_pulses_per_degree)
    if len(pulses) != len(ppd):
        raise ValueError(f"Expected {len(ppd)} pulse values, got {len(pulses)}")
    return [p / s for p, s in zip(pulses, ppd)]


def station_deg_to_ec(
    deg: float,
    *,
    pulses_per_degree: float | None = None,
    second_axis: int = 0,
) -> tuple[int, int]:
    """TURN ST1: production jobs use ``EC#=pulse,0`` (TURN-1)."""
    ppd = pulses_per_degree if pulses_per_degree is not None else CELL_PULSE.station_pulses_per_degree
    return int(round(deg * ppd)), int(second_axis)


def base_mm_to_bc(
    x_mm: float,
    *,
    origin_bc: int | None = None,
    pulses_per_mm: float | None = None,
) -> int:
    """RECT-X: rail millimetres → INFORM ``BC#`` pulses.

    Convention: ``x_mm = 0`` maps to ``origin_bc`` (default ``BC_HOME_PULSE`` =
    production bookend 570125). Positive ``x_mm`` increases BC.

    When ``RECT_X_LEAD_VERIFIED`` is False the scale is
    ``UNVERIFIED_PLACEHOLDER`` — do not treat the result as pendant truth.
    """
    _warn_rect_x_unverified()
    origin = BC_HOME_PULSE if origin_bc is None else int(origin_bc)
    ppm = RECT_X_PULSES_PER_MM if pulses_per_mm is None else float(pulses_per_mm)
    if ppm == 0.0:
        raise ValueError("RECT-X pulses/mm must be non-zero")
    return int(round(origin + float(x_mm) * ppm))


def bc_to_base_mm(
    bc_pulse: int,
    *,
    origin_bc: int | None = None,
    pulses_per_mm: float | None = None,
) -> float:
    """RECT-X: INFORM ``BC#`` pulses → millimetres relative to ``origin_bc``."""
    _warn_rect_x_unverified()
    origin = BC_HOME_PULSE if origin_bc is None else int(origin_bc)
    ppm = RECT_X_PULSES_PER_MM if pulses_per_mm is None else float(pulses_per_mm)
    if ppm == 0.0:
        raise ValueError("RECT-X pulses/mm must be non-zero")
    return (int(bc_pulse) - origin) / ppm


def rect_x_jbi_warning_comment() -> str:
    """Single-line INFORM comment for unverified RECT-X scaling."""
    if RECT_X_LEAD_VERIFIED:
        return "' RECT-X pulses/mm verified from cell dump / teach"
    return (
        f"' WARN: RECT-X pulses/mm={RECT_X_PULSES_PER_MM} UNVERIFIED_PLACEHOLDER "
        f"(RC1G/RC2G lead TBD) — BC# mm mapping is APP_ESTIMATE only"
    )
