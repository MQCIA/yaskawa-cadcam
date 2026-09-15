"""
Pulse ↔ degree / mm conversion mined from the production DX200 CF dump
(`reference/robot-cell-dx200`, 2026-09-15).

Sources
-------
* Robot softlimits: ``RC.PRM`` / ``RC1G`` indices 924–937 (pulse ±limits)
  paired with MA2010 URDF joint limits (``frontend/public/models/ma2010``).
* Station TURN: ``RBCALIB.DAT`` SPULSE steps (E Δ≈91771 over ~90°) and
  ``RC3G``/``RC4G`` (S1/S2 TURN-1). Production jobs write ``EC#=p,0``.
* Base RECT-X: park pulse from ``jobs/000.JBI`` / ``RBCALIB`` MBSC; stroke
  softlimit hint ``RC1G[8]=1082000``. mm→pulse not certified (no lead in dump).
* Encoder resolution: ``SV.PRM`` / ``SV1G[24..29]=4096``.

Validate in MotoSim before any controller load. Absolute-encoder ABSO offsets
are NOT applied here — Motoman INFORM PULSE is relative to the mechanical
origin used by the pendant.
"""

from __future__ import annotations

from dataclasses import dataclass


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

# Production park / teach defaults
BASE_PARK_PULSE: int = 878036  # jobs/000.JBI BC#
BASE_SOFTLIMIT_PULSE: int = 1_082_000  # RC1G[8] travel hint
STATION_IDLE_PULSE: int = -1746  # jobs/000.JBI EC# first component at weld

ENCODER_RESOLUTION: int = 4096  # SV1G


@dataclass(frozen=True)
class PulseConfig:
    """Cell pulse scaling used by the postprocessor."""

    robot_pulses_per_degree: tuple[float, ...] = tuple(MA2010_PULSES_PER_DEGREE)
    station_pulses_per_degree: float = STATION_PULSES_PER_DEGREE
    base_park_pulse: int = BASE_PARK_PULSE
    station_idle_pulse: int = STATION_IDLE_PULSE
    encoder_resolution: int = ENCODER_RESOLUTION


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
    mm: float | None = None,
    *,
    park_pulse: int | None = None,
    pulses_per_mm: float | None = None,
) -> int:
    """RECT-X BC#. Without certified pulses/mm, return park pulse (matches 000.JBI)."""
    park = park_pulse if park_pulse is not None else CELL_PULSE.base_park_pulse
    if mm is None or pulses_per_mm is None:
        return park
    return int(round(park + mm * pulses_per_mm))
