"""
Pulse ↔ degree / mm conversion mined from the production DX200 CF dump
(`reference/robot-cell-dx200`, 2026-09-15).

Sources
-------
* Robot softlimits: ``RC.PRM`` / ``RC1G`` indices 924–937 (pulse ±limits)
  paired with MA2010 URDF joint limits (``frontend/public/models/ma2010``).
* Station TURN: ``RBCALIB.DAT`` SPULSE steps (E Δ≈91771 over ~90°) and
  ``RC3G``/``RC4G`` (S1/S2 TURN-1). Production jobs write ``EC#=p,0``.
* Base RECT-X (BS1): no rack-lead / gear register is present in the CF text we
  keep in-repo (``RC1G`` first row is **link lengths in 0.001 mm**, e.g.
  index 8 = ``1082000`` → forearm **1082 mm**, not a travel softlimit).
  ``PULSES_PER_MM`` is therefore calibrated from taught ``BC#`` ranges in
  production jobs vs the cell layout (stations 2400 mm apart along the rail).
  Encoder resolution ``SV1G = 4096`` is recorded but not enough alone without
  the missing lead/gear.
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

# ---------------------------------------------------------------------------
# RECT-X / BS1 — taught BC# anchors + calibrated pulses/mm
# ---------------------------------------------------------------------------
# Production home / transport over the rail (repeating C00000 jobs).
BC_HOME_PULSE: int = 570125
# jobs/000.JBI park on the ST1 side of the track.
BC_PARK_PULSE: int = 878036
# Alias kept for older call sites.
BASE_PARK_PULSE: int = BC_PARK_PULSE

# Taught working envelopes (min..max BC) from production jobs.
BC_ST2_RANGE: tuple[int, int] = (124171, 375361)
BC_ST1_ST3_RANGE: tuple[int, int] = (808680, 927851)

# Cell layout: RobotWorkspace stations at world Z = ±1.2 m → 2400 mm apart.
# ST2 uses the low-BC end; ST1/ST3 the high-BC (park) end.
_STATION_SPAN_MM: float = 2400.0
_ST2_MID: float = (BC_ST2_RANGE[0] + BC_ST2_RANGE[1]) / 2.0  # 249766
_ST13_MID: float = (BC_ST1_ST3_RANGE[0] + BC_ST1_ST3_RANGE[1]) / 2.0  # 868265.5

# Calibrated: ΔBC between station midpoints / 2400 mm.
# Cross-check: park − home = 307911 ≈ 1195 mm ≈ half span to ST1.
BASE_PULSES_PER_MM: float = (_ST13_MID - _ST2_MID) / _STATION_SPAN_MM  # ≈ 257.708
# Explicit name requested by M1.3.
PULSES_PER_MM: float = BASE_PULSES_PER_MM

# RC1G first data row (0.001 mm units): a1=150, a2=760, a3=200, d4=1082.
# Previously mis-labelled as a base softlimit — kept for documentation only.
RC1G_LINK_D4_UM: int = 1_082_000  # 1082.000 mm forearm (URDF d4)

STATION_IDLE_PULSE: int = -1746  # jobs/000.JBI EC# first component at weld

ENCODER_RESOLUTION: int = 4096  # SV1G


@dataclass(frozen=True)
class PulseConfig:
    """Cell pulse scaling used by the postprocessor."""

    robot_pulses_per_degree: tuple[float, ...] = tuple(MA2010_PULSES_PER_DEGREE)
    station_pulses_per_degree: float = STATION_PULSES_PER_DEGREE
    base_park_pulse: int = BC_PARK_PULSE
    base_home_pulse: int = BC_HOME_PULSE
    base_pulses_per_mm: float = BASE_PULSES_PER_MM
    station_idle_pulse: int = STATION_IDLE_PULSE
    encoder_resolution: int = ENCODER_RESOLUTION


CELL_PULSE = PulseConfig()

# Nominal BC for each station (midpoint of taught envelope).
STATION_BC_MID: dict[str, int] = {
    "ST1": int(round(_ST13_MID)),
    "ST2": int(round(_ST2_MID)),
    "ST3": int(round(_ST13_MID)),
    # ST4 not listed in the taught envelopes; opposite end of the rail (ST2 side).
    "ST4": int(round(_ST2_MID)),
}


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


def track_mm_to_pulses(
    mm: float,
    *,
    origin_pulse: int | None = None,
    pulses_per_mm: float | None = None,
) -> int:
    """RECT-X travel in millimetres → INFORM ``BC#``.

    ``mm = 0`` is the production home / rail centre (``BC_HOME_PULSE``).
    Positive ``mm`` increases BC (toward ST1 / park, world −Z).
    """
    origin = BC_HOME_PULSE if origin_pulse is None else origin_pulse
    ppm = BASE_PULSES_PER_MM if pulses_per_mm is None else pulses_per_mm
    return int(round(origin + mm * ppm))


def track_pulses_to_mm(
    pulses: int | float,
    *,
    origin_pulse: int | None = None,
    pulses_per_mm: float | None = None,
) -> float:
    """INFORM ``BC#`` → millimetres from rail home."""
    origin = BC_HOME_PULSE if origin_pulse is None else origin_pulse
    ppm = BASE_PULSES_PER_MM if pulses_per_mm is None else pulses_per_mm
    return (float(pulses) - origin) / ppm


def track_mm_from_world_z(z: float) -> float:
    """World Z (metres if |z|<10, else mm) → track mm from home.

    Cell layout: rail along Z, ST1 at Z≈−1.2 m (high BC), ST2 at Z≈+1.2 m.
    """
    z_mm = z * 1000.0 if abs(z) < 10.0 else float(z)
    return -z_mm


def base_mm_to_bc(
    mm: float | None = None,
    *,
    park_pulse: int | None = None,
    pulses_per_mm: float | None = None,
    origin_pulse: int | None = None,
) -> int:
    """RECT-X ``BC#``.

    * ``mm is None`` → park pulse (legacy default matching ``000.JBI``).
    * otherwise → ``track_mm_to_pulses`` from home (or ``origin_pulse``).
    """
    if mm is None:
        return park_pulse if park_pulse is not None else CELL_PULSE.base_park_pulse
    return track_mm_to_pulses(
        mm, origin_pulse=origin_pulse, pulses_per_mm=pulses_per_mm
    )


def resolve_base_pulses(
    npos: int,
    *,
    base_pulse: list[int] | int | None = None,
    base_mm: list[float] | float | None = None,
    points: list[dict] | None = None,
    station: str | None = None,
    default_park: bool = False,
) -> list[int]:
    """Build a per-point BC# list for a job of length ``npos``.

    Priority: explicit pulses → explicit mm → station midpoint → world Z of
    each point → park (if ``default_park``) → home.
    """
    if isinstance(base_pulse, int):
        return [base_pulse] * npos
    if isinstance(base_pulse, list) and base_pulse:
        if len(base_pulse) == 1:
            return base_pulse * npos
        if len(base_pulse) != npos:
            raise ValueError(f"base_pulse length {len(base_pulse)} != npos {npos}")
        return [int(p) for p in base_pulse]

    if isinstance(base_mm, (int, float)):
        return [track_mm_to_pulses(float(base_mm))] * npos
    if isinstance(base_mm, list) and base_mm:
        if len(base_mm) == 1:
            return [track_mm_to_pulses(float(base_mm[0]))] * npos
        if len(base_mm) != npos:
            raise ValueError(f"base_mm length {len(base_mm)} != npos {npos}")
        return [track_mm_to_pulses(float(m)) for m in base_mm]

    if station:
        key = station.upper().replace(" ", "")
        if key not in STATION_BC_MID:
            raise ValueError(f"Unknown station '{station}'. Use ST1..ST4.")
        return [STATION_BC_MID[key]] * npos

    if points is not None and len(points) == npos:
        return [
            track_mm_to_pulses(track_mm_from_world_z(float(p.get("z", 0.0))))
            for p in points
        ]

    fill = BC_PARK_PULSE if default_park else BC_HOME_PULSE
    return [fill] * npos
