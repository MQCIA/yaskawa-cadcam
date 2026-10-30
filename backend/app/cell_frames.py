"""
Cell frame registry (M2.1) + TouchSense / CUBEINTF anchors (M2.4–2.5).

Sources (DX200 CF dump ``reference/robot-cell-dx200``, 2026-09-15):
  * ``TOOL.CND`` — TOOL 0 live torch TCP (also fixture ``tests/fixtures/robot-cell``)
  * ``jobs/*.JBI`` — ``///TOOL 0`` on production PULSE jobs; ``///USER 1`` on
    offline RECTAN review (``DROP01.JBI``)
  * ``jobs/SRCH-1D1.JBI``, ``SRCH-2D1.JBI``, ``SZUKAJ1.JBI`` — TouchSense search
  * ``cnd/CUBEINTF.CND`` — named cubes ``STACJA_1`` / ``STACJA_2`` (pulse boxes)

Validate every exported path in MotoSim before loading the controller.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class ToolFrame:
    """One TOOL.CND entry (flange → TCP)."""

    tool_no: int
    name: str
    xyz_mm: tuple[float, float, float]
    rpy_deg: tuple[float, float, float]
    # Extra TOOL.CND rows kept for documentation (mass / inertia placeholders).
    secondary_xyz_mm: tuple[float, float, float] | None = None
    mass_kg: float | None = None
    active: bool = True


@dataclass(frozen=True)
class UserFrame:
    """USER / UF# used in teach or offline review jobs."""

    user_no: int
    name: str
    used_by: tuple[str, ...]
    # Production PULSE welding jobs do not select UF# — robot base + TOOL 0.
    pulse_jobs_use: bool = False
    notes: str = ""


@dataclass(frozen=True)
class CubeZone:
    """CUBEINTF soft-interference / approach cube (pulse corner pair)."""

    cube_no: int
    name: str
    # Raw CUBEINTF corner rows (first three components are the geometric axes
    # monitored by the controller — left as pulses; see CELL_FRAMES.md).
    corner_a: tuple[int, ...]
    corner_b: tuple[int, ...]
    # Linked GROUP2 station when the cube name matches STACJA_n.
    station: str | None = None
    flags: tuple[int, ...] = (2, 0, 1)


@dataclass(frozen=True)
class TouchSensePattern:
    """Production search grammar mined from SRCH-* / SZUKAJ1."""

    name: str
    master_job: str
    search_job: str
    rin: int
    speed_v: float
    timeout_s: float
    distance_mm: float
    # Optional second search in SRCH-2D1.
    distance2_mm: float | None = None
    postype: tuple[str, ...] = ("ROBOT", "BASE")
    notes: str = ""


# ---------------------------------------------------------------------------
# TOOL.CND — only TOOL 0 is taught / live on this cell
# ---------------------------------------------------------------------------
TOOL0 = ToolFrame(
    tool_no=0,
    name="STANDARD TOOL11",
    xyz_mm=(-54.280, 1.052, 415.702),
    rpy_deg=(1.3119, -42.6215, -6.8387),
    secondary_xyz_mm=(-34.329, -59.651, 325.206),
    mass_kg=1.836,
    active=True,
)

TOOL_FRAMES: dict[int, ToolFrame] = {
    0: TOOL0,
    # TOOL 1…63 exist in TOOL.CND as zero placeholders — not used by live jobs.
}

DEFAULT_TOOL_NO: int = 0


# ---------------------------------------------------------------------------
# USER / UF#
# ---------------------------------------------------------------------------
USER_FRAMES: dict[int, UserFrame] = {
    1: UserFrame(
        user_no=1,
        name="UF1 (offline / DROP01 review)",
        used_by=("DROP01.JBI",),
        pulse_jobs_use=False,
        notes=(
            "Cartesian USER/RECTAN review jobs select ///USER 1 and "
            "////FRAME USER 1. Production welding jobs are ///POSTYPE PULSE "
            "with ///TOOL 0 only — no UF# in the POS header."
        ),
    ),
}

# TCP calibration helpers in the dump (headers only in-repo) — document as
# teach aids, not runtime UF numbers.
UF_TEACH_JOBS: tuple[str, ...] = (
    "TCP-R1-Y-BA.JBI",
    "TCP-R1-Y-BAREL.JBI",
    "EICH-R1-Y-BA.JBI",
)

DEFAULT_USER_FRAME: int = 1


# ---------------------------------------------------------------------------
# CUBEINTF.CND — STACJA_1 / STACJA_2 (pulse boxes from dump scan)
# ---------------------------------------------------------------------------
CUBE_ZONES: dict[str, CubeZone] = {
    "STACJA_1": CubeZone(
        cube_no=1,
        name="STACJA_1",
        corner_a=(632891, -23550, 139716, 0, 0, 0, 0, 0),
        corner_b=(622891, -33550, 129716, 0, 0, 0, 0, 0),
        station="ST1",
        flags=(2, 0, 1),
    ),
    "STACJA_2": CubeZone(
        cube_no=2,
        name="STACJA_2",
        corner_a=(633450, 15097, 139701, 0, 0, 0, 0, 0),
        corner_b=(623450, 5097, 129701, 0, 0, 0, 0, 0),
        station="ST2",
        flags=(2, 0, 1),
    ),
}


def cube_for_station(station: str) -> CubeZone | None:
    key = station.upper().replace(" ", "")
    for cube in CUBE_ZONES.values():
        if cube.station == key:
            return cube
    # ST3 shares the high-BC end with ST1; ST4 with ST2.
    if key == "ST3":
        return CUBE_ZONES.get("STACJA_1")
    if key == "ST4":
        return CUBE_ZONES.get("STACJA_2")
    return None


# ---------------------------------------------------------------------------
# TouchSense — SRCH-1D1 / SRCH-2D1 / SZUKAJ1
# ---------------------------------------------------------------------------
TOUCH_SENSE_1D = TouchSensePattern(
    name="SRCH-1D1",
    master_job="SZUKAJ1.JBI",
    search_job="SRCH-1D1.JBI",
    rin=1,
    speed_v=15.0,
    timeout_s=0.20,
    distance_mm=10.0,
    postype=("ROBOT", "BASE"),
    notes="Single search: MOVL P001 V=15.0 SRCH RIN#(1)=ON T=0.20 DIS=10.0",
)

TOUCH_SENSE_2D = TouchSensePattern(
    name="SRCH-2D1",
    master_job="SZUKAJ1.JBI",
    search_job="SRCH-2D1.JBI",
    rin=1,
    speed_v=15.0,
    timeout_s=0.20,
    distance_mm=20.0,
    distance2_mm=10.0,
    postype=("ROBOT", "PULSE", "BASE"),
    notes=(
        "Two-direction search called from SZUKAJ1: "
        "MOVL P002 SRCH DIS=20 → retract → MOVL P003 SRCH DIS=10. "
        "Master SZUKAJ1 is ///POSTYPE PULSE + CALL JOB:SRCH-2D1."
    ),
)

DEFAULT_TOUCH_SENSE: TouchSensePattern = TOUCH_SENSE_2D


@dataclass
class CellFrameConfig:
    """Active TOOL / UF / TouchSense selection for the postprocessor."""

    tool_no: int = DEFAULT_TOOL_NO
    user_frame: int = DEFAULT_USER_FRAME
    touch_sense: TouchSensePattern = field(default_factory=lambda: DEFAULT_TOUCH_SENSE)
    # When True, PULSE jobs emit CALL JOB:SRCH-2D1 before the first ARCON
    # (SZUKAJ1 master pattern). Sense waypoints still mark the CAD search poses.
    emit_search_call: bool = True


CELL_FRAMES = CellFrameConfig()


def tool_tcp_mm(tool_no: int = DEFAULT_TOOL_NO) -> tuple[float, float, float]:
    frame = TOOL_FRAMES.get(tool_no, TOOL0)
    return frame.xyz_mm


def tool_rpy_deg(tool_no: int = DEFAULT_TOOL_NO) -> tuple[float, float, float]:
    frame = TOOL_FRAMES.get(tool_no, TOOL0)
    return frame.rpy_deg


def touch_sense_jbi_comments(
    pattern: TouchSensePattern | None = None,
    *,
    emit_call: bool = True,
) -> list[str]:
    """INFORM comment / CALL lines matching SZUKAJ1 → SRCH-2D1."""
    pat = pattern or DEFAULT_TOUCH_SENSE
    lines = [
        f"' TouchSense CAD↔part — production {pat.master_job} → CALL JOB:{pat.search_job}",
        (
            f"' SRCH RIN#({pat.rin})=ON T={pat.timeout_s:.2f} "
            f"DIS={pat.distance_mm:.1f}"
            + (
                f" / DIS2={pat.distance2_mm:.1f}"
                if pat.distance2_mm is not None
                else ""
            )
            + f" V={pat.speed_v:.1f}"
        ),
    ]
    if emit_call:
        lines.append(f"CALL JOB:{pat.search_job.replace('.JBI', '')}")
    return lines
