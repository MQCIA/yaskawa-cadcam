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
# TCP / UF teach helpers in the dump (mixed TOOL/BASE/USER postype blocks).
# Headers in docs/_dump_scan.json; full bodies stay under reference/ (gitignore).
UF_TEACH_JOBS: tuple[str, ...] = (
    "TCP-R1-Y-BA.JBI",
    "TCP-R1-Y-BAREL.JBI",
    "EICH-R1-Y-BA.JBI",
)

USER_FRAMES: dict[int, UserFrame] = {
    1: UserFrame(
        user_no=1,
        name="UF1 (DROP01 review + TCP teach)",
        used_by=("DROP01.JBI",) + UF_TEACH_JOBS,
        pulse_jobs_use=False,
        notes=(
            "DROP01: ///USER 1 + ////FRAME USER 1 (RECTAN review). "
            "TCP-R1-Y-BA* / EICH-R1-Y-BA: teach helpers with mixed "
            "TOOL/BASE/USER postype blocks. Live ARCON jobs stay ///TOOL 0 PULSE "
            "without UF#."
        ),
    ),
}

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


# Motoman CUBEINTF geometric axes: cartesian pulse ≈ 1000 pulse/mm
# (same numeric scale as 0.001 mm units in RC1G link lengths).
CUBE_PULSES_PER_MM: float = 1000.0


def cube_corner_mm(corner: tuple[int, ...]) -> tuple[float, float, float]:
    """First three CUBEINTF components → millimetres."""
    return (
        float(corner[0]) / CUBE_PULSES_PER_MM,
        float(corner[1]) / CUBE_PULSES_PER_MM,
        float(corner[2]) / CUBE_PULSES_PER_MM,
    )


def cube_aabb_m(
    cube: CubeZone,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Axis-aligned bbox of a CUBEINTF zone in metres (robot / cell frame).

    Returns ``(min_xyz_m, max_xyz_m)`` from the two taught corners.
    """
    a = cube_corner_mm(cube.corner_a)
    b = cube_corner_mm(cube.corner_b)
    mn = (min(a[0], b[0]) / 1000.0, min(a[1], b[1]) / 1000.0, min(a[2], b[2]) / 1000.0)
    mx = (max(a[0], b[0]) / 1000.0, max(a[1], b[1]) / 1000.0, max(a[2], b[2]) / 1000.0)
    return mn, mx


def all_cube_aabbs_m() -> list[
    tuple[str, tuple[float, float, float], tuple[float, float, float]]
]:
    """Named interference cubes for coarse collision checks."""
    out: list[tuple[str, tuple[float, float, float], tuple[float, float, float]]] = []
    for name, cube in CUBE_ZONES.items():
        mn, mx = cube_aabb_m(cube)
        out.append((name, mn, mx))
    return out


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


def touch_sense_srch_block_comments(
    pattern: TouchSensePattern | None = None,
) -> list[str]:
    """Document the production SRCH-* grammar as INFORM comments (not executable).

    Live search stays on the controller job (``CALL JOB:SRCH-2D1``); CAD only
    supplies approach poses. Inline ``MOVL … SRCH`` would need taught P-vars.
    """
    pat = pattern or DEFAULT_TOUCH_SENSE
    job = pat.search_job.replace(".JBI", "")
    lines = [
        f"' --- TouchSense block (production {pat.master_job} → {job}) ---",
        f"' MOVL P001 V={pat.speed_v:.1f}   (pre-search)",
        (
            f"' MOVL P002 V={pat.speed_v:.1f} SRCH RIN#({pat.rin})=ON "
            f"T={pat.timeout_s:.2f} DIS={pat.distance_mm:.1f}"
        ),
    ]
    if pat.distance2_mm is not None:
        lines.extend(
            [
                f"' MOVL P001 V=25.0          (retract)",
                (
                    f"' MOVL P003 V={pat.speed_v:.1f} SRCH RIN#({pat.rin})=ON "
                    f"T={pat.timeout_s:.2f} DIS={pat.distance2_mm:.1f}"
                ),
            ]
        )
    lines.append(f"' Runtime: CALL JOB:{job}  (job must exist on CF / MotoSim)")
    return lines


def touch_sense_jbi_comments(
    pattern: TouchSensePattern | None = None,
    *,
    emit_call: bool = True,
    emit_srch_block: bool = True,
) -> list[str]:
    """INFORM comment / CALL lines matching SZUKAJ1 → SRCH-2D1.

    When ``emit_srch_block`` is True, also document the full SRCH grammar from
    the dump (DIS / RIN / T / V) so the exported job is self-describing.
    """
    pat = pattern or DEFAULT_TOUCH_SENSE
    lines: list[str] = []
    if emit_srch_block:
        lines.extend(touch_sense_srch_block_comments(pat))
    else:
        lines.append(
            f"' TouchSense CAD↔part — production {pat.master_job} → "
            f"CALL JOB:{pat.search_job.replace('.JBI', '')}"
        )
        lines.append(
            f"' SRCH RIN#({pat.rin})=ON T={pat.timeout_s:.2f} "
            f"DIS={pat.distance_mm:.1f}"
            + (
                f" / DIS2={pat.distance2_mm:.1f}"
                if pat.distance2_mm is not None
                else ""
            )
            + f" V={pat.speed_v:.1f}"
        )
    if emit_call:
        lines.append(f"CALL JOB:{pat.search_job.replace('.JBI', '')}")
    return lines
