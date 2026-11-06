"""
Yaskawa DX200 postprocessor: path -> INFORM III ``.JBI`` job.

Two profiles (``PostprocessorConfig.profile``):

* ``ma2010_cell`` (default) — matches the production dump in
  ``reference/robot-cell-dx200``:
  TOOL 0, GROUP1 RB1,BS1 + GROUP2 ST1, ARCON/ARCOF, WVON, MACRO1, coordinated
  ``+MOVJ EC#``. Production ``///POSTYPE PULSE`` requires a full
  ``joint_angles_deg`` list from IK — never invents C# pulses. Without IK,
  only a diagnostic RECTAN preview named ``PREVIEW_RECTAN_DO_NOT_LOAD`` is
  allowed (``allow_preview_rectan=True``).

* ``cartesian_review`` — earlier DROP01-style USER/RECTAN + single-group ST1
  layout (handy for MotoSim / docs).

============================================================================
  !!!  UWAGA / WARNING  !!!
============================================================================
Validate every job in MotoSim before loading the controller. Pulse scaling,
RCONF, MACRO numbers and weave files (WEV#) are cell-specific.
============================================================================
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Literal

from .pulse_config import (
    BC_HOME_PULSE,
    CELL_PULSE,
    RECT_X_LEAD_VERIFIED,
    STATION_PULSES_PER_DEGREE,
    base_mm_to_bc,
    deg_to_joint_pulses,
    rect_x_jbi_warning_comment,
    station_deg_to_ec,
)
from .robot_config import CELL_HOME_PULSES, CELL_STATIONS, cell_home_deg
from .welding_config import LORCH_S8_SCHEDULES, WeldSchedule

StationId = Literal["ST1", "ST2", "ST3", "ST4"]


class PulseExportError(ValueError):
    """Raised when production PULSE export is requested without valid IK joints."""


@dataclass
class WeldSegment:
    """A contiguous run of path points that should be welded (arc on)."""

    start_index: int
    end_index: int
    weld_speed: float = 5.0  # V= travel (matches cell jobs ~5.0–7.5)
    arc_file: int = 1
    weave_no: int = 21  # WEV# used on the production cell


@dataclass
class PostprocessorConfig:
    job_name: str = "WELD_AUTO"
    tool_no: int = 0
    user_frame: int = 1
    # "ma2010_cell" | "cartesian_review"
    profile: str = "ma2010_cell"
    # M2.2 — INFORM GROUP2 target station.
    station: StationId = "ST1"
    move_speed: float = 80.0  # air VJ= on the production cell
    weld_speed: float = 5.0
    joint_speed_pct: float = 80.0
    rconf: str = "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
    use_station_axis: bool = True
    station_pulses_per_deg: float = STATION_PULSES_PER_DEGREE
    joint_approach: bool = True
    # Production TOOL 0 tip Z ≈ 415.7 mm (informational / comments).
    tool_tcp_mm: tuple[float, float, float] = (-54.280, 1.052, 415.702)
    default_weave: int = 21
    macro_arg: int = 8  # MACRO1 MJ#(0) ARGF8 seen before ARCON
    # Production PULSE requires IK joints. Preview RECTAN is opt-in only.
    prefer_pulse: bool = True
    allow_preview_rectan: bool = False
    # When True and path lacks role=home ends, prepend/append cell home.
    home_bookends: bool = True
    # Rail X in mm relative to BC_HOME (0 → home bookend). Per-point overrides
    # via ``base_mm`` argument to ``generate_jbi``. None → use station zone BC.
    base_mm: float | None = None
    base_park_pulse: int = CELL_PULSE.base_park_pulse
    robot_pulses_per_degree: list[float] = field(
        default_factory=lambda: list(CELL_PULSE.robot_pulses_per_degree)
    )


def _fmt(v: float, decimals: int = 4) -> str:
    return f"{v:.{decimals}f}"


def _fmt3(v: float) -> str:
    return f"{v:.3f}"


def _fmt1(v: float) -> str:
    return f"{v:.1f}"


def _weld_comment(cond_no: int) -> str:
    sched: WeldSchedule | None = LORCH_S8_SCHEDULES.get(cond_no)
    if not sched:
        return f"' weld condition {cond_no}"
    return (
        f"' Motoweld cond {cond_no} / ARGF{sched.lorch_job}: "
        f"{sched.current_a:.0f}A / {sched.voltage_v:.1f}V / "
        f"wire {sched.wire_speed:.1f} m/min / {sched.process}"
    )


def _to_mm(v: float) -> float:
    return v * 1000.0 if abs(v) < 10 else v


def _station_pulse(deg: float, pulses_per_deg: float) -> int:
    return int(round(deg * pulses_per_deg))


def _joints_ready(
    joint_angles_deg: list[list[float]] | None,
    npos: int,
) -> bool:
    if joint_angles_deg is None or len(joint_angles_deg) != npos:
        return False
    for q in joint_angles_deg:
        if not isinstance(q, (list, tuple)) or len(q) != 6:
            return False
        if any(v is None for v in q):
            return False
    return True


def _normalize_station(station: str) -> StationId:
    key = str(station).upper().strip()
    if key not in CELL_STATIONS:
        raise ValueError(f"Unknown station '{station}'. Expected ST1..ST4.")
    return key  # type: ignore[return-value]


def _ensure_home_bookends(
    cartesian_points: list[dict],
    weld_segments: list[WeldSegment],
    joint_angles_deg: list[list[float]] | None,
    station_deg: list[float] | None,
    station_deg2: list[float] | None,
    base_mm: list[float] | float | None,
    cfg: PostprocessorConfig,
) -> tuple[
    list[dict],
    list[WeldSegment],
    list[list[float]] | None,
    list[float] | None,
    list[float] | None,
    list[float] | float | None,
]:
    """Prepend/append cell-home poses when missing (M2.3)."""
    if not cfg.home_bookends or not cartesian_points:
        return (
            cartesian_points,
            weld_segments,
            joint_angles_deg,
            station_deg,
            station_deg2,
            base_mm,
        )

    def is_home(p: dict) -> bool:
        return str(p.get("role", "")).lower() == "home"

    need_pre = not is_home(cartesian_points[0])
    need_post = not is_home(cartesian_points[-1])
    if not need_pre and not need_post:
        return (
            cartesian_points,
            weld_segments,
            joint_angles_deg,
            station_deg,
            station_deg2,
            base_mm,
        )

    from .kinematics_engine import forward_kinematics

    home_q = cell_home_deg(CELL_HOME_PULSES)
    fk = forward_kinematics(home_q, frame="tcp")
    home_pt = {
        "x": float(fk["x"]),
        "y": float(fk["y"]),
        "z": float(fk["z"]),
        "rx": float(cartesian_points[0].get("rx", 180.0)),
        "ry": float(cartesian_points[0].get("ry", 0.0)),
        "rz": float(cartesian_points[0].get("rz", 0.0)),
        "role": "home",
    }

    pts = list(cartesian_points)
    segs = list(weld_segments)
    joints = list(joint_angles_deg) if joint_angles_deg is not None else None
    sd = list(station_deg) if station_deg is not None else None
    sd2 = list(station_deg2) if station_deg2 is not None else None
    shift = 0
    if need_pre:
        pts = [home_pt, *pts]
        segs = [
            WeldSegment(
                start_index=s.start_index + 1,
                end_index=s.end_index + 1,
                weld_speed=s.weld_speed,
                arc_file=s.arc_file,
                weave_no=s.weave_no,
            )
            for s in segs
        ]
        if joints is not None:
            joints = [home_q, *joints]
        if sd is not None:
            sd = [sd[0] if sd else 0.0, *sd]
        if sd2 is not None:
            sd2 = [sd2[0] if sd2 else 0.0, *sd2]
        if isinstance(base_mm, list):
            base_mm = [0.0, *base_mm]
        shift = 1
    if need_post:
        pts = [*pts, dict(home_pt)]
        if joints is not None:
            joints = [*joints, home_q]
        if sd is not None:
            sd = [*sd, sd[-1] if sd else 0.0]
        if sd2 is not None:
            sd2 = [*sd2, sd2[-1] if sd2 else 0.0]
        if isinstance(base_mm, list):
            base_mm = [*base_mm, 0.0]
    _ = shift
    return pts, segs, joints, sd, sd2, base_mm


def _resolve_bc_pulses(
    points: list[dict],
    cfg: PostprocessorConfig,
    base_mm: list[float] | float | None,
) -> list[int]:
    """Per-point BC#: home → BC_HOME; else base_mm_to_bc or station default zone."""
    st = CELL_STATIONS[_normalize_station(cfg.station)]
    zone_bc = st.default_bc
    n = len(points)
    out: list[int] = []
    for i, p in enumerate(points):
        role = str(p.get("role", "")).lower()
        is_bookend = cfg.home_bookends and (i == 0 or i == n - 1)
        if role == "home" or is_bookend:
            out.append(BC_HOME_PULSE)
            continue
        if isinstance(base_mm, list) and i < len(base_mm) and base_mm[i] is not None:
            out.append(base_mm_to_bc(float(base_mm[i])))
        elif isinstance(base_mm, (int, float)):
            out.append(base_mm_to_bc(float(base_mm)))
        elif cfg.base_mm is not None:
            out.append(base_mm_to_bc(float(cfg.base_mm)))
        else:
            # Station zone BC (kosz-style stable rail pose for the selected ST).
            # Equivalent mm: bc_to_base_mm(zone_bc) under the unverified scale.
            out.append(zone_bc)
    return out


def generate_jbi(
    cartesian_points: list[dict],
    weld_segments: list[WeldSegment] | None = None,
    config: PostprocessorConfig | None = None,
    station_deg: list[float] | None = None,
    joint_angles_deg: list[list[float]] | None = None,
    base_mm: list[float] | float | None = None,
    station_deg2: list[float] | None = None,
) -> str:
    """Generate a DX200 ``.JBI`` job from Cartesian TCP points.

    Pass ``joint_angles_deg`` (one [S,L,U,R,B,T] per point from IK) to emit
    production ``///POSTYPE PULSE`` C#. Without IK, set
    ``config.allow_preview_rectan=True`` for a diagnostic RECTAN file only —
    never invents C# pulse integers.
    """
    cfg = config or PostprocessorConfig()
    cfg.station = _normalize_station(cfg.station)
    weld_segments = weld_segments or []
    if station_deg is not None:
        cfg.use_station_axis = True
    elif cfg.profile == "ma2010_cell":
        cfg.use_station_axis = True
        station_deg = [0.0] * len(cartesian_points)

    if cfg.profile == "ma2010_cell":
        (
            cartesian_points,
            weld_segments,
            joint_angles_deg,
            station_deg,
            station_deg2,
            base_mm,
        ) = _ensure_home_bookends(
            cartesian_points,
            weld_segments,
            joint_angles_deg,
            station_deg,
            station_deg2,
            base_mm,
            cfg,
        )
        return _generate_ma2010_cell(
            cartesian_points,
            weld_segments,
            cfg,
            station_deg,
            joint_angles_deg,
            base_mm,
            station_deg2,
        )
    return _generate_cartesian_review(cartesian_points, weld_segments, cfg, station_deg)


def _generate_ma2010_cell(
    cartesian_points: list[dict],
    weld_segments: list[WeldSegment],
    cfg: PostprocessorConfig,
    station_deg: list[float] | None,
    joint_angles_deg: list[list[float]] | None,
    base_mm: list[float] | float | None,
    station_deg2: list[float] | None,
) -> str:
    """INST layout matching production MA2010 cell jobs (GROUP2 STn)."""
    npos = len(cartesian_points)
    station_deg = station_deg or [0.0] * npos
    if len(station_deg) < npos:
        station_deg = list(station_deg) + [station_deg[-1] if station_deg else 0.0] * (
            npos - len(station_deg)
        )
    st_meta = CELL_STATIONS[cfg.station]
    if station_deg2 is None:
        station_deg2 = [0.0] * npos
    elif len(station_deg2) < npos:
        station_deg2 = list(station_deg2) + [
            station_deg2[-1] if station_deg2 else 0.0
        ] * (npos - len(station_deg2))

    joints_ok = _joints_ready(joint_angles_deg, npos)
    use_pulse = bool(cfg.prefer_pulse) and joints_ok

    if cfg.prefer_pulse and not joints_ok and not cfg.allow_preview_rectan:
        raise PulseExportError(
            "Production POSTYPE PULSE requires IK joint_angles_deg for every "
            "point (no fake C# pulses). Retry after /api/calculate-ik "
            "all_reachable=true, or set allow_preview_rectan for "
            "PREVIEW_RECTAN_DO_NOT_LOAD only."
        )

    bc_pulses = _resolve_bc_pulses(cartesian_points, cfg, base_mm)

    arc_on_at: dict[int, WeldSegment] = {s.start_index: s for s in weld_segments}
    arc_off_at: set[int] = {s.end_index for s in weld_segments}
    # Inter-seam air moves: retract → next approach (and home bookends).
    travel_indices = {
        i
        for i, p in enumerate(cartesian_points)
        if str(p.get("role", "")).lower() in ("approach", "retract", "home", "travel")
    }
    lines: list[str] = []

    job_name = cfg.job_name
    if not use_pulse:
        if "PREVIEW_RECTAN_DO_NOT_LOAD" not in job_name.upper():
            job_name = f"PREVIEW_RECTAN_DO_NOT_LOAD_{cfg.job_name}"

    lines.append("/JOB")
    lines.append(f"//NAME {job_name}")
    if use_pulse:
        lines.append("' PULSE export — MA2010 cell (RC1G / RBCALIB scaling)")
        lines.append("' C# from IK → deg_to_joint_pulses (no invented pulses)")
    else:
        lines.append("' PREVIEW_RECTAN_DO_NOT_LOAD — offline diagnostic only")
        lines.append("' NOT for DX200 load — IK missing or unreachable; no PULSE C#")
    lines.append(
        f"' Cell: MA2010 + RECT-X (BS1) + TURN ({cfg.station}); TOOL0 TCP mm "
        f"{cfg.tool_tcp_mm[0]},{cfg.tool_tcp_mm[1]},{cfg.tool_tcp_mm[2]}"
    )
    lines.append(
        f"' Station pulses/deg~{cfg.station_pulses_per_deg:.4f}; "
        f"BC_HOME={BC_HOME_PULSE}; zone_BC={st_meta.default_bc}; "
        f"RECT_X_LEAD_VERIFIED={RECT_X_LEAD_VERIFIED}"
    )
    lines.append(rect_x_jbi_warning_comment())
    lines.append("//POS")
    lines.append(f"///NPOS {npos},{npos},{npos},0,0,0")
    lines.append(f"///TOOL {cfg.tool_no}")

    if use_pulse:
        lines.append("///POSTYPE PULSE")
        lines.append("///PULSE")
        for i, q in enumerate(joint_angles_deg):  # type: ignore[arg-type]
            pulses = deg_to_joint_pulses(list(q), cfg.robot_pulses_per_degree)
            lines.append(f"C{i:05d}={','.join(str(p) for p in pulses)}")
    else:
        lines.append("///POSTYPE USER")
        lines.append("///RECTAN")
        lines.append(f"///RCONF {cfg.rconf}")
        for i, p in enumerate(cartesian_points):
            x, y, z = _to_mm(p["x"]), _to_mm(p["y"]), _to_mm(p["z"])
            rx, ry, rz = p.get("rx", 0.0), p.get("ry", 0.0), p.get("rz", 0.0)
            lines.append(
                f"C{i:05d}={_fmt3(x)},{_fmt3(y)},{_fmt3(z)},"
                f"{_fmt(rx)},{_fmt(ry)},{_fmt(rz)}"
            )

    lines.append(
        f"' BC# home={BC_HOME_PULSE}; work zone={st_meta.default_bc} ({cfg.station})"
    )
    for i, bc in enumerate(bc_pulses):
        lines.append(f"BC{i:05d}={bc}")

    if not use_pulse:
        lines.append("///POSTYPE PULSE")
        lines.append("///PULSE")
    for i, ang in enumerate(station_deg):
        e1, e2 = station_deg_to_ec(ang, pulses_per_degree=cfg.station_pulses_per_deg)
        if st_meta.ec_axes >= 2:
            e2, _ = station_deg_to_ec(
                station_deg2[i], pulses_per_degree=cfg.station_pulses_per_deg
            )
        else:
            e2 = 0
        # Always emit two comma-separated values (production INFORM style).
        lines.append(f"EC{i:05d}={e1},{e2}")

    lines.append("//INST")
    lines.append(f"///DATE {datetime.now().strftime('%Y/%m/%d %H:%M')}")
    lines.append("///ATTR SC,RW")
    lines.append("///GROUP1 RB1,BS1")
    lines.append(f"///GROUP2 {cfg.station}")
    lines.append("NOP")

    weave_on = False
    for i in range(npos):
        c = f"C{i:05d}"
        bc = f"BC{i:05d}"
        ec = f"EC{i:05d}"
        role = str(cartesian_points[i].get("role", "")).lower()
        use_joint = (
            cfg.joint_approach
            and i not in arc_on_at
            and (
                i == 0
                or role in ("home", "approach", "retract", "travel")
                or i in travel_indices
            )
        )

        if i in arc_on_at:
            seg = arc_on_at[i]
            lines.append("TIMER T=0.50")
            lines.append(f"MACRO1 MJ#(0) ARGF{cfg.macro_arg}")
            lines.append(_weld_comment(seg.arc_file))
            lines.append("ARCON")
            lines.append(f"WVON WEV#({seg.weave_no})")
            weave_on = True
            lines.append(
                f"MOVL {c} {bc} V={_fmt1(seg.weld_speed)} DEC=20  "
                f"+MOVJ {ec} VJ={_fmt(cfg.joint_speed_pct, 2)}"
            )
        elif use_joint:
            lines.append(
                f"MOVJ {c} {bc} VJ={_fmt(cfg.joint_speed_pct, 2)} DEC=20  "
                f"+MOVJ {ec} VJ={_fmt(cfg.joint_speed_pct, 2)}"
            )
        else:
            in_weld = any(s.start_index < i <= s.end_index for s in weld_segments)
            speed = cfg.weld_speed
            if in_weld:
                for s in weld_segments:
                    if s.start_index < i <= s.end_index:
                        speed = s.weld_speed
                        break
                lines.append(
                    f"MOVL {c} {bc} V={_fmt1(speed)} DEC=20  "
                    f"+MOVJ {ec} VJ={_fmt(cfg.joint_speed_pct, 2)}"
                )
            else:
                lines.append(
                    f"MOVJ {c} {bc} VJ={_fmt(cfg.joint_speed_pct, 2)} DEC=20  "
                    f"+MOVJ {ec} VJ={_fmt(cfg.joint_speed_pct, 2)}"
                )

        if i in arc_off_at:
            lines.append("ARCOF")
            if weave_on:
                lines.append("WVOF")
                weave_on = False

    lines.append("END")
    return "\n".join(lines) + "\n"


def _generate_cartesian_review(
    cartesian_points: list[dict],
    weld_segments: list[WeldSegment],
    cfg: PostprocessorConfig,
    station_deg: list[float] | None,
) -> str:
    """DROP01-inspired USER/RECTAN single-group layout."""
    npos = len(cartesian_points)
    n_ec = npos if cfg.use_station_axis else 0
    arc_on_at: dict[int, WeldSegment] = {s.start_index: s for s in weld_segments}
    arc_off_at: set[int] = {s.end_index for s in weld_segments}
    lines: list[str] = []

    lines.append("/JOB")
    lines.append(f"//NAME {cfg.job_name}")
    lines.append("//POS")
    lines.append(f"///NPOS {npos},0,{n_ec},0,0,0")
    lines.append(f"///USER {cfg.user_frame}")
    lines.append(f"///TOOL {cfg.tool_no}")
    lines.append("///POSTYPE USER")
    lines.append("///RECTAN")
    lines.append(f"///RCONF {cfg.rconf}")

    for i, p in enumerate(cartesian_points):
        x, y, z = _to_mm(p["x"]), _to_mm(p["y"]), _to_mm(p["z"])
        rx, ry, rz = p.get("rx", 0.0), p.get("ry", 0.0), p.get("rz", 0.0)
        lines.append(
            f"C{i:05d}={_fmt3(x)},{_fmt3(y)},{_fmt3(z)},"
            f"{_fmt(rx)},{_fmt(ry)},{_fmt(rz)}"
        )

    if cfg.use_station_axis and station_deg is not None:
        lines.append("///POSTYPE PULSE")
        lines.append("///PULSE")
        for i, ang in enumerate(station_deg):
            lines.append(f"EC{i:05d}={_station_pulse(ang, cfg.station_pulses_per_deg)}")

    lines.append("//INST")
    lines.append(f"///DATE {datetime.now().strftime('%Y/%m/%d %H:%M')}")
    lines.append("///ATTR SC,RW,RJ")
    lines.append(f"////FRAME USER {cfg.user_frame}")
    if cfg.use_station_axis:
        lines.append("///GROUP1 RB1,ST1")
    else:
        lines.append("///GROUP1 RB1")
    lines.append("NOP")

    for i in range(npos):
        use_joint = cfg.joint_approach and i == 0 and i not in arc_on_at
        tag = f"C{i:05d}"
        ec = f" EC{i:05d}" if cfg.use_station_axis else ""
        if use_joint:
            lines.append(f"MOVJ {tag}{ec} VJ={_fmt(cfg.joint_speed_pct, 1)}")
        elif i in arc_on_at:
            seg = arc_on_at[i]
            lines.append(f"MOVL {tag}{ec} V={_fmt(seg.weld_speed, 1)}")
            lines.append(_weld_comment(seg.arc_file))
            lines.append("ARCON")
        else:
            speed = cfg.move_speed
            in_weld = any(s.start_index < i <= s.end_index for s in weld_segments)
            if in_weld:
                for s in weld_segments:
                    if s.start_index < i <= s.end_index:
                        speed = s.weld_speed
                        break
            lines.append(f"MOVL {tag}{ec} V={_fmt(speed, 1)}")
        if i in arc_off_at:
            lines.append("ARCOF")

    lines.append("END")
    return "\n".join(lines) + "\n"


def generate_station_master_jbi(
    master_name: str,
    child_job_name: str,
    station: str = "ST1",
) -> str:
    """Optional station master that ``CALL JOB:`` the weld child (M4.2).

    Matches the empty / call-only header convention from the cell
    (``///NPOS 0,0,0,0,0,0``, ``///ATTR SC,RW,CJ``, ``GROUP1 RB1,BS1`` +
    ``GROUP2 STn`` as in production ``STACJA_*.JBI``).

    This does **not** emit curtain/gate ``DOUT`` or bypass FSU. Production
    masters also ``CALL`` ``RESET-KURTYNY`` / ``ZAMKNIJ-KURTYNA-*`` /
    ``OTWORZ-BRAMA-*`` — keep those jobs on the controller; see
    ``docs/CELL_IO.md`` and ``docs/GO_LIVE.md``.
    """
    st = _normalize_station(station)
    master = str(master_name).strip()
    child = str(child_job_name).strip()
    if not master:
        raise ValueError("master_name must be non-empty")
    if not child:
        raise ValueError("child_job_name must be non-empty")
    # INFORM job names are typically short; strip accidental .JBI suffix.
    if master.upper().endswith(".JBI"):
        master = master[:-4]
    if child.upper().endswith(".JBI"):
        child = child[:-4]

    lines = [
        "/JOB",
        f"//NAME {master}",
        "//POS",
        "///NPOS 0,0,0,0,0,0",
        "//INST",
        f"///DATE {datetime.now().strftime('%Y/%m/%d %H:%M')}",
        "///ATTR SC,RW,CJ",
        "///GROUP1 RB1,BS1",
        f"///GROUP2 {st}",
        "NOP",
        f"' CAD/CAM station wrapper → CALL {child}; curtains stay on controller",
        f"' See docs/CELL_IO.md + docs/GO_LIVE.md — do not bypass FSU/kurtyny",
        f"CALL JOB:{child}",
        "END",
    ]
    return "\n".join(lines) + "\n"


def load_reference_jbi(name: str = "DROP01.JBI") -> str:
    here = Path(__file__).resolve()
    candidates = [
        here.parents[2] / "reference" / "yaskawa-dx200-tools" / "jbi-samples" / name,
        here.parents[2] / "reference" / "robot-cell-dx200" / "jobs" / name,
        here.parents[1] / "tests" / "fixtures" / name,
        here.parents[1] / "tests" / "fixtures" / "robot-cell" / name,
    ]
    for path in candidates:
        if path.is_file():
            return path.read_text(encoding="utf-8", errors="replace")
    raise FileNotFoundError(f"Reference JBI not found: {name}")
