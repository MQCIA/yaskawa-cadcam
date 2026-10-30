"""
Yaskawa DX200 postprocessor: path -> INFORM III ``.JBI`` job.

Two profiles (``PostprocessorConfig.profile``):

* ``ma2010_cell`` (default) — matches production dump jobs
  (``1-DEFRO-5``, ``02025_KOSZ…``, ``11``/``12``, ``1KT``, ``000``):
  TOOL 0, ``///NPOS n,n,n,0,0,0``, ``///POSTYPE PULSE``, GROUP1 RB1,BS1 +
  GROUP2 STn, home bookends (BC=570125), ``MACRO1 MJ#(0) ARGFn`` + plain
  ``ARCON``, optional ``WVON WEV#(n)`` / ``WVOF``, ``DEC=20``, CRLF line ends.
  When ``joint_angles_deg`` is supplied, C#/BC#/EC# are PULSE. Otherwise C#
  stays RECTAN for offline review (home bookends require PULSE joints).

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

from .cell_frames import (
    DEFAULT_TOUCH_SENSE,
    DEFAULT_TOOL_NO,
    DEFAULT_USER_FRAME,
    TOOL0,
    touch_sense_jbi_comments,
)
from .pulse_config import (
    BASE_PULSES_PER_MM,
    BC_HOME_PULSE,
    CELL_PULSE,
    STATION_PULSES_PER_DEGREE,
    deg_to_joint_pulses,
    resolve_base_pulses,
    station_deg_to_ec,
)
from .robot_config import CELL_HOME_PULSES
from .welding_config import MOTOWELD_SCHEDULES, WeldSchedule

# CRLF — Motoman pendant / CF jobs use DOS line endings.
_CRLF = "\r\n"

# Safe home over the rail (repeating C00000 / last point in production jobs).
# Fixture HOME_TRACK.JBI / CELL_POSES; teach variants differ by a few pulses.
SAFE_HOME_PULSES: tuple[int, ...] = CELL_HOME_PULSES
SAFE_HOME_BC: int = BC_HOME_PULSE
# EC seen with home on ST1 one-axis jobs (HOME_TRACK / DEFRO family).
SAFE_HOME_EC: tuple[int, int] = (192, 0)

_VALID_GROUP2 = ("ST1", "ST2", "ST3", "ST4")


@dataclass
class WeldSegment:
    """A contiguous run of path points that should be welded (arc on)."""

    start_index: int
    end_index: int
    weld_speed: float = 5.0  # V= travel (matches cell jobs ~5.0–7.5)
    arc_file: int = 1  # UI condition → Motoweld schedule / ARGF hint
    # None → inherit PostprocessorConfig.default_weave (or no weave if that is None).
    weave_no: int | None = None


@dataclass
class PostprocessorConfig:
    job_name: str = "WELD_AUTO"
    tool_no: int = DEFAULT_TOOL_NO
    user_frame: int = DEFAULT_USER_FRAME
    # "ma2010_cell" | "cartesian_review"
    profile: str = "ma2010_cell"
    move_speed: float = 80.0  # air VJ= on the production cell
    weld_speed: float = 5.0
    joint_speed_pct: float = 80.0
    rconf: str = "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
    use_station_axis: bool = True
    station_pulses_per_deg: float = STATION_PULSES_PER_DEGREE
    joint_approach: bool = True
    # Production TOOL 0 tip (TOOL.CND / cell_frames.TOOL0).
    tool_tcp_mm: tuple[float, float, float] = TOOL0.xyz_mm
    # Default weave; override per WeldSegment or pass weave_no=None to disable.
    default_weave: int | None = 21
    # MACRO1 MJ#(0) ARGFn — production uses 1, 4, 5 (also 2, 8, 11…).
    macro_arg: int = 4
    # Emit full PULSE C# when True and joint_angles_deg is provided.
    prefer_pulse: bool = True
    # Wrap path with safe home (C/BC/EC) at start and end — production pattern.
    bookend_home: bool = True
    # ///GROUP2 STn
    group2_station: str = "ST1"
    home_pulses: tuple[int, ...] = SAFE_HOME_PULSES
    home_bc: int = SAFE_HOME_BC
    home_ec: tuple[int, int] = SAFE_HOME_EC
    # DOS line endings (production CF / pendant).
    crlf: bool = True
    base_park_pulse: int = CELL_PULSE.base_park_pulse
    base_home_pulse: int = CELL_PULSE.base_home_pulse
    base_pulses_per_mm: float = CELL_PULSE.base_pulses_per_mm
    robot_pulses_per_degree: list[float] = field(
        default_factory=lambda: list(CELL_PULSE.robot_pulses_per_degree)
    )
    # SZUKAJ1 pattern: CALL JOB:SRCH-2D1 before first ARCON (needs job on CF).
    emit_touch_sense_call: bool = False


def _fmt(v: float, decimals: int = 4) -> str:
    return f"{v:.{decimals}f}"


def _fmt3(v: float) -> str:
    return f"{v:.3f}"


def _fmt1(v: float) -> str:
    return f"{v:.1f}"


def _join(lines: list[str], crlf: bool) -> str:
    nl = _CRLF if crlf else "\n"
    return nl.join(lines) + nl


def _normalize_group2(station: str | None) -> str:
    if not station:
        return "ST1"
    key = station.upper().replace(" ", "")
    if key.isdigit():
        key = f"ST{key}"
    if not key.startswith("ST"):
        key = f"ST{key}"
    if key not in _VALID_GROUP2:
        raise ValueError(f"GROUP2 station must be ST1..ST4, got '{station}'")
    return key


def _weld_comment(cond_no: int, macro_arg: int) -> str:
    sched: WeldSchedule | None = MOTOWELD_SCHEDULES.get(cond_no)
    if not sched:
        return f"' MOTOWELD-E MACRO1 ARGF{macro_arg} (cond {cond_no})"
    return (
        f"' MOTOWELD-E cond {cond_no} / MACRO1 ARGF{macro_arg} "
        f"(schedule ARGF{sched.argf}): "
        f"{sched.current_a:.0f}A / {sched.voltage_v:.1f}V / "
        f"wire {sched.wire_speed:.1f} m/min — UI estimate; verify ARCSRT.CND"
    )


def _macro_arg_for_segment(seg: WeldSegment, cfg: PostprocessorConfig) -> int:
    """Prefer schedule.argf for the segment condition; else config.macro_arg."""
    sched = MOTOWELD_SCHEDULES.get(seg.arc_file)
    if sched is not None:
        return int(sched.argf)
    return int(cfg.macro_arg)


def _weave_for_segment(seg: WeldSegment, cfg: PostprocessorConfig) -> int | None:
    if seg.weave_no is not None:
        return seg.weave_no
    return cfg.default_weave


def _to_mm(v: float) -> float:
    return v * 1000.0 if abs(v) < 10 else v


def _station_pulse(deg: float, pulses_per_deg: float) -> int:
    return int(round(deg * pulses_per_deg))


def generate_jbi(
    cartesian_points: list[dict],
    weld_segments: list[WeldSegment] | None = None,
    config: PostprocessorConfig | None = None,
    station_deg: list[float] | None = None,
    joint_angles_deg: list[list[float]] | None = None,
    *,
    base_pulse: list[int] | int | None = None,
    base_mm: list[float] | float | None = None,
    station: str | None = None,
    macro_arg: int | None = None,
    weave_no: int | None = None,
    bookend_home: bool | None = None,
    disable_weave: bool = False,
    emit_touch_sense_call: bool | None = None,
) -> str:
    """Generate a DX200 ``.JBI`` job from Cartesian TCP points.

    Pass ``joint_angles_deg`` (one [S,L,U,R,B,T] per point) to emit production
    ``///POSTYPE PULSE`` C# values using ``pulse_config`` scaling.

    RECT-X ``BC#`` per point: explicit ``base_pulse`` / ``base_mm``, or
    ``station`` midpoint (ST1..ST4), else auto from each point's world Z
    (rail along Z; ``mm=0`` at ``BC_HOME_PULSE``).

    ``station`` also sets ``///GROUP2 STn`` when provided.
    Home bookends (safe C/BC over the rail) are on by default when PULSE
    joints are supplied (``bookend_home``).

    ``emit_touch_sense_call`` inserts ``CALL JOB:SRCH-2D1`` before the first
    ARCON (SZUKAJ1 pattern — job must exist on the controller / MotoSim).
    """
    cfg = config or PostprocessorConfig()
    if macro_arg is not None:
        cfg.macro_arg = macro_arg
    if disable_weave:
        cfg.default_weave = None
    elif weave_no is not None:
        cfg.default_weave = weave_no
    if bookend_home is not None:
        cfg.bookend_home = bookend_home
    if emit_touch_sense_call is not None:
        cfg.emit_touch_sense_call = emit_touch_sense_call
    if station:
        cfg.group2_station = _normalize_group2(station)

    weld_segments = list(weld_segments or [])
    if disable_weave:
        weld_segments = [
            WeldSegment(
                start_index=s.start_index,
                end_index=s.end_index,
                weld_speed=s.weld_speed,
                arc_file=s.arc_file,
                weave_no=None,
            )
            for s in weld_segments
        ]
    elif weave_no is not None:
        weld_segments = [
            WeldSegment(
                start_index=s.start_index,
                end_index=s.end_index,
                weld_speed=s.weld_speed,
                arc_file=s.arc_file,
                weave_no=weave_no,
            )
            for s in weld_segments
        ]

    if station_deg is not None:
        cfg.use_station_axis = True
    elif cfg.profile == "ma2010_cell":
        cfg.use_station_axis = True
        station_deg = [0.0] * len(cartesian_points)

    if cfg.profile == "ma2010_cell":
        return _generate_ma2010_cell(
            cartesian_points,
            weld_segments,
            cfg,
            station_deg,
            joint_angles_deg,
            base_pulse=base_pulse,
            base_mm=base_mm,
            station=station,
        )
    return _generate_cartesian_review(cartesian_points, weld_segments, cfg, station_deg)


def _apply_home_bookends(
    cartesian_points: list[dict],
    weld_segments: list[WeldSegment],
    station_deg: list[float],
    joint_angles_deg: list[list[float]],
    bc_list: list[int],
    cfg: PostprocessorConfig,
) -> tuple[
    list[dict],
    list[WeldSegment],
    list[list[float]],
    list[int],
    list[tuple[int, int]],
]:
    """Prepend/append safe home. Path joints stay in degrees; home C uses pulses."""
    home_pt = {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "rx": 0.0,
        "ry": 0.0,
        "rz": 0.0,
        "_home": True,
    }
    points = [home_pt, *cartesian_points, dict(home_pt)]
    segs = [
        WeldSegment(
            start_index=s.start_index + 1,
            end_index=s.end_index + 1,
            weld_speed=s.weld_speed,
            arc_file=s.arc_file,
            weave_no=s.weave_no,
        )
        for s in weld_segments
    ]
    ec_home = (int(cfg.home_ec[0]), int(cfg.home_ec[1]))
    ec_pairs: list[tuple[int, int]] = [ec_home]
    for ang in station_deg:
        ec_pairs.append(
            station_deg_to_ec(ang, pulses_per_degree=cfg.station_pulses_per_deg)
        )
    ec_pairs.append(ec_home)

    # Placeholder deg rows for home slots (overridden when emitting C#).
    home_deg_placeholder = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joints = [home_deg_placeholder, *joint_angles_deg, home_deg_placeholder]
    bc = [int(cfg.home_bc), *bc_list, int(cfg.home_bc)]
    return points, segs, joints, bc, ec_pairs


def _generate_ma2010_cell(
    cartesian_points: list[dict],
    weld_segments: list[WeldSegment],
    cfg: PostprocessorConfig,
    station_deg: list[float] | None,
    joint_angles_deg: list[list[float]] | None,
    *,
    base_pulse: list[int] | int | None = None,
    base_mm: list[float] | float | None = None,
    station: str | None = None,
) -> str:
    """INST layout matching production MA2010 cell jobs."""
    path_n = len(cartesian_points)
    station_deg = station_deg or [0.0] * path_n
    group2 = _normalize_group2(station or cfg.group2_station)

    bc_path = resolve_base_pulses(
        path_n,
        base_pulse=base_pulse,
        base_mm=base_mm,
        points=cartesian_points,
        station=station,
        default_park=False,
    )

    use_pulse = (
        bool(cfg.prefer_pulse)
        and joint_angles_deg is not None
        and len(joint_angles_deg) == path_n
    )
    # Home bookends need PULSE C# (production grammar).
    do_bookend = bool(cfg.bookend_home) and use_pulse

    home_pulse_idx: set[int] = set()
    if do_bookend:
        assert joint_angles_deg is not None
        cartesian_points, weld_segments, joint_angles_deg, bc_list, ec_pairs = (
            _apply_home_bookends(
                cartesian_points,
                weld_segments,
                station_deg,
                joint_angles_deg,
                bc_path,
                cfg,
            )
        )
        home_pulse_idx = {0, len(cartesian_points) - 1}
        use_pulse = True
    else:
        bc_list = bc_path
        ec_pairs = [
            station_deg_to_ec(ang, pulses_per_degree=cfg.station_pulses_per_deg)
            for ang in station_deg
        ]

    npos = len(cartesian_points)
    arc_on_at: dict[int, WeldSegment] = {s.start_index: s for s in weld_segments}
    arc_off_at: set[int] = {s.end_index for s in weld_segments}
    lines: list[str] = []

    lines.append("/JOB")
    lines.append(f"//NAME {cfg.job_name}")
    if use_pulse:
        lines.append("' PULSE export — DX200 MA2010 + RECT-X + TURN + MOTOWELD-E")
    else:
        lines.append("' Offline RECTAN review — supply IK joints for PULSE C#")
    lines.append(
        f"' Cell: DX200 · MA2010 · RECT-X · TURN-{group2} · MOTOWELD-E 350A; "
        f"TOOL0 TCP mm {cfg.tool_tcp_mm[0]},{cfg.tool_tcp_mm[1]},{cfg.tool_tcp_mm[2]}"
    )
    lines.append(
        f"' Station pulses/deg~{cfg.station_pulses_per_deg:.4f}; "
        f"RECT-X ~{cfg.base_pulses_per_mm:.4f} p/mm "
        f"(home BC={cfg.base_home_pulse}, park={cfg.base_park_pulse})"
    )
    if do_bookend:
        lines.append(
            f"' Home bookends C00000/C{npos - 1:05d} = "
            f"{','.join(str(p) for p in cfg.home_pulses)} BC={cfg.home_bc}"
        )
    lines.append("//POS")
    lines.append(f"///NPOS {npos},{npos},{npos},0,0,0")
    lines.append(f"///TOOL {cfg.tool_no}")

    if use_pulse:
        lines.append("///POSTYPE PULSE")
        lines.append("///PULSE")
        for i in range(npos):
            if i in home_pulse_idx:
                pulses = [int(p) for p in cfg.home_pulses]
            else:
                q = joint_angles_deg[i]  # type: ignore[index]
                pulses = deg_to_joint_pulses(q, cfg.robot_pulses_per_degree)
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
        f"' BC# RECT-X BS1 (~{BASE_PULSES_PER_MM:.4f} p/mm; "
        f"0 mm @ BC={BC_HOME_PULSE})"
    )
    for i, bc in enumerate(bc_list):
        lines.append(f"BC{i:05d}={bc}")

    if not use_pulse:
        lines.append("///POSTYPE PULSE")
        lines.append("///PULSE")
    for i, (e1, e2) in enumerate(ec_pairs):
        lines.append(f"EC{i:05d}={e1},{e2}")

    lines.append("//INST")
    lines.append(f"///DATE {datetime.now().strftime('%Y/%m/%d %H:%M')}")
    lines.append("///ATTR SC,RW")
    lines.append("///GROUP1 RB1,BS1")
    lines.append(f"///GROUP2 {group2}")
    lines.append("NOP")

    weave_on = False
    # Motoweld preset: emit MACRO1 at the first ARCON, then again only when
    # the schedule / ARGF changes between seams (02025_KOSZ_… pattern).
    last_macro_arg: int | None = None
    touch_sense_emitted = False
    for i in range(npos):
        c = f"C{i:05d}"
        bc = f"BC{i:05d}"
        ec = f"EC{i:05d}"
        is_home = i in home_pulse_idx
        use_joint = is_home or (
            cfg.joint_approach and i == 0 and i not in arc_on_at
        )

        if i in arc_on_at:
            seg = arc_on_at[i]
            argf = _macro_arg_for_segment(seg, cfg)
            weave = _weave_for_segment(seg, cfg)

            # SZUKAJ1 → CALL JOB:SRCH-2D1 once before the first weld.
            if cfg.emit_touch_sense_call and not touch_sense_emitted:
                lines.extend(
                    touch_sense_jbi_comments(
                        DEFAULT_TOUCH_SENSE, emit_call=True
                    )
                )
                touch_sense_emitted = True

            lines.append("TIMER T=0.50")
            if last_macro_arg is None or argf != last_macro_arg:
                lines.append(f"MACRO1 MJ#(0) ARGF{argf}")
                lines.append(_weld_comment(seg.arc_file, argf))
                last_macro_arg = argf
            lines.append("ARCON")
            if weave is not None:
                lines.append(f"WVON WEV#({weave})")
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
            # WVOF before ARCOF (clear weave before extinguish).
            if weave_on:
                lines.append("WVOF")
                weave_on = False
            lines.append("ARCOF")

    lines.append("END")
    return _join(lines, cfg.crlf)


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
            # Keep single-value EC for DROP01-like review; ma2010_cell uses v1,v2.
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
            lines.append(_weld_comment(seg.arc_file, cfg.macro_arg))
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
    return _join(lines, cfg.crlf)


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
