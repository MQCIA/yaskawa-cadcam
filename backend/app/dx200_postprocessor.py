"""
Yaskawa DX200 postprocessor: path -> INFORM III ``.JBI`` job.

Two profiles (``PostprocessorConfig.profile``):

* ``ma2010_cell`` (default) — matches the production dump in
  ``reference/robot-cell-dx200``:
  TOOL 0, GROUP1 RB1,BS1 + GROUP2 ST1, ARCON/ARCOF, WVON, MACRO1, coordinated
  ``+MOVJ EC#``. When ``joint_angles_deg`` is supplied, C#/BC#/EC# are emitted
  as ``///POSTYPE PULSE`` (cell teach format). Otherwise C# stays RECTAN for
  offline review and only EC#/BC# use pulse units.

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

from .pulse_config import (
    CELL_PULSE,
    STATION_PULSES_PER_DEGREE,
    base_mm_to_bc,
    deg_to_joint_pulses,
    station_deg_to_ec,
)
from .welding_config import LORCH_S8_SCHEDULES, WeldSchedule


@dataclass
class WeldSegment:
    """A contiguous run of path points that should be welded (arc on)."""

    start_index: int
    end_index: int
    weld_speed: float = 5.0  # V= travel (matches cell jobs ~5.0–7.5)
    arc_file: int = 1
    weave_no: int = 21  # WEV#(21) from WEAV.CND — default production weave
    # INFORM move for weld samples: MOVL (line) or MOVC (circle, ≥3 pts).
    # Matches production 12.JBI / 123.JBI pipe arcs.
    move_type: str = "MOVL"


@dataclass
class PostprocessorConfig:
    job_name: str = "WELD_AUTO"
    tool_no: int = 0
    user_frame: int = 1
    # "ma2010_cell" | "cartesian_review"
    profile: str = "ma2010_cell"
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
    # Emit full PULSE C# when True and joint_angles_deg is provided.
    prefer_pulse: bool = True
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


def _ec_from_point(p: dict, fallback: tuple[int, int]) -> tuple[int, int]:
    """Read per-point EC pair from ``ec`` / ``ec1``+``ec2`` keys."""
    raw = p.get("ec")
    if isinstance(raw, (list, tuple)) and len(raw) >= 2:
        return int(raw[0]), int(raw[1])
    if "ec1" in p or "ec2" in p:
        return int(p.get("ec1", fallback[0])), int(p.get("ec2", fallback[1]))
    return fallback


def _resolve_ec_pairs(
    cartesian_points: list[dict],
    station_deg: list[float] | None,
    cfg: PostprocessorConfig,
    ec_pulses: list[tuple[int, int]] | list[list[int]] | None,
) -> list[tuple[int, int]]:
    """Build EC# pairs — explicit pulses beat station_deg (ST3 2-axis pattern).

    Production ``1-DEFRO-5.JBI`` keeps C# fixed while EC axis-2 rotates
    (``EC00004=-2638,-94527`` → ``EC00005=-2638,-726117``). Pass those pairs
    via ``ec_pulses`` or each point's ``ec`` field.
    """
    n = len(cartesian_points)
    if ec_pulses is not None:
        if len(ec_pulses) != n:
            raise ValueError(
                f"ec_pulses length {len(ec_pulses)} != number of points {n}"
            )
        return [(int(a), int(b)) for a, b in ec_pulses]

    station_deg = station_deg or [0.0] * n
    if len(station_deg) != n:
        raise ValueError("station_deg length must match number of points")
    pairs: list[tuple[int, int]] = []
    for i, p in enumerate(cartesian_points):
        base = station_deg_to_ec(
            station_deg[i], pulses_per_degree=cfg.station_pulses_per_deg
        )
        pairs.append(_ec_from_point(p, base))
    return pairs


def _move_opcode_for_point(
    point: dict,
    index: int,
    weld_segments: list[WeldSegment],
    *,
    in_weld: bool,
) -> str:
    """Choose MOVJ / MOVL / MOVC for one INST line.

    Default air / linear weld is ``MOVL``. ``MOVC`` only when the point or its
    weld segment opts in (CAD arc / pipe pattern). Explicit ``move_type`` wins.
    """
    explicit = str(
        point.get("move_type") or point.get("move") or ""
    ).upper()
    if explicit in ("MOVJ", "MOVL", "MOVC"):
        return explicit
    if in_weld:
        for s in weld_segments:
            if s.start_index <= index <= s.end_index:
                mt = str(getattr(s, "move_type", "MOVL") or "MOVL").upper()
                return mt if mt in ("MOVL", "MOVC") else "MOVL"
    return "MOVL"


def generate_jbi(
    cartesian_points: list[dict],
    weld_segments: list[WeldSegment] | None = None,
    config: PostprocessorConfig | None = None,
    station_deg: list[float] | None = None,
    joint_angles_deg: list[list[float]] | None = None,
    *,
    ec_pulses: list[tuple[int, int]] | list[list[int]] | None = None,
) -> str:
    """Generate a DX200 ``.JBI`` job from Cartesian TCP points.

    Pass ``joint_angles_deg`` (one [S,L,U,R,B,T] per point) to emit production
    ``///POSTYPE PULSE`` C# values using ``pulse_config`` scaling.

    ``ec_pulses`` — optional per-point ``(ec1, ec2)`` for TURN ST1–ST4 /
    circumferential welding (``1-DEFRO-5.JBI`` ST3 axis-2). When omitted, EC
    comes from ``station_deg`` or each point's ``ec`` field.

    Points / weld segments with ``move_type="MOVC"`` emit INFORM ``MOVC``
    (pipe arcs in ``12.JBI`` / ``123.JBI``; ≥3 points per arc).
    """
    cfg = config or PostprocessorConfig()
    weld_segments = weld_segments or []
    if station_deg is not None or ec_pulses is not None:
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
            ec_pulses=ec_pulses,
        )
    return _generate_cartesian_review(
        cartesian_points, weld_segments, cfg, station_deg, ec_pulses=ec_pulses
    )


def _generate_ma2010_cell(
    cartesian_points: list[dict],
    weld_segments: list[WeldSegment],
    cfg: PostprocessorConfig,
    station_deg: list[float] | None,
    joint_angles_deg: list[list[float]] | None,
    *,
    ec_pulses: list[tuple[int, int]] | list[list[int]] | None = None,
) -> str:
    """INST layout matching ``000.JBI`` / ``12.JBI`` from the production cell."""
    npos = len(cartesian_points)
    station_deg = station_deg or [0.0] * npos
    use_pulse = (
        bool(cfg.prefer_pulse)
        and joint_angles_deg is not None
        and len(joint_angles_deg) == npos
    )
    arc_on_at: dict[int, WeldSegment] = {s.start_index: s for s in weld_segments}
    arc_off_at: set[int] = {s.end_index for s in weld_segments}
    ec_pairs = _resolve_ec_pairs(cartesian_points, station_deg, cfg, ec_pulses)
    lines: list[str] = []

    lines.append("/JOB")
    lines.append(f"//NAME {cfg.job_name}")
    if use_pulse:
        lines.append("' PULSE export — MA2010 cell (RC1G / RBCALIB scaling)")
    else:
        lines.append("' Offline RECTAN review — supply IK joints for PULSE C#")
    lines.append(
        f"' Cell: MA2010 + RECT-X (BS1) + TURN (ST1); TOOL0 TCP mm "
        f"{cfg.tool_tcp_mm[0]},{cfg.tool_tcp_mm[1]},{cfg.tool_tcp_mm[2]}"
    )
    lines.append(
        f"' Station pulses/deg~{cfg.station_pulses_per_deg:.4f}; "
        f"BC park={cfg.base_park_pulse}"
    )
    lines.append(
        "' MOVC = circular weld (≥3 pts, 12/123.JBI); "
        "per-point EC enables ST3 circumferential (1-DEFRO-5.JBI)"
    )
    lines.append("//POS")
    lines.append(f"///NPOS {npos},{npos},{npos},0,0,0")
    lines.append(f"///TOOL {cfg.tool_no}")

    if use_pulse:
        lines.append("///POSTYPE PULSE")
        lines.append("///PULSE")
        for i, q in enumerate(joint_angles_deg):  # type: ignore[arg-type]
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

    lines.append(f"' BC# RECT-X park pulse ({cfg.base_park_pulse})")
    for i in range(npos):
        lines.append(f"BC{i:05d}={base_mm_to_bc(park_pulse=cfg.base_park_pulse)}")

    if not use_pulse:
        lines.append("///POSTYPE PULSE")
        lines.append("///PULSE")
    for i, (e1, e2) in enumerate(ec_pairs):
        lines.append(f"EC{i:05d}={e1},{e2}")

    lines.append("//INST")
    lines.append(f"///DATE {datetime.now().strftime('%Y/%m/%d %H:%M')}")
    lines.append("///ATTR SC,RW")
    lines.append("///GROUP1 RB1,BS1")
    lines.append("///GROUP2 ST1")
    lines.append("NOP")

    weave_on = False
    for i in range(npos):
        c = f"C{i:05d}"
        bc = f"BC{i:05d}"
        ec = f"EC{i:05d}"
        use_joint = cfg.joint_approach and i == 0 and i not in arc_on_at
        in_weld = any(s.start_index <= i <= s.end_index for s in weld_segments)
        opcode = _move_opcode_for_point(
            cartesian_points[i], i, weld_segments, in_weld=in_weld
        )

        if i in arc_on_at:
            seg = arc_on_at[i]
            lines.append("TIMER T=0.50")
            lines.append(f"MACRO1 MJ#(0) ARGF{cfg.macro_arg}")
            lines.append(_weld_comment(seg.arc_file))
            lines.append("ARCON")
            lines.append(f"WVON WEV#({seg.weave_no})")
            weave_on = True
            weld_op = str(seg.move_type or opcode or "MOVL").upper()
            if weld_op not in ("MOVL", "MOVC"):
                weld_op = "MOVL"
            lines.append(
                f"{weld_op} {c} {bc} V={_fmt1(seg.weld_speed)} DEC=20  "
                f"+MOVJ {ec} VJ={_fmt(cfg.joint_speed_pct, 2)}"
            )
        elif use_joint or opcode == "MOVJ":
            lines.append(
                f"MOVJ {c} {bc} VJ={_fmt(cfg.joint_speed_pct, 2)} DEC=20  "
                f"+MOVJ {ec} VJ={_fmt(cfg.joint_speed_pct, 2)}"
            )
        elif in_weld:
            speed = cfg.weld_speed
            for s in weld_segments:
                if s.start_index < i <= s.end_index:
                    speed = s.weld_speed
                    break
            op = opcode if opcode in ("MOVL", "MOVC") else "MOVL"
            lines.append(
                f"{op} {c} {bc} V={_fmt1(speed)} DEC=20  "
                f"+MOVJ {ec} VJ={_fmt(cfg.joint_speed_pct, 2)}"
            )
        else:
            # Air move without explicit MOVJ — production uses MOVJ between seams.
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
    *,
    ec_pulses: list[tuple[int, int]] | list[list[int]] | None = None,
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

    if cfg.use_station_axis:
        lines.append("///POSTYPE PULSE")
        lines.append("///PULSE")
        if ec_pulses is not None:
            for i, (e1, e2) in enumerate(ec_pulses):
                lines.append(f"EC{i:05d}={int(e1)},{int(e2)}")
        elif station_deg is not None:
            for i, ang in enumerate(station_deg):
                lines.append(
                    f"EC{i:05d}={_station_pulse(ang, cfg.station_pulses_per_deg)}"
                )

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
        in_weld = any(s.start_index <= i <= s.end_index for s in weld_segments)
        opcode = _move_opcode_for_point(
            cartesian_points[i], i, weld_segments, in_weld=in_weld
        )
        if use_joint:
            lines.append(f"MOVJ {tag}{ec} VJ={_fmt(cfg.joint_speed_pct, 1)}")
        elif i in arc_on_at:
            seg = arc_on_at[i]
            weld_op = str(seg.move_type or opcode or "MOVL").upper()
            if weld_op not in ("MOVL", "MOVC"):
                weld_op = "MOVL"
            lines.append(f"{weld_op} {tag}{ec} V={_fmt(seg.weld_speed, 1)}")
            lines.append(_weld_comment(seg.arc_file))
            lines.append("ARCON")
        else:
            speed = cfg.move_speed
            if in_weld:
                for s in weld_segments:
                    if s.start_index < i <= s.end_index:
                        speed = s.weld_speed
                        break
            if opcode == "MOVJ":
                lines.append(f"MOVJ {tag}{ec} VJ={_fmt(cfg.joint_speed_pct, 1)}")
            else:
                op = opcode if opcode in ("MOVL", "MOVC") else "MOVL"
                lines.append(f"{op} {tag}{ec} V={_fmt(speed, 1)}")
        if i in arc_off_at:
            lines.append("ARCOF")

    lines.append("END")
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
