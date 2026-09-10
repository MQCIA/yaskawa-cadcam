"""
Yaskawa DX200 postprocessor: path -> INFORM III ``.JBI`` job.

Cell: Yaskawa AR-series robot + H1000D positioner (external station axis) +
Lorch S8 power source.

============================================================================
  !!!  UWAGA / WARNING  !!!
============================================================================
The exact ``.JBI`` layout (RCONF configuration word, TOOL/USER frames, external
/ station-axis handling, coordinated (SMOVL) motion, pulse<->degree scaling,
speed tags, and the weld condition numbers referenced by ARCON/ARCOF) is CELL-
AND ROBOT-SPECIFIC. The output is a structurally plausible template only.
Always generate/inspect a reference job on your real DX200 (or MotoSim), diff
it against this output, then import into MotoSim and dry-run with collision
checking before loading a controller. Coordinated robot+positioner welding in
particular MUST be validated in simulation.
============================================================================
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime

from .welding_config import LORCH_S8_SCHEDULES, WeldSchedule


@dataclass
class WeldSegment:
    """A contiguous run of path points that should be welded (arc on)."""

    start_index: int
    end_index: int
    weld_speed: float = 10.0   # travel speed tag for the welded moves
    arc_file: int = 1          # ARCON weld condition number (Lorch schedule)


@dataclass
class PostprocessorConfig:
    job_name: str = "WELD_AUTO"
    tool_no: int = 0
    move_speed: float = 10.0
    weld_speed: float = 10.0
    # RCONF placeholder; MUST match your teach configuration on the DX200.
    rconf: str = "1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
    # External/station axis (H1000D). If True, positions carry a station value.
    use_station_axis: bool = False
    station_axis_label: str = "S1E"


def _fmt(v: float) -> str:
    return f"{v:.4f}"


def _weld_comment(cond_no: int) -> str:
    sched: WeldSchedule | None = LORCH_S8_SCHEDULES.get(cond_no)
    if not sched:
        return f"' weld condition {cond_no} (define in welding_config.py)"
    return (
        f"' Lorch S8 job {sched.lorch_job}: {sched.current_a:.0f}A / "
        f"{sched.voltage_v:.1f}V / wire {sched.wire_speed:.1f} m/min / "
        f"{sched.process} / {sched.gas}"
    )


def generate_jbi(
    cartesian_points: list[dict],
    weld_segments: list[WeldSegment] | None = None,
    config: PostprocessorConfig | None = None,
    station_deg: list[float] | None = None,
) -> str:
    """Generate a DX200 ``.JBI`` job from Cartesian TCP points.

    ``cartesian_points`` : ordered {x,y,z,rx,ry,rz}; coordinates emitted in mm
                           and degrees (RECTAN). Metres are scaled by 1000.
    ``weld_segments``    : point ranges wrapped with ARCON/ARCOF.
    ``station_deg``      : optional per-point H1000D positioner angle [deg];
                           enables the external-axis annotations.
    """
    cfg = config or PostprocessorConfig()
    weld_segments = weld_segments or []
    if station_deg is not None:
        cfg.use_station_axis = True

    npos = len(cartesian_points)
    arc_on_at: dict[int, WeldSegment] = {s.start_index: s for s in weld_segments}
    arc_off_at: set[int] = {s.end_index for s in weld_segments}

    lines: list[str] = []

    # ---- Header -----------------------------------------------------------
    lines.append("/JOB")
    lines.append(f"//NAME {cfg.job_name}")
    lines.append("//POS")
    lines.append(f"///NPOS {npos},0,0,0,0,0")
    lines.append(f"///TOOL {cfg.tool_no}")
    lines.append("///POSTYPE ROBOT")
    lines.append("///RECTAN")
    lines.append(f"///RCONF {cfg.rconf}")

    # ---- Robot position variables (C-variables, Cartesian) ----------------
    for i, p in enumerate(cartesian_points):
        x = p["x"] * 1000.0 if abs(p["x"]) < 10 else p["x"]
        y = p["y"] * 1000.0 if abs(p["y"]) < 10 else p["y"]
        z = p["z"] * 1000.0 if abs(p["z"]) < 10 else p["z"]
        rx, ry, rz = p.get("rx", 0.0), p.get("ry", 0.0), p.get("rz", 0.0)
        lines.append(
            f"C{i:05d}={_fmt(x)},{_fmt(y)},{_fmt(z)},{_fmt(rx)},{_fmt(ry)},{_fmt(rz)}"
        )

    # ---- External / station axis (H1000D positioner) ----------------------
    # NOTE: On a real DX200 the station axis is carried in EC#/base-axis fields
    # tied to each robot position (coordinated motion). Emitted here as EC
    # variables for review; reconcile with your controller's job format.
    if cfg.use_station_axis and station_deg is not None:
        lines.append(f"//POS-EX {cfg.station_axis_label}")
        for i, ang in enumerate(station_deg):
            lines.append(f"EC{i:05d}={_fmt(ang)}")

    # ---- Instruction section ---------------------------------------------
    lines.append("//INST")
    lines.append(f"///DATE {datetime.now().strftime('%Y/%m/%d %H:%M')}")
    lines.append("///ATTR SC,RW")
    if cfg.use_station_axis:
        lines.append("///GROUP1 RB1,ST1")   # robot + station coordinated group
    else:
        lines.append("///GROUP1 RB1")
    lines.append("NOP")

    for i in range(npos):
        move = "SMOVL" if cfg.use_station_axis else "MOVL"
        if i in arc_on_at:
            seg = arc_on_at[i]
            lines.append(f"{move} C{i:05d} V={_fmt(seg.weld_speed)}")
            lines.append(_weld_comment(seg.arc_file))
            lines.append(f"ARCON ASF#({seg.arc_file})")
        else:
            lines.append(f"{move} C{i:05d} V={_fmt(cfg.move_speed)}")

        if i in arc_off_at:
            lines.append("ARCOF")

    lines.append("END")

    return "\n".join(lines) + "\n"
