"""
Yaskawa DX200 postprocessor: joint/Cartesian path -> INFORM III ``.JBI`` job.

============================================================================
  !!!  UWAGA / WARNING  !!!
============================================================================
The exact ``.JBI`` layout (RCONF configuration word, TOOL number, USER frame,
pulse<->degree scaling, speed tags, weld condition file numbers referenced by
ARCON/ARCOF) is CELL- AND ROBOT-SPECIFIC. The output below is a structurally
plausible template only. Always:
  * generate/inspect a reference job on your real DX200 (or in MotoSim),
  * diff it against this output,
  * import into MotoSim and dry-run with collision checking,
before loading anything onto a controller. Do not run generated jobs on a
live cell without a qualified Yaskawa integrator signing off.
============================================================================
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime


@dataclass
class WeldSegment:
    """A contiguous run of path points that should be welded (arc on)."""

    start_index: int  # first point index (inclusive) where the arc is ON
    end_index: int    # last point index (inclusive) where the arc is ON
    weld_speed: float = 10.0   # cm/min or the unit your weld schedule uses
    arc_file: int = 1          # ASF#/weld condition file referenced by ARCON


@dataclass
class PostprocessorConfig:
    job_name: str = "WELD_AUTO"
    tool_no: int = 0
    move_speed: float = 10.0   # V= tag for approach/air moves (mm/s style)
    weld_speed: float = 10.0
    # RCONF placeholder exactly as requested; MUST match your teach config.
    rconf: str = "1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"


def _fmt(v: float) -> str:
    return f"{v:.4f}"


def generate_jbi(
    cartesian_points: list[dict],
    weld_segments: list[WeldSegment] | None = None,
    config: PostprocessorConfig | None = None,
) -> str:
    """Generate a DX200 ``.JBI`` job from Cartesian TCP points.

    ``cartesian_points`` is an ordered list of {x,y,z,rx,ry,rz}. Coordinates
    are emitted in millimetres and degrees (Yaskawa RECTAN convention), so
    inputs given in metres are scaled by 1000.

    ``weld_segments`` marks which point ranges get ARCON/ARCOF wrapped around
    the MOVL instructions.
    """
    cfg = config or PostprocessorConfig()
    weld_segments = weld_segments or []

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

    # ---- Position variables (C-variables, Cartesian) ----------------------
    for i, p in enumerate(cartesian_points):
        x = p["x"] * 1000.0 if abs(p["x"]) < 10 else p["x"]
        y = p["y"] * 1000.0 if abs(p["y"]) < 10 else p["y"]
        z = p["z"] * 1000.0 if abs(p["z"]) < 10 else p["z"]
        rx, ry, rz = p.get("rx", 0.0), p.get("ry", 0.0), p.get("rz", 0.0)
        lines.append(
            f"C{i:05d}={_fmt(x)},{_fmt(y)},{_fmt(z)},{_fmt(rx)},{_fmt(ry)},{_fmt(rz)}"
        )

    # ---- Instruction section ---------------------------------------------
    lines.append("//INST")
    lines.append(f"///DATE {datetime.now().strftime('%Y/%m/%d %H:%M')}")
    lines.append("///ATTR SC,RW")
    lines.append("///GROUP1 RB1")
    lines.append("NOP")

    for i in range(npos):
        if i in arc_on_at:
            seg = arc_on_at[i]
            lines.append(f"MOVL C{i:05d} V={_fmt(seg.weld_speed)}")
            lines.append(f"ARCON ASF#({seg.arc_file})")
        else:
            lines.append(f"MOVL C{i:05d} V={_fmt(cfg.move_speed)}")

        if i in arc_off_at:
            lines.append("ARCOF")

    lines.append("END")

    return "\n".join(lines) + "\n"
