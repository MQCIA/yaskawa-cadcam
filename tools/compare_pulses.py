#!/usr/bin/env python3
"""Round-trip production INFORM PULSE jobs through the cell scale and MA2010 URDF.

Usage
-----
    python tools/compare_pulses.py path/to/job.JBI
    python tools/compare_pulses.py path/to/jobs_dir
    python tools/compare_pulses.py job.JBI --no-ik
    python tools/compare_pulses.py backend/tests/fixtures/robot-cell --analyze-stations

For every ``C#`` the script:

1. Converts pulses to degrees with ``app.pulse_config.joint_pulses_to_deg``.
2. Runs MA2010 FK (ROS-Industrial URDF chain) and applies TOOL 0 so the
   reported pose is the torch TCP, not the flange.
3. Solves IK for that TCP seeded with the previous taught point, or with
   ``C00000`` / cell home when there is no previous point.
4. Converts the IK joints back to pulses with ``deg_to_joint_pulses``.

``ΔPULSE`` / ``ΔDEG`` measure numeric IK consistency on the URDF model.
They are not the error against the pendant XYZ (still needs MotoSim /
pendant golden Cartesian for M1.4 sign-off).

``BC#`` stays in pulses (RECT-X mm/pulse is not certified). ``EC#`` is two
components: axis 1 uses the RBCALIB TURN scale (91771 pulse / 90°); axis 2
is printed in pulses only.

``--analyze-stations`` gathers ARCON / MOVL / MOVC work points per GROUP2
``ST1``…``ST4``, reports TCP in the robot-base frame, RECT-X rail offset from
``BC#``, and the weld-cloud centre + AABB used by ``STATION_FRAMES``.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "backend"))

from app.kinematics_engine import forward_kinematics, solve_ik_path  # noqa: E402
from app.pulse_config import (  # noqa: E402
    CELL_PULSE,
    RC1G_SOFTLIMIT_NEG,
    RC1G_SOFTLIMIT_POS,
    STATION_BC_MID,
    deg_to_joint_pulses,
    joint_pulses_to_deg,
    track_pulses_to_mm,
)

AXIS = ("S", "L", "U", "R", "B", "T")
OPCODES = ("MOVJ", "MOVL", "MOVC", "MOVS", "ARCON", "ARCOF", "WVON", "MACRO1")
WORK_MOVE_OPCODES = frozenset({"MOVL", "MOVC"})
STATION_IDS = ("ST1", "ST2", "ST3", "ST4")
_C_INDEX_RE = re.compile(r"\bC(\d+)\b", re.IGNORECASE)

DEFAULT_FIXTURES = ROOT / "backend" / "tests" / "fixtures" / "robot-cell"


def pulses_to_degrees(pulses: list[int]) -> list[float]:
    """S,L,U,R,B,T pulses → degrees (``pulse_config.joint_pulses_to_deg``)."""
    return joint_pulses_to_deg(pulses)


def degrees_to_pulses(q_deg: list[float]) -> list[int]:
    """S,L,U,R,B,T degrees → INFORM pulse integers."""
    return deg_to_joint_pulses(q_deg)


def tcp_from_joints(q_deg: list[float]) -> dict:
    """Torch TCP = URDF flange * TOOL 0."""
    return forward_kinematics(q_deg, frame="tcp")


def flange_from_joints(q_deg: list[float]) -> dict:
    """URDF tool0 (mechanical flange)."""
    return forward_kinematics(q_deg, frame="flange")


@dataclass
class Instruction:
    opcode: str
    text: str


@dataclass
class JbiPoint:
    index: int
    c: list[int] | None = None
    bc: int | None = None
    ec: list[int] | None = None


@dataclass
class JbiJob:
    source: str
    name: str = ""
    npos: str | None = None
    tool: int | None = None
    postype: str | None = None
    group1: str | None = None
    group2: str | None = None
    points: dict[int, JbiPoint] = field(default_factory=dict)
    instructions: list[Instruction] = field(default_factory=list)

    def point(self, index: int) -> JbiPoint:
        pt = self.points.get(index)
        if pt is None:
            pt = JbiPoint(index=index)
            self.points[index] = pt
        return pt


@dataclass
class PointReport:
    index: int
    pulses: list[int]
    degrees: list[float]
    tcp: dict
    ik_degrees: list[float] | None
    ik_pulses: list[int] | None
    delta_pulses: list[int] | None
    delta_degrees: list[float] | None
    ik_ok: bool | None
    tcp_err_mm: float | None
    bc: int | None
    ec: list[int] | None
    ec_deg0: float | None
    softlimit_axes: list[str]


def read_text(path: Path) -> str:
    raw = path.read_bytes()
    for enc in ("utf-8", "cp1250", "latin-1"):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    return raw.decode("latin-1", errors="replace")


def _parse_ints(body: str) -> list[int] | None:
    parts = [part.strip() for part in body.split(",") if part.strip()]
    try:
        return [int(part) for part in parts]
    except ValueError:
        return None


def parse_jbi(text: str, source: str = "") -> JbiJob:
    """Parse INFORM headers, C#/BC#/EC# and the requested instruction opcodes."""
    job = JbiJob(source=source)
    pos_mode: str | None = None
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("'"):
            continue
        if line.startswith("//NAME "):
            job.name = line.split(None, 1)[1].strip()
            continue
        if line.startswith("///NPOS "):
            job.npos = line.split(None, 1)[1].strip()
            continue
        if line.startswith("///TOOL "):
            job.tool = int(line.split()[1])
            continue
        if line.startswith("///POSTYPE "):
            # Joint C# follows the first POSTYPE. A later PULSE block may only
            # hold EC# (see DROP01.JBI: RECTAN C# then PULSE station data).
            pos_mode = line.split()[1].strip()
            if job.postype is None:
                job.postype = pos_mode
            continue
        if line.startswith("///GROUP1 "):
            job.group1 = line.split(None, 1)[1].strip()
            continue
        if line.startswith("///GROUP2 "):
            job.group2 = line.split(None, 1)[1].strip()
            continue
        if len(line) > 1 and line[0] == "C" and line[1].isdigit() and "=" in line:
            idx_s, body = line[1:].split("=", 1)
            pulses = _parse_ints(body)
            if pulses is not None and (pos_mode or "").upper() == "PULSE":
                job.point(int(idx_s)).c = pulses
            continue
        if line.startswith("BC") and "=" in line:
            idx_s, body = line[2:].split("=", 1)
            if idx_s.isdigit():
                job.point(int(idx_s)).bc = int(body.strip())
            continue
        if line.startswith("EC") and "=" in line:
            idx_s, body = line[2:].split("=", 1)
            if idx_s.isdigit():
                ec = _parse_ints(body)
                if ec is not None:
                    job.point(int(idx_s)).ec = ec
            continue
        opcode = line.split(None, 1)[0].upper()
        if opcode in OPCODES:
            job.instructions.append(Instruction(opcode=opcode, text=line))
    return job


def parse_jbi_file(path: Path) -> JbiJob:
    return parse_jbi(read_text(path), source=str(path))


def iter_jbi_paths(path: Path) -> list[Path]:
    if path.is_dir():
        files = sorted(path.glob("*.JBI")) + sorted(path.glob("*.jbi"))
        # Glob is case-sensitive on Linux; keep a stable unique list.
        seen: set[Path] = set()
        out: list[Path] = []
        for p in files:
            key = p.resolve()
            if key not in seen:
                seen.add(key)
                out.append(p)
        return out
    return [path]


def softlimit_axes(pulses: list[int]) -> list[str]:
    flags: list[str] = []
    for i, pulse in enumerate(pulses):
        if pulse > RC1G_SOFTLIMIT_POS[i] or pulse < RC1G_SOFTLIMIT_NEG[i]:
            flags.append(AXIS[i])
    return flags


def ec_axis1_deg(ec: list[int] | None) -> float | None:
    if not ec:
        return None
    return ec[0] / CELL_PULSE.station_pulses_per_degree


def compare_job(job: JbiJob, *, do_ik: bool = True) -> list[PointReport]:
    """Pulse → degrees → TCP (TOOL 0) → optional IK → pulses."""
    if job.postype and job.postype.upper() != "PULSE":
        raise ValueError(
            f"{job.source or job.name}: POSTYPE is {job.postype}, not PULSE. "
            "C# values are not joint pulses."
        )
    ordered = [job.points[i] for i in sorted(job.points)]
    home = job.points.get(0)
    home_seed = pulses_to_degrees(home.c) if home and home.c and len(home.c) == 6 else None
    seed: list[float] | None = home_seed
    rows: list[PointReport] = []

    for pt in ordered:
        if not pt.c or len(pt.c) != 6:
            continue
        q = pulses_to_degrees(pt.c)
        tcp = tcp_from_joints(q)
        ik_deg: list[float] | None = None
        ik_pulses: list[int] | None = None
        d_pulses: list[int] | None = None
        d_deg: list[float] | None = None
        ik_ok: bool | None = None
        tcp_err_mm: float | None = None
        if do_ik:
            used_seed = seed if seed is not None else q
            solved = solve_ik_path([tcp], q_seed_deg=used_seed, frame="tcp")
            ik_ok = bool(solved["diagnostics"][0]["success"])
            ik_deg = [float(a) for a in solved["joint_angles_deg"][0]]
            ik_pulses = degrees_to_pulses(ik_deg)
            d_pulses = [b - a for a, b in zip(pt.c, ik_pulses)]
            d_deg = [b - a for a, b in zip(q, ik_deg)]
            tcp_back = tcp_from_joints(ik_deg)
            tcp_err_mm = math.dist(
                (tcp_back["x"] * 1000.0, tcp_back["y"] * 1000.0, tcp_back["z"] * 1000.0),
                (tcp["x"] * 1000.0, tcp["y"] * 1000.0, tcp["z"] * 1000.0),
            )
            # Stay on the taught branch. A "successful" IK in another wrist
            # configuration must not become the seed for the next point.
            seed = q
        rows.append(
            PointReport(
                index=pt.index,
                pulses=list(pt.c),
                degrees=q,
                tcp=tcp,
                ik_degrees=ik_deg,
                ik_pulses=ik_pulses,
                delta_pulses=d_pulses,
                delta_degrees=d_deg,
                ik_ok=ik_ok,
                tcp_err_mm=tcp_err_mm,
                bc=pt.bc,
                ec=list(pt.ec) if pt.ec else None,
                ec_deg0=ec_axis1_deg(pt.ec),
                softlimit_axes=softlimit_axes(pt.c),
            )
        )
    return rows


def _fmt_nums(values: list[float] | list[int], decimals: int = 3) -> str:
    out: list[str] = []
    for v in values:
        if isinstance(v, float):
            out.append(f"{v:10.{decimals}f}")
        else:
            out.append(f"{int(v):10d}")
    return " ".join(out)


def _max_abs(values: list[float] | list[int] | None) -> float | None:
    if not values:
        return None
    return max(abs(float(v)) for v in values)


def normalize_group2(group2: str | None) -> str | None:
    if not group2:
        return None
    key = group2.upper().replace(" ", "")
    # GROUP2 may be "ST1" or "ST1,BS1"-like; take the STn token.
    for tok in key.replace(",", " ").split():
        if tok in STATION_IDS:
            return tok
    return key if key in STATION_IDS else None


def work_point_indices(job: JbiJob) -> set[int]:
    """C# indices used as weld/work moves (MOVL / MOVC, or any move inside ARCON)."""
    out: set[int] = set()
    in_arc = False
    for inst in job.instructions:
        match = _C_INDEX_RE.search(inst.text)
        idx = int(match.group(1)) if match else None
        if inst.opcode == "ARCON":
            in_arc = True
            continue
        if inst.opcode == "ARCOF":
            in_arc = False
            continue
        if idx is None:
            continue
        if inst.opcode in WORK_MOVE_OPCODES or (in_arc and inst.opcode.startswith("MOV")):
            out.add(idx)
    return out


@dataclass
class WorkSample:
    station: str
    job_name: str
    source: str
    index: int
    bc: int
    rail_mm: float
    tcp_base_m: tuple[float, float, float]
    world_m: tuple[float, float, float]


@dataclass
class StationStats:
    station: str
    n_points: int = 0
    jobs: list[str] = field(default_factory=list)
    bc_mean: float | None = None
    bc_min: int | None = None
    bc_max: int | None = None
    rail_mm_mean: float | None = None
    center_base_m: tuple[float, float, float] | None = None
    bbox_min_m: tuple[float, float, float] | None = None
    bbox_max_m: tuple[float, float, float] | None = None
    center_world_m: tuple[float, float, float] | None = None
    bbox_world_min_m: tuple[float, float, float] | None = None
    bbox_world_max_m: tuple[float, float, float] | None = None
    samples: list[WorkSample] = field(default_factory=list)


def tcp_base_to_world(
    tcp_m: tuple[float, float, float],
    rail_mm: float,
) -> tuple[float, float, float]:
    """Robot-base TCP (Z-up) + RECT-X travel → cell world (rail along Z).

    World X/Y follow the robot base; world Z shifts with the carriage so
    ``mm = 0`` at ``BC_HOME`` sits at ``Z = 0``, and positive track mm (toward
    ST1 / park) maps to negative world Z — same sign as ``track_mm_from_world_z``.
    """
    x, y, z = tcp_m
    return (x, y, z - rail_mm / 1000.0)


def collect_work_samples(job: JbiJob) -> list[WorkSample]:
    station = normalize_group2(job.group2)
    if station is None:
        return []
    if job.postype and job.postype.upper() != "PULSE":
        return []
    samples: list[WorkSample] = []
    job_name = job.name or Path(job.source).stem
    for idx in sorted(work_point_indices(job)):
        pt = job.points.get(idx)
        if pt is None or not pt.c or len(pt.c) != 6 or pt.bc is None:
            continue
        tcp = tcp_from_joints(pulses_to_degrees(pt.c))
        base = (float(tcp["x"]), float(tcp["y"]), float(tcp["z"]))
        rail_mm = track_pulses_to_mm(pt.bc)
        samples.append(
            WorkSample(
                station=station,
                job_name=job_name,
                source=job.source,
                index=idx,
                bc=int(pt.bc),
                rail_mm=rail_mm,
                tcp_base_m=base,
                world_m=tcp_base_to_world(base, rail_mm),
            )
        )
    return samples


def _mean3(points: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    n = len(points) or 1
    return (
        sum(p[0] for p in points) / n,
        sum(p[1] for p in points) / n,
        sum(p[2] for p in points) / n,
    )


def _bbox3(
    points: list[tuple[float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def summarize_station(station: str, samples: list[WorkSample]) -> StationStats:
    stats = StationStats(station=station, samples=list(samples))
    if not samples:
        # Fall back to taught envelope mid so empty stations still get a BC anchor.
        mid = STATION_BC_MID.get(station)
        if mid is not None:
            stats.bc_mean = float(mid)
            stats.bc_min = mid
            stats.bc_max = mid
            stats.rail_mm_mean = track_pulses_to_mm(mid)
        return stats
    stats.n_points = len(samples)
    stats.jobs = sorted({s.job_name for s in samples})
    bcs = [s.bc for s in samples]
    stats.bc_mean = sum(bcs) / len(bcs)
    stats.bc_min = min(bcs)
    stats.bc_max = max(bcs)
    stats.rail_mm_mean = sum(s.rail_mm for s in samples) / len(samples)
    bases = [s.tcp_base_m for s in samples]
    worlds = [s.world_m for s in samples]
    stats.center_base_m = _mean3(bases)
    stats.bbox_min_m, stats.bbox_max_m = _bbox3(bases)
    stats.center_world_m = _mean3(worlds)
    stats.bbox_world_min_m, stats.bbox_world_max_m = _bbox3(worlds)
    return stats


def analyze_stations(paths: list[Path]) -> dict[str, StationStats]:
    """Aggregate work-point clouds per GROUP2 station from the given .JBI files."""
    by_station: dict[str, list[WorkSample]] = defaultdict(list)
    for path in paths:
        job = parse_jbi_file(path)
        for sample in collect_work_samples(job):
            by_station[sample.station].append(sample)
    return {st: summarize_station(st, by_station.get(st, [])) for st in STATION_IDS}


def format_station_analysis(stats_map: dict[str, StationStats]) -> str:
    lines = [
        "== Station workspaces (ARCON / MOVL / MOVC TCP clouds)",
        "   TCP base = FK(MA2010 URDF)×TOOL0 in the robot-base frame (metres).",
        "   Rail mm  = track_pulses_to_mm(BC) from BC_HOME; world Z = base Z − rail_m.",
        "",
    ]
    for st in STATION_IDS:
        stats = stats_map[st]
        lines.append(f"-- {st}")
        if stats.n_points == 0:
            mid = STATION_BC_MID.get(st)
            rail = track_pulses_to_mm(mid) if mid is not None else None
            lines.append(
                f"   no work points in scanned jobs; "
                f"envelope BC mid={mid}  rail_mm={rail:.1f}" if mid is not None and rail is not None
                else "   no work points in scanned jobs"
            )
            lines.append("")
            continue
        c = stats.center_base_m
        mn = stats.bbox_min_m
        mx = stats.bbox_max_m
        cw = stats.center_world_m
        assert c and mn and mx and cw and stats.bc_mean is not None
        lines.append(f"   jobs ({len(stats.jobs)}): {', '.join(stats.jobs)}")
        lines.append(f"   work points: {stats.n_points}")
        lines.append(
            f"   BC mean/min/max: {stats.bc_mean:.0f} / {stats.bc_min} / {stats.bc_max}  "
            f"rail_mm_mean={stats.rail_mm_mean:.1f}"
        )
        lines.append(
            f"   centre base m:  ({c[0]: .4f}, {c[1]: .4f}, {c[2]: .4f})"
        )
        lines.append(
            f"   bbox base m:    "
            f"[{mn[0]: .4f}…{mx[0]: .4f}]  "
            f"[{mn[1]: .4f}…{mx[1]: .4f}]  "
            f"[{mn[2]: .4f}…{mx[2]: .4f}]"
        )
        lines.append(
            f"   centre world m: ({cw[0]: .4f}, {cw[1]: .4f}, {cw[2]: .4f})  "
            f"(rail Z shift included)"
        )
        lines.append("")
    return "\n".join(lines)


def format_job(job: JbiJob, rows: list[PointReport]) -> str:
    counts: dict[str, int] = {op: 0 for op in OPCODES}
    for inst in job.instructions:
        counts[inst.opcode] = counts.get(inst.opcode, 0) + 1
    inst_txt = " ".join(f"{op}={n}" for op, n in counts.items() if n)
    lines = [
        f"== {job.name or Path(job.source).name}  file={job.source}",
        (
            f"   NPOS={job.npos or '-'}  TOOL={job.tool if job.tool is not None else '-'}  "
            f"POSTYPE={job.postype or '-'}  GROUP1={job.group1 or '-'}  GROUP2={job.group2 or '-'}"
        ),
        f"   instructions: {inst_txt or '(none)'}",
        "   TCP = FK(MA2010 URDF) * TOOL0.  ΔPULSE/ΔDEG = IK round-trip on that model, not pendant error.",
        "   EC axis1 deg uses RBCALIB 91771/90.  EC axis2 has no certified scale.  BC is pulses (mm unknown).",
        "",
        (
            f"{'idx':<8} {'BC':>8} {'EC':>16} {'ECdeg':>8}  "
            f"{'S':>8} {'L':>8} {'U':>8} {'R':>8} {'B':>8} {'T':>8}  "
            f"{'Xmm':>8} {'Ymm':>8} {'Zmm':>8} {'Rx':>7} {'Ry':>7} {'Rz':>7}  "
            f"{'dPmax':>7} {'dDmax':>7} {'TCPmm':>7} {'IK':>4} {'lim':>4}"
        ),
    ]
    worst_p = 0.0
    worst_d = 0.0
    ik_fail = 0
    limit_n = 0
    for row in rows:
        ec = row.ec or []
        ec_txt = ",".join(str(v) for v in ec) if ec else "-"
        ec_deg = f"{row.ec_deg0:.3f}" if row.ec_deg0 is not None else "-"
        deg = row.degrees
        tcp = row.tcp
        d_p = _max_abs(row.delta_pulses)
        d_d = _max_abs(row.delta_degrees)
        if d_p is not None:
            worst_p = max(worst_p, d_p)
        if d_d is not None:
            worst_d = max(worst_d, d_d)
        if row.ik_ok is False:
            ik_fail += 1
        if row.softlimit_axes:
            limit_n += 1
        ik_txt = "-" if row.ik_ok is None else ("ok" if row.ik_ok else "FAIL")
        lim_txt = "ok" if not row.softlimit_axes else ",".join(row.softlimit_axes)
        dp_txt = f"{d_p:7.0f}" if d_p is not None else f"{'-':>7}"
        dd_txt = f"{d_d:7.3f}" if d_d is not None else f"{'-':>7}"
        err_txt = f"{row.tcp_err_mm:7.2f}" if row.tcp_err_mm is not None else f"{'-':>7}"
        bc_txt = f"{row.bc:8d}" if row.bc is not None else f"{'-':>8}"
        lines.append(
            f"C{row.index:05d} {bc_txt} {ec_txt:>16} {ec_deg:>8}  "
            f"{deg[0]:8.3f} {deg[1]:8.3f} {deg[2]:8.3f} {deg[3]:8.3f} {deg[4]:8.3f} {deg[5]:8.3f}  "
            f"{tcp['x']*1000:8.1f} {tcp['y']*1000:8.1f} {tcp['z']*1000:8.1f} "
            f"{tcp['rx']:7.2f} {tcp['ry']:7.2f} {tcp['rz']:7.2f}  "
            f"{dp_txt} {dd_txt} {err_txt} {ik_txt:>4} {lim_txt:>4}"
        )
        lines.append(f"         C pulse {_fmt_nums(row.pulses, 0)}")
        if row.delta_pulses is not None and row.delta_degrees is not None:
            lines.append(f"         Δpulse  {_fmt_nums(row.delta_pulses, 0)}")
            lines.append(f"         Δdeg    {_fmt_nums(row.delta_degrees, 3)}")
    lines.append("")
    lines.append(
        f"   summary  points={len(rows)}  max|Δpulse|={worst_p:.0f}  "
        f"max|Δdeg|={worst_d:.3f}  ik_fail={ik_fail}  outside_softlimit={limit_n}"
    )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Compare INFORM C# pulses with FK/IK round-trip.")
    parser.add_argument(
        "path",
        type=Path,
        nargs="?",
        default=None,
        help="A .JBI file or a directory of .JBI files "
        f"(default with --analyze-stations: {DEFAULT_FIXTURES})",
    )
    parser.add_argument(
        "--no-ik",
        action="store_true",
        help="Skip IK round-trip (pulses, degrees, TCP and softlimits only)",
    )
    parser.add_argument(
        "--analyze-stations",
        action="store_true",
        help="Aggregate ARCON/MOVL/MOVC TCP clouds per GROUP2 ST1..ST4 "
        "(bounding box, centre, mean BC / rail mm)",
    )
    args = parser.parse_args(argv)
    path = args.path
    if path is None:
        if not args.analyze_stations:
            parser.error("path is required unless --analyze-stations is set")
        path = DEFAULT_FIXTURES
    paths = iter_jbi_paths(path)
    if not paths:
        print(f"No .JBI files under {path}", file=sys.stderr)
        return 2
    missing = [p for p in paths if not p.is_file()]
    if missing:
        print(f"Not a file: {missing[0]}", file=sys.stderr)
        return 2

    if args.analyze_stations:
        print(format_station_analysis(analyze_stations(paths)))
        return 0

    for i, jbi_path in enumerate(paths):
        job = parse_jbi_file(jbi_path)
        try:
            rows = compare_job(job, do_ik=not args.no_ik)
        except ValueError as exc:
            print(f"== skip {jbi_path}: {exc}")
            continue
        if i:
            print()
        print(format_job(job, rows))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
