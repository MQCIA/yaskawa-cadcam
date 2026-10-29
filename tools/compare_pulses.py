#!/usr/bin/env python3
"""Round-trip production INFORM PULSE jobs through the cell scale and DH model.

Usage
-----
    python tools/compare_pulses.py path/to/job.JBI
    python tools/compare_pulses.py path/to/jobs_dir
    python tools/compare_pulses.py job.JBI --no-ik

For every ``C#`` the script:

1. Converts pulses to degrees with ``app.pulse_config.joint_pulses_to_deg``.
2. Runs forward kinematics and applies TOOL 0
   (xyz ``-54.280, 1.052, 415.702`` mm, RPY ``1.3119, -42.6215, -6.8387`` deg)
   so the reported pose is the torch TCP, not the flange.
3. Solves IK for that TCP (tool removed again, so the chain is consistent)
   seeded with the previous solution, or with ``C00000`` when there is no
   previous point.
4. Converts the IK joints back to pulses with ``deg_to_joint_pulses``.

``ΔPULSE`` / ``ΔDEG`` measure numeric IK consistency on the *placeholder* DH
model. They are not the error against the pendant. TCP millimetres will not
match taught XYZ until M1.4 replaces the DH lengths.

``BC#`` stays in pulses (RECT-X mm/pulse is not certified). ``EC#`` is two
components: axis 1 uses the RBCALIB TURN scale (91771 pulse / 90°); axis 2
is printed in pulses only.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

from spatialmath import SE3

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "backend"))

from app.kinematics_engine import _pose_to_se3, forward_kinematics, solve_ik_path  # noqa: E402
from app.pulse_config import (  # noqa: E402
    CELL_PULSE,
    RC1G_SOFTLIMIT_NEG,
    RC1G_SOFTLIMIT_POS,
    deg_to_joint_pulses,
    joint_pulses_to_deg,
)

AXIS = ("S", "L", "U", "R", "B", "T")
OPCODES = ("MOVJ", "MOVL", "MOVC", "MOVS", "ARCON", "ARCOF", "WVON", "MACRO1")

# TOOL.CND / docs/CELL.md — tool frame relative to the flange.
TOOL0_XYZ_MM = (-54.280, 1.052, 415.702)
TOOL0_RPY_DEG = (1.3119, -42.6215, -6.8387)


def pulses_to_degrees(pulses: list[int]) -> list[float]:
    """S,L,U,R,B,T pulses → degrees (``pulse_config.joint_pulses_to_deg``)."""
    return joint_pulses_to_deg(pulses)


def degrees_to_pulses(q_deg: list[float]) -> list[int]:
    """S,L,U,R,B,T degrees → INFORM pulse integers."""
    return deg_to_joint_pulses(q_deg)


def tool0_se3() -> SE3:
    xyz_m = [v / 1000.0 for v in TOOL0_XYZ_MM]
    return SE3(xyz_m[0], xyz_m[1], xyz_m[2]) * SE3.RPY(list(TOOL0_RPY_DEG), unit="deg", order="xyz")


def _se3_to_pose(T: SE3) -> dict:
    rpy = T.rpy(unit="deg", order="xyz")
    t = T.t
    return {
        "x": float(t[0]),
        "y": float(t[1]),
        "z": float(t[2]),
        "rx": float(rpy[0]),
        "ry": float(rpy[1]),
        "rz": float(rpy[2]),
    }


def tcp_from_joints(q_deg: list[float]) -> dict:
    """Flange FK composed with TOOL 0. Translation in metres, RPY in degrees."""
    flange = forward_kinematics(q_deg)
    T_tcp = _pose_to_se3(flange) * tool0_se3()
    return _se3_to_pose(T_tcp)


def flange_pose_for_tcp(tcp: dict) -> dict:
    """Invert TOOL 0 so IK targets the flange that produces this TCP."""
    T_flange = _pose_to_se3(tcp) * tool0_se3().inv()
    return _se3_to_pose(T_flange)


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
            flange = flange_pose_for_tcp(tcp)
            solved = solve_ik_path([flange], q_seed_deg=used_seed)
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
        "   TCP = FK(DH placeholder) * TOOL0.  ΔPULSE/ΔDEG = IK round-trip on that model, not pendant error.",
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
    parser.add_argument("path", type=Path, help="A .JBI file or a directory of .JBI files")
    parser.add_argument(
        "--no-ik",
        action="store_true",
        help="Skip IK round-trip (pulses, degrees, TCP and softlimits only)",
    )
    args = parser.parse_args(argv)
    paths = iter_jbi_paths(args.path)
    if not paths:
        print(f"No .JBI files under {args.path}", file=sys.stderr)
        return 2
    missing = [p for p in paths if not p.is_file()]
    if missing:
        print(f"Not a file: {missing[0]}", file=sys.stderr)
        return 2
    for i, path in enumerate(paths):
        job = parse_jbi_file(path)
        try:
            rows = compare_job(job, do_ik=not args.no_ik)
        except ValueError as exc:
            print(f"== skip {path}: {exc}")
            continue
        if i:
            print()
        print(format_job(job, rows))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
