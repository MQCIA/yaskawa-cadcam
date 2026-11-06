"""Parse DX200 ``CUBEINTF.CND`` soft cubes (M2.5).

Production dump stores **pulse-space** boxes (not world AABB). Named cubes
``STACJA_1`` / ``STACJA_2`` appear first; remaining slots are empty.

CAD/CAM uses these for coarse soft-interference flags only — they do **not**
replace MotoSim / FSU. Cartesian station AABBs live in ``CELL_STATIONS``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CUBEINTF = (
    ROOT / "reference" / "dx200-backup-20260915" / "CUBEINTF.CND"
)


@dataclass(frozen=True)
class PulseCube:
    """Axis-aligned box in INFORM joint/base pulse space (8 ints per corner)."""

    index: int
    name: str
    hi: tuple[int, ...]  # typically S,L,U,R,B,T + pad
    lo: tuple[int, ...]
    enabled_flags: tuple[int, int, int] = (0, 0, 0)

    @property
    def named(self) -> bool:
        return bool(self.name.strip())


def parse_cubeintf(path: Path | None = None) -> list[PulseCube]:
    """Parse ``CUBEINTF.CND`` into named/unnamed pulse cubes."""
    p = path or DEFAULT_CUBEINTF
    if not p.is_file():
        return []
    lines = p.read_text(encoding="utf-8", errors="replace").splitlines()
    cubes: list[PulseCube] = []
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if not line.startswith("//CUBEINTF"):
            i += 1
            continue
        try:
            idx = int(line.split()[1])
        except (IndexError, ValueError):
            i += 1
            continue
        name = ""
        if i + 1 < len(lines) and lines[i + 1].strip().startswith("///NAME"):
            name = lines[i + 1].strip()[len("///NAME") :].strip()
        flags = (0, 0, 0)
        if i + 2 < len(lines):
            parts = [int(x) for x in lines[i + 2].split(",") if x.strip() != ""]
            if len(parts) >= 3:
                flags = (parts[0], parts[1], parts[2])
        hi = _ints(lines[i + 3]) if i + 3 < len(lines) else ()
        lo = _ints(lines[i + 4]) if i + 4 < len(lines) else ()
        if len(hi) >= 6 and len(lo) >= 6:
            cubes.append(
                PulseCube(
                    index=idx,
                    name=name,
                    hi=tuple(hi[:8]),
                    lo=tuple(lo[:8]),
                    enabled_flags=flags,
                )
            )
        i += 1
    return cubes


def _ints(line: str) -> tuple[int, ...]:
    out: list[int] = []
    for tok in line.strip().split(","):
        tok = tok.strip()
        if not tok:
            continue
        try:
            out.append(int(tok))
        except ValueError:
            break
    return tuple(out)


def cube_contains_pulses(cube: PulseCube, pulses: list[int] | tuple[int, ...]) -> bool:
    """True if the first ``len`` pulse axes lie inside ``[lo, hi]`` per axis (order-safe)."""
    n = min(6, len(pulses), len(cube.hi), len(cube.lo))
    if n < 6:
        return False
    for a in range(n):
        lo = min(cube.lo[a], cube.hi[a])
        hi = max(cube.lo[a], cube.hi[a])
        if not (lo <= int(pulses[a]) <= hi):
            return False
    return True


def path_cube_hits(
    joint_pulses: list[list[int]],
    cubes: list[PulseCube] | None = None,
    *,
    named_only: bool = True,
) -> list[dict]:
    """Return soft-cube hits along a PULSE path (coarse flag for UI / GO_LIVE)."""
    cubes = cubes if cubes is not None else [c for c in parse_cubeintf() if c.named]
    if named_only:
        cubes = [c for c in cubes if c.named]
    hits: list[dict] = []
    for i, q in enumerate(joint_pulses):
        for c in cubes:
            if cube_contains_pulses(c, q):
                hits.append(
                    {
                        "index": i,
                        "cube": c.name or f"CUBEINTF_{c.index}",
                        "cube_index": c.index,
                    }
                )
    return hits


def cubes_public() -> list[dict]:
    out: list[dict] = []
    for c in parse_cubeintf():
        if not c.named and c.hi == (0,) * len(c.hi):
            continue
        out.append(
            {
                "index": c.index,
                "name": c.name,
                "hi_pulses": list(c.hi),
                "lo_pulses": list(c.lo),
                "flags": list(c.enabled_flags),
                "space": "PULSE",
                "note": "Soft interference — not FSU; validate in MotoSim",
            }
        )
    return out
