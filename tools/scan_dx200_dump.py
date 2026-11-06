#!/usr/bin/env python3
"""Refresh a lightweight DX200 dump scan JSON (M5.5).

Walks a CF text dump directory, indexes ``*.JBI`` headers / first instructions
and key ``*.DAT`` / ``*.CND`` presence, then writes ``docs/_dump_scan.json``
(merge-friendly stub: keeps existing keys when only a partial dump is present).

Usage
-----
  python3 tools/scan_dx200_dump.py [dump_dir]

Default dump_dir: ``reference/dx200-backup-20260915``
"""

from __future__ import annotations

import json
import re
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DUMP = ROOT / "reference" / "dx200-backup-20260915"
OUT = ROOT / "docs" / "_dump_scan.json"

NPOS_RE = re.compile(r"///NPOS\s+([0-9,]+)", re.I)
NAME_RE = re.compile(r"^//NAME\s+(\S+)", re.M | re.I)
GROUP_RE = re.compile(r"///GROUP[12]\s+(.+)", re.I)
TOOL_RE = re.compile(r"///TOOL\s+(\d+)", re.I)
INST_START = re.compile(r"^//INST\s*$", re.M | re.I)


def _first_inst(text: str, limit: int = 12) -> list[str]:
    m = INST_START.search(text)
    if not m:
        return []
    body = text[m.end() :]
    out: list[str] = []
    for line in body.splitlines():
        s = line.strip()
        if not s or s.startswith("/") or s.startswith("'"):
            continue
        if s in ("NOP",):
            continue
        if s == "END":
            break
        out.append(s)
        if len(out) >= limit:
            break
    return out


def scan_jbi(path: Path) -> dict:
    raw = path.read_text(encoding="utf-8", errors="replace")
    name_m = NAME_RE.search(raw)
    npos_m = NPOS_RE.search(raw)
    tool_m = TOOL_RE.search(raw)
    groups = [g.strip() for g in GROUP_RE.findall(raw)]
    flags = {
        "ARCON": "ARCON" in raw,
        "ARCOF": "ARCOF" in raw,
        "WVON": "WVON" in raw,
        "MACRO": "MACRO1" in raw or "MACRO " in raw,
        "CALL": "CALL JOB:" in raw,
        "PULSE": "///POSTYPE PULSE" in raw or "///PULSE" in raw,
        "RECTAN": "///RECTAN" in raw,
        "SRCH": " SRCH " in raw or "\nSRCH " in raw,
    }
    return {
        "file": path.name,
        "bytes": path.stat().st_size,
        "name": name_m.group(1) if name_m else path.stem,
        "npos": npos_m.group(1) if npos_m else None,
        "tool": int(tool_m.group(1)) if tool_m else None,
        "groups": groups,
        "flags": flags,
        "first_inst": _first_inst(raw),
    }


def main() -> int:
    import argparse

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "dump_dir",
        nargs="?",
        default=str(DEFAULT_DUMP),
        help="Path to DX200 CF text dump",
    )
    ap.add_argument(
        "-o",
        "--output",
        default=str(OUT),
        help="Output JSON path (default docs/_dump_scan.json)",
    )
    args = ap.parse_args()
    dump = Path(args.dump_dir)
    if not dump.is_dir():
        raise SystemExit(f"Dump dir not found: {dump}")

    existing: dict = {}
    out_path = Path(args.output)
    if out_path.is_file():
        try:
            existing = json.loads(out_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            existing = {}

    jobs = [scan_jbi(p) for p in sorted(dump.glob("*.JBI"))]
    dat = {
        p.name: {"bytes": p.stat().st_size, "path": str(p.relative_to(dump))}
        for p in sorted(dump.glob("*.DAT"))
    }
    cnd = {
        p.name: {"bytes": p.stat().st_size, "path": str(p.relative_to(dump))}
        for p in sorted(dump.glob("*.CND"))
    }

    payload = {
        **existing,
        "scan_meta": {
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "dump_dir": str(dump),
            "jbi_count": len(jobs),
            "tool": "tools/scan_dx200_dump.py",
        },
        "jobs_scanned_here": jobs,
        "dat_files_here": dat,
        "cnd_files_here": cnd,
    }
    # Preserve legacy top-level "jobs" from full historical scan when present.
    if "jobs" not in payload and jobs:
        payload["jobs"] = jobs

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {out_path} ({len(jobs)} JBI, {len(dat)} DAT, {len(cnd)} CND)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
