#!/usr/bin/env python3
"""Inventory a local Verbotics Weld 2026 install (Windows).

This cloud agent cannot see ``C:\\Program Files\\...``. Run this script ON the
machine where Verbotics is installed, then commit or paste the JSON it writes
so a cloud agent can use the real cell/torch TCP.

  python tools/verbotics_local_inventory.py
  python tools/verbotics_local_inventory.py --extract-json

.vbmodel / .vbpreset are zip archives (URDF + model.json + meshes).
"""

from __future__ import annotations

import argparse
import json
import os
import zipfile
from pathlib import Path
from typing import Any, Iterable

CAD_EXT = {".step", ".stp", ".stl", ".dae", ".obj", ".glb", ".gltf", ".iges", ".igs"}
CELL_EXT = {".vbmodel", ".vbpreset"}
CONFIG_NAMES = {"model.json", "model.urdf", "model.srdf", "preset.json"}
TCP_KEYS = {
    "tcp",
    "tcplink",
    "tooltcp",
    "outputtcp",
    "flange",
    "nozzle",
    "nozzlelink",
    "toolpointingin",
    "tipdistance",
    "ctwd",
}


def default_roots() -> list[Path]:
    roots: list[Path] = []
    pf = os.environ.get("ProgramFiles", r"C:\Program Files")
    pf86 = os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")
    local = os.environ.get("LOCALAPPDATA", "")
    roaming = os.environ.get("APPDATA", "")
    home = Path.home()
    for base in (pf, pf86):
        roots.append(Path(base) / "Verbotics Weld 2026")
        roots.append(Path(base) / "Verbotics Weld")
        roots.append(Path(base) / "Verbotics")
    for base in filter(None, (local, roaming)):
        p = Path(base)
        roots.extend(
            [
                p / "Verbotics",
                p / "Verbotics Weld",
                p / "VerboticsWeld",
            ]
        )
    roots.extend(
        [
            home / "Documents" / "Verbotics",
            home / "Documents" / "Verbotics Weld",
        ]
    )
    # de-dupe while preserving order
    seen: set[str] = set()
    out: list[Path] = []
    for r in roots:
        key = str(r).lower()
        if key in seen:
            continue
        seen.add(key)
        out.append(r)
    return out


def iter_files(root: Path) -> Iterable[Path]:
    if not root.exists():
        return
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d.lower() not in {".git", "node_modules"}]
        for name in filenames:
            yield Path(dirpath) / name


def looks_like_zip(path: Path) -> bool:
    try:
        return zipfile.is_zipfile(path)
    except OSError:
        return False


def walk_json_tcp(obj: Any, prefix: str = "") -> list[dict[str, Any]]:
    hits: list[dict[str, Any]] = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            key = str(k)
            path = f"{prefix}.{key}" if prefix else key
            if key.replace("_", "").replace("-", "").lower() in TCP_KEYS:
                hits.append({"path": path, "value": v})
            hits.extend(walk_json_tcp(v, path))
    elif isinstance(obj, list):
        for i, v in enumerate(obj[:80]):
            hits.extend(walk_json_tcp(v, f"{prefix}[{i}]"))
    return hits


def summarize_archive(path: Path, extract_json_to: Path | None) -> dict[str, Any]:
    info: dict[str, Any] = {"kind": "archive", "entries": [], "tcpHints": []}
    try:
        with zipfile.ZipFile(path) as zf:
            names = zf.namelist()
            info["entries"] = names[:400]
            info["entryCount"] = len(names)
            json_members = [n for n in names if n.lower().endswith(".json")]
            urdf_members = [n for n in names if "urdf" in n.lower()]
            mesh_members = [
                n
                for n in names
                if Path(n).suffix.lower() in CAD_EXT
            ]
            info["jsonFiles"] = json_members[:80]
            info["urdfFiles"] = urdf_members[:40]
            info["meshFiles"] = mesh_members[:80]
            info["meshCount"] = len(mesh_members)
            for member in json_members[:20]:
                try:
                    raw = zf.read(member)
                    data = json.loads(raw.decode("utf-8", errors="replace"))
                except (ValueError, UnicodeError, KeyError, json.JSONDecodeError):
                    continue
                info["tcpHints"].extend(
                    {"file": member, **h} for h in walk_json_tcp(data)
                )
                if extract_json_to is not None:
                    dest_dir = extract_json_to / path.stem
                    dest_dir.mkdir(parents=True, exist_ok=True)
                    safe = Path(member).name
                    (dest_dir / safe).write_bytes(raw)
    except (zipfile.BadZipFile, OSError) as exc:
        info["error"] = str(exc)
    return info


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--root",
        action="append",
        default=[],
        help="Extra root to scan (repeatable). Defaults include Verbotics Weld 2026.",
    )
    ap.add_argument(
        "--out",
        default=str(
            Path(__file__).resolve().parents[1]
            / "frontend"
            / "public"
            / "models"
            / "from-verbotics"
            / "inventory.json"
        ),
    )
    ap.add_argument(
        "--extract-json",
        action="store_true",
        help="Copy model.json (and other JSON) out of .vbmodel/.vbpreset archives.",
    )
    args = ap.parse_args()

    roots = [Path(r) for r in args.root] + default_roots()
    existing = [r for r in roots if r.exists()]
    missing = [str(r) for r in roots if not r.exists()]

    cells: list[dict[str, Any]] = []
    cad: list[dict[str, Any]] = []
    configs: list[dict[str, Any]] = []

    out_path = Path(args.out)
    extract_to = out_path.parent / "extracted-json" if args.extract_json else None

    seen: set[str] = set()
    for root in existing:
        for path in iter_files(root):
            key = str(path).lower()
            if key in seen:
                continue
            seen.add(key)
            suf = path.suffix.lower()
            rec: dict[str, Any] = {
                "path": str(path),
                "size": path.stat().st_size,
                "root": str(root),
            }
            if suf in CELL_EXT or (suf == ".zip" and "verbotic" in key):
                rec.update(summarize_archive(path, extract_to))
                cells.append(rec)
            elif path.name.lower() in CONFIG_NAMES:
                rec["kind"] = "config"
                if path.suffix.lower() == ".json":
                    try:
                        data = json.loads(path.read_text(encoding="utf-8", errors="replace"))
                        rec["tcpHints"] = walk_json_tcp(data)
                    except json.JSONDecodeError:
                        rec["tcpHints"] = []
                configs.append(rec)
            elif suf in CAD_EXT:
                rec["kind"] = "cad"
                cad.append(rec)

    report = {
        "ok": bool(existing),
        "scannedRoots": [str(r) for r in existing],
        "missingRoots": missing,
        "counts": {
            "cellsOrPresets": len(cells),
            "cadFiles": len(cad),
            "configFiles": len(configs),
        },
        "cellsOrPresets": cells,
        "configFiles": configs[:200],
        "cadFiles": cad[:400],
        "note": (
            "Cloud agents cannot read C:\\. Run this on the Verbotics PC and "
            "commit inventory.json (plus extracted-json/) into the repo."
        ),
    }

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"wrote {out_path}")
    print(json.dumps(report["counts"], indent=2))
    if not existing:
        print("No Verbotics directories found. Pass --root to the install folder.")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
