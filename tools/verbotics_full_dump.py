#!/usr/bin/env python3
"""Full local dump of a Verbotics Weld install (Windows).

Unlike a torch-only scrape, this inventories the whole application surface:
install tree, AppData, workcells, presets, generators, settings, projects,
URDF/meshes, and TCP hints — for a learning CAD/CAM prototype.

Run ON the Verbotics PC (self-hosted worker or local PowerShell):

  python tools/verbotics_full_dump.py
  python tools/verbotics_full_dump.py --copy-torch-meshes

Cloud VMs cannot see C:\\; this must execute on the machine with the install.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import shutil
import socket
import zipfile
from collections import Counter
from pathlib import Path
from typing import Any, Iterable
from xml.etree import ElementTree as ET

CAD_EXT = {".step", ".stp", ".stl", ".dae", ".obj", ".glb", ".gltf", ".iges", ".igs"}
CELL_EXT = {".vbmodel", ".vbpreset", ".vbproject"}
TEXT_EXT = {".json", ".urdf", ".srdf", ".xacro", ".xml", ".yaml", ".yml", ".js", ".md", ".txt", ".csv"}
SKIP_DIR = {
    ".git",
    "node_modules",
    "__pycache__",
    "crashdumps",
    "cache",
    "gpuCache",
    "Code Cache",
    "ShaderCache",
}
TCP_KEYS = {
    "tcp",
    "tcplink",
    "tooltcp",
    "outputtcp",
    "flange",
    "nozzle",
    "nozzlelink",
    "toolpointPosing",
    "toolpointposing",
    "tipdistance",
    "ctwd",
    "tool0",
    "tool_0",
}
TORCH_MESH_MAX = 5 * 1024 * 1024  # per task: copy only small torch meshes


def default_roots() -> list[Path]:
    roots: list[Path] = []
    pf = os.environ.get("ProgramFiles", r"C:\Program Files")
    pf86 = os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")
    local = os.environ.get("LOCALAPPDATA", "")
    roaming = os.environ.get("APPDATA", "")
    home = Path.home()

    for base in (pf, pf86):
        b = Path(base)
        roots.extend(
            [
                b / "Verbotics Weld 2026",
                b / "Verbotics Weld",
                b / "Verbotics",
                b / "Verbotics Cell Editor",
                b / "Verbotics Cell Editor 2026",
            ]
        )
        if b.exists():
            try:
                for child in b.iterdir():
                    if "verbotic" in child.name.lower():
                        roots.append(child)
            except OSError:
                pass

    for base in filter(None, (local, roaming)):
        p = Path(base)
        roots.extend([p / "Verbotics", p / "Verbotics Weld", p / "VerboticsWeld"])
        try:
            for child in p.iterdir():
                if "verbotic" in child.name.lower():
                    roots.append(child)
        except OSError:
            pass

    docs = home / "Documents"
    roots.extend(
        [
            docs / "Verbotics",
            docs / "Verbotics Weld",
            docs / "VerboticsWeld",
            docs / "Verbotics Cell Editor",
        ]
    )
    if docs.exists():
        try:
            for child in docs.iterdir():
                if "verbotic" in child.name.lower():
                    roots.append(child)
        except OSError:
            pass

    # Start Menu shortcuts often point at the real install
    for sm in (
        Path(os.environ.get("ProgramData", r"C:\ProgramData")) / "Microsoft" / "Windows" / "Start Menu" / "Programs",
        Path(os.environ.get("APPDATA", "")) / "Microsoft" / "Windows" / "Start Menu" / "Programs",
    ):
        if not sm.exists():
            continue
        try:
            for child in sm.rglob("*"):
                if "verbotic" in child.name.lower() and child.is_dir():
                    roots.append(child)
        except OSError:
            pass

    seen: set[str] = set()
    out: list[Path] = []
    for r in roots:
        key = str(r).lower()
        if key in seen:
            continue
        seen.add(key)
        out.append(r)
    return out


def sha256_file(path: Path, limit: int = 32 * 1024 * 1024) -> str | None:
    try:
        h = hashlib.sha256()
        size = 0
        with path.open("rb") as f:
            while True:
                chunk = f.read(1024 * 1024)
                if not chunk:
                    break
                size += len(chunk)
                if size > limit:
                    return None
                h.update(chunk)
        return h.hexdigest()
    except OSError:
        return None


def walk_json(obj: Any, prefix: str = "") -> list[dict[str, Any]]:
    hits: list[dict[str, Any]] = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            key = str(k)
            path = f"{prefix}.{key}" if prefix else key
            norm = key.replace("_", "").replace("-", "").lower()
            if norm in TCP_KEYS or "tcp" in norm or ("tool" in norm and "frame" in norm):
                hits.append({"path": path, "value": v})
            hits.extend(walk_json(v, path))
    elif isinstance(obj, list):
        for i, v in enumerate(obj[:120]):
            hits.extend(walk_json(v, f"{prefix}[{i}]"))
    return hits


_XYZ_RE = re.compile(r"xyz\s*=\s*[\"']([^\"']+)[\"']", re.I)
_RPY_RE = re.compile(r"rpy\s*=\s*[\"']([^\"']+)[\"']", re.I)
_NAME_RE = re.compile(r"name\s*=\s*[\"']([^\"']+)[\"']", re.I)


def parse_urdf_tcp_hints(text: str) -> list[dict[str, Any]]:
    """Extract joint/link origins that look like tool/TCP frames from URDF XML."""
    hits: list[dict[str, Any]] = []
    try:
        root = ET.fromstring(text)
    except ET.ParseError:
        # fallback: regex scan for tool/tcp named joints
        for block in re.finditer(r"<(joint|link)\b[^>]*>.*?</\1>", text, re.I | re.S):
            chunk = block.group(0)
            names = _NAME_RE.findall(chunk)
            if not names:
                continue
            name = names[0]
            if not re.search(r"tcp|tool|torch|nozzle|flange|tip", name, re.I):
                continue
            xyz = _XYZ_RE.search(chunk)
            rpy = _RPY_RE.search(chunk)
            hits.append(
                {
                    "path": f"urdf.{block.group(1)}.{name}",
                    "value": {
                        "xyz": xyz.group(1) if xyz else None,
                        "rpy": rpy.group(1) if rpy else None,
                    },
                }
            )
        return hits

    def local(tag: str) -> str:
        if "}" in tag:
            return tag.rsplit("}", 1)[-1]
        return tag

    for elem in root.iter():
        tag = local(elem.tag).lower()
        if tag not in {"joint", "link", "origin"}:
            continue
        name = elem.attrib.get("name", "")
        if tag in {"joint", "link"}:
            if not re.search(r"tcp|tool|torch|nozzle|flange|tip|ee_link|tool0", name, re.I):
                continue
            origin = None
            for child in list(elem):
                if local(child.tag).lower() == "origin":
                    origin = child
                    break
            # also parent joint origin pointing at this link
            hits.append(
                {
                    "path": f"urdf.{tag}.{name}",
                    "value": {
                        "xyz": origin.attrib.get("xyz") if origin is not None else None,
                        "rpy": origin.attrib.get("rpy") if origin is not None else None,
                        "attribs": dict(elem.attrib),
                    },
                }
            )
        elif tag == "origin":
            parent = elem
            # skip bare origins without interesting parents (handled above)
            _ = parent
    return hits


def tree_summary(root: Path, max_depth: int = 3) -> dict[str, Any]:
    folders: dict[str, dict[str, Any]] = {}
    ext_counts: Counter[str] = Counter()
    total_files = 0
    total_bytes = 0
    if not root.exists():
        return {"error": "missing"}

    root_s = str(root)
    for dirpath, dirnames, filenames in os.walk(root):
        rel = os.path.relpath(dirpath, root_s)
        depth = 0 if rel == "." else rel.count(os.sep) + 1
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIR and not d.startswith(".")]
        if depth > max_depth:
            dirnames[:] = []
            continue
        file_count = 0
        byte_count = 0
        for name in filenames:
            p = Path(dirpath) / name
            try:
                sz = p.stat().st_size
            except OSError:
                continue
            file_count += 1
            byte_count += sz
            total_files += 1
            total_bytes += sz
            ext_counts[p.suffix.lower() or "<none>"] += 1
        folders[rel.replace("\\", "/")] = {"files": file_count, "bytes": byte_count}

    return {
        "folders": folders,
        "extensionCounts": dict(ext_counts.most_common(40)),
        "totalFiles": total_files,
        "totalBytes": total_bytes,
    }


def summarize_archive(path: Path, extract_json_to: Path | None) -> dict[str, Any]:
    info: dict[str, Any] = {"kind": "archive", "tcpHints": []}
    try:
        with zipfile.ZipFile(path) as zf:
            names = zf.namelist()
            info["entryCount"] = len(names)
            info["entriesSample"] = names[:200]
            json_members = [n for n in names if n.lower().endswith(".json")]
            urdf_members = [n for n in names if "urdf" in n.lower() or n.lower().endswith(".srdf")]
            mesh_members = [n for n in names if Path(n).suffix.lower() in CAD_EXT]
            js_members = [n for n in names if n.lower().endswith(".js")]
            info["jsonFiles"] = json_members[:100]
            info["urdfFiles"] = urdf_members[:40]
            info["meshFiles"] = mesh_members[:100]
            info["meshCount"] = len(mesh_members)
            info["jsFiles"] = js_members[:40]
            for member in json_members[:30]:
                try:
                    raw = zf.read(member)
                    data = json.loads(raw.decode("utf-8", errors="replace"))
                except (ValueError, UnicodeError, KeyError, json.JSONDecodeError):
                    continue
                info["tcpHints"].extend({"file": member, **h} for h in walk_json(data))
                if extract_json_to is not None:
                    dest_dir = extract_json_to / path.stem
                    dest_dir.mkdir(parents=True, exist_ok=True)
                    (dest_dir / Path(member).name).write_bytes(raw)
            for member in urdf_members[:10]:
                try:
                    raw = zf.read(member)
                except KeyError:
                    continue
                try:
                    text = raw.decode("utf-8", errors="replace")
                    info["tcpHints"].extend({"file": member, **h} for h in parse_urdf_tcp_hints(text))
                except Exception:
                    pass
                if extract_json_to is not None:
                    dest_dir = extract_json_to / path.stem
                    dest_dir.mkdir(parents=True, exist_ok=True)
                    (dest_dir / Path(member).name).write_bytes(raw)
    except (zipfile.BadZipFile, OSError) as exc:
        info["error"] = str(exc)
    return info


def looks_like_torch(path: Path) -> bool:
    n = path.name.lower()
    return any(
        k in n
        for k in (
            "torch",
            "gun",
            "tbi",
            "binzel",
            "abicor",
            "lorch",
            "fronius",
            "skelp",
            "nozzle",
            "welding_tool",
            "weldtool",
        )
    )


def collect_exe_versions(roots: list[Path]) -> list[dict[str, Any]]:
    """Best-effort version metadata; rich FileVersionInfo comes from the PS1 helper."""
    out: list[dict[str, Any]] = []
    for root in roots:
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            if path.suffix.lower() not in {".exe", ".dll"}:
                continue
            name = path.name.lower()
            if not any(k in name for k in ("verbotic", "weld", "cell", "editor")):
                if path.parent == root or path.relative_to(root).parts[0:1] == ("bin",):
                    pass
                else:
                    continue
            rec: dict[str, Any] = {
                "path": str(path),
                "size": path.stat().st_size,
                "mtime": path.stat().st_mtime,
            }
            # companion .version / VERSION files nearby
            for sib in (path.with_suffix(".version"), path.parent / "VERSION", path.parent / "version.txt"):
                if sib.exists() and sib.is_file() and sib.stat().st_size < 64_000:
                    try:
                        rec["versionFile"] = sib.read_text(encoding="utf-8", errors="replace")[:2000]
                    except OSError:
                        pass
            out.append(rec)
            if len(out) >= 80:
                return out
    return out


def write_summary_md(report: dict[str, Any], path: Path) -> None:
    lines = [
        "# Verbotics local dump",
        "",
        f"- OK: **{report.get('ok')}**",
        f"- Host: `{report.get('host', {}).get('hostname')}` ({report.get('host', {}).get('system')})",
        f"- Scanned roots: {len(report.get('scannedRoots', []))}",
        "",
        "## Counts",
        "",
    ]
    for k, v in (report.get("counts") or {}).items():
        lines.append(f"- `{k}`: {v}")

    lines += ["", "## Install / data roots", ""]
    for r in report.get("scannedRoots", []):
        lines.append(f"- `{r}`")

    if report.get("placementError"):
        lines += ["", "## Placement error", "", report["placementError"], ""]

    lines += ["", "## Key TCP / tool hints", ""]
    tcp_shown = 0
    for cell in report.get("cellsOrPresets") or []:
        hints = cell.get("tcpHints") or []
        if not hints:
            continue
        lines.append(f"### `{Path(cell.get('path', '')).name}`")
        lines.append(f"- path: `{cell.get('path')}`")
        for h in hints[:12]:
            lines.append(f"  - `{h.get('path')}` = `{h.get('value')!r}`")
            tcp_shown += 1
        if tcp_shown >= 40:
            break
    if tcp_shown == 0:
        lines.append("_No TCP hints found._")

    lines += ["", "## Workcells / presets (sample)", ""]
    for cell in (report.get("cellsOrPresets") or [])[:40]:
        lines.append(
            f"- `{cell.get('path')}` ({cell.get('size', 0)} bytes, meshes={cell.get('meshCount', '?')})"
        )

    lines += ["", "## Generators", ""]
    for g in (report.get("generators") or [])[:60]:
        lines.append(f"- `{g.get('path')}`")

    lines += ["", "## Versions (sample)", ""]
    for v in (report.get("exeVersions") or [])[:20]:
        lines.append(f"- `{v.get('path')}` size={v.get('size')}")
        if v.get("FileVersion") or v.get("ProductVersion"):
            lines.append(f"  - FileVersion={v.get('FileVersion')} ProductVersion={v.get('ProductVersion')}")
        if v.get("versionFile"):
            lines.append(f"  - versionFile: {v.get('versionFile')!r}")

    lines += ["", "## Notes", "", report.get("note", ""), ""]
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", action="append", default=[])
    ap.add_argument(
        "--out-dir",
        default=str(
            Path(__file__).resolve().parents[1]
            / "frontend"
            / "public"
            / "models"
            / "from-verbotics"
        ),
    )
    ap.add_argument(
        "--copy-torch-meshes",
        action="store_true",
        help=f"Copy small torch-like CAD (<{TORCH_MESH_MAX // (1024*1024)}MB) into meshes/",
    )
    ap.add_argument("--max-tree-depth", type=int, default=3)
    ap.add_argument(
        "--versions-json",
        default="",
        help="Optional JSON from PowerShell FileVersionInfo dump to merge in.",
    )
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    extract_to = out_dir / "extracted-json"
    meshes_to = out_dir / "meshes"
    extract_to.mkdir(exist_ok=True)

    host = {
        "hostname": socket.gethostname(),
        "system": platform.system(),
        "release": platform.release(),
        "machine": platform.machine(),
        "python": platform.python_version(),
        "cwd": str(Path.cwd()),
    }

    roots = [Path(r) for r in args.root] + default_roots()
    existing = [r for r in roots if r.exists()]
    missing = [str(r) for r in roots if not r.exists()]

    placement_error = None
    if platform.system() != "Windows":
        placement_error = (
            f"This dump ran on {host['system']} host '{host['hostname']}', not a Windows "
            "Verbotics PC. C:\\Program Files is unreachable. Re-run on self-hosted worker "
            "verbotics-pc (workerId bfb7e410-b0af-4fb8-8243-15bad1b54d7d) via: "
            "python tools/verbotics_full_dump.py --copy-torch-meshes"
        )

    cells: list[dict[str, Any]] = []
    cad: list[dict[str, Any]] = []
    configs: list[dict[str, Any]] = []
    generators: list[dict[str, Any]] = []
    projects: list[dict[str, Any]] = []
    licenses: list[dict[str, Any]] = []
    trees: dict[str, Any] = {}

    seen: set[str] = set()
    for root in existing:
        trees[str(root)] = tree_summary(root, max_depth=args.max_tree_depth)
        for path in _iter(root):
            key = str(path).lower()
            if key in seen:
                continue
            seen.add(key)
            try:
                size = path.stat().st_size
            except OSError:
                continue
            suf = path.suffix.lower()
            name = path.name.lower()
            rec: dict[str, Any] = {"path": str(path), "size": size, "root": str(root)}

            if name.endswith(".vblicense") or suf == ".vblicense":
                licenses.append(
                    {"path": str(path), "size": size, "note": "presence only; secret not copied"}
                )
                continue
            if suf in CELL_EXT or (suf == ".zip" and "verbotic" in key):
                rec.update(summarize_archive(path, extract_to))
                cells.append(rec)
            elif suf == ".js" and ("generator" in key or "generators" in key or "post" in key):
                generators.append(rec)
                try:
                    dest = extract_to / "generators"
                    dest.mkdir(exist_ok=True)
                    if size < 2_000_000:
                        shutil.copy2(path, dest / path.name)
                except OSError:
                    pass
            elif name in {
                "model.json",
                "model.urdf",
                "model.srdf",
                "preset.json",
                "settings.json",
                "preferences.json",
            }:
                rec["kind"] = "config"
                if suf == ".json":
                    try:
                        data = json.loads(path.read_text(encoding="utf-8", errors="replace"))
                        rec["tcpHints"] = walk_json(data)
                    except json.JSONDecodeError:
                        rec["tcpHints"] = []
                elif suf in {".urdf", ".srdf"}:
                    try:
                        rec["tcpHints"] = parse_urdf_tcp_hints(
                            path.read_text(encoding="utf-8", errors="replace")
                        )
                    except OSError:
                        rec["tcpHints"] = []
                configs.append(rec)
                try:
                    dest = extract_to / "loose-config"
                    dest.mkdir(exist_ok=True)
                    if size < 5_000_000:
                        shutil.copy2(path, dest / f"{path.parent.name}__{path.name}")
                except OSError:
                    pass
            elif suf == ".vbproject" or name.endswith(".vbproj"):
                projects.append(rec)
            elif suf in CAD_EXT:
                rec["kind"] = "cad"
                rec["sha256"] = sha256_file(path)
                cad.append(rec)
                if args.copy_torch_meshes and looks_like_torch(path) and size <= TORCH_MESH_MAX:
                    meshes_to.mkdir(exist_ok=True)
                    try:
                        shutil.copy2(path, meshes_to / path.name)
                        rec["copied"] = True
                    except OSError as exc:
                        rec["copyError"] = str(exc)
            elif suf == ".js":
                generators.append(rec)

    exe_versions = collect_exe_versions(existing)
    if args.versions_json:
        vpath = Path(args.versions_json)
        if vpath.exists():
            try:
                extra = json.loads(vpath.read_text(encoding="utf-8"))
                if isinstance(extra, list):
                    exe_versions = extra + exe_versions
                elif isinstance(extra, dict) and "versions" in extra:
                    exe_versions = list(extra["versions"]) + exe_versions
            except (OSError, json.JSONDecodeError):
                pass

    # Aggregate notable link/tool names from TCP hints
    tcp_index: list[dict[str, Any]] = []
    for cell in cells:
        for h in cell.get("tcpHints") or []:
            tcp_index.append(
                {
                    "cell": cell.get("path"),
                    "file": h.get("file"),
                    "path": h.get("path"),
                    "value": h.get("value"),
                }
            )

    report = {
        "ok": bool(existing) and placement_error is None,
        "host": host,
        "placementError": placement_error,
        "scannedRoots": [str(r) for r in existing],
        "missingRoots": missing,
        "trees": trees,
        "counts": {
            "cellsOrPresets": len(cells),
            "cadFiles": len(cad),
            "configFiles": len(configs),
            "generators": len(generators),
            "projects": len(projects),
            "licenseFilesSeen": len(licenses),
            "tcpHints": len(tcp_index),
        },
        "tcpIndex": tcp_index[:500],
        "cellsOrPresets": cells,
        "configFiles": configs[:300],
        "generators": generators[:200],
        "projects": projects[:100],
        "licenseFilesSeen": licenses,
        "exeVersions": exe_versions[:100],
        "cadFiles": cad[:800],
        "note": (
            "Full application dump for learning/personal use. Binary meshes are listed "
            "(and torch-like ones optionally copied if <5MB). Do not redistribute Verbotics IP. "
            "License secrets (.vblicense) are noted by path only."
        ),
    }

    inv = out_dir / "inventory.json"
    inv.write_text(json.dumps(report, indent=2), encoding="utf-8")
    write_summary_md(report, out_dir / "SUMMARY.md")
    print(f"wrote {inv}")
    print(json.dumps(report["counts"], indent=2))
    if placement_error:
        print(placement_error)
        return 3
    if not existing:
        print("No Verbotics directories found. Pass --root.")
        return 2
    return 0


def _iter(root: Path) -> Iterable[Path]:
    if not root.exists():
        return
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIR and not d.startswith(".")]
        for name in filenames:
            yield Path(dirpath) / name


if __name__ == "__main__":
    raise SystemExit(main())
