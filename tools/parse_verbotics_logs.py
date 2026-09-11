#!/usr/bin/env python3
"""Mine Verbotics Weld 2026 desktop logs into a compact JSON summary.

The cloud agent cannot read C:\\Program Files\\Verbotics Weld 2026. Session
logs still expose the real planner pipeline (workcell UUIDs, CTWD, action
numbers, touch features, home interpolation). Point this at a folder of
``Weld2026_*.log`` files:

    python3 tools/parse_verbotics_logs.py /path/to/logs \\
        -o frontend/public/models/from-verbotics/logs-derived.json
"""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from pathlib import Path

LINE = re.compile(r"^\[(.*?)\]\s+\[.*?\]\s+\[(DBG|INF|WRN|ERR)\]\s+(.*)$")


def parse_logs(logdir: Path) -> dict:
    sessions: list[dict] = []
    worlds: Counter[str] = Counter()
    projects: Counter[str] = Counter()
    generators: list[str] = []
    weld_names: Counter[str] = Counter()
    ctwd: Counter[str] = Counter()
    actions: Counter[str] = Counter()
    toolpath_states: list[int] = []
    seeded: Counter[str] = Counter()
    touch_fail: Counter[str] = Counter()
    api: Counter[str] = Counter()
    identify_cores: Counter[str] = Counter()
    warnings: Counter[str] = Counter()
    profiling_modules: Counter[str] = Counter()
    joint_unwrap: Counter[str] = Counter()
    joint_insert: list[str] = []
    workcell_downloads = 0
    max_attempts: Counter[str] = Counter()

    files = sorted(logdir.glob("*.log")) + sorted(logdir.glob("*.LOG"))
    for path in files:
        lines = path.read_text(errors="replace").splitlines()
        first = last = None
        levels: Counter[str] = Counter()
        for raw in lines:
            m = LINE.match(raw.rstrip())
            if not m:
                continue
            ts, level, msg = m.group(1), m.group(2), m.group(3)
            if first is None:
                first = ts
            last = ts
            levels[level] += 1
            if "Loading world" in msg:
                worlds[msg.split()[-1]] += 1
            if "Loading project from" in msg:
                projects[msg] += 1
            if msg.startswith("Loaded generator"):
                name = msg.replace("Loaded generator ", "").strip("'")
                if name not in generators:
                    generators.append(name)
            if "Planning weld '" in msg:
                named = re.search(r"Planning weld '(Weld [0-9]+-[0-9]+)", msg)
                att = re.search(r"attempt (\d+)/(\d+)", msg)
                if named:
                    weld_names[named.group(1)] += 1
                if att:
                    max_attempts[att.group(2)] += 1
            if "point at" in msg and "mm" in msg:
                stick = re.search(r"/ ([0-9.]+)mm \((welding|not welding)\)", msg)
                if stick:
                    ctwd[f"{stick.group(1)}mm {stick.group(2)}"] += 1
            if "between action" in msg:
                pair = re.search(r"action (\d+) and (\d+)", msg)
                if pair:
                    actions[f"{pair.group(1)}->{pair.group(2)}"] += 1
                warnings["Missing from previous path … going home"] += 1
            if "Created toolpath with" in msg:
                n = re.search(r"(\d+) states", msg)
                if n:
                    toolpath_states.append(int(n.group(1)))
            if msg.startswith("Seeded ") and "toolpath states" in msg:
                seeded[msg] += 1
            if "Touch failed" in msg or "Touch not valid" in msg:
                touch_fail[msg] += 1
            if "Making API request" in msg:
                api[msg] += 1
            if "Identifying welds with" in msg:
                identify_cores[msg] += 1
            if "Downloaded workcell" in msg:
                workcell_downloads += 1
            if "Unwrapped '" in msg and "joint" in msg:
                jn = re.search(r"joint '([^']+)'", msg)
                if jn:
                    joint_unwrap[jn.group(1)] += 1
            if "Inserted" in msg and "intermediate" in msg:
                joint_insert.append(msg)
            if "::" in msg and "s (" in msg and "avg" in msg:
                profiling_modules[msg.split(":")[0].strip()] += 1

        sessions.append(
            {
                "file": path.name,
                "bytes": path.stat().st_size,
                "lines": len(lines),
                "first": first,
                "last": last,
                "levels": dict(levels),
            }
        )

    return {
        "source": "Verbotics Weld 2026 desktop logs",
        "app_version": "2026.1.2",
        "app_rev": "e178f425127c452db06adc90785792417287d7c0",
        "file_version": 95,
        "locale": "pl_PL",
        "generator_default_for_this_cell": "Yaskawa Motoman INFORM",
        "generators_loaded": generators,
        "workcells": [
            {
                "id": wid,
                "seen": count,
            }
            for wid, count in worlds.most_common()
        ],
        "projects": dict(projects),
        "planner": {
            "weld_name_pattern": "Weld {part}-{index}",
            "attempts": dict(max_attempts),
            "seed_states_typical": 5000,
            "seed_from": "reference position (home)",
            "replan": "ToolpathPlanner::replanConfigurationToHome",
            "touch": {
                "approach_states": 3,
                "features": ["plane", "line", "point"],
                "failures": dict(touch_fail),
            },
            "actions": {
                "home": 1,
                "touch": 3,
                "weld": 5,
                "missing_transitions_go_home": dict(actions),
            },
            "motions_planned": [
                "home to touches",
                "touches to home",
                "home to toolpath",
                "toolpath to home",
            ],
            "ctwd_mm": {
                "welding_mode": 12,
                "air_move": 0,
                "observed_histogram": dict(ctwd),
            },
            "joint_unwrap_deg": 360,
            "positioner_joint_name": "positioner_joint",
            "max_joint_change_deg_without_intermediate": 175,
            "unique_weld_names": len(weld_names),
            "example_welds": [n for n, _ in weld_names.most_common(24)],
            "toolpath_states": {
                "n": len(toolpath_states),
                "min": min(toolpath_states) if toolpath_states else None,
                "max": max(toolpath_states) if toolpath_states else None,
                "histogram_top": {
                    str(k): v for k, v in Counter(toolpath_states).most_common(8)
                },
            },
            "seeded": dict(seeded.most_common(8)),
            "profiling_hotspots": [n for n, _ in profiling_modules.most_common(15)],
        },
        "sessions": sessions,
        "workcell_downloads": workcell_downloads,
        "api": dict(api),
        "identify_cores": dict(identify_cores),
        "joint_unwrap": dict(joint_unwrap),
        "joint_insert_examples": list(dict.fromkeys(joint_insert))[:5],
        "warnings": dict(warnings),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logdir", type=Path)
    ap.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("frontend/public/models/from-verbotics/logs-derived.json"),
    )
    args = ap.parse_args()
    if not args.logdir.is_dir():
        raise SystemExit(f"not a directory: {args.logdir}")
    data = parse_logs(args.logdir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.output} ({len(data.get('sessions', []))} sessions)")


if __name__ == "__main__":
    main()
