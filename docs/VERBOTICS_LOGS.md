# Verbotics Weld 2026 — planner evidence from session logs

Eleven desktop logs (`Weld2026_*.log`, March–September 2026) were mined
because this environment cannot read `C:\Program Files\Verbotics Weld 2026`.
Structured output: [`frontend/public/models/from-verbotics/logs-derived.json`](../frontend/public/models/from-verbotics/logs-derived.json).
Re-run with:

```bash
python3 tools/parse_verbotics_logs.py /path/to/logs \
  -o frontend/public/models/from-verbotics/logs-derived.json
```

This is **behavioural evidence**, not a CAD/URDF dump. Torch TCP millimetres,
signed cell meshes and `.vbmodel` contents are still only on the Windows PC.

## Cell / project

| Field | Value |
|-------|--------|
| App | Weld **2026.1.2** (`e178f425…`, file version 95) |
| Locale | `pl_PL` (no translation pack found) |
| Generator used here | **Yaskawa Motoman INFORM** (`yaskawa_inform`) |
| EXPODREW project | `C:/Users/macie/Desktop/EXPODREW/Untitled.vbweld` |
| Workcell A | `856eb580-e7fa-4cf3-b0be-50949f0f0aeb` (Mar–May, reopened 11 Sep) |
| Workcell B | `88d44813-4907-4107-8cbe-a84ebe93d876` (11 Sep autosaves — current) |
| Cell files | signed `model.urdf` + `model.srdf` under `%AppData%/Verbotics/Weld 2026/models/<uuid>/` |
| Positioner joint | `positioner_joint` (unwrapped by ±2π) |

Workcells are downloaded from `https://accounts.verbotics.com/api/workcells`.

## Planner pipeline (from log order + profiling)

Verbotics names the C++ stages in the “All planning tasks finished” dump:

1. **Identify welds** (`Identifying welds with N CPU cores`)
2. **Stage** (`Staging N welds`) — names `Weld {part}-{index}`
3. **Seed** `ToolpathStateSampler::seed` — typically **5000** samples from the **home / reference** configuration
4. **Plan toolpath** (`Invocation k P% along path, … intermediate motions checked`)
5. **Re-plan closer to home** (`ToolpathPlanner::replanConfigurationToHome`) — usually “Unable to re-solve…”
6. **Simplify / optimise nozzle** (`ToolpathSimplifier`, cost often driven to 0)
7. **Convert to robot trajectory** (`Created toolpath with N states`, usually 2)
8. **Approach / retreat** for the weld
9. **Calibration / TouchSense**
    - aligned touch **planes**, then **lines**, then **points**
    - approach for touch with **3 states**
    - frequent `Touch failed - Touch frame not found`
10. **Motions through home only**
    - home → touches, touches → home
    - home → toolpath, toolpath → home
11. **Validate** (`PlanValidator::validate`)
12. **Shortcut** between welds, else **go home**

Collision checking is Bullet (`rm::BulletCollisionChecker`). IK is 6-DoF plus
`PositionerIK`. Between-weld air moves that were never planned emit:

```
Missing from previous path 'Weld 1-2' between action 5 and 3, going home
Missing from previous path 'Weld 1-2' between action 3 and 1, going home
```

So the action ids in a weld are **1 = home**, **3 = touches**, **5 = weld
toolpath**. Touches and the weld do **not** connect directly.

Other limits seen in logs:

- insert an intermediate point if a joint would change by **> 175°**
- unwrap trajectories toward the reference by **±360°**
- retries: **attempt 1/2** (occasionally 1/3)

## Stick-out (CTWD)

`point at i / Nmm (welding|not welding)`:

| Stick-out | Meaning | Count (these logs) |
|-----------|---------|---------------------|
| **0 mm** | not welding (air) | 441 |
| **12 mm** | welding (dominant) | 140 |
| 11.875 mm | welding (same 12 mm grid) | 132 |

The prototype therefore holds the torch tip **12 mm** off the seam while the
arc is on (`CTWD_WELD_MM = 12`).

## What this prototype now mirrors

- Weld names `Weld 1-1`, `Weld 1-2`, …
- Identify → seed-from-home → plan → replan-home → simplify → touch (plane+line, 3-state approach) → home interpolation
- INFORM `.JBI` export (already the Yaskawa generator)
- Planner log lines phrased like the desktop app

Still missing without a local `%ProgramFiles%` dump: exact URDF/SRDF of those
two workcell UUIDs, torch TCP in `model.json`, and official Yaskawa meshes.
