# from-verbotics

Cloud Linux cannot read `C:\Program Files\Verbotics Weld 2026`. What landed
here comes from **session logs** the user attached (Mar 29, May 26, Sep 11 2026).

| File | What it is |
|------|------------|
| `logs-derived.json` | Parsed planner/cell evidence (workcell UUIDs, CTWD, actions, generators) |
| `SUMMARY.md` | This note |

See [`docs/VERBOTICS_LOGS.md`](../../../../docs/VERBOTICS_LOGS.md) for how the
CAD/CAM prototype uses the numbers.

To refresh after another desktop session:

```bash
python3 tools/parse_verbotics_logs.py /path/to/Weld2026-logs
```

A full cell dump (URDF, SRDF, torch TCP, STEP) still needs
`tools/verbotics_local_inventory.ps1` on the Windows PC.
