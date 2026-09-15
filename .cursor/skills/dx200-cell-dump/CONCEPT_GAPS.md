# Cell concept — what else should be here

This skill + dump catalog cover the **CF text dump**. A complete “production cell concept” for CAD/CAM agents should also include the items below (many still missing or only approximate in-repo).

## Already covered (dump + code)

- [x] Controller / robot / base / station identity (`SYSTEM.SYS`)
- [x] TOOL0 TCP + RPY
- [x] Welder brand (Motoweld-E)
- [x] Real INFORM grammar samples (PULSE JBI)
- [x] Mined robot + station pulse scaling (`pulse_config.py`)
- [x] File-level dump catalog for agents
- [x] Viewport meshes (MA2010 URDF/STL, torch, rail)
- [x] Test fixtures subset (`000.JBI`, SYSTEM, TOOL, WELDER)

## Should also exist in this concept

### Geometry & frames

- [ ] Certified DH / MotoSim robot definition (replace placeholder DH)
- [ ] Measured cell layout: rail origin, station axes, fixture frames (mm)
- [ ] USER frames used in production (`UF#` list + teach jobs → document numbers)
- [ ] RECT-X **pulses/mm** (ballscrew lead / RC2 calibration) — park pulse alone is not enough
- [ ] Station mechanical zero vs INFORM zero (ABSO + teach)

### Calibration & validation

- [ ] Pendant screenshots / CSV of known poses: deg + pulse + Cartesian side-by-side
- [ ] Scripted compare: `jobs/*.JBI` C# → deg → FK vs expected TCP
- [ ] MotoSim EG-VRC project matching this cell (collision models)
- [ ] Golden `.JBI` round-trip test (export → import on controller dry-run)

### Process / welding

- [ ] Mapping `MACRO1 ARGF` / condition numbers → `ARCSRT` EN / `WELDUDEF` USER-n
- [ ] Qualified WPS table (A, V, wire, travel) — UI schedules are examples only
- [ ] Gas / wire / stickout standard for this cell
- [ ] Which `WEV#` are actually used in live production (grep jobs; document)

### IO & safety

- [ ] IO map for curtains/gates (`OTWORZ-BRAMA` / `ZAMKNIJ-KURTYNA` SOUT/SIN numbers)
- [ ] Cube / interference zones named ↔ `CUBEINTF` ↔ station masters
- [ ] Safety logic summary (`YSFLOGIC.DAT`) in plain language

### Ops packaging

- [ ] “Safe subset” of dump always in git (fixtures) vs full dump local-only policy
- [ ] Refresh procedure: how to re-dump CF and re-run catalog scan
- [ ] Version stamp: controller software + dump date in CELL.md (already partially)

### CAD/CAM product

- [ ] Offline → online checklist (MotoSim → CF load)
- [ ] Explicit non-goals (e.g. never auto-load CMOS from CI)
- [ ] Multi-station export: ST1 vs ST2/ST3/ST4 group selection

## Suggested future artifacts

| Artifact | Location idea |
|----------|----------------|
| Pose compare notebook/script | `tools/compare_pulses.py` |
| IO cheat sheet | `docs/CELL_IO.md` |
| UF/TOOL register | `docs/CELL_FRAMES.md` |
| WPS ↔ ARGF map | `docs/CELL_WELD_MAP.md` |
| Dump refresh script | `tools/scan_dx200_dump.py` → regenerates `_dump_scan.json` + catalog stubs |

## Agent behavior when gaps block a task

1. Say which checklist item is missing.
2. Prefer reading dump files over inventing numbers.
3. Do not invent pulses/mm or DH “certification.”
4. Ship RECTAN review + documented EC/BC park rather than fake PULSE C# if IK/DH is untrusted.
