---
name: dx200-cell-dump
description: >-
  Navigate and use the production Yaskawa DX200 CF dump for the MA2010 welding
  cell (PULSE JBI, RC/SC/SV params, Motoweld, TURN/RECT-X). Use when working on
  INFORM .JBI export, pulse↔degree scaling, TOOL/TCP, ARCON/WVON, station EC#,
  base BC#, cell hardware identity, or when the user mentions robot-cell-dx200,
  z robota, CF dump, or DX200 parameters.
---

# DX200 production cell dump

## Before changing JBI / pulse / cell config

1. Read [docs/CELL.md](../../../docs/CELL.md) (identity + mined pulses).
2. Read [docs/CELL_DUMP_CATALOG.md](../../../docs/CELL_DUMP_CATALOG.md) (per-file map).
3. For machine flags/headers: [docs/_dump_scan.json](../../../docs/_dump_scan.json).
4. Code of record: `backend/app/pulse_config.py`, `dx200_postprocessor.py`.

Dump root (local, usually gitignored): `reference/robot-cell-dx200/`.  
Tiny fixtures for tests: `backend/tests/fixtures/robot-cell/`.

## Lookup cheatsheet

| Task | File(s) |
|------|---------|
| Who is R1/B1/S*? | `SYSTEM.SYS` |
| TOOL0 TCP | `TOOL.CND` |
| Welder | `WELDER.DAT` → Motoweld-E 350A |
| Example production job | `jobs/000.JBI` |
| Robot pulses/deg | `params/RC.PRM` RC1G softlimits → already in `pulse_config.py` |
| Station pulses/deg | `RBCALIB.DAT` + RC3G–RC6G |
| Encoder PPR | `params/SV.PRM` SV1G (=4096) |
| Weave `WEV#` | `WEAV.CND` (255) |
| Arc start/end | `cnd/ARCSRT.CND`, `cnd/ARCEND.CND` (1000 each) |
| Station orchestration | `jobs/STACJA_*.JBI` |
| Do not parse | `CMOS.BIN` (24 MB binary) |

## Hard rules for this cell

- Jobs are **`///POSTYPE PULSE`**, not RECTAN (RECTAN only for offline review without IK).
- Groups: **`RB1,BS1` + `ST1`**; moves `C# BC#` + `+MOVJ EC#`.
- **`ARCON` / `ARCOF` without `ASF#`**; conditions via macros + `ARCSRT.CND`.
- Default weave in samples: **`WVON WEV#(21)`**.
- Never commit `CMOS.BIN` / full `params/` / huge CND if policy says so — catalog + fixtures are enough for CI.

## Workflow: change pulse export

1. Confirm constants still match RC1G / RBCALIB (catalog + `pulse_config.py`).
2. Prefer emitting PULSE only when IK joints exist.
3. Diff grammar against `jobs/000.JBI` (NPOS, GROUP, MACRO, WVON).
4. Run `backend` pytest `test_pipeline.py` + `smoke_test.py`.

## Workflow: interpret an unknown dump file

1. Find it in CELL_DUMP_CATALOG (root / jobs / cnd / dat / params).
2. If missing from catalog, open file header (`//` / `///` lines) and extend catalog.
3. Prefer split `RC`/`SC`/`SV` over `ALL.PRM`.

## Additional resources

- Full per-file catalog: [docs/CELL_DUMP_CATALOG.md](../../../docs/CELL_DUMP_CATALOG.md)
- What a complete “cell concept” should also contain: [CONCEPT_GAPS.md](CONCEPT_GAPS.md)
