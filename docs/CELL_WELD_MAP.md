# Cell weld map — Lorch Tiptronic ARGF, ARCSRT, WEAV, TouchSense

Source dump: `reference/dx200-backup-20260915/` (fixtures mirror under
`backend/tests/fixtures/robot-cell/`).
Full reverse-engineering notes: **[BACKUP_AND_LORCH_ANALYSIS.md](BACKUP_AND_LORCH_ANALYSIS.md)**.
Code: `backend/app/welding_config.py`, `frontend/lib/weldProgram.ts`.

## How conditions are selected (not ASF#)

| Layer | File / instruction | Role |
|-------|-------------------|------|
| Macro | `MACRO.DAT` → `//MACRO1 1` | `///JOB1 LORCH`, tag `PROG-NR:` |
| Call | `MACRO1 MJ#(0) ARGFn` | Sets **Lorch Tiptronic program n** |
| Macro body | `LORCH.JBI` | **Missing from CF zip** — DOUT/OG map unknown |
| Handshake | `IONAME.DAT` `IN#51` | `LORCH OK` |
| Arc | `ARCON` / `ARCOF` | Motoman ARC WELDING CIO |
| Arc start file | `ARCSRT.CND` | Shared template `200,20.0,100` |
| Weave | `WVON WEV#(n)` / `WEAV.CND` | Geometry + dwell for file **n** |
| TouchSense | `MOVS` in `123123.JBI` | Search before weld |

Jobs never use `ARCON ASF#(...)` on this cell.

## ARCSRT.CND

All **1000** `//ARCSRT EN` blocks are **byte-identical**: **200 A / 20.0 V / 100 %**.  
`ARGFn` does **not** pick an ARCSRT row — it selects Tiptronic PROG-NR.

`WELDER.DAT` still says Motoweld-E (template + wire↔A curve used only as UI approx.).

## ARGF usage in live jobs

| ARGF (Tiptronic PROG-NR) | Example jobs | Typical `V=` | Typical `WEV#` |
|--------------------------|--------------|--------------|----------------|
| **1** | `12.JBI`, `11.JBI`, `123.JBI` | 4.2–5.0 | (often none) |
| **4** | `1-DEFRO-5.JBI`, DEFRO-70*, kosz | ≈5.0 | 15, 16 |
| **5** | `1.JBI`, `1C.JBI`, kosz | ≈4.2 | 1 |
| **6** | `1B.JBI` | ≈5.0 | — |
| **8** | `000.JBI`, DEFRO-70-NOWE | 5.0–7.5 | **21** |
| **10** | KASUJ | ≈5.0 | 1, 2 |

UI conditions → ARGF **8 / 5 / 4 / 1**.

## WEAV.CND — `WEV#` 1, 15, 16, 21

| WEV# | Mode | Amp L/R (mm) | Angle L/R (°) | Freq | Dwell note |
|------|------|--------------|---------------|------|------------|
| **1** | 0 | 2.0 / 2.0 | 70 / 70 | 2.0 | 0.1 s corners |
| **15** | 1 | 3.0 / 3.0 | 45 / 45 | 3.0 | one side dwell 0.3 s |
| **16** | 1 | 2.0 / 2.0 | 45 / 0 | 2.0 | 0.1 s |
| **21** | 0 | 4.0 / 0.0 | 0 / 0 | 2.0 | default production |

Default export weave: **`WEV#(21)`**.

## TouchSense — `123123.JBI`

```
MOVS C# BC# V=11.0  +MOVJ EC# VJ=0.78
```

## Warning

UI A/V/wire are **not** a qualified WPS. Confirm Tiptronic PROG on the Lorch pendant; validate pulse jobs in MotoSim before CF load.
