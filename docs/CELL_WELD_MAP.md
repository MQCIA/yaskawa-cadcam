# Cell weld map — Motoweld ARGF, ARCSRT, WEAV, TouchSense

Source dump: `backend/tests/fixtures/robot-cell/` (mirror of `reference/robot-cell-dx200`).
Code of record: `backend/app/welding_config.py`, `frontend/lib/weldProgram.ts`.

## How conditions are selected (not ASF#)

| Layer | File / instruction | Role |
|-------|-------------------|------|
| Macro | `MACRO.DAT` → `//MACRO1 1` | `///JOB1 LORCH`, tag `PROG-NR:` (legacy name; power source is Motoweld-E) |
| Call | `MACRO1 MJ#(0) ARGFn` | Passes Motoweld **program number n** (`ARGF` = PROG-NR) |
| Arc start file | `ARCSRT.CND` `//ARCSRT EN k` | Shared start template (see below) |
| Arc end file | `ARCEND.CND` | Shared crater/end template (`160,16.0,80`) |
| Weave | `WVON WEV#(n)` / `WEAV.CND` | Weave geometry for file **n** |
| TouchSense | `MOVS` in `123123.JBI` | Search move before weld (no `SRCH` opcode in this dump’s gold jobs) |

Jobs never use `ARCON ASF#(...)` on this cell.

## ARCSRT.CND — important finding

All **1000** `//ARCSRT EN` blocks on this dump are **byte-identical**.  
Teachable arc-start row:

```
200,20.0,100
```

→ **200 A / 20.0 V / 100 %**. Aux/comment lines also show `13.3` (wire-related field).

Therefore **`ARGFn` does not pick a distinct ARCSRT EN row** in this CF image — it selects the Motoweld pendant program. Synergic wire↔current samples live in `WELDER.DAT` / `WELDUDEF.DAT`.

Motoweld `WELDER.DAT` curve (wire m/min ↔ A):

| wire | 0.00 | 1.35 | 2.70 | 10.80 | 12.15 | 13.50 |
|------|------|------|------|-------|-------|-------|
| A    | 30   | 62   | 94   | 286   | 318   | 350   |

Interpolated wire at ARCSRT 200 A ≈ **7.2 m/min** (used in UI / JBI comments).

## ARGF usage in live jobs

| ARGF (PROG-NR) | Example jobs | Typical `V=` near ARCON | Typical `WEV#` |
|----------------|--------------|-------------------------|----------------|
| **1** | `12.JBI`, `11.JBI`, `123.JBI` | 4.2–5.0 | (often none) |
| **4** | `1-DEFRO-5.JBI`, DEFRO-70*, `02025_KOSZ…` | ≈5.0 (sometimes 4.2) | 15, 16 |
| **5** | `1.JBI`, `1C.JBI`, `1-DEFRO-5-KOSZ.JBI` | ≈4.2 | 1 (kosz) |
| **6** | `1B.JBI` | ≈5.0 | — |
| **8** | `000.JBI`, DEFRO-70-NOWE/KOPIA | 5.0–7.5 | **21**, 22, 25 |
| **10** | `-DEFRO-70-PRZYLACZE-70KASUJ.JBI` | ≈5.0 | 1, 2 |

CAD/CAM UI conditions map to ARGF **8 / 5 / 4 / 1** (see `LORCH_S8_SCHEDULES` / `MOTOWELD_ARGF_SCHEDULES`).

## WEAV.CND — mined `WEV#` (1, 15, 16, 21)

Parsed primary row: `amp_L, amp_R, angle_L, angle_R, frequency, aux`.

| WEV# | Mode | Amp L/R (mm) | Angle L/R (°) | Freq | Used in |
|------|------|--------------|---------------|------|---------|
| **1** | 0 | 2.0 / 2.0 | 70 / 70 | 2.0 | DEFRO-60_STARE, kosz, KASUJ |
| **15** | 1 | 3.0 / 3.0 | 45 / 45 | 3.0 | `1KT.JBI`, `02025_KOSZ…` |
| **16** | 1 | 2.0 / 2.0 | 45 / 0 | 2.0 | `02025_KOSZ…` (after 15) |
| **21** | 0 | 4.0 / 0.0 | 0 / 0 | 2.0 | DEFRO-70 production default |

Also seen in jobs but not required for M3 UI: `WEV#(2)`, `22`, `23`, `24`, `25`.

Default export weave: **`WEV#(21)`**.

## TouchSense / search — `123123.JBI`

```
MOVS C00001 BC00001 V=11.0  +MOVJ EC00001 VJ=0.78
MOVS C00002 BC00002 V=11.0  +MOVJ EC00002 VJ=0.78
```

| Field | Value |
|-------|--------|
| Opcode | `MOVS` |
| Search speed | `V=11.0` (cm/min) |
| Station sync | `+MOVJ EC# VJ=0.78` |
| Groups | `RB1,BS1` + `ST2` in this job |

CAD/CAM emits the same `MOVS` grammar for waypoints with `kind: "sense"` / `tag: "TOUCH"`.

## Warning

Values in the UI and JBI comments are **not a qualified WPS**. Confirm Motoweld PROG-NR, gas, stick-out, and weave on the pendant; validate pulse jobs in MotoSim before CF load.
