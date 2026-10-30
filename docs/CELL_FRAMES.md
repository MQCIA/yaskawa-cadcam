# Ramki komórki — TOOL / USER / CUBE / TouchSense (M2.1 / M2.4)

Rejestr z dumpa DX200 (`reference/robot-cell-dx200`, 2026-09-15).
Kod: `backend/app/cell_frames.py` (lustrzane stałe w `frontend/lib/weldProgram.ts`).

## TOOL (``TOOL.CND``)

Produkcyjne joby spawania (`000.JBI`, `HOME_TRACK`, DEFRO, KOSZ…) mają
**`///TOOL 0`**. TOOL 1…63 w CND to placeholdery zerowe.

| TOOL | Nazwa | TCP xyz mm | RPY deg | Uwagi |
|------|-------|------------|---------|-------|
| **0** | STANDARD TOOL11 | `-54.280, 1.052, 415.702` | `1.3119, -42.6215, -6.8387` | Live torch; FK/IK × TOOL 0 |
| 1…63 | (puste) | `0,0,0` | `0,0,0` | Nieużywane w jobach spawania |

Dodatkowe wiersze TOOL 0 w CND (masa / drugi punkt): secondary
`-34.329, -59.651, 325.206` mm · mass `1.836` kg.

Aplikacja: `robot_config.TOOL0_XYZ_MM` / `TOOL0_RPY_DEG` = TOOL 0;
postprocessor emituje `///TOOL {tool_no}` (domyślnie 0).

## USER / UF#

| UF# | Gdzie | Pulsowe joby spawania? |
|-----|-------|-------------------------|
| **1** | `DROP01.JBI` — `///USER 1`, `///POSTYPE USER`, `////FRAME USER 1` | **Nie** — tylko przegląd RECTAN / offline |
| (teach) | `TCP-R1-Y-BA*`, `EICH-R1-Y-BA` | Kalibracja TCP; brak pełnych C# w gicie |

Ścieżka produkcyjna CAD→IK→PULSE **nie wybiera UF#**: pozycje są w impulsach
przegubów względem bazy robota + TOOL 0. Profil `cartesian_review` nadal może
użyć `///USER 1` jak DROP01.

## CUBEINTF (``cnd/CUBEINTF.CND``)

Miękkie strefy / approach cubes (narożniki w impulsach monitorowanych osi):

| Cube | Nazwa | Stacja | corner A (pierwsze 3) | corner B (pierwsze 3) |
|------|-------|--------|----------------------|----------------------|
| 1 | STACJA_1 | ST1 (ST3≈) | 632891, -23550, 139716 | 622891, -33550, 129716 |
| 2 | STACJA_2 | ST2 (ST4≈) | 633450, 15097, 139701 | 623450, 5097, 129701 |
| 3… | (puste) | — | — | — |

Flagi wiersza typu: `2,0,1`. Pełna interpretacja mm wymaga MotoSim /
dokumentacji CUBEINTF — w aplikacji cuby wiążemy ze `STATION_FRAMES` po nazwie
stacji do checków kolizji / dokumentacji.

## TouchSense (``SRCH-*`` / ``SZUKAJ1``)

Wzorzec produkcyjny:

```
SZUKAJ1.JBI          ///POSTYPE PULSE + BASE
  MOVL C# …
  CALL JOB:SRCH-2D1
  MOVL C# …

SRCH-2D1.JBI         ROBOT / PULSE / BASE
  MOVL P001 V=33.3
  MOVL P002 V=15.0 SRCH RIN#(1)=ON T=0.20 DIS=20.0
  … retract …
  MOVL P003 V=15.0 SRCH RIN#(1)=ON T=0.20 DIS=10.0

SRCH-1D1.JBI         1D: SRCH DIS=10.0
```

| | |
|--|--|
| RIN | `#(1)` |
| V search | `15.0` |
| T | `0.20` s |
| DIS | `20` / `10` mm (2D) lub `10` mm (1D) |

Eksport PULSE z włączonym TouchSense:
1. Waypointy `kind=sense` (pozycje CAD przed ARCON).
2. Komentarze + **`CALL JOB:SRCH-2D1`** przed pierwszym ARCON (jak master
   `SZUKAJ1`) — job `SRCH-2D1` musi istnieć na kontrolerze / w MotoSim.

Wyłączenie: `emit_search_call=False` / opcja UI bez TouchSense.

## Powiązane

- [CELL_POSES.md](CELL_POSES.md) — home, ST1–ST4 strefy robocze
- [CELL.md](CELL.md) — tożsamość komórki
- [CELL_DUMP_CATALOG.md](CELL_DUMP_CATALOG.md) — mapa plików dumpa
