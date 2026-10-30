# Ramki komórki — TOOL / USER / CUBE / TouchSense (M2.1 / M2.4 / M2.5)

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
| (teach) | `TCP-R1-Y-BA.JBI`, `TCP-R1-Y-BAREL.JBI`, `EICH-R1-Y-BA.JBI` | Kalibracja TCP; mixed TOOL/BASE/USER postype; brak pełnych C# w gicie |

Ścieżka produkcyjna CAD→IK→PULSE **nie wybiera UF#**: pozycje są w impulsach
przegubów względem bazy robota + TOOL 0. Profil `cartesian_review` nadal może
użyć `///USER 1` jak DROP01.

Teach-joby UF/TCP są wymienione w `cell_frames.UF_TEACH_JOBS` (nagłówki w
`docs/_dump_scan.json`; pełne ciała pod `reference/`, gitignore).

## CUBEINTF (``cnd/CUBEINTF.CND``) — M2.5

Miękkie strefy / approach cubes. Narożniki w impulsach monitorowanych osi;
skala geometryczna ≈ **1000 pulse/mm** (jednostki 0.001 mm jak długości RC1G).

| Cube | Nazwa | Stacja | corner A (pierwsze 3) | corner B (pierwsze 3) | AABB mm (min→max) |
|------|-------|--------|----------------------|----------------------|-------------------|
| 1 | STACJA_1 | ST1 (ST3≈) | 632891, -23550, 139716 | 622891, -33550, 129716 | X 622.9–632.9 · Y −33.6…−23.6 · Z 129.7–139.7 |
| 2 | STACJA_2 | ST2 (ST4≈) | 633450, 15097, 139701 | 623450, 5097, 129701 | X 623.5–633.5 · Y 5.1–15.1 · Z 129.7–139.7 |
| 3… | (puste) | — | — | — | — |

Flagi wiersza typu: `2,0,1`.

Kod:
* `cube_aabb_m` / `all_cube_aabbs_m` (`cell_frames.py`)
* `cube_interference_aabbs_m` / `path_hits_cube_interference` / `check_collision_free` (`path_planning.py`)
* Frontend: `cubeAabbM` / `allCubeAabbsM` → `cellObstacles()` w `checkCollisionFree`

Cuby (~10 mm) to soft keep-outs w ramie robota; pedestały / szyna i
`STATION_FRAMES` bbox pozostają osobnymi przeszkodami / strefami roboczymi.
Walidacja w MotoSim przed CF.

## TouchSense (``SRCH-*`` / ``SZUKAJ1``) — M2.4

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
1. Waypointy `kind=sense` w odległościach **DIS=20 mm** i **DIS=10 mm** wzdłuż
   +bisector (CAD) — `ensureTouchSense` / `insert_touch_sense_points`.
2. Komentarze z pełną gramatyką SRCH-2D1 + **`CALL JOB:SRCH-2D1`** przed
   pierwszym ARCON (jak master `SZUKAJ1`) — job musi istnieć na kontrolerze /
   w MotoSim.
3. Inline `MOVL … SRCH` nie jest emitowane (wymaga taught P-vars na CF);
   dokumentacja bloku jest w komentarzach INFORM.

Wyłączenie: `emit_search_call=False` / `emit_touch_sense_call=false` / UI bez
TouchSense.

## Powiązane

- [CELL_POSES.md](CELL_POSES.md) — home, ST1–ST4 strefy robocze
- [CELL.md](CELL.md) — tożsamość komórki
- [CELL_DUMP_CATALOG.md](CELL_DUMP_CATALOG.md) — mapa plików dumpa
