# Złote pozy komórki MA2010 / DX200

Pozy odniesienia do zgodności PULSE / TOOL 0. Impulsy pochodzą z jobów INFORM.
Stopnie liczy skala `backend/app/pulse_config.py` (RC1G + RBCALIB).

**TOOL 0** (`TOOL.CND` / `///TOOL 0` w każdym jobie produkcyjnym):

| | wartość |
|--|--|
| xyz mm | `-54.280, 1.052, 415.702` |
| RPY deg | `1.3119, -42.6215, -6.8387` |
| ‖tcp_0 − tool0‖ | `√(54.280² + 1.052² + 415.702²) ≈ 419.232 mm` |

Stałe: `backend/app/robot_config.py` (`TOOL0_*`, `CELL_HOME_PULSES`) oraz
`frontend/lib/weldProgram.ts`. Łańcuch MotoROS2:
`flange (REP-103) → tool0 (Z+ out) → tcp_0 (TOOL.CND)`.

**XYZ w tabelach to FK modelu DH × TOOL 0**, nie współrzędne z pendantu
(`MA2010.approximate=True` do M1.4).

## Skala impulsów

| Oś | impuls/stopień |
|----|----------------|
| S L U R B T | 1341.38, 1907.68, 1592.89, 1022.87, 980.24, 454.76 |
| TURN oś 1 (`EC` pierwsza) | 91771 / 90° ≈ 1019.69 |
| RECT-X `BC` | park `878036` / home szyny `570125` (impuls/mm TBD) |

## Home nad torem (`C00000`)

Bezpieczna poza transportowa nad szyną BS1 — `C00000` i ostatni punkt wielu
jobów DEFRO / KOSZ. Kanoniczna szóstka (źródło prawdy w kodzie):

| | S | L | U | R | B | T |
|--|--|--|--|--|--|--|
| impulsy | **-281** | **-101163** | **-84214** | **3730** | **-80188** | **-1936** |
| stopnie | −0.209 | −53.029 | −52.869 | +3.647 | −81.804 | −4.257 |

Źródła w fixture’ach: `-DEFRO-70_PRZYLACZE_60_STARE.JBI`,
`02025_KOSZ_DO_RURY_70-PLUS-RAMKI.JBI`, `-DEFRO-70-PRZYLACZE-70-NOWE.JBI`.

Wariant ±2 impulsy na S/L/U/R (np. `-283,-101162,-84213,3710,…` w `1B.JBI`)
to ta sama poza uczona — różnice uczenia / zaokrągleń.

| Szyna | Typowy EC |
|--|--|
| `BC = 570125` | `192,0` … `1166,0` (TURN-1) |

Przy home fajka TOOL 0 (wygięcie ≈ −42.6° Ry) jest skierowana bezpiecznie
w dół/przód nad torem, bez kolizji z `link_3_u` / `link_4_r`.

## Park i pozy z `000.JBI` (ST1)

Źródło: `backend/tests/fixtures/robot-cell/000.JBI`.

- `///TOOL 0`, `///POSTYPE PULSE`, `///GROUP1 RB1,BS1`, `///GROUP2 ST1`
- `BC` park = **878036**
- `EC` ≈ `-1746,0` → −1.712°

| Rola | punkt | impulsy S,L,U,R,B,T |
|--|--|--|
| dojazd MOVJ | C00000 | 95902, -19559, -65353, -68228, -62443, 70439 |
| pierwszy MOVC (po ARCON) | C00002 | 96643, 1036, -73382, -77097, -56952, 78162 |
| spawanie MOVL | C00005 | 144285, 13854, -56584, 51164, -49608, 19486 |
| ostatni MOVL przed ARCOF | C00018 | 135827, -7962, -68903, 53184, -44238, -62042 |
| odjazd MOVJ | C00019 | 116865, -4881, -33684, -3254, -73880, -61311 |

`000.JBI` **nie** używa home nad torem — szyna stoi na parku.

## Orientacja palnika względem spoiny

Podczas planowania (`path_planning.plan_from_seams`):

* `tcp_0` +Z (kierunek wysuwu drutu) = wektor podejścia do pachwiny
  (`−bisector` + kąt pochylenia / travel),
* approach / retract offset wzdłuż `+bisector` (nie world +Z),
* korpus fajki (TOOL 0 RPY) zostaje w ramce `tool0` — nie jest zgadywany
  wizualnie na froncie.

## Czego ta tabela jeszcze nie jest

- Nie zastępuje XYZ z pendantu.
- Nie daje `BC` w mm ani stopni drugiej osi TURN-2.
