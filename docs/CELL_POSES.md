# Złote pozy komórki MA2010 / DX200

Pozy odniesienia do M1 (zaufany PULSE). Impulsy pochodzą z jobów INFORM.
Stopnie liczy `tools/compare_pulses.py` skalą z `backend/app/pulse_config.py`
(RC1G + RBCALIB).

Kinematyka: ROS-Industrial URDF MA2010
(`frontend/public/models/ma2010/ma2010.urdf`) przez `ERobot` w
`kinematics_engine.py`, plus TOOL 0 z dumpa.

| Element | Wartość |
|---------|---------|
| d1 (baza→S) | 505 mm |
| a1 (S→L) | 150 mm |
| a2 (L→U) | 760 mm |
| a3 (U→R) | 200 mm |
| d4 (R→B) | 1082 mm |
| d6 (B/T→flange / tool0) | 100 mm |
| osie URDF | S +Z, L +Y, U −Y, R −X, B −Y, T −X |
| limity | z URDF = `MA2010_LIMITS_DEG` (S ±180 … T ±210) |
| TOOL 0 | xyz `-54.280, 1.052, 415.702` mm · RPY `1.3119, -42.6215, -6.8387` deg |

**XYZ/RPY w tabelach to FK modelu URDF × TOOL 0**, nie nauczone współrzędne z
pendantu. Round-trip IK na tych pozach wraca do tych samych impulsów
(ΔPULSE ≈ 0…kilkadziesiąt). Porównanie z XYZ z pendantu / MotoSim to nadal
M1.4.

```bash
python tools/compare_pulses.py backend/tests/fixtures/robot-cell
```

Pełne joby produkcyjne: `reference/robot-cell-dx200/jobs/` (gitignore).
W gicie: `000.JBI` + fixture home. Nagłówki innych jobów: `docs/_dump_scan.json`.

## Skala

| Oś | impuls/stopień |
|----|----------------|
| S L U R B T | 1341.38, 1907.68, 1592.89, 1022.87, 980.24, 454.76 |
| TURN oś 1 (`EC` pierwsza liczba) | 91771 / 90° ≈ 1019.69 |
| TURN oś 2 (`EC` druga liczba) | brak certyfikowanej skali |
| RECT-X `BC` | **≈257.708 impuls/mm** (kalibracja ze środków zakresów ST2 vs ST1/ST3 / 2400 mm; brak leadu w RC1G) |

Osie home i wszystkich `C#` z `000.JBI` mieszczą się w softlimitach RC1G.

## Home nad torem

Ta sama szóstka `C` jest pozycją bezpieczną: `C00000` i ostatni punkt joba.
Widziana w `1-DEFRO-5.JBI`, `1-DEFRO-5-KOSZ.JBI`,
`02025_KOSZ_DO_RURY_70-PLUS-RAMKI.JBI`, `11.JBI`, `12.JBI`, `1KT.JBI`,
`123.JBI`. Fixture: `backend/tests/fixtures/robot-cell/HOME_TRACK.JBI`.

`000.JBI` tej pozy **nie** używa. Tam szyna stoi na parku.

Domyślne ziarno IK (`q0`) dla pierwszego punktu ścieżki to właśnie ta poza
(`robot_config.CELL_HOME_PULSES` → `cell_home_deg()`).

| | S | L | U | R | B | T |
|--|--|--|--|--|--|--|
| impulsy | -283 | -101162 | -84213 | 3710 | -80188 | -1936 |
| stopnie | -0.211 | -53.029 | -52.868 | 3.627 | -81.804 | -4.257 |

| | X mm | Y mm | Z mm | Rx | Ry | Rz |
|--|--|--|--|--|--|--|
| flange (URDF tool0) | 638.7 | -8.6 | 1066.4 | -28.04 | 80.54 | 24.71 |
| TCP (× TOOL 0) | 1041.0 | -32.7 | 1182.0 | 4.31 | 38.99 | -5.89 |

| Szyna | Stacja (przykład) | round-trip |
|--|--|--|
| `BC = 570125` | `EC = 192,0` (0.188°) albo dwuosiowe `-2638,-94527` | ΔPULSE 0, ΔTCP 0.00 mm |

Różnica szyny park − home = `878036 − 570125 = 307911` impulsów ≈ **1194.8 mm**
przy `PULSES_PER_MM ≈ 257.708` (`pulse_config.track_*`). `mm = 0` w `BC_HOME`.

## Park szyny i pozy z `000.JBI` (ST1, TURN-1)

Źródło: `backend/tests/fixtures/robot-cell/000.JBI`.

- `///NPOS 20,20,20,0,0,0`, `///TOOL 0`, `///POSTYPE PULSE`
- `///GROUP1 RB1,BS1`, `///GROUP2 ST1`
- `BC` = 878036 na starcie, dalej 878036 lub 878037
- `EC` ≈ `-1746,0` → **-1.712°** na osi 1, druga oś 0
- instrukcje: 3× MOVJ, 7× MOVL, 10× MOVC, ARCON, ARCOF, 4× WVON, MACRO1 `ARGF8`

TCP poniżej = FK(URDF) × TOOL 0.

| Rola | punkt | impulsy S,L,U,R,B,T | stopnie | TCP mm / RPY deg | ΔPULSE max | ΔTCP mm |
|--|--|--|--|--|--|--|
| dojazd MOVJ | C00000 | 95902, -19559, -65353, -68228, -62443, 70439 | 71.495, -10.253, -41.028, -66.703, -63.701, 154.894 | 11.5, 854.7, 1101.4 / -178.23, -1.52, 40.76 | 0 | 0.00 |
| pierwszy łuk MOVC | C00002 | 96643, 1036, -73382, -77097, -56952, 78162 | 72.047, 0.543, -46.068, -75.373, -58.100, 171.876 | 1.6, 857.1, 846.4 / -178.24, -1.52, 40.76 | 5 | 0.05 |
| spawanie MOVL | C00005 | 144285, 13854, -56584, 51164, -49608, 19486 | 107.564, 7.262, -35.523, 50.020, -50.608, 42.849 | -678.4, 1139.4, 406.3 / -121.47, -72.48, 67.40 | 28 | 0.46 |
| ostatni MOVL przed ARCOF | C00018 | 135827, -7962, -68903, 53184, -44238, -62042 | 101.259, -4.174, -43.256, 51.995, -45.130, -136.429 | 173.3, 1021.8, 908.9 / 14.64, 16.48, 14.40 | 0 | 0.00 |
| odjazd MOVJ | C00019 | 116865, -4881, -33684, -3254, -73880, -61311 | 87.123, -2.559, -21.146, -3.181, -75.369, -134.821 | 333.7, 889.8, 1068.1 / -53.78, 35.98, 33.41 | 1 | 0.01 |

`BC00000 = 878036`. Od C00003 w górę `BC = 878037`. `EC00000 = -1746,0`.

Na URDF łańcuchu cały `000.JBI` round-tripuje w granicach dziesiątek impulsów
(wcześniejszy placeholder DH gubił konfigurację nadgarstka już na C00003).

## Stacje ST1–ST4

`EC` ma zawsze dwie liczby. S1/S2 = TURN-1 (`p,0`). S3/S4 = TURN-2 (dwie osie).

| Stacja | Sprzęt | Job w skanie dumpa | W gicie |
|--|--|--|--|
| ST1 | S1 TURN-1 | `000.JBI`, `1.JBI` | pełne `C#` dla `000.JBI` |
| ST2 | S2 TURN-1 | `12.JBI`, `02025_KOSZ_…` | nagłówki w `_dump_scan.json` |
| ST3 | S3 TURN-2 | `1-DEFRO-5.JBI` | nagłówek + home fixture |
| ST4 | S4 TURN-2 | `BAZA1.JBI` | nagłówek |

Przykłady `EC`:

| Zapis | Oś 1 ° | Oś 2 | Gdzie |
|--|--|--|--|
| `192,0` | 0.188 | 0 | TURN-1 |
| `-1746,0` | -1.712 | 0 | `000.JBI` ST1 |
| `-2638,-94527` | -2.587 | impulsy, bez skali ° | TURN-2 |

## Czego ta tabela jeszcze nie jest

- Nie zastępuje złotych poz z pendantu (XYZ uczone na kontrolerze).
- Nie daje `BC` w milimetrach.
- Nie daje stopni drugiej osi TURN-2.
- Duży ΔPULSE przy małym błędzie TCP znaczyłby drugą konfigurację IK — na
  obecnym URDF tego praktycznie nie widać na `000.JBI`.
