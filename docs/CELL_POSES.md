# Złote pozy komórki MA2010 / DX200

Pozy odniesienia do M1 (zaufany PULSE). Impulsy pochodzą z jobów INFORM.
Stopnie liczy `tools/compare_pulses.py` skalą z `backend/app/pulse_config.py`
(RC1G + RBCALIB). Poza TCP jest FK modelu DH razy TOOL 0.

TOOL 0 (`TOOL.CND`): xyz `-54.280, 1.052, 415.702` mm, RPY `1.3119, -42.6215, -6.8387` deg.

**XYZ i RPY w tabelach nie są współrzędnymi z pendantu.** Długości DH są
placeholderem (`robot_config.MA2010.approximate`). Round-trip IK na home i na
większości punktów `000.JBI` wraca do tych samych impulsów, więc skala
pulse↔stopień jest wewnętrznie spójna. Zgodność z uczoną pozycją kartezjańską
to dopiero M1.4.

Powtórzenie liczb:

```bash
python tools/compare_pulses.py backend/tests/fixtures/robot-cell
```

Pełne joby produkcyjne leżą w `reference/robot-cell-dx200/jobs/` (gitignore).
W tym repozytorium jest kompletny `000.JBI` oraz fixture home. Reszta nazw
jest w `docs/_dump_scan.json` (nagłówki, bez `C#`).

## Skala

| Oś | impuls/stopień |
|----|----------------|
| S L U R B T | 1341.38, 1907.68, 1592.89, 1022.87, 980.24, 454.76 |
| TURN oś 1 (`EC` pierwsza liczba) | 91771 / 90° ≈ 1019.69 |
| TURN oś 2 (`EC` druga liczba) | brak certyfikowanej skali |
| RECT-X `BC` | brak impuls/mm. Znane są dwa punkty szyny, nie lead |

Osie home i wszystkich `C#` z `000.JBI` mieszczą się w softlimitach RC1G.

## Home nad torem

Ta sama szóstka `C` jest pozycją bezpieczną: `C00000` i ostatni punkt joba.
Widziana w `1-DEFRO-5.JBI`, `1-DEFRO-5-KOSZ.JBI`,
`02025_KOSZ_DO_RURY_70-PLUS-RAMKI.JBI`, `11.JBI`, `12.JBI`, `1KT.JBI`,
`123.JBI`. W gicie trzyma ją `backend/tests/fixtures/robot-cell/HOME_TRACK.JBI`
(nie jest to kopia całego joba, tylko ta poza).

`000.JBI` tej pozy **nie** używa. Tam szyna stoi na parku.

| | S | L | U | R | B | T |
|--|--|--|--|--|--|--|
| impulsy | -283 | -101162 | -84213 | 3710 | -80188 | -1936 |
| stopnie | -0.211 | -53.029 | -52.868 | 3.627 | -81.804 | -4.257 |

| Szyna | Stacja | TCP modelu (mm, deg) | round-trip |
|--|--|--|--|
| `BC = 570125` | osobno, nie jest częścią tej pozy robota | X -1463.1, Y -31.5, Z 1967.7, Rx -160.60, Ry -53.58, Rz 17.06 | ΔPULSE 0, ΔTCP 0.00 mm |

Różnica szyny park − home = `878036 − 570125 = 307911` impulsów. Na milimetry
nie przeliczać, dopóki nie ma leadu RECT-X (M1.3).

## Park szyny i pozy z `000.JBI` (ST1, TURN-1)

Źródło: `backend/tests/fixtures/robot-cell/000.JBI`.

- `///NPOS 20,20,20,0,0,0`, `///TOOL 0`, `///POSTYPE PULSE`
- `///GROUP1 RB1,BS1`, `///GROUP2 ST1`
- `BC` = 878036 na starcie, dalej 878036 lub 878037 (jeden impuls)
- `EC` ≈ `-1746,0` → **-1.712°** na osi 1, druga oś 0
- instrukcje: 3× MOVJ, 7× MOVL, 10× MOVC, ARCON, ARCOF, 4× WVON, MACRO1 `ARGF8`

| Rola | punkt | impulsy S,L,U,R,B,T | stopnie S,L,U,R,B,T | TCP modelu mm / RPY deg | ΔPULSE max | ΔTCP mm |
|--|--|--|--|--|--|--|
| dojazd, pierwszy MOVJ, przed łukiem | C00000 | 95902, -19559, -65353, -68228, -62443, 70439 | 71.495, -10.253, -41.028, -66.703, -63.701, 154.894 | -265.8, 636.2, 2507.3 / -105.23, -76.97, -127.05 | 0 | 0.00 |
| pierwszy punkt łuku (MOVC po ARCON) | C00002 | 96643, 1036, -73382, -77097, -56952, 78162 | 72.047, 0.543, -46.068, -75.373, -58.100, 171.876 | -168.0, 951.7, 2450.7 / -75.19, -80.75, -118.04 | 30 | 0.16 |
| spawanie MOVL, `V=5.0` | C00005 | 144285, 13854, -56584, 51164, -49608, 19486 | 107.564, 7.262, -35.523, 50.020, -50.608, 42.849 | -140.2, 1312.6, 2387.5 / 12.09, -15.23, -32.88 | 5 | 0.02 |
| ostatni MOVL przed ARCOF | C00018 | 135827, -7962, -68903, 53184, -44238, -62042 | 101.259, -4.174, -43.256, 51.995, -45.130, -136.429 | 180.2, 799.6, 2621.6 / -164.80, 74.18, -20.89 | 86 | 0.29 |
| odjazd MOVJ po ARCOF | C00019 | 116865, -4881, -33684, -3254, -73880, -61311 | 87.123, -2.559, -21.146, -3.181, -75.369, -134.821 | 54.9, 904.4, 2421.7 / 130.01, 19.74, 45.78 | 0 | 0.00 |

`BC00000 = 878036`. Od C00003 w górę `BC = 878037`. `EC00000 = -1746,0`,
przy łuku `-1743,0` albo `-1744,0`.

Na C00003 solver IK, z ziarnem z poprzedniej nauczanej pozy, trafia w TCP
(błąd poniżej 1 mm), ale inną konfiguracją nadgarstka (max |ΔPULSE| ≈ 90812).
To nie jest błąd skali impulsów. Ziarno kolejnego punktu jest znowu nauczanym
`C#`, nie tą obcą konfiguracją. Od C00004 round-trip wraca do impulsów joba.

## Stacje ST1–ST4

`EC` ma zawsze dwie liczby. S1 i S2 to TURN-1 (jedna oś, druga liczba 0).
S3 i S4 to TURN-2 (dwie osie). Grupa w jobie i tak nazywa się `ST1`…`ST4`.

| Stacja | Sprzęt | Job w skanie dumpa | Co jest w gicie |
|--|--|--|--|
| ST1 | S1 TURN-1 | `000.JBI` (`NPOS` 20), `1.JBI` (162) | pełne `C#`/`BC#`/`EC#` tylko dla `000.JBI` |
| ST2 | S2 TURN-1 | `12.JBI` (25), `02025_KOSZ_DO_RURY_70-PLUS-RAMKI.JBI` (218) | nagłówki w `_dump_scan.json`, bez ciał `C#` |
| ST3 | S3 TURN-2 | `1-DEFRO-5.JBI` (9, GROUP2 ST3) | nagłówek; home `C` jak wyżej, reszta punktów nie |
| ST4 | S4 TURN-2 | `BAZA1.JBI` (6, GROUP2 ST4) | nagłówek |

`11.JBI`, `1KT.JBI`, `123.JBI` i `1-DEFRO-5-KOSZ.JBI` niosą to samo home, ale
nie ma ich wpisu w `_dump_scan.json`, więc grupy z tego checkoutu nie da się
potwierdzić.

Przykłady `EC` z jobów (nie parować ich na siłę z home `C#`):

| Zapis | Oś 1 stopnie | Oś 2 | Gdzie |
|--|--|--|--|
| `192,0` | 0.188 | 0, oś nieużywana | TURN-1, dojazd 1-osiowy |
| `-1746,0` | -1.712 | 0 | `000.JBI`, spawanie na ST1 |
| `-2638,-94527` | -2.587 | -94527 impulsów, stopni nie liczyć | TURN-2, zapis dwuosiowy |

## Czego ta tabela jeszcze nie jest

- Nie zastępuje złotych poz z pendantu (XYZ uczone, ta sama poza zmierzona na
  kontrolerze).
- Nie daje `BC` w milimetrach.
- Nie daje stopni drugiej osi TURN-2.
- Duży ΔPULSE przy małym błędzie TCP znaczy drugą konfigurację IK, nie złą
  skalę RC1G.
