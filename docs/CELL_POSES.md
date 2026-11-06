# Złote pozy komórki MA2010 / DX200

Rejestr **8** reprezentatywnych póz z produkcyjnych `.JBI` (impulsy bez zmian).
Stopnie: `pulse_config.py` (RC1G / RBCALIB).  
`APP_ESTIMATE`: FK aplikacji = **URDF MA2010** (ROS-I) × TOOL.CND na flange — **nie** jest odczytem z pendanta (dopóki `verified_on_pendant=false`).

Maszynowo: `backend/tests/fixtures/robot-cell/golden_poses.json`  
Sprawdzenie: `python3 tools/compare_pulses.py --check-golden`

TOOL 0 (`TOOL.CND`): xyz `-54.280, 1.052, 415.702` mm · RPY `1.3119, -42.6215, -6.8387` deg.

| Status kartezjański | Znaczenie |
|--|--|
| `UNVERIFIED_ON_PENDANT` | Brak wpisanego XYZ/RPY z DX200 / MotoSim — **zakaz** traktować `APP_ESTIMATE` jako certyfikat |
| `VERIFIED_ON_PENDANT` | Wypełnione `pendant_or_motosim` + `verified_on_pendant: true` w JSON |

---

## Tabela 8 póz

Impulsy skopiowane z jobów. `APP_ESTIMATE` przelicz live: `--check-golden` (poniżej wartości z generacji dokumentu — mogą lekko dryfować przy zmianie DH).

| ID | Źródło | ST | C# | S | L | U | R | B | T | BC | EC1 | EC2 | S…T ° (pulse_config) | APP_ESTIMATE XYZ mm / RPY ° | PENDANT_OR_MOTOSIM_XYZ_RPY |
|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|
| `POSE_01_SAFE_HOME` | `02025_KOSZ…JBI` | rail-home | C00000 | -281 | -101163 | -84214 | 3730 | -80188 | -1936 | 570125 | 192 | 0 | -0.209, -53.029, -52.869, 3.647, -81.804, -4.257 | 1041.0, -32.7, 1182.0 / 4.32, 38.99, -5.91 | `TODO` — CURRENT POSITION → ROBOT (CARTESIAN), TOOL 0 |
| `POSE_02_PARK_000` | `000.JBI` | ST1 | C00000 | 95902 | -19559 | -65353 | -68228 | -62443 | 70439 | 878036 | -1746 | 0 | 71.495, -10.253, -41.028, -66.703, -63.701, 154.894 | 11.5, 854.7, 1101.4 / -178.23, -1.52, 40.76 | `TODO` |
| `POSE_03_ST1_WELD_START` | `11.JBI` | ST1 | C00003 | 97267 | 52739 | -6802 | 12719 | -17839 | -9459 | 927851 | 1163 | 0 | 72.512, 27.646, -4.270, 12.435, -18.199, -20.800 | 624.6, 1760.6, 1010.1 / 69.68, 1.12, -7.05 | `TODO` |
| `POSE_04_ST2_KOSZ_WELD_1` | `02025_KOSZ…JBI` | ST2 | C00003 | 150409 | 61689 | -12910 | 81671 | -34099 | -23144 | 124171 | -932 | 0 | 112.130, 32.337, -8.105, 79.845, -34.786, -50.893 | -828.7, 1692.3, 742.4 / 115.41, -48.61, 5.22 | `TODO` |
| `POSE_05_ST2_KOSZ_WELD_MID` | `02025_KOSZ…JBI` | ST2 | C00100 | 119061 | 39982 | -23058 | 19092 | -67348 | 19868 | 352913 | -933 | 0 | 88.760, 20.958, -14.476, 18.665, -68.705, 43.689 | -258.3, 1649.5, 554.9 / 122.57, -51.60, -54.59 | `TODO` |
| `POSE_06_ST2_ARC_MOVC` | `12.JBI` | ST2 | C00013 | 94466 | 78056 | 15184 | 22601 | -26793 | -15578 | 262818 | 2712 | 0 | 70.424, 40.917, 9.532, 22.096, -27.333, -34.256 | 782.2, 1874.6, 883.2 / 59.86, 3.44, -13.45 | `TODO` |
| `POSE_07_ST3_ROTARY_WELD` | `1-DEFRO-5.JBI` | ST3 | C00004 | -83555 | 68320 | -11474 | 26050 | -31610 | -59171 | 808680 | -2638 | -94527 | -62.290, 35.813, -7.203, 25.468, -32.247, -130.116 | 294.4, -1410.5, 535.2 / -158.41, -41.49, -6.54 | `TODO` |
| `POSE_08_ST4_WELD` | `1KT.JBI` | ST4 | C00003 | -103675 | 64756 | 3591 | -48976 | -53898 | 45246 | 190240 | -2 | -1 | -77.290, 33.945, 2.254, -47.881, -54.984, 99.495 | 699.6, -1591.4, 967.9 / 1.28, 7.84, -4.52 | `TODO` |

Wszystkie wiersze: **`cartesian_status = UNVERIFIED_ON_PENDANT`**, `verified_on_pendant = false`.

Uwagi:

- Warianty „home” w innych jobach różnią się o kilka impulsów (`-283…`, `-288…`) — kanoniczny `POSE_01` to wpis z `02025_KOSZ…` C00000.
- `EC` oś 2 (TURN-2) bez certyfikowanej skali ° — tylko impulsy.
- `BC` bez impuls/mm (M1.3).

---

## Instrukcja operatora — spisać XYZ z pendanta / MotoSim

Cel: dla każdej z 8 póz wpisać **6 liczb** `(X, Y, Z, Rx, Ry, Rz)` w układzie **ROBOT (CARTESIAN)** przy **TOOL 0**, **bez ruszania robotem** (albo z MotoSim EG-VRC na tej samej konfiguracji impulsów).

### A) Pendant DX200 (preferowane)

1. Upewnij się, że aktywne jest **TOOL 0** (ekran TOOL / CURRENT TOOL).
2. Otwórz job źródłowy z tabeli (np. `11.JBI`) w trybie teach / play stop.
3. Ustaw kursor na linii ruchu z danym `C#####` **albo** ręcznie ustaw osie S…T / BC / EC na wartości z tabeli (pulse) — robot musi stać w tej konfiguracji.
4. Ekran **CURRENT POSITION**:
   - przełącz wyświetlanie z **PULSE** na **ROBOT** (czasem etykieta **CARTESIAN** / **BASE** zależnie od oprogramowania — wybierz układ **robot / tool0**, nie user-frame spoiny, chyba że dokumentujesz UF osobno).
5. Spisz `X Y Z Rx Ry Rz` (mm, °) dokładnie jak na ekranie.
6. W `golden_poses.json` dla tej pozy:
   - ustaw `"pendant_or_motosim": {"x_mm": …, "y_mm": …, "z_mm": …, "rx_deg": …, "ry_deg": …, "rz_deg": …}`,
   - `"verified_on_pendant": true`,
   - `"cartesian_status": "VERIFIED_ON_PENDANT"`,
   - dodaj `"verified_at"` (data ISO) i `"verified_by"` (inicjały).
7. Zaktualizuj kolumnę w tej tabeli Markdown (zamień `TODO` na liczby).
8. Uruchom `python3 tools/compare_pulses.py --check-golden` — pojawi się `Δtcp_mm` vs `APP_ESTIMATE`.

### B) MotoSim EG-VRC

1. Załaduj projekt komórki zgodny z dumpem CF.
2. Otwórz ten sam `.JBI`, przejdź do `C#####`.
3. Odczytaj TCP w TOOL 0 / robot frame (jak w pendancie).
4. Wpisz jak w kroku 6 powyżej; w notatce zaznacz `source: MotoSim`.

### Czego nie robić

- Nie kopiuj wartości `APP_ESTIMATE` do kolumny pendant.
- Nie „zaokrąglaj na oko” impulsów z joba.
- Nie ładuj nic na produkcyjny DX200 tylko po to, by spisać pozy — teach/stop wystarczy.

---

## Skala (przypomnienie)

| Oś | impuls/° |
|--|--|
| S L U R B T | ≈ 1341.38, 1907.68, 1592.89, 1022.87, 980.24, 454.76 |
| TURN oś 1 (`EC` pierwsza) | 91771 / 90° ≈ 1019.69 |
| TURN oś 2 / RECT-X `BC` | brak certyfikatu ° / mm |

---

## Powiązanie z M1

| Kamień | Rola tego rejestru |
|--|--|
| M1.1 `compare_pulses` | `--check-golden` czyta ten JSON |
| M1.2 (ten plik) | 8 póz + miejsce na pendant |
| M1.3 | BC → mm po leadzie |
| M1.4 | DH vs pendant `Δtcp_mm` w limicie integratora |
