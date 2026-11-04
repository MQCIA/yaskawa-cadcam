# Analiza backupu DX200 + Lorch + iCAT mini

Źródła (upload 2026-11-04, lokalnie w `reference/` — katalog gitignore):

| Artefakt | Plik |
|----------|------|
| Backup CF DX200 | `kopia-DX200-15092026.zip` → `reference/dx200-backup-20260915/` (bez `CMOS.BIN`) |
| Lorch | `lorchs.pdf` = *Podręcznik użytkownika S SpeedPulse* `909.1180.5-01` (PL, 2009) |
| Palnik / mocowanie | `PRO_R154_GB_iCAT-Mini_WEB.pdf`, `iCAT-mini_BAL_0485_PL.pdf` (ABICOR BINZEL **iCAT mini**) |

**Brak w dostawie:** pliku CAD palnika (`.step` / `.stp` / `.stl` / `.glb`). Nie ma też joba **`LORCH.JBI`** (makro `MACRO1` wskazuje `///JOB1 LORCH`, ale pliku nie ma w zipie). Nie ma **`UFRAME.CND`**.

Kod wynikowy: `backend/app/pulse_config.py`, `robot_config.py`, `welding_config.py`, `frontend/lib/weldProgram.ts`, `frontend/lib/torchMesh.ts`.

---

## 1. TOOL.CND

Jedyny niezerowy tool w dumpie:

| Tool | NAME | X mm | Y mm | Z mm | Rx ° | Ry ° | Rz ° |
|------|------|------|------|------|------|------|------|
| **TOOL 0** | `STANDARD TOOL11` | **−54.280** | **1.052** | **415.702** | **1.3119** | **−42.6215** | **−6.8387** |
| TOOL 1…63 | (puste) | 0 | 0 | 0 | 0 | 0 | 0 |

Dodatkowe pola bloku TOOL 0 (dump):

| Linia | Wartość | Uwaga |
|-------|---------|--------|
| 2. punkt (COMARC / sensor?) | `−34.329, −59.651, 325.206` | nie używany w FK aplikacji |
| masa | `1.836` | kg (pole Motoman) |
| moment | `0.001,0.001,0.001` | |
| flagi | `0.000,0,2` | |

**Wszystkie joby `.JBI` w backupie mają `///TOOL 0`.**  
‖tcp_0 − tool0‖ = √(54.280² + 1.052² + 415.702²) ≈ **419.232 mm**.

Ry ≈ −42.62° jest najbliższe katalogowym szyjkom **ABIROB 45°** (różnica ~2.4°) — TCP z dumpa ma pierwszeństwo nad katalogiem.

---

## 2. Impulsy — RB1 / BS1 / ST1–ST4

### 2.1 Robot RB1 (MA2010) — `RC.PRM` / `ALL.PRM` `///RC1G`

Softlimity impulsów (indeksy 924–929 / 932–937):

| Oś | +pulse | −pulse | Limit URDF ° | ≈ pulse/° |
|----|--------|--------|--------------|-----------|
| S | 241449 | −241449 | ±180 | **1341.38** |
| L | 295690 | −200306 | +155 / −105 | **1907.68** |
| U | 254863 | −136989 | +160 / −86 | **1592.89** |
| R | 153430 | −153430 | ±150 | **1022.87** |
| B | 88222 | −132333 | +90 / −135 | **980.24** |
| T | 95499 | −95499 | ±210 | **454.76** |

`SV1G[24..29] = 4096` (rozdzielczość enkodera).  
`ABSO.DAT` — offsety absolutne (nie stosowane w INFORM PULSE export).

### 2.2 Tor BS1 (RECT-X)

| Fakt | Wartość | Źródło |
|------|---------|--------|
| Softlimit hint | `RC1G[8] = 1_082_000` | ALL/RC.PRM |
| Park (000.JBI) | **BC = 878036** | joby |
| Home nad torem | **BC = 570125** | DEFRO / HOME |
| Δ park−home | 307911 impulsów | |
| Zakres BC w jobach | 124171 … 1070065 | skan `*.JBI` |

**`PULSES_PER_MM` — nie da się wyliczyć z twardego dumpa.**  
W `ALL.PRM` / `RC.PRM` nie ma skoku śruby / leadu mm. Aplikacja nadal zwraca park pulse, dopóki lead nie zostanie zmierzony (pendant / tabliczka RECT-X).

### 2.3 Stacje ST1–ST4 (`SYSTEM.SYS` + `RBCALIB.DAT`)

| Stacja | Sprzęt | Osie | INFORM |
|--------|--------|------|--------|
| ST1 | S1 **TURN-1** | 1 | `EC#=p,0` |
| ST2 | S2 **TURN-1** | 1 | `EC#=p,0` |
| ST3 | S3 **TURN-2** | 2 | `EC#=a,b` |
| ST4 | S4 **TURN-2** | 2 | `EC#=a,b` |

Kalibracja RBCALIB (grupa stacji, oś 2 ≈ 90°):

| Calib | SSTC1 → SSTC2 (E2) | ΔE2 | ⇒ pulse/° (oś 2) |
|-------|--------------------|-----|------------------|
| RBCALIB 3 (SGROUP … S1) | 0 → −91771 | −91771 | **1019.688…** |
| RBCALIB 4 (SGROUP … S2) | −13771 → 91799 | +91799 | **1019.989…** |

W kodzie: `STATION_PULSES_PER_DEGREE = 91771/90` (oś produkcyjna TURN).  
Dla TURN-2 druga składowa EC używa tej samej skali, dopóki nie ma osobnego softlimitu (RC5G/RC6G softlimity = ±99999999 — nieskalowalne).

Oś 1 TURN-1 w jobach: małe wartości (np. −1746, 192, 1166) → stopnie przez tę samą skalę ≈ 1019.69 °⁻¹.

---

## 3. UFRAME.CND

**Brak pliku w backupie.** Żaden job nie używa `///USER n` / RECTAN USER frame — wyłącznie `///POSTYPE PULSE` + `///TOOL 0`.  
Wniosek: **brak zdefiniowanych ramek użytkownika** w tej kopii CF (albo nie zostały zbackupowane).

---

## 4. MACRO1 MJ#(0) → job `LORCH`

### 4.1 `MACRO.DAT` — `//MACRO1 1` (MJ#(0))

```
///ATTR 1,0,1,0,0,0,0,0
///JOB1 LORCH
///ARGSET 1,0,0,...
///ARGTYPE KD,...
///COMMENT1 WERT
///TAGCOMM PROG-NR:
```

- Wywołanie w jobach: `MACRO1 MJ#(0) ARGFn`
- **`ARGFn` = Tiptronic / PROG-NR n** przekazywane do joba `LORCH` (tag `PROG-NR:`)
- Zaobserwowane n w backupie: **1, 4, 5, 6, 8, 10**

### 4.2 Brak `LORCH.JBI`

Plik joba makra **nie jest w zipie**. Nie da się odczytać linijka-po-linijce `DOUT OG#` / `OT#` / `SET M#`.  
To jest największa luka backupu względem integracji Lorch.

### 4.3 Sygnały I/O z `IONAME.DAT` (twarde nazwy)

| Kierunek | # | Nazwa | Interpretacja |
|----------|---|-------|---------------|
| IN | 51 | **LORCH OK** | Gotowość / OK źródła Lorch |
| IN | 9 | POWIETRZE OK | powietrze / media |
| OUT | 7 | WIRECUT | obcinanie drutu |
| OUT | 26 | BLOKADA SPAWARKI | blokada źródła |
| OUT | 4091 | MAN GAS | test gazu (uniwersalne Motoman) |
| OUT | 4094 | WIRE FEED | posuw drutu |
| OUT | 4095 | WIRE REVERSE | cofanie drutu |

Start łuku w jobach: instrukcje **`ARCON` / `ARCOF`** (CIO aplikacji ARC WELDING), nie ręczne DOUT w `.JBI` użytkownika.  
TouchSense: **`MOVS`** (`123123.JBI`), bez osobnego bitowego „Touch” w IONAME.

---

## 5. WEAV / ARCSRT / ARCEND

### WEAV.CND — używane WEV#

Format wiersza geometrii: `ampL, ampR, angL, angR, freq, aux`  
Wiersz dwell: `t1,d1,t2,d2,...` (s)

| WEV# | Mode | Amp L/R mm | Kąt L/R ° | Freq Hz | Aux | Dwell (pierwszy rząd) |
|------|------|------------|-----------|---------|-----|------------------------|
| 1 | 0 | 2.0 / 2.0 | 70 / 70 | 2.0 | 2.0 | 0.1 / 0.0 … |
| 15 | 1 | 3.0 / 3.0 | 45 / 45 | 3.0 | 5.0 | 0.1 … + 0.3 na jednym boku |
| 16 | 1 | 2.0 / 2.0 | 45 / 0 | 2.0 | 1.0 | 0.1 / 0.0 … |
| 21 | 0 | 4.0 / 0.0 | 0 / 0 | 2.0 | 3.5 | 0.1 / 0.0 … |

### ARCSRT.CND

Wszystkie bloki `//ARCSRT EN` **identyczne**: teach row **`200, 20.0, 100`** → 200 A / 20.0 V / 100 %.  
`ARGFn` **nie** wybiera wiersza ARCSRT — wybiera Tiptronic PROG-NR w Lorch.

### WELDER.DAT

Nadal podpisane **`MOTOWELD-E Series 350A class`** (8× ten sam blok + krzywa synergic).  
To **nie przeczy** Lorchowi w makro: CF trzyma szablon Motoman Motoweld, a ścieżka produkcyjna woła Tiptronic przez `LORCH`. Krzywa wire↔A z `WELDER.DAT` zostaje tylko jako przybliżenie UI.

---

## 6. Lorch — architektura w tej celi

### Z backupu (pewne)

1. DX200 aplikacja **ARC WELDING** + CIO `ARCWELD`.
2. Przed `ARCON`: `MACRO1 MJ#(0) ARGFn` → job **`LORCH`** ustawia **PROG-NR = n** (Tiptronic).
3. Wejście **`LORCH OK` (IN#51)** — handshake ze źródłem.
4. Uniwersalne OT: gaz / wire feed / reverse / wirecut / blokada.

### Z PDF *S SpeedPulse* (ograniczenia)

- Tiptronic = zestawy zleceń / programów spawania wybierane numerem.
- Menu **„8: Interfejs robota”** odsyła do **osobnego** „Podręcznik … interfejs urządzenia INT” — **tego PDF-a nie dostarczono**.
- Brak w tym PDF mapowania bitów Tiptronic ↔ Yaskawa `OG#`.

### Wniosek mapowania ARGF

| ARGF | Rola w celi | Przykładowe joby |
|------|-------------|------------------|
| 1 | Tiptronic PROG 1 | 11, 12, 123 |
| 4 | Tiptronic PROG 4 | DEFRO, kosz |
| 5 | Tiptronic PROG 5 | 1, 1C, kosz |
| 6 | Tiptronic PROG 6 | 1B |
| 8 | Tiptronic PROG 8 (domyślny UI) | 000, DEFRO-70 |
| 10 | Tiptronic PROG 10 | KASUJ |

A/V w UI = szablon ARCSRT (200 A / 20 V), dopóki Lorch nie udostępni tabeli Tiptronic per PROG.

---

## 7. Palnik 3D — iCAT mini / ABIROB

### Identyfikacja sprzętu (PDF)

- Mocowanie: **ABICOR BINZEL iCAT mini** (kolizyjne), warianty pod **YASKAWA MA 2010** w katalogu PRO_R154.
- Szyjki: **ABIROB® G / W**, kąty katalogowe **0 / 22 / 30 / 35 / 45°**.
- TCP produkcyjny: wyłącznie **TOOL.CND** (Ry −42.62° ≈ 45° + stick-out).

### Model 3D w repo

| Stan | Opis |
|------|------|
| CAD z uploadu | **Brak** (tylko PDF iCAT) |
| Mesh w UI | `frontend/public/models/torch_esab/rm62_36deg.stl` — placeholder Verbotics, tip → TOOL 0 |
| Odrzucony download | Marathon „Robo WH PP W500” / części **Holder Abirob A300 ECO** — **zła rodzina** (A / ROBO WH ≠ ABIROB® G/W + iCAT mini) |
| Publiczny STEP G/W+iCAT | **Nie znaleziony** (Marathon / GrabCAD / TraceParts / OEM) |

Gdy pojawi się OEM STEP G/W + iCAT: `tools/convert_torch_cad.py` → `models/torch/` + macierz w `torchMesh.ts`.

---

## 8. Home C00000

| | S | L | U | R | B | T |
|--|--|--|--|--|--|--|
| impulsy | −281 | −101163 | −84214 | 3730 | −80188 | −1936 |
| ° (RC1G) | −0.209 | −53.029 | −52.869 | +3.647 | −81.804 | −4.257 |

Szczegóły: [CELL_POSES.md](CELL_POSES.md).

---

## 9. Checklist luk (wymaga kolejnego dumpa / dokumentów)

1. **`LORCH.JBI`** — pełna treść makra (DOUT / OG / M-reg).
2. **Lorch „INT” / robot interface manual** — mapa Tiptronic ↔ I/O.
3. **OEM CAD ABIROB G/W + iCAT mini** (STEP) — prawdziwy mesh (publiczny A300 ECO / ROBO WH odrzucony).
4. **Lead RECT-X (mm/rev lub pulses/mm)** — pomiar lub tabliczka.
5. **`UFRAME.CND`** — jeśli ramki są uczone poza tą kopią CF.
