# Mapa I/O bezpieczeństwa i stacji (MA2010 / DX200)

Źródła tekstowe z dumpa CF (`reference/dx200-backup-20260915` / pełny katalog w `docs/_dump_scan.json`):

| Plik | Rola |
|------|------|
| `IONAME.DAT` | Symboliczne nazwy wejść (`//IN`) i wyjść (`//OUT`) |
| `EXIONAME.DAT` | Rozszerzone IO — w tym dumpie **puste** sloty |
| `CIOPRG.LST` / `CIO.PRM` | Logika CIO (brak czytelnych etykiet bram w skanie tekstowym) |
| `YSFLOGIC.DAT` | FSU / safety circuit — **binarny / nieczytelny** bez schemata |
| `CUBEINTF.CND` | Soft-cube `STACJA_1` / `STACJA_2` (+ nienazwane) |
| `jobs/STACJA_*.JBI`, `OTWORZ-BRAMA-*`, `ZAMKNIJ-KURTYNA-*`, `RESET-KURTYNY` | Orkiestracja bram / kurtyn przez `CALL JOB:` |

**Zasada:** ten dokument **nie** podaje numerów `SIN#` / `SOUT#` / bitów FSU do „obejścia”. Numery fizyczne → `TODO: weryfikacja w CIO / schemacie elektrycznym`. CAD/CAM generuje wyłącznie joby ruchu / opcjonalny master `CALL` — **nie** steruje kurtynami poza wywołaniem istniejących jobów na kontrolerze.

Indeks `IONAME[n]` poniżej to **kolejny slot 4-nazwowy w `IONAME.DAT`** (pomocniczy), **nie** certyfikowany Universal Input/Output DX200.

---

## 1. Kurtyny świetlne (`KURTYNA-*`)

Nazwy w dumpie: strony **1–4**, kanały **G** / **D** (góra/dół lub lewa/prawa — potwierdzić na schemacie).

### Wejścia (`//IN`)

| IONAME slot | Nazwa | Znaczenie (robocze) | Adres SIN / CIO |
|------------:|-------|--------------------|-----------------|
| 37–39 | `KURTYNA-4-G`, `KURTYNA-3-G`, `KURTYNA-3-D` | Feedback / OK kurtyny | `TODO: weryfikacja w CIO / schemacie elektrycznym` |
| 40–43 | `KURTYNA-3-G/D`, `KURTYNA-4-G/D` | Duplikaty / druga grupa nazw | `TODO: …` |
| 56–59 | `KURTYNA-2-G/D` (×2) | Feedback strona 2 | `TODO: …` |
| 60–63 | `KURTYNA-1-G/D` | Feedback strona 1 | `TODO: …` |

### Wyjścia (`//OUT`)

| IONAME slot | Nazwa | Znaczenie (robocze) | Adres SOUT / CIO |
|------------:|-------|--------------------|------------------|
| 56 | `KURTYNA-1-G` | Sterowanie / lampka kurtyny 1 | `TODO: weryfikacja w CIO / schemacie elektrycznym` |
| 57 | `KURTYNA-1` | Kurtyna 1 (zbiorczo) | `TODO: …` |
| 58–59 | `KURTYNA-2-G`, `KURTYNA-2-D` | Strona 2 | `TODO: …` |
| 60–63 | `KURTYNA-3-G/D`, `KURTYNA-4-G/D` | Strony 3–4 | `TODO: …` |

### Joby IO (bez pozycji — `///NPOS 0,0,0,0,0,0`)

| Job | Rola w komórce |
|-----|----------------|
| `RESET-KURTYNY` | Reset stanu kurtyn przed cyklem stacji |
| `ZAMKNIJ-KURTYNA-STRONA-1` … `-4` | Zamknięcie / aktywacja kurtyny danej strony |
| `OTWORZ-BRAMA-STRONA-1` … `-4` | Otwarcie bramy danej strony (dostęp operatora / wjazd) |

Treści `DOUT`/`WAIT` tych jobów **nie są w podzbiorze git** — pełne ciała są na CF dumpie lokalnym. Nie odtwarzać numerów IO „na ślepo”.

---

## 2. Potwierdzenia / start stacji ST1–ST4

### Wejścia operator / powietrze

| IONAME slot | Nazwa | Uwagi |
|------------:|-------|-------|
| 2–4 | `JOG-ST1 ON`, `JOG+ ST1`, `JOG-ST1` | Jog stacji 1 |
| 5–7 | `JOG ON ST2`, `JOG+ ST2`, `JOG- ST2` | Jog stacji 2 |
| 9 | `POWIETRZE OK` | Potwierdzenie sprężonego powietrza |
| 10 | `START_ST1` | Start cyklu stacji 1 |
| 11 | `START_ST2` | Start cyklu stacji 2 |
| 14–15 | `JOG ST3 ON`, `JOG ST4 ON` | Jog ST3/ST4 |
| — | `START_ST3` / `START_ST4` | **Brak nazw** w `IONAME.DAT` → `TODO: weryfikacja w CIO / schemacie elektrycznym` |

### Wyjścia statusu stacji

| IONAME slot | Nazwa |
|------------:|-------|
| 3–4 | `JOG_ON_ST1`, `JOG_ON_ST2` |
| 16 | `RUNNING_ST1` |
| 18 | `RUNNING_ST2` |
| 38–39 | `RUNNING ST3`, `RUNNING ST4` |

INFORM grupy stacji (ruch): `///GROUP2 ST1`…`ST4` — patrz [CELL_FRAMES.md](CELL_FRAMES.md) / `CELL_STATIONS`.

---

## 3. Bramki / master `STACJA_n` (wzorzec produkcyjny)

Z `docs/_dump_scan.json` (pełny dump jobów):

| Master | GROUP | Sekwencja (skrót) | Child weld (przykład w dumpie) |
|--------|-------|-------------------|--------------------------------|
| `STACJA_1` | `RB1,BS1` + `ST1` | `RESET-KURTYNY` → `ZAMKNIJ-KURTYNA-STRONA-1` ×4 → `OTWORZ-BRAMA-STRONA-1` → `TIMER T=1.00` → `CALL JOB:…` | `KANON-RAMKA120-60` |
| `STACJA_2` | `RB1,BS1` + `ST2` | reset → zamknij 2,2,3,4 → otwórz bramę 2 → timer → CALL | `2025-RAMKI-KANION14-60NOWE2` |
| `STACJA_3` | `RB1,BS1` + `ST3` | reset → zamknij 1–4 → otwórz bramę 3 → timer → CALL | `CEUWKATYGULA` |
| `STACJA_3-BEZ-CLEAN` | j.w. | jak ST3 | `25RUKA` |
| `STACJA_4` | `RB1,BS1` + `ST4` | reset → zamknij 1–4 → otwórz bramę 4 → timer → CALL | `25RUKA` |

Nagłówek pustych / call-only jobów (np. `1-DEFRO-60.JBI` w podzbiorze git):

```
///NPOS 0,0,0,0,0,0
///ATTR SC,RW,CJ
///GROUP1 RB1,BS1,ST3   (albo GROUP1 RB1,BS1 + GROUP2 STn jak STACJA_*)
```

Atrybut **`CJ`** = job wywoływalny / call-job. Generator CAD/CAM: `generate_station_master_jbi()` — **opcjonalny** cienki wrapper `CALL JOB:<weld>`; pełna choreografia kurtyn zostaje w produkcyjnych `STACJA_*` na kontrolerze.

Soft interference: `CUBEINTF.CND` nazwane `STACJA_1`, `STACJA_2` (+ kolejne nienazwane) — strefy podejścia, nie zastępują FSU.

---

## 4. Sygnały źródła spawalniczego / process IO

| Kierunek | IONAME slot | Nazwa | Uwagi |
|----------|------------:|-------|-------|
| IN | 51 | `LORCH OK` | Legacy label — komórka ma **Motoweld-E**; traktować jako „power source ready” do potwierdzenia |
| OUT | 5–7 | `CLEANING MOTOR`, `CLEANING OIL`, `WIRECUT` | Cleaning / obcinanie drutu |
| OUT | 9–10 | `ZAWOR GLOWNY`, `PRZEDMUCH` | Gaz / przedmuch |
| OUT | 26 | `BLOKADA SPAWARKI` | Blokada źródła |
| OUT | (wysoki slot tabeli) | `MAN GAS`, `WIRE FEED`, `WIRE REVERSE` | Ręczne / feed — mapowanie Universal OUT → `TODO: weryfikacja w CIO / schemacie elektrycznym` |

Wiadomość CIO: `MISSING ARC START CONFIRM` (`CIOPRG.LST`) — potwierdza, że start łuku jest nadzorowany sprzętowo/CIO, nie tylko `ARCON` w JBI.

Warunki procesu (`MACRO1 ARGF`, `WVON WEV#`) → [CELL_WELD_MAP.md](CELL_WELD_MAP.md) (gdy obecny) / `ARCSRT.CND` / `WEAV.CND`.

---

## 5. Świadomie poza zakresem CAD/CAM

- Omijanie kurtyn, bram, deadmana, FSU, `YSFLOGIC` — **zakazane**.
- Autoload `.JBI` na produkcyjny DX200 z pominięciem MotoSim i [GO_LIVE.md](GO_LIVE.md) — **zakazane**.
- Wymyślanie numerów `SOUT`/`SIN` gdy brak ich w tekstowym dumpie.

---

## 6. Checklist weryfikacji IO (integrator)

- [ ] Porównać każdą nazwę `KURTYNA-*` / `START_ST*` z szafy / schematem → wpisać prawdziwy Universal I/O #
- [ ] Potwierdzić polarność (OK = 1 czy 0) dla `POWIETRZE OK`, `LORCH OK`
- [ ] Odtworzyć ciała `OTWORZ-BRAMA-*` / `ZAMKNIJ-KURTYNA-*` z pełnego CF i dopisać DOUT/WAIT
- [ ] Sprawdzić `CUBEINTF` vs layout hali w MotoSim EG-VRC
- [ ] Podpisać [GO_LIVE.md](GO_LIVE.md) przed pierwszym dry-run na DX200
