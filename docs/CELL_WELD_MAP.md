# Mapa spawalnicza komórki (Motoweld / ARGF / WEV#)

Źródło: dump DX200 `reference/dx200-backup-20260915` (2026-09-15) + joby
produkcyjne. Kod: `backend/app/welding_config.py`.

**UWAGA:** wartości A/V w UI są **heurystyką APP_ESTIMATE** — **nie**
kwalifikowanym WPS. Przed załadowaniem na DX200 zweryfikuj
`MACRO1 MJ#(0) ARGFn` oraz `WVON WEV#(n)` na pendancie / w MotoSim.

---

## Mechanizm na kontrolerze

| Element | Plik / instrukcja | Rola |
|---------|-------------------|------|
| Zasilacz | `WELDER.DAT` | MOTOWELD-E Series 350A |
| Start łuku | `ARCSRT.CND` (`//ARCSRT EN n`, 1000 slotów) | Parametry startu |
| Koniec łuku | `ARCEND.CND` | Parametry wyłączenia |
| Tabele USER | `WELDUDEF.DAT` (`USER-1`…`USER-64`) | Krzywe wire/A/V |
| Splot | `WEAV.CND` (`//WEAV 1`…`255`) | Amplituda / kąt / czas |
| Wybór w jobie | `MACRO1 MJ#(0) ARGFn` → `ARCON` | **Bez** `ASF#(...)` |
| Makro | `MACRO.DAT` MACRO1#1 `///JOB1 LORCH` | Legacy nazwa; `ARGF` = PROG-NR |

W tej komórce **nigdy** nie używamy `ARCON ASF#(…)`.

---

## MACRO1 `ARGF` w jobach z hali (M3.1)

| ARGF | Joby | WELDUDEF | ARCSRT EN | Uwagi dump |
|------|------|----------|-----------|------------|
| **1** | `11.JBI`, `12.JBI`, `123.JBI` | USER-1 | EN 1 | ST1/ST2 krótkie programy |
| **4** | `1-DEFRO-5.JBI`, `02025_KOSZ_DO_RURY_70-PLUS-RAMKI.JBI` | USER-4 | EN 4 | DEFRO / KOSZ |
| **5** | `1-DEFRO-5-KOSZ.JBI` | USER-5 | EN 5 | DEFRO kosz + WEV#1 |
| **8** | `000.JBI` | USER-8 | EN 8 | Park / demo |

`ARCSRT EN 1/4/5/8` w tym dumpie mają **identyczny** szablon wiersza
`200,20.0,100` (A / V / %) — różnica programów idzie przez makro + USER table,
nie przez oczywisty unikalny amperaż w EN. Stąd zakaz „certyfikowania” A/V z UI.

Dodatkowe ARGF w innych jobach dumpa (poza listą M3.1): `ARGF6` (`1B.JBI`),
`ARGF10` (`-DEFRO-70-…KASUJ.JBI`) — nie w dropdownie domyślnym.

---

## `WVON WEV#` w jobach (M3.3)

| WEV# | Joby | WEAV.CND (skrót) |
|------|------|------------------|
| **Brak** | `11.JBI`, `12.JBI`, `1-DEFRO-5.JBI` | — (tylko ARCON) |
| **1** | `1-DEFRO-5-KOSZ.JBI` | amp ≈2/2 mm, 70/70 |
| **15** | `1KT.JBI`, `02025_KOSZ…` | amp ≈3/3 mm, 45° |
| **16** | `02025_KOSZ…` | amp ≈2/2 mm, 45° |
| **21** | `-DEFRO-70-PRZYLACZE-70-KOPIA-.JBI` (+ slot w WEAV.CND) | amp ≈4 mm |
| 22 / 24 / 25 | `000.JBI` (rzeczywisty dump) | różne amp/kąt — poza listą UI M3.3 |

Dropdown UI (zgodnie z M3.3): **Brak, 1, 15, 16, 21**.

---

## Presety UI (`MOTOWELD_SCHEDULES`)

| Cond / ARGF | Heurystyka A/V (nie WPS) | Travel V≈ |
|-------------|--------------------------|-----------|
| 1 | 180 A / 22.0 V | 5.0 |
| 4 | 200 A / 20.0 V | 5.0 |
| 5 | 180 A / 22.0 V | 5.0 |
| 8 | 220 A / 24.0 V | 6.7 |

Alias wsteczny: `LORCH_S8_SCHEDULES = MOTOWELD_SCHEDULES`.

---

## Ostrzeżenie w `.JBI`

Każdy eksport `ma2010_cell` zawiera komentarz:

```
' NOTE: UNQUALIFIED WPS - VERIFY MACRO1 ARGF AND WEAV ON PENDANT
```

---

## Gas / wire / stickout (M3.4)

Z `WELDER.DAT` (MOTOWELD-E Series 350A):

| Parametr | Wartość w dumpie | Uwagi |
|----------|------------------|-------|
| Wire Ø | **1.2** mm | Pierwszy token wiersza `1.2,0,15,0.1,0.60` |
| Gas | „COMBINATION GAS AND WIRE” | Brak składu % w tekście → UI: `CO2/Mix (cell)` |
| Stickout | **nie nazwany** w CND | `TODO: weryfikacja na hali` — nie inventować mm |
| UI gas default | `CO2/Mix (cell)` | `MOTOWELD_SCHEDULES[].gas` / `CELL_PROCESS_DEFAULTS` |

Kod: `backend/app/welding_config.py` → `CELL_PROCESS_DEFAULTS`.
