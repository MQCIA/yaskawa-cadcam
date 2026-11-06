# GO_LIVE — procedura odbiorcza (offline → MotoSim EG-VRC → DX200)

**Definition of Done** przed załadowaniem pliku `.JBI` wygenerowanego przez CAD/CAM na kontroler produkcyjny MA2010 / DX200.

Cel: żaden program z tej aplikacji **nie** trafia na DX200 z pominięciem MotoSim EG-VRC i poniższej listy.  
**Zakaz:** omijanie kurtyn, bram, FSU, deadmana; auto-upload `.JBI` z CI / skryptu bez tej checklisty.

Powiązane: [CELL_IO.md](CELL_IO.md) · [CELL_POSES.md](CELL_POSES.md) · [CELL.md](CELL.md) · [ROADMAP.md](ROADMAP.md)

---

## 0. Artefakty wejściowe

| Artefakt | Wymagane |
|----------|----------|
| Child weld `.JBI` (PULSE, `ma2010_cell`) | Tak |
| Opcjonalny master `STACJA_n.JBI` z `generate_station_master_jbi` | Gdy używany `CALL` |
| Docelowa stacja `ST1`…`ST4` + `MACRO1 ARGF` + `WEV#` | Tak |
| Osoba odpowiedzialna za komórkę (podpis §6) | Tak |

---

## 1. Weryfikacja kinematyki — `compare_pulses --check-golden`

```bash
python3 tools/compare_pulses.py --check-golden
```

| Kryterium | Status | Uwagi / wynik |
|-----------|:------:|---------------|
| Skrypt kończy się kodem 0 | ☐ | |
| Błąd TCP na złotych pozach ≤ limit uzgodniony z integratorem | ☐ | Limit: ______ mm (wpisać; do czasu `VERIFIED_ON_PENDANT` w [CELL_POSES.md](CELL_POSES.md) wynik jest **APP_ESTIMATE**) |
| Softlimity RC1G bez przekroczeń na golden | ☐ | |
| `RECT_X_LEAD_VERIFIED` — świadomość skali szyny | ☐ | Dziś `false` → BC# traktować ostrożnie |

**STOP** jeśli golden fail albo limit TCP nieuzgodniony — nie iść do MotoSim z produkcyjnym PULSE.

---

## 2. Walidacja nagłówka / gramatyki `.JBI`

Dla **child** weld joba (i mastera, jeśli używany):

| Check | Oczekiwane | ☐ |
|-------|------------|:-:|
| `///NPOS` | `n,n,n,0,0,0` (weld) lub `0,0,0,0,0,0` (master CALL) | ☐ |
| `///TOOL 0` | TOOL 0 (produkcja) | ☐ |
| `///POSTYPE PULSE` | Na child weld (nie RECTAN preview) | ☐ |
| Końce linii | CRLF `\r\n` przed load na DX200 / CF | ☐ |
| Start / home rail | `C00000` bookend oraz `BC=570125` (`BC_HOME`) gdy home bookends | ☐ |
| Grupy | `///GROUP1 RB1,BS1` + `///GROUP2 STn` | ☐ |
| Master (opcjonalnie) | `///ATTR SC,RW,CJ`, `CALL JOB:<child>`, **bez** własnych DOUT kurtyn | ☐ |
| Nazwa | Brak `PREVIEW_RECTAN_DO_NOT_LOAD` | ☐ |

Konwersja LF→CRLF: wyłącznie świadomie (np. przed CF); nie zmieniać treści instrukcji.

---

## 3. Import i pełny dry-run w **MotoSim EG-VRC**

| Krok | ☐ | Wynik |
|------|:-:|-------|
| Import child (+ master) do projektu komórki zgodnego z dumpem | ☐ | |
| Brak błędów składni INFORM | ☐ | |
| Brak kolizji 3D na ścieżce (robot / szyna / stół / cube `STACJA_*`) | ☐ | |
| Brak przekroczeń limitów osi / osobliwości | ☐ | |
| Master `CALL` znajduje child na kontrolerze symulowanym | ☐ | |
| Kurtyny / bramy: użyć **produkcyjnych** `STACJA_*` / `OTWORZ-*` / `ZAMKNIJ-*` z CF — nie „wyłączać” w symulacji na skróty | ☐ | |

**STOP** przy alarmie formatu, kolizji lub limitu — wróć do planu / IK.

---

## 4. Warunki spawania — `MACRO1` / `WVON` vs tabele kontrolera

| Check | ☐ |
|-------|:-:|
| `MACRO1 MJ#(0) ARGF#` w JBI = wybrany warunek; istnieje odpowiadający blok w `ARCSRT.CND` (EN) / `WELDUDEF` USER-n na **tym** DX200 | ☐ |
| `WVON WEV#(n)` (jeśli użyte) istnieje w `WEAV.CND` na kontrolerze | ☐ |
| UI A/V / wire = **APP_ESTIMATE** — nie kwalifikowane WPS; potwierdzenie technologa | ☐ |
| Brak `ARCON ASF#` (ta komórka: bare `ARCON` po MACRO1) | ☐ |

---

## 5. Pierwszy przejazd na fizycznym DX200

**Tryb:** TEACH + **DRY RUN** (bez łuku / źródło wyłączone lub zablokowane).

| Warunek | ☐ |
|---------|:-:|
| Plik wgrany ręcznie po zielonym MotoSim (nie z CI auto-push) | ☐ |
| Prędkość obniżona; ręka na **deadmanie** | ☐ |
| Kurtyny / bramy / FSU **aktywne** — bez bypassu | ☐ |
| Obserwacja BC/EC vs oczekiwana stacja; gotowość do E-STOP | ☐ |
| Po dry-run OK: osobna zgoda na próbę z łukiem (poza tą checklistą CAD/CAM) | ☐ |

---

## 6. Tabela podpisów

| Rola | Imię i nazwisko | Data (Europe/Warsaw) | Podpis |
|------|-----------------|----------------------|--------|
| Autor programu (CAD/CAM) | | | |
| Weryfikacja MotoSim EG-VRC | | | |
| Osoba odpowiedzialna za komórkę (DX200) | | | |
| Technolog spawalniczy (ARGF / WEV / WPS) | | | |

Potwierdzam, że kroki 1–5 są wykonane, kurtyny/FSU nie były omijane, a plik **nie** został wgrany z pominięciem MotoSim.

---

## Szybka komenda lokalna (przed §1)

```bash
cd backend && python -m pytest -q
python3 tools/compare_pulses.py --check-golden
```
