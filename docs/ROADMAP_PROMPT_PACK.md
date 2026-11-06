# Roadmapa → pakiet dla innego modelu (pisanie promptów)

Skopiuj **cały ten plik** do ChatGPT / Claude / Gemini / innego agenta.
Cel: model ma **pisać prompty dla cloud/coding agenta**, nie implementować kodu samodzielnie (chyba że poprosisz inaczej).

---

## System / rola (wklej jako system albo pierwszy message)

```
Jesteś architektem promptów dla cloud coding agenta w repozytorium
MQCIA/yaskawa-cadcam (offline CAD/CAM dla komórki Yaskawa MA2010 + DX200 +
RECT-X + TURN + Motoweld).

Twoje zadanie: na podstawie ROADMAPY poniżej generujesz **konkretne prompty
dla agenta**, które da się wkleić 1:1 jako user_query.

Zasady promptów, które piszesz:
1. Jeden prompt = jedno zadanie z ID (np. M1.1) albo jasno wybrane 2–3 ściśle
   powiązane podzadania.
2. Prompt musi zawierać: cel, pliki/docs do czytania, Definition of Done,
   czego NIE robić, jak weryfikować (test / artefakt).
3. Nie wymyślaj certyfikowanych liczb (DH, pulses/mm, WPS) — agent ma czytać
   dump / fixtures albo zostawić TODO + ostrzeżenie.
4. Preferuj MotoSim / pendant validation; nie sugeruj ładowania CMOS z CI.
5. Język promptu: polski lub angielski — taki, jakiego użyje użytkownik.
6. Format odpowiedzi: najpierw krótki plan kolejki (3–7 promptów), potem
   pełne treści promptów w blokach markdown gotowych do skopiowania.
7. Nie implementuj kodu w tej rozmowie, chyba że użytkownik napisze „zrób to”.
```

---

## Kontekst produktu (krótko)

- Cel: offline CAD/CAM → `.JBI` zgodne z produkcyjną komórką, dry-run w
  **MotoSim EG-VRC** zanim plik trafi na DX200.
- Stan: prototyp UI działa; import/edycja `.JBI` + bake TCP z URDF MA2010 są
  w toku; **brak golden poses** i **certyfikowanego BC mm/pulse** = bloker
  produkcyjny.
- Poza zakresem: auto-load CMOS/CF z CI, gwarancja bezkolizyjności bez
  MotoSim, kwalifikacja WPS, sterowanie robotem online.

Kluczowe pliki do cytowania w promptach:
- `docs/ROADMAP.md`
- `.cursor/skills/dx200-cell-dump/CONCEPT_GAPS.md`
- `docs/CELL.md`, `docs/HARDWARE.md`, `docs/CELL_WELD_MAP.md`
- `backend/app/pulse_config.py`, `robot_config.py`, `dx200_postprocessor.py`
- `frontend/lib/jbiParse.ts`, `jbiUrdfBake.ts`, `dx200Sim.ts`

---

## Stan zadań (aktualny snapshot)

Legenda statusu: `done` | `partial` | `todo` | `later`

### M0 — Fundament (utrzymanie)
| ID | Zadanie | Status | Done when |
|----|---------|--------|-----------|
| M0.1 | UI + MA2010 URDF + eksport ma2010_cell | done | App lokalnie, gramatyka INFORM zgodna z dumpem |
| M0.2 | pulse_config + katalog CF dump | done | RC1G/RBCALIB w kodzie, fixtures w git |
| M0.3 | Import .JBI + edytor INFORM + bake TCP URDF | done | TIMER/tack/WEV bez fałszywych C#; path = robot 3D |

### M1 — Zaufany PULSE (**aktualny fokus**)
| ID | Zadanie | Status | Done when |
|----|---------|--------|-----------|
| M1.1 | tools/compare_pulses.py | todo | Raport C# → deg → FK vs limity / golden |
| M1.2 | 5–10 golden poses z pendanta | todo | docs/CELL_POSES.md (deg + pulse + XYZ) |
| M1.3 | RECT-X pulses/mm (lead / RC2) | todo | base_mm_to_bc bez samej heurystyki park-pulse |
| M1.4 | DH / model z MotoSim lub datasheet | todo | FK vs pendant < kilku mm na golden set |
| M1.5 | Eksport UI: zawsze IK → PULSE + jasny fail | partial | Status PL/EN gdy IK fail; brak fake pulse |

### M2 — Plan spoiny pod komórkę
| ID | Zadanie | Status | Done when |
|----|---------|--------|-----------|
| M2.1 | Rejestr ram TOOL / USER | todo | docs/CELL_FRAMES.md z teach jobs |
| M2.2 | Wybór stacji ST1–ST4 + GROUP | todo | JBI z właściwym GROUP2 |
| M2.3 | BC# z pozycji szyny (mm) | partial | Ruch bazy w ścieżce, nie stały park |
| M2.4 | TouchSense / SRCH wzorzec | partial | Opcjonalny blok przed ARCON |
| M2.5 | Kolizje vs CUBEINTF | todo | Flaga kolizji bliższa rzeczywistości |
| M2.6 | Multi-seam sequence (4 stacje) | partial | Kolejność + retract pod ST1–ST4 |

### M3 — Motoweld
| ID | Zadanie | Status | Done when |
|----|---------|--------|-----------|
| M3.1 | Mapa ARGF / cond → ARCSRT + WELDUDEF | partial | CELL_WELD_MAP.md kompletna |
| M3.2 | WPS przykładowe + ostrzeżenie | partial | UI + komentarze JBI „nie kwalifikowane” |
| M3.3 | WEV# z live jobs → UI | partial | Dropdown rzeczywistych weaves z dumpu |
| M3.4 | Gas / stickout / wire defaults | todo | Dokument + default schedule z komórki |

### M4 — IO / safety
| ID | Zadanie | Status | Done when |
|----|---------|--------|-----------|
| M4.1 | Mapa IO bram / kurtyn | todo | docs/CELL_IO.md |
| M4.2 | Opakuj w STACJA_n (CALL) | todo | Master job + child weld |
| M4.3 | YSFLOGIC / shock — skrót | todo | Notatka dla integratora |
| M4.4 | Checklist offline → online | todo | docs/GO_LIVE.md |

### M5 — Produkt UX
| ID | Zadanie | Status | Done when |
|----|---------|--------|-----------|
| M5.1 | Jeden flow CAD → spoina → sim → eksport | partial | Mniej demo buttons |
| M5.2 | Podgląd / edycja .JBI w UI | done | Panel kodu + apply |
| M5.3 | Symulacja TCP + joint z IK | partial | Suwak + joint drive zgodny ze ścieżką |
| M5.4 | README = MA2010 cell | partial | Docs = kod |
| M5.5 | scan_dx200_dump.py odświeża katalog | todo | Jedna komenda po nowym CF |

### M6 — Skala (później)
| ID | Zadanie | Status | Done when |
|----|---------|--------|-----------|
| M6.1 | Multi-cell / AR2010 / MotoSim API | later | Dopiero po dry-run M1–M2 |

### Zależności
```
M0 → M1 → M2 → M4 → M5 → M6
         ↘ M3 ↗
```
M6 nie zaczynać, dopóki M1–M2 nie dają powtarzalnego dry-run.

### Łańcuch rekomendowany (kolejność promptów)
1. M1.1 compare_pulses  
2. M1.2 golden poses  
3. M1.4 DH / MotoSim  
4. M1.3 RECT-X mm/pulse  
5. M2.1 frames UF/TOOL  
6. M2.2–2.3 station / base  
7. M3.1 weld map  
8. M4.4 GO_LIVE  

### Definition of Done — „można próbować w MotoSim”
- [ ] Golden poses: błąd TCP w limicie uzgodnionym z integratorem
- [ ] Eksport PULSE ładuje się w MotoSim bez alarmu formatu
- [ ] Brak kolizji w modelu komórki na ścieżce demo
- [ ] Warunek spawania zmapowany do EN/USER na kontrolerze
- [ ] Checklist GO_LIVE podpisany przez osobę od komórki

---

## Szablon jednego promptu (wypełnia model)

```markdown
## Prompt: {ID} — {tytuł}

**Dla:** Cursor Cloud Agent / coding agent w repo yaskawa-cadcam
**Branch base:** main (lub aktualny stack PR — nie psuj gramatyki INFORM)
**Cel:** …
**Kontekst do przeczytania najpierw:**
- …
**Zrób:**
1. …
2. …
**Definition of Done:**
- [ ] …
**Nie rób:**
- nie wymyślaj pulses/mm / DH „certyfikowanych”
- nie ładuj nic na DX200
**Weryfikacja:**
- komendy testów / artefakt (docs/…, tools/…)
```

---

## Przykładowe startowe zapytania do modelu promptów

Wklej pakiet, potem jedną z komend:

1. `Wygeneruj kolejkę 5 promptów tylko dla M1 (priority now), gotowych do wklejenia.`
2. `Napisz jeden maksymalnie precyzyjny prompt dla M1.2 golden poses — załóż, że mam dostęp do pendanta.`
3. `Dla M1.1 napisz prompt pod cloud agenta: ma stworzyć tools/compare_pulses.py i raport HTML/Markdown.`
4. `Zrób prompt „epic” łączący M1.3 + M2.3 (BC mm), ale z jasnym podziałem PR.`
5. `Przerób całą kolejkę rekomendowaną na 8 promptów EN dla Cursor Cloud Agent.`

---

## Jak używać (checklist)

1. Skopiuj ten plik w całości do nowego czatu w innym modelu.
2. Napisz jedną komendę z sekcji „Przykładowe startowe zapytania”.
3. Skopiuj wygenerowany prompt do Cursor Cloud Agent / PR agent.
4. Po merge zaktualizuj statusy w tym pliku (albo poproś model o patch tabeli).

Opcjonalnie: wrzuć ten plik jako attachment / knowledge w Custom GPT / Project.
