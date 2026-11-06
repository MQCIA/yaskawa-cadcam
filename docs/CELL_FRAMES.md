# Ramki TOOL / USER i strefy stacji ST1–ST4

Rejestr **M2.1** wyekstrahowany z dumpa DX200 (`reference/dx200-backup-20260915`,
zgodny z fixture `backend/tests/fixtures/robot-cell/`) oraz produkcyjnych
`.JBI`. Kartezjańskie obwiednie spoin to **APP_ESTIMATE**: impulsy C# →
`pulse_config` → FK URDF MA2010 × TOOL.CND — **nie** odczyt z pendanta /
MotoSim, dopóki złote pozy nie mają `verified_on_pendant: true`.

Kod: `backend/app/robot_config.py` → `CELL_STATIONS` (eksponowane w
`GET /api/config`).

---

## TOOL.CND

| Slot | Nazwa | xyz mm | RPY deg | Uwagi |
|------|-------|--------|---------|-------|
| **TOOL 0** | `STANDARD TOOL11` | **−54.280, 1.052, 415.702** | **1.3119, −42.6215, −6.8387** | Jedyny niezerowy TCP w dumpie |
| TOOL 1…63 | (puste) | `0,0,0` | `0,0,0` | Sloty wyzerowane — nieużywane w jobach |

Dodatkowe pola TOOL 0 (dump, informacyjnie): CoG ≈ `−34.329, −59.651, 325.206` mm;
masa ≈ `1.836` kg; momenty bezwładności ≈ `0.001, 0.001, 0.001`.

**Produkcja:** wszystkie przeglądane joby mają `///TOOL 0`. Postprocesor
zostaje na TOOL 0 (bez zmiany).

---

## UFRAME.CND / USER

| Źródło | Stan |
|--------|------|
| `UFRAME.CND` w dumpie / fixture | **Brak pliku** (nie ma w CF dumpie z 2026-09-15) |
| Nagłówki `.JBI` | **Żaden** produkcyjny job nie ma `///USER n` |
| Uczenie | `///POSTYPE PULSE` + `///TOOL 0` + `///GROUP1 RB1,BS1` + `///GROUP2 STn` |

**Wniosek:** nie inventujemy ramek USER. Lokalizacja stanowiska jest zadana
impulsami `BC#` (RECT-X) + `EC#` (TURN) w przestrzeni pulsu kontrolera, nie
osobnym UFRAME.

---

## Nagłówki jobów (próbka)

| Job | GROUP2 | TOOL | USER | POSTYPE |
|-----|--------|------|------|---------|
| `000.JBI` | ST1 | 0 | — | PULSE |
| `11.JBI` | ST1 | 0 | — | PULSE |
| `12.JBI` | ST2 | 0 | — | PULSE |
| `02025_KOSZ_DO_RURY_70-PLUS-RAMKI.JBI` | ST2 | 0 | — | PULSE |
| `123123.JBI` | ST2 | 0 | — | PULSE (brak ARCON) |
| `1-DEFRO-5.JBI` | ST3 | 0 | — | PULSE |
| `1-DEFRO-5-KOSZ.JBI` | ST3 | 0 | — | PULSE |
| `123.JBI` | ST3 | 0 | — | PULSE (brak ARCON) |
| `1KT.JBI` | ST4 | 0 | — | PULSE |

Chmura spawania = ruchy `MOVL` / `MOVC` / `MOVJ` między `ARCON` a `ARCOF`.

---

## Strefy ST1–ST4 (weld AABB + BC/EC)

Osie `EC#`:

* **ST1 / ST2** — 1 aktywna oś TURN (`EC=val,0`), SYSTEM.SYS: S1/S2 TURN-1
* **ST3 / ST4** — 2 osie TURN (`EC=val1,val2`), SYSTEM.SYS: S3/S4 TURN-2

`default_bc` = mediana BC# punktów spawania danej stacji.
`mount_xyz_m` = środek chmury TCP (APP_ESTIMATE) w metrach.

### ST1 — `11.JBI`

| Wielkość | Wartość |
|----------|---------|
| Punkty spawania | 1 |
| AABB XYZ mm (min…max) | X −149.9…−149.9 · Y 1760.6…1760.6 · Z 1010.1…1010.1 |
| Centrum / mount | (−149.9, 1760.6, 1010.1) mm → `(−0.1499, 1.7606, 1.0101)` m |
| BC# (weld) | **927851** |
| EC1 / EC2 | 1163 / **0** (1-osiowe) |

### ST2 — `12.JBI`, `02025_KOSZ…`, `123123.JBI`

| Wielkość | Wartość |
|----------|---------|
| Punkty spawania | 66 (`123123.JBI` bez ARCON — pominięty w chmurze) |
| AABB XYZ mm | X −915.8…833.7 · Y 817.3…2216.6 · Z 201.5…1406.9 |
| Centrum / mount | (50.8, 1656.3, 633.8) mm → `(0.0508, 1.6563, 0.6338)` m |
| BC# (weld) | **124171…375361** (mediana ≈ 250331) |
| EC1 / EC2 | −933…120166 / **0** (1-osiowe) |

### ST3 — `1-DEFRO-5.JBI`, `1-DEFRO-5-KOSZ.JBI`, `123.JBI`

| Wielkość | Wartość |
|----------|---------|
| Punkty spawania | 9 (`123.JBI` bez ARCON — pominięty) |
| AABB XYZ mm | X 22.0…776.5 · Y −1807.8…−1207.8 · Z 326.7…535.2 |
| Centrum / mount | (363.3, −1560.4, 404.8) mm → `(0.3633, −1.5604, 0.4048)` m |
| BC# (weld) | **808679…808680** (mediana 808680; dojazdy w jobach bywają do ≈847209) |
| EC1 / EC2 | −2638…119164 / **−726117…167367** (2-osiowe) |

### ST4 — `1KT.JBI`

| Wielkość | Wartość |
|----------|---------|
| Punkty spawania | 1 |
| AABB XYZ mm | X 699.6…699.6 · Y −1273.0…−1273.0 · Z 961.8…961.8 |
| Centrum / mount | (699.6, −1273.0, 961.8) mm → `(0.6996, −1.2730, 0.9618)` m |
| BC# (weld) | **190240** |
| EC1 / EC2 | −2 / **−1** (2-osiowe TURN-2; wartości bliskie zeru w tym jobie) |

---

## Uwagi / TODO

* XYZ = APP_ESTIMATE — uzupełnij pendant/MotoSim w `golden_poses.json` (M1.2).
* `UFRAME.CND`: jeśli pojawi się w kolejnym dumpie, wgrać fixture i uzupełnić tę tabelę — **nie** zgadywać numerów USER.
* RECT-X mm↔BC nadal `RECT_X_LEAD_VERIFIED=false` (M1.3).
