# TouchSense / search — wzorzec komórki (M2.4)

Źródło: joby `SRCH-1D1.JBI`, `SRCH-2D1.JBI`, `SZUKAJ1.JBI` (pełny dump → `docs/_dump_scan.json`).

## Produkcja

| Job | Rola |
|-----|------|
| `SRCH-1D1` | Pojedynczy `MOVL … SRCH RIN#(1)=ON T=0.20 DIS=…` (P-variables) |
| `SRCH-2D1` | Dwukierunkowy search (kilka `MOVL` + `SRCH RIN#(1)`) |
| `SZUKAJ1` | Wrapper ruchu + **`CALL JOB:SRCH-2D1`** między punktami C#/BC# |

Przykład z `SRCH-2D1` (skrót):

```
MOVL P002 V=15.0 SRCH RIN#(1)=ON T=0.20 DIS=20.0
```

`RIN#(1)` i `DIS=` są **specyficzne dla komórki** — `TODO: weryfikacja w CIO / schemacie elektrycznym` przed zmianą.

## CAD/CAM

`PostprocessorConfig.enable_touchsense=True` wstawia **przed pierwszym `ARCON`**:

```
' TouchSense — requires SRCH-2D1.JBI on controller (dump SZUKAJ1 pattern)
CALL JOB:SRCH-2D1
```

- Nie generujemy własnych `DOUT` / fałszywych `RIN#`.
- Job `SRCH-2D1` musi już istnieć na DX200 / w projekcie MotoSim.
- Frontend demo może mieć waypointy `kind=sense` / `tag=TOUCH` (komentarz) — pełny search = `CALL` jak wyżej.

Walidacja: [GO_LIVE.md](GO_LIVE.md) + dry-run MotoSim z aktywnymi kurtynami.
