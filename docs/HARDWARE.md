# Cell hardware configuration

Production dump: `reference/robot-cell-dx200` (see [CELL.md](CELL.md)).

| Component | Value | Where configured |
|-----------|--------|------------------|
| Robot | Yaskawa **MA2010** (MA02010-A0*) on DX200 | `backend/app/robot_config.py` (`ROBOT_MODEL=MA2010`) |
| Base | **RECT-X** → INFORM **BS1** | postprocessor `BC#` park pulse |
| Positioner | **TURN ST1** (S1–S4 TURN) | `POSITIONER=TURN-ST1` |
| Power source | **MOTOWELD-E Series 350A** | `backend/app/welding_config.py` |
| Pulse↔deg | RC1G / RBCALIB | `backend/app/pulse_config.py` |

## Robot (MA2010)

Default model matches the CF dump. DH link lengths are still approximate;
**pulse-per-degree** values are mined from `RC.PRM` softlimits and are used
when exporting `///POSTYPE PULSE` jobs.

```bash
ROBOT_MODEL=MA2010 POSITIONER=TURN-ST1 uvicorn app.main:app --reload
```

Validate FK / pulse jobs against the pendant and MotoSim before any controller load.

## Positioner (TURN ST1)

Jobs use `///GROUP2 ST1` with `EC#=pulse,0` (TURN-1). Scaling ≈ **1019.7 pulse/°**
from `RBCALIB.DAT`. Optional `POSITIONER=H1000D` keeps the older UI placeholder.

## Power source (Motoweld-E)

Cell jobs use plain `ARCON` / `ARCOF` (no `ASF#`). UI schedule numbers are examples
only — real amps/volts live in `ARCSRT.CND` / macros on the controller.
