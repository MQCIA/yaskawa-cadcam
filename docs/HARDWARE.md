# Cell hardware configuration

This project is configured for the following welding cell:

| Component     | Value                          | Where configured                        |
|---------------|--------------------------------|------------------------------------------|
| Robot         | Yaskawa **AR series** (DX200)  | `backend/app/robot_config.py` (`MODELS`) |
| Positioner    | **H1000D** (external axis)     | `backend/app/robot_config.py` (`POSITIONER_H1000D`) |
| Power source  | **Lorch S8**                   | `backend/app/welding_config.py`          |

## Robot (Yaskawa AR series)

You told me "AR series" but not the exact model. The catalogue in
`robot_config.py` contains selectable placeholders for **AR900, AR1440,
AR1730, AR2010, AR3120**. The active one defaults to **AR1440** and is chosen
with an environment variable:

```bash
ROBOT_MODEL=AR2010 uvicorn app.main:app --reload
```

> The **reach** and **joint ranges** follow the public AR specs, but the
> per-link DH lengths, motor directions, home pose and pulse-per-degree
> constants are **approximate placeholders**. Replace them with the certified
> values from the Yaskawa data sheet / MotoSim model / DX200 parameter file for
> your exact unit, then verify FK against the pendant.

Planner behaviour (weld names, 12 mm CTWD, home interpolation, 3-state
touch) is taken from Verbotics Weld 2026 session logs — see
[`docs/VERBOTICS_LOGS.md`](VERBOTICS_LOGS.md). That is not a substitute for
the signed workcell URDF/SRDF still sitting on the Windows install.

To finish the model correctly, please provide:
- the exact AR model number (e.g. AR1440 / AR2010 / …),
- its data sheet (P-point / reach diagram) or the MotoSim robot definition,
- the DX200 parameter file (pulse↔degree, TOOL/USER frames),
- a reference `.JBI` exported from your controller (to match `RCONF`, speeds,
  and weld-condition numbers).

## Positioner (H1000D)

Modelled as an **external / station axis** (`S1E`), assumed to be a single
rotary headstock of the ~1000 kg class. If your H1000D is a 2-axis (tilt +
rotate) or head/tail unit, add the extra axis in `POSITIONER_H1000D.axes`.

When a per-point positioner angle is supplied, the postprocessor switches robot
moves to coordinated **`SMOVL`** and declares a `RB1,ST1` group. Coordinated
robot+positioner motion **must** be validated in MotoSim before use — the exact
EC/base-axis encoding depends on your controller setup.

## Power source (Lorch S8)

`welding_config.py` maps logical weld conditions (referenced by DX200
`ARCON ASF#(n)`) to Lorch S8 jobs plus example amps/volts/wire-speed/process.
These are **example values, not a qualified WPS** — replace them with your
welding procedure and validate on coupons. The robot↔S8 link is typically a
digital fieldbus (EtherNet/IP, DeviceNet, PROFINET) or the analog/arc
interface; confirm how yours is wired with your integrator.
