# Production DX200 cell (dump from the robot pendant / CF)

Parsed from `D:\YASKAWA\z robota` → `reference/robot-cell-dx200` (2026-09-15).

## Hardware

| Item | Value |
|------|--------|
| Controller | DX200 `DN2.27.00A(US/PL)` — **ARC WELDING** |
| Robot R1 | **MA02010-A0\*** (MA2010) |
| Base B1 | **RECT-X** (linear travel) → INFORM group **BS1** |
| Station | S1–S4 **TURN** → INFORM group **ST1** |
| Welder | **MOTOWELD-E Series 350A** (`WELDER.DAT`) |
| TOOL 0 TCP | `-54.280, 1.052, 415.702` mm |
| TOOL 0 RPY | `1.3119, -42.6215, -6.8387` deg |

## Pulse scaling (mined)

From `RC.PRM` / `RC1G` softlimits + MA2010 URDF limits, and `RBCALIB.DAT` station steps.
Implemented in `backend/app/pulse_config.py` and `frontend/lib/weldProgram.ts` (`CELL_PULSE`).

| Axis | Softlimit pulses (±) | Limit deg | ≈ pulses/deg |
|------|----------------------|-----------|--------------|
| S | ±241449 | ±180 | **1341.38** |
| L | +295690 / −200306 | +155 / −105 | **1907.68** |
| U | +254863 / −136989 | +160 / −86 | **1592.89** |
| R | ±153430 | ±150 | **1022.87** |
| B | +88222 / −132333 | +90 / −135 | **980.24** |
| T | ±95499 | ±210 | **454.76** |
| ST1 (TURN) | RBCALIB ΔE≈91771 / 90° | | **1019.69** |
| BS1 (RECT-X) | home / park BC | | **570125** home, **878036** park; **≈257.708 p/mm** (calibrated from taught ST ranges vs 2400 mm span; no rack lead in RC1G text) |
| MOTOWELD-E | MACRO1 ARGF | | Production: **ARGF1 / 4 / 5** (+2, 8, 11…); never `ARCON ASF#` |

Encoder resolution (`SV1G`): **4096**.

## Real job grammar (see `jobs/000.JBI`)

```
///NPOS n,n,n,0,0,0
///TOOL 0
///POSTYPE PULSE
C#####=s,l,u,r,b,t     (joint pulses)
BC#####=…              (base RECT-X pulses)
EC#####=a,b            (station pulses; TURN-1 → a,0)
///GROUP1 RB1,BS1
///GROUP2 ST1
MOVJ C# BC# VJ=… DEC=20  +MOVJ EC# VJ=…
ARCON / ARCOF
WVON WEV#(n) / WVOF
MACRO1 MJ#(0) ARGFn
```

No `ARCON ASF#(...)` on this cell — conditions come from `ARCSRT.CND` / macros.

## App integration

- Default robot model: **MA2010** (`ROBOT_MODEL=MA2010`)
- Kinematics: URDF chain (`d1=505, a1=150, a2=760, a3=200, d4=1082, d6=100` mm;
  axes S+Z L+Y U−Y R−X B−Y T−X) + TOOL 0; IK seed = cell home pulses
- Default positioner: **TURN-ST1** (`POSITIONER=TURN-ST1`)
- Default JBI profile: **`ma2010_cell`**
- Pass IK `joint_angles_deg` into `generate_jbi` / `/api/generate-jbi` → full **PULSE** C#
- Without joints: C# stays RECTAN; BC/EC still use pulse units
- Torch length in viewer ≈ **416 mm** from TOOL 0

Large binaries (`CMOS.BIN`, `ALL.PRM`, …) stay under `reference/` (gitignored).

## Agent index

- **Per-file dump catalog:** [CELL_DUMP_CATALOG.md](CELL_DUMP_CATALOG.md)
- **Machine scan JSON:** [_dump_scan.json](_dump_scan.json)
- **Cursor skill:** `.cursor/skills/dx200-cell-dump/` (lookup + concept gaps)
- **Golden poses + pulse diagnostic:** [CELL_POSES.md](CELL_POSES.md), `tools/compare_pulses.py`
