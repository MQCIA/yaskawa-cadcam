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
| BS1 (RECT-X) | park BC | | **878036** (park; mm lead TBD) |

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

## Motoweld ARGF / WEAV / TouchSense (M3)

Full map: **[CELL_WELD_MAP.md](CELL_WELD_MAP.md)** · code: `backend/app/welding_config.py`.

| Topic | Dump fact |
|-------|-----------|
| `MACRO1 MJ#(0) ARGFn` | Motoweld **PROG-NR = n** (`MACRO.DAT` MACRO1 1, tag `PROG-NR:`) |
| `ARCSRT.CND` | All 1000 EN blocks identical → **200 A / 20.0 V / 100 %** |
| Live ARGF | **1, 4, 5, 8** (also 6, 10) — travel V≈4.2–7.5 in jobs |
| `WEV#(1)` | 2 mm / 70° / 2 Hz |
| `WEV#(15)` | 3 mm / 45° / 3 Hz |
| `WEV#(16)` | 2 mm / 45°·0° / 2 Hz |
| `WEV#(21)` | 4 mm L / 2 Hz — **default** production weave |
| TouchSense | `123123.JBI`: `MOVS C# BC# V=11.0  +MOVJ EC# VJ=0.78` |

## App integration

- Default robot model: **MA2010** (`ROBOT_MODEL=MA2010`)
- Default positioner: **TURN-ST1** (`POSITIONER=TURN-ST1`)
- Default JBI profile: **`ma2010_cell`**
- Pass IK `joint_angles_deg` into `generate_jbi` / `/api/generate-jbi` → full **PULSE** C#
- Without joints: C# stays RECTAN; BC/EC still use pulse units
- Torch in viewer: `createPortal` onto URDF `tool0` / `flange` (MA2010
  `link_6_t` flange face at X≈0.100 m). Live TOOL 0 + mount calibration UI
  (`frontend/lib/tool0.ts`, `WeldTorch3D`, panel in Robot dock). Default TCP
  `xyz=(-54.280, 1.052, 415.702) mm` RPY `(1.3119, -42.6215, -6.8387)°`
  from `TOOL.CND`. Default mesh: **ESAB `rm62_36deg.stl`**
  (`frontend/public/models/torch_esab/`, alignment in `lib/torchMesh.ts`).

Large binaries (`CMOS.BIN`, `ALL.PRM`, …) stay under `reference/` (gitignored).

## Agent index

- **Weld / ARGF / WEAV map:** [CELL_WELD_MAP.md](CELL_WELD_MAP.md)
- **Per-file dump catalog:** [CELL_DUMP_CATALOG.md](CELL_DUMP_CATALOG.md)
- **Machine scan JSON:** [_dump_scan.json](_dump_scan.json)
- **Cursor skill:** `.cursor/skills/dx200-cell-dump/` (lookup + concept gaps)
