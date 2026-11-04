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
- Torch in viewer: MotoROS2 chain `flange → tool0 → tcp_0` (`WeldUrdfRobot`
  portals `WeldTorch3D` into Motoman `tool0`). Live TOOL 0 + mount calibration
  UI (`frontend/lib/tool0.ts`). Default TCP from `TOOL.CND`. Default mesh:
  **ESAB `rm62_36deg.stl`**.
- Robot link API: `GET /api/robot-link` · FK frames:
  `POST /api/forward-kinematics` → `{ flange, tool0, tcp_0 }`.

Large binaries (`CMOS.BIN`, `ALL.PRM`, …) stay under `reference/` (gitignored).

## MotoROS2 on this DX200 (optional telemetry)

Official docs: [Yaskawa-Global/motoros2](https://github.com/Yaskawa-Global/motoros2)
`README.md` + `doc/ros_api.md`. App welding path stays **native `.JBI`**
(`ARCON` / `WVON` / `MACRO1`). MotoROS2 is for joint/TF stream and air moves
after `start_traj_mode` / `INIT_ROS`.

### Firmware

| | Value |
|--|--|
| Cell dump (`SYSTEM.SYS`) | **`DN2.27.00A(US/PL)-00`** |
| MotoROS2 minimum | **`DN2.44.00-00`** |
| Status | **Upgrade required** before installing MotoROS2 |

### Binaries (MotoROS2 **v0.2.1**)

| Distro | File | MD5 |
|--------|------|-----|
| Humble | `mr2_dx2_h.out` | `6672e218b60c42f00ce335dc4a977ee9` |
| Jazzy | `mr2_dx2_j.out` | `f31673a8559e331719f8b25f174419a3` |

### Hardware / Maintenance

- USB stick with `motoros2_config.yaml` must stay in cabinet port **`CN106`**
  (missing → alarm **`8013 [0] Missing MotoROS2 cfg file`**).
- Maintenance: `MotoPlus FUNC. = USED`, `MOTOMAN DRIVER = USED`.
- Incompatible: **Simple Connect**.
- With FSU: temporarily disable **`SAVE DATA CRC CHECK FUNC (FSU)`** during install.

### Required `S2C` (DX200)

| Param | Value | Note |
|-------|-------|------|
| `S2C541` | `0` | |
| `S2C542` | `0` | |
| `S2C1102` | `2` | |
| `S2C1104` | `2` | |
| `S2C1117` | `1` | DX200 Speed FB → alarm **`8001[10]`**, then reboot |
| `S2C1250` | `1` | |
| `S2C1402` | `3` | |

### TF chain (`doc/ros_api.md`)

```
world → base → flange → tool0 → tcp_0
```

| Frame | Meaning |
|-------|---------|
| `flange` | REP-103: **X+** out of flange, Y+ left, Z+ up |
| `tool0` | Yaskawa all-zeros tool: **Z+** out of flange (static `flange→tool0`) |
| `tcp_0` | Active TOOL 0 from `TOOL.CND` in Motoman `tool0`: xyz `[-54.280, 1.052, 415.702]` mm, RPY `[1.3119, -42.6215, -6.8387]` deg |

Also published: `flange → tcp_N` for the active tool (here N=0).

### Group topics (this cell)

| INFORM | ROS topic | Unit |
|--------|-----------|------|
| RB1 | `ctrl_groups/r1/joint_states` | rad |
| BS1 | `ctrl_groups/b1/joint_states` | m |
| ST1…ST4 | `ctrl_groups/s1…s4/joint_states` | rad |

Plus `joint_states`, `robot_status`, `tf`.

### Known limitations (README)

- FastDDS only · no Cartesian motion · no Absolute Accuracy Compensation benefit
- No asynchronous motion · no partial `FollowJointTrajectory` goals
- Welding with `ARCON` / `WVON` / `MACRO1` → **native `.JBI` only**

Code: `backend/app/robot_link.py` (`describe_robot_link`, `decode_motoros2_error`).

## Agent index

- **Weld / ARGF / WEAV map:** [CELL_WELD_MAP.md](CELL_WELD_MAP.md)
- **Per-file dump catalog:** [CELL_DUMP_CATALOG.md](CELL_DUMP_CATALOG.md)
- **Machine scan JSON:** [_dump_scan.json](_dump_scan.json)
- **Cursor skill:** `.cursor/skills/dx200-cell-dump/` (lookup + concept gaps)
