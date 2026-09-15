# DX200 cell dump — full file catalog

**Path:** `reference/robot-cell-dx200/` (local; gitignored bulk under `reference/`)  
**Source:** pendant CF dump `D:\YASKAWA\z robota` (2026-09-15)  
**Scan:** machine-read of every text file (137 files). `CMOS.BIN` = header only. Large `*.PRM` / `ARCSRT`/`ARCEND` = section-indexed, not every numeric row recited here.  
**Machine index:** [`_dump_scan.json`](_dump_scan.json) (job flags, PRM section lists, DAT headers).  
**Code that consumes this:** `backend/app/pulse_config.py`, `dx200_postprocessor.py`, `docs/CELL.md`.

---

## Quick “where do I look?”

| Need | Open first |
|------|------------|
| Robot / base / station names | `SYSTEM.SYS` |
| TOOL0 TCP | `TOOL.CND` / `cnd/TOOL.CND` |
| Welder brand | `WELDER.DAT` |
| Real INFORM grammar | `jobs/000.JBI` |
| Pulse↔deg robot | `params/RC.PRM` → `RC1G` softlimits + `pulse_config.py` |
| Station pulse step | `RBCALIB.DAT` SPULSE + `params/RC.PRM` RC3G–RC6G |
| Encoder resolution | `params/SV.PRM` → `SV1G` (4096) |
| Angle/gear tables | `params/SC.PRM` → `S1CxG` |
| Arc start/end conditions | `cnd/ARCSRT.CND`, `cnd/ARCEND.CND` |
| Weave files `WEV#(n)` | `WEAV.CND` / `cnd/WEAV.CND` (255 entries) |
| User weld tables | `WELDUDEF.DAT` (64 USER-n) |
| Station cubes / zones | `cnd/CUBEINTF.CND` |
| Master + station call flow | `jobs/STACJA_*.JBI` → `CALL JOB:…` |
| TCP / UF calibration | `jobs/TCP-R1-Y-BA*.JBI`, `EICH-R1-Y-BA.JBI` |
| Absolute encoders | `dat/ABSO.DAT` |
| IO names / CIO | `dat/IONAME.DAT`, `dat/CIOPRG.LST`, `params/CIO.PRM` |
| Full parameter blob | `params/ALL.PRM` (union of many; prefer split `RC`/`SC`/`SV`) |

---

## Cell identity (from `SYSTEM.SYS`)

| Field | Value |
|-------|--------|
| Controller | `DN2.27.00A(US/PL)` — **ARC WELDING** |
| R1 | **MA02010-A0\*** (MA2010) |
| B1 | **RECT-X** → INFORM **BS1** |
| S1, S2 | **TURN-1** (1 axis) |
| S3, S4 | **TURN-2** (2 axes) |
| Welder | **MOTOWELD-E Series 350A** (`WELDER.DAT`) |
| TOOL 0 | TCP mm `-54.280, 1.052, 415.702` · RPY `1.3119, -42.6215, -6.8387` |

Production move grammar:

```
///NPOS n,n,n,0,0,0
///TOOL 0
///POSTYPE PULSE
C#=s,l,u,r,b,t   BC#=…   EC#=a,b
///GROUP1 RB1,BS1
///GROUP2 ST1
MOVJ/MOVL C# BC# …  +MOVJ EC# …
ARCON / ARCOF   (no ASF# in these jobs)
WVON WEV#(n) / WVOF
MACRO1 MJ#(0) ARGFn
```

---

## Root files (12)

| File | Size | What it is | Agent use |
|------|------|------------|-----------|
| `SYSTEM.SYS` | 1.6 KB | System identity, axes, hours | **Always read first** for cell identity |
| `TOOL.CND` | 7.3 KB | Tools 0… — TOOL0 is live TCP | Torch length / flange→TCP |
| `WELDER.DAT` | 2.1 KB | 8× Motoweld-E 350A class blocks | Power-source label; not ASF maps |
| `WELDUDEF.DAT` | 15 KB | 64 `USER-n` weld tables (wire/A/V curves) | Map UI conditions ↔ controller tables |
| `WEAV.CND` | 27 KB | 255 weave definitions | Default `WEV#(21)` used in jobs |
| `RBCALIB.DAT` | 1.6 KB | Robot+base+station calib pulses + `SRANG` | Station Δpulse (~91771/90°) |
| `HOME2.DAT` | 0.8 KB | Home group mask (mostly zeros) | Low priority |
| `VAR.DAT` | 6.6 KB | B/I/D/R/S variable pools | Job `SET` / logic vars |
| `ARCSUP.DAT` | 1.3 KB | Arc support params (8 blocks) | Arc application support |
| `IPNETCFG.DAT` | 0.1 KB | Host/DHCP/DNS/SNTP (mostly off) | Network; not kinematics |
| `README.txt` | 1.1 KB | Human dump notes | Orientation |
| `CMOS.BIN` | **24 MB** | Controller CMOS image (`$$NX.CMOSTYPE…`) | **Do not commit / do not text-parse**; restore only |

Duplicates of several root files also live under `dat/` and `cnd/`.

---

## `jobs/` (63 × `.JBI`)

### Categories

| Category | Count | Examples | Role |
|----------|------:|----------|------|
| Weld motion (ARCON) | 25 | `000`, `1`, `PRODUK-PONT`, `UL-*`, `UP*-STACJA4`, `STACIA2-CERAD1700` | **Gold** for PULSE C#/BC#/EC# + MACRO/WVON |
| Station masters | 5 | `STACJA_1`…`4`, `STACJA_3-BEZ-CLEAN` | Curtain/gate + `CALL JOB:` weld program |
| Gates / curtains | 9 | `OTWORZ-BRAMA-*`, `ZAMKNIJ-KURTYNA-*`, `RESET-KURTYNY` | IO only, no positions |
| Tests / cubes | 10 | `TEST-S*`, `TEST-ST*`, `CUBE_*`, `TEST2024` | Station/base pulse ranges, SMOVL |
| TCP / calibrate | 3 | `TCP-R1-Y-BA`, `TCP-R1-Y-BAREL`, `EICH-R1-Y-BA` | UF/TOOL teach helpers |
| Utility | 8 | `BAZA1`, `CLEAN_R*`, `SMAR`, `DRAHT-R1`, `RST*` | Home/clean/wire |
| Other | 3 | `LORCH-R1` (empty stub), `SRCH-*`, `SZUKAJ1` | Search / legacy |

### Canonical references

| Job | Why |
|-----|-----|
| **`000.JBI`** | Short production weld: PULSE, BC≈878036, EC≈−1746,0, ARCON/WVON/MACRO |
| **`CUBE_ST1.JBI`** | Tiny weld + cube check; BC span 838k–1138k |
| **`TEST-ST1.JBI`** | Coordinated `SMOVL` + large EC swing on ST1 |
| **`RBCALIB` + `TEST-S1`** | Station/base pulse experiments |
| **`STACJA_1.JBI`** | Orchestration pattern (gates → `CALL` product job) |
| **`02025_KOSZ_…JBI`** | Largest path (218 pts) — stress grammar |

Almost all weld jobs: `///POSTYPE PULSE`, groups `RB1,BS1` + `ST1`.

---

## `cnd/` (7)

| File | Size | Contents | Agent use |
|------|------|----------|-----------|
| `ARCSRT.CND` | 247 KB | **1000** `//ARCSRT EN n` arc-start blocks | Real start conditions (not ASF#) |
| `ARCEND.CND` | 228 KB | **1000** arc-end blocks | Crater/end params |
| `WEAV.CND` | 27 KB | Copy of root weaves | `WVON WEV#(n)` |
| `TOOL.CND` | 7 KB | Tools | Same as root TOOL0 |
| `CUBEINTF.CND` | 6.6 KB | Named cubes `STACJA_1`… + pulse boxes | Soft interference / approach cubes |
| `SHOCKLVL.CND` | 6.9 KB | Shock/collision levels per axis | Safety thresholds |
| `PMCOND.CND` | 2 KB | Preventive maintenance thresholds | Ops, not CAM |

---

## `dat/` (37)

| File | Purpose | Priority for CAD/CAM |
|------|---------|----------------------|
| `SYSTEM.SYS` | Dup of root | High |
| `TOOL` via cnd | — | High |
| `WELDER.DAT` / `WELDUDEF.DAT` | Welder + user tables | High |
| `RBCALIB.DAT` | Calib pulses | **High** (pulse) |
| `ABSO.DAT` | Absolute encoder offsets | Medium (zeroing) |
| `VAR.DAT` / `VARNAME.DAT` | Variables + names | Medium (macros) |
| `IONAME.DAT` / `EXIONAME.DAT` / `IOMNAME.DAT` | IO symbolic names | Medium (gates) |
| `IOAXIS.DAT` | Axis↔IO | Medium |
| `EIOALLOC.DAT` | External IO alloc | Low–med |
| `CIOPRG.LST` | CIO program listing | Low–med |
| `YSFLOGIC.DAT` | Safety logic | Low–med |
| `CUBE` related | see cnd | — |
| `ALMHIST.DAT` | Alarm history | Low |
| `LOGDATA.DAT` / `IOMSGHST.DAT` / `PANELBOX.LOG` | Logs | Low |
| `KEYALLOC.DAT` / `IFPANEL.DAT` | Pendant UI | Low |
| `PM*.DAT` / `ENCHEAT` / `SVMON` / `TRQDAT` | Maint / torque / servo mon | Low |
| `IPNETCFG.DAT` | Network | Low |
| `HOME2` / `OPEORG` / `SETTM` / `TM*` / `UWORD` / `USERMENU` / `PSEUDOIN` | Misc system | Low |
| `ARCSUP.DAT` | Arc support | Low–med |
| `RBSTPFCT.DAT` | Robot stop factors | Low |

---

## `params/` (18 × `.PRM`)

| File | Sections (examples) | Agent use |
|------|---------------------|-----------|
| **`RC.PRM`** | `RC1G`… robot/base/station control + **softlimits** | **Pulse softlimits** (RC1G[924..937]); RC2=RECT-X; RC3–4 TURN-1; RC5–6 TURN-2 |
| **`SC.PRM`** | `S1C1G`…`S1C32G`, `S2C`/`S3C`/`S4C` | Gear/angle conversion tables; S3C softlimit-like rows |
| **`SV.PRM`** | `SV1G`… | Servo; **encoder 4096** at SV1G[24..29] |
| `ALL.PRM` | 337 `///` sections | Megablob; prefer split files |
| `AMC.PRM` | `AMC1G`… | Axis motion control |
| `MF.PRM` | `MF1G`… | Motion function |
| `RE.PRM` | `RE1G`… | Robot expansion / external |
| `SVM`/`SVP`/`SVC`/`SVS` | Servo variants | Advanced servo only |
| `AP.PRM` | `A1P`…`A8P` | Application params |
| `SD.PRM` / `SE.PRM` | `S1D`… / `S1E`… | Station D/E |
| `RO.PRM` | `RO1G`… | Robot options |
| `CIO.PRM` / `FD.PRM` / `RS.PRM` | (few/`///` headers) | CIO / filter / RS — read as needed |

Mined pulse scaling lives in `backend/app/pulse_config.py` (do not re-derive blindly).

---

## Coverage honesty

| Scope | Status |
|-------|--------|
| Every file opened / header-parsed | **Yes** (this catalog + `_dump_scan.json`) |
| Every `.JBI` INST line interpreted | No — flags + counts; open job when needed |
| Every ARCSRT/ARCEND row decoded | No — 1000× templates; sample EN 1 |
| Every PRM integer | No — sections + known RC1G/SV1G/RBCALIB mining |
| `CMOS.BIN` full | **No** (binary image) |

---

## Related project docs

- [CELL.md](CELL.md) — hardware + pulse table + app wiring  
- [HARDWARE.md](HARDWARE.md) — env vars / active config  
- Skill: `.cursor/skills/dx200-cell-dump/SKILL.md`
