# Soft cubes `CUBEINTF` (M2.5)

Źródło: `reference/dx200-backup-20260915/CUBEINTF.CND` (fixture: `backend/tests/fixtures/robot-cell/CUBEINTF.CND`).

## Co to jest

DX200 **soft interference cubes** w przestrzeni **impulsów** (nie mm świata).  
Nie zastępują FSU / kurtyn / MotoSim.

| Index | Nazwa | Uwagi |
|------:|-------|-------|
| 1 | `STACJA_1` | Dwa narożniki pulse (hi/lo), 8 intów |
| 2 | `STACJA_2` | j.w. |
| 3… | (puste / nienazwane) | Sloty zerowe w dumpie |

Kod: `backend/app/cubeintf.py` → `parse_cubeintf`, `path_cube_hits`.  
API: `GET /api/config` → `cubeintf`; `POST /api/check-cubes` z `joint_pulses`.

## Cartesian vs pulse

| Warstwa | Skąd | Użycie |
|---------|------|--------|
| Station AABB mm | `CELL_STATIONS.aabb_xyz_mm` / FE `mountXyzM` box | Coarse TCP air-move demo |
| CUBEINTF | Pulse hi/lo | Flaga gdy C# ścieżki wchodzi w soft-cube |

Obie = **APP_ESTIMATE / soft** — obowiązkowy dry-run w MotoSim ([GO_LIVE.md](GO_LIVE.md)).
